#!/usr/bin/env python3

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


class ValidationError(Exception):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def field(obj: dict[str, Any], key: str, location: str) -> Any:
    require(key in obj, f"{location}.{key} is missing")
    return obj[key]


def as_object(value: Any, location: str) -> dict[str, Any]:
    require(isinstance(value, dict), f"{location} must be an object")
    return value


def as_list(value: Any, location: str) -> list[Any]:
    require(isinstance(value, list), f"{location} must be an array")
    return value


def as_int(value: Any, location: str) -> int:
    require(
        isinstance(value, int) and not isinstance(value, bool),
        f"{location} must be an integer",
    )
    return value


def as_bool(value: Any, location: str) -> bool:
    require(isinstance(value, bool), f"{location} must be a boolean")
    return value


def as_string(value: Any, location: str) -> str:
    require(isinstance(value, str), f"{location} must be a string")
    require(value != "", f"{location} must not be empty")
    return value


def numeric_vector(value: Any, location: str) -> list[float]:
    values = as_list(value, location)
    require(values, f"{location} must not be empty")

    result = []
    for index, element in enumerate(values):
        require(
            isinstance(element, (int, float))
            and not isinstance(element, bool)
            and math.isfinite(element),
            f"{location}[{index}] must be a finite number",
        )
        result.append(float(element))

    return result


def endpoint_error(
    actual_value: Any,
    expected_value: Any,
    location: str,
) -> float:
    actual = numeric_vector(actual_value, f"{location}.actual")
    expected = numeric_vector(expected_value, f"{location}.expected")

    require(
        len(actual) == len(expected),
        f"{location} dimension mismatch: "
        f"actual={len(actual)}, expected={len(expected)}",
    )

    return max(
        abs(actual_element - expected_element)
        for actual_element, expected_element in zip(actual, expected)
    )


def validate_result(result_path: Path, tolerance: float) -> None:
    with result_path.open(encoding="utf-8") as result_file:
        result = as_object(json.load(result_file), "result")

    solver = as_string(field(result, "solver", "result"), "result.solver")
    require(solver == "GuidedARC", f"unexpected solver: {solver!r}")

    planner_status = as_string(
        field(result, "planner_status", "result"),
        "result.planner_status",
    )
    require(
        planner_status == "Exact solution",
        f"planner did not return an exact solution: {planner_status!r}",
    )

    success = as_bool(field(result, "success", "result"), "result.success")
    require(success, "result.success is false")

    verification_conflict = field(
        result,
        "verification_conflict",
        "result",
    )
    require(
        verification_conflict is None,
        "the C++ application reported an exported-path conflict: "
        f"{verification_conflict!r}",
    )

    benchmark = as_object(
        field(result, "benchmark", "result"),
        "result.benchmark",
    )
    context = as_object(
        field(benchmark, "context", "result.benchmark"),
        "result.benchmark.context",
    )

    num_robots = as_int(
        field(context, "num_robots", "result.benchmark.context"),
        "result.benchmark.context.num_robots",
    )
    require(num_robots > 0, "num_robots must be positive")

    starts = as_list(
        field(context, "starts", "result.benchmark.context"),
        "result.benchmark.context.starts",
    )
    goals = as_list(
        field(context, "goals", "result.benchmark.context"),
        "result.benchmark.context.goals",
    )
    robots = as_list(field(result, "robots", "result"), "result.robots")

    require(len(starts) == num_robots, "start count does not match num_robots")
    require(len(goals) == num_robots, "goal count does not match num_robots")
    require(len(robots) == num_robots, "robot count does not match num_robots")

    print(f"artifact: {result_path}")
    print(f"solver: {solver}")
    print(f"planner status: {planner_status}")
    print(f"success: {success}")
    print("C++ verification conflict: null")
    print(f"robots: {num_robots}")

    for robot_index in range(num_robots):
        robot_location = f"result.robots[{robot_index}]"
        robot = as_object(robots[robot_index], robot_location)
        path = as_list(field(robot, "path", robot_location), f"{robot_location}.path")
        require(path, f"{robot_location}.path must not be empty")

        start_error = endpoint_error(
            path[0],
            starts[robot_index],
            f"robot {robot_index} start",
        )
        goal_error = endpoint_error(
            path[-1],
            goals[robot_index],
            f"robot {robot_index} goal",
        )

        require(
            start_error <= tolerance,
            f"robot {robot_index} start error {start_error} "
            f"exceeds tolerance {tolerance}",
        )
        require(
            goal_error <= tolerance,
            f"robot {robot_index} goal error {goal_error} "
            f"exceeds tolerance {tolerance}",
        )

        print(
            f"robot {robot_index}: states={len(path)}, "
            f"start_error={start_error:.3g}, "
            f"goal_error={goal_error:.3g}"
        )

    history = as_object(
        field(result, "guided_arc_resolution_history", "result"),
        "result.guided_arc_resolution_history",
    )
    conflicts = as_list(
        field(history, "conflicts", "history"),
        "history.conflicts",
    )
    resolved_count = as_int(
        field(history, "resolved_conflict_count", "history"),
        "history.resolved_conflict_count",
    )

    require(
        resolved_count == len(conflicts),
        "resolved_conflict_count does not match the conflict array",
    )
    require(resolved_count > 0, "the run did not complete any conflict repair")

    print(f"resolved conflicts: {resolved_count}")

    for conflict_index, conflict_value in enumerate(conflicts):
        location = f"history.conflicts[{conflict_index}]"
        conflict = as_object(conflict_value, location)

        robot_i = as_int(field(conflict, "robot_i", location), f"{location}.robot_i")
        robot_j = as_int(field(conflict, "robot_j", location), f"{location}.robot_j")
        timestep = as_int(
            field(conflict, "timestep", location),
            f"{location}.timestep",
        )
        attempts = as_list(
            field(conflict, "attempts", location),
            f"{location}.attempts",
        )

        require(0 <= robot_i < num_robots, f"{location}.robot_i is out of range")
        require(0 <= robot_j < num_robots, f"{location}.robot_j is out of range")
        require(attempts, f"{location}.attempts must not be empty")

        print(
            f"conflict {conflict_index}: "
            f"robots=({robot_i},{robot_j}), "
            f"timestep={timestep}, attempts={len(attempts)}"
        )

        final_outcome = None
        for attempt_position, attempt_value in enumerate(attempts):
            attempt_location = f"{location}.attempts[{attempt_position}]"
            attempt = as_object(attempt_value, attempt_location)

            attempt_index = as_int(
                field(attempt, "attempt_index", attempt_location),
                f"{attempt_location}.attempt_index",
            )
            attempt_robots = as_list(
                field(attempt, "robots", attempt_location),
                f"{attempt_location}.robots",
            )
            solver_name = as_string(
                field(attempt, "solver", attempt_location),
                f"{attempt_location}.solver",
            )
            outcome = as_string(
                field(attempt, "outcome", attempt_location),
                f"{attempt_location}.outcome",
            )
            window_start = as_int(
                field(attempt, "window_start_t", attempt_location),
                f"{attempt_location}.window_start_t",
            )
            window_end = as_int(
                field(attempt, "window_end_t", attempt_location),
                f"{attempt_location}.window_end_t",
            )
            global_end = as_int(
                field(attempt, "global_end_t", attempt_location),
                f"{attempt_location}.global_end_t",
            )
            seed = as_int(
                field(attempt, "attempt_root_seed", attempt_location),
                f"{attempt_location}.attempt_root_seed",
            )
            spans_global_time = as_bool(
                field(attempt, "spans_global_time", attempt_location),
                f"{attempt_location}.spans_global_time",
            )
            uses_global_cspace = as_bool(
                field(attempt, "uses_global_cspace", attempt_location),
                f"{attempt_location}.uses_global_cspace",
            )

            require(
                all(
                    isinstance(robot, int)
                    and not isinstance(robot, bool)
                    and 0 <= robot < num_robots
                    for robot in attempt_robots
                ),
                f"{attempt_location}.robots contains an invalid robot index",
            )
            require(
                robot_i in attempt_robots and robot_j in attempt_robots,
                f"{attempt_location}.robots does not contain the conflict pair",
            )
            require(
                0 <= window_start <= window_end <= global_end,
                f"{attempt_location} has an invalid Temporal window",
            )

            final_outcome = outcome
            print(
                f"  attempt {attempt_index}: robots={attempt_robots}, "
                f"window=[{window_start},{window_end}], "
                f"global_end={global_end}, solver={solver_name}, "
                f"outcome={outcome}, seed={seed}, "
                f"spans_global_time={spans_global_time}, "
                f"uses_global_cspace={uses_global_cspace}"
            )

        require(
            final_outcome == "success",
            f"{location} does not end with a successful attempt",
        )

    print("validation: passed")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate a Temporal GuidedARC result artifact."
    )
    parser.add_argument("result_json", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    arguments = parser.parse_args()

    try:
        require(
            math.isfinite(arguments.tolerance) and arguments.tolerance >= 0.0,
            "tolerance must be a finite, non-negative number",
        )
        validate_result(arguments.result_json, arguments.tolerance)
    except (OSError, json.JSONDecodeError, ValidationError) as error:
        print(f"validation failed: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())