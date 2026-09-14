#!/usr/bin/env python3
"""Run the paired ARC/Temporal GuidedARC policy pilot.

The default invocation is a dry run. Pass --execute to launch planners.
CSpace GuidedARC is intentionally excluded.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_BUILD_DIR = REPO_ROOT / "build" / "apps"
DEFAULT_RESULTS_DIR = REPO_ROOT / "benchmarks" / "results"
SEEDS = tuple(range(5))
TIMEOUT_GRACE_SECONDS = 30.0


@dataclass(frozen=True)
class Variant:
    key: str
    algorithm: str
    robot_policy: str | None = None

    @property
    def args(self) -> tuple[str, ...]:
        args = ["--algorithm", self.algorithm]
        if self.robot_policy is not None:
            args.extend(["--guided-arc-robot-selection", self.robot_policy])
        return tuple(args)


@dataclass(frozen=True)
class Case:
    key: str
    family: str
    executable: str
    scenario_args: tuple[str, ...]
    profile_args: tuple[str, ...]
    time_limit: float
    resource_paths: tuple[str, ...] = ()


VARIANTS = (
    Variant("baseline_arc", "arc"),
    Variant(
        "guided_arc_repair_history",
        "temporal_guided_arc",
        "arc_repair_history",
    ),
    Variant(
        "guided_historical_direct_neighbors",
        "temporal_guided_arc",
        "historical_direct_neighbors",
    ),
    Variant(
        "guided_current_conflict_component",
        "temporal_guided_arc",
        "current_conflict_component",
    ),
)

COMMON_ARGS = (
    "--collision-backend",
    "vamp",
    "--vamp-validation-strategy",
    "combined_rake",
    "--resolution",
    "16",
    "--arc-local-solvers",
    "composite",
    "--arc-local-composite-use-makespan-metric",
    "--arc-cspace-bound-margin",
    "2.0",
    "--arc-min-cspace-bound-range",
    "2.0",
    "--arc-simplify-initial-solutions",
    "--no-arc-simplify-conflict-solutions",
)

MOBILE_PROFILE_ARGS = (
    "--arc-initial-window",
    "50",
    "--arc-expansion-policy",
    "exponential",
    "--arc-expansion-step",
    "2.0",
    "--arc-initial-valid-expansion-policy",
    "linear",
    "--arc-initial-valid-expansion-step",
    "50",
    "--arc-initial-valid-symmetric-expansion",
    "--arc-local-composite-max-samples",
    "1000",
)

HETEROGENEOUS_PROFILE_ARGS = (
    "--arc-initial-window",
    "20",
    "--arc-expansion-policy",
    "exponential",
    "--arc-expansion-step",
    "1.05",
    "--arc-initial-valid-expansion-policy",
    "linear",
    "--arc-initial-valid-expansion-step",
    "20",
    "--arc-initial-valid-symmetric-expansion",
    "--arc-local-composite-max-samples",
    "50000",
)

PANDA_RESOURCES = (
    "resources/panda/panda_spherized.urdf",
    "resources/panda/panda.urdf",
    "resources/panda/panda.srdf",
)

PLANAR_RESOURCES = (
    "resources/planar3/planar3_spherized.urdf",
    "resources/planar3/planar3.urdf",
    "resources/planar3/planar3.srdf",
)

CASES = (
    Case(
        "mobile_circle_n4",
        "mobile_circle",
        "mobile_robot_2d_crossing",
        ("--scenario", "circle", "--num-robots", "4"),
        MOBILE_PROFILE_ARGS,
        30.0,
    ),
    Case(
        "mobile_circle_n8",
        "mobile_circle",
        "mobile_robot_2d_crossing",
        ("--scenario", "circle", "--num-robots", "8"),
        MOBILE_PROFILE_ARGS,
        30.0,
    ),
    Case(
        "planar_cross_n4",
        "planar_cross",
        "planar_manipulator_cross",
        ("--scenario", "cross", "--num-robots", "4"),
        HETEROGENEOUS_PROFILE_ARGS,
        120.0,
        PLANAR_RESOURCES,
    ),
    Case(
        "planar_cross_n8",
        "planar_cross",
        "planar_manipulator_cross",
        ("--scenario", "cross", "--num-robots", "8"),
        HETEROGENEOUS_PROFILE_ARGS,
        120.0,
        PLANAR_RESOURCES,
    ),
    Case(
        "panda_flat_n4",
        "panda_flat",
        "panda_flat",
        ("--num-robots", "4", "--task-index", "0"),
        HETEROGENEOUS_PROFILE_ARGS,
        300.0,
        PANDA_RESOURCES,
    ),
    Case(
        "panda_flat_n8",
        "panda_flat",
        "panda_flat",
        ("--num-robots", "8", "--task-index", "0"),
        HETEROGENEOUS_PROFILE_ARGS,
        300.0,
        PANDA_RESOURCES,
    ),
    Case(
        "panda_cage_n4",
        "panda_cage",
        "panda_cage",
        ("--num-robots", "4", "--task-index", "0"),
        HETEROGENEOUS_PROFILE_ARGS,
        300.0,
        PANDA_RESOURCES,
    ),
    Case(
        "panda_cage_n8",
        "panda_cage",
        "panda_cage",
        ("--num-robots", "8", "--task-index", "0"),
        HETEROGENEOUS_PROFILE_ARGS,
        300.0,
        PANDA_RESOURCES,
    ),
    Case(
        "heterogeneous_p4_s8",
        "heterogeneous_corridor",
        "heterogeneous_corridor",
        (
            "--num-pandas",
            "4",
            "--num-spheres",
            "8",
            "--task-file",
            "resources/benchmarks/heterogeneous_four_pandas_eight_spheres_tasks.json",
            "--task-index",
            "0",
        ),
        HETEROGENEOUS_PROFILE_ARGS,
        300.0,
        (
            "resources/benchmarks/heterogeneous_four_pandas_eight_spheres_tasks.json",
            *PANDA_RESOURCES,
        ),
    ),
    Case(
        "heterogeneous_p8_s16",
        "heterogeneous_corridor",
        "heterogeneous_corridor",
        (
            "--num-pandas",
            "8",
            "--num-spheres",
            "16",
            "--task-file",
            "resources/benchmarks/heterogeneous_eight_pandas_sixteen_spheres_tasks.json",
            "--task-index",
            "0",
        ),
        HETEROGENEOUS_PROFILE_ARGS,
        300.0,
        (
            "resources/benchmarks/heterogeneous_eight_pandas_sixteen_spheres_tasks.json",
            *PANDA_RESOURCES,
        ),
    ),
)


@dataclass(frozen=True)
class Trial:
    case: Case
    variant: Variant
    seed: int
    ordinal: int

    @property
    def block_key(self) -> str:
        return f"{self.case.key}_seed{self.seed}"

    @property
    def trial_id(self) -> str:
        return f"{self.block_key}_{self.variant.key}"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def command_output(command: Sequence[str]) -> str:
    try:
        completed = subprocess.run(
            list(command),
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return "unavailable"
    return (completed.stdout or completed.stderr).strip()


def cmake_executable() -> str:
    workspace_cmake = REPO_ROOT / ".venv" / "bin" / "cmake"
    if workspace_cmake.is_file() and os.access(workspace_cmake, os.X_OK):
        return str(workspace_cmake)
    return shutil.which("cmake") or "cmake"


def cmake_cache_values(path: Path) -> dict[str, str]:
    wanted = {
        "CMAKE_BUILD_TYPE",
        "CMAKE_C_COMPILER",
        "CMAKE_CXX_COMPILER",
        "CMAKE_C_FLAGS",
        "CMAKE_CXX_FLAGS",
        "CMAKE_GENERATOR",
    }
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("//") or line.startswith("#") or "=" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key = key_and_type.split(":", 1)[0]
        if key in wanted:
            values[key] = value
    return values


def machine_metadata() -> dict[str, Any]:
    cpu_model = ""
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text(errors="replace").splitlines():
            if line.lower().startswith("model name") and ":" in line:
                cpu_model = line.split(":", 1)[1].strip()
                break
    memory_kib: int | None = None
    meminfo = Path("/proc/meminfo")
    if meminfo.is_file():
        for line in meminfo.read_text(errors="replace").splitlines():
            if line.startswith("MemTotal:"):
                memory_kib = int(line.split()[1])
                break
    affinity = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []
    return {
        "hostname": platform.node(),
        "platform": platform.platform(),
        "kernel": platform.release(),
        "python": sys.version,
        "cpu_model": cpu_model,
        "logical_cpu_count": os.cpu_count(),
        "available_cpu_affinity": affinity,
        "memory_total_kib": memory_kib,
    }


def repository_metadata(build_dir: Path) -> dict[str, Any]:
    resource_paths = sorted(
        {resource for case in CASES for resource in case.resource_paths}
    )
    executable_paths = sorted({case.executable for case in CASES})
    return {
        "git_revision": command_output(["git", "rev-parse", "HEAD"]),
        "git_status": command_output(["git", "status", "--short", "--branch"]),
        "cmake_cache": cmake_cache_values(build_dir.parent / "CMakeCache.txt"),
        "cmake_executable": cmake_executable(),
        "cmake_version": command_output(
            [cmake_executable(), "--version"]
        ).splitlines()[0],
        "cxx_version": command_output(["/usr/bin/c++", "--version"]).splitlines()[0],
        "executables": {
            executable: {
                "path": str(build_dir / executable),
                "sha256": sha256_file(build_dir / executable),
            }
            for executable in executable_paths
        },
        "resources": {
            resource: sha256_file(REPO_ROOT / resource)
            for resource in resource_paths
        },
        "machine": machine_metadata(),
    }


def build_trials(cases: Sequence[Case], seeds: Sequence[int]) -> list[Trial]:
    trials: list[Trial] = []
    for seed in seeds:
        case_shift = seed % len(cases)
        ordered_cases = tuple(cases[case_shift:]) + tuple(cases[:case_shift])
        for case in ordered_cases:
            case_index = CASES.index(case)
            variant_shift = (seed + case_index) % len(VARIANTS)
            ordered_variants = (
                VARIANTS[variant_shift:] + VARIANTS[:variant_shift]
            )
            for variant in ordered_variants:
                trials.append(Trial(case, variant, seed, len(trials) + 1))
    return trials


def trial_dir(output_root: Path, trial: Trial) -> Path:
    return (
        output_root
        / "trials"
        / trial.case.key
        / f"seed_{trial.seed}"
        / trial.variant.key
    )


def trial_command(
    build_dir: Path,
    output_root: Path,
    trial: Trial,
    track_history: bool,
) -> list[str]:
    directory = trial_dir(output_root, trial)
    history_args = ["--track-arc-history"] if track_history else []
    return [
        str(build_dir / trial.case.executable),
        *trial.case.scenario_args,
        *trial.variant.args,
        *COMMON_ARGS,
        *trial.case.profile_args,
        "--time-limit",
        str(int(trial.case.time_limit)),
        "--seed",
        str(trial.seed),
        "--metrics-json",
        str(directory / "metrics.json"),
        "--output-paths",
        *history_args,
        "--output-dir",
        str(directory / "artifacts"),
    ]


def pinned_command(command: Sequence[str], cpu: int | None) -> list[str]:
    if cpu is None:
        return list(command)
    code = (
        "import os,sys; "
        "os.sched_setaffinity(0,{int(sys.argv[1])}); "
        "os.execv(sys.argv[2],sys.argv[2:])"
    )
    return [sys.executable, "-c", code, str(cpu), *command]


def result_artifact(directory: Path) -> Path | None:
    results = sorted((directory / "artifacts").glob("*_result.json"))
    return results[0] if len(results) == 1 else None


def endpoint_error(result: dict[str, Any]) -> float | None:
    try:
        context = result["benchmark"]["context"]
        starts = context["starts"]
        goals = context["goals"]
        robots = result["robots"]
        errors: list[float] = []
        for index, robot in enumerate(robots):
            path = robot["path"]
            for actual, expected in ((path[0], starts[index]), (path[-1], goals[index])):
                if len(actual) != len(expected):
                    return None
                errors.extend(abs(float(a) - float(b)) for a, b in zip(actual, expected))
        return max(errors, default=0.0)
    except (KeyError, IndexError, TypeError, ValueError):
        return None


def diagnostic_summary(
    metrics: dict[str, Any] | None,
    result: dict[str, Any] | None,
    full_history_requested: bool,
) -> dict[str, Any]:
    planner_stats = metrics.get("planner_stats", {}) if metrics else {}
    if not isinstance(planner_stats, dict):
        planner_stats = {}
    events = planner_stats.get("repair_attempt_events")
    event_list = events if isinstance(events, list) else []

    guided_history = result.get("guided_arc_resolution_history") if result else None
    guided_conflicts = (
        guided_history.get("conflicts", [])
        if isinstance(guided_history, dict)
        else []
    )
    guided_attempts = [
        attempt
        for conflict in guided_conflicts
        if isinstance(conflict, dict)
        for attempt in conflict.get("attempts", [])
        if isinstance(attempt, dict)
    ]
    attempts = guided_attempts or event_list
    team_sizes = [
        len(attempt["robots"])
        for attempt in attempts
        if isinstance(attempt.get("robots"), list)
    ]
    window_widths = [
        attempt["window_end_t"] - attempt["window_start_t"] + 1
        for attempt in attempts
        if isinstance(attempt.get("window_start_t"), int)
        and isinstance(attempt.get("window_end_t"), int)
    ]
    return {
        "num_conflicts": planner_stats.get("num_conflicts"),
        "subproblem_attempts": planner_stats.get("subproblem_attempts"),
        "resolved_repairs": (
            guided_history.get("resolved_conflict_count")
            if isinstance(guided_history, dict)
            else sum(bool(event.get("resolved")) for event in event_list)
        ),
        "history_attempt_count": len(attempts) if attempts else None,
        "selected_robot_set_sizes": team_sizes,
        "window_widths_timesteps": window_widths,
        "diagnostics_complete": bool(
            full_history_requested and metrics and metrics.get("success")
        ),
        "diagnostics_note": (
            "Full repair history was not requested for this seed."
            if not full_history_requested
            else
            "GuidedARC history contains resolved conflicts only; timeout-era "
            "pending attempts may be absent."
            if guided_attempts or (metrics and metrics.get("planner") == "GuidedARC")
            else "Baseline ARC repair_attempt_events are used."
        ),
    }


def classify_trial(
    *,
    timed_out: bool,
    returncode: int | None,
    metrics: dict[str, Any] | None,
    result: dict[str, Any] | None,
    max_endpoint_error: float | None,
) -> str:
    if timed_out:
        return "external_timeout"
    if returncode is not None and returncode < 0:
        return "crash_signal"
    if returncode not in (0, None):
        return "nonzero_exit"
    if metrics is None:
        return "missing_metrics"
    if not metrics.get("success", False):
        status = str(metrics.get("planner_status", "")).lower()
        return "planner_timeout" if "time" in status else "planner_failure"
    if result is None:
        return "validation_missing_artifact"
    if result.get("verification_conflict") is not None:
        return "validation_collision"
    if max_endpoint_error is None or max_endpoint_error > 1e-9:
        return "validation_endpoint"
    return "success"


def run_trial(
    trial: Trial,
    *,
    build_dir: Path,
    output_root: Path,
    cpu: int | None,
    existing: str,
    track_history: bool,
    total_trials: int,
) -> dict[str, Any]:
    directory = trial_dir(output_root, trial)
    record_path = directory / "trial.json"
    if record_path.exists():
        if existing == "skip":
            existing_record = read_json(record_path)
            if existing_record is not None:
                print(
                    f"[{trial.ordinal:02d}/{total_trials}] skip {trial.trial_id}",
                    flush=True,
                )
                return existing_record
        elif existing == "error":
            raise RuntimeError(f"Trial record already exists: {record_path}")
        else:
            shutil.rmtree(directory)

    directory.mkdir(parents=True, exist_ok=True)
    command = trial_command(build_dir, output_root, trial, track_history)
    launch_command = pinned_command(command, cpu)
    print(
        f"[{trial.ordinal:02d}/{total_trials}] run {trial.trial_id}", flush=True
    )

    started_utc = utc_now()
    process_start = time.monotonic()
    timed_out = False
    returncode: int | None = None
    stdout = ""
    stderr = ""
    try:
        completed = subprocess.run(
            launch_command,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
            timeout=trial.case.time_limit + TIMEOUT_GRACE_SECONDS,
        )
        returncode = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as error:
        timed_out = True
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else (error.stdout or "")
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else (error.stderr or "")
    process_wall_seconds = time.monotonic() - process_start
    (directory / "stdout.txt").write_text(stdout, encoding="utf-8")
    (directory / "stderr.txt").write_text(stderr, encoding="utf-8")

    metrics_path = directory / "metrics.json"
    metrics = read_json(metrics_path)
    artifact_path = result_artifact(directory)
    result = read_json(artifact_path) if artifact_path else None
    max_endpoint_error = endpoint_error(result) if result else None
    outcome = classify_trial(
        timed_out=timed_out,
        returncode=returncode,
        metrics=metrics,
        result=result,
        max_endpoint_error=max_endpoint_error,
    )
    full_history_requested = track_history
    diagnostics = diagnostic_summary(
        metrics, result, full_history_requested
    )
    record = {
        "schema": "comotion.guided_arc_policy_pilot.trial.v1",
        "trial_id": trial.trial_id,
        "block_key": trial.block_key,
        "ordinal": trial.ordinal,
        "case": trial.case.key,
        "family": trial.case.family,
        "seed": trial.seed,
        "variant": trial.variant.key,
        "algorithm": trial.variant.algorithm,
        "robot_selection_policy": trial.variant.robot_policy,
        "started_utc": started_utc,
        "completed_utc": utc_now(),
        "command": command,
        "launch_command": launch_command,
        "cpu_affinity": cpu,
        "time_limit_seconds": trial.case.time_limit,
        "external_timeout_seconds": trial.case.time_limit + TIMEOUT_GRACE_SECONDS,
        "process_wall_seconds": process_wall_seconds,
        "returncode": returncode,
        "external_timed_out": timed_out,
        "full_history_requested": full_history_requested,
        "outcome": outcome,
        "metrics_path": str(metrics_path.relative_to(output_root)),
        "result_artifact": (
            str(artifact_path.relative_to(output_root)) if artifact_path else None
        ),
        "stdout_path": str((directory / "stdout.txt").relative_to(output_root)),
        "stderr_path": str((directory / "stderr.txt").relative_to(output_root)),
        "planner_status": metrics.get("planner_status") if metrics else None,
        "success": bool(metrics and metrics.get("success") and outcome == "success"),
        "planning_time_seconds": metrics.get("planning_time_seconds") if metrics else None,
        "makespan_timesteps": metrics.get("makespan_timesteps") if metrics else None,
        "sum_of_cost_timesteps": metrics.get("sum_of_cost_timesteps") if metrics else None,
        "verification_conflict": result.get("verification_conflict") if result else None,
        "max_endpoint_error": max_endpoint_error,
        "instance_context": metrics.get("benchmark_context") if metrics else None,
        "diagnostics": diagnostics,
    }
    write_json(record_path, record)
    return record


def instance_payload(record: dict[str, Any], resource_hashes: dict[str, str]) -> Any:
    context = record.get("instance_context")
    if not isinstance(context, dict):
        return None
    return {
        "benchmark_context": context,
        "collision_backend": "vamp",
        "resources": resource_hashes,
    }


def write_instance_checks(
    output_root: Path,
    records: Sequence[dict[str, Any]],
    metadata: dict[str, Any],
) -> dict[str, bool]:
    equality: dict[str, bool] = {}
    resources = metadata["resources"]
    for block_key in sorted({str(record["block_key"]) for record in records}):
        block_records = [record for record in records if record["block_key"] == block_key]
        case = next(case for case in CASES if case.key == block_records[0]["case"])
        case_resources = {path: resources[path] for path in case.resource_paths}
        fingerprints: dict[str, str | None] = {}
        payloads: dict[str, Any] = {}
        for record in block_records:
            payload = instance_payload(record, case_resources)
            payloads[record["variant"]] = payload
            fingerprints[record["variant"]] = (
                canonical_sha256(payload) if payload is not None else None
            )
        complete = len(block_records) == len(VARIANTS) and all(fingerprints.values())
        equal = complete and len(set(fingerprints.values())) == 1
        equality[block_key] = equal
        write_json(
            output_root / "instance_checks" / f"{block_key}.json",
            {
                "schema": "comotion.guided_arc_policy_pilot.instance_check.v1",
                "block_key": block_key,
                "complete": complete,
                "equal": equal,
                "fingerprints": fingerprints,
                "payloads": payloads if not equal else None,
            },
        )
    return equality


def write_results_csv(
    output_root: Path,
    records: Sequence[dict[str, Any]],
    instance_equality: dict[str, bool],
) -> None:
    columns = (
        "ordinal",
        "case",
        "family",
        "seed",
        "variant",
        "algorithm",
        "robot_selection_policy",
        "outcome",
        "success",
        "planner_status",
        "returncode",
        "external_timed_out",
        "instance_equal",
        "planning_time_seconds",
        "process_wall_seconds",
        "makespan_timesteps",
        "sum_of_cost_timesteps",
        "num_conflicts",
        "subproblem_attempts",
        "resolved_repairs",
        "history_attempt_count",
        "selected_robot_set_sizes",
        "window_widths_timesteps",
        "diagnostics_complete",
        "result_artifact",
    )
    with (output_root / "results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for record in sorted(records, key=lambda item: int(item["ordinal"])):
            diagnostics = record["diagnostics"]
            writer.writerow(
                {
                    **{key: record.get(key) for key in columns},
                    "instance_equal": instance_equality.get(record["block_key"], False),
                    "num_conflicts": diagnostics.get("num_conflicts"),
                    "subproblem_attempts": diagnostics.get("subproblem_attempts"),
                    "resolved_repairs": diagnostics.get("resolved_repairs"),
                    "history_attempt_count": diagnostics.get("history_attempt_count"),
                    "selected_robot_set_sizes": json.dumps(
                        diagnostics.get("selected_robot_set_sizes", [])
                    ),
                    "window_widths_timesteps": json.dumps(
                        diagnostics.get("window_widths_timesteps", [])
                    ),
                    "diagnostics_complete": diagnostics.get("diagnostics_complete"),
                }
            )


def selected_cases(case_keys: Sequence[str]) -> tuple[Case, ...]:
    if not case_keys:
        return CASES
    requested = set(case_keys)
    unknown = requested - {case.key for case in CASES}
    if unknown:
        raise RuntimeError(f"Unknown cases: {', '.join(sorted(unknown))}")
    return tuple(case for case in CASES if case.key in requested)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Launch planner processes. Without this flag, only print commands.",
    )
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--seeds", default="0,1,2,3,4")
    parser.add_argument("--cpu", type=int)
    parser.add_argument("--no-affinity", action="store_true")
    parser.add_argument(
        "--existing", choices=("error", "skip", "overwrite"), default="error"
    )
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="Allow execution from a dirty Git worktree.",
    )
    parser.add_argument(
        "--no-history",
        action="store_true",
        help="Do not embed full repair history for seed 0; final paths are still exported.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cases = selected_cases(args.case)
    seeds = tuple(int(token) for token in args.seeds.split(",") if token.strip())
    if not seeds:
        raise RuntimeError("At least one seed is required")
    trials = build_trials(cases, seeds)
    expected_count = len(cases) * len(seeds) * len(VARIANTS)
    if len(trials) != expected_count or len({trial.trial_id for trial in trials}) != len(trials):
        raise RuntimeError("Trial matrix is incomplete or contains duplicates")
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output_root = args.output_root or (
        DEFAULT_RESULTS_DIR / f"guided_arc_policy_pilot_{timestamp}"
    )
    available_cpus = (
        sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []
    )
    cpu = None if args.no_affinity else (
        args.cpu if args.cpu is not None else (available_cpus[0] if available_cpus else None)
    )
    if cpu is not None and available_cpus and cpu not in available_cpus:
        raise RuntimeError(f"CPU {cpu} is outside the available affinity set")

    print(f"mode: {'execute' if args.execute else 'dry-run'}")
    print(f"trials: {len(trials)}")
    print(f"output_root: {output_root}")
    print(f"cpu_affinity: {cpu}")
    for trial in trials:
        track_history = trial.seed == 0 and not args.no_history
        print(
            f"[{trial.ordinal:02d}/{len(trials)}] {trial.trial_id}\n  "
            + " ".join(
                trial_command(args.build_dir, output_root, trial, track_history)
            )
        )
    if not args.execute:
        return 0

    for case in cases:
        executable = args.build_dir / case.executable
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f"Missing executable: {executable}")
        for resource in case.resource_paths:
            if not (REPO_ROOT / resource).is_file():
                raise RuntimeError(f"Missing resource: {resource}")
    status = command_output(["git", "status", "--porcelain"])
    if status and not args.allow_dirty:
        raise RuntimeError("Refusing to execute from a dirty worktree; use --allow-dirty")

    metadata = repository_metadata(args.build_dir)
    manifest = {
        "schema": "comotion.guided_arc_policy_pilot.manifest.v1",
        "created_utc": utc_now(),
        "runner_command": sys.argv,
        "output_root": str(output_root),
        "trial_count": len(trials),
        "serial_execution": True,
        "cpu_affinity": cpu,
        "timeout_grace_seconds": TIMEOUT_GRACE_SECONDS,
        "full_history_seeds": [
            seed for seed in seeds if seed == 0 and not args.no_history
        ],
        "seeds": list(seeds),
        "case_keys": [case.key for case in cases],
        "variant_keys": [variant.key for variant in VARIANTS],
        "order": [trial.trial_id for trial in trials],
        "planning_time_boundary": (
            "Application steady-clock interval around planner->solve(); includes "
            "conflict scanning and current-component graph construction, excludes "
            "scenario setup, result serialization, and post-export verification."
        ),
        "validation_scope": (
            "The result artifact's verification_conflict is a post-export scan "
            "using the configured C++ collision checker; the runner reads that "
            "result and checks endpoints but does not independently recompute collisions."
        ),
        "diagnostic_limitations": (
            "GuidedARC dedicated history contains resolved conflicts only, so pending "
            "attempts on unsuccessful runs may be missing."
        ),
        "repository": metadata,
        "cases": [
            {
                "key": case.key,
                "family": case.family,
                "executable": case.executable,
                "scenario_args": list(case.scenario_args),
                "profile_args": list(case.profile_args),
                "time_limit_seconds": case.time_limit,
                "resource_paths": list(case.resource_paths),
            }
            for case in cases
        ],
        "variants": [
            {
                "key": variant.key,
                "algorithm": variant.algorithm,
                "robot_selection_policy": variant.robot_policy,
            }
            for variant in VARIANTS
        ],
        "common_args": list(COMMON_ARGS),
    }
    write_json(output_root / "manifest.json", manifest)

    records = [
        run_trial(
            trial,
            build_dir=args.build_dir,
            output_root=output_root,
            cpu=cpu,
            existing=args.existing,
            track_history=trial.seed == 0 and not args.no_history,
            total_trials=len(trials),
        )
        for trial in trials
    ]
    instance_equality = write_instance_checks(output_root, records, metadata)
    write_results_csv(output_root, records, instance_equality)
    failures = sum(record["outcome"] != "success" for record in records)
    unequal_blocks = sum(not equal for equal in instance_equality.values())
    print(f"completed: {len(records)} trials")
    print(f"non-success outcomes retained: {failures}")
    print(f"instance-equality failures: {unequal_blocks}")
    print(f"results: {output_root / 'results.csv'}")
    return 0 if unequal_blocks == 0 else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)