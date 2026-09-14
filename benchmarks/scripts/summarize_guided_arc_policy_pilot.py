#!/usr/bin/env python3
"""Summarize a GuidedARC policy pilot without modifying its results."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


VARIANT_ORDER = (
    "baseline_arc",
    "guided_arc_repair_history",
    "guided_historical_direct_neighbors",
    "guided_current_conflict_component",
)

REQUIRED_COLUMNS = {
    "case",
    "family",
    "seed",
    "variant",
    "outcome",
    "success",
    "instance_equal",
    "planning_time_seconds",
    "makespan_timesteps",
    "num_conflicts",
    "subproblem_attempts",
    "resolved_repairs",
    "history_attempt_count",
    "selected_robot_set_sizes",
    "window_widths_timesteps",
    "diagnostics_complete",
}


@dataclass(frozen=True)
class Trial:
    case: str
    family: str
    seed: int
    variant: str
    outcome: str
    success: bool
    instance_equal: bool
    planning_time_seconds: float | None
    makespan_timesteps: float | None
    num_conflicts: float | None
    subproblem_attempts: float | None
    resolved_repairs: float | None
    history_attempt_count: float | None
    selected_robot_set_sizes: tuple[float, ...]
    window_widths_timesteps: tuple[float, ...]
    diagnostics_complete: bool


def parse_bool(value: str, location: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no"}:
        return False
    raise ValueError(f"{location} must be a boolean, got {value!r}")


def optional_float(value: str, location: str) -> float | None:
    if value.strip() == "":
        return None
    try:
        return float(value)
    except ValueError as error:
        raise ValueError(f"{location} must be numeric, got {value!r}") from error


def numeric_list(value: str, location: str) -> tuple[float, ...]:
    if value.strip() == "":
        return ()
    try:
        decoded = json.loads(value)
    except json.JSONDecodeError as error:
        raise ValueError(f"{location} must be a JSON array") from error
    if not isinstance(decoded, list) or not all(
        isinstance(item, (int, float)) and not isinstance(item, bool)
        for item in decoded
    ):
        raise ValueError(f"{location} must be a numeric JSON array")
    return tuple(float(item) for item in decoded)


def read_trials(path: Path) -> list[Trial]:
    if not path.is_file():
        raise ValueError(f"Results CSV does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle, skipinitialspace=True)
        fieldnames = [name.strip() for name in reader.fieldnames or ()]
        if len(fieldnames) != len(set(fieldnames)):
            raise ValueError("Results CSV has duplicate columns after trimming")
        reader.fieldnames = fieldnames
        columns = set(fieldnames)
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError(f"Results CSV is missing columns: {', '.join(missing)}")

        trials = []
        for line_number, row in enumerate(reader, 2):
            location = f"{path}:{line_number}"
            try:
                seed = int(row["seed"])
            except ValueError as error:
                raise ValueError(
                    f"{location}.seed must be an integer, got {row['seed']!r}"
                ) from error
            variant = row["variant"].strip()
            if variant not in VARIANT_ORDER:
                raise ValueError(f"{location}.variant is unknown: {variant!r}")
            trials.append(
                Trial(
                    case=row["case"].strip(),
                    family=row["family"].strip(),
                    seed=seed,
                    variant=variant,
                    outcome=row["outcome"].strip(),
                    success=parse_bool(row["success"], f"{location}.success"),
                    instance_equal=parse_bool(
                        row["instance_equal"], f"{location}.instance_equal"
                    ),
                    planning_time_seconds=optional_float(
                        row["planning_time_seconds"],
                        f"{location}.planning_time_seconds",
                    ),
                    makespan_timesteps=optional_float(
                        row["makespan_timesteps"],
                        f"{location}.makespan_timesteps",
                    ),
                    num_conflicts=optional_float(
                        row["num_conflicts"], f"{location}.num_conflicts"
                    ),
                    subproblem_attempts=optional_float(
                        row["subproblem_attempts"],
                        f"{location}.subproblem_attempts",
                    ),
                    resolved_repairs=optional_float(
                        row["resolved_repairs"], f"{location}.resolved_repairs"
                    ),
                    history_attempt_count=optional_float(
                        row["history_attempt_count"],
                        f"{location}.history_attempt_count",
                    ),
                    selected_robot_set_sizes=numeric_list(
                        row["selected_robot_set_sizes"],
                        f"{location}.selected_robot_set_sizes",
                    ),
                    window_widths_timesteps=numeric_list(
                        row["window_widths_timesteps"],
                        f"{location}.window_widths_timesteps",
                    ),
                    diagnostics_complete=parse_bool(
                        row["diagnostics_complete"],
                        f"{location}.diagnostics_complete",
                    ),
                )
            )
    if not trials:
        raise ValueError(f"Results CSV contains no trials: {path}")
    return trials


def format_number(value: float | None, digits: int = 3) -> str:
    return "NA" if value is None else f"{value:.{digits}f}"


def mean(values: Iterable[float | None]) -> float | None:
    present = [value for value in values if value is not None]
    return statistics.fmean(present) if present else None


def median(values: Iterable[float | None]) -> float | None:
    present = [value for value in values if value is not None]
    return statistics.median(present) if present else None


def flattened_mean(values: Iterable[Sequence[float]]) -> float | None:
    flattened = [item for group in values for item in group]
    return statistics.fmean(flattened) if flattened else None


def validate_pairs(trials: Sequence[Trial]) -> list[str]:
    issues = []
    blocks: dict[tuple[str, int], list[Trial]] = defaultdict(list)
    seen: set[tuple[str, int, str]] = set()
    for trial in trials:
        identity = (trial.case, trial.seed, trial.variant)
        if identity in seen:
            issues.append(f"duplicate trial: {trial.case} seed={trial.seed} {trial.variant}")
        seen.add(identity)
        blocks[(trial.case, trial.seed)].append(trial)

    expected = set(VARIANT_ORDER)
    for (case, seed), block in sorted(blocks.items()):
        actual = {trial.variant for trial in block}
        if actual != expected:
            missing = ",".join(sorted(expected - actual)) or "none"
            extra = ",".join(sorted(actual - expected)) or "none"
            issues.append(
                f"incomplete block: {case} seed={seed} missing={missing} extra={extra}"
            )
        if not all(trial.instance_equal for trial in block):
            issues.append(f"instance mismatch: {case} seed={seed}")
    return issues


def print_trials(trials: Sequence[Trial]) -> None:
    print("TRIALS")
    for trial in sorted(
        trials,
        key=lambda item: (item.case, item.seed, VARIANT_ORDER.index(item.variant)),
    ):
        print(
            f"  {trial.case:28} seed={trial.seed:<2} "
            f"{trial.variant:44} outcome={trial.outcome:<28} "
            f"time={format_number(trial.planning_time_seconds)} "
            f"makespan={format_number(trial.makespan_timesteps, 0)} "
            f"instance_equal={trial.instance_equal}"
        )


def print_summary(trials: Sequence[Trial]) -> None:
    print("\nPRIMARY OUTCOMES")
    print(
        "  case                         variant                                      "
        "n  success   total_s  median_s  median_makespan  outcomes"
    )
    groups: dict[tuple[str, str], list[Trial]] = defaultdict(list)
    for trial in trials:
        groups[(trial.case, trial.variant)].append(trial)
    for case in sorted({trial.case for trial in trials}):
        for variant in VARIANT_ORDER:
            group = groups.get((case, variant), [])
            if not group:
                continue
            successful = [trial for trial in group if trial.success]
            times = [trial.planning_time_seconds for trial in group]
            total_time = sum(value for value in times if value is not None)
            outcome_counts = Counter(trial.outcome for trial in group)
            outcomes = ",".join(
                f"{name}:{count}" for name, count in sorted(outcome_counts.items())
            )
            print(
                f"  {case:28} {variant:44} {len(group):<2} "
                f"{len(successful):>2}/{len(group):<2} "
                f"{total_time:>8.3f}  {format_number(median(times)):>8}  "
                f"{format_number(median(t.makespan_timesteps for t in successful), 1):>15}  "
                f"{outcomes}"
            )


def print_diagnostics(trials: Sequence[Trial]) -> None:
    print("\nDIAGNOSTICS")
    print(
        "  variant                                      complete  mean_conflicts  "
        "mean_attempts  mean_repairs  mean_team  mean_window"
    )
    for variant in VARIANT_ORDER:
        group = [trial for trial in trials if trial.variant == variant]
        if not group:
            continue
        complete = sum(trial.diagnostics_complete for trial in group)
        print(
            f"  {variant:44} {complete:>3}/{len(group):<3}     "
            f"{format_number(mean(t.num_conflicts for t in group)):>10}      "
            f"{format_number(mean(t.subproblem_attempts for t in group)):>10}   "
            f"{format_number(mean(t.resolved_repairs for t in group)):>10}  "
            f"{format_number(flattened_mean(t.selected_robot_set_sizes for t in group)):>9}  "
            f"{format_number(flattened_mean(t.window_widths_timesteps for t in group)):>11}"
        )
    incomplete = sum(not trial.diagnostics_complete for trial in trials)
    if incomplete:
        print(
            f"  note: {incomplete} trial(s) have incomplete diagnostics; missing "
            "values are reported as NA, never zero."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "pilot_root",
        type=Path,
        help="Pilot output directory containing results.csv, or the CSV itself.",
    )
    parser.add_argument(
        "--summary-only", action="store_true", help="Omit individual trial rows."
    )
    parser.add_argument(
        "--allow-pairing-issues",
        action="store_true",
        help="Return success despite incomplete blocks or instance mismatches.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results_path = (
        args.pilot_root
        if args.pilot_root.suffix.lower() == ".csv"
        else args.pilot_root / "results.csv"
    )
    trials = read_trials(results_path)
    issues = validate_pairs(trials)
    print(f"results: {results_path}")
    print(f"trials: {len(trials)}")
    print(f"paired blocks: {len({(trial.case, trial.seed) for trial in trials})}")
    if issues:
        print("PAIRING/INSTANCE ISSUES")
        for issue in issues:
            print(f"  {issue}")
    else:
        print("pairing and instance equality: passed")
    if not args.summary_only:
        print_trials(trials)
    print_summary(trials)
    print_diagnostics(trials)
    return 0 if not issues or args.allow_pairing_issues else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)