#!/usr/bin/env python3
"""Combine GuidedARC policy pilot roots into comparative reports."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable, Sequence


VARIANTS = (
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
    "process_wall_seconds",
    "makespan_timesteps",
    "sum_of_cost_timesteps",
    "diagnostics_complete",
}

OUTPUT_NAMES = (
    "comparative_report.md",
    "merged_results.csv",
    "summary_by_case_variant.csv",
    "paired_baseline_deltas.csv",
)


def parse_bool(value: str, location: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no"}:
        return False
    raise ValueError(f"{location} must be a boolean, got {value!r}")


def optional_float(value: str) -> float | None:
    return float(value) if value.strip() else None


def median(values: Iterable[float | None]) -> float | None:
    present = [value for value in values if value is not None]
    return statistics.median(present) if present else None


def format_number(value: float | None, digits: int = 3) -> str:
    return "NA" if value is None else f"{value:.{digits}f}"


def markdown_cell(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def read_manifest(root: Path) -> dict[str, Any]:
    path = root / "manifest.json"
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Cannot read manifest {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"Manifest must contain an object: {path}")
    return value


def read_results(root: Path) -> tuple[list[str], list[dict[str, str]]]:
    path = root / "results.csv"
    try:
        handle = path.open(newline="", encoding="utf-8")
    except OSError as error:
        raise ValueError(f"Cannot read results {path}: {error}") from error
    with handle:
        reader = csv.DictReader(handle, skipinitialspace=True)
        fieldnames = [name.strip() for name in reader.fieldnames or ()]
        if len(fieldnames) != len(set(fieldnames)):
            raise ValueError(f"Duplicate columns after trimming: {path}")
        missing = sorted(REQUIRED_COLUMNS - set(fieldnames))
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
        reader.fieldnames = fieldnames
        rows = []
        for line_number, row in enumerate(reader, 2):
            if None in row:
                raise ValueError(f"Unexpected extra fields at {path}:{line_number}")
            normalized = {
                key.strip(): value.strip() if value is not None else ""
                for key, value in row.items()
            }
            normalized["source_dataset"] = root.name
            normalized["source_root"] = str(root)
            rows.append(normalized)
    if not rows:
        raise ValueError(f"Results contain no trials: {path}")
    return fieldnames, rows


def validate_rows(rows: Sequence[dict[str, str]]) -> None:
    blocks: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    identities: set[tuple[str, int, str]] = set()
    for row in rows:
        try:
            seed = int(row["seed"])
        except ValueError as error:
            raise ValueError(f"Invalid seed in {row['source_dataset']}: {row['seed']!r}") from error
        variant = row["variant"]
        if variant not in VARIANTS:
            raise ValueError(f"Unknown variant: {variant!r}")
        identity = (row["case"], seed, variant)
        if identity in identities:
            raise ValueError(f"Duplicate trial identity: {identity}")
        identities.add(identity)
        blocks[(row["case"], seed)].append(row)

    expected = set(VARIANTS)
    for block_key, block in sorted(blocks.items()):
        actual = {row["variant"] for row in block}
        if actual != expected:
            raise ValueError(
                f"Incomplete paired block {block_key}: expected {sorted(expected)}, "
                f"got {sorted(actual)}"
            )
        if not all(
            parse_bool(row["instance_equal"], f"{block_key}.instance_equal")
            for row in block
        ):
            raise ValueError(f"Instance mismatch in paired block {block_key}")


def case_limits(manifests: Sequence[dict[str, Any]]) -> dict[str, float]:
    limits: dict[str, float] = {}
    for manifest in manifests:
        for case in manifest.get("cases", []):
            if not isinstance(case, dict) or "key" not in case:
                continue
            key = str(case["key"])
            limit = float(case["time_limit_seconds"])
            if key in limits and limits[key] != limit:
                raise ValueError(f"Conflicting time limits for {key}")
            limits[key] = limit
    return limits


def summarize(rows: Sequence[dict[str, str]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        groups[(row["case"], row["variant"])].append(row)

    summaries = []
    for (case, variant), group in sorted(
        groups.items(), key=lambda item: (item[0][0], VARIANTS.index(item[0][1]))
    ):
        successful = [
            row
            for row in group
            if parse_bool(row["success"], f"{case}.{variant}.success")
        ]
        outcomes = Counter(row["outcome"] for row in group)
        summaries.append(
            {
                "family": group[0]["family"],
                "case": case,
                "variant": variant,
                "n": len(group),
                "success_count": len(successful),
                "success_rate": len(successful) / len(group),
                "timeout_count": outcomes.get("planner_timeout", 0)
                + outcomes.get("external_timeout", 0),
                "outcomes": ";".join(
                    f"{name}:{count}" for name, count in sorted(outcomes.items())
                ),
                "median_planning_time_seconds_all": median(
                    optional_float(row["planning_time_seconds"]) for row in group
                ),
                "median_planning_time_seconds_success": median(
                    optional_float(row["planning_time_seconds"]) for row in successful
                ),
                "median_makespan_timesteps_success": median(
                    optional_float(row["makespan_timesteps"]) for row in successful
                ),
                "median_sum_of_cost_timesteps_success": median(
                    optional_float(row["sum_of_cost_timesteps"]) for row in successful
                ),
                "diagnostics_complete_count": sum(
                    parse_bool(
                        row["diagnostics_complete"],
                        f"{case}.{variant}.diagnostics_complete",
                    )
                    for row in group
                ),
                "source_datasets": ";".join(
                    sorted({row["source_dataset"] for row in group})
                ),
            }
        )
    return summaries


def paired_deltas(rows: Sequence[dict[str, str]]) -> list[dict[str, Any]]:
    blocks: dict[tuple[str, int], dict[str, dict[str, str]]] = defaultdict(dict)
    for row in rows:
        blocks[(row["case"], int(row["seed"]))][row["variant"]] = row

    deltas = []
    for (case, seed), block in sorted(blocks.items()):
        baseline = block["baseline_arc"]
        baseline_success = parse_bool(baseline["success"], "baseline.success")
        baseline_time = optional_float(baseline["planning_time_seconds"])
        baseline_makespan = optional_float(baseline["makespan_timesteps"])
        for variant in VARIANTS[1:]:
            policy = block[variant]
            policy_success = parse_bool(policy["success"], f"{variant}.success")
            policy_time = optional_float(policy["planning_time_seconds"])
            policy_makespan = optional_float(policy["makespan_timesteps"])
            both_success = baseline_success and policy_success
            deltas.append(
                {
                    "family": policy["family"],
                    "case": case,
                    "seed": seed,
                    "variant": variant,
                    "baseline_outcome": baseline["outcome"],
                    "policy_outcome": policy["outcome"],
                    "both_success": both_success,
                    "planning_time_delta_seconds": (
                        policy_time - baseline_time
                        if policy_time is not None and baseline_time is not None
                        else None
                    ),
                    "planning_time_ratio": (
                        policy_time / baseline_time
                        if policy_time is not None
                        and baseline_time is not None
                        and baseline_time > 0
                        else None
                    ),
                    "makespan_delta_timesteps": (
                        policy_makespan - baseline_makespan
                        if both_success
                        and policy_makespan is not None
                        and baseline_makespan is not None
                        else None
                    ),
                    "makespan_ratio": (
                        policy_makespan / baseline_makespan
                        if both_success
                        and policy_makespan is not None
                        and baseline_makespan is not None
                        and baseline_makespan > 0
                        else None
                    ),
                    "source_dataset": policy["source_dataset"],
                }
            )
    return deltas


def write_csv(path: Path, rows: Sequence[dict[str, Any]], fields: Sequence[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_report(
    path: Path,
    rows: Sequence[dict[str, str]],
    summaries: Sequence[dict[str, Any]],
    deltas: Sequence[dict[str, Any]],
    roots: Sequence[Path],
    manifests: Sequence[dict[str, Any]],
    limits: dict[str, float],
) -> None:
    blocks = {(row["case"], int(row["seed"])) for row in rows}
    successes = sum(parse_bool(row["success"], "success") for row in rows)
    outcomes = Counter(row["outcome"] for row in rows)
    revisions = sorted(
        {
            str(manifest.get("repository", {}).get("git_revision", "unknown"))
            for manifest in manifests
        }
    )
    lines = [
        "# GuidedARC Policy Comparative Report",
        "",
        "## Scope",
        "",
        f"- Trials: {len(rows)}",
        f"- Paired case/seed blocks: {len(blocks)}",
        f"- Cases: {len({row['case'] for row in rows})}",
        f"- Successful trials: {successes}/{len(rows)} ({100 * successes / len(rows):.1f}%)",
        f"- Outcomes: {', '.join(f'{name}={count}' for name, count in sorted(outcomes.items()))}",
        "- Pairing and instance equality: passed",
        "",
        "## Data Sources",
        "",
        "| Dataset | Trials | Cases | Seeds | Git revision |",
        "|---|---:|---|---|---|",
    ]
    for root, manifest in zip(roots, manifests):
        source_rows = [row for row in rows if row["source_dataset"] == root.name]
        source_cases = sorted({row["case"] for row in source_rows})
        source_seeds = sorted({int(row["seed"]) for row in source_rows})
        revision = manifest.get("repository", {}).get("git_revision", "unknown")
        lines.append(
            f"| {markdown_cell(root.name)} | {len(source_rows)} | "
            f"{markdown_cell(', '.join(source_cases))} | "
            f"{markdown_cell(','.join(map(str, source_seeds)))} | "
            f"{markdown_cell(revision)} |"
        )

    lines.extend(
        [
            "",
            "## Primary Outcomes",
            "",
            "Planning-time medians include timeout-censored trials in the `all` column. "
            "Makespan medians use successful trials only.",
            "",
            "| Family | Case | Limit (s) | Variant | Success | Time median, all (s) | "
            "Time median, success (s) | Makespan median, success | Outcomes |",
            "|---|---|---:|---|---:|---:|---:|---:|---|",
        ]
    )
    for summary in summaries:
        lines.append(
            f"| {markdown_cell(summary['family'])} | {markdown_cell(summary['case'])} | "
            f"{format_number(limits.get(summary['case']), 0)} | "
            f"{markdown_cell(summary['variant'])} | "
            f"{summary['success_count']}/{summary['n']} | "
            f"{format_number(summary['median_planning_time_seconds_all'])} | "
            f"{format_number(summary['median_planning_time_seconds_success'])} | "
            f"{format_number(summary['median_makespan_timesteps_success'], 1)} | "
            f"{markdown_cell(summary['outcomes'])} |"
        )

    lines.extend(
        [
            "",
            "## Paired Baseline Comparisons",
            "",
            "Ratios below use only blocks where baseline ARC and the policy both succeeded. "
            "Values below 1 favor the GuidedARC policy.",
            "",
            "| Case | Policy | Successful pairs | Median time ratio | Median makespan ratio |",
            "|---|---|---:|---:|---:|",
        ]
    )
    delta_groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for delta in deltas:
        delta_groups[(delta["case"], delta["variant"])].append(delta)
    for (case, variant), group in sorted(delta_groups.items()):
        successful = [delta for delta in group if delta["both_success"]]
        lines.append(
            f"| {markdown_cell(case)} | {markdown_cell(variant)} | "
            f"{len(successful)}/{len(group)} | "
            f"{format_number(median(delta['planning_time_ratio'] for delta in successful))} | "
            f"{format_number(median(delta['makespan_ratio'] for delta in successful))} |"
        )

    failures = [row for row in rows if not parse_bool(row["success"], "success")]
    lines.extend(
        [
            "",
            "## Non-successful Trials",
            "",
            "| Case | Seed | Variant | Outcome | Planning time (s) |",
            "|---|---:|---|---|---:|",
        ]
    )
    for row in sorted(
        failures,
        key=lambda item: (item["case"], int(item["seed"]), item["variant"]),
    ):
        lines.append(
            f"| {markdown_cell(row['case'])} | {row['seed']} | "
            f"{markdown_cell(row['variant'])} | {markdown_cell(row['outcome'])} | "
            f"{format_number(optional_float(row['planning_time_seconds']))} |"
        )

    complete_diagnostics = sum(
        parse_bool(row["diagnostics_complete"], "diagnostics_complete") for row in rows
    )
    lines.extend(
        [
            "",
            "## Interpretation Notes",
            "",
            f"- Complete repair-history diagnostics are available for {complete_diagnostics}/{len(rows)} trials; "
            "missing diagnostics are not treated as zero.",
            "- Time limits and ARC profiles differ by benchmark family. Cross-family timing comparisons "
            "are descriptive, not controlled head-to-head comparisons.",
            "- The p8/s16 heterogeneous case contains one paired seed and is a feasibility result, "
            "not a five-seed estimate.",
            "- Final path artifacts were checked by the original runner for reported collisions and endpoints.",
            f"- Source manifests reference {len(revisions)} Git revision(s): {', '.join(revisions)}.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pilot_roots", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace existing comparative output files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    roots = [root.resolve() for root in args.pilot_roots]
    if len(roots) != len(set(roots)):
        raise ValueError("Pilot roots must be unique")

    manifests = []
    all_rows: list[dict[str, str]] = []
    canonical_fields: list[str] | None = None
    for root in roots:
        manifests.append(read_manifest(root))
        fields, rows = read_results(root)
        if canonical_fields is None:
            canonical_fields = fields
        elif fields != canonical_fields:
            raise ValueError(f"CSV schema or column order differs in {root}")
        all_rows.extend(rows)

    validate_rows(all_rows)
    limits = case_limits(manifests)
    summaries = summarize(all_rows)
    deltas = paired_deltas(all_rows)

    output_dir = args.output_dir
    existing = [output_dir / name for name in OUTPUT_NAMES if (output_dir / name).exists()]
    if existing and not args.overwrite:
        raise ValueError(
            "Refusing to replace existing outputs: "
            + ", ".join(str(path) for path in existing)
        )
    output_dir.mkdir(parents=True, exist_ok=True)

    merged_fields = ["source_dataset", "source_root", *(canonical_fields or [])]
    write_csv(output_dir / "merged_results.csv", all_rows, merged_fields)
    write_csv(
        output_dir / "summary_by_case_variant.csv",
        summaries,
        tuple(summaries[0]),
    )
    write_csv(
        output_dir / "paired_baseline_deltas.csv",
        deltas,
        tuple(deltas[0]),
    )
    write_report(
        output_dir / "comparative_report.md",
        all_rows,
        summaries,
        deltas,
        roots,
        manifests,
        limits,
    )

    outcome_counts = Counter(row["outcome"] for row in all_rows)
    print(f"trials: {len(all_rows)}")
    print(f"paired blocks: {len({(row['case'], row['seed']) for row in all_rows})}")
    print(f"cases: {len({row['case'] for row in all_rows})}")
    print(
        "outcomes: "
        + ", ".join(f"{name}={count}" for name, count in sorted(outcome_counts.items()))
    )
    print("pairing and instance equality: passed")
    print(f"outputs: {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)