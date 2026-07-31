#!/usr/bin/env python3
"""Run early-termination or exhaustive validation replay in parallel batches.

The benchmark app reports validation time as the sum of per-record validation
calls. This runner splits motion records into disjoint trace files, executes
them in separate pinned worker processes, then merges the additive metrics.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any


VARIANTS = [
    "fcl",
    "sphere",
    "vamp_combined_rake",
    "vamp_combined_linear",
    "vamp_hierarchical_rake",
    "vamp_hierarchical_linear",
]


@dataclass(frozen=True)
class Case:
    robots: int
    obstacles: bool

    @property
    def tag(self) -> str:
        suffix = "" if self.obstacles else "_empty"
        return f"n{self.robots}{suffix}"

    @property
    def label(self) -> str:
        env = "obstacles" if self.obstacles else "empty"
        return f"n={self.robots} {env}"


def parse_cases(value: str) -> list[Case]:
    if value == "all":
        return [
            Case(4, True),
            Case(4, False),
            Case(8, True),
            Case(8, False),
            Case(16, True),
            Case(16, False),
        ]

    out: list[Case] = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        empty = token.endswith("_empty")
        base = token[:-6] if empty else token
        if not base.startswith("n"):
            raise ValueError(f"case must look like n4 or n4_empty: {token}")
        out.append(Case(int(base[1:]), not empty))
    return out


def load_json(path: pathlib.Path) -> dict[str, Any]:
    with path.open() as f:
        return json.load(f)


def write_json(path: pathlib.Path, doc: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    tmp.replace(path)


def motion_records(doc: dict[str, Any]) -> list[dict[str, Any]]:
    return [record for record in doc["records"] if record.get("type") == "composite_motion"]


def timestep_count(record: dict[str, Any]) -> int:
    paths = record.get("paths")
    if paths:
        return max(len(path) for path in paths)
    options = record.get("options", {})
    checks = int(options.get("discrete_num_checks_hint", record.get("timesteps", 1)))
    return max(1, checks) + 1


def trace_paths(results_dir: pathlib.Path, case: Case) -> list[pathlib.Path]:
    prefix = f"validation_timing_panda_cage_n{case.robots}"
    if not case.obstacles:
        prefix += "_empty"
    return [
        results_dir / f"{prefix}_seed{seed}_exact1000_trace.json"
        for seed in (0, 1, 2)
    ]


def load_case_records(
    results_dir: pathlib.Path,
    case: Case,
    explicit_paths: list[pathlib.Path] | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    sources = trace_paths(results_dir, case) if explicit_paths is None else explicit_paths
    docs = []
    for path in sources:
        if not path.exists():
            raise FileNotFoundError(path)
        docs.append(load_json(path))

    records: list[dict[str, Any]] = []
    for doc in docs:
        records.extend(motion_records(doc))

    metadata = dict(docs[0])
    metadata["records"] = []
    metadata["source"] = f"validation_timing_panda_cage_{case.tag}_all_seed_motion_batches"
    metadata["seeds"] = sorted(
        {
            int(seed)
            for doc in docs
            for seed in doc.get("seeds", [])
        }
    )
    metadata["seed_summaries"] = [
        summary
        for doc in docs
        for summary in doc.get("seed_summaries", [])
    ]
    metadata["derived_from"] = [str(path) for path in sources]
    metadata["derived_filter"] = "all composite_motion records from source traces"
    return metadata, records


def partition_records(records: list[dict[str, Any]], workers: int) -> list[list[dict[str, Any]]]:
    bins: list[tuple[int, int, list[dict[str, Any]]]] = [(0, idx, []) for idx in range(workers)]
    heapq.heapify(bins)

    indexed = list(enumerate(records))
    indexed.sort(key=lambda item: timestep_count(item[1]), reverse=True)
    for _, record in indexed:
        total, idx, batch = heapq.heappop(bins)
        batch.append(record)
        heapq.heappush(bins, (total + timestep_count(record), idx, batch))

    ordered = sorted(bins, key=lambda item: item[1])
    return [batch for _, _, batch in ordered if batch]


def available_cores(limit: int) -> list[int]:
    try:
        cores = sorted(os.sched_getaffinity(0))
    except AttributeError:
        cores = list(range(os.cpu_count() or 1))
    if len(cores) < limit:
        raise RuntimeError(f"requested {limit} workers but only {len(cores)} CPUs are available")
    return cores[:limit]


def summarize_batches(batches: list[list[dict[str, Any]]]) -> str:
    loads = [sum(timestep_count(record) for record in batch) for batch in batches]
    counts = [len(batch) for batch in batches]
    return (
        f"batches={len(batches)} records={sum(counts)} "
        f"timesteps={sum(loads)} load_min={min(loads)} load_max={max(loads)}"
    )


def metrics_complete(
    path: pathlib.Path,
    expected_variants: list[str],
    exhaustive: bool,
    expected_record_count: int,
    expected_fcl_urdf: str,
) -> bool:
    if not path.exists() or path.stat().st_size == 0:
        return False
    try:
        doc = load_json(path)
    except json.JSONDecodeError:
        return False
    names = [variant.get("name") for variant in doc.get("variants", [])]
    work_complete = all(
        "motion_timesteps_possible" in variant.get("validation_work", {})
        and "motion_timesteps_checked" in variant.get("validation_work", {})
        and "valid_validation_time_seconds" in variant
        and "invalid_validation_time_seconds" in variant
        for variant in doc.get("variants", [])
    )
    fcl_variant = next(
        (
            variant
            for variant in doc.get("variants", [])
            if variant.get("name") == "fcl"
        ),
        None,
    )
    fcl_geometry_complete = (
        "fcl" not in expected_variants
        or (
            fcl_variant is not None
            and fcl_variant.get("robot_urdf_resource")
            == expected_fcl_urdf
            and fcl_variant.get("robot_collision_geometry") == "mesh"
        )
    )
    return (
        doc.get("trace", {}).get("motion_only") is True
        and doc.get("trace", {}).get("exhaustive_motion_validation") is exhaustive
        and doc.get("trace", {}).get("timing_clock") == "cpu"
        and names == expected_variants
        and work_complete
        and fcl_geometry_complete
        and doc.get("trace", {}).get("record_count") == expected_record_count
    )


def run_batch(
    executable: pathlib.Path,
    trace: pathlib.Path,
    metrics: pathlib.Path,
    log: pathlib.Path,
    case: Case,
    variants: list[str],
    core: int,
    progress_interval: int,
    exhaustive: bool,
    fcl_urdf: str,
) -> subprocess.Popen[str]:
    metrics.parent.mkdir(parents=True, exist_ok=True)
    log.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(executable),
        "--mode",
        "replay",
        "--num-robots",
        str(case.robots),
        "--task-index",
        "0",
        "--trace-input",
        str(trace),
        "--motion-only",
        "--timing-clock",
        "cpu",
        "--variants",
        ",".join(variants),
        "--progress-interval",
        str(progress_interval),
        "--metrics-json",
        str(metrics),
        "--fcl-urdf",
        fcl_urdf,
    ]
    if exhaustive:
        cmd.append("--exhaustive-motion-validation")
    if not case.obstacles:
        cmd.append("--empty-environment")

    taskset = shutil.which("taskset")
    if taskset:
        cmd = [taskset, "-c", str(core), *cmd]

    handle = log.open("w")
    process = subprocess.Popen(
        cmd,
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
    )
    process._codex_log_handle = handle  # type: ignore[attr-defined]
    return process


def close_process_log(process: subprocess.Popen[str]) -> None:
    handle = getattr(process, "_codex_log_handle", None)
    if handle is not None:
        handle.close()


def distribution(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0, "min": 0.0, "p50": 0.0, "p90": 0.0, "p99": 0.0, "max": 0.0, "mean": 0.0}
    values = sorted(values)

    def percentile(p: float) -> float:
        if len(values) == 1:
            return values[0]
        pos = p * (len(values) - 1)
        lo = int(pos)
        hi = min(lo + 1, len(values) - 1)
        frac = pos - lo
        return values[lo] * (1.0 - frac) + values[hi] * frac

    return {
        "count": len(values),
        "min": values[0],
        "p50": percentile(0.50),
        "p90": percentile(0.90),
        "p99": percentile(0.99),
        "max": values[-1],
        "mean": sum(values) / len(values),
    }


def merge_variant(name: str, docs: list[dict[str, Any]]) -> dict[str, Any]:
    variants = [
        next(variant for variant in doc["variants"] if variant["name"] == name)
        for doc in docs
    ]
    latencies: list[float] = []
    path_lengths: list[float] = []
    timesteps: list[float] = []
    scope_counts: dict[str, int] = {}
    by_type: dict[str, Any] = {}
    work_keys = [
        "motion_timesteps_possible",
        "motion_timesteps_checked",
        "robot_state_checks_possible",
        "robot_state_checks_completed",
        "robot_pair_checks_possible",
        "robot_pair_checks_completed",
        "simd_packs_checked",
        "simd_lanes_checked",
    ]
    validation_work = {key: 0 for key in work_keys}

    for variant in variants:
        latencies.extend(variant["latency_seconds"].get("values", []))
        path_lengths.extend(variant["path_length"].get("values", []))
        timesteps.extend(variant["timestep_count"].get("values", []))
        for key, value in variant.get("collision_scope_counts", {}).items():
            scope_counts[key] = scope_counts.get(key, 0) + int(value)
        for key in work_keys:
            validation_work[key] += int(
                variant.get("validation_work", {}).get(key, 0)
            )

        for type_name, type_stats in variant.get("by_type", {}).items():
            merged = by_type.setdefault(
                type_name,
                {
                    "count": 0,
                    "valid": 0,
                    "invalid": 0,
                    "total_validation_time_seconds": 0.0,
                    "valid_validation_time_seconds": 0.0,
                    "invalid_validation_time_seconds": 0.0,
                    "result_mismatches_vs_trace": 0,
                    "collision_scope_counts": {},
                    "_latencies": [],
                    "_path_lengths": [],
                    "_timesteps": [],
                    "validation_work": {key: 0 for key in work_keys},
                },
            )
            merged["count"] += int(type_stats["count"])
            merged["valid"] += int(type_stats["valid"])
            merged["invalid"] += int(type_stats["invalid"])
            merged["total_validation_time_seconds"] += float(type_stats["total_validation_time_seconds"])
            merged["valid_validation_time_seconds"] += float(
                type_stats["valid_validation_time_seconds"]
            )
            merged["invalid_validation_time_seconds"] += float(
                type_stats["invalid_validation_time_seconds"]
            )
            merged["result_mismatches_vs_trace"] += int(type_stats["result_mismatches_vs_trace"])
            merged["_latencies"].extend(
                type_stats.get("latency_seconds", {}).get("values", [])
            )
            merged["_path_lengths"].extend(
                type_stats.get("path_length", {}).get("values", [])
            )
            merged["_timesteps"].extend(
                type_stats.get("timestep_count", {}).get("values", [])
            )
            for key, value in type_stats.get("collision_scope_counts", {}).items():
                merged["collision_scope_counts"][key] = merged["collision_scope_counts"].get(key, 0) + int(value)
            for key in work_keys:
                merged["validation_work"][key] += int(
                    type_stats.get("validation_work", {}).get(key, 0)
                )

    for type_stats in by_type.values():
        type_stats["valid_ratio"] = (
            0.0 if type_stats["count"] == 0 else type_stats["valid"] / type_stats["count"]
        )
        type_stats["latency_seconds"] = distribution(type_stats["_latencies"])
        type_stats["path_length"] = distribution(type_stats["_path_lengths"])
        type_stats["timestep_count"] = distribution(type_stats["_timesteps"])
        del type_stats["_latencies"]
        del type_stats["_path_lengths"]
        del type_stats["_timesteps"]
        possible = type_stats["validation_work"]["motion_timesteps_possible"]
        checked = type_stats["validation_work"]["motion_timesteps_checked"]
        type_stats["validation_work"]["motion_timestep_coverage"] = (
            0.0 if possible == 0 else checked / possible
        )

    count = sum(int(variant["count"]) for variant in variants)
    valid = sum(int(variant["valid"]) for variant in variants)
    invalid = sum(int(variant["invalid"]) for variant in variants)
    elapsed = sum(float(variant["total_validation_time_seconds"]) for variant in variants)
    valid_elapsed = sum(
        float(variant["valid_validation_time_seconds"]) for variant in variants
    )
    invalid_elapsed = sum(
        float(variant["invalid_validation_time_seconds"]) for variant in variants
    )
    mismatches = sum(int(variant["result_mismatches_vs_trace"]) for variant in variants)
    provenance_keys = (
        "robot_urdf_resource",
        "robot_urdf_path",
        "robot_collision_geometry",
    )
    provenance = {
        key: variants[0].get(key)
        for key in provenance_keys
    }
    if any(
        variant.get(key) != provenance[key]
        for variant in variants
        for key in provenance_keys
    ):
        raise RuntimeError(
            f"inconsistent robot geometry provenance for {name}"
        )
    return {
        "name": name,
        "backend": variants[0]["backend"],
        **provenance,
        "count": count,
        "valid": valid,
        "invalid": invalid,
        "valid_ratio": 0.0 if count == 0 else valid / count,
        "total_validation_time_seconds": elapsed,
        "valid_validation_time_seconds": valid_elapsed,
        "invalid_validation_time_seconds": invalid_elapsed,
        "result_mismatches_vs_trace": mismatches,
        "collision_scope_counts": scope_counts,
        "latency_seconds": distribution(latencies),
        "path_length": distribution(path_lengths),
        "timestep_count": distribution(timesteps),
        "by_type": by_type,
        "validation_work": {
            **validation_work,
            "motion_timestep_coverage": (
                0.0
                if validation_work["motion_timesteps_possible"] == 0
                else validation_work["motion_timesteps_checked"]
                / validation_work["motion_timesteps_possible"]
            ),
        },
    }


def merge_metrics(
    case: Case,
    batch_metrics: list[pathlib.Path],
    variants: list[str],
    output: pathlib.Path,
    worker_count: int,
    wall_seconds: float,
) -> dict[str, Any]:
    docs = [load_json(path) for path in batch_metrics]
    merged_variants = [merge_variant(name, docs) for name in variants]
    trace = dict(docs[0]["trace"])
    trace["source"] = f"validation_timing_panda_cage_{case.tag}_parallel_exhaustive"
    trace["record_count"] = sum(doc["trace"]["record_count"] for doc in docs)
    trace["source_record_count"] = trace["record_count"]
    trace["motion_record_count"] = sum(doc["trace"]["motion_record_count"] for doc in docs)
    trace["parallel_batches"] = len(docs)
    trace["parallel_workers"] = worker_count
    trace["parallel_wall_time_seconds"] = wall_seconds
    trace["batch_metrics"] = [str(path) for path in batch_metrics]

    seconds = {variant["name"]: variant["total_validation_time_seconds"] for variant in merged_variants}
    speedups = []
    fcl_seconds = seconds.get("fcl", 0.0)
    sphere_seconds = seconds.get("sphere", 0.0)
    for name, value in seconds.items():
        if name.startswith("vamp"):
            speedups.append(
                {
                    "variant": name,
                    "vs_fcl": 0.0 if value == 0.0 else fcl_seconds / value,
                    "vs_sphere": 0.0 if value == 0.0 else sphere_seconds / value,
                }
            )

    doc = {
        "schema": "comotion.validation_replay_parallel.v1",
        "trace": trace,
        "variants": merged_variants,
        "speedups": speedups,
    }
    write_json(output, doc)
    return doc


def write_summary(rows: list[dict[str, Any]], output: pathlib.Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        fieldnames = [
            "num_robots",
            "obstacles",
            "motions",
            "valid",
            "invalid",
            "workers",
            "wall_seconds",
            *[f"{variant}_cpu_seconds" for variant in VARIANTS],
            *[f"{variant}_valid_cpu_seconds" for variant in VARIANTS],
            *[f"{variant}_invalid_cpu_seconds" for variant in VARIANTS],
            *[f"{variant}_timesteps_checked" for variant in VARIANTS],
            *[f"{variant}_timesteps_possible" for variant in VARIANTS],
            *[f"{variant}_timestep_coverage" for variant in VARIANTS],
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_case(args: argparse.Namespace, case: Case, cores: list[int]) -> dict[str, Any]:
    variants = args.variants.split(",") if args.variants else VARIANTS
    exhaustive = args.validation_mode == "exhaustive"
    work_dir = (
        args.results_dir
        / "validation_timing_parallel_batches"
        / args.validation_mode
        / case.tag
    )
    trace_dir = work_dir / "traces"
    metrics_dir = work_dir / "metrics"
    log_dir = work_dir / "logs"
    merged_path = args.results_dir / (
        f"validation_timing_panda_cage_{case.tag}_motion_only_"
        f"{args.validation_mode}_parallel_metrics.json"
    )

    metadata, records = load_case_records(
        args.trace_results_dir, case, args.trace_inputs
    )
    if args.record_limit > 0:
        records = records[: args.record_limit]
    batches = partition_records(records, args.workers)
    print(f"[{time.strftime('%H:%M:%S')}] {case.label}: {summarize_batches(batches)}", flush=True)

    trace_paths: list[pathlib.Path] = []
    metric_paths: list[pathlib.Path] = []
    for idx, batch in enumerate(batches):
        trace = trace_dir / f"batch_{idx:02d}.json"
        metrics = metrics_dir / f"batch_{idx:02d}_metrics.json"
        batch_doc = dict(metadata)
        batch_doc["source"] = f"{metadata['source']}_batch_{idx:02d}"
        batch_doc["records"] = batch
        batch_doc["parallel_batch_index"] = idx
        batch_doc["parallel_batch_count"] = len(batches)
        write_json(trace, batch_doc)
        trace_paths.append(trace)
        metric_paths.append(metrics)

    start = time.monotonic()
    processes: list[tuple[int, subprocess.Popen[str], pathlib.Path]] = []
    for idx, (trace, metrics) in enumerate(zip(trace_paths, metric_paths)):
        if not args.force and metrics_complete(
            metrics,
            variants,
            exhaustive,
            len(batches[idx]),
            args.fcl_urdf,
        ):
            print(f"[{time.strftime('%H:%M:%S')}] {case.label}: skip complete batch {idx:02d}", flush=True)
            continue
        core = cores[idx % len(cores)]
        log = log_dir / f"batch_{idx:02d}.log"
        process = run_batch(
            args.executable,
            trace,
            metrics,
            log,
            case,
            variants,
            core,
            args.progress_interval,
            exhaustive,
            args.fcl_urdf,
        )
        processes.append((idx, process, log))
        print(f"[{time.strftime('%H:%M:%S')}] {case.label}: started batch {idx:02d} on core {core}", flush=True)

    failed = False
    for idx, process, log in processes:
        code = process.wait()
        close_process_log(process)
        if code != 0:
            failed = True
            print(f"[{time.strftime('%H:%M:%S')}] {case.label}: batch {idx:02d} failed with {code}; log={log}", flush=True)
        else:
            print(f"[{time.strftime('%H:%M:%S')}] {case.label}: finished batch {idx:02d}", flush=True)
    if failed:
        raise RuntimeError(f"{case.label} had failed batch workers")

    for metrics, batch in zip(metric_paths, batches):
        if not metrics_complete(
            metrics,
            variants,
            exhaustive,
            len(batch),
            args.fcl_urdf,
        ):
            raise RuntimeError(f"incomplete metrics: {metrics}")

    wall_seconds = time.monotonic() - start
    merged = merge_metrics(case, metric_paths, variants, merged_path, args.workers, wall_seconds)
    first = merged["variants"][0]
    row = {
        "num_robots": case.robots,
        "obstacles": str(case.obstacles).lower(),
        "motions": first["count"],
        "valid": first["valid"],
        "invalid": first["invalid"],
        "workers": args.workers,
        "wall_seconds": wall_seconds,
    }
    for variant in VARIANTS:
        match = next((item for item in merged["variants"] if item["name"] == variant), None)
        row[f"{variant}_cpu_seconds"] = "" if match is None else match["total_validation_time_seconds"]
        row[f"{variant}_valid_cpu_seconds"] = (
            "" if match is None else match["valid_validation_time_seconds"]
        )
        row[f"{variant}_invalid_cpu_seconds"] = (
            "" if match is None else match["invalid_validation_time_seconds"]
        )
        work = {} if match is None else match.get("validation_work", {})
        row[f"{variant}_timesteps_checked"] = work.get(
            "motion_timesteps_checked", ""
        )
        row[f"{variant}_timesteps_possible"] = work.get(
            "motion_timesteps_possible", ""
        )
        row[f"{variant}_timestep_coverage"] = work.get(
            "motion_timestep_coverage", ""
        )

    print(f"[{time.strftime('%H:%M:%S')}] {case.label}: merged {merged_path}", flush=True)
    return row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--cases", default="all", help="all or comma-separated n4,n4_empty,n8,...")
    parser.add_argument("--results-dir", type=pathlib.Path, default=pathlib.Path("benchmarks/results"))
    parser.add_argument(
        "--trace-results-dir",
        type=pathlib.Path,
        help="Directory containing source trace JSON; defaults to --results-dir",
    )
    parser.add_argument(
        "--trace-inputs",
        help="Comma-separated trace JSON paths; valid only with one --cases entry",
    )
    parser.add_argument("--executable", type=pathlib.Path, default=pathlib.Path("./build/apps/validation_timing"))
    parser.add_argument("--variants", default=",".join(VARIANTS))
    parser.add_argument(
        "--fcl-urdf",
        default="panda/panda.urdf",
        help="Resource-relative mesh URDF passed to FCL workers",
    )
    parser.add_argument("--progress-interval", type=int, default=0)
    parser.add_argument(
        "--validation-mode",
        choices=("early", "exhaustive"),
        default="exhaustive",
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--record-limit",
        type=int,
        default=0,
        help="Limit motions per case for smoke tests; zero uses all records",
    )
    args = parser.parse_args()

    args.results_dir = args.results_dir.resolve()
    args.trace_results_dir = (
        args.results_dir
        if args.trace_results_dir is None
        else args.trace_results_dir.resolve()
    )
    args.trace_inputs = (
        None
        if not args.trace_inputs
        else [pathlib.Path(path).resolve() for path in args.trace_inputs.split(",")]
    )
    args.executable = args.executable.resolve()
    if not args.executable.exists():
        raise FileNotFoundError(args.executable)
    if args.workers < 1:
        raise ValueError("--workers must be positive")
    if args.record_limit < 0:
        raise ValueError("--record-limit must be nonnegative")
    if not args.fcl_urdf:
        raise ValueError("--fcl-urdf must not be empty")

    cores = available_cores(args.workers)
    print(f"Using workers={args.workers} pinned_cores={','.join(map(str, cores))}", flush=True)

    cases = parse_cases(args.cases)
    if args.trace_inputs is not None and len(cases) != 1:
        raise ValueError("--trace-inputs requires exactly one case")
    rows = []
    for case in cases:
        rows.append(run_case(args, case, cores))

    summary = args.results_dir / (
        "validation_timing_panda_cage_motion_only_"
        f"{args.validation_mode}_parallel_summary.csv"
    )
    write_summary(rows, summary)
    print(f"summary_csv: {summary}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
