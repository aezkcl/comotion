#!/usr/bin/env python3
"""Run exhaustive validation for one indexed chunk of an exported corpus."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import subprocess
import sys
from typing import Any


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
PARALLEL_RUNNER = pathlib.Path(__file__).with_name(
    "run_validation_timing_exhaustive_parallel.py"
)
CASES = (
    ("n4_obstacles", "n4"),
    ("n4_empty", "n4_empty"),
    ("n8_obstacles", "n8"),
    ("n8_empty", "n8_empty"),
    ("n16_obstacles", "n16"),
    ("n16_empty", "n16_empty"),
)
VARIANTS = (
    "fcl",
    "sphere",
    "vamp_combined_rake",
    "vamp_combined_linear",
    "vamp_hierarchical_rake",
    "vamp_hierarchical_linear",
)


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: pathlib.Path) -> dict[str, Any]:
    with path.open() as stream:
        return json.load(stream)


def metrics_name(runner_case: str) -> str:
    return (
        f"validation_timing_panda_cage_{runner_case}_motion_only_"
        "exhaustive_parallel_metrics.json"
    )


def exhaustive_complete(path: pathlib.Path, workers: int) -> bool:
    if not path.is_file():
        return False
    try:
        document = load_json(path)
    except (OSError, json.JSONDecodeError):
        return False
    trace = document.get("trace", {})
    variants = document.get("variants", [])
    work_pairs = (
        ("motion_timesteps_checked", "motion_timesteps_possible"),
        ("robot_state_checks_completed", "robot_state_checks_possible"),
        ("robot_pair_checks_completed", "robot_pair_checks_possible"),
    )
    return (
        trace.get("exhaustive_motion_validation") is True
        and trace.get("parallel_workers") == workers
        and trace.get("record_count") == 100
        and [variant.get("name") for variant in variants] == list(VARIANTS)
        and all(
            variant.get("count") == 100
            and all(
                key_checked in variant.get("validation_work", {})
                and key_possible in variant.get("validation_work", {})
                and variant["validation_work"][key_checked]
                == variant["validation_work"][key_possible]
                for key_checked, key_possible in work_pairs
            )
            for variant in variants
        )
    )


def manifest_entries(
    corpus: pathlib.Path, chunk: int
) -> dict[str, dict[str, Any]]:
    manifest_path = corpus / "corpus_manifest.json"
    if not manifest_path.is_file():
        return {}
    manifest = load_json(manifest_path)
    if manifest.get("schema") != "comotion.validation_motion_corpus.v1":
        raise RuntimeError(f"unsupported manifest schema in {manifest_path}")
    return {
        entry["case"]: entry
        for entry in manifest.get("entries", [])
        if int(entry.get("chunk", -1)) == chunk
    }


def write_json(path: pathlib.Path, document: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("index", type=int, help="zero-based corpus chunk index")
    parser.add_argument(
        "--corpus-dir",
        type=pathlib.Path,
        required=True,
        help="exported corpus root containing corpus_manifest.json",
    )
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        required=True,
        help="root under which exhaustive results will be written",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="parallel replay workers; defaults to one for benchmark fidelity",
    )
    parser.add_argument(
        "--cases",
        help="optional comma-separated subset of corpus case names",
    )
    args = parser.parse_args()
    if args.index < 0:
        parser.error("index must be non-negative")
    if args.workers < 1:
        parser.error("--workers must be positive")

    corpus = args.corpus_dir.resolve()
    output = args.output_dir.resolve()
    selected = (
        set(args.cases.split(",")) if args.cases else {case for case, _ in CASES}
    )
    unknown = selected - {case for case, _ in CASES}
    if unknown:
        parser.error(f"unknown cases: {','.join(sorted(unknown))}")
    entries = manifest_entries(corpus, args.index)
    if not entries:
        raise RuntimeError(
            f"chunk {args.index:04d} is absent from {corpus / 'corpus_manifest.json'}"
        )

    completed_cases: list[dict[str, Any]] = []
    for case, runner_case in CASES:
        if case not in selected:
            continue
        relative = pathlib.Path(case) / "chunks" / f"{args.index:04d}" / "trace.json"
        trace = corpus / relative
        entry = entries.get(case)
        if entry is None or not trace.is_file():
            raise FileNotFoundError(f"missing manifest entry or trace for {relative}")
        observed_hash = sha256(trace)
        if observed_hash != entry.get("sha256"):
            raise RuntimeError(f"SHA-256 mismatch for {trace}")
        if int(entry.get("motion_count", -1)) != 100:
            raise RuntimeError(f"unexpected motion count for {trace}")

        case_output = output / case / "chunks" / f"{args.index:04d}"
        case_output.mkdir(parents=True, exist_ok=True)
        metrics = case_output / metrics_name(runner_case)
        if exhaustive_complete(metrics, args.workers):
            print(f"skip complete {case} chunk={args.index:04d}", flush=True)
        else:
            command = [
                sys.executable,
                str(PARALLEL_RUNNER),
                "--workers",
                str(args.workers),
                "--cases",
                runner_case,
                "--results-dir",
                str(case_output),
                "--trace-inputs",
                str(trace),
                "--variants",
                ",".join(VARIANTS),
                "--validation-mode",
                "exhaustive",
            ]
            print(f"start {case} chunk={args.index:04d}", flush=True)
            completed = subprocess.run(command, cwd=REPO_ROOT)
            if completed.returncode != 0:
                raise RuntimeError(f"runner failed for {case}")
            if not exhaustive_complete(metrics, args.workers):
                raise RuntimeError(f"exhaustive audit failed for {case}")
            print(f"done {case} chunk={args.index:04d}", flush=True)
        completed_cases.append(
            {
                "case": case,
                "trace": str(trace),
                "trace_sha256": observed_hash,
                "metrics": str(metrics),
            }
        )

    summary = {
        "schema": "comotion.validation_exhaustive_corpus_chunk.v1",
        "chunk": args.index,
        "workers": args.workers,
        "variants": list(VARIANTS),
        "cases": completed_cases,
    }
    summary_path = output / f"exhaustive_chunk_{args.index:04d}_summary.json"
    output.mkdir(parents=True, exist_ok=True)
    write_json(summary_path, summary)
    print(f"summary: {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
