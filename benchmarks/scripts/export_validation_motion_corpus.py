#!/usr/bin/env python3
"""Export sampled-motion traces as a portable, integrity-checked corpus."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil
import tarfile
from typing import Any


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = (
    REPO_ROOT
    / "benchmarks"
    / "results"
    / "validation_sampled_motions_07_28"
    / "incremental_early_v1"
)
CASES = (
    "n4_obstacles",
    "n4_empty",
    "n8_obstacles",
    "n8_empty",
    "n16_obstacles",
    "n16_empty",
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


def motion_count(document: dict[str, Any]) -> int:
    return sum(
        record.get("type") == "composite_motion"
        for record in document.get("records", [])
    )


def complete_chunk_indices(source: pathlib.Path) -> list[int]:
    available: list[set[int]] = []
    for case in CASES:
        chunks = source / case / "chunks"
        indices = {
            int(path.parent.name)
            for path in chunks.glob("*/trace.json")
            if path.parent.name.isdigit()
        }
        available.append(indices)
    return sorted(set.intersection(*available)) if available else []


def write_json(path: pathlib.Path, document: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-dir",
        type=pathlib.Path,
        default=DEFAULT_SOURCE,
        help="incremental validation result root containing case/chunks trees",
    )
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        required=True,
        help="new or existing directory to receive the portable corpus",
    )
    parser.add_argument(
        "--chunks",
        help="comma-separated chunk indices; default exports every complete chunk",
    )
    parser.add_argument(
        "--archive",
        type=pathlib.Path,
        help="also create this .tar.gz archive from the exported corpus",
    )
    args = parser.parse_args()

    source = args.source_dir.resolve()
    output = args.output_dir.resolve()
    if args.chunks:
        chunks = sorted({int(value) for value in args.chunks.split(",")})
    else:
        chunks = complete_chunk_indices(source)
    if not chunks:
        raise RuntimeError("no complete chunks found")

    entries: list[dict[str, Any]] = []
    for chunk in chunks:
        for case in CASES:
            relative = pathlib.Path(case) / "chunks" / f"{chunk:04d}" / "trace.json"
            source_trace = source / relative
            if not source_trace.is_file():
                raise FileNotFoundError(source_trace)
            document = load_json(source_trace)
            count = motion_count(document)
            if count != 100:
                raise RuntimeError(
                    f"{source_trace} has {count} composite motions, expected 100"
                )
            destination = output / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_trace, destination)
            entries.append(
                {
                    "case": case,
                    "chunk": chunk,
                    "path": relative.as_posix(),
                    "sha256": sha256(destination),
                    "motion_count": count,
                    "num_robots": document.get("scenario", {}).get("num_robots"),
                    "empty_environment": document.get("empty_environment"),
                    "seed": document.get("seeds", [None])[0],
                }
            )

    manifest = {
        "schema": "comotion.validation_motion_corpus.v1",
        "cases": list(CASES),
        "chunks": chunks,
        "entries": entries,
    }
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "corpus_manifest.json", manifest)
    print(f"exported {len(entries)} traces to {output}", flush=True)

    if args.archive:
        archive = args.archive.resolve()
        archive.parent.mkdir(parents=True, exist_ok=True)
        with tarfile.open(archive, "w:gz") as stream:
            stream.add(output, arcname=output.name)
        print(f"archive: {archive}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
