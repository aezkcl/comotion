#!/usr/bin/env python3
"""Generate benchmark plots from existing CSV outputs."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    write_anytime_panel_plots,
    write_success_plots,
)


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise RuntimeError(f"CSV file does not exist: {path}")
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Regenerate cumulative-success or anytime plots from benchmark "
            "results.csv and solution_events.csv files."
        )
    )
    parser.add_argument(
        "results_csv",
        type=Path,
        help="Path to a benchmark results.csv file.",
    )
    parser.add_argument(
        "--solution-events-csv",
        type=Path,
        help=(
            "Path to solution_events.csv. Defaults to a sibling of results.csv "
            "when --plot-kind anytime is used."
        ),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Directory for plots. Defaults to the directory containing results.csv.",
    )
    parser.add_argument(
        "--plot-kind",
        choices=("success", "anytime"),
        default="success",
        help="Plot type to generate.",
    )
    parser.add_argument(
        "--plot-backends",
        action="store_true",
        help=(
            "Draw separate curves for each collision backend using common "
            "algorithm colors and backend line styles."
        ),
    )
    return parser.parse_args()


def main() -> int:
    try:
        args = parse_args()
        output_root = args.output_root or args.results_csv.parent
        rows = read_csv(args.results_csv)
        if args.plot_kind == "success":
            plots = write_success_plots(
                rows,
                output_root,
                plot_backends=args.plot_backends,
            )
        else:
            events_path = args.solution_events_csv or (
                args.results_csv.parent / "solution_events.csv"
            )
            event_rows = read_csv(events_path)
            plots = write_anytime_panel_plots(
                rows,
                event_rows,
                output_root,
                plot_backends=args.plot_backends,
            )

        for plot in plots:
            print(f"plot: {plot}")
        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
