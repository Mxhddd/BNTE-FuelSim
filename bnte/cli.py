"""Command line interface for BNTE simulations."""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

from .mission.runner import run_simulation


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="BNTE digital twin runner")
    sub = parser.add_subparsers(dest="cmd", required=True)

    run_parser = sub.add_parser("run", help="Run a BNTE case")
    run_parser.add_argument("--cfg", required=True, help="Path to YAML configuration")
    run_parser.add_argument(
        "--outdir",
        type=str,
        default=None,
        help="Optional output directory base (defaults to outputs/<date>/)",
    )

    args = parser.parse_args(argv)
    if args.cmd == "run":
        run_simulation(Path(args.cfg), Path(args.outdir) if args.outdir else None)


if __name__ == "__main__":  # pragma: no cover
    main()
