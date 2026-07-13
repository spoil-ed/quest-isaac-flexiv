#!/usr/bin/env python3
"""Convert Flexiv Unitree JSON into a LeRobot-style dataset."""

from __future__ import annotations

import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    repo_root = Path(__file__).resolve().parents[1]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from flexiv_data_collection.converter import main as converter_main

    return converter_main(argv if argv is not None else sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
