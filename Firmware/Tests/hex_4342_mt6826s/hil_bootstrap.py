"""Make the shared test-support package importable for direct script runs."""

from pathlib import Path
import sys


def ensure_test_support() -> None:
    tests_root = Path(__file__).resolve().parents[1]
    path = str(tests_root)
    if path not in sys.path:
        sys.path.insert(0, path)
