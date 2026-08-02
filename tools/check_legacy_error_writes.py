#!/usr/bin/env python3
"""Reject unclassified writes to legacy component/axis error bitfields."""

from __future__ import annotations

import re
import sys
from pathlib import Path


PATTERN = re.compile(r"\berror_\s*\|=")
ALLOWED_MARKERS = ("LEGACY_ERROR_PROJECTION", "SYSTEM_ERROR_PROJECTION")


def main() -> int:
    firmware = Path(__file__).resolve().parents[1] / "Firmware"
    roots = (firmware / "MotorControl", firmware / "communication" / "can")
    violations: list[str] = []
    projections = 0
    for root in roots:
        for path in root.rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".h"}:
                continue
            for line_number, line in enumerate(
                    path.read_text(encoding="utf-8").splitlines(), start=1):
                if line.lstrip().startswith("//") or not PATTERN.search(line):
                    continue
                if any(marker in line for marker in ALLOWED_MARKERS):
                    projections += 1
                else:
                    violations.append(f"{path.relative_to(firmware)}:{line_number}: {line.strip()}")
    if violations:
        print("legacy error write gate failed", file=sys.stderr)
        print("Every direct write must be an audited compatibility projection:", file=sys.stderr)
        for violation in violations:
            print(f"  {violation}", file=sys.stderr)
        return 1
    print(f"legacy error write gate passed: {projections} audited projections")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
