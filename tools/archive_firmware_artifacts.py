#!/usr/bin/env python3
"""Archive exact-build symbols and schemas for Crash Recorder decoding."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--elf", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--archive-root", required=True)
    parser.add_argument("--schema", action="append", default=[])
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    build_id = str(manifest["build_id"])
    if len(build_id) < 16 or any(character not in "0123456789abcdef" for character in build_id.lower()):
        raise ValueError("manifest contains an invalid build_id")
    destination = Path(args.archive_root) / build_id
    destination.mkdir(parents=True, exist_ok=True)
    for source in (manifest_path, Path(args.elf), Path(args.map),
                   *(Path(value) for value in args.schema)):
        if not source.is_file():
            raise FileNotFoundError(source)
        shutil.copy2(source, destination / source.name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
