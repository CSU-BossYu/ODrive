"""Exact-build Cortex crash symbolization without a debugger."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
from typing import Any


ROOT = Path(__file__).resolve().parents[3]


def _wire_identity(value: str, encoded_length: int) -> bytes | None:
    if len(value) != encoded_length * 2:
        return None
    try:
        decoded = bytes.fromhex(value)
    except ValueError:
        return None
    return decoded if len(decoded) == encoded_length else None


def _matching_artifact(build_id: str, manifest_identity: str) -> tuple[Path, dict[str, Any]] | None:
    build_bytes = _wire_identity(build_id, 8)
    manifest_bytes = _wire_identity(manifest_identity, 16)
    if build_bytes is None or manifest_bytes is None:
        return None
    for manifest_path in (ROOT / "Firmware" / "build").rglob("build_manifest.json"):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if (not str(manifest.get("build_id", "")).startswith(build_id) or
                    not str(manifest.get("dirty_sha256", "")).startswith(manifest_identity)):
                continue
            elf = manifest_path.parent / "ODriveFirmware.elf"
            image = elf.read_bytes()
            if build_bytes not in image or manifest_bytes not in image:
                continue
            return elf, manifest
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            continue
    return None


def _addr2line_tool(manifest: dict[str, Any]) -> str | None:
    compiler = Path(str(manifest.get("compiler", {}).get("path", "")))
    candidates = []
    if compiler.parent:
        candidates.extend((compiler.parent / "arm-none-eabi-addr2line.exe",
                           compiler.parent / "arm-none-eabi-addr2line"))
    discovered = shutil.which("arm-none-eabi-addr2line")
    if discovered:
        candidates.append(Path(discovered))
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return None


def _decode_pairs(output: str, labels: list[str]) -> dict[str, Any]:
    lines = output.splitlines()
    result: dict[str, Any] = {}
    for index, label in enumerate(labels):
        function_index = index * 2
        location_index = function_index + 1
        function = lines[function_index].strip() if function_index < len(lines) else "??"
        location = lines[location_index].strip() if location_index < len(lines) else "??:0"
        file_name, separator, line_text = location.rpartition(":")
        line = int(line_text) if separator and line_text.isdigit() else 0
        result[label] = {
            "function": function,
            "file": file_name if file_name and file_name != "??" else None,
            "line": line or None,
            "resolved": function != "??" or (file_name not in ("", "??") and line != 0),
        }
    return result


def symbolize_crash(crash: dict[str, Any]) -> dict[str, Any]:
    """Resolve PC/LR only against an ELF with the exact retained identity."""
    build_id = str(crash.get("record_build_id", ""))
    manifest_identity = str(crash.get("record_manifest_identity", ""))
    match = _matching_artifact(build_id, manifest_identity)
    if match is None:
        return {"reliable": False, "error": "no exact build/manifest ELF match"}
    elf, manifest = match
    tool = _addr2line_tool(manifest)
    if tool is None:
        return {"reliable": False, "elf": str(elf),
                "error": "arm-none-eabi-addr2line not found"}
    stacked = crash.get("stacked", {})
    addresses = [int(stacked.get("pc", 0)) & ~1,
                 int(stacked.get("lr", 0)) & ~1]
    try:
        completed = subprocess.run(
            [tool, "-e", str(elf), "-f", "-C", *(f"0x{value:08x}" for value in addresses)],
            check=True, capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError) as exc:
        return {"reliable": False, "elf": str(elf), "error": str(exc)}
    return {"reliable": True, "elf": str(elf),
            "locations": _decode_pairs(completed.stdout, ["pc", "lr"])}
