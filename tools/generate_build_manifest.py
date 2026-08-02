#!/usr/bin/env python3
"""Generate a reproducibility manifest for a firmware or host build."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def git_output(repo: Path, *args: str) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), *args], text=True,
            stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def git_bytes(repo: Path, *args: str) -> bytes:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), *args], stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return b"unknown"


def cmake_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "on", "true", "yes"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--configuration", default="")
    parser.add_argument("--board", required=True)
    parser.add_argument("--protocol-version", required=True)
    parser.add_argument("--usb-protocol-version", default="")
    parser.add_argument("--compiler-id", default="unknown")
    parser.add_argument("--compiler-version", default="unknown")
    parser.add_argument("--compiler-path", default="unknown")
    parser.add_argument("--fault-schema")
    parser.add_argument("--protocol-schema")
    parser.add_argument("--scope-schema")
    parser.add_argument("--can-schema")
    parser.add_argument("--identity-header")
    parser.add_argument("--build-firmware", required=True)
    parser.add_argument("--build-native-tests", required=True)
    parser.add_argument("--compile-option", action="append", default=[])
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    revision = git_output(repo, "rev-parse", "HEAD")
    status = git_output(repo, "status", "--porcelain")
    dirty_hasher = hashlib.sha256()
    dirty_hasher.update(status.encode("utf-8"))
    dirty_hasher.update(git_bytes(repo, "diff", "--binary", "HEAD"))
    untracked = git_bytes(repo, "ls-files", "--others", "--exclude-standard", "-z")
    if untracked != b"unknown":
        for encoded_path in filter(None, untracked.split(b"\0")):
            relative_path = encoded_path.decode("utf-8", errors="surrogateescape")
            path = repo / relative_path
            if path.is_file():
                dirty_hasher.update(encoded_path)
                dirty_hasher.update(b"\0")
                dirty_hasher.update(path.read_bytes())
    dirty_sha256 = dirty_hasher.hexdigest()
    schema_sha256 = "unknown"
    if args.fault_schema:
        schema_path = Path(args.fault_schema)
        if schema_path.is_file():
            schema_sha256 = hashlib.sha256(schema_path.read_bytes()).hexdigest()
    protocol_schema_sha256 = "unknown"
    if args.protocol_schema:
        protocol_schema_path = Path(args.protocol_schema)
        if protocol_schema_path.is_file():
            protocol_schema_sha256 = hashlib.sha256(
                protocol_schema_path.read_bytes()).hexdigest()
    scope_schema_sha256 = "unknown"
    if args.scope_schema:
        scope_schema_path = Path(args.scope_schema)
        if scope_schema_path.is_file():
            scope_schema_sha256 = hashlib.sha256(
                scope_schema_path.read_bytes()).hexdigest()
    can_schema_sha256 = "unknown"
    if args.can_schema:
        can_schema_path = Path(args.can_schema)
        if can_schema_path.is_file():
            can_schema_sha256 = hashlib.sha256(
                can_schema_path.read_bytes()).hexdigest()
    identity = {
        "revision": revision,
        "dirty_sha256": dirty_sha256,
        "configuration": args.configuration,
        "board": args.board,
        "protocol_version": args.protocol_version,
        "usb_protocol_version": args.usb_protocol_version,
        "compiler_id": args.compiler_id,
        "compiler_version": args.compiler_version,
        "compile_options": args.compile_option,
        "fault_schema_sha256": schema_sha256,
        "protocol_schema_sha256": protocol_schema_sha256,
        "scope_schema_sha256": scope_schema_sha256,
        "can_schema_sha256": can_schema_sha256,
    }
    manifest = {
        "manifest_version": 2,
        "build_id": hashlib.sha256(
            json.dumps(identity, sort_keys=True).encode("utf-8")).hexdigest()[:20],
        "git_revision": revision,
        "dirty": status not in ("", "unknown"),
        "dirty_sha256": dirty_sha256,
        "configuration": args.configuration,
        "compile_options": [option for option in args.compile_option if option],
        "board": args.board,
        "protocol_version": args.protocol_version,
        "usb_protocol_version": args.usb_protocol_version,
        "fault_schema_sha256": schema_sha256,
        "protocol_schema_sha256": protocol_schema_sha256,
        "scope_schema_sha256": scope_schema_sha256,
        "can_schema_sha256": can_schema_sha256,
        "compiler": {
            "id": args.compiler_id,
            "version": args.compiler_version,
            "path": args.compiler_path,
        },
        "targets": {
            "firmware": cmake_bool(args.build_firmware),
            "native_tests": cmake_bool(args.build_native_tests),
        },
    }
    if args.identity_header:
        identity_path = Path(args.identity_header)
        identity_path.parent.mkdir(parents=True, exist_ok=True)
        build_id_bytes = bytes.fromhex(manifest["build_id"][:16])
        manifest_identity = bytes.fromhex(dirty_sha256[:32])
        build_id_initializer = ", ".join(
            f"0x{value:02x}u" for value in build_id_bytes)
        manifest_initializer = ", ".join(
            f"0x{value:02x}u" for value in manifest_identity)
        identity_path.write_text(
            "#ifndef ODRIVE_USB_DEBUG_IDENTITY_HPP\n"
            "#define ODRIVE_USB_DEBUG_IDENTITY_HPP\n\n"
            "#include <array>\n"
            "#include <cstdint>\n\n"
            "namespace odrive::usb::identity {\n"
            f"inline constexpr std::array<uint8_t, 8> kBuildId = "
            f"{{{build_id_initializer}}};\n"
            f"inline constexpr std::array<uint8_t, 16> kManifestIdentity = "
            f"{{{manifest_initializer}}};\n"
            f"inline constexpr char kBuildIdHex[] = \"{manifest['build_id']}\";\n"
            "}  // namespace odrive::usb::identity\n\n"
            "#endif\n",
            encoding="utf-8",
        )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
