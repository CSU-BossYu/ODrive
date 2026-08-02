#!/usr/bin/env python3
"""Run the non-destructive architecture acceptance suite.

The optional hardware check uses only USB diagnostics and a read-only Test
Session. It never requests an operation, calibration, ARM-capable session, or
motor setpoint.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]


def run(name: str, command: list[str], cwd: Path = ROOT) -> dict[str, object]:
    completed = subprocess.run(command, cwd=cwd, check=False)
    if completed.returncode:
        raise RuntimeError(f"{name} failed with exit code {completed.returncode}")
    return {"name": name, "ok": True}


def request_json(base: str, path: str, body: dict[str, object] | None = None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = Request(base.rstrip("/") + path, data=data,
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=10) as response:
        return json.loads(response.read())


def hardware_readonly(base: str, port: str,
                      expected_build_id: str | None) -> dict[str, object]:
    status = request_json(base, "/api/usb/status")
    was_connected = bool(status.get("connected"))
    if not was_connected:
        request_json(base, "/api/usb/connect", {"port": port, "fake": False})
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        status = request_json(base, "/api/usb/status")
        if status.get("handshake_complete"):
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("USB handshake did not complete")
    build_id = str(status.get("build_id") or "")
    if expected_build_id and not build_id.startswith(expected_build_id):
        raise RuntimeError(
            f"firmware build mismatch: expected {expected_build_id}, got {build_id}")
    parser = status.get("parser") or {}
    for field in ("length_errors", "crc_errors", "unknown_frames",
                  "dropped_frames"):
        if int(parser.get(field, 0)) != 0:
            raise RuntimeError(f"USB parser {field} is non-zero")

    request_json(base, "/api/usb/session/begin", {
        "motion": False, "hardware_estop_confirmed": False,
        "lease_ms": 3000, "velocity_limit": 0.3,
        "current_limit": 0.5, "torque_limit": 0.1})
    time.sleep(0.2)
    arm_request = request_json(base, "/api/usb/session/command", {
        "command_type": 2, "operation": 0, "arg0": 0})["request_id"]
    time.sleep(0.2)
    replay = request_json(base, "/api/usb/export/replay")
    results = [event for event in replay["capture"]["events"]
               if event.get("type") == "command_result" and
               event.get("request_id") == arm_request]
    if not results or results[-1].get("status") != 1:
        raise RuntimeError("read-only Test Session did not reject ARM")
    request_json(base, "/api/usb/session/end", {})
    time.sleep(0.2)
    if not was_connected:
        request_json(base, "/api/usb/disconnect", {})
    return {"name": "hardware-readonly", "ok": True,
            "build_id": build_id, "arm_reject_reason": results[-1]["reason"]}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware-readonly", action="store_true")
    parser.add_argument("--backend-url", default="http://127.0.0.1:8000")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--expected-build-id")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    checks = [
        run("phase8-static", [sys.executable,
            "tools/check_phase8_static_gate.py"]),
        run("firmware-python", [sys.executable, "-m", "pytest",
            "Firmware/Tests", "-q"]),
        run("host-build", ["cmake", "--build", "--preset", "host-debug",
            "-j", "4"], ROOT / "Firmware"),
        run("host-ctest", ["ctest", "--preset", "host-debug",
            "--output-on-failure"], ROOT / "Firmware"),
        run("firmware-release", ["cmake", "--build", "--preset",
            "firmware-release", "-j", "4"], ROOT / "Firmware"),
    ]
    if args.hardware_readonly:
        checks.append(hardware_readonly(
            args.backend_url, args.port, args.expected_build_id))
    report = {"schema_version": 1, "checks": checks, "ok": True}
    encoded = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.report:
        args.report.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (HTTPError, OSError, RuntimeError) as exc:
        print(f"qualification failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
