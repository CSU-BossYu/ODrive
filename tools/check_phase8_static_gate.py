#!/usr/bin/env python3
"""Guard the first Phase 8 Python/HIL architecture boundary."""

from __future__ import annotations

import ast
import sys
from pathlib import Path
import json


ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "Firmware/Tests"
LEGACY = TESTS / "hex_4342_mt6826s"


def main() -> int:
    failures: list[str] = []
    required = (
        TESTS / "odrive_hil/__init__.py",
        TESTS / "odrive_hil/can_simple.py",
        TESTS / "odrive_hil/session.py",
        TESTS / "odrive_hil/can_product_schema_generated.py",
        TESTS / "odrive_hil/replay.py",
        TESTS / "odrive_hil/native_scenario.py",
        TESTS / "pytest.ini",
    )
    for path in required:
        if not path.is_file():
            failures.append(f"missing shared HIL package file: {path}")
    if not (TESTS / "test_layers.json").is_file():
        failures.append("missing machine-readable L0-L4 test inventory")
    pytest_config = (TESTS / "pytest.ini").read_text(encoding="utf-8")
    if "norecursedirs = hex_4342_mt6826s" not in pytest_config:
        failures.append("offline pytest can still collect energized HIL scripts")

    if (LEGACY / "common.py").exists():
        failures.append("deleted common.py compatibility surface returned")
    bootstrap = LEGACY / "hil_bootstrap.py"
    if not bootstrap.is_file():
        failures.append("direct-run HIL import bootstrap is missing")
    for path in LEGACY.glob("*.py"):
        body = path.read_text(encoding="utf-8")
        if "from common" in body or "import common" in body:
            failures.append(f"{path.name} imports removed common.py")
        if "from odrive_hil.can_simple import" in body and path != bootstrap:
            if ("ensure_test_support()" not in body and
                    "_TESTS_ROOT" not in body):
                failures.append(
                    f"{path.name} cannot import odrive_hil when run directly")

    direct_bus_allowlist = set()
    actual_direct_bus = set()
    for path in LEGACY.glob("*.py"):
        body = path.read_text(encoding="utf-8")
        if "can.Bus(" in body:
            actual_direct_bus.add(path.name)
    if actual_direct_bus != direct_bus_allowlist:
        failures.append(
            f"direct python-can inventory changed: {sorted(actual_direct_bus)}")

    for name in ("test_position_steps.py", "test_velocity_steps.py"):
        body = (LEGACY / name).read_text(encoding="utf-8")
        for fragment in ("HilSession(", "HilSafetyLimits(",
                         "--hardware-estop-ready", "session.arm_closed_loop()",
                         "session.close()"):
            if fragment not in body:
                failures.append(f"{name} missing safe HIL lifecycle: {fragment}")
        if "CURRENT_LIMIT_A = 3.0" in body:
            failures.append(f"{name} retains unsafe 3 A test default")
        if "send(bus, nid, CMD_SET_REQUESTED_STATE" in body:
            failures.append(f"{name} bypasses ACKed product management")

    migrated = (LEGACY / "repro_ui_velocity_stream.py").read_text(
        encoding="utf-8")
    for fragment in ("HilSession(", "HilSafetyLimits(",
                     "--hardware-estop-ready", "session.close()"):
        if fragment not in migrated:
            failures.append(f"migrated HIL script missing {fragment}")
    if "from common import" in migrated or "open_bus(" in migrated:
        failures.append("migrated HIL script fell back to legacy transport")
    if "set_requested_state(" in migrated:
        failures.append("migrated HIL script bypasses ACKed product management")

    usb_schema = json.loads((ROOT / "docs/usb_debug_protocol_schema.json").read_text(
        encoding="utf-8"))
    if (usb_schema.get("protocol_release") != "4.5" or
            usb_schema.get("schema_version") != 4):
        failures.append("USB Test Session is not release 4.5/schema 4")
    messages = usb_schema.get("message_types", {})
    for name in ("TEST_SESSION_BEGIN", "TEST_SESSION_STATUS",
                 "TEST_SESSION_KEEPALIVE", "TEST_SESSION_STOP",
                 "TEST_SESSION_END"):
        if name not in messages:
            failures.append(f"USB Test Session message missing: {name}")
    usb_transport = (ROOT / "Firmware/USB/usb_debug_transport.cpp").read_text(
        encoding="utf-8")
    for fragment in ("begin_test_session", "expire_test_session",
                     "force_session_disarm", "kReasonSessionRequired",
                     "test_limits_apply_", "test_session_stop_confirmed_"):
        if fragment not in usb_transport:
            failures.append(f"firmware USB Test Session missing: {fragment}")
    backend_transport = (ROOT / "foc_ui/backend/odrive_usb/transport.py").read_text(
        encoding="utf-8")
    for fragment in ("begin_test_session", "keepalive_test_session",
                     "stop_test_session", "end_test_session",
                     "_maintain_test_session"):
        if fragment not in backend_transport:
            failures.append(f"backend USB Test Session missing: {fragment}")

    replay_schema = json.loads((ROOT / "docs/usb_trace_replay_schema.json").read_text(
        encoding="utf-8"))
    if (replay_schema.get("format") != "odrive-usb-replay" or
            replay_schema.get("schema_version") != 1):
        failures.append("USB replay schema is missing or unversioned")
    replay = (ROOT / "foc_ui/backend/odrive_usb/replay.py").read_text(
        encoding="utf-8")
    for fragment in ("drop_control_sequences", "corrupt_frame_crc",
                     "mark_stale_feedback", "_json_safe"):
        if fragment not in replay:
            failures.append(f"USB replay mutation missing: {fragment}")
    backend_app = (ROOT / "foc_ui/backend/odrive_usb/app.py").read_text(
        encoding="utf-8")
    if '"/api/usb/export/replay"' not in backend_app:
        failures.append("backend USB replay export endpoint missing")

    scenario_header = ROOT / "Firmware/MotorControl/native_scenario_c_api.h"
    scenario_source = ROOT / "Firmware/MotorControl/native_scenario_c_api.cpp"
    for path in (scenario_header, scenario_source,
                 TESTS / "test_native_scenario_c_api.cpp"):
        if not path.is_file():
            failures.append(f"native scenario C ABI file missing: {path}")
    scenario_text = "\n".join(
        path.read_text(encoding="utf-8") for path in
        (scenario_header, scenario_source) if path.is_file())
    for fragment in ("ODRIVE_SCENARIO_ABI_VERSION",
                     "odrive_scenario_advance",
                     "odrive_scenario_inject_event",
                     "odrive_scenario_read_first_fault",
                     "static_assert(sizeof(ODriveScenarioEvent)"):
        if fragment not in scenario_text:
            failures.append(f"native scenario ABI contract missing: {fragment}")
    native_cmake = (TESTS / "CMakeLists.txt").read_text(encoding="utf-8")
    if "add_library(odrive_scenario SHARED" not in native_cmake:
        failures.append("native scenario shared library target missing")

    platform_header = ROOT / "Firmware/MotorControl/platform_ports.hpp"
    if not platform_header.is_file():
        failures.append("injectable platform port contract is missing")
    platform_consumers = "\n".join(
        (ROOT / path).read_text(encoding="utf-8") for path in (
            "Firmware/MotorControl/encoder.cpp",
            "Firmware/MotorControl/motor.cpp",
            "Firmware/MotorControl/controller.cpp"))
    for fragment in ("sensor_source_->sample", "power_stage_->apply",
                     "power_stage_->force_disarm",
                     "odrive::platform::cycle_count()"):
        if fragment not in platform_consumers:
            failures.append(f"platform dependency is not consumed: {fragment}")
    if not (ROOT / ".github/workflows/native-sanitizers.yml").is_file():
        failures.append("Clang sanitizer CI workflow is missing")
    if not (ROOT / "tools/run_architecture_qualification.py").is_file():
        failures.append("non-destructive architecture qualification runner missing")

    # Syntax-check every Python test utility without importing hardware modules.
    for path in TESTS.rglob("*.py"):
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as exc:
            failures.append(f"Python syntax error in {path}: {exc}")

    if failures:
        for failure in failures:
            print(f"phase8 static gate: FAIL: {failure}")
        return 1
    print("phase8 static gate passed: shared codecs, fail-closed HIL lifecycle, "
          "versioned replay, and removed compatibility imports are bounded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
