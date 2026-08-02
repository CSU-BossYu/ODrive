#!/usr/bin/env python3
"""Static architecture and bus-load gate for Phase 7 product CAN."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import generate_can_product_schema as can_generator


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    failures: list[str] = []
    schema = json.loads((ROOT / "docs/can_product_protocol_schema.json").read_text(
        encoding="utf-8"))
    messages = schema.get("messages", [])
    ids = [int(item.get("id", -1)) for item in messages]
    names = [str(item.get("name", "")) for item in messages]
    if len(ids) != len(set(ids)) or any(not 0 <= value <= 31 for value in ids):
        failures.append("CAN schema IDs are not unique five-bit values")
    for required in ("COMMAND_ACK", "MANAGEMENT_COMMAND", "PRODUCT_STATUS"):
        if required not in names:
            failures.append(f"CAN schema missing product message {required}")
    if schema.get("protocol_version_u32") != 0x10C:
        failures.append("CAN protocol is not release 1.12")

    profile = schema.get("load_profiles", {}).get("typical", {})
    # Conservative Classic CAN upper bound including inter-frame spacing and
    # worst-case stuffing. All profile messages carry eight data bytes.
    frames_per_second = sum(int(value) for value in profile.values())
    utilization = frames_per_second * 135 * 100 / int(schema.get("bitrate", 1))
    maximum = float(schema.get("load_profiles", {}).get(
        "maximum_allowed_percent", 0.0))
    if utilization > maximum:
        failures.append(
            f"typical product CAN load {utilization:.2f}% exceeds {maximum:.2f}%")

    generated = (
        ROOT / "Firmware/communication/can/can_product_schema_generated.hpp",
        ROOT / "foc_ui/backend/odrive_can/can_product_schema_generated.py",
        ROOT / "foc_ui/frontend/src/can_product_schema_generated.ts",
        ROOT / "Firmware/Tests/odrive_hil/can_product_schema_generated.py",
    )
    for path in generated:
        if not path.is_file():
            failures.append(f"missing generated CAN contract: {path}")
    expected_generated = (
        can_generator.cpp(schema),
        can_generator.python(schema),
        can_generator.typescript(schema),
        can_generator.python(schema),
    )
    for path, expected in zip(generated, expected_generated):
        if path.is_file() and path.read_text(encoding="utf-8") != expected:
            failures.append(f"generated CAN contract is stale: {path}")

    header = (ROOT / "Firmware/communication/can/can_simple.hpp").read_text(
        encoding="utf-8")
    source = (ROOT / "Firmware/communication/can/can_simple.cpp").read_text(
        encoding="utf-8")
    axis_header = (ROOT / "Firmware/MotorControl/axis.hpp").read_text(
        encoding="utf-8")
    axis_source = (ROOT / "Firmware/MotorControl/axis.cpp").read_text(
        encoding="utf-8")
    supervisor = (ROOT / "Firmware/MotorControl/safety_supervisor.cpp").read_text(
        encoding="utf-8")
    realtime_header = (ROOT / "Firmware/MotorControl/realtime_event_ring.hpp").read_text(
        encoding="utf-8")
    timeout_source = (ROOT / "Firmware/MotorControl/control_timeout.cpp").read_text(
        encoding="utf-8")
    supervisor_tests = (ROOT / "Firmware/Tests/test_safety_supervisor.cpp").read_text(
        encoding="utf-8")
    backend = (ROOT / "foc_ui/backend/odrive_can/service.py").read_text(
        encoding="utf-8")
    cmake = (ROOT / "Firmware/CMakeLists.txt").read_text(encoding="utf-8")
    dbc_generator = (ROOT / "tools/create_can_dbc.py").read_text(
        encoding="utf-8")
    dbc = (ROOT / "tools/odrive-cansimple.dbc").read_text(encoding="utf-8")
    protocol_doc = (ROOT / "Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md").read_text(
        encoding="utf-8")
    for fragment in ("can_product_schema_generated.hpp", "MSG_COMMAND_ACK",
                     "MSG_MANAGEMENT_COMMAND", "MSG_PRODUCT_STATUS"):
        if fragment not in header:
            failures.append(f"CANSimple missing generated product boundary: {fragment}")
    for fragment in ("handle_management_command", "send_command_ack",
                     "send_product_status", "kCanManagementNamespace"):
        if fragment not in source:
            failures.append(f"CANSimple missing product behavior: {fragment}")
    if re.search(r"MSG_SET_TRAJ_VEL_LIMIT\s*(?:,|=\s*0x10)", header):
        failures.append("implicit/off-by-one CAN enum gap has returned")
    if "kCommandResultSinkCapacity = 2u" not in axis_header or \
            "command_result_sinks_" not in axis_header:
        failures.append("Axis CommandResult fan-out is missing")
    for fragment, body in (
            ("RealtimeEventType::PREPARE_REJECTED", axis_source),
            ("PREPARE_REJECTED = 4", realtime_header),
            ("process_prepare_rejected_event", supervisor),
            ("state_ == SafetyState::SAFE_OFF ||", supervisor),
            ("prepare rejection returns safe-off without latching a fault",
             supervisor_tests),
            ("disarm is idempotent in already-safe states", supervisor_tests)):
        if fragment not in body:
            failures.append(f"hardware acceptance contract missing: {fragment}")
    if timeout_source.count("(st.flags & FLAG_ENABLED) == 0u") < 2:
        failures.append("idle state still evaluates command/heartbeat timeouts")
    arm_match = re.search(
        r"bool Axis::arm_closed_loop_control\(\).*?"
        r"bool Axis::start_closed_loop_control", axis_source, re.DOTALL)
    stop_match = re.search(
        r"bool Axis::stop_closed_loop_control\(\).*?"
        r"bool Axis::run_closed_loop_control_loop", axis_source, re.DOTALL)
    if arm_match is None or "ControlTimeout::feed_command" not in arm_match.group(0):
        failures.append("armed interval lacks a fresh command-watchdog deadline")
    if stop_match is None or "ControlTimeout::clear(*this)" not in stop_match.group(0):
        failures.append("closed-loop stop retains timeout state")
    for fragment in ("management_command", "decode_command_ack",
                     "CmdId.MANAGEMENT_COMMAND"):
        if fragment not in backend:
            failures.append(f"Python backend missing ACKed CAN management: {fragment}")
    poll_match = re.search(
        r"async def _poll_loop\(self\).*?async def _send_request",
        backend, re.DOTALL)
    if poll_match is None or any(fragment in poll_match.group(0) for fragment in (
            "GET_ENCODER_COUNT", "_request_error_detail",
            "_request_system_error_detail")):
        failures.append("normal backend polling still carries CAN diagnostics")
    periodic_match = re.search(
        r"std::array<periodic,\s*4>\s+periodics.*?\}\};",
        source, re.DOTALL)
    if periodic_match is None or any(fragment in periodic_match.group(0) for fragment in (
            "get_motor_error_callback", "get_encoder_error_callback",
            "get_controller_error_callback", "get_encoder_count_callback")):
        failures.append("firmware still periodically broadcasts CAN diagnostics")
    if re.search(r"\b(?:SCOPE_(?:ARM|READ|STATUS)|CRASH_REPORT)\b|crash_recorder",
                 source, re.IGNORECASE):
        failures.append("CAN transport contains USB diagnostic/Crash functionality")
    for fragment in ("generate_can_product_schema", "phase7_static_gate",
                     "0x0000010C", "--can-schema"):
        if fragment not in cmake:
            failures.append(f"CMake missing Phase 7 contract: {fragment}")
    for fragment in ("Command_Ack", "Management_Command", "Product_Status",
                     "version='1.12'"):
        if fragment not in dbc_generator:
            failures.append(f"DBC generator missing Phase 7 contract: {fragment}")
    if "Start_Anticogging" in dbc_generator or "Reserved_005" in dbc_generator:
        failures.append("DBC generator retains reassigned Phase 7 IDs")
    for fragment in ('VERSION "1.12"', "BO_ 5 Axis0_Command_Ack: 8",
                     "BO_ 8 Axis0_Management_Command: 8",
                     "BO_ 16 Axis0_Product_Status: 8"):
        if fragment not in dbc:
            failures.append(f"generated DBC is stale: {fragment}")
    if "Start_Anticogging" in dbc or "Reserved_005" in dbc:
        failures.append("generated DBC retains reassigned Phase 7 IDs")
    dbc_ids = {int(value) for value in re.findall(
        r"^BO_\s+(\d+)\s+Axis0_", dbc, re.MULTILINE)}
    if dbc_ids != set(ids):
        failures.append("generated DBC ID coverage differs from product schema")
    for fragment in ("0x0000010C", "Product_Status", "Management_Command"):
        if fragment not in protocol_doc:
            failures.append(f"production CAN document is stale: {fragment}")

    dispatcher_match = re.search(
        r"bool CANSimple::legacy_extended_command_callback.*?"
        r"switch \(sub_cmd\) \{(.*?)\n\s*\}",
        source, re.DOTALL)
    allowed_legacy = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                      0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}
    if dispatcher_match is None:
        failures.append("legacy Extended dispatcher is not isolated")
    else:
        actual = {int(value, 16) for value in re.findall(
            r"case\s+0x([0-9A-Fa-f]+)", dispatcher_match.group(1))}
        if actual != allowed_legacy:
            failures.append("legacy Extended dispatcher surface changed")

    if failures:
        for failure in failures:
            print(f"phase7 static gate: FAIL: {failure}")
        return 1
    print(f"phase7 static gate passed: generated product CAN, ACKed management, "
          f"legacy diagnostics isolated, typical load={utilization:.2f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
