#!/usr/bin/env python3
"""Static boundary checks for Phase 5 Scope and USB diagnostic ownership."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    failures: list[str] = []
    schema_path = ROOT / "docs/scope_channel_schema.json"
    protocol_path = ROOT / "docs/usb_debug_protocol_schema.json"
    try:
        scope_schema = json.loads(schema_path.read_text(encoding="utf-8"))
        protocol_schema = json.loads(protocol_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"phase5 static gate: FAIL: schema read failed: {exc}")
        return 1

    channels = scope_schema.get("channels", [])
    required_names = {
        "control_sequence", "timestamp_cycles", "state_epoch", "SafetyState",
        "Operation", "phase", "phase_velocity", "position", "velocity",
        "Id measured", "Iq measured", "Id setpoint", "Iq setpoint",
        "torque setpoint", "controller output",
    }
    actual_names = {channel.get("name") for channel in channels}
    for name in sorted(required_names - actual_names):
        failures.append(f"missing ScopeChannel: {name}")
    for field in ("id", "name", "wire_type", "unit", "display_min",
                  "display_max", "max_sample_rate_hz", "threshold_allowed",
                  "source"):
        if any(field not in channel for channel in channels):
            failures.append(f"ScopeChannel schema is missing field {field}")

    generated = [
        ROOT / "Firmware/MotorControl/scope_channels_generated.hpp",
        ROOT / "foc_ui/backend/odrive_usb/scope_channels_generated.py",
        ROOT / "foc_ui/frontend/src/scope_channels_generated.ts",
    ]
    for path in generated:
        if not path.is_file():
            failures.append(f"missing generated scope definition: {path}")
    cpp = (ROOT / "Firmware/MotorControl/scope_channels_generated.hpp").read_text(
        encoding="utf-8") if generated[0].is_file() else ""
    if "read_channel" not in cpp or "switch (id)" not in cpp:
        failures.append("generated C++ scope reader is not a channel whitelist")

    capture_files = [ROOT / "Firmware/MotorControl/scope_capture.hpp",
                     ROOT / "Firmware/MotorControl/scope_capture.cpp"]
    capture_source = "\n".join(
        path.read_text(encoding="utf-8") for path in capture_files if path.is_file())
    for pattern, label in {
        r"\bvolatile\s+bool\b": "volatile bool",
        r"\b(?:USB|usb|CAN|can)[A-Za-z0-9_]*\s*\(": "USB/CAN call",
        r"\b(?:printf|snprintf|sprintf|vsnprintf|malloc|calloc|realloc|free)\s*\(": "format/allocation call",
        r"\b(?:osDelay|osWait|taskYIELD|vTaskDelay|xQueue)\s*": "blocking/RTOS API",
        r"\bstd::string\b": "dynamic string",
    }.items():
        if re.search(pattern, capture_source):
            failures.append(f"forbidden {label} in Capture Engine")
    for fragment in ("CapturePlan", "build_capture_plan", "FixedTripleBuffer",
                     "SAMPLE_SEQUENCE_GAP", "trigger_fault_from_isr",
                     "pre_samples", "post_samples", "decimation",
                     "current_state != CaptureState::COMPLETE"):
        if fragment not in capture_source:
            failures.append(f"Capture Engine missing contract: {fragment}")

    if protocol_schema.get("protocol_release") not in ("4.4", "4.5"):
        failures.append("USB protocol no longer retains the Scope contract")
    if int(protocol_schema.get("schema_version", 0)) < 3:
        failures.append("USB protocol schema predates the Scope contract")
    messages = protocol_schema.get("message_types", {})
    for name in ("SCOPE_CONFIG", "SCOPE_ARM", "SCOPE_STATUS",
                 "SCOPE_DATA_BATCH", "SCOPE_STOP"):
        if name not in messages:
            failures.append(f"missing USB scope message: {name}")

    cmake = (ROOT / "Firmware/CMakeLists.txt").read_text(encoding="utf-8")
    for fragment in ("MotorControl/scope_capture.cpp", "generate_scope_schema",
                     "phase5_static_gate", "USB_DEBUG_PROTOCOL_VERSION"):
        if fragment not in cmake:
            failures.append(f"CMake missing Phase 5 fragment: {fragment}")

    transport = (ROOT / "Firmware/USB/usb_debug_transport.cpp").read_text(
        encoding="utf-8")
    for fragment in ("SCOPE_CONFIG", "SCOPE_DATA_BATCH", "service_scope_stream",
                     "kMaxPreSamples", "kMaxPostSamples"):
        if fragment not in transport:
            failures.append(f"USB transport missing scope fragment: {fragment}")
    protocol_source = (ROOT / "Firmware/USB/usb_debug_protocol.cpp").read_text(
        encoding="utf-8")
    if ("value <= static_cast<uint8_t>(MessageType::CRASH_ACK)" not in protocol_source and
            "value <= static_cast<uint8_t>(MessageType::TEST_SESSION_END)" not in protocol_source):
        failures.append("USB frame decoder does not accept all Phase 5 Scope messages")
    for fragment in ("pending_scope_batch_valid_", "if (!queue_frame("):
        if fragment not in transport:
            failures.append(f"USB scope TX retry contract missing: {fragment}")

    can_source = (ROOT / "Firmware/communication/can/can_simple.cpp").read_text(
        encoding="utf-8")
    if re.search(r"SCOPE|scope_capture|SCOPE_RECORD", can_source):
        failures.append("CAN transport contains a new Scope/debug item")

    usb_backend = ROOT / "foc_ui/backend/odrive_usb"
    if not (usb_backend / "transport.py").is_file() or not (usb_backend / "protocol.py").is_file():
        failures.append("independent Python USB transport package is missing")
    else:
        usb_source = "\n".join(path.read_text(encoding="utf-8") for path in usb_backend.glob("*.py"))
        if "odrive_can" in usb_source:
            failures.append("USB backend is coupled to odrive_can")
        for fragment in ("FakeSerialTransport", "SCOPE_DATA_BATCH",
                         "timestamp", "schema", "build_id"):
            if fragment not in usb_source:
                failures.append(f"USB backend missing fragment: {fragment}")
        protocol_source_py = (usb_backend / "protocol.py").read_text(
            encoding="utf-8")
        append_at = protocol_source_py.find("self.buffer.extend(data)")
        parse_at = protocol_source_py.find("while True:", append_at)
        bound_at = protocol_source_py.find("if len(self.buffer) > self.max_buffer", append_at)
        if append_at < 0 or parse_at < 0 or bound_at < parse_at:
            failures.append(
                "USB stream decoder must parse a large serial read before bounding residue")

    if failures:
        for failure in failures:
            print(f"phase5 static gate: FAIL: {failure}")
        return 1
    print("phase5 static gate passed: generated Scope schema, fixed Capture Engine, USB/CAN separation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
