#!/usr/bin/env python3
"""Static boundary checks for the Phase 4 USB Debug/Test protocol."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_FILES = (
    ROOT / "Firmware/USB/usb_debug_protocol.hpp",
    ROOT / "Firmware/USB/usb_debug_protocol.cpp",
    ROOT / "Firmware/USB/usb_debug_transport.hpp",
    ROOT / "Firmware/USB/usb_debug_transport.cpp",
)


def main() -> int:
    failures = []
    contents = {}
    for path in PROTOCOL_FILES:
        if not path.is_file():
            failures.append(f"missing Phase 4 source: {path}")
        else:
            contents[path] = path.read_text(encoding="utf-8")

    protocol = "\n".join(contents.values())
    forbidden = {
        r"\bvolatile\s+bool\b": "volatile bool",
        r"\b(?:CDC_|USBD_|HAL_USB)\w*\s*\(": "USB HAL call",
        r"\b(?:can_|CAN)\w*\s*\(": "CAN call",
        r"\b(?:printf|snprintf|sprintf|vsnprintf|malloc|calloc|realloc|free)\s*\(": "format/allocation call",
        r"\b(?:osDelay|osWait|taskYIELD|vTaskDelay|xQueue)\s*": "blocking/RTOS API",
        r"\bstd::string\b": "dynamic string",
    }
    for pattern, label in forbidden.items():
        if re.search(pattern, protocol):
            failures.append(f"forbidden {label} in protocol/transport")

    required = (
        "kFrameMagic", "kProtocolVersion", "kSchemaVersion",
        "kHeaderSize", "kMaxPayloadSize", "crc32", "StreamParser",
        "unknown_versions", "unknown_messages", "rx_overflow",
        "kCriticalTxCapacity", "kResultStateTxCapacity",
        "kCalibrationTxCapacity", "kScopeTxCapacity", "kLogTxCapacity",
        "RequestTracker", "DUPLICATE", "EXPIRED",
        "binary_session_active", "publish_command_result",
        "begin_tx_chunk", "complete_tx_chunk", "active_tx_offset_",
        "std::atomic<uint32_t> tx_sequence_", "encode_device_frame_parts",
    )
    for fragment in required:
        if fragment not in protocol:
            failures.append(f"missing Phase 4 contract fragment: {fragment}")

    cdc_source = (ROOT / "Firmware/Board/v3/Src/usbd_cdc_if.c").read_text(
        encoding="utf-8")
    receive_start = cdc_source.find("static int8_t CDC_Receive_FS")
    receive_end = cdc_source.find("uint8_t CDC_Transmit_FS", receive_start)
    receive_body = cdc_source[receive_start:receive_end]
    for forbidden_text in ("decode_frame", "StreamParser", "crc32", "Axis",
                           "requested_state_", "error_", "is_armed_"):
        if forbidden_text in receive_body:
            failures.append(f"USB callback directly contains {forbidden_text}")
    if "usb_cdc_ingress" not in receive_body:
        failures.append("USB callback does not enqueue bytes into the C ingress ring")
    if "usb_debug_receive_bytes" in receive_body:
        failures.append("USB callback crosses directly into the C++ transport")

    interface_usb = (ROOT / "Firmware/communication/interface_usb.cpp").read_text(
        encoding="utf-8")
    tx_event_start = interface_usb.find("case 3: // CDC TX complete")
    tx_event_end = interface_usb.find("break;", tx_event_start)
    tx_event_body = interface_usb[tx_event_start:tx_event_end]
    if "complete_active_packet" in tx_event_body:
        failures.append("USB TX event mutates the active descriptor instead of reconciling CDC state")

    transport = contents.get(ROOT / "Firmware/USB/usb_debug_transport.cpp", "")
    for forbidden_text in ("requested_state_", "error_", "is_armed_",
                           "pwm_", "CDC_Transmit_FS", "USBD_"):
        if forbidden_text in transport:
            failures.append(f"USB command transport directly touches {forbidden_text}")
    if "command_submit_" not in transport or "CommandSource::USB" not in transport:
        failures.append("USB command path does not submit a typed USB Command")
    if "queue_command_result" not in transport:
        failures.append("USB command path has no explicit CommandResult response")

    interface = (ROOT / "Firmware/communication/interface_usb.cpp").read_text(
        encoding="utf-8")
    if ("usb_debug_begin_tx_chunk" not in interface or
            "usb_debug_complete_tx_chunk" not in interface or
            "CDC_Transmit_FS" not in interface):
        failures.append("CDC task does not own bounded binary TX chunk submission")
    if "USB_TX_DATA_SIZE" not in interface:
        failures.append("CDC binary TX is not bounded by the endpoint packet size")
    if "kUsbTxChunkSize = USB_TX_DATA_SIZE - 1u" not in interface:
        failures.append("CDC binary TX can re-enter the fragile full-packet ZLP path")
    if "log_buffer" in interface or "@usb_" in interface:
        failures.append("CDC task still multiplexes legacy text with binary frames")
    if "usb_stdout_write_frame" in interface:
        failures.append("removed raw stdout-frame compatibility API returned")
    if "binary protocol transport from the moment it is configured" not in interface:
        failures.append("CDC task does not declare the binary-only stream contract")
    if "!binary_session_active_.load" not in transport:
        failures.append("binary telemetry is not gated by HELLO session state")

    main_source = (ROOT / "Firmware/MotorControl/main.cpp").read_text(
        encoding="utf-8")
    if "log_task_create()" in main_source:
        failures.append("legacy @log producer is still started on binary CDC")

    supervisor_header = (ROOT / "Firmware/MotorControl/safety_supervisor.hpp").read_text(
        encoding="utf-8")
    if "CommandSource source" not in supervisor_header:
        failures.append("CommandResult does not retain CommandSource")
    if "result.source != odrive::safety::CommandSource::USB" not in transport:
        failures.append("USB transport does not isolate non-USB command results")

    calibration = (ROOT / "Firmware/communication/calibration_transport.cpp").read_text(
        encoding="utf-8")
    if "usb_stdout_write_frame" in calibration:
        failures.append("calibration still writes a raw frame to stdout CDC")

    cmake = (ROOT / "Firmware/CMakeLists.txt").read_text(encoding="utf-8")
    if "MotorControl/log_task.cpp" in cmake:
        failures.append("legacy log task is still linked into binary-only firmware")
    manifest = (ROOT / "tools/generate_build_manifest.py").read_text(
        encoding="utf-8")
    for fragment in ("USB/usb_debug_protocol.cpp", "USB/usb_debug_transport.cpp",
                     "phase4_static_gate", "USB_PROTOCOL_SCHEMA",
                     "--protocol-schema", "--identity-header",
                     "--usb-protocol-version"):
        if fragment not in cmake:
            failures.append(f"CMake missing Phase 4 manifest/source fragment: {fragment}")
    for fragment in ("protocol_schema_sha256", "usb_protocol_version",
                     "identity_header", "kManifestIdentity"):
        if fragment not in manifest:
            failures.append(f"build manifest missing Phase 4 identity fragment: {fragment}")

    axis = (ROOT / "Firmware/MotorControl/axis.cpp").read_text(encoding="utf-8")
    supervisor = (ROOT / "Firmware/MotorControl/safety_supervisor.cpp").read_text(
        encoding="utf-8")
    phase3 = (ROOT / "Firmware/MotorControl/trace_rings.hpp").read_text(
        encoding="utf-8")
    for fragment, source in (
        ("CommandType::SET_OPERATION", supervisor),
        ("RealtimeRequestType::PREPARE_OPERATION", supervisor),
        ("RealtimeEventType::READY_SNAPSHOT", supervisor),
        ("RealtimeEventType::ARM_CONFIRMED", supervisor),
        ("SafetyState::ARMED", supervisor),
        ("trace_consumer_.consume()", axis),
        ("CriticalEventRing", phase3),
        ("ScopeRing", phase3),
    ):
        if fragment not in source:
            failures.append(f"Phase 2/3 boundary missing: {fragment}")

    schema = ROOT / "docs/usb_debug_protocol_schema.json"
    if not schema.is_file():
        failures.append("missing authoritative USB protocol schema")
    else:
        schema_text = schema.read_text(encoding="utf-8")
        for fragment in ("payloads", "FAULT_EVENT", "SCOPE_RECORD",
                         "polynomial_reflected", "usb_packet_fragmentation",
                         "source_isolation"):
            if fragment not in schema_text:
                failures.append(
                    f"USB protocol schema missing contract fragment: {fragment}")

    if failures:
        for failure in failures:
            print(f"phase4 static gate: FAIL: {failure}")
        return 1
    print("phase4 static gate passed: framed USB boundary, task-only command routing, and Phase 2/3 invariants retained")
    return 0


if __name__ == "__main__":
    sys.exit(main())
