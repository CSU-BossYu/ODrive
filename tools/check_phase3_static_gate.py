#!/usr/bin/env python3
"""Static checks for the Phase 3 snapshot and trace boundary."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
TRACE_FILES = (
    ROOT / "Firmware/MotorControl/realtime_snapshot.hpp",
    ROOT / "Firmware/MotorControl/realtime_snapshot.cpp",
    ROOT / "Firmware/MotorControl/trace_rings.hpp",
    ROOT / "Firmware/MotorControl/trace_rings.cpp",
)


def main() -> int:
    failures = []
    contents = {}
    for path in TRACE_FILES:
        if not path.is_file():
            failures.append(f"missing Phase 3 source: {path}")
            continue
        contents[path] = path.read_text(encoding="utf-8")

    trace_source = "\n".join(contents.values())
    forbidden_patterns = {
        r"\bvolatile\s+bool\b": "volatile bool",
        r"\b(?:usb|USB)_[A-Za-z0-9_]*\s*\(|\bcan_(?:send|write|transmit|get|set)_[A-Za-z0-9_]*\s*\(": "USB/CAN call",
        r"\b(?:printf|snprintf|sprintf|vsnprintf|malloc|calloc|realloc|free)\s*\(": "formatting or allocation call",
        r"\b(?:osDelay|osWait|taskYIELD|vTaskDelay|xQueue)\s*": "blocking/RTOS API",
        r"\bstd::string\b": "dynamic string",
    }
    for pattern, label in forbidden_patterns.items():
        if re.search(pattern, trace_source):
            failures.append(f"forbidden {label} in Phase 3 trace boundary")

    required_fragments = (
        "CriticalEventRing",
        "StateEventRing",
        "LogRing",
        "ScopeRing",
        "CriticalBlackBox",
        "RealtimeSnapshotBuffer",
        "std::atomic",
        "overflow_count",
        "dropped_count",
        "freeze_on_first_fault",
        "post_trigger_remaining",
        "TraceConsumer",
        "sort_batch",
    )
    for fragment in required_fragments:
        if fragment not in trace_source:
            failures.append(f"missing Phase 3 contract fragment: {fragment}")

    axis_source = (ROOT / "Firmware/MotorControl/axis.cpp").read_text(
        encoding="utf-8"
    )
    main_source = (ROOT / "Firmware/MotorControl/main.cpp").read_text(
        encoding="utf-8"
    )
    can_source = (ROOT / "Firmware/communication/can/can_simple.cpp").read_text(
        encoding="utf-8"
    )
    cmake_source = (ROOT / "Firmware/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    if "capture_realtime_snapshot_from_isr" not in main_source:
        failures.append("control ISR does not publish RealtimeSnapshot")
    if "finish_realtime_snapshot_from_isr" not in main_source:
        failures.append("control ISR does not finalize complete-cycle timing")
    if "supervisor_trace_mailbox_.read_from_isr" not in axis_source:
        failures.append("ISR does not read a coherent Supervisor trace mailbox")
    capture_start = axis_source.find("void Axis::capture_realtime_snapshot_from_isr")
    capture_end = axis_source.find("bool Axis::read_realtime_snapshot", capture_start)
    if capture_start >= 0 and capture_end >= 0:
        capture_body = axis_source[capture_start:capture_end]
        if "safety_supervisor_." in capture_body:
            failures.append("snapshot ISR reads task-owned SafetySupervisor fields")
    if "service_trace_buffers" not in axis_source or "trace_consumer_.consume()" not in axis_source:
        failures.append("Axis task does not consume trace rings")
    if re.search(r"requested_state_\s*=", can_source):
        failures.append("CAN callback writes requested_state_ directly")
    if "phase3_static_gate" not in cmake_source:
        failures.append("Phase 3 static gate is not in CMake")
    if "MotorControl/trace_rings.cpp" not in cmake_source:
        failures.append("trace ring source is not in the firmware manifest")

    trace_impl = contents.get(ROOT / "Firmware/MotorControl/trace_rings.cpp", "")
    pressure_pos = trace_impl.find("const bool critical_pressure")
    drain_pos = trace_impl.find("critical_.pop(&critical)")
    if pressure_pos < 0 or drain_pos < 0 or pressure_pos > drain_pos:
        failures.append("Critical pressure is measured after draining the ring")
    isr_record_start = axis_source.find("bool Axis::record_critical_event_from_isr")
    isr_record_end = axis_source.find(
        "bool Axis::snapshot_critical_black_box", isr_record_start)
    if isr_record_start < 0 or isr_record_end < 0 or "freeze_on_first_fault" not in axis_source[isr_record_start:isr_record_end]:
        failures.append("first ISR fault does not freeze the critical black box")

    # Phase 2 transaction invariants must remain visible after Phase 3.
    phase2_fragments = (
        "CommandType::SET_OPERATION",
        "RealtimeRequestType::PREPARE_OPERATION",
        "RealtimeEventType::READY_SNAPSHOT",
        "RealtimeEventType::ARM_CONFIRMED",
        "SafetyState::ARMED",
    )
    phase2_source = axis_source + "\n" + (
        ROOT / "Firmware/MotorControl/safety_supervisor.cpp"
    ).read_text(encoding="utf-8")
    for fragment in phase2_fragments:
        if fragment not in phase2_source:
            failures.append(f"Phase 2 transaction fragment disappeared: {fragment}")

    if failures:
        for failure in failures:
            print(f"phase3 static gate: FAIL: {failure}")
        return 1

    print("phase3 static gate passed: fixed trace boundary, isolated critical ring, and Phase 2 transaction retained")
    return 0


if __name__ == "__main__":
    sys.exit(main())
