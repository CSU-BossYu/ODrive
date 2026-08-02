#!/usr/bin/env python3
"""Static checks for the Phase 2 Supervisor and ISR event boundary."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
NEW_FILES = (
    ROOT / "Firmware/MotorControl/realtime_event_ring.hpp",
    ROOT / "Firmware/MotorControl/realtime_event_ring.cpp",
    ROOT / "Firmware/MotorControl/safety_supervisor.hpp",
    ROOT / "Firmware/MotorControl/safety_supervisor.cpp",
)


def main() -> int:
    failures = []
    contents = {}
    for path in NEW_FILES:
        if not path.is_file():
            failures.append(f"missing Phase 2 source: {path}")
            continue
        contents[path] = path.read_text(encoding="utf-8")

    ring_and_supervisor = "\n".join(contents.values())
    forbidden_patterns = {
        r"\bvolatile\s+bool\b": "volatile bool",
        r"\b(?:usb|USB)_[A-Za-z0-9_]*\s*\(|\bcan_(?:send|write|transmit|get|set)_[A-Za-z0-9_]*\s*\(": "USB/CAN call",
        r"\b(?:printf|snprintf|sprintf|vsnprintf|malloc|calloc|realloc|free)\s*\(": "formatting or allocation call",
        r"\b(?:osDelay|osWait|taskYIELD)\s*\(": "blocking or scheduling call",
        r"\bstd::string\b": "dynamic string",
        r"closed_loop_(?:phase_feedback_ready|controller_ready)_": "unverified readiness bool",
        r"error_\s*\|=": "unmarked legacy error write",
    }
    for pattern, label in forbidden_patterns.items():
        if re.search(pattern, ring_and_supervisor):
            failures.append(f"forbidden {label} in Phase 2 boundary")

    can_source = (ROOT / "Firmware/communication/can/can_simple.cpp").read_text(
        encoding="utf-8"
    )
    axis_source = (ROOT / "Firmware/MotorControl/axis.cpp").read_text(
        encoding="utf-8"
    )
    supervisor_source = contents.get(
        ROOT / "Firmware/MotorControl/safety_supervisor.cpp", ""
    )
    if re.search(r"requested_state_\s*=", can_source):
        failures.append("CAN callback writes requested_state_ directly")

    integration_fragments = {
        "SafetySupervisor::tick": "safety_supervisor_.tick(" in axis_source,
        "RealtimeRequest consumer": "pop_realtime_request(" in axis_source,
        "task-context confirmation": "handle_realtime_event(" in axis_source,
        "CAN clear command": "CommandType::CLEAR_FAULTS" in can_source,
    }
    for label, present in integration_fragments.items():
        if not present:
            failures.append(f"missing Phase 2 integration: {label}")

    arm_confirmation = re.compile(
        r"SafetyState::READY\s*,\s*TransitionEvent::ARM_CONFIRMED\s*,"
        r"\s*SafetyState::ARMED",
        re.MULTILINE,
    )
    if not arm_confirmation.search(supervisor_source):
        failures.append("missing READY + ARM_CONFIRMED -> ARMED transition")

    required_fragments = {
        "std::atomic": ring_and_supervisor,
        "kRequiredReadinessFlags": ring_and_supervisor,
        "kTransitions": ring_and_supervisor,
        "RealtimeRequestType": ring_and_supervisor,
        "request_id": ring_and_supervisor,
        "state_epoch": ring_and_supervisor,
        "FaultManager": ring_and_supervisor,
    }
    for fragment, source in required_fragments.items():
        if fragment not in source:
            failures.append(f"missing Phase 2 contract fragment: {fragment}")

    if failures:
        for failure in failures:
            print(f"phase2 static gate: FAIL: {failure}")
        return 1

    print("phase2 static gate passed: bounded ISR boundary and explicit Supervisor contracts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
