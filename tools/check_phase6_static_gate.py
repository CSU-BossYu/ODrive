#!/usr/bin/env python3
"""Static safety and ownership checks for the retained Crash Recorder."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    failures: list[str] = []
    recorder_path = ROOT / "Firmware/MotorControl/crash_recorder.cpp"
    handler_path = ROOT / "Firmware/Board/v3/Src/stm32f4xx_it.c"
    linker_path = ROOT / "Firmware/Board/v3/STM32F405RGTx_FLASH.ld"
    transport_path = ROOT / "Firmware/USB/usb_debug_transport.cpp"
    protocol_path = ROOT / "Firmware/USB/usb_debug_protocol.hpp"
    symbolizer_path = ROOT / "foc_ui/backend/odrive_usb/symbolizer.py"
    for path in (recorder_path, handler_path, linker_path, transport_path,
                 protocol_path, symbolizer_path):
        if not path.is_file():
            failures.append(f"missing Phase 6 source: {path}")

    recorder = recorder_path.read_text(encoding="utf-8") if recorder_path.is_file() else ""
    handlers = handler_path.read_text(encoding="utf-8") if handler_path.is_file() else ""
    linker = linker_path.read_text(encoding="utf-8") if linker_path.is_file() else ""
    transport = transport_path.read_text(encoding="utf-8") if transport_path.is_file() else ""
    protocol = protocol_path.read_text(encoding="utf-8") if protocol_path.is_file() else ""
    symbolizer = symbolizer_path.read_text(encoding="utf-8") if symbolizer_path.is_file() else ""

    for pattern, label in {
        r"\b(?:USB|usb|CAN|can)[A-Za-z0-9_]*\s*\(": "USB/CAN call",
        r"\b(?:printf|snprintf|sprintf|vsnprintf|malloc|calloc|realloc|free)\s*\(": "format/allocation call",
        r"\b(?:osDelay|osWait|taskYIELD|vTaskDelay|xQueue)\w*\s*\(": "blocking/RTOS call",
    }.items():
        if re.search(pattern, recorder):
            failures.append(f"forbidden {label} in crash capture implementation")

    for name in ("HardFault_Handler", "MemManage_Handler", "BusFault_Handler",
                 "UsageFault_Handler"):
        start = handlers.find(f"void {name}")
        end = handlers.find("\n}", start)
        body = handlers[start:end]
        if start < 0 or "crash_recorder_capture" not in body:
            failures.append(f"{name} does not tail-call Crash Recorder")
        if "while" in body or "for (" in body:
            failures.append(f"{name} still contains a permanent fault loop")

    for fragment in ("TIM1->BDTR", "TIM8->BDTR", "TIM2->CCR3",
                     "NVIC_SystemReset", "SCB->CFSR", "SCB->HFSR",
                     "calculate_record_crc", "kManifestIdentity"):
        if fragment not in recorder:
            failures.append(f"Crash Recorder missing contract: {fragment}")
    for fragment in (".noinit (NOLOAD)", "KEEP(*(.noinit.*))"):
        if fragment not in linker:
            failures.append(f"linker missing retained-RAM contract: {fragment}")
    for fragment in ("CRASH_REPORT", "CRASH_ACK"):
        if fragment not in protocol or fragment not in transport:
            failures.append(f"USB Crash Recorder message missing: {fragment}")
    for fragment in ("record_build_id", "record_manifest_identity",
                     "arm-none-eabi-addr2line", "reliable",
                     "_wire_identity(build_id, 8)",
                     "_wire_identity(manifest_identity, 16)"):
        if fragment not in symbolizer:
            failures.append(f"backend exact symbolization contract missing: {fragment}")

    schema_path = ROOT / "docs/usb_debug_protocol_schema.json"
    try:
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        if schema.get("protocol_release") not in ("4.4", "4.5") or int(schema.get("schema_version", 0)) < 3:
            failures.append("USB schema no longer retains the Phase 6 Crash contract")
        if schema.get("payloads", {}).get("CRASH_REPORT", {}).get("length") != 204:
            failures.append("CRASH_REPORT payload is not fixed at 204 bytes")
    except (OSError, json.JSONDecodeError) as exc:
        failures.append(f"Crash Recorder schema read failed: {exc}")

    if failures:
        for failure in failures:
            print(f"phase6 static gate: FAIL: {failure}")
        return 1
    print("phase6 static gate passed: retained crash capture, safe reset, exact-build USB symbolization")
    return 0


if __name__ == "__main__":
    sys.exit(main())
