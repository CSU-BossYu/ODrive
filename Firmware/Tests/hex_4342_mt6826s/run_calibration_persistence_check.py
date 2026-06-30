#!/usr/bin/env python3
"""
Check and optionally persist the runtime motor/encoder calibration.

By default this is read-only. Use --mark-precalibrated to set runtime/config
pre_calibrated flags and --save to write NVM. Saving triggers the firmware's
normal reset-after-ACK path.
"""

import argparse
import math
import time

from common import (
    AXIS_STATE_IDLE,
    AXIS_STATES,
    clear_errors,
    get_calib_result,
    get_status_ex,
    open_bus,
    request_u32,
    request_u64,
    save_configuration,
    set_precalibrated,
    set_requested_state,
    wait_heartbeat,
    CMD_GET_CONTROLLER_ERROR,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_MOTOR_ERROR,
)


CALIB_ITEMS = [
    (0x01, "phase_resistance", "float"),
    (0x02, "phase_inductance", "float"),
    (0x03, "phase_offset", "int"),
    (0x04, "direction", "int"),
]


def require_clean_heartbeat(hb, context):
    if hb is None:
        raise RuntimeError(f"{context}: heartbeat timeout")
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError(
            f"{context}: heartbeat reports error state={hb['axis_state']} "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )


def read_errors(bus, args):
    _, motor_error = request_u64(bus, args.node_id, CMD_GET_MOTOR_ERROR, args.extended_id, timeout=args.timeout)
    _, encoder_error = request_u32(bus, args.node_id, CMD_GET_ENCODER_ERROR, args.extended_id, timeout=args.timeout)
    _, controller_error = request_u32(bus, args.node_id, CMD_GET_CONTROLLER_ERROR, args.extended_id, timeout=args.timeout)
    return {
        "motor_error": motor_error,
        "encoder_error": encoder_error,
        "controller_error": controller_error,
    }


def read_calibration(bus, args):
    values = {}
    raws = {}
    for item_id, name, _kind in CALIB_ITEMS:
        result = get_calib_result(bus, args.node_id, item_id, args.extended_id)
        if result is None:
            raise RuntimeError(f"{name}: GET_CALIB_RESULT timeout")
        resp, value = result
        if resp["status"] != 0:
            raise RuntimeError(f"{name}: status={resp['status']} raw={resp['raw']}")
        values[name] = value
        raws[name] = resp["raw"]
    return values, raws


def validate_calibration(values):
    failures = []
    if not math.isfinite(float(values["phase_resistance"])) or values["phase_resistance"] <= 0.0:
        failures.append(f"phase_resistance invalid: {values['phase_resistance']}")
    if not math.isfinite(float(values["phase_inductance"])) or values["phase_inductance"] <= 0.0:
        failures.append(f"phase_inductance invalid: {values['phase_inductance']}")
    if values["direction"] == 0:
        failures.append("direction is 0; closed-loop states will be rejected")
    return failures


def print_snapshot(title, hb, status, errors, values):
    print(title)
    print(
        f"  heartbeat: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
        f"axis_error=0x{hb['axis_error']:08X} flags="
        f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X} raw={hb['raw']}"
    )
    print(f"  status_ex: {status}")
    print(
        "  errors: motor={motor} encoder={encoder} controller={controller}".format(
            motor="TIMEOUT" if errors["motor_error"] is None else f"0x{errors['motor_error']:016X}",
            encoder="TIMEOUT" if errors["encoder_error"] is None else f"0x{errors['encoder_error']:08X}",
            controller="TIMEOUT" if errors["controller_error"] is None else f"0x{errors['controller_error']:08X}",
        )
    )
    print(
        f"  calibration: R={values['phase_resistance']:.9g} "
        f"L={values['phase_inductance']:.9g} "
        f"phase_offset={values['phase_offset']} direction={values['direction']}"
    )


def read_snapshot(bus, args, title):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
    require_clean_heartbeat(hb, title)
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
    if status is None:
        raise RuntimeError(f"{title}: GET_AXIS_STATUS_EX timeout")
    if status["axis_error"]:
        raise RuntimeError(f"{title}: axis_error=0x{status['axis_error']:08X}")
    errors = read_errors(bus, args)
    values, _raws = read_calibration(bus, args)
    print_snapshot(title, hb, status, errors, values)
    failures = validate_calibration(values)
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
        raise RuntimeError("Calibration snapshot is not valid for closed-loop use")
    return hb, status, errors, values


def wait_for_reboot(bus, args):
    print("Waiting for save/reset cycle...")
    saw_gap = False
    deadline = time.monotonic() + args.reboot_timeout
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if hb is None:
            saw_gap = True
            break
    if saw_gap:
        print("  heartbeat gap observed")
    else:
        print("  no heartbeat gap observed; continuing to wait for a clean heartbeat")

    deadline = time.monotonic() + args.reboot_timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.5)
        if hb is None:
            continue
        last = hb
        print(
            f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error=0x{hb['axis_error']:08X}"
        )
        if hb["axis_state"] == AXIS_STATE_IDLE and hb["axis_error"] == 0:
            return hb
    raise RuntimeError(f"Timed out waiting for rebooted IDLE heartbeat; last={last}")


def main():
    parser = argparse.ArgumentParser(description="Check and optionally persist MT6826S/vernier calibration.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--reboot-timeout", type=float, default=10.0)
    parser.add_argument("--mark-precalibrated", action="store_true", help="Set motor+encoder pre_calibrated flags before saving/checking.")
    parser.add_argument("--save", action="store_true", help="Save configuration to NVM. This triggers firmware reset after ACK.")
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    if args.save and not args.mark_precalibrated:
        raise RuntimeError("--save requires --mark-precalibrated so the saved config can boot ready")

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        if args.clear_at_start:
            print("Clearing errors...")
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        read_snapshot(bus, args, "Before")

        if args.mark_precalibrated:
            print("Setting motor+encoder pre_calibrated flags...")
            resp = set_precalibrated(bus, args.node_id, flags=0x03, extended_id=args.extended_id)
            if resp is None or resp["status"] != 0:
                raise RuntimeError(f"SET_PRECALIBRATED failed: {resp}")
            time.sleep(0.2)
            read_snapshot(bus, args, "After SET_PRECALIBRATED")

        if args.save:
            print("Saving configuration; firmware should ACK then reset...")
            resp = save_configuration(bus, args.node_id, args.extended_id, timeout=2.0)
            if resp is None:
                print("  save_configuration response timed out; reset may already be in progress")
            elif resp["status"] != 0:
                raise RuntimeError(f"SAVE_CONFIGURATION failed: {resp}")
            else:
                print(f"  save response: {resp}")
            wait_for_reboot(bus, args)
            read_snapshot(bus, args, "After reboot")

        if args.clear_at_end:
            print("Clearing errors at end...")
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        print("PASS: calibration persistence check completed")
        return 0
    except Exception:
        print("Stopping motor due to failure...")
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
