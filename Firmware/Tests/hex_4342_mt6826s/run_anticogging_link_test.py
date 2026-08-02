#!/usr/bin/env python3
"""
Stage D: conservative anticogging CAN link test for the MT6826S/vernier build.

This script intentionally does not start a full anticogging calibration by
default. Full calibration depends on reliable position following; until that is
qualified, this gate only proves the production CAN anticogging surface:

- read flags/index/thresholds/ratio/system error
- read selected cogging_map entries
- optionally write the current threshold values back unchanged
- verify the axis remains error-free and returns/stays IDLE
"""

import argparse
import math
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_IDLE,
    AXIS_STATES,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    clear_errors,
    decode_anticogging_flags,
    get_anticogging_status,
    get_status_ex,
    open_bus,
    set_anticogging_config,
    set_requested_state,
    wait_heartbeat,
)


ANTICOGGING_ITEMS = {
    0x01: ("flags", "uint32"),
    0x02: ("index", "uint32"),
    0x03: ("calib_pos_threshold", "float32"),
    0x04: ("calib_vel_threshold", "float32"),
    0x05: ("cogging_ratio", "float32"),
    0x06: ("odrv_error", "uint32"),
}


def require_ok_response(resp, what):
    if resp is None:
        raise RuntimeError(f"{what}: no response")
    status = EXT_STATUS.get(resp["status"], resp["status"])
    if resp["status"] != 0:
        raise RuntimeError(f"{what}: status={status} raw={resp['raw']}")
    return resp


def read_item(bus, args, item_id, value=0):
    name, _ = ANTICOGGING_ITEMS.get(item_id, (f"item_0x{item_id:02X}", "?"))
    resp = require_ok_response(
        get_anticogging_status(
            bus,
            args.node_id,
            item_id,
            value=value,
            extended_id=args.extended_id,
            timeout=args.timeout,
        ),
        f"GET_ANTICOGGING_STATUS {name}",
    )
    if resp["type"] == EXT_TYPE_FLOAT32:
        value_out = resp["value_f"]
    else:
        value_out = resp["value_u"]
    return resp, value_out


def write_item(bus, args, item_id, req_type, value=0, value_float=None):
    resp = require_ok_response(
        set_anticogging_config(
            bus,
            args.node_id,
            item_id,
            req_type,
            value=value,
            value_float=value_float,
            extended_id=args.extended_id,
        ),
        f"SET_ANTICOGGING_CONFIG item=0x{item_id:02X}",
    )
    print(f"  set item=0x{item_id:02X}: {EXT_STATUS.get(resp['status'], resp['status'])} raw={resp['raw']}")
    return resp


def read_snapshot(bus, args):
    values = {}
    raw = {}
    for item_id, (name, _) in ANTICOGGING_ITEMS.items():
        resp, value = read_item(bus, args, item_id)
        values[name] = value
        raw[name] = resp["raw"]

    values["decoded_flags"] = decode_anticogging_flags(values["flags"])
    return values, raw


def print_snapshot(title, values):
    flags = values["decoded_flags"]
    print(title)
    print(
        "  flags=0x{flags:08X} calib={calib} valid={valid} pre_calibrated={pre} enabled={enabled}".format(
            flags=values["flags"],
            calib=int(flags["calib_anticogging"]),
            valid=int(flags["anticogging_valid"]),
            pre=int(flags["pre_calibrated"]),
            enabled=int(flags["anticogging_enabled"]),
        )
    )
    print(
        f"  index={values['index']} pos_threshold={values['calib_pos_threshold']:.6g} counts "
        f"vel_threshold={values['calib_vel_threshold']:.6g} counts/s"
    )
    print(f"  cogging_ratio={values['cogging_ratio']:.9g} turns/index odrv_error=0x{values['odrv_error']:08X}")


def parse_indices(text):
    indices = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        index = int(part, 0)
        if index < 0 or index >= 3600:
            raise argparse.ArgumentTypeError("map indices must be in [0, 3599]")
        indices.append(index)
    return indices


def check_heartbeat_clean(hb, context):
    if hb is None:
        raise RuntimeError(f"{context}: heartbeat timeout")
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError(
            f"{context}: heartbeat reports error state={hb['axis_state']} "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )


def ensure_idle(bus, args):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
    check_heartbeat_clean(hb, "initial")
    print(
        f"Heartbeat: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
        f"axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}"
    )
    if hb["axis_state"] == AXIS_STATE_IDLE:
        return

    print("Axis is armed; requesting IDLE before anticogging config access...")
    set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
    deadline = time.monotonic() + args.idle_timeout
    last = hb
    while time.monotonic() < deadline:
        last = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if last is None:
            continue
        check_heartbeat_clean(last, "waiting for IDLE")
        print(
            f"  hb state={last['axis_state']}({AXIS_STATES.get(last['axis_state'], '?')}) "
            f"axis_error=0x{last['axis_error']:08X}"
        )
        if last["axis_state"] == AXIS_STATE_IDLE:
            return
    raise RuntimeError(f"Timed out waiting for IDLE; last heartbeat={last}")


def main():
    parser = argparse.ArgumentParser(description="Stage D: anticogging CAN link test for MT6826S/vernier firmware.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--idle-timeout", type=float, default=3.0)
    parser.add_argument("--map-indices", type=parse_indices, default=parse_indices("0,900,1800,2700,3599"))
    parser.add_argument(
        "--write-same-config",
        action="store_true",
        help="Write the currently read anticogging enable/threshold values back unchanged while IDLE.",
    )
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        if args.clear_at_start:
            print("Clearing existing errors...")
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        ensure_idle(bus, args)

        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
        if status is None:
            raise RuntimeError("GET_AXIS_STATUS_EX timeout")
        print(
            f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
            f"motor_calibrated={status['motor_calibrated']} encoder_ready={status['encoder_ready']} "
            f"trajectory_done={status['trajectory_done']} axis_error=0x{status['axis_error']:08X}"
        )
        if status["axis_error"]:
            raise RuntimeError("Axis status reports error")

        before, before_raw = read_snapshot(bus, args)
        print_snapshot("Anticogging before:", before)

        print("Reading cogging_map samples:")
        map_values = {}
        for index in args.map_indices:
            resp = require_ok_response(
                get_anticogging_status(
                    bus,
                    args.node_id,
                    0x10,
                    value=index,
                    extended_id=args.extended_id,
                    timeout=args.timeout,
                ),
                f"GET_ANTICOGGING_STATUS map[{index}]",
            )
            value = resp["value_f"]
            if not math.isfinite(value):
                raise RuntimeError(f"map[{index}] is not finite: {value}")
            map_values[index] = value
            print(f"  map[{index:4d}]={value:+.9g} raw={resp['raw']}")

        if args.write_same_config:
            print("Writing current anticogging config values back unchanged...")
            enabled = 1 if before["decoded_flags"]["anticogging_enabled"] else 0
            write_item(bus, args, 0x01, EXT_TYPE_UINT32, value=enabled)
            write_item(bus, args, 0x03, EXT_TYPE_FLOAT32, value_float=before["calib_pos_threshold"])
            write_item(bus, args, 0x04, EXT_TYPE_FLOAT32, value_float=before["calib_vel_threshold"])

            after, _ = read_snapshot(bus, args)
            print_snapshot("Anticogging after same-config write:", after)
            if after["decoded_flags"]["anticogging_enabled"] != before["decoded_flags"]["anticogging_enabled"]:
                raise RuntimeError("anticogging_enabled changed unexpectedly")
            for key in ("calib_pos_threshold", "calib_vel_threshold"):
                if not math.isclose(after[key], before[key], rel_tol=1e-6, abs_tol=1e-6):
                    raise RuntimeError(f"{key} changed unexpectedly: before={before[key]} after={after[key]}")

        final_hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=2.0)
        check_heartbeat_clean(final_hb, "final")
        final_status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
        print(f"Final heartbeat: {final_hb}")
        print(f"Final status: {final_status}")
        if final_status is None or final_status["axis_error"]:
            raise RuntimeError("Final status reports axis error")

        print("PASS: anticogging CAN link test completed")
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
