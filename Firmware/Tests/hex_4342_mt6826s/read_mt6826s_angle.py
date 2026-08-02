#!/usr/bin/env python3
import argparse
import math
import struct
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    CMD_GET_ENCODER_ERROR,
    CMD_GET_ENCODER_ESTIMATES,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    clear_errors,
    get_basic_config,
    get_status_ex,
    open_bus,
    request_two_floats,
    request_u32,
    send,
    wait_heartbeat,
)


CMD_GET_ENCODER_COUNT = 0x00A
ENCODER_MODE_SPI_ABS_MT6826S = 0x105


def read_encoder_count(bus, node_id, extended_id=False, timeout=1.0):
    send(bus, node_id, CMD_GET_ENCODER_COUNT, b"", extended_id)
    expected_id = (node_id << 5) | CMD_GET_ENCODER_COUNT
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(deadline - time.monotonic())
        if msg is None:
            continue
        if msg.arbitration_id != expected_id or msg.is_extended_id != extended_id or msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        if len(data) < 8:
            continue
        shadow_count, count_in_cpr = struct.unpack("<ii", data[:8])
        return {
            "raw": data.hex(" "),
            "shadow_count": shadow_count,
            "count_in_cpr": count_in_cpr,
        }
    return None


def read_config_value(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"{name}: TIMEOUT")
        return None
    status = EXT_STATUS.get(resp["status"], resp["status"])
    if resp["type"] == EXT_TYPE_FLOAT32:
        value = resp["value_f"]
    else:
        value = resp["value_i"]
    print(f"{name}: {value} status={status} raw={resp['raw']}")
    return value if resp["status"] == 0 else None


def print_sample(bus, args):
    count = read_encoder_count(bus, args.node_id, args.extended_id, args.timeout)
    _, estimates = request_two_floats(
        bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=args.timeout
    )
    _, encoder_error = request_u32(
        bus, args.node_id, CMD_GET_ENCODER_ERROR, args.extended_id, timeout=args.timeout
    )
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)

    if count is None:
        print("Encoder count: TIMEOUT")
        return

    count_in_cpr = count["count_in_cpr"] % args.cpr
    angle_turns = count_in_cpr / args.cpr
    angle_deg = angle_turns * 360.0
    angle_rad = angle_turns * 2.0 * math.pi

    line = (
        f"count_in_cpr={count_in_cpr:5d}/{args.cpr} "
        f"angle={angle_turns:.8f} turns "
        f"{angle_deg:9.4f} deg "
        f"{angle_rad:9.6f} rad "
        f"shadow_count={count['shadow_count']}"
    )

    if estimates is not None:
        line += f" pos_est={estimates[0]: .8f} turns vel_est={estimates[1]: .8f} turns/s"

    if status is not None:
        line += f" axis_state={status['axis_state']} axis_error=0x{status['axis_error']:08X}"

    if encoder_error is not None:
        line += f" encoder_error=0x{encoder_error:08X}"

    print(line)


def main():
    parser = argparse.ArgumentParser(description="Read MT6826S angle through ODrive CANSimple.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--cpr", type=int, default=32768)
    parser.add_argument("--period", type=float, default=0.2)
    parser.add_argument("--samples", type=int, default=1, help="Number of samples to print. Use 0 for forever.")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--show-config", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(
            f"Heartbeat: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} "
            f"raw={hb['raw']}"
        )

        if args.clear_at_start:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        if args.show_config:
            mode = read_config_value(bus, args, 0x20, "encoder.mode")
            cpr = read_config_value(bus, args, 0x21, "encoder.cpr")
            read_config_value(bus, args, 0x22, "encoder.abs_spi_cs_gpio_pin")
            if mode is not None and int(mode) != ENCODER_MODE_SPI_ABS_MT6826S:
                print(f"WARN: encoder.mode is {mode}, expected {ENCODER_MODE_SPI_ABS_MT6826S} for MT6826S")
            if cpr is not None:
                args.cpr = int(cpr)

        n = 0
        while args.samples == 0 or n < args.samples:
            print_sample(bus, args)
            n += 1
            if args.samples == 0 or n < args.samples:
                time.sleep(args.period)
        return 0
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
