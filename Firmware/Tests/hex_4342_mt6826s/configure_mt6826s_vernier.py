#!/usr/bin/env python3
"""Configure ODrive for dual MT6826S vernier diagnostic mode over CANSimple."""
import argparse
import time

from common import (
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_INT32,
    EXT_TYPE_UINT32,
    clear_errors,
    get_basic_config,
    open_bus,
    save_configuration,
    set_basic_config,
    wait_heartbeat,
)


ENCODER_MODE_SPI_ABS_MT6826S_VERNIER = 0x106


def format_value(resp):
    if resp["type"] == EXT_TYPE_FLOAT32:
        return resp["value_f"]
    return resp["value_i"]


def read_and_print(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"{name}: TIMEOUT")
        return None
    status = EXT_STATUS.get(resp["status"], resp["status"])
    value = format_value(resp)
    print(f"{name}: {value} status={status} raw={resp['raw']}")
    return value if resp["status"] == 0 else None


def check_status(resp, label):
    if resp is None:
        print(f"{label}: TIMEOUT")
        return False
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"{label}: {status} raw={resp['raw']}")
    return resp["status"] == 0


def set_u32(bus, args, param_id, name, value):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_UINT32,
        value=value, extended_id=args.extended_id,
    )
    return check_status(resp, f"{name} <- {value}")


def set_i32(bus, args, param_id, name, value):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_INT32,
        value=value, extended_id=args.extended_id,
    )
    return check_status(resp, f"{name} <- {value}")


def set_f32(bus, args, param_id, name, value):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_FLOAT32,
        value_float=value, extended_id=args.extended_id,
    )
    return check_status(resp, f"{name} <- {value}")


def read_all(bus, args):
    print("Current encoder config:")
    read_and_print(bus, args, 0x20, "  mode")
    read_and_print(bus, args, 0x21, "  cpr")
    read_and_print(bus, args, 0x22, "  main_cs")
    read_and_print(bus, args, 0x24, "  aux_cs")
    read_and_print(bus, args, 0x25, "  vernier_virtual_cpr")
    read_and_print(bus, args, 0x26, "  vernier_main_ratio")
    read_and_print(bus, args, 0x27, "  vernier_aux_ratio")
    read_and_print(bus, args, 0x28, "  vernier_main_offset")
    read_and_print(bus, args, 0x29, "  vernier_aux_offset")
    read_and_print(bus, args, 0x2A, "  vernier_main_reversed")
    read_and_print(bus, args, 0x2B, "  vernier_aux_reversed")
    read_and_print(bus, args, 0x33, "  vernier_output_reversed")
    read_and_print(bus, args, 0x2C, "  mt6826s_spi_mode")
    read_and_print(bus, args, 0x2D, "  vernier_err_accept")
    read_and_print(bus, args, 0x2E, "  vernier_err_reject")
    read_and_print(bus, args, 0x2F, "  mt6826s_spi_prescaler")


def main():
    parser = argparse.ArgumentParser(description="Configure dual MT6826S vernier mode.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--clear-errors", action="store_true")
    parser.add_argument("--save", action="store_true")
    parser.add_argument("--skip-mode", action="store_true")
    parser.add_argument("--cpr", type=int, default=32768)
    parser.add_argument("--pole-pairs", type=int, default=14)
    parser.add_argument("--main-cs", type=int, default=2)
    parser.add_argument("--aux-cs", type=int, default=4)
    parser.add_argument("--virtual-cpr", type=int, default=32768)
    parser.add_argument("--main-ratio", type=float, default=41.0)
    parser.add_argument("--aux-ratio", type=float, default=42.0)
    parser.add_argument("--main-offset", type=float)
    parser.add_argument("--aux-offset", type=float)
    parser.add_argument("--main-reversed", type=int, choices=[0, 1])
    parser.add_argument("--aux-reversed", type=int, choices=[0, 1])
    parser.add_argument("--output-reversed", type=int, choices=[0, 1])
    parser.add_argument("--spi-mode", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--spi-prescaler", type=int, choices=[2, 4, 8, 16, 32, 64, 128, 256], default=8)
    parser.add_argument("--err-accept", type=float)
    parser.add_argument("--err-reject", type=float)
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}")

        if args.clear_errors:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        if args.read_only:
            read_all(bus, args)
            return 0

        ok = True
        ok &= set_i32(bus, args, 0x11, "motor.pole_pairs", args.pole_pairs)
        if not args.skip_mode:
            ok &= set_u32(bus, args, 0x20, "encoder.mode", ENCODER_MODE_SPI_ABS_MT6826S_VERNIER)
        ok &= set_i32(bus, args, 0x21, "encoder.cpr", args.cpr)
        ok &= set_u32(bus, args, 0x22, "encoder.abs_spi_cs_gpio_pin", args.main_cs)
        ok &= set_u32(bus, args, 0x24, "encoder.abs_spi_aux_cs_gpio_pin", args.aux_cs)
        ok &= set_i32(bus, args, 0x25, "encoder.vernier_virtual_cpr", args.virtual_cpr)
        ok &= set_u32(bus, args, 0x2C, "encoder.mt6826s_spi_mode", args.spi_mode)
        ok &= set_u32(bus, args, 0x2F, "encoder.mt6826s_spi_prescaler", args.spi_prescaler)

        optional_floats = [
            (args.main_ratio, 0x26, "encoder.vernier_main_ratio"),
            (args.aux_ratio, 0x27, "encoder.vernier_aux_ratio"),
            (args.main_offset, 0x28, "encoder.vernier_main_offset"),
            (args.aux_offset, 0x29, "encoder.vernier_aux_offset"),
            (args.err_accept, 0x2D, "encoder.vernier_err_accept"),
            (args.err_reject, 0x2E, "encoder.vernier_err_reject"),
        ]
        for value, param_id, name in optional_floats:
            if value is not None:
                ok &= set_f32(bus, args, param_id, name, value)

        if args.main_reversed is not None:
            ok &= set_u32(bus, args, 0x2A, "encoder.vernier_main_reversed", args.main_reversed)
        if args.aux_reversed is not None:
            ok &= set_u32(bus, args, 0x2B, "encoder.vernier_aux_reversed", args.aux_reversed)
        if args.output_reversed is not None:
            ok &= set_u32(bus, args, 0x33, "encoder.vernier_output_reversed", args.output_reversed)

        if args.save:
            resp = save_configuration(bus, args.node_id, args.extended_id, timeout=2.0)
            ok &= check_status(resp, "save_configuration")

        print()
        read_all(bus, args)
        return 0 if ok else 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
