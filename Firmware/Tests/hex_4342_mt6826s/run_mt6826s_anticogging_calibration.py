#!/usr/bin/env python3
"""Run motor-side anticogging calibration through the MT6826S vernier coordinate."""

import math
import runpy
import sys
from pathlib import Path

from common import (
    EXT_TYPE_FLOAT32,
    get_anticogging_status,
    get_basic_config,
    open_bus,
    save_configuration,
)


ENCODER_MODE_SPI_ABS_MT6826S_VERNIER = 0x106
MAP_SIZE = 3600

DEFAULT_ARGS = {
    "--bitrate": "1000000",
    "--vel-limit": "0.02",
    "--current-limit": "3.0",
    "--pos-gain": "0.5",
    "--vel-gain": "1.0",
    "--vel-integrator-gain": "0.05",
    "--calib-pos-threshold": "2.0",
    "--calib-vel-threshold": "200.0",
    "--max-duration": "1800",
}


def option_value(argv, name, default=None):
    try:
        return argv[argv.index(name) + 1]
    except (ValueError, IndexError):
        return default


def add_defaults(argv):
    result = list(argv)
    for name, value in DEFAULT_ARGS.items():
        if name not in result:
            result.extend((name, value))
    return result


def preflight(argv):
    channel = option_value(argv, "--channel", "PCAN_USBBUS1")
    bitrate = int(option_value(argv, "--bitrate", DEFAULT_ARGS["--bitrate"]))
    node_id = int(option_value(argv, "--node-id", "0"))
    extended_id = "--extended-id" in argv

    bus = open_bus(channel, bitrate)
    try:
        mode = get_basic_config(bus, node_id, 0x20, extended_id)
        main_ratio = get_basic_config(bus, node_id, 0x26, extended_id)
        cogging_ratio = get_anticogging_status(
            bus, node_id, 0x05, extended_id=extended_id, timeout=1.0
        )
        if mode is None or mode["status"] != 0:
            raise RuntimeError("Failed to read encoder.mode")
        if mode["value_i"] != ENCODER_MODE_SPI_ABS_MT6826S_VERNIER:
            raise RuntimeError(
                f"Expected MT6826S vernier mode 0x106, got 0x{mode['value_i']:X}"
            )
        if main_ratio is None or main_ratio["status"] != 0:
            raise RuntimeError("Failed to read encoder.vernier_main_ratio")
        if cogging_ratio is None or cogging_ratio["status"] != 0:
            raise RuntimeError("Failed to read firmware cogging ratio")

        ratio = main_ratio["value_f"]
        step = cogging_ratio["value_f"]
        expected = 1.0 / (MAP_SIZE * ratio)
        print(
            f"Anticogging preflight: main_ratio={ratio:.6g}, "
            f"step={step:.9g} output turn/index, "
            f"sweep={step * MAP_SIZE * 360.0:.3f} deg"
        )
        if not math.isfinite(ratio) or ratio <= 0.0:
            raise RuntimeError(f"Invalid vernier main ratio: {ratio}")
        if not math.isclose(step, expected, rel_tol=1e-4, abs_tol=1e-9):
            raise RuntimeError(
                "Firmware still reports the legacy anticogging step. "
                f"Expected {expected:.9g}, got {step:.9g}; flash the rebuilt firmware first."
            )
    finally:
        bus.shutdown()


def save_after_calibration(argv):
    channel = option_value(argv, "--channel", "PCAN_USBBUS1")
    bitrate = int(option_value(argv, "--bitrate", DEFAULT_ARGS["--bitrate"]))
    node_id = int(option_value(argv, "--node-id", "0"))
    extended_id = "--extended-id" in argv
    bus = open_bus(channel, bitrate)
    try:
        print("Saving anticogging map; the board will reboot...")
        response = save_configuration(bus, node_id, extended_id, timeout=2.0)
        if response is None:
            print("Save response timed out after reset; verify persistence after reconnect.")
        elif response["status"] != 0:
            raise RuntimeError(f"save_configuration failed: {response}")
    finally:
        bus.shutdown()


def base_script_path():
    return (
        Path(__file__).resolve().parents[1]
        / "can_m0_abz"
        / "anticogging_calibration.py"
    )


def main():
    argv = sys.argv[1:]
    save_map = "--save" in argv
    argv = [arg for arg in argv if arg != "--save"]
    if "--zero-linear-count" in argv:
        raise RuntimeError("--zero-linear-count is invalid for the absolute vernier coordinate")

    argv = add_defaults(argv)
    if "-h" in argv or "--help" in argv:
        print("MT6826S vernier wrapper defaults to a one-motor-turn sweep. Add --save to persist a completed map.")
        sys.argv = [str(base_script_path()), *argv]
        runpy.run_path(str(base_script_path()), run_name="__main__")
        return 0

    preflight(argv)

    base_script = base_script_path()
    sys.argv = [str(base_script), *argv]
    try:
        runpy.run_path(str(base_script), run_name="__main__")
    except SystemExit as exc:
        code = int(exc.code or 0)
        if code != 0:
            return code

    if save_map:
        save_after_calibration(argv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
