#!/usr/bin/env python3
import argparse
import time

from common import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_INT32,
    EXT_TYPE_UINT32,
    clear_errors,
    ext_request,
    get_calib_result,
    get_status_ex,
    open_bus,
    save_configuration,
    set_basic_config,
    set_precalibrated,
    set_requested_state,
    wait_heartbeat,
)


def print_status(prefix, status):
    if status is None:
        print(f"{prefix}: TIMEOUT")
        return
    state_name = AXIS_STATES.get(status["axis_state"], "?")
    print(
        f"{prefix}: raw={status['raw']} state={status['axis_state']}({state_name}) "
        f"flags=0x{status['flags']:02X} motor_calibrated={status['motor_calibrated']} "
        f"encoder_ready={status['encoder_ready']} axis_error=0x{status['axis_error']:08X}"
    )


def configure_abz(bus, args):
    print("Writing M0 ABZ base configuration...")
    writes = [
        ("motor_type=HIGH_CURRENT(0)", 0x10, EXT_TYPE_UINT32, 0, None),
        (f"pole_pairs={args.pole_pairs}", 0x11, EXT_TYPE_INT32, args.pole_pairs, None),
        (f"calibration_current={args.calibration_current}", 0x12, EXT_TYPE_FLOAT32, 0, args.calibration_current),
        (f"current_lim={args.current_limit}", 0x14, EXT_TYPE_FLOAT32, 0, args.current_limit),
        (f"torque_constant={args.torque_constant:.6f}", 0x15, EXT_TYPE_FLOAT32, 0, args.torque_constant),
        ("encoder_mode=INCREMENTAL(0)", 0x20, EXT_TYPE_UINT32, 0, None),
        (f"encoder_cpr={args.encoder_cpr}", 0x21, EXT_TYPE_INT32, args.encoder_cpr, None),
    ]
    for label, param_id, req_type, value, value_float in writes:
        resp = set_basic_config(
            bus, args.node_id, param_id, req_type, value=value, value_float=value_float,
            extended_id=args.extended_id,
        )
        if resp is None:
            raise RuntimeError(f"{label}: no response")
        status = EXT_STATUS.get(resp["status"], resp["status"])
        print(f"  {label}: {status} raw={resp['raw']}")
        if resp["status"] != 0:
            raise RuntimeError(f"{label} failed with {status}")


def run_calibration(bus, args):
    print("Clearing errors and requesting FULL_CALIBRATION_SEQUENCE...")
    clear_errors(bus, args.node_id, args.extended_id)
    time.sleep(0.2)
    set_requested_state(bus, args.node_id, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, args.extended_id)

    start = time.monotonic()
    last_state = None
    idle_since = None
    while time.monotonic() - start < args.timeout:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=1.0)
        if hb is None:
            print(f"  {time.monotonic() - start:5.1f}s heartbeat timeout")
            continue

        state = hb["axis_state"]
        if state != last_state:
            print(
                f"  {time.monotonic() - start:5.1f}s state={state}({AXIS_STATES.get(state, '?')}) "
                f"axis_error=0x{hb['axis_error']:08X} flags="
                f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
            )
            last_state = state

        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            raise RuntimeError("Calibration stopped with error flags")

        if state == AXIS_STATE_IDLE:
            if idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > 1.0 and time.monotonic() - start > 2.0:
                return
        else:
            idle_since = None

    raise RuntimeError(f"Calibration did not finish within {args.timeout:.1f}s")


def print_calibration_results(bus, args):
    print("Calibration result readback:")
    for item_id, name in [
        (0x01, "phase_resistance"),
        (0x02, "phase_inductance"),
        (0x03, "phase_offset"),
        (0x04, "direction"),
    ]:
        result = get_calib_result(bus, args.node_id, item_id, args.extended_id)
        if result is None:
            print(f"  {name}: TIMEOUT")
            continue
        resp, value = result
        print(f"  {name}: status={EXT_STATUS.get(resp['status'], resp['status'])} value={value} raw={resp['raw']}")


def main():
    parser = argparse.ArgumentParser(description="Calibrate ODrive M0 with ABZ incremental encoder over CANSimple.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--pole-pairs", type=int, default=10)
    parser.add_argument("--encoder-cpr", type=int, default=4096, help="1024-line ABZ usually uses 1024*4=4096 CPR.")
    parser.add_argument("--calibration-current", type=float, default=3.0)
    parser.add_argument("--current-limit", type=float, default=8.0)
    parser.add_argument("--kv", type=float, default=650.0, help="Motor KV. Used to derive torque_constant if not provided.")
    parser.add_argument("--torque-constant", type=float, default=None, help="Nm/A. Defaults to 8.27 / KV.")
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--save", action="store_true", help="Save configuration after calibration. This reboots the ODrive.")
    args = parser.parse_args()
    if args.torque_constant is None:
        args.torque_constant = 8.27 / args.kv

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat OK: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        configure_abz(bus, args)
        # Clear stale precalibrated runtime flags before measuring fresh values.
        set_precalibrated(bus, args.node_id, flags=0x30, extended_id=args.extended_id)
        run_calibration(bus, args)

        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
        print_status("After calibration", status)
        if status is None or not status["motor_calibrated"] or not status["encoder_ready"] or status["axis_error"]:
            raise RuntimeError("Calibration did not leave M0 calibrated and ready")

        print_calibration_results(bus, args)

        print("Setting motor+encoder pre_calibrated runtime flags...")
        resp = set_precalibrated(bus, args.node_id, flags=0x03, extended_id=args.extended_id)
        if resp is None or resp["status"] != 0:
            raise RuntimeError("SET_PRECALIBRATED failed")
        print(f"  raw={resp['raw']} status={EXT_STATUS.get(resp['status'], resp['status'])}")

        if args.save:
            print("Saving configuration. ODrive will reboot...")
            ack = save_configuration(bus, args.node_id, args.extended_id, timeout=2.0)
            if ack is None:
                print("  save ACK timeout; reboot may have happened before ACK was observed")
            else:
                print(f"  save ACK raw={ack['raw']} status={EXT_STATUS.get(ack['status'], ack['status'])}")
            time.sleep(2.0)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=10.0)
            if hb is None:
                raise RuntimeError("No heartbeat after save/reboot")
            print(f"Heartbeat after reboot: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        print("PASS: calibration completed")
        return 0
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
