#!/usr/bin/env python3
import argparse
import math
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_controller_modes,
    set_input_torque,
    set_limits,
    set_requested_state,
    wait_heartbeat,
)


def read_optional(bus, args, cmd):
    _, value = request_two_floats(bus, args.node_id, cmd, args.extended_id, timeout=0.05)
    return value


def require_ready(bus, args):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
    if hb is None:
        raise RuntimeError("No heartbeat received")
    print(
        f"Heartbeat: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
        f"axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}"
    )
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError("Heartbeat reports an existing error")

    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
    if status is None:
        raise RuntimeError("GET_AXIS_STATUS_EX timeout")
    print(
        f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
        f"motor_calibrated={status['motor_calibrated']} encoder_ready={status['encoder_ready']} "
        f"axis_error=0x{status['axis_error']:08X}"
    )
    if status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
        raise RuntimeError("Axis is not calibrated/ready")


def wait_for_state(bus, args, target_state, timeout):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if hb is None:
            continue
        last = hb
        print(
            f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )
        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            raise RuntimeError("Heartbeat reports error while waiting for state")
        if hb["axis_state"] == target_state:
            return hb
    raise RuntimeError(f"Timed out waiting for state {target_state}; last heartbeat={last}")


def sample_phase(bus, args, torque_nm, duration, label):
    print(f"{label}: commanding {torque_nm:+.4f} Nm for {duration:.2f}s")
    start = time.monotonic()
    next_print = start
    samples = []
    set_input_torque(bus, args.node_id, torque_nm, args.extended_id)
    while time.monotonic() - start < duration:
        now = time.monotonic()
        if now >= next_print:
            enc = read_optional(bus, args, CMD_GET_ENCODER_ESTIMATES)
            iq = read_optional(bus, args, CMD_GET_IQ)
            bus_vi = read_optional(bus, args, CMD_GET_BUS_VOLTAGE_CURRENT)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)

            pos = enc[0] if enc else float("nan")
            vel = enc[1] if enc else float("nan")
            iq_set = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            vbus = bus_vi[0] if bus_vi else float("nan")
            ibus = bus_vi[1] if bus_vi else float("nan")
            samples.append((now - start, torque_nm, pos, vel, iq_set, iq_meas, vbus, ibus))

            hb_tail = ""
            if hb:
                hb_tail = f" state={hb['axis_state']} err=0x{hb['axis_error']:08X}"
                if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
                    raise RuntimeError("Heartbeat reports error during torque test")

            print(
                f"  t={now - start:4.2f}s pos={pos: .5f} vel={vel: .5f} "
                f"Iq={iq_set: .3f}/{iq_meas: .3f} bus={vbus: .2f}V/{ibus: .3f}A{hb_tail}"
            )
            next_print = now + args.print_period
        time.sleep(0.01)
    return samples


def finite_values(samples, index):
    return [row[index] for row in samples if math.isfinite(row[index])]


def validate(samples, args):
    nonzero = [s for s in samples if abs(s[1]) >= args.torque * 0.5]
    zero = [s for s in samples if abs(s[1]) < args.torque * 0.1]
    velocities = [abs(v) for v in finite_values(samples, 3)]
    iq_set_nonzero = [abs(s[4]) for s in nonzero if math.isfinite(s[4])]
    iq_set_zero = [abs(s[4]) for s in zero if math.isfinite(s[4])]
    vbus = finite_values(samples, 6)

    if velocities and max(velocities) > args.max_velocity:
        raise RuntimeError(f"Velocity exceeded limit: {max(velocities):.4f} turns/s > {args.max_velocity}")
    if iq_set_nonzero and max(iq_set_nonzero) < args.min_nonzero_iq:
        raise RuntimeError(f"Nonzero torque did not produce enough Iq_set: max {max(iq_set_nonzero):.4f} A")
    if iq_set_zero and max(iq_set_zero) > args.max_zero_iq:
        raise RuntimeError(f"Zero torque did not settle Iq_set: max {max(iq_set_zero):.4f} A")
    if vbus and (min(vbus) < args.min_bus_voltage or max(vbus) > args.max_bus_voltage):
        raise RuntimeError(f"Bus voltage out of range: {min(vbus):.3f} .. {max(vbus):.3f} V")

    print("Summary:")
    if velocities:
        print(f"  max |velocity|: {max(velocities):.5f} turns/s")
    if iq_set_nonzero:
        print(f"  max |Iq_set| during nonzero torque: {max(iq_set_nonzero):.5f} A")
    if iq_set_zero:
        print(f"  max |Iq_set| during zero torque: {max(iq_set_zero):.5f} A")
    if vbus:
        print(f"  bus voltage: {min(vbus):.3f} .. {max(vbus):.3f} V")


def main():
    parser = argparse.ArgumentParser(description="Stage A: conservative TORQUE_CONTROL + PASSTHROUGH test.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument(
        "--torque",
        type=float,
        default=0.84,
        help="Positive/negative output-shaft torque command in Nm. Vernier firmware converts this to motor torque internally.",
    )
    parser.add_argument("--duration", type=float, default=1.5, help="Duration of each torque phase.")
    parser.add_argument("--settle-duration", type=float, default=1.0, help="Duration of each zero-torque phase.")
    parser.add_argument("--vel-limit", type=float, default=0.5)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--max-velocity", type=float, default=1.0)
    parser.add_argument("--min-nonzero-iq", type=float, default=0.05)
    parser.add_argument("--max-zero-iq", type=float, default=0.08)
    parser.add_argument("--min-bus-voltage", type=float, default=20.0)
    parser.add_argument("--max-bus-voltage", type=float, default=30.0)
    parser.add_argument("--print-period", type=float, default=0.25)
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    all_samples = []
    try:
        print(f"Opened {bus.channel_info}")
        require_ready(bus, args)

        print("Preparing TORQUE_CONTROL + PASSTHROUGH...")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.1)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_controller_modes(bus, args.node_id, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
        set_input_torque(bus, args.node_id, 0.0, args.extended_id)

        print("Requesting CLOSED_LOOP_CONTROL...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout=5.0)

        phases = [
            (0.0, args.settle_duration, "zero-before"),
            (+args.torque, args.duration, "positive"),
            (0.0, args.settle_duration, "zero-middle"),
            (-args.torque, args.duration, "negative"),
            (0.0, args.settle_duration, "zero-after"),
        ]
        for torque, duration, label in phases:
            all_samples.extend(sample_phase(bus, args, torque, duration, label))

        print("Stopping motor and returning to IDLE...")
        set_input_torque(bus, args.node_id, 0.0, args.extended_id)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        final_hb = wait_for_state(bus, args, AXIS_STATE_IDLE, timeout=3.0)

        validate(all_samples, args)

        final_status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
        print(f"Final heartbeat: {final_hb}")
        print(f"Final status: {final_status}")
        if final_status is None or final_status["axis_error"]:
            raise RuntimeError("Final status reports axis error")
        print("PASS: torque mode test completed")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        set_input_torque(bus, args.node_id, 0.0, args.extended_id)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
