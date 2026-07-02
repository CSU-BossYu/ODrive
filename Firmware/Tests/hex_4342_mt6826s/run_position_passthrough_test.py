#!/usr/bin/env python3
import argparse
import math
import time

from common import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_controller_modes,
    set_input_pos,
    set_limits,
    set_pos_gain,
    set_requested_state,
    set_vel_gains,
    wait_heartbeat,
)


def read_pair(bus, args, cmd):
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


def sample_target(bus, args, target_pos, duration, label):
    print(f"{label}: commanding position {target_pos:+.6f} turns for {duration:.2f}s")
    set_input_pos(bus, args.node_id, target_pos, 0.0, 0.0, args.extended_id)
    start = time.monotonic()
    next_print = start
    samples = []

    while time.monotonic() - start < duration:
        now = time.monotonic()
        if now >= next_print:
            enc = read_pair(bus, args, CMD_GET_ENCODER_ESTIMATES)
            iq = read_pair(bus, args, CMD_GET_IQ)
            bus_vi = read_pair(bus, args, CMD_GET_BUS_VOLTAGE_CURRENT)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)

            pos = enc[0] if enc else float("nan")
            vel = enc[1] if enc else float("nan")
            iq_set = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            vbus = bus_vi[0] if bus_vi else float("nan")
            ibus = bus_vi[1] if bus_vi else float("nan")
            pos_err = target_pos - pos if math.isfinite(pos) else float("nan")
            samples.append((now - start, target_pos, pos, vel, pos_err, iq_set, iq_meas, vbus, ibus))

            hb_tail = ""
            if hb:
                hb_tail = f" state={hb['axis_state']} err=0x{hb['axis_error']:08X}"
                if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
                    raise RuntimeError("Heartbeat reports error during position test")

            print(
                f"  t={now - start:4.2f}s pos={pos: .6f} err={pos_err: .6f} "
                f"vel={vel: .5f} Iq={iq_set: .3f}/{iq_meas: .3f} "
                f"bus={vbus: .2f}V/{ibus: .3f}A{hb_tail}"
            )
            next_print = now + args.print_period
        time.sleep(0.01)

    return samples


def finite_values(samples, index):
    return [row[index] for row in samples if math.isfinite(row[index])]


def validate(samples, args):
    velocities = [abs(v) for v in finite_values(samples, 3)]
    pos_errors = [abs(e) for e in finite_values(samples, 4)]
    iq_set = [abs(i) for i in finite_values(samples, 5)]
    vbus = finite_values(samples, 7)

    if velocities and max(velocities) > args.max_velocity:
        raise RuntimeError(f"Velocity exceeded limit: {max(velocities):.5f} turns/s > {args.max_velocity}")
    if pos_errors and max(pos_errors) > args.max_position_error:
        raise RuntimeError(f"Position error exceeded limit: {max(pos_errors):.6f} turns > {args.max_position_error}")
    if iq_set and max(iq_set) > args.max_iq_set:
        raise RuntimeError(f"Iq_set exceeded limit: {max(iq_set):.5f} A > {args.max_iq_set}")
    if vbus and (min(vbus) < args.min_bus_voltage or max(vbus) > args.max_bus_voltage):
        raise RuntimeError(f"Bus voltage out of range: {min(vbus):.3f} .. {max(vbus):.3f} V")

    print("Summary:")
    if velocities:
        print(f"  max |velocity|: {max(velocities):.5f} turns/s")
    if pos_errors:
        print(f"  max |position error|: {max(pos_errors):.6f} turns")
    if iq_set:
        print(f"  max |Iq_set|: {max(iq_set):.5f} A")
    if vbus:
        print(f"  bus voltage: {min(vbus):.3f} .. {max(vbus):.3f} V")


def main():
    parser = argparse.ArgumentParser(description="Stage C: conservative POSITION_CONTROL + PASSTHROUGH test.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--step", type=float, default=0.01, help="Relative position step in turns.")
    parser.add_argument("--duration", type=float, default=1.2, help="Duration at each nonzero target.")
    parser.add_argument("--settle-duration", type=float, default=0.8, help="Duration at home/current target.")
    parser.add_argument("--pos-gain", type=float, default=2.0)
    parser.add_argument("--vel-gain", type=float, default=42.0, help="Output-shaft torque per output turn/s.")
    parser.add_argument("--vel-integrator-gain", type=float, default=2.1, help="Output-shaft torque per output turn/s/s.")
    parser.add_argument("--vel-limit", type=float, default=0.1)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--max-velocity", type=float, default=0.2)
    parser.add_argument("--max-position-error", type=float, default=0.05)
    parser.add_argument("--max-iq-set", type=float, default=2.0)
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
        enc = read_pair(bus, args, CMD_GET_ENCODER_ESTIMATES)
        if enc is None:
            raise RuntimeError("No encoder estimates")
        home = enc[0]
        print(f"Home position: {home:+.6f} turns")

        print("Preparing POSITION_CONTROL + PASSTHROUGH...")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.1)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_pos_gain(bus, args.node_id, args.pos_gain, args.extended_id)
        set_vel_gains(bus, args.node_id, args.vel_gain, args.vel_integrator_gain, args.extended_id)
        set_controller_modes(bus, args.node_id, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
        set_input_pos(bus, args.node_id, home, 0.0, 0.0, args.extended_id)

        print("Requesting CLOSED_LOOP_CONTROL...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout=5.0)

        phases = [
            (home, args.settle_duration, "home-before"),
            (home + args.step, args.duration, "positive-step"),
            (home, args.settle_duration, "home-middle"),
            (home - args.step, args.duration, "negative-step"),
            (home, args.settle_duration, "home-after"),
        ]
        for target, duration, label in phases:
            all_samples.extend(sample_target(bus, args, target, duration, label))

        print("Stopping motor and returning to IDLE...")
        set_input_pos(bus, args.node_id, home, 0.0, 0.0, args.extended_id)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        final_hb = wait_for_state(bus, args, AXIS_STATE_IDLE, timeout=3.0)

        validate(all_samples, args)

        final_status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
        print(f"Final heartbeat: {final_hb}")
        print(f"Final status: {final_status}")
        if final_status is None or final_status["axis_error"]:
            raise RuntimeError("Final status reports axis error")
        print("PASS: position passthrough test completed")
        return 0
    except BaseException:
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
