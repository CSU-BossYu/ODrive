#!/usr/bin/env python3
"""Move the MT6826S vernier output shaft by a relative angle from its current
position.  A single-step position-loop test: read the current output-shaft
position, command a relative move, hold, and (optionally) return home.

The controller coordinate is the output shaft (turns), so --angle-deg is an
output-shaft angle.  Defaults are conservative; override with CLI flags.
"""
import argparse
import math
import struct
import time

from common import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_POSITION_CONTROL,
    clear_errors,
    get_status_ex,
    open_bus,
    request_two_floats,
    send,
    set_controller_modes,
    set_input_pos,
    set_limits,
    set_pos_gain,
    set_requested_state,
    set_vel_gains,
    wait_heartbeat,
)

from run_mt6826s_velocity_loop import (
    format_axis_error,
    fmt_diag_value,
    print_error_summary,
    read_error_summary,
    read_vernier_diag,
)

CMD_SET_TRAJ_VEL_LIMIT = 0x011
CMD_SET_TRAJ_ACCEL_LIMITS = 0x012
INPUT_MODE_TRAP_TRAJ = 5


def turns_to_deg(turns):
    return turns * 360.0


def deg_to_turns(deg):
    return deg / 360.0


def sign(value, deadband=1e-5):
    if value > deadband:
        return "+"
    if value < -deadband:
        return "-"
    return "0"


def trajectory_timeout(distance, move_vel, move_accel):
    distance = abs(distance)
    accel_distance = move_vel * move_vel / move_accel
    if distance <= accel_distance:
        motion_time = 2.0 * math.sqrt(distance / move_accel)
    else:
        motion_time = distance / move_vel + move_vel / move_accel
    return motion_time + 1.0


def set_traj_limits(bus, args):
    send(bus, args.node_id, CMD_SET_TRAJ_VEL_LIMIT,
         struct.pack("<f", args.move_vel), args.extended_id)
    send(bus, args.node_id, CMD_SET_TRAJ_ACCEL_LIMITS,
         struct.pack("<ff", args.move_accel, args.move_accel), args.extended_id)


def read_pair(bus, args, cmd, timeout=0.15):
    _, value = request_two_floats(bus, args.node_id, cmd, args.extended_id, timeout=timeout)
    return value


def live_position(bus, args):
    diag = read_vernier_diag(bus, args)
    pos = diag.get("diag_pos")
    if pos is not None and math.isfinite(pos):
        return pos
    enc = read_pair(bus, args, CMD_GET_ENCODER_ESTIMATES)
    if enc is None:
        raise RuntimeError("No encoder estimates")
    return enc[0]


def latest_heartbeat(bus, args, timeout=0.8):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.15)
        if hb is not None:
            last = hb
    return last


def require_ready(bus, args):
    hb = latest_heartbeat(bus, args, timeout=2.0)
    if hb is None:
        raise RuntimeError("No heartbeat received")
    print(f"Heartbeat: raw={hb['raw']} state={hb['axis_state']} "
          f"axis_error={format_axis_error(hb['axis_error'])}")

    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
    if status is None:
        raise RuntimeError("GET_AXIS_STATUS_EX timeout")
    print(f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
          f"motor_calibrated={status['motor_calibrated']} "
          f"encoder_ready={status['encoder_ready']} "
          f"axis_error={format_axis_error(status['axis_error'])}")
    if status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
        print_error_summary("Errors before abort:", read_error_summary(bus, args))
        raise RuntimeError("Axis is not calibrated/ready or has an error")


def wait_for_state(bus, args, target_state, timeout):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if hb is None:
            continue
        last = hb
        print(f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
              f"axis_error={format_axis_error(hb['axis_error'])} flags="
              f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}")
        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            print_error_summary("Errors while waiting for state:", read_error_summary(bus, args))
            raise RuntimeError("Heartbeat reports error while waiting for state")
        if hb["axis_state"] == target_state:
            return hb
    raise RuntimeError(f"Timed out waiting for state {target_state}; last heartbeat={last}")


def sample_once(bus, args, target, home, phase_start, label, verbose=False):
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
    err = target - pos if math.isfinite(pos) else float("nan")
    d_phase = pos - phase_start if math.isfinite(pos) else float("nan")
    d_home = pos - home if math.isfinite(pos) else float("nan")

    hb_tail = ""
    if hb is not None:
        hb_tail = (f" state={hb['axis_state']} err={format_axis_error(hb['axis_error'])} "
                   f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}")

    print(f"  pos={pos: .6f} target={target: .6f} err={err: .6f} "
          f"dphase={d_phase: .6f}({turns_to_deg(d_phase): .1f}deg {sign(d_phase)}) "
          f"dhome={d_home: .6f}({turns_to_deg(d_home): .1f}deg) "
          f"vel={vel: .5f} Iq={iq_set: .3f}/{iq_meas: .3f} "
          f"bus={vbus: .2f}V/{ibus: .4f}A{hb_tail}")

    if verbose:
        diag = read_vernier_diag(bus, args)
        print("      diag "
              f"pair={fmt_diag_value(diag['pair_seq'], 'd')} "
              f"valid={fmt_diag_value(diag['pair_valid'], 'd')} "
              f"state={fmt_diag_value(diag['resolver_state'], 'd')} "
              f"vern={fmt_diag_value(diag['vernier_pos'], '.6f')} "
              f"enc={fmt_diag_value(diag['diag_pos'], '.6f')} "
              f"v={fmt_diag_value(diag['diag_vel'], '.6f')} "
              f"busy/ok={fmt_diag_value(diag['pair_busy'], 'd')}/{fmt_diag_value(diag['pair_ok'], 'd')}")

    if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
        print_error_summary("Errors:", read_error_summary(bus, args))
        raise RuntimeError(f"Error observed during {label}")

    command_delta = target - phase_start
    wrong_dir_limit = deg_to_turns(args.abort_wrong_direction_deg)
    if (math.isfinite(d_phase) and abs(command_delta) > 1e-6
            and abs(d_phase) > wrong_dir_limit and command_delta * d_phase < 0.0):
        raise RuntimeError(f"Abort: moved {turns_to_deg(d_phase):+.2f} deg opposite the commanded direction")
    if math.isfinite(d_home) and abs(d_home) > args.abort_displacement:
        raise RuntimeError(f"Abort: displacement from home {d_home:.6f} exceeds {args.abort_displacement}")
    if math.isfinite(iq_set) and abs(iq_set) > args.abort_iq_set:
        raise RuntimeError(f"Abort: |Iq_set| {abs(iq_set):.3f} exceeds {args.abort_iq_set}")

    return pos, vel, err, iq_set, vbus, ibus


def run_move(bus, args, target, home, label, timeout):
    phase_start = live_position(bus, args)
    set_input_pos(bus, args.node_id, target, 0.0, 0.0, args.extended_id)
    print(f"{label}: target={target:+.6f} turns ({turns_to_deg(target - home):+.2f} deg from home) "
          f"for {timeout:.2f}s")

    start = time.monotonic()
    next_print = start
    samples = []
    while time.monotonic() - start < timeout:
        now = time.monotonic()
        if now >= next_print:
            samples.append(sample_once(bus, args, target, home, phase_start, label, args.verbose))
            next_print = now + args.sample_period
        time.sleep(0.01)
    final_row = sample_once(bus, args, target, home, phase_start, label, args.verbose)
    samples.append(final_row)
    return samples, final_row


def finite(values):
    return [v for v in values if math.isfinite(v)]


def summarize(all_samples):
    positions = finite([row[0] for row in all_samples])
    velocities = finite([row[1] for row in all_samples])
    errors = finite([row[2] for row in all_samples])
    iq_set = finite([row[3] for row in all_samples])
    vbus = finite([row[4] for row in all_samples])
    ibus = finite([row[5] for row in all_samples])

    print("Summary:")
    if positions:
        print(f"  position range: {min(positions):+.6f} .. {max(positions):+.6f} turns "
              f"({turns_to_deg(min(positions)):+.1f} .. {turns_to_deg(max(positions)):+.1f} deg)")
    if velocities:
        print(f"  max |velocity|: {max(abs(v) for v in velocities):.5f} turns/s")
    if errors:
        print(f"  max |position error|: {max(abs(e) for e in errors):.6f} turns")
    if iq_set:
        print(f"  max |Iq_set|: {max(abs(i) for i in iq_set):.5f} A")
    if vbus and ibus:
        print(f"  bus voltage: {min(vbus):.3f} .. {max(vbus):.3f} V; "
              f"bus current: {min(ibus):.5f} .. {max(ibus):.5f} A")


def main():
    parser = argparse.ArgumentParser(description="Move MT6826S vernier output shaft by a relative angle.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--angle-deg", type=float, default=5.0, help="Relative output-shaft angle in degrees.")
    parser.add_argument("--step", type=float, default=None,
                        help="Relative output-shaft step in turns. Overrides --angle-deg.")
    parser.add_argument("--direction", choices=("clockwise", "counterclockwise"), default="clockwise")
    parser.add_argument("--clockwise-sign", type=float, choices=(-1.0, 1.0), default=1.0,
                        help="Coordinate sign for a visually clockwise output-shaft move.")
    parser.add_argument("--move-vel", type=float, default=0.05,
                        help="Trapezoidal trajectory cruise speed in output turns/s.")
    parser.add_argument("--move-accel", type=float, default=0.1,
                        help="Trapezoidal trajectory acceleration in output turns/s^2.")
    parser.add_argument("--vel-limit", type=float, default=0.5,
                        help="Controller overspeed/absolute velocity limit in output turns/s.")
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--pos-gain", type=float, default=0.5)
    parser.add_argument("--vel-gain", type=float, default=1.0)
    parser.add_argument("--vel-integrator-gain", type=float, default=0.05)
    parser.add_argument("--settle-duration", type=float, default=0.8, help="Hold time at target after trajectory.")
    parser.add_argument("--sample-period", type=float, default=0.1)
    parser.add_argument("--position-tolerance-deg", type=float, default=1.0,
                        help="Required final position accuracy.")
    parser.add_argument("--abort-wrong-direction-deg", type=float, default=2.0,
                        help="Abort after this much motion opposite the commanded direction.")
    parser.add_argument("--abort-displacement", type=float, default=0.25,
                        help="Abort if displacement from home exceeds this (turns).")
    parser.add_argument("--abort-iq-set", type=float, default=2.5)
    parser.add_argument("--enter-timeout", type=float, default=5.0)
    parser.add_argument("--return-home", action="store_true", help="Return to the start position after the move.")
    parser.add_argument("--verbose", action="store_true", help="Print vernier diagnostics each sample.")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    if not math.isfinite(args.vel_limit) or args.vel_limit <= 0.0:
        raise ValueError("--vel-limit must be positive and finite")
    if not math.isfinite(args.move_vel) or args.move_vel <= 0.0:
        raise ValueError("--move-vel must be positive and finite")
    if not math.isfinite(args.move_accel) or args.move_accel <= 0.0:
        raise ValueError("--move-accel must be positive and finite")
    if args.move_vel >= args.vel_limit:
        raise ValueError("--move-vel must be lower than the controller --vel-limit")
    if not math.isfinite(args.current_limit) or args.current_limit <= 0.0:
        raise ValueError("--current-limit must be positive and finite")
    if not math.isfinite(args.position_tolerance_deg) or args.position_tolerance_deg <= 0.0:
        raise ValueError("--position-tolerance-deg must be positive and finite")

    if args.step is None:
        step = deg_to_turns(abs(args.angle_deg)) * args.clockwise_sign
    else:
        step = args.step
    if args.direction == "counterclockwise":
        step = -step

    bus = open_bus(args.channel, args.bitrate)
    motion_started = False
    all_samples = []
    try:
        print(f"Opened {bus.channel_info}")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.2)
        require_ready(bus, args)

        print("Preparing position loop...")
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_traj_limits(bus, args)
        set_pos_gain(bus, args.node_id, args.pos_gain, args.extended_id)
        set_vel_gains(bus, args.node_id, args.vel_gain, args.vel_integrator_gain, args.extended_id)
        set_controller_modes(bus, args.node_id, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ, args.extended_id)

        home = live_position(bus, args)
        print(f"Current position (home): {home:+.6f} turns ({turns_to_deg(home):+.1f} deg absolute)")
        set_input_pos(bus, args.node_id, home, 0.0, 0.0, args.extended_id)

        print("Requesting CLOSED_LOOP_CONTROL...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout=args.enter_timeout)
        motion_started = True

        target = home + step
        move_timeout = trajectory_timeout(step, args.move_vel, args.move_accel)
        all_samples.extend(run_move(bus, args, target, home, "move",
                                    move_timeout + args.settle_duration)[0])

        final_row = all_samples[-1]
        final_err = target - final_row[0]
        tolerance = deg_to_turns(args.position_tolerance_deg)
        if not math.isfinite(final_err) or abs(final_err) > tolerance:
            raise RuntimeError(f"Final position error {turns_to_deg(final_err):+.2f} deg exceeds "
                               f"{args.position_tolerance_deg:.2f} deg tolerance")
        print(f"Reached target within tolerance: err={turns_to_deg(final_err):+.2f} deg")

        if args.return_home:
            ret_timeout = trajectory_timeout(home - target, args.move_vel, args.move_accel)
            all_samples.extend(run_move(bus, args, home, home, "return-home",
                                        ret_timeout + args.settle_duration)[0])
            ret_err = home - all_samples[-1][0]
            if not math.isfinite(ret_err) or abs(ret_err) > tolerance:
                raise RuntimeError(f"Return-home error {turns_to_deg(ret_err):+.2f} deg exceeds tolerance")

        print("Stopping motor and returning to IDLE...")
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        wait_for_state(bus, args, AXIS_STATE_IDLE, timeout=3.0)

        summarize(all_samples)
        print_error_summary("Final errors:", read_error_summary(bus, args))
        print("PASS: position step completed")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        try:
            set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            if args.clear_at_end:
                time.sleep(0.2)
                clear_errors(bus, args.node_id, args.extended_id)
        except Exception as ex:
            print(f"WARNING: failed to stop during cleanup: {ex}")
        raise
    finally:
        if motion_started:
            try:
                set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            except Exception:
                pass
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
