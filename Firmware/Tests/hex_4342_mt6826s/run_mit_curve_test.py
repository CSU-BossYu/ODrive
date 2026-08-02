#!/usr/bin/env python3
"""
MIT smooth-curve tracking test.

Streams MIT frames along a quintic minimum-jerk position curve. The command is
small by default and is intended to validate MIT position/velocity units before
trying large moves.
"""

import argparse
import csv
import math
import os
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_TORQUE_CONTROL,
    clear_errors,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_controller_modes,
    set_limits,
    set_linear_count,
    set_requested_state,
    wait_heartbeat,
)
from run_mit_mode_link_test import (
    INPUT_MODE_MIT,
    P_MAX,
    P_MIN,
    pack_mit_frame,
    pack_neutral_mit_frame,
    send_mit_frame,
)


def check_heartbeat_clean(hb, context):
    if hb is None:
        raise RuntimeError(f"{context}: heartbeat timeout")
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError(
            f"{context}: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} "
            f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )


def read_encoder(bus, args, timeout=0.2):
    _, enc = request_two_floats(
        bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=timeout)
    if enc is None:
        return None, None
    return enc


def read_iq(bus, args, timeout=0.05):
    _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=timeout)
    if iq is None:
        return float("nan"), float("nan")
    return iq


def wait_closed_loop(bus, args, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        send_mit_frame(bus, args, pack_neutral_mit_frame(0))
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.15)
        if hb is None:
            continue
        check_heartbeat_clean(hb, "enter closed loop")
        if hb["axis_state"] == AXIS_STATE_CLOSED_LOOP_CONTROL:
            return
    raise RuntimeError("Timed out entering CLOSED_LOOP_CONTROL")


def smoothstep5(u):
    u = max(0.0, min(1.0, u))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def smoothstep5_d(u):
    u = max(0.0, min(1.0, u))
    return 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4


def command_at(t, waypoints_rad, segment_s):
    if t <= 0.0:
        return waypoints_rad[0], 0.0
    total_s = segment_s * (len(waypoints_rad) - 1)
    if t >= total_s:
        return waypoints_rad[-1], 0.0

    idx = int(t // segment_s)
    local_t = t - idx * segment_s
    u = local_t / segment_s
    p0 = waypoints_rad[idx]
    p1 = waypoints_rad[idx + 1]
    dp = p1 - p0
    pos = p0 + dp * smoothstep5(u)
    vel = dp * smoothstep5_d(u) / segment_s
    return pos, vel


def parse_waypoints_deg(text):
    values = []
    for part in text.replace(";", ",").split(","):
        part = part.strip()
        if part:
            values.append(float(part))
    if len(values) < 2:
        raise argparse.ArgumentTypeError("need at least two comma-separated waypoints")
    return values


def main():
    parser = argparse.ArgumentParser(description="MIT smooth-curve tracking test")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--waypoints-deg", type=parse_waypoints_deg,
                        default=parse_waypoints_deg("0,5,10,5,0,-5,0"),
                        help="Relative output-shaft targets in deg, comma-separated.")
    parser.add_argument("--segment", type=float, default=2.0,
                        help="Seconds between two adjacent waypoints.")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="MIT stream rate in Hz.")
    parser.add_argument("--kp", type=float, default=0.5,
                        help="MIT stiffness, Nm/rad if firmware follows AK/T-Motor convention.")
    parser.add_argument("--kd", type=float, default=0.02,
                        help="MIT damping, Nm/(rad/s) if firmware follows AK/T-Motor convention.")
    parser.add_argument("--torque-ff", type=float, default=0.0)
    parser.add_argument("--vel-limit", type=float, default=1.0,
                        help="Torque-mode velocity safety limit in output turns/s.")
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--zero-linear-count", action="store_true",
                        help="Set encoder linear count to 0 before the MIT test.")
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    parser.add_argument("--csv", default="")
    parser.add_argument("--plot", default="",
                        help="Save a PNG plot. Default: next to CSV as mit_curve_trace.png.")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    args.period = 1.0 / args.rate

    bus = open_bus(args.channel, args.bitrate)
    rows = []
    try:
        print(f"Opened {bus.channel_info}")
        if args.clear_at_start:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        check_heartbeat_clean(hb, "initial")
        print(f"Heartbeat: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
        if status is not None:
            print(
                f"Status: motor_calibrated={status['motor_calibrated']} "
                f"encoder_ready={status['encoder_ready']} axis_error=0x{status['axis_error']:08X}"
            )
            if status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
                raise RuntimeError("Axis is not calibrated/ready")

        if args.zero_linear_count:
            print("Setting linear count to 0 for MIT command range...")
            set_linear_count(bus, args.node_id, 0, args.extended_id)
            time.sleep(0.2)

        pos_turns, _ = read_encoder(bus, args, timeout=1.0)
        if pos_turns is None:
            raise RuntimeError("No encoder estimates")
        zero_rad = pos_turns * 2.0 * math.pi

        rel_waypoints_rad = [math.radians(x) for x in args.waypoints_deg]
        abs_waypoints_rad = [zero_rad + x for x in rel_waypoints_rad]
        cmd_min = min(abs_waypoints_rad)
        cmd_max = max(abs_waypoints_rad)
        margin = 0.1
        if cmd_min < P_MIN + margin or cmd_max > P_MAX - margin:
            raise RuntimeError(
                f"MIT command range [{cmd_min:.3f}, {cmd_max:.3f}] rad exceeds "
                f"[{P_MIN + margin:.3f}, {P_MAX - margin:.3f}] rad. "
                "Use --zero-linear-count or smaller/nearer waypoints."
            )

        print(
            f"MIT curve: waypoints={args.waypoints_deg} deg, segment={args.segment:.3f}s, "
            f"rate={args.rate:.1f}Hz, kp={args.kp:.4f}, kd={args.kd:.4f}"
        )
        print(f"Software zero: {zero_rad:.6f} rad ({pos_turns:.6f} turns)")

        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.1)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_controller_modes(
            bus, args.node_id, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_MIT, args.extended_id)

        print("Preloading neutral MIT frames...")
        for i in range(30):
            send_mit_frame(bus, args, pack_neutral_mit_frame(i))
            time.sleep(min(args.period, 0.01))

        print("Requesting CLOSED_LOOP_CONTROL...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_closed_loop(bus, args, timeout=5.0)

        total_s = args.segment * (len(abs_waypoints_rad) - 1)
        start = time.monotonic()
        next_send = start
        next_print = start
        frame_count = 0
        print("Streaming MIT curve...")
        while True:
            now = time.monotonic()
            t = now - start
            if t > total_s:
                break
            if now < next_send:
                time.sleep(min(next_send - now, 0.001))
                continue

            p_cmd, v_cmd = command_at(t, abs_waypoints_rad, args.segment)
            send_mit_frame(
                bus, args,
                pack_mit_frame(p_cmd, v_cmd, args.kp, args.kd, args.torque_ff))
            frame_count += 1
            next_send += args.period
            if next_send < now:
                next_send = now + args.period

            pos_turns, vel_turns_s = read_encoder(bus, args, timeout=0.001)
            iq_set, iq_meas = read_iq(bus, args, timeout=0.001)
            if pos_turns is not None:
                pos_rad = pos_turns * 2.0 * math.pi
                err_rad = pos_rad - p_cmd
                rows.append({
                    "t": t,
                    "p_cmd_rad": p_cmd,
                    "v_cmd_rad_s": v_cmd,
                    "p_meas_rad": pos_rad,
                    "v_meas_rad_s": vel_turns_s * 2.0 * math.pi,
                    "err_deg": math.degrees(err_rad),
                    "iq_set": iq_set,
                    "iq_meas": iq_meas,
                })

            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.0)
            if hb is not None:
                check_heartbeat_clean(hb, "MIT curve")

            if now >= next_print:
                last_err = rows[-1]["err_deg"] if rows else float("nan")
                print(f"  t={t:5.2f}s p_cmd={math.degrees(p_cmd - zero_rad):7.3f}deg "
                      f"v_cmd={v_cmd:7.4f}rad/s err={last_err:8.4f}deg frames={frame_count}")
                next_print = now + 0.5

        print("Holding final point briefly...")
        final_pos = abs_waypoints_rad[-1]
        for i in range(max(1, int(args.rate * 0.5))):
            send_mit_frame(bus, args, pack_mit_frame(final_pos, 0.0, args.kp, args.kd, args.torque_ff))
            time.sleep(args.period)

        if rows:
            err = [r["err_deg"] for r in rows]
            rms = math.sqrt(sum(x * x for x in err) / len(err))
            max_abs = max(abs(x) for x in err)
            mean = sum(err) / len(err)
            print(f"Result: mean_err={mean:.4f}deg rms_err={rms:.4f}deg max_abs_err={max_abs:.4f}deg samples={len(rows)}")
        print(f"MIT frames sent: {frame_count}")

        out_csv = args.csv
        if not out_csv:
            out_csv = os.path.join(os.path.dirname(__file__) or ".", "mit_curve_trace.csv")
        if rows:
            with open(out_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
            print(f"Trace saved to: {out_csv}")
            if not args.no_plot:
                out_plot = args.plot
                if not out_plot:
                    out_plot = os.path.splitext(out_csv)[0] + ".png"
                plot_trace(rows, zero_rad, out_plot)

        print("Returning to IDLE...")
        for i in range(10):
            send_mit_frame(bus, args, pack_neutral_mit_frame(i))
            time.sleep(min(args.period, 0.01))
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        for i in range(10):
            send_mit_frame(bus, args, pack_neutral_mit_frame(i))
            time.sleep(0.005)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


def plot_trace(rows, zero_rad, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skip plot")
        return

    t = [r["t"] for r in rows]
    p_cmd = [math.degrees(r["p_cmd_rad"] - zero_rad) for r in rows]
    p_meas = [math.degrees(r["p_meas_rad"] - zero_rad) for r in rows]
    v_cmd = [r["v_cmd_rad_s"] for r in rows]
    v_meas = [r["v_meas_rad_s"] for r in rows]
    err = [r["err_deg"] for r in rows]
    iq_set = [r["iq_set"] for r in rows]
    iq_meas = [r["iq_meas"] for r in rows]

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    axes[0].plot(t, p_cmd, "k--", lw=1.0, label="p_des")
    axes[0].plot(t, p_meas, "#2563eb", lw=1.0, label="position")
    axes[0].set_ylabel("pos (deg)")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, err, "#dc2626", lw=1.0, label="error")
    axes[1].axhline(0.0, color="k", lw=0.8, alpha=0.4)
    axes[1].set_ylabel("err (deg)")
    axes[1].legend(loc="upper left")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, v_cmd, "k--", lw=1.0, label="v_des")
    axes[2].plot(t, v_meas, "#16a34a", lw=1.0, label="velocity")
    axes[2].set_ylabel("vel (rad/s)")
    axes[2].legend(loc="upper left")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, iq_set, "#7c3aed", lw=1.0, label="Iq_set")
    axes[3].plot(t, iq_meas, "#f97316", lw=1.0, label="Iq_meas")
    axes[3].set_xlabel("time (s)")
    axes[3].set_ylabel("Iq (A)")
    axes[3].legend(loc="upper left")
    axes[3].grid(True, alpha=0.3)

    fig.suptitle("MIT smooth-curve tracking")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Plot saved to: {out_path}")


if __name__ == "__main__":
    raise SystemExit(main())
