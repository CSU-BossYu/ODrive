#!/usr/bin/env python3
"""
Stage B: conservative MIT packed-control link test for MT6826S/vernier firmware.

The test streams AK/T-Motor-compatible 0x01F MIT frames while the controller is
configured as TORQUE_CONTROL + INPUT_MODE_MIT. By default it sends neutral
frames only; optional tiny torque feed-forward can be enabled explicitly.
"""

import argparse
import math
import time

from common import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_TORQUE_CONTROL,
    clear_errors,
    decode_heartbeat,
    get_status_ex,
    open_bus,
    request_two_floats,
    send,
    set_controller_modes,
    set_limits,
    set_precalibrated,
    set_requested_state,
    wait_heartbeat,
)


CMD_SET_MIT_CONTROL = 0x01F
INPUT_MODE_MIT = 9

P_MIN = -12.5
P_MAX = 12.5
V_MIN = -45.0
V_MAX = 45.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0


def clamp(value, low, high):
    return max(low, min(high, value))


def float_to_uint(value, low, high, bits):
    value = clamp(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low) + 0.5)


def pack_mit_frame(pos_rad, vel_rad_s, kp, kd, torque_ff):
    p_int = float_to_uint(pos_rad, P_MIN, P_MAX, 16)
    v_int = float_to_uint(vel_rad_s, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(torque_ff, T_MIN, T_MAX, 12)
    return bytes([
        (p_int >> 8) & 0xFF,
        p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
        kp_int & 0xFF,
        (kd_int >> 4) & 0xFF,
        ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
        t_int & 0xFF,
    ])


def pack_neutral_mit_frame(index):
    # New firmware snaps both adjacent center codes for signed MIT fields to
    # exact zero. Alternating them keeps older firmware average-neutral too.
    center_raw = 2047 if index % 2 else 2048
    torque_raw = center_raw
    pos_raw = 32767 if index % 2 else 32768
    vel_raw = center_raw
    kp_raw = 0
    kd_raw = 0
    return bytes([
        (pos_raw >> 8) & 0xFF,
        pos_raw & 0xFF,
        (vel_raw >> 4) & 0xFF,
        ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F),
        kp_raw & 0xFF,
        (kd_raw >> 4) & 0xFF,
        ((kd_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F),
        torque_raw & 0xFF,
    ])


def send_mit_frame(bus, args, frame):
    send(bus, args.node_id, CMD_SET_MIT_CONTROL, frame, args.extended_id)


def decode_latest_heartbeat_nonblocking(bus, args):
    expected_id = (args.node_id << 5) | 0x001
    latest = None
    # Drain a small bounded amount so monitoring never starves MIT output.
    for _ in range(16):
        msg = bus.recv(0.0)
        if msg is None:
            break
        if msg.arbitration_id != expected_id or msg.is_extended_id != args.extended_id or msg.is_remote_frame:
            continue
        latest = decode_heartbeat(bytes(msg.data))
    return latest


def drain_can_rx(bus, max_frames=128):
    for _ in range(max_frames):
        if bus.recv(0.0) is None:
            break


def wait_for_clean_heartbeat(bus, args, context, timeout=3.0):
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if hb is None:
            continue
        latest = hb
        if (
            hb["axis_error"] == 0
            and not hb["motor_error_flag"]
            and not hb["encoder_error_flag"]
            and not hb["controller_error_flag"]
        ):
            return hb
    check_heartbeat_clean(latest, context)
    return latest


def check_heartbeat_clean(hb, context):
    if hb is None:
        raise RuntimeError(f"{context}: heartbeat timeout")
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError(
            f"{context}: heartbeat reports error state={hb['axis_state']} "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )


def require_ready(bus, args):
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
    if status is None:
        raise RuntimeError("GET_AXIS_STATUS_EX timeout")
    print(
        f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
        f"motor_calibrated={status['motor_calibrated']} encoder_ready={status['encoder_ready']} "
        f"axis_error=0x{status['axis_error']:08X}"
    )
    if args.set_precalibrated_if_needed and (not status["motor_calibrated"] or not status["encoder_ready"]):
        print("Runtime ready flags are not set; sending SET_PRECALIBRATED flags=0x03...")
        resp = set_precalibrated(bus, args.node_id, flags=0x03, extended_id=args.extended_id)
        if resp is None or resp["status"] != 0:
            raise RuntimeError("SET_PRECALIBRATED failed")
        time.sleep(0.2)
        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
        print(f"Status after SET_PRECALIBRATED: raw={status['raw']} flags=0x{status['flags']:02X}")

    if status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
        raise RuntimeError("M0 is not calibrated/ready or has axis error")


def wait_for_state_while_streaming(bus, args, target_state, timeout):
    start = time.monotonic()
    next_send = start
    next_print = start
    count = 0
    last = None
    while time.monotonic() - start < timeout:
        now = time.monotonic()
        if now < next_send:
            time.sleep(min(next_send - now, 0.001))
            continue

        send_mit_frame(bus, args, pack_neutral_mit_frame(count))
        count += 1
        next_send += args.period
        if next_send < now:
            next_send = now + args.period

        hb = decode_latest_heartbeat_nonblocking(bus, args)
        if hb is None:
            continue
        last = hb
        if now >= next_print:
            print(
                f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
                f"axis_error=0x{hb['axis_error']:08X} flags="
                f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
            )
            next_print = now + args.print_period
        check_heartbeat_clean(hb, f"waiting for {AXIS_STATES.get(target_state, target_state)}")
        if hb["axis_state"] == target_state:
            print(
                f"  reached state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
                f"axis_error=0x{hb['axis_error']:08X}"
            )
            return count
    raise RuntimeError(f"Timed out waiting for state {target_state}; last heartbeat={last}")


def stream_mit(bus, args, label, make_frame, duration):
    print(label)
    start = time.monotonic()
    next_print = start
    next_send = start
    count = 0
    latest_hb = None
    while time.monotonic() - start < duration:
        now = time.monotonic()
        if now < next_send:
            time.sleep(min(next_send - now, 0.001))
            continue

        send_mit_frame(bus, args, make_frame(count))
        count += 1
        next_send += args.period
        if next_send < now:
            next_send = now + args.period

        hb = decode_latest_heartbeat_nonblocking(bus, args)
        if hb is not None:
            check_heartbeat_clean(hb, "MIT stream")
            latest_hb = hb

        if now >= next_print:
            hb_tail = "" if latest_hb is None else f" state={latest_hb['axis_state']} err=0x{latest_hb['axis_error']:08X}"
            print(
                f"  t={now - start:4.2f}s frames={count} "
                f"period={args.period * 1000.0:.1f}ms{hb_tail}"
            )
            next_print = now + args.print_period

    _, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=0.2)
    _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=0.2)
    pos = enc[0] if enc else float("nan")
    vel = enc[1] if enc else float("nan")
    iq_set = iq[0] if iq else float("nan")
    iq_meas = iq[1] if iq else float("nan")
    print(f"  post-stream sample: pos={pos: .5f} vel={vel: .5f} Iq_set={iq_set: .3f} Iq_meas={iq_meas: .3f}")
    return count, [(duration, pos, vel, iq_set, iq_meas)]


def final_check(bus, args):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=2.0)
    check_heartbeat_clean(hb, "final")
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
    if status is None:
        raise RuntimeError("Final GET_AXIS_STATUS_EX timeout")
    if status["axis_error"]:
        raise RuntimeError(f"Final status reports axis_error=0x{status['axis_error']:08X}")
    print(f"Final heartbeat: {hb}")
    print(f"Final status: {status}")


def main():
    parser = argparse.ArgumentParser(description="Stage B: conservative MIT packed-control link test.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--period", type=float, default=0.02, help="MIT frame period in seconds.")
    parser.add_argument("--print-period", type=float, default=0.25)
    parser.add_argument(
        "--vel-limit",
        type=float,
        default=20.0,
        help="Torque-mode velocity safety limit in turns/s. Keep high enough to avoid false overspeed during MIT entry.",
    )
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--hold-kp", type=float, default=0.0)
    parser.add_argument("--hold-kd", type=float, default=0.0)
    parser.add_argument("--hold-vel", type=float, default=0.0, help="MIT v_des in rad/s.")
    parser.add_argument("--torque-ff", type=float, default=0.0, help="Small MIT torque feed-forward in Nm.")
    parser.add_argument("--max-iq-set", type=float, default=1.0)
    parser.add_argument(
        "--allow-clamped-hold-position",
        action="store_true",
        help="Allow hold_kp tests even when current absolute position is outside the MIT +/-12.5 rad command window.",
    )
    parser.add_argument("--set-precalibrated-if-needed", action="store_true")
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        if args.clear_at_start:
            print("Clearing existing errors...")
            drain_can_rx(bus)
            clear_errors(bus, args.node_id, args.extended_id)
            hb = wait_for_clean_heartbeat(bus, args, "after clear-at-start", timeout=3.0)
            print(f"Clean heartbeat after clear: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")
            drain_can_rx(bus)

        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        check_heartbeat_clean(hb, "initial")
        print(f"Heartbeat OK: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        require_ready(bus, args)

        _, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=1.0)
        if enc is None:
            raise RuntimeError("No encoder estimates")
        pos_turns, _ = enc
        pos_rad = pos_turns * 2.0 * math.pi
        pos_cmd = clamp(pos_rad, P_MIN + 0.5, P_MAX - 0.5)
        if pos_cmd != pos_rad:
            print(f"Current position {pos_rad:.4f} rad is outside MIT range; command is clamped to {pos_cmd:.4f} rad.")
            if args.hold_kp != 0.0 and not args.allow_clamped_hold_position:
                raise RuntimeError(
                    "Refusing MIT hold_kp test with clamped position. Zero/set the encoder linear count near the joint "
                    "position first, or pass --allow-clamped-hold-position if this large position error is intentional."
                )
        else:
            print(f"Current position: {pos_turns:.5f} turns ({pos_rad:.4f} rad)")

        print("Preparing TORQUE_CONTROL + INPUT_MODE_MIT...")
        drain_can_rx(bus)
        clear_errors(bus, args.node_id, args.extended_id)
        hb = wait_for_clean_heartbeat(bus, args, "after pre-arm clear", timeout=3.0)
        print(f"Clean heartbeat before arm: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")
        drain_can_rx(bus)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_controller_modes(bus, args.node_id, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_MIT, args.extended_id)

        print("Preloading neutral MIT frames in IDLE...")
        for i in range(30):
            send_mit_frame(bus, args, pack_neutral_mit_frame(i))
            time.sleep(min(args.period, 0.01))

        print("Requesting CLOSED_LOOP_CONTROL while continuing to send neutral MIT frames...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_for_state_while_streaming(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout=5.0)

        neutral_count, neutral_samples = stream_mit(
            bus,
            args,
            "Streaming neutral MIT frames...",
            lambda i: pack_neutral_mit_frame(i),
            duration=args.duration,
        )

        hold_count = 0
        hold_samples = []
        if args.hold_kp != 0.0 or args.hold_kd != 0.0 or args.hold_vel != 0.0 or args.torque_ff != 0.0:
            frame = pack_mit_frame(pos_cmd, args.hold_vel, args.hold_kp, args.hold_kd, args.torque_ff)
            print(
                f"Hold frame: pos={pos_cmd:.4f} rad vel={args.hold_vel:.4f} kp={args.hold_kp:.4f} "
                f"kd={args.hold_kd:.4f} torque_ff={args.torque_ff:.4f} raw={frame.hex(' ')}"
            )
            hold_count, hold_samples = stream_mit(
                bus,
                args,
                "Streaming optional MIT hold/torque frames...",
                lambda i: frame,
                duration=args.duration,
            )

        print("Stopping and returning to IDLE...")
        for i in range(10):
            send_mit_frame(bus, args, pack_neutral_mit_frame(i))
            time.sleep(min(args.period, 0.01))
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        final_stream_count = wait_for_state_while_streaming(bus, args, AXIS_STATE_IDLE, timeout=3.0)

        all_samples = neutral_samples + hold_samples
        finite_iq = [abs(sample[3]) for sample in all_samples if math.isfinite(sample[3])]
        if finite_iq:
            max_iq = max(finite_iq)
            print(f"Max |Iq_set| observed: {max_iq:.4f} A")
            if max_iq > args.max_iq_set:
                raise RuntimeError(f"Iq_set exceeded limit: {max_iq:.4f} A > {args.max_iq_set:.4f} A")
        print(f"MIT frames sent: {neutral_count + hold_count + final_stream_count}")
        final_check(bus, args)
        print("PASS: MIT mode link test completed")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        for i in range(10):
            send_mit_frame(bus, args, pack_neutral_mit_frame(i))
            time.sleep(0.005)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        if args.clear_at_end:
            time.sleep(0.2)
            drain_can_rx(bus)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
