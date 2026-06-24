#!/usr/bin/env python3
import argparse
import math
import time

from common import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    clear_errors,
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
CONTROL_MODE_TORQUE_CONTROL = 1
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
    # With the common [-18, 18] Nm / 12-bit range, zero torque is between two
    # raw values. Alternate them so the average command is near zero.
    torque_raw = 2047 if index % 2 else 2048
    pos_raw = 32768
    vel_raw = 2048
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
    count = 0
    last = None
    while time.monotonic() - start < timeout:
        send_mit_frame(bus, args, pack_neutral_mit_frame(count))
        count += 1
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)
        if hb is None:
            time.sleep(args.period)
            continue
        last = hb
        if count % 10 == 1:
            print(
                f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
                f"axis_error=0x{hb['axis_error']:08X} flags="
                f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
            )
        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            raise RuntimeError("Heartbeat reports error while waiting for state")
        if hb["axis_state"] == target_state:
            print(
                f"  reached state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
                f"axis_error=0x{hb['axis_error']:08X}"
            )
            return count
        time.sleep(args.period)
    raise RuntimeError(f"Timed out waiting for state {target_state}; last heartbeat={last}")


def stream_mit(bus, args, label, make_frame, duration):
    print(label)
    start = time.monotonic()
    next_print = start
    count = 0
    samples = []
    while time.monotonic() - start < duration:
        frame = make_frame(count)
        send_mit_frame(bus, args, frame)
        count += 1
        now = time.monotonic()

        if now >= next_print:
            enc_data, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=0.05)
            iq_data, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=0.05)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)

            pos = enc[0] if enc else float("nan")
            vel = enc[1] if enc else float("nan")
            iq_set = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            samples.append((now - start, pos, vel, iq_set, iq_meas))

            hb_tail = ""
            if hb:
                hb_tail = f" state={hb['axis_state']} err=0x{hb['axis_error']:08X}"
            print(f"  t={now - start:4.2f}s pos={pos: .5f} vel={vel: .5f} Iq_set={iq_set: .3f} Iq_meas={iq_meas: .3f}{hb_tail}")

            if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
                raise RuntimeError("Heartbeat reports error during MIT stream")
            next_print = now + 0.25

        time.sleep(args.period)

    return count, samples


def main():
    parser = argparse.ArgumentParser(description="Run a conservative MIT packed-control test on calibrated ODrive M0.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--period", type=float, default=0.02, help="MIT frame period in seconds.")
    parser.add_argument("--vel-limit", type=float, default=20.0)
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--hold-kp", type=float, default=0.0)
    parser.add_argument("--hold-kd", type=float, default=0.0)
    parser.add_argument("--hold-vel", type=float, default=0.0, help="MIT v_des in rad/s for the hold frame (velocity-like mode when kp=0, t_ff=0).")
    parser.add_argument("--torque-ff", type=float, default=0.0, help="Small MIT torque feed-forward in Nm. Keep this near zero.")
    parser.add_argument("--set-precalibrated-if-needed", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat OK: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        require_ready(bus, args)

        enc_data, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=1.0)
        if enc is None:
            raise RuntimeError("No encoder estimates")
        pos_turns, vel_turns = enc
        pos_rad = pos_turns * 2.0 * math.pi
        pos_cmd = clamp(pos_rad, P_MIN + 0.5, P_MAX - 0.5)
        if pos_cmd != pos_rad:
            print(f"Current position {pos_rad:.4f} rad is outside MIT range; command is clamped to {pos_cmd:.4f} rad.")
        else:
            print(f"Current position: {pos_turns:.5f} turns ({pos_rad:.4f} rad)")

        print("Preparing TORQUE_CONTROL + INPUT_MODE_MIT...")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.1)
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

        hold_samples = []
        hold_count = 0
        if args.hold_kp != 0.0 or args.hold_kd != 0.0 or args.torque_ff != 0.0 or args.hold_vel != 0.0:
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
        final_count = wait_for_state_while_streaming(bus, args, AXIS_STATE_IDLE, timeout=3.0)

        all_samples = neutral_samples + hold_samples
        finite_iq = [abs(sample[3]) for sample in all_samples if math.isfinite(sample[3])]
        if finite_iq:
            print(f"Max |Iq_set| observed: {max(finite_iq):.4f} A")
        print(f"MIT frames sent: {neutral_count + hold_count + final_count}")
        print("PASS: MIT mode test completed")
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
