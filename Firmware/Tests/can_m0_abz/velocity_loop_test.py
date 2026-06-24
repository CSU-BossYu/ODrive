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
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_controller_modes,
    set_input_vel,
    set_limits,
    set_precalibrated,
    set_requested_state,
    wait_heartbeat,
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


def wait_for_closed_loop(bus, args):
    start = time.monotonic()
    while time.monotonic() - start < args.enter_timeout:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.5)
        if hb is None:
            continue
        print(
            f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )
        if (
            hb["axis_state"] == AXIS_STATE_CLOSED_LOOP_CONTROL
            and hb["axis_error"] == 0
            and not hb["motor_error_flag"]
            and not hb["encoder_error_flag"]
            and not hb["controller_error_flag"]
        ):
            return
        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            raise RuntimeError("Error while entering closed loop")
    raise RuntimeError("Timed out waiting for CLOSED_LOOP_CONTROL")


def wait_for_idle(bus, args, timeout=3.0):
    start = time.monotonic()
    last = None
    while time.monotonic() - start < timeout:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.5)
        if hb is None:
            continue
        last = hb
        if hb["axis_state"] == AXIS_STATE_IDLE:
            return hb
    return last


def main():
    parser = argparse.ArgumentParser(description="Run a conservative velocity-loop test on calibrated ODrive M0.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--velocity", type=float, default=0.05, help="Commanded velocity in turns/s.")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--vel-limit", type=float, default=20.0)
    parser.add_argument("--current-limit", type=float, default=5.0)
    parser.add_argument("--enter-timeout", type=float, default=5.0)
    parser.add_argument("--set-precalibrated-if-needed", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true", help="Clear errors after the test if any are observed.")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    samples = []
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat OK: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        require_ready(bus, args)

        bus_vi = request_two_floats(bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, args.extended_id, timeout=1.0)
        if bus_vi[1] is not None:
            print(f"Bus: voltage={bus_vi[1][0]:.3f} V current={bus_vi[1][1]:.3f} A raw={bus_vi[0].hex(' ')}")

        print("Preparing velocity control...")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.1)
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        set_controller_modes(bus, args.node_id, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        time.sleep(0.1)

        print("Requesting CLOSED_LOOP_CONTROL...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_for_closed_loop(bus, args)

        print(f"Commanding velocity {args.velocity:.4f} turns/s for {args.duration:.2f}s...")
        set_input_vel(bus, args.node_id, args.velocity, 0.0, args.extended_id)
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            enc_data, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=0.5)
            iq_data, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=0.2)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.2)
            t = time.monotonic() - start

            pos = enc[0] if enc else float("nan")
            vel = enc[1] if enc else float("nan")
            iq_set = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            samples.append((t, pos, vel, iq_set, iq_meas))

            hb_tail = ""
            if hb is not None:
                hb_tail = f" state={hb['axis_state']} err=0x{hb['axis_error']:08X}"
            print(f"  t={t:4.2f}s pos={pos: .5f} vel={vel: .5f} Iq_set={iq_set: .3f} Iq_meas={iq_meas: .3f}{hb_tail}")

            if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
                raise RuntimeError("Error observed during velocity-loop test")
            time.sleep(0.25)

        print("Stopping motor and returning to IDLE...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        time.sleep(0.5)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        final = wait_for_idle(bus, args, timeout=3.0)

        finite_vel = [sample[2] for sample in samples if math.isfinite(sample[2])]
        if finite_vel:
            avg_vel = sum(finite_vel) / len(finite_vel)
            print(f"Average measured velocity: {avg_vel:.5f} turns/s from {len(finite_vel)} samples")

        if final:
            print(
                f"Final heartbeat: raw={final['raw']} state={final['axis_state']} "
                f"axis_error=0x{final['axis_error']:08X} flags="
                f"0x{final['motor_flags']:02X}/0x{final['encoder_flags']:02X}/0x{final['controller_flags']:02X}"
            )
            if final["axis_error"] or final["motor_error_flag"] or final["encoder_error_flag"] or final["controller_error_flag"]:
                raise RuntimeError("Final heartbeat reports errors")
            if final["axis_state"] != AXIS_STATE_IDLE:
                raise RuntimeError("M0 did not return to IDLE after velocity-loop test")
        else:
            raise RuntimeError("No final heartbeat after returning to IDLE")
        print("PASS: velocity-loop test completed")
        return 0
    except Exception:
        print("Stopping motor due to failure...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
