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
    CONTROL_MODE_VELOCITY_CONTROL,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    decode_anticogging_flags,
    get_anticogging_status,
    get_basic_config,
    get_status_ex,
    open_bus,
    request_u32,
    request_u64,
    request_two_floats,
    save_configuration,
    set_basic_config,
    set_anticogging_config,
    set_controller_modes,
    set_input_pos,
    set_input_vel,
    set_limits,
    set_precalibrated,
    set_requested_state,
    wait_heartbeat,
)


PARAM_POS_GAIN = 0x30
PARAM_VEL_GAIN = 0x31
PARAM_VEL_INTEGRATOR_GAIN = 0x32


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
        print("Detailed errors before abort:", " ".join(read_error_summary(bus, args)))
        raise RuntimeError("M0 is not calibrated/ready or has axis error")


def write_float_config(bus, args, param_id, name, value):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_FLOAT32,
        value_float=value, extended_id=args.extended_id,
    )
    if resp is None:
        raise RuntimeError(f"{name}: no response")
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"  {name}={value}: {status} raw={resp['raw']}")
    if resp["status"] != 0:
        raise RuntimeError(f"{name} failed with {status}")


def read_float_config(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"  {name}: TIMEOUT")
        return None
    if resp["status"] != 0 or resp["type"] != EXT_TYPE_FLOAT32:
        print(f"  {name}: status={EXT_STATUS.get(resp['status'], resp['status'])} raw={resp['raw']}")
        return None
    print(f"  {name}={resp['value_f']:.8g} raw={resp['raw']}")
    return resp["value_f"]


def read_error_summary(bus, args):
    odrv = get_anticogging_status(bus, args.node_id, 0x06, extended_id=args.extended_id, timeout=0.5)
    motor_data, motor = request_u64(bus, args.node_id, 0x003, args.extended_id, timeout=0.5)
    encoder_data, encoder = request_u32(bus, args.node_id, 0x004, args.extended_id, timeout=0.5)
    controller_data, controller = request_u32(bus, args.node_id, 0x01D, args.extended_id, timeout=0.5)
    return (
        f"odrv=0x{odrv['value_u']:08X}" if odrv else "odrv=TIMEOUT",
        f"motor=0x{motor:016X}" if motor is not None else "motor=TIMEOUT",
        f"encoder=0x{encoder:08X}" if encoder is not None else "encoder=TIMEOUT",
        f"controller=0x{controller:08X}" if controller is not None else "controller=TIMEOUT",
    )


def configure_anticogging(bus, args):
    flags_resp = get_anticogging_status(bus, args.node_id, 0x01, extended_id=args.extended_id, timeout=1.0)
    if flags_resp is None or flags_resp["status"] != 0:
        print("Anticogging status unavailable; skipping anticogging toggle.")
        return None

    flags = decode_anticogging_flags(flags_resp["value_u"])
    print(f"Anticogging before test: flags=0x{flags_resp['value_u']:02X} {flags}")
    if args.disable_anticogging and flags["anticogging_enabled"]:
        resp = set_anticogging_config(
            bus, args.node_id, 0x01, EXT_TYPE_UINT32,
            value=0, extended_id=args.extended_id,
        )
        if resp is None or resp["status"] != 0:
            raise RuntimeError("Failed to disable anticogging")
        print("Anticogging disabled for this tuning run.")
    return flags


def restore_anticogging(bus, args, previous_flags):
    if not previous_flags or not args.disable_anticogging or not previous_flags["anticogging_enabled"]:
        return
    resp = set_anticogging_config(
        bus, args.node_id, 0x01, EXT_TYPE_UINT32,
        value=1, extended_id=args.extended_id,
    )
    if resp is None or resp["status"] != 0:
        print("WARN: failed to restore anticogging_enabled=True")
    else:
        print("Anticogging restored to enabled.")


def apply_gains(bus, args):
    print("Current gain readback:")
    read_float_config(bus, args, PARAM_POS_GAIN, "pos_gain")
    read_float_config(bus, args, PARAM_VEL_GAIN, "vel_gain")
    read_float_config(bus, args, PARAM_VEL_INTEGRATOR_GAIN, "vel_integrator_gain")

    print("Writing requested gains...")
    if args.pos_gain is not None:
        write_float_config(bus, args, PARAM_POS_GAIN, "pos_gain", args.pos_gain)
    if args.vel_gain is not None:
        write_float_config(bus, args, PARAM_VEL_GAIN, "vel_gain", args.vel_gain)
    if args.vel_integrator_gain is not None:
        write_float_config(bus, args, PARAM_VEL_INTEGRATOR_GAIN, "vel_integrator_gain", args.vel_integrator_gain)

    print("Gain readback after write:")
    read_float_config(bus, args, PARAM_POS_GAIN, "pos_gain")
    read_float_config(bus, args, PARAM_VEL_GAIN, "vel_gain")
    read_float_config(bus, args, PARAM_VEL_INTEGRATOR_GAIN, "vel_integrator_gain")


def wait_for_state(bus, args, target_state, timeout):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.5)
        if hb is None:
            continue
        print(
            f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )
        if (
            hb["axis_state"] == target_state
            and hb["axis_error"] == 0
            and not hb["motor_error_flag"]
            and not hb["encoder_error_flag"]
            and not hb["controller_error_flag"]
        ):
            return hb
        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            raise RuntimeError(f"Error while waiting for {AXIS_STATES.get(target_state, target_state)}")
    raise RuntimeError(f"Timed out waiting for {AXIS_STATES.get(target_state, target_state)}")


def read_sample(bus, args, start_time, target):
    _, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=0.5)
    _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=0.2)
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.2)
    t = time.monotonic() - start_time

    pos = enc[0] if enc else float("nan")
    vel = enc[1] if enc else float("nan")
    iq_set = iq[0] if iq else float("nan")
    iq_meas = iq[1] if iq else float("nan")
    pos_err = target - pos if args.mode == "position" and math.isfinite(pos) else float("nan")
    vel_err = target - vel if args.mode == "velocity" and math.isfinite(vel) else float("nan")

    tail = ""
    if hb is not None:
        tail = f" state={hb['axis_state']} err=0x{hb['axis_error']:08X}"
    if args.mode == "position":
        print(
            f"  t={t:5.2f}s target={target: .5f} pos={pos: .5f} pos_err={pos_err: .5f} "
            f"vel={vel: .5f} Iq_set={iq_set: .3f} Iq_meas={iq_meas: .3f}{tail}"
        )
    else:
        print(
            f"  t={t:5.2f}s target={target: .5f} vel={vel: .5f} vel_err={vel_err: .5f} "
            f"pos={pos: .5f} Iq_set={iq_set: .3f} Iq_meas={iq_meas: .3f}{tail}"
        )
    if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
        print("  detailed errors:", " ".join(read_error_summary(bus, args)))
        raise RuntimeError("Error observed during gain test")
    return pos, vel, iq_set, iq_meas


def run_velocity_test(bus, args):
    print("Preparing velocity-control test...")
    set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
    set_controller_modes(bus, args.node_id, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
    set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
    time.sleep(0.1)

    print("Requesting CLOSED_LOOP_CONTROL...")
    set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
    wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, args.enter_timeout)

    print(f"Commanding velocity {args.velocity:.5f} turns/s for {args.duration:.2f}s...")
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        elapsed = time.monotonic() - start
        if args.ramp_time > 0.0:
            ramp = min(1.0, elapsed / args.ramp_time)
            target = args.velocity * ramp
        else:
            target = args.velocity
        set_input_vel(bus, args.node_id, target, args.torque_ff, args.extended_id)
        read_sample(bus, args, start, target)
        time.sleep(args.sample_period)

    print("Stopping velocity command...")
    set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
    time.sleep(args.stop_time)


def run_position_test(bus, args):
    print("Preparing position-control test...")
    _, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=1.0)
    if enc is None:
        raise RuntimeError("GET_ENCODER_ESTIMATES timeout")
    start_pos = enc[0]
    target = start_pos + args.step
    print(f"Initial position={start_pos:.6f} turn, target={target:.6f} turn")

    set_input_pos(bus, args.node_id, start_pos, 0.0, 0.0, args.extended_id)
    set_controller_modes(bus, args.node_id, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
    set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
    time.sleep(0.1)

    print("Requesting CLOSED_LOOP_CONTROL...")
    set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
    wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, args.enter_timeout)
    time.sleep(args.settle_before_step)

    print(f"Commanding position step {args.step:.5f} turn for {args.duration:.2f}s...")
    set_input_pos(bus, args.node_id, target, args.vel_ff, args.torque_ff, args.extended_id)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        read_sample(bus, args, start, target)
        time.sleep(args.sample_period)

    print("Holding current target briefly, then returning to IDLE...")
    set_input_pos(bus, args.node_id, target, 0.0, 0.0, args.extended_id)
    time.sleep(args.stop_time)


def main():
    parser = argparse.ArgumentParser(description="Tune ODrive M0 velocity/position loop gains over CAN.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--mode", choices=["write-only", "velocity", "position"], default="write-only")
    parser.add_argument("--pos-gain", type=float, default=None)
    parser.add_argument("--vel-gain", type=float, default=None)
    parser.add_argument("--vel-integrator-gain", type=float, default=None)
    parser.add_argument("--velocity", type=float, default=0.1, help="Velocity command in turns/s for velocity mode.")
    parser.add_argument("--ramp-time", type=float, default=0.0, help="Velocity ramp time in seconds. 0 means step command.")
    parser.add_argument("--step", type=float, default=0.02, help="Position step in turns for position mode.")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--sample-period", type=float, default=0.1)
    parser.add_argument("--vel-limit", type=float, default=20.0)
    parser.add_argument("--current-limit", type=float, default=5.0)
    parser.add_argument("--torque-ff", type=float, default=0.0)
    parser.add_argument("--vel-ff", type=float, default=0.0)
    parser.add_argument("--enter-timeout", type=float, default=5.0)
    parser.add_argument("--settle-before-step", type=float, default=0.3)
    parser.add_argument("--stop-time", type=float, default=0.5)
    parser.add_argument("--set-precalibrated-if-needed", action="store_true")
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    parser.add_argument("--save", action="store_true", help="Save gains after writing. This reboots ODrive.")
    parser.add_argument("--disable-anticogging", action="store_true", help="Temporarily disable anticogging feedforward during the tuning test.")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    previous_anticogging_flags = None
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat OK: raw={hb['raw']} state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")

        if args.clear_at_start:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        require_ready(bus, args)

        bus_vi = request_two_floats(bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, args.extended_id, timeout=1.0)
        if bus_vi[1] is not None:
            print(f"Bus: voltage={bus_vi[1][0]:.3f} V current={bus_vi[1][1]:.3f} A raw={bus_vi[0].hex(' ')}")

        apply_gains(bus, args)
        previous_anticogging_flags = configure_anticogging(bus, args)

        if args.save:
            print("Saving configuration. ODrive will reboot...")
            ack = save_configuration(bus, args.node_id, args.extended_id, timeout=2.0)
            if ack is None:
                print("  save ACK timeout; reboot may have happened before ACK was observed")
            else:
                print(f"  save ACK raw={ack['raw']} status={EXT_STATUS.get(ack['status'], ack['status'])}")
            time.sleep(2.0)
            return 0

        if args.mode == "velocity":
            run_velocity_test(bus, args)
        elif args.mode == "position":
            run_position_test(bus, args)
        else:
            print("write-only mode: gains written, no motor motion requested.")

        print("Returning to IDLE...")
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        final = wait_for_state(bus, args, AXIS_STATE_IDLE, timeout=3.0)
        restore_anticogging(bus, args, previous_anticogging_flags)
        print(f"Final heartbeat: raw={final['raw']} state={final['axis_state']} axis_error=0x{final['axis_error']:08X}")
        print("PASS: gain tuning command completed")
        return 0
    except Exception:
        print("Stopping motor due to failure...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        try:
            restore_anticogging(bus, args, previous_anticogging_flags)
        except Exception:
            pass
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
