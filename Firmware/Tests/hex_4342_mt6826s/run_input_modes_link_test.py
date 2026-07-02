#!/usr/bin/env python3
"""
Stage E: conservative input-mode/trajectory CAN link test.

This is a no-motion link gate. It checks that the retained input modes and
trajectory command surfaces still accept CAN commands after trimming:

- VELOCITY_CONTROL + VEL_RAMP
- TORQUE_CONTROL + TORQUE_RAMP
- POSITION_CONTROL + POS_FILTER
- POSITION_CONTROL + TRAP_TRAJ
- trajectory vel/accel/inertia command IDs
- circular encoder diagnostic readout

The current production CAN surface does not expose direct readback for
controller.config.input_mode/control_mode or every input-mode tuning parameter.
Therefore this script validates link reachability by sending conservative zero
setpoints while IDLE and checking heartbeat/status/errors remain clean.
"""

import argparse
import math
import struct
import time

from common import (
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_SET_INPUT_POS,
    CMD_SET_INPUT_TORQUE,
    CMD_SET_INPUT_VEL,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    EXT_TYPE_FLOAT32,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    ext_request,
    get_status_ex,
    open_bus,
    request_two_floats,
    send,
    set_controller_modes,
    set_input_pos,
    set_input_torque,
    set_input_vel,
    set_limits,
    set_pos_gain,
    set_requested_state,
    wait_heartbeat,
)


CMD_SET_TRAJ_VEL_LIMIT = 0x011
CMD_SET_TRAJ_ACCEL_LIMITS = 0x012
CMD_SET_TRAJ_INERTIA = 0x013

INPUT_MODE_VEL_RAMP = 2
INPUT_MODE_POS_FILTER = 3
INPUT_MODE_TRAP_TRAJ = 5
INPUT_MODE_TORQUE_RAMP = 6

EXT_STATUS = {
    0: "OK",
    1: "UNKNOWN",
    2: "READONLY",
    3: "INVALID_TYPE",
    4: "INVALID_VALUE",
    5: "BUSY_ARMED",
}


def check_heartbeat_clean(hb, context):
    if hb is None:
        raise RuntimeError(f"{context}: heartbeat timeout")
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError(
            f"{context}: heartbeat reports error state={hb['axis_state']} "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )


def ensure_idle(bus, args):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
    check_heartbeat_clean(hb, "initial")
    print(
        f"Heartbeat: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
        f"axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}"
    )
    if hb["axis_state"] == AXIS_STATE_IDLE:
        return

    print("Axis is armed; requesting IDLE before input-mode link test...")
    set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
    deadline = time.monotonic() + args.idle_timeout
    last = hb
    while time.monotonic() < deadline:
        last = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if last is None:
            continue
        check_heartbeat_clean(last, "waiting for IDLE")
        print(
            f"  hb state={last['axis_state']}({AXIS_STATES.get(last['axis_state'], '?')}) "
            f"axis_error=0x{last['axis_error']:08X}"
        )
        if last["axis_state"] == AXIS_STATE_IDLE:
            return
    raise RuntimeError(f"Timed out waiting for IDLE; last heartbeat={last}")


def check_status_clean(bus, args, context):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=1.0)
    check_heartbeat_clean(hb, context)
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
    if status is None:
        raise RuntimeError(f"{context}: GET_AXIS_STATUS_EX timeout")
    if status["axis_error"]:
        raise RuntimeError(f"{context}: axis_error=0x{status['axis_error']:08X} status={status}")
    print(
        f"  {context}: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
        f"traj_done={hb['trajectory_done']} status_flags=0x{status['flags']:02X}"
    )
    return hb, status


def get_encoder_pos(bus, args):
    _, values = request_two_floats(
        bus,
        args.node_id,
        CMD_GET_ENCODER_ESTIMATES,
        args.extended_id,
        timeout=args.timeout,
    )
    if values is None:
        raise RuntimeError("GET_ENCODER_ESTIMATES timeout")
    pos, vel = values
    if not math.isfinite(pos) or not math.isfinite(vel):
        raise RuntimeError(f"Encoder estimate is not finite: pos={pos} vel={vel}")
    return pos, vel


def get_vernier_float(bus, args, item_id, name):
    resp = ext_request(bus, args.node_id, 0x0A, item=item_id, extended_id=args.extended_id, timeout=args.timeout)
    if resp is None:
        raise RuntimeError(f"GET_VERNIER_DIAGNOSTICS {name}: no response")
    if resp["status"] != 0:
        raise RuntimeError(
            f"GET_VERNIER_DIAGNOSTICS {name}: status={EXT_STATUS.get(resp['status'], resp['status'])} raw={resp['raw']}"
        )
    if resp["type"] != EXT_TYPE_FLOAT32:
        raise RuntimeError(f"GET_VERNIER_DIAGNOSTICS {name}: unexpected type={resp['type']} raw={resp['raw']}")
    if not math.isfinite(resp["value_f"]):
        raise RuntimeError(f"GET_VERNIER_DIAGNOSTICS {name}: non-finite value={resp['value_f']}")
    return resp["value_f"], resp["raw"]


def set_traj_vel_limit(bus, args, value):
    send(bus, args.node_id, CMD_SET_TRAJ_VEL_LIMIT, struct.pack("<f", float(value)), args.extended_id)


def set_traj_accel_limits(bus, args, accel, decel):
    send(bus, args.node_id, CMD_SET_TRAJ_ACCEL_LIMITS, struct.pack("<ff", float(accel), float(decel)), args.extended_id)


def set_traj_inertia(bus, args, inertia):
    send(bus, args.node_id, CMD_SET_TRAJ_INERTIA, struct.pack("<f", float(inertia)), args.extended_id)


def exercise_mode(bus, args, name, control_mode, input_mode, command_callback):
    print(f"Testing {name}: control_mode={control_mode} input_mode={input_mode}")
    set_controller_modes(bus, args.node_id, control_mode, input_mode, args.extended_id)
    time.sleep(args.settle)
    command_callback()
    time.sleep(args.settle)
    check_status_clean(bus, args, name)


def main():
    parser = argparse.ArgumentParser(description="Stage E: input-mode and trajectory CAN link test.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--idle-timeout", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=0.05)
    parser.add_argument("--vel-limit", type=float, default=0.3)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--pos-gain", type=float, default=0.0)
    parser.add_argument("--traj-vel-limit", type=float, default=0.1)
    parser.add_argument("--traj-accel-limit", type=float, default=0.2)
    parser.add_argument("--traj-inertia", type=float, default=0.0)
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        if args.clear_at_start:
            print("Clearing existing errors...")
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        ensure_idle(bus, args)
        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
        if status is None:
            raise RuntimeError("GET_AXIS_STATUS_EX timeout")
        print(
            f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
            f"motor_calibrated={status['motor_calibrated']} encoder_ready={status['encoder_ready']} "
            f"trajectory_done={status['trajectory_done']} axis_error=0x{status['axis_error']:08X}"
        )
        if status["axis_error"]:
            raise RuntimeError("Axis status reports error")

        pos, vel = get_encoder_pos(bus, args)
        circular_pos, circular_raw = get_vernier_float(bus, args, 0x22, "encoder_pos_circular")
        print(f"Encoder: pos={pos:.6f} turn vel={vel:.6f} turn/s circular={circular_pos:.6f} raw={circular_raw}")

        print("Sending conservative limits/gains/trajectory config...")
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_pos_gain(bus, args.node_id, args.pos_gain, args.extended_id)
        set_traj_vel_limit(bus, args, args.traj_vel_limit)
        set_traj_accel_limits(bus, args, args.traj_accel_limit, args.traj_accel_limit)
        set_traj_inertia(bus, args, args.traj_inertia)
        time.sleep(args.settle)
        check_status_clean(bus, args, "after trajectory config")

        exercise_mode(
            bus,
            args,
            "VELOCITY_CONTROL + VEL_RAMP",
            CONTROL_MODE_VELOCITY_CONTROL,
            INPUT_MODE_VEL_RAMP,
            lambda: set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id),
        )
        exercise_mode(
            bus,
            args,
            "TORQUE_CONTROL + TORQUE_RAMP",
            CONTROL_MODE_TORQUE_CONTROL,
            INPUT_MODE_TORQUE_RAMP,
            lambda: set_input_torque(bus, args.node_id, 0.0, args.extended_id),
        )
        exercise_mode(
            bus,
            args,
            "POSITION_CONTROL + POS_FILTER",
            CONTROL_MODE_POSITION_CONTROL,
            INPUT_MODE_POS_FILTER,
            lambda: set_input_pos(bus, args.node_id, pos, 0.0, 0.0, args.extended_id),
        )
        exercise_mode(
            bus,
            args,
            "POSITION_CONTROL + TRAP_TRAJ",
            CONTROL_MODE_POSITION_CONTROL,
            INPUT_MODE_TRAP_TRAJ,
            lambda: set_input_pos(bus, args.node_id, pos, 0.0, 0.0, args.extended_id),
        )

        print("Restoring POSITION_CONTROL + PASSTHROUGH with current position target while IDLE...")
        set_controller_modes(
            bus,
            args.node_id,
            CONTROL_MODE_POSITION_CONTROL,
            INPUT_MODE_PASSTHROUGH,
            args.extended_id,
        )
        send(bus, args.node_id, CMD_SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0), args.extended_id)
        send(bus, args.node_id, CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0), args.extended_id)
        send(bus, args.node_id, CMD_SET_INPUT_TORQUE, struct.pack("<f", 0.0), args.extended_id)
        time.sleep(args.settle)

        final_hb, final_status = check_status_clean(bus, args, "final")
        print(f"Final heartbeat: {final_hb}")
        print(f"Final status: {final_status}")
        print("PASS: input-mode and trajectory CAN link test completed")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        set_input_torque(bus, args.node_id, 0.0, args.extended_id)
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
