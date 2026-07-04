#!/usr/bin/env python3
"""Phase 1+2 verification: mode-aware command watchdog + graceful timeout.

Tests (direct PCAN, no foc_ui backend needed):
  1. ext 0x50-0x5C read/write round-trip.
  2. Position mode one-shot: send target, go silent > watchdog -> holds final
     position, stays in CLOSED_LOOP, no error. (HOLD_LAST_POSITION)
  3. Velocity mode: send vel, go silent -> quick-stop to 0, stays armed.
     (QUICK_STOP_AND_HOLD)
  4. Torque mode: send torque, go silent -> torque zeroed, stays armed.
     (TORQUE_ZERO)
  5. Read/status frames do NOT feed the command watchdog (only motion frames do).
  6. servo_mode (0x5B) set/get round-trip.

Conservative bench limits (current_limit=0.5A, vel_limit=2.0). The motor moves
slowly; this is intentional.

Usage:
  python test_command_watchdog.py --bitrate 1000000
"""
import argparse
import struct
import time
from common import (
    open_bus, send, wait_heartbeat, get_status_ex,
    ext_request, clear_errors, set_limits, set_requested_state,
    set_controller_modes, set_input_pos, set_input_vel, set_input_torque,
    CMD_GET_ENCODER_ESTIMATES, CMD_GET_BUS_VOLTAGE_CURRENT,
)

# ext sub-commands + items
GET_CTRL = 0x0B
SET_CTRL = 0x0C
EXT_TYPE_FLOAT = 1
EXT_TYPE_UINT = 3
EXT_STATUS_OK = 0

# control_config items
I_VEL_ACCEL = 0x50
I_VEL_DECEL = 0x51
I_QUICK_STOP = 0x52
I_CAN_WD_MS = 0x53
I_TIMEOUT_ACTION = 0x54
I_RUNTIME_STATE = 0x58
I_LAST_REASON = 0x59
I_TRAJ_DONE = 0x5A
I_SERVO_MODE = 0x5B

# ServoControlMode
SERVO_TORQUE = 0
SERVO_VELOCITY = 1
SERVO_PROFILE_POSITION = 2
SERVO_MIT = 3

# control_runtime_state flag bits
F_RUNNING = 1 << 1
F_HOLDING = 1 << 2
F_QUICK_STOP = 1 << 3
F_COMM_TIMEOUT = 1 << 4
F_CMD_WD = 1 << 5


def ctrl_get(bus, nid, item, ext, is_float):
    r = ext_request(bus, nid, GET_CTRL, item=item, extended_id=ext)
    if r is None or r["status"] != EXT_STATUS_OK:
        return None
    return r["value_f"] if is_float else r["value_u"]


def ctrl_set(bus, nid, item, ext, value, is_float):
    r = ext_request(bus, nid, SET_CTRL, item=item,
                    req_type=EXT_TYPE_FLOAT if is_float else EXT_TYPE_UINT,
                    value_float=value if is_float else None,
                    value=0 if is_float else int(value),
                    extended_id=ext)
    return r is not None and r["status"] == EXT_STATUS_OK


def hb_flags(bus, nid, ext, timeout=0.2):
    """Get a decoded heartbeat dict (None if no frame in timeout)."""
    return wait_heartbeat(bus, nid, ext, timeout=timeout)


def silent_for(bus, nid, ext, dur, label):
    """Go silent (no motion commands) for `dur` seconds, sampling heartbeat."""
    print(f"  [{label}] silent {dur:.1f}s (no motion commands)...")
    samples = []
    end = time.monotonic() + dur
    while time.monotonic() < end:
        hb = hb_flags(bus, nid, ext, timeout=0.15)
        if hb:
            samples.append(hb)
    return samples


def check(name, cond, detail=""):
    mark = "PASS" if cond else "FAIL"
    print(f"  [{mark}] {name}" + (f"  ({detail})" if detail else ""))
    return cond


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node-id", type=int, default=0)
    ap.add_argument("--extended-id", action="store_true")
    ap.add_argument("--wd-ms", type=int, default=800,
                    help="command watchdog timeout ms (default 800)")
    ap.add_argument("--vel", type=float, default=0.3, help="test velocity rev/s")
    ap.add_argument("--torque", type=float, default=0.05, help="test torque Nm")
    ap.add_argument("--pos-step", type=float, default=0.1, help="test position step turns")
    args = ap.parse_args()

    ext = args.extended_id
    nid = args.node_id
    bus = open_bus(args.channel, args.bitrate)
    results = []

    print(f"=== Phase 1+2 command-watchdog test (wd={args.wd_ms}ms) ===")

    # --- setup ---
    clear_errors(bus, nid, ext)
    set_limits(bus, nid, 2.0, 0.5, ext)  # vel_limit=2, current=0.5A
    # velocity accel/decel + quick-stop decel
    ctrl_set(bus, nid, I_VEL_ACCEL, ext, 0.5, True)
    ctrl_set(bus, nid, I_VEL_DECEL, ext, 0.5, True)
    ctrl_set(bus, nid, I_QUICK_STOP, ext, 1.0, True)
    # enable command watchdog
    ctrl_set(bus, nid, I_CAN_WD_MS, ext, args.wd_ms, False)
    time.sleep(0.1)

    # --- Test 1: ext read/write round-trip ---
    print("\n-- Test 1: ext 0x50-0x5C read/write --")
    ctrl_set(bus, nid, I_VEL_ACCEL, ext, 0.42, True)
    v = ctrl_get(bus, nid, I_VEL_ACCEL, ext, True)
    results.append(check("velocity_accel_limit write/read 0.42",
                         v is not None and abs(v - 0.42) < 1e-4, f"got {v}"))
    wd = ctrl_get(bus, nid, I_CAN_WD_MS, ext, False)
    results.append(check("can_watchdog_timeout_ms read",
                         wd == args.wd_ms, f"got {wd}"))

    # --- Test 2: Position one-shot holds after CAN gap ---
    print("\n-- Test 2: position one-shot + CAN gap -> HOLD --")
    clear_errors(bus, nid, ext)
    set_controller_modes(bus, nid, 3, 5, ext)  # POSITION, TRAP_TRAJ
    set_requested_state(bus, nid, 8, ext)  # CLOSED_LOOP
    time.sleep(0.4)
    home = ctrl_get(bus, nid, 0x55, ext, True)  # profile_vel_limit (not pos)
    # send a position target, then go silent
    target = args.pos_step  # relative-ish; just move
    set_input_pos(bus, nid, target, 0.0, 0.0, ext)
    time.sleep(0.3)  # let trajectory start
    samples = silent_for(bus, nid, ext, (args.wd_ms / 1000.0) + 1.5, "pos hold")
    last = samples[-1] if samples else {}
    held = (last.get("axis_state") == 8 and
            not last.get("controller_error_flag") and
            not last.get("motor_error_flag"))
    results.append(check("position holds after watchdog, no disarm",
                         held, f"state={last.get('axis_state')} "
                               f"cerr={last.get('controller_error_flag')} "
                               f"merr={last.get('motor_error_flag')}"))
    cflags = last.get("controller_flags", 0)
    results.append(check("comm_timeout flag set",
                         bool(cflags & F_COMM_TIMEOUT),
                         f"controller_flags=0x{cflags:02x}"))
    set_requested_state(bus, nid, 1, ext)  # IDLE
    time.sleep(0.3)

    # --- Test 3: Velocity quick-stop on CAN gap ---
    print("\n-- Test 3: velocity + CAN gap -> QUICK_STOP_AND_HOLD --")
    clear_errors(bus, nid, ext)
    set_controller_modes(bus, nid, 2, 2, ext)  # VELOCITY, VEL_RAMP
    set_requested_state(bus, nid, 8, ext)
    time.sleep(0.4)
    set_input_vel(bus, nid, args.vel, 0.0, ext)
    time.sleep(0.3)
    samples = silent_for(bus, nid, ext, (args.wd_ms / 1000.0) + 2.0, "vel quick-stop")
    last = samples[-1] if samples else {}
    armed = (last.get("axis_state") == 8 and
             not last.get("controller_error_flag"))
    results.append(check("velocity stays armed after watchdog (no disarm)",
                         armed, f"state={last.get('axis_state')} "
                                f"cerr={last.get('controller_error_flag')}"))
    cflags = last.get("controller_flags", 0)
    results.append(check("quick_stop/comm_timeout flag set",
                         bool(cflags & (F_QUICK_STOP | F_COMM_TIMEOUT)),
                         f"controller_flags=0x{cflags:02x}"))
    set_requested_state(bus, nid, 1, ext)
    time.sleep(0.3)

    # --- Test 4: Torque zero on CAN gap ---
    print("\n-- Test 4: torque + CAN gap -> TORQUE_ZERO --")
    clear_errors(bus, nid, ext)
    set_controller_modes(bus, nid, 1, 1, ext)  # TORQUE, PASSTHROUGH
    set_requested_state(bus, nid, 8, ext)
    time.sleep(0.4)
    set_input_torque(bus, nid, args.torque, ext)
    time.sleep(0.2)
    samples = silent_for(bus, nid, ext, (args.wd_ms / 1000.0) + 1.5, "torque zero")
    last = samples[-1] if samples else {}
    armed = (last.get("axis_state") == 8 and
             not last.get("controller_error_flag"))
    results.append(check("torque stays armed after watchdog (no disarm)",
                         armed, f"state={last.get('axis_state')}"))
    cflags = last.get("controller_flags", 0)
    results.append(check("comm_timeout flag set (torque zero)",
                         bool(cflags & F_COMM_TIMEOUT),
                         f"controller_flags=0x{cflags:02x}"))
    set_requested_state(bus, nid, 1, ext)
    time.sleep(0.3)

    # --- Test 5: read frames do NOT feed command watchdog ---
    print("\n-- Test 5: read frames don't feed command watchdog --")
    clear_errors(bus, nid, ext)
    set_controller_modes(bus, nid, 2, 2, ext)  # VELOCITY
    set_requested_state(bus, nid, 8, ext)
    time.sleep(0.4)
    set_input_vel(bus, nid, args.vel, 0.0, ext)  # last motion command
    # now send ONLY read frames for > watchdog
    print(f"  sending only read frames for {args.wd_ms/1000.0 + 1.0:.1f}s...")
    end = time.monotonic() + (args.wd_ms / 1000.0) + 1.0
    last = None
    while time.monotonic() < end:
        # read encoder estimates + bus voltage (read frames, should NOT feed)
        send(bus, nid, CMD_GET_ENCODER_ESTIMATES, b"", ext)
        send(bus, nid, CMD_GET_BUS_VOLTAGE_CURRENT, b"", ext)
        hb = hb_flags(bus, nid, ext, timeout=0.1)
        if hb:
            last = hb
        time.sleep(0.05)
    cflags = last.get("controller_flags", 0) if last else 0
    results.append(check("read frames did NOT prevent watchdog timeout",
                         bool(cflags & F_COMM_TIMEOUT),
                         f"controller_flags=0x{cflags:02x}"))
    set_requested_state(bus, nid, 1, ext)
    time.sleep(0.3)

    # --- Test 6: servo_mode set/get ---
    print("\n-- Test 6: servo_mode (0x5B) set/get --")
    for mode, name in [(SERVO_VELOCITY, "VELOCITY"),
                       (SERVO_PROFILE_POSITION, "PROFILE_POSITION"),
                       (SERVO_TORQUE, "TORQUE")]:
        ok = ctrl_set(bus, nid, I_SERVO_MODE, ext, mode, False)
        time.sleep(0.05)
        got = ctrl_get(bus, nid, I_SERVO_MODE, ext, False)
        results.append(check(f"servo_mode set/get {name}",
                             ok and got == mode, f"got 0x{got:x}" if got is not None else "None"))

    # --- disable watchdog + cleanup ---
    ctrl_set(bus, nid, I_CAN_WD_MS, ext, 0, False)
    clear_errors(bus, nid, ext)
    set_requested_state(bus, nid, 1, ext)

    print(f"\n=== {sum(results)}/{len(results)} passed ===")
    bus.shutdown()
    raise SystemExit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
