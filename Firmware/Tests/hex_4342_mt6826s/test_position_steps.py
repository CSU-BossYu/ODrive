#!/usr/bin/env python3
"""
Position static-error test.

The test captures the current output-shaft position as a software zero, then
commands 0 -> 180 -> 360 -> 540 -> 720 -> 1080 deg and records static error at
each target. By default it uses POSITION_CONTROL + TRAP_TRAJ and does not write
velocity or trajectory limits, so the limits configured from the UI remain in
effect.
"""

import argparse
import os
import struct
import sys
import time

try:
    import numpy as np
except ImportError:
    print("pip install numpy")
    sys.exit(1)

try:
    import can
except ImportError:
    print("pip install python-can")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
except ImportError:
    print("pip install matplotlib")
    sys.exit(1)


CMD_HEARTBEAT = 0x001
CMD_SET_REQUESTED_STATE = 0x007
CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODES = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_SET_LIMITS = 0x00F
CMD_SET_TRAJ_VEL_LIMIT = 0x011
CMD_SET_TRAJ_ACCEL_LIMITS = 0x012
CMD_CLEAR_ERRORS = 0x018
CMD_EXTENDED = 0x01E

SUB_CMD_VERNIER_DIAG = 0x0A
VERNIER_OUTPUT_VALID = 0x27
VERNIER_OUTPUT_POS = 0x28
VERNIER_OUTPUT_VEL = 0x29

CTRL_POSITION = 3
INP_PASSTHROUGH = 1
INP_TRAP_TRAJ = 5

AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8

EXT_OK = 0
EXT_TYPE_FLOAT32 = 1
EXT_TYPE_UINT32 = 3
EXT_STATUS = {
    0: "OK",
    1: "UNKNOWN",
    2: "READONLY",
    3: "INVALID_TYPE",
    4: "INVALID_VALUE",
    5: "BUSY_ARMED",
}

TARGET_DEG = [0, 180, 360, 540, 720, 1080]
HOLD_S = 2.0
SETTLE_S = 5.0
VEL_LIMIT_RPS = 1.0
CURRENT_LIMIT_A = 3.0
TRAJ_VEL_RPS = 1.0
TRAJ_ACCEL_RPS2 = 1.0


def arb_id(nid, cid):
    return (nid << 5) | cid


def send(bus, nid, cid, data=b"", ext=False):
    bus.send(can.Message(arbitration_id=arb_id(nid, cid),
                         is_extended_id=ext, data=data, dlc=len(data)))


def _recv(bus, nid, cid, ext, timeout, skip_dlc0=True):
    eid = arb_id(nid, cid)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(max(0.001, deadline - time.monotonic()))
        if msg is None or msg.arbitration_id != eid or msg.is_extended_id != ext:
            continue
        if msg.is_remote_frame:
            continue
        if skip_dlc0 and msg.dlc == 0:
            continue
        return bytes(msg.data)
    return None


def recv_heartbeat(bus, nid, ext=False, timeout=2.0):
    data = _recv(bus, nid, CMD_HEARTBEAT, ext, timeout)
    if data and len(data) >= 8:
        axis_error, axis_state = struct.unpack("<IB", data[:5])
        ctrl_flags = data[7]
        return {
            "axis_error": axis_error,
            "axis_state": axis_state,
            "traj_done": bool(ctrl_flags & 0x80),
            "raw": data,
        }
    return None


def poll_encoder(bus, nid, ext=False, timeout=0.2):
    data = _recv(bus, nid, CMD_GET_ENCODER_ESTIMATES, ext, timeout)
    if data and len(data) >= 8:
        return struct.unpack("<ff", data[:8])
    return None, None


def ext_request(bus, nid, sub, item=0, rtype=0, value=0, ext=False,
                timeout=1.0, match_item=False):
    payload = struct.pack("<BBBBi", sub, item, rtype, 0, int(value))
    send(bus, nid, CMD_EXTENDED, payload, ext)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = _recv(bus, nid, CMD_EXTENDED, ext,
                     max(0.001, deadline - time.monotonic()), skip_dlc0=False)
        if data is None:
            return None
        if data == payload or len(data) < 8:
            continue
        sub_cmd, resp_item, status, value_type, value_i = struct.unpack("<BBBBi", data[:8])
        if sub_cmd != sub:
            continue
        if match_item and resp_item != item:
            continue
        value_u = struct.unpack("<BBBBI", data[:8])[4]
        value_f = struct.unpack("<f", data[4:8])[0]
        return {
            "sub_cmd": sub_cmd,
            "item": resp_item,
            "status": status,
            "type": value_type,
            "value_i": value_i,
            "value_u": value_u,
            "value_f": value_f,
        }
    return None


def set_precalibrated(bus, nid, ext=False):
    return ext_request(bus, nid, 0x02, item=0x03, ext=ext)


def get_status_ex(bus, nid, ext=False, timeout=1.0):
    resp = ext_request(bus, nid, 0x01, ext=ext, timeout=timeout)
    if resp is None:
        return None
    flags = resp["type"]
    return {
        "axis_state": resp["status"],
        "axis_error": resp["value_u"],
        "motor_calibrated": bool(flags & 0x01),
        "encoder_ready": bool(flags & 0x02),
        "trajectory_done": bool(flags & 0x20),
        "raw": resp,
    }


def read_vernier_diag(bus, nid, item, ext=False, timeout=0.2):
    resp = ext_request(bus, nid, SUB_CMD_VERNIER_DIAG, item=item, ext=ext,
                       timeout=timeout, match_item=True)
    if resp is None or resp["status"] != EXT_OK:
        return None
    if resp["type"] == EXT_TYPE_FLOAT32:
        return resp["value_f"]
    if resp["type"] == EXT_TYPE_UINT32:
        return resp["value_u"]
    return resp["value_f"]


def poll_position_sample(bus, nid, ext=False, source="vernier-output", timeout=0.2):
    if source in ("vernier-output", "auto"):
        valid = read_vernier_diag(bus, nid, VERNIER_OUTPUT_VALID, ext, timeout)
        pos = read_vernier_diag(bus, nid, VERNIER_OUTPUT_POS, ext, timeout)
        vel = read_vernier_diag(bus, nid, VERNIER_OUTPUT_VEL, ext, timeout)
        if pos is not None and (valid is None or int(valid) != 0):
            return pos, 0.0 if vel is None else vel, "vernier-output"
        if source == "vernier-output":
            return None, None, "vernier-output"

    pos, vel = poll_encoder(bus, nid, ext, timeout)
    if pos is None:
        return None, None, "encoder"
    return pos, vel, "encoder"


def position_error_deg(measured_deg, target_deg):
    return measured_deg - target_deg


def send_input_pos(bus, nid, pos_rev, ext=False):
    send(bus, nid, CMD_SET_INPUT_POS, struct.pack("<fhh", float(pos_rev), 0, 0), ext)


def wait_until_settled(bus, nid, zero_abs_rev, target_deg, ext, source,
                       timeout_s, pos_tol_deg, vel_tol_rps):
    deadline = time.monotonic() + timeout_s
    consecutive = 0
    last_err = None
    last_vel = None
    used_source = source
    while time.monotonic() < deadline:
        pos_rev, vel_rps, used_source = poll_position_sample(bus, nid, ext, source, 0.15)
        if pos_rev is None:
            continue
        measured_deg = (pos_rev - zero_abs_rev) * 360.0
        err_deg = position_error_deg(measured_deg, target_deg)
        last_err = err_deg
        last_vel = vel_rps
        if abs(err_deg) <= pos_tol_deg and abs(vel_rps) <= vel_tol_rps:
            consecutive += 1
            if consecutive >= 5:
                return True, last_err, last_vel, used_source
        else:
            consecutive = 0
        time.sleep(0.03)
    return False, last_err, last_vel, used_source


def main():
    parser = argparse.ArgumentParser(description="Position static-error test")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended", action="store_true", default=False)
    parser.add_argument("--source", choices=("vernier-output", "encoder", "auto"),
                        default="vernier-output",
                        help="Default uses calibrated output-shaft estimate.")
    parser.add_argument("--input-mode", choices=("trap", "passthrough"), default="trap")
    parser.add_argument("--hold", type=float, default=HOLD_S)
    parser.add_argument("--settle-timeout", type=float, default=SETTLE_S)
    parser.add_argument("--pos-tol-deg", type=float, default=0.5)
    parser.add_argument("--vel-tol-rps", type=float, default=0.01)
    parser.add_argument("--configure-limits", action="store_true",
                        help="Write legacy SET_LIMITS and trajectory limits before the test.")
    args = parser.parse_args()

    print(f"Opening CAN: {args.channel} @ {args.bitrate/1e6:.1f} Mbps  node_id={args.node_id}")
    bus = can.Bus(interface="pcan", channel=args.channel, bitrate=args.bitrate)
    ext = args.extended
    nid = args.node_id

    hb = recv_heartbeat(bus, nid, ext, 5.0)
    if hb is None:
        print("ERROR: no heartbeat")
        bus.shutdown()
        sys.exit(1)
    print(f"  axis_error=0x{hb['axis_error']:08X}  axis_state={hb['axis_state']}")

    if hb["axis_error"]:
        send(bus, nid, CMD_CLEAR_ERRORS, b"\x00" * 8, ext)
        time.sleep(0.3)

    if hb["axis_state"] != AXIS_CLOSED_LOOP:
        print("  axis is idle; requesting CLOSED_LOOP")
        send(bus, nid, CMD_SET_REQUESTED_STATE, struct.pack("<I", AXIS_CLOSED_LOOP), ext)
        time.sleep(0.5)
        hb = recv_heartbeat(bus, nid, ext, 2.0)

    if hb is None or hb["axis_state"] != AXIS_CLOSED_LOOP:
        status = get_status_ex(bus, nid, ext, timeout=1.0)
        if status is not None:
            print("  status_ex: axis_state=%s motor_calibrated=%s encoder_ready=%s axis_error=0x%08X" %
                  (status["axis_state"], status["motor_calibrated"],
                   status["encoder_ready"], status["axis_error"]))

        if status is None or not (status["motor_calibrated"] and status["encoder_ready"]):
            resp = set_precalibrated(bus, nid, ext)
            if resp is None or resp["status"] != EXT_OK:
                status_name = EXT_STATUS.get(resp["status"] if resp else -1, str(resp))
                print(f"ERROR: cannot enter closed loop and set_precalibrated failed ({status_name}).")
                print("       Calibration may not be loaded, or this firmware rejects marking while armed/busy.")
                bus.shutdown()
                sys.exit(1)
            send(bus, nid, CMD_SET_REQUESTED_STATE, struct.pack("<I", AXIS_CLOSED_LOOP), ext)
            time.sleep(0.5)
            hb = recv_heartbeat(bus, nid, ext, 2.0)

    if hb is None or hb["axis_state"] != AXIS_CLOSED_LOOP:
        state = hb["axis_state"] if hb else "?"
        print(f"ERROR: cannot enter closed loop (state={state}). Check axis_error/status_ex above.")
        bus.shutdown()
        sys.exit(1)

    if args.configure_limits:
        send(bus, nid, CMD_SET_LIMITS, struct.pack("<ff", VEL_LIMIT_RPS, CURRENT_LIMIT_A), ext)
        send(bus, nid, CMD_SET_TRAJ_VEL_LIMIT, struct.pack("<f", TRAJ_VEL_RPS), ext)
        send(bus, nid, CMD_SET_TRAJ_ACCEL_LIMITS,
             struct.pack("<ff", TRAJ_ACCEL_RPS2, TRAJ_ACCEL_RPS2), ext)
        time.sleep(0.05)

    input_mode = INP_PASSTHROUGH if args.input_mode == "passthrough" else INP_TRAP_TRAJ
    send(bus, nid, CMD_SET_CONTROLLER_MODES, struct.pack("<ii", CTRL_POSITION, input_mode), ext)
    time.sleep(0.1)
    print(f"  state={hb['axis_state']}, mode=POSITION+{args.input_mode.upper()}")
    print(f"  feedback source={args.source}")

    zero_pos_rev, _, used_source = poll_position_sample(bus, nid, ext, args.source, 1.0)
    if zero_pos_rev is None:
        print("ERROR: cannot read initial position")
        bus.shutdown()
        sys.exit(1)

    send_input_pos(bus, nid, zero_pos_rev, ext)
    settled, err, vel, used_source = wait_until_settled(
        bus, nid, zero_pos_rev, 0.0, ext, args.source,
        args.settle_timeout, args.pos_tol_deg, args.vel_tol_rps)
    if not settled:
        print(f"  WARNING: initial 0deg settle timeout; last_err={err}, last_vel={vel}")
    print(f"  software zero captured at {zero_pos_rev:.6f} rev ({used_source})\n")

    all_data = []
    metric_rows = []

    col_hdr = (
        f"{'Step':>4s}  {'Target':>7s}  {'Mean err':>9s}  {'Std err':>8s}  "
        f"{'Max |err|':>9s}  {'RMS err':>8s}"
    )
    print(col_hdr)
    print("-" * len(col_hdr))

    try:
        for step_idx, target_deg in enumerate(TARGET_DEG, start=1):
            target_abs_rev = zero_pos_rev + target_deg / 360.0
            send_input_pos(bus, nid, target_abs_rev, ext)

            settled, err, vel, used_source = wait_until_settled(
                bus, nid, zero_pos_rev, target_deg, ext, args.source,
                args.settle_timeout, args.pos_tol_deg, args.vel_tol_rps)
            if not settled:
                print(f"  [{step_idx}] WARNING: target not settled within {args.settle_timeout}s "
                      f"(last_err={err}, last_vel={vel})")

            hold_start = time.monotonic()
            hold_data = []
            while (time.monotonic() - hold_start) < args.hold:
                pos_rev, vel_rps, used_source = poll_position_sample(
                    bus, nid, ext, args.source, 0.2)
                if pos_rev is None:
                    continue
                pos_deg = (pos_rev - zero_pos_rev) * 360.0
                err_deg = position_error_deg(pos_deg, target_deg)
                sample = {
                    "t": time.monotonic(),
                    "pos_rev": pos_rev,
                    "pos_deg": pos_deg,
                    "target_deg": target_deg,
                    "err_deg": err_deg,
                    "vel_rps": vel_rps,
                }
                hold_data.append(sample)
                all_data.append(sample)
                time.sleep(0.05)

            if hold_data:
                err_arr = np.array([d["err_deg"] for d in hold_data])
                mean_err = float(np.mean(err_arr))
                std_err = float(np.std(err_arr))
                max_abs = float(np.max(np.abs(err_arr)))
                rms = float(np.sqrt(np.mean(err_arr**2)))
            else:
                mean_err = std_err = max_abs = rms = 0.0

            metric_rows.append({
                "step": step_idx,
                "target_deg": target_deg,
                "mean_err": mean_err,
                "std_err": std_err,
                "max_abs": max_abs,
                "rms_err": rms,
                "n": len(hold_data),
            })
            print(f" {step_idx:3d}   {target_deg:5.0f}deg   {mean_err:7.4f}  "
                  f"{std_err:7.4f}  {max_abs:9.4f}  {rms:7.4f}")
    finally:
        send(bus, nid, CMD_SET_REQUESTED_STATE, struct.pack("<I", AXIS_IDLE), ext)
        bus.shutdown()
        print("\nMotor stopped, bus closed.")

    print("\n" + "=" * 74)
    print("  STATIC POSITION ERROR SUMMARY")
    print("=" * 74)
    print(f"{'Step':>4s}  {'Target':>7s}  {'Mean err':>9s}  {'Std err':>8s}  "
          f"{'Max |err|':>9s}  {'RMS err':>8s}  {'Samples':>7s}")
    print("-" * 74)
    for row in metric_rows:
        print(f" {row['step']:3d}   {row['target_deg']:5.0f}deg   "
              f"{row['mean_err']:7.4f}   {row['std_err']:7.4f}   "
              f"{row['max_abs']:7.4f}    {row['rms_err']:7.4f}   "
              f"{row['n']:5d}")

    if all_data:
        plot_position(all_data, args)
    else:
        print("\nNo data to plot.")


def plot_position(all_data, args):
    t0 = all_data[0]["t"]
    t = np.array([d["t"] - t0 for d in all_data])
    pos = np.array([d["pos_deg"] for d in all_data])
    ref = np.array([d["target_deg"] for d in all_data])
    err = np.array([d["err_deg"] for d in all_data])

    fig, (ax_pos, ax_err) = plt.subplots(2, 1, figsize=(14, 7.0), sharex=True)
    ax_pos.plot(t, ref, "k--", lw=1.0, alpha=0.5, label="target")
    ax_pos.plot(t, pos, "#2563eb", lw=1.0, label="measured")
    ax_pos.set_ylabel("Position (deg)")
    ax_pos.legend(loc="upper left")
    ax_pos.grid(True, alpha=0.3)

    ax_err.plot(t, err, "#dc2626", lw=1.0, label="error")
    ax_err.axhline(0.0, color="k", lw=0.8, alpha=0.4)
    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Error (deg)")
    ax_err.legend(loc="upper left")
    ax_err.grid(True, alpha=0.3)

    fig.suptitle(f"Position static-error test ({args.channel}, node_id={args.node_id})")
    fig.tight_layout()
    out = os.path.join(os.path.dirname(__file__) or ".", "position_step_response.png")
    fig.savefig(out, dpi=150)
    print(f"\nPlot saved to: {out}")
    plt.show()


if __name__ == "__main__":
    main()
