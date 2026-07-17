#!/usr/bin/env python3
"""
Velocity step-response test: sweep 0.1 → 60 rpm, record encoder data,
compute per-step metrics, and plot results.

Usage:
  python test_velocity_steps.py [--channel PCAN_USBBUS1] [--node-id 0]

Step sequence (output shaft):
  0.1 rpm → 1 → 10 → 20 → 40 → 60 rpm, each held for 20 seconds.

Firmware command units are output-shaft rev/s in MT6826S vernier mode, so
rpm / 60 = rev/s. The default measurement source is the vernier output-shaft
diagnostic velocity (ext 0x0A item 0x29), not the raw motor encoder velocity.
"""

import argparse
import math
import struct
import time
import sys
import os

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
    matplotlib.use("TkAgg")  # Windows-friendly default
    import matplotlib.pyplot as plt
except ImportError:
    print("pip install matplotlib")
    sys.exit(1)

# ── CAN protocol constants ─────────────────────────────────────────────
CMD_HEARTBEAT           = 0x001
CMD_SET_REQUESTED_STATE = 0x007
CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODES = 0x00B
CMD_SET_INPUT_VEL       = 0x00D
CMD_SET_LIMITS          = 0x00F
CMD_CLEAR_ERRORS        = 0x018
CMD_EXTENDED            = 0x01E
SUB_CMD_VERNIER_DIAG    = 0x0A
VERNIER_OUTPUT_VALID    = 0x27
VERNIER_OUTPUT_VEL      = 0x29

CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_PASSTHROUGH       = 1
INPUT_MODE_VEL_RAMP          = 2

AXIS_STATE_IDLE                = 1
AXIS_STATE_FULL_CALIBRATION    = 3
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

EXT_STATUS = {0: "OK", 1: "UNKNOWN", 2: "READONLY",
              3: "INVALID_TYPE", 4: "INVALID_VALUE", 5: "BUSY_ARMED"}
EXT_TYPE_FLOAT32 = 1
EXT_TYPE_UINT32 = 3

def ext_request(bus, node_id, sub_cmd, item=0, req_type=0,
                value=0, extended_id=False, timeout=1.0):
    """Send extended command, return decoded response dict or None."""
    payload = struct.pack("<BBBBi", sub_cmd, item, req_type, 0, int(value))
    send(bus, node_id, CMD_EXTENDED, payload, extended_id)
    data = recv_matching(bus, node_id, CMD_EXTENDED, extended_id,
                         timeout, first_byte=sub_cmd, not_data=payload)
    if data is None:
        return None
    sub_cmd_r, b1, b2, b3, value_i = struct.unpack("<BBBBi", data[:8])
    value_u = struct.unpack("<BBBBI", data[:8])[4]
    value_f = struct.unpack("<f", data[4:8])[0]
    return {"sub_cmd": sub_cmd_r, "item": b1, "status": b2,
            "type": b3, "value_i": value_i, "value_u": value_u,
            "value_f": value_f}


def set_precalibrated(bus, node_id, extended_id=False):
    """Mark motor + encoder as pre-calibrated (must have been calibrated
    at least once this power cycle). Flags = 0x03 = motor + encoder."""
    return ext_request(bus, node_id, 0x02, item=0x03,
                       extended_id=extended_id)
def arb_id(node_id, cmd_id):
    return (node_id << 5) | cmd_id


def send(bus, node_id, cmd_id, data=b"", extended_id=False):
    bus.send(can.Message(
        arbitration_id=arb_id(node_id, cmd_id),
        is_extended_id=extended_id,
        data=data,
        dlc=len(data),
    ))


def recv_matching(bus, node_id, cmd_id, extended_id=False, timeout=1.0,
                  first_byte=None, not_data=None):
    expected_id = arb_id(node_id, cmd_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = bus.recv(max(0.001, remaining))
        if msg is None:
            continue
        if msg.arbitration_id != expected_id:
            continue
        if msg.is_extended_id != extended_id:
            continue
        if msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        # first_byte: match ext responses by sub_cmd (data[0]); not_data:
        # skip a bus-echoed copy of the request payload we just sent.
        if first_byte is not None and (not data or data[0] != first_byte):
            continue
        if not_data is not None and data == not_data:
            continue
        return data
    return None


def poll_encoder(bus, node_id, extended_id=False, timeout=0.1):
    """Wait for the next periodic encoder estimate frame. Falls back to
    sending a request if no periodic frame arrives within the timeout."""
    expected_id = arb_id(node_id, CMD_GET_ENCODER_ESTIMATES)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = bus.recv(max(0.001, remaining))
        if msg is None:
            continue
        if msg.arbitration_id != expected_id:
            continue
        if msg.is_extended_id != extended_id:
            continue
        if msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        # DLC=0 is a request from the host — skip
        if msg.dlc == 0:
            continue
        if len(data) >= 8:
            pos, vel = struct.unpack("<ff", data[:8])
            return pos, vel

    # Fallback: explicitly request
    send(bus, node_id, CMD_GET_ENCODER_ESTIMATES, b"", extended_id)
    data = recv_matching(bus, node_id, CMD_GET_ENCODER_ESTIMATES,
                         extended_id, timeout)
    if data and len(data) >= 8:
        pos, vel = struct.unpack("<ff", data[:8])
        return pos, vel
    return None, None


def read_vernier_diag(bus, node_id, item_id, extended_id=False, timeout=0.05):
    """Read one vernier diagnostic item via extended command 0x0A."""
    resp = ext_request(bus, node_id, SUB_CMD_VERNIER_DIAG, item=item_id,
                       req_type=EXT_TYPE_UINT32, extended_id=extended_id,
                       timeout=timeout)
    if resp is None or resp["status"] != 0:
        return None
    if resp["type"] == EXT_TYPE_FLOAT32:
        return resp["value_f"]
    return resp["value_u"]


def poll_velocity_sample(bus, node_id, source, extended_id=False, timeout=0.1):
    """Return (velocity_rev_s, source_name)."""
    if source in ("vernier-output", "auto"):
        valid = read_vernier_diag(bus, node_id, VERNIER_OUTPUT_VALID,
                                  extended_id, timeout=timeout * 0.5)
        vel = read_vernier_diag(bus, node_id, VERNIER_OUTPUT_VEL,
                                extended_id, timeout=timeout * 0.5)
        if vel is not None and (valid is None or int(valid) != 0):
            return float(vel), "vernier-output"
        if source == "vernier-output":
            return None, "vernier-output"

    _, vel = poll_encoder(bus, node_id, extended_id, timeout=timeout)
    if vel is None:
        return None, "encoder"
    return float(vel), "encoder"


# ── Test parameters ─────────────────────────────────────────────────────
VEL_STEPS_RPM = [0.1, 1.0, 10.0, 20.0, 40.0, 60.0]
STEP_DURATION_S = 20.0
POLL_PERIOD_S = 0.02  # 50 Hz polling
VEL_LIMIT_REV_S = 2.0     # 120 rpm — well above our max
CURRENT_LIMIT_A = 3.0     # safe bench current
GEAR_RATIO = 42.0          # vernier main ratio


def main():
    parser = argparse.ArgumentParser(description="Velocity step-response test")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended", action="store_true", default=False)
    parser.add_argument(
        "--source",
        choices=("vernier-output", "encoder", "auto"),
        default="vernier-output",
        help=("Velocity measurement source. Use vernier-output for output-shaft "
              "rpm in MT6826S vernier mode; encoder is the normal CAN encoder "
              "estimate and is mostly for debugging."),
    )
    parser.add_argument("--duration", type=float, default=STEP_DURATION_S,
                        help="Hold time per velocity step in seconds.")
    args = parser.parse_args()

    print(f"Opening CAN: {args.channel} @ {args.bitrate/1e6:.1f} Mbps  node_id={args.node_id}")
    bus = can.Bus(interface="pcan", channel=args.channel, bitrate=args.bitrate)

    ext = args.extended
    nid = args.node_id

    # ── 1) Check heartbeat ──────────────────────────────────────────
    print("Waiting for heartbeat ...")
    data = recv_matching(bus, nid, CMD_HEARTBEAT, ext, timeout=5.0)
    if data is None:
        print("ERROR: no heartbeat — check power / CAN wiring")
        bus.shutdown()
        sys.exit(1)
    axis_error, axis_state = struct.unpack("<IB", data[:5])
    print(f"  axis_error=0x{axis_error:08X}  axis_state={axis_state}")

    # ── 2) Clear errors if any ──────────────────────────────────────
    if axis_error != 0:
        send(bus, nid, CMD_CLEAR_ERRORS, b"\x00" * 8, ext)
        time.sleep(0.3)
        print("  errors cleared")

    # ── 3) Ensure closed-loop: if not already, request it ────────────
    if axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("Axis not in closed loop — requesting it now.")
        # Set pre-calibrated first so the motor can enter closed loop
        resp = set_precalibrated(bus, nid, ext)
        if resp is None or resp["status"] != 0:
            print("ERROR: set_precalibrated failed (status=%s). "
                  "Run calibration via upper computer first." %
                  (EXT_STATUS.get(resp["status"] if resp else -1, str(resp))))
            bus.shutdown()
            sys.exit(1)
        send(bus, nid, CMD_SET_REQUESTED_STATE,
             struct.pack("<I", AXIS_STATE_CLOSED_LOOP_CONTROL), ext)
        time.sleep(0.5)
        data = recv_matching(bus, nid, CMD_HEARTBEAT, ext, timeout=2.0)
        if data is None:
            print("ERROR: lost heartbeat after entering closed loop")
            bus.shutdown()
            sys.exit(1)
        axis_state = struct.unpack("<IB", data[:5])[1]
        if axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"ERROR: axis_state={axis_state} (not CLOSED_LOOP). "
                  "Check motor/encoder calibration.")
            bus.shutdown()
            sys.exit(1)

    # ── 4) Set limits + velocity control mode ────────────────────────
    set_limits = struct.pack("<ff", VEL_LIMIT_REV_S, CURRENT_LIMIT_A)
    send(bus, nid, CMD_SET_LIMITS, set_limits, ext)
    time.sleep(0.05)

    send(bus, nid, CMD_SET_CONTROLLER_MODES,
         struct.pack("<ii", CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH), ext)
    time.sleep(0.1)

    print(f"  axis_state={axis_state}, mode=VELOCITY+PASSTHROUGH, "
          f"vel_limit={VEL_LIMIT_REV_S}rps, current_limit={CURRENT_LIMIT_A}A, "
          f"source={args.source}\n")

    # ── 6) Run step sequence ────────────────────────────────────────
    all_data = []   # list of {t, vel_rpm, vel_rev_s, vel_ref_rpm, source}
    metric_rows = []

    print(f"{'Step':>4s}  {'Ref rpm':>8s}  {'Raw rev/s':>10s}  "
          f"{'Vel rpm':>9s}  {'Std rpm':>8s}  {'RMS err':>8s}")
    print("-" * 60)

    for step_idx, rpm_ref in enumerate(VEL_STEPS_RPM):
        rev_s_ref = rpm_ref / 60.0
        data_rev_s = struct.pack("<ff", rev_s_ref, 0.0)
        send(bus, nid, CMD_SET_INPUT_VEL, data_rev_s, ext)

        step_start = time.monotonic()
        step_velocities = []

        while (time.monotonic() - step_start) < args.duration:
            vel_rev_s, sample_source = poll_velocity_sample(
                bus, nid, args.source, ext, timeout=0.1)
            if vel_rev_s is None:
                time.sleep(POLL_PERIOD_S)
                continue

            vel_rpm = vel_rev_s * 60.0
            point = {
                "t": time.monotonic(),
                "vel_rpm": vel_rpm,
                "vel_rev_s": vel_rev_s,
                "vel_ref_rpm": rpm_ref,
                "source": sample_source,
            }
            step_velocities.append(point)
            all_data.append(point)

            elapsed = time.monotonic() - step_start
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)
            else:
                time.sleep(POLL_PERIOD_S)

        # ── Per-step metrics ────────────────────────────────────────
        if step_velocities:
            vel_arr = np.array([p["vel_rpm"] for p in step_velocities])
            raw_arr = np.array([p["vel_rev_s"] for p in step_velocities])
            mean_vel = float(np.mean(vel_arr))
            mean_raw = float(np.mean(raw_arr))
            std_vel = float(np.std(vel_arr))
            rms_err = float(np.sqrt(np.mean((vel_arr - rpm_ref)**2)))
        else:
            mean_vel = mean_raw = std_vel = rms_err = 0.0

        metric_rows.append({
            "step": step_idx + 1,
            "rpm_ref": rpm_ref,
            "mean_raw": mean_raw,
            "mean_rpm": mean_vel,
            "std_rpm": std_vel,
            "rms_err": rms_err,
            "point_count": len(step_velocities),
        })

        print(f" {step_idx+1:3d}   {rpm_ref:6.1f}   {mean_raw:8.6f}     "
              f"{mean_vel:7.3f}     {std_vel:6.3f}   {rms_err:6.3f}")

    # ── 7) Stop ─────────────────────────────────────────────────────
    send(bus, nid, CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0), ext)
    time.sleep(0.5)
    send(bus, nid, CMD_SET_REQUESTED_STATE,
         struct.pack("<I", AXIS_STATE_IDLE), ext)
    bus.shutdown()
    print("\nMotor stopped, bus closed.")

    # ── 8) Print summary table ──────────────────────────────────────
    print("\n" + "=" * 70)
    print("  PER-STEP METRICS SUMMARY")
    print("=" * 70)
    print(f"{'Step':>4s}  {'Ref rpm':>8s}  {'Raw rev/s':>10s}  "
          f"{'Vel rpm':>9s}  {'Std rpm':>8s}  {'RMS err':>8s}  {'Samples':>7s}")
    print("-" * 70)
    for row in metric_rows:
        print(f" {row['step']:3d}   {row['rpm_ref']:6.1f}   {row['mean_raw']:8.6f}     "
              f"{row['mean_rpm']:7.3f}     {row['std_rpm']:6.3f}   "
              f"{row['rms_err']:6.3f}   {row['point_count']:5d}")

    # ── 9) Plot ─────────────────────────────────────────────────────
    if not all_data:
        print("\nNo data to plot.")
        return

    plot(all_data, metric_rows, args)


def plot(all_data, metrics, args):
    """Single-panel velocity vs time plot."""
    t0 = all_data[0]["t"]
    t_arr = np.array([d["t"] - t0 for d in all_data])
    vel_arr = np.array([d["vel_rpm"] for d in all_data])
    ref_arr = np.array([d["vel_ref_rpm"] for d in all_data])

    step_boundaries = [0.0]
    acc = 0.0
    for _ in VEL_STEPS_RPM:
        acc += args.duration
        step_boundaries.append(acc)

    fig, ax = plt.subplots(figsize=(14, 5))

    ax.plot(t_arr, ref_arr, "k--", linewidth=1.0, alpha=0.5, label="reference")
    ax.plot(t_arr, vel_arr, "#2563eb", linewidth=1.0, label="measured")
    for b in step_boundaries[:-1]:
        ax.axvline(b, color="gray", linestyle=":", alpha=0.4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (rpm)")
    ax.legend(loc="upper left")
    ax.set_title(
        f"Velocity step-response  ({args.channel}, node_id={args.node_id}, source={args.source})")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__) or ".",
                            "velocity_step_response.png")
    fig.savefig(out_path, dpi=150)
    print(f"\nPlot saved to: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
