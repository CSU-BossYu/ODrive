#!/usr/bin/env python3
"""Read-only observer for slow creep after a position trajectory completes."""

import argparse
import csv
import math
import statistics
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CMD_GET_IQ,
    EXT_TYPE_UINT32,
    ext_request,
    open_bus,
    request_two_floats,
    wait_heartbeat,
)


SUB_CMD_VERNIER_DIAG = 0x0A

ITEMS = (
    (0x20, "encoder_pos_turns", "f32"),
    (0x21, "encoder_vel_turns_per_s", "f32"),
    (0x22, "circular_pos_turns", "f32"),
    (0x28, "output_pos_turns", "f32"),
    (0x29, "output_vel_turns_per_s", "f32"),
    (0x64, "shadow_count", "i32"),
    (0x65, "count_in_cpr", "i32"),
    (0x66, "pos_setpoint_turns", "f32"),
    (0x67, "input_pos_turns", "f32"),
    (0x68, "held_pos_error_turns", "f32"),
    (0x69, "vel_setpoint_turns_per_s", "f32"),
    (0x6A, "vel_des_turns_per_s", "f32"),
    (0x6B, "vel_integrator_torque_nm", "f32"),
    (0x6C, "torque_feedforward_nm", "f32"),
    (0x6D, "held_motor_torque_nm", "f32"),
    (0x6E, "input_vel_turns_per_s", "f32"),
    (0x6F, "trajectory_done", "u32"),
    (0x70, "circular_setpoints", "u32"),
    (0x71, "trajectory_start_turns", "f32"),
    (0x72, "trajectory_goal_turns", "f32"),
    (0x73, "controller_position_feedback_turns", "f32"),
)

OPTIONAL_ITEMS = {0x70, 0x71, 0x72, 0x73}


def read_item(bus, args, item_id, kind):
    response = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item_id, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=args.timeout,
    )
    if response is None:
        return None
    if response["status"] != 0:
        if item_id in OPTIONAL_ITEMS:
            return None
        raise RuntimeError(
            f"diagnostic item 0x{item_id:02X} unavailable "
            "(flash the newly built firmware)")
    if kind == "f32":
        return response["value_f"]
    if kind == "i32":
        return response["value_i"]
    return response["value_u"]


def finite(rows, key):
    return [
        float(row[key]) for row in rows
        if row.get(key) is not None and math.isfinite(float(row[key]))
    ]


def span(rows, key):
    values = finite(rows, key)
    return max(values) - min(values) if values else math.nan


def last(rows, key):
    values = finite(rows, key)
    return values[-1] if values else math.nan


def wrap_pm(value, period):
    return value - math.floor((value + period / 2.0) / period) * period


def main():
    parser = argparse.ArgumentParser(
        description="Observe the live position/velocity cascade without sending commands.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=0.08)
    parser.add_argument(
        "--wait-closed-loop", type=float, default=0.0,
        help=("optional seconds to wait for CLOSED_LOOP_CONTROL; by default "
              "the script also samples IDLE because it must own the CAN "
              "adapter exclusively"))
    parser.add_argument("--csv", default="position_hold_diagnostics.csv")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    rows = []
    try:
        heartbeat = wait_heartbeat(
            bus, args.node_id, args.extended_id, timeout=3.0)
        if heartbeat is None:
            raise RuntimeError("no heartbeat")
        if heartbeat["axis_state"] != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(
                "Axis is not in CLOSED_LOOP_CONTROL "
                f"(state={heartbeat['axis_state']}, "
                f"axis_error=0x{heartbeat['axis_error']:08X}).")
            if args.wait_closed_loop > 0.0:
                print("Waiting for closed loop using this same CAN owner...")
                deadline = time.monotonic() + args.wait_closed_loop
                while time.monotonic() < deadline:
                    heartbeat = wait_heartbeat(
                        bus, args.node_id, args.extended_id, timeout=1.0)
                    if (heartbeat is not None and
                            heartbeat["axis_state"] ==
                            AXIS_STATE_CLOSED_LOOP_CONTROL):
                        break
                if (heartbeat is None or
                        heartbeat["axis_state"] !=
                        AXIS_STATE_CLOSED_LOOP_CONTROL):
                    raise RuntimeError(
                        "timed out waiting for CLOSED_LOOP_CONTROL")
            else:
                print(
                    "Sampling the idle estimator state. Keep the UI/backend "
                    "CAN connection closed while this script is running.")
        else:
            print("CLOSED_LOOP_CONTROL detected; sampling read-only state.")

        period = 1.0 / max(args.rate, 1.0)
        start = time.monotonic()
        next_sample = start
        index = 0
        while time.monotonic() - start < args.duration:
            row = {"index": index, "time_s": time.monotonic() - start}
            for item_id, name, kind in ITEMS:
                row[name] = read_item(bus, args, item_id, kind)
            _, iq = request_two_floats(
                bus, args.node_id, CMD_GET_IQ,
                args.extended_id, timeout=args.timeout)
            row["iq_setpoint_a"] = None if iq is None else iq[0]
            row["iq_measured_a"] = None if iq is None else iq[1]
            rows.append(row)

            if index % max(1, int(args.rate)) == 0:
                circular = row.get("circular_setpoints")
                circular_text = "?" if circular is None else str(int(circular))
                trajectory_goal = row.get("trajectory_goal_turns")
                trajectory_goal_text = (
                    "?" if trajectory_goal is None
                    else f"{trajectory_goal:+.7g}t")
                print(
                    f"t={row['time_s']:6.2f}s "
                    f"shadow={row['shadow_count']} "
                    f"linear={row['encoder_pos_turns']:+.8f}t "
                    f"circular={row['circular_pos_turns']:.8f}t "
                    f"ePos={row['held_pos_error_turns']:+.8g}t "
                    f"vDes={row['vel_des_turns_per_s']:+.7g}t/s "
                    f"v={row['output_vel_turns_per_s']:+.7g}t/s "
                    f"Ti={row['vel_integrator_torque_nm']:+.6g}Nm "
                    f"T={row['held_motor_torque_nm']:+.6g}Nm "
                    f"circ={circular_text} "
                    f"Xf={trajectory_goal_text}")
            index += 1
            next_sample += period
            delay = next_sample - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        bus.shutdown()

    if not rows:
        raise RuntimeError("no samples")
    with open(args.csv, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    shadow_delta = int(rows[-1]["shadow_count"]) - int(rows[0]["shadow_count"])
    pos_delta = wrap_pm(
        float(rows[-1]["circular_pos_turns"]) -
        float(rows[0]["circular_pos_turns"]), 1.0)
    print("\nPosition-hold summary")
    print(f"  shadow displacement       {shadow_delta:+d} count")
    print(f"  output position change    {pos_delta:+.9g} turn")
    print(f"  output speed p2p          {span(rows, 'output_vel_turns_per_s') * 60:.6g} rpm")
    print(f"  held position error last  {last(rows, 'held_pos_error_turns'):+.9g} turn")
    print(f"  desired speed last        {last(rows, 'vel_des_turns_per_s') * 60:+.6g} rpm")
    print(f"  velocity integral last    {last(rows, 'vel_integrator_torque_nm'):+.6g} Nm")
    print(f"  held motor torque last    {last(rows, 'held_motor_torque_nm'):+.6g} Nm")
    iq_values = finite(rows, "iq_measured_a")
    iq_text = (
        f"{statistics.fmean(iq_values):+.6g} A"
        if iq_values else "unavailable while IDLE")
    print(f"  measured Iq mean          {iq_text}")
    circular_last = last(rows, "circular_setpoints")
    circular_text = (
        "unavailable (older firmware)"
        if not math.isfinite(circular_last) else str(int(circular_last)))
    print(f"  circular setpoints        {circular_text}")
    print(f"  trajectory start/goal     {last(rows, 'trajectory_start_turns'):+.9g} / "
          f"{last(rows, 'trajectory_goal_turns'):+.9g} turn")
    print(f"  controller feedback       "
          f"{last(rows, 'controller_position_feedback_turns'):+.9g} turn")

    print("\nDiagnosis")
    trajectory_velocities = finite(rows, "vel_setpoint_turns_per_s")
    moving_velocities = [
        value for value in trajectory_velocities if abs(value) > 1.0e-5]
    linear_positions = finite(rows, "encoder_pos_turns")
    circular_positions = finite(rows, "circular_pos_turns")
    initial_branch_offset = math.nan
    if linear_positions and circular_positions:
        initial_branch_offset = (
            linear_positions[0] - circular_positions[0])

    if math.isfinite(circular_last) and int(circular_last) != 0:
        print("  Firmware is still using circular position control.")
    elif moving_velocities and all(value > 0.0 for value in moving_velocities):
        print("  The captured trajectory was planned in the positive direction.")
    elif moving_velocities and all(value < 0.0 for value in moving_velocities):
        print("  The captured trajectory was planned in the negative direction.")
    elif (math.isfinite(initial_branch_offset) and
          abs(initial_branch_offset) > 0.5):
        print(
            "  Linear and circular feedback differ by an integer-turn branch "
            f"({initial_branch_offset:+.6g} turn).")
    elif abs(shadow_delta) >= 2 and pos_delta == 0.0:
        print("  Raw motion was lost in the continuous-position numeric representation.")
    elif not all(int(row["trajectory_done"] or 0) for row in rows[-5:]):
        print("  The trajectory had not reached its terminal hold state.")
    elif abs(last(rows, "vel_setpoint_turns_per_s")) > 1.0e-6:
        print("  Trajectory velocity setpoint remained nonzero after completion.")
    elif abs(last(rows, "input_vel_turns_per_s")) > 1.0e-6:
        print("  The position command carried a nonzero velocity feed-forward.")
    elif abs(last(rows, "vel_integrator_torque_nm")) > 1.0e-4:
        print("  A residual velocity-PI integral is sustaining the hold torque/creep.")
    else:
        print("  No persistent command term was detected in the sampled cascade state.")
    print(f"\nCSV saved: {args.csv}")


if __name__ == "__main__":
    main()
