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
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    decode_anticogging_flags,
    get_anticogging_status,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_anticogging_config,
    set_controller_modes,
    set_input_pos,
    set_linear_count,
    set_limits,
    set_pos_gain,
    set_precalibrated,
    set_requested_state,
    set_vel_gains,
    start_anticogging,
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


def wait_for_state(bus, args, target_state, timeout):
    start = time.monotonic()
    last = None
    while time.monotonic() - start < timeout:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.5)
        if hb is None:
            continue
        last = hb
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
    raise RuntimeError(f"Timed out waiting for {AXIS_STATES.get(target_state, target_state)}; last={last}")


def get_encoder(bus, args, timeout=1.0):
    data, values = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=timeout)
    if values is None:
        raise RuntimeError("GET_ENCODER_ESTIMATES timeout")
    return data, values[0], values[1]


def get_stable_encoder(bus, args, label, samples=12, period=0.1):
    readings = []
    for _ in range(samples):
        _, pos, vel = get_encoder(bus, args, timeout=1.0)
        if not math.isfinite(pos) or not math.isfinite(vel):
            raise RuntimeError(f"{label}: non-finite encoder estimate pos={pos} vel={vel}")
        readings.append((pos, vel))
        time.sleep(period)

    recent = readings[-5:]
    pos_span = max(pos for pos, _ in recent) - min(pos for pos, _ in recent)
    pos, vel = recent[-1]
    print(
        f"{label}: pos={pos:.6f} turn vel={vel:.6f} turn/s "
        f"recent_span={pos_span:.6f} turn"
    )
    if pos_span > 0.002:
        raise RuntimeError(f"{label}: encoder position is not stationary")
    return pos, vel


def read_anticogging_snapshot(bus, args):
    flags_resp = get_anticogging_status(bus, args.node_id, 0x01, extended_id=args.extended_id, timeout=1.0)
    index_resp = get_anticogging_status(bus, args.node_id, 0x02, extended_id=args.extended_id, timeout=1.0)
    odrv_resp = get_anticogging_status(bus, args.node_id, 0x06, extended_id=args.extended_id, timeout=1.0)
    if flags_resp is None or index_resp is None:
        raise RuntimeError("GET_ANTICOGGING_STATUS timeout. The flashed firmware may not include the new CAN extension.")
    if flags_resp["status"] != 0 or index_resp["status"] != 0:
        raise RuntimeError(
            f"GET_ANTICOGGING_STATUS failed: flags={EXT_STATUS.get(flags_resp['status'], flags_resp['status'])} "
            f"index={EXT_STATUS.get(index_resp['status'], index_resp['status'])}"
        )
    flags = decode_anticogging_flags(flags_resp["value_u"])
    return {
        "flags_raw": flags_resp["value_u"],
        "flags": flags,
        "index": index_resp["value_u"],
        "odrv_error": odrv_resp["value_u"] if odrv_resp and odrv_resp["status"] == 0 else None,
    }


def set_anticogging_param(bus, args, item_id, req_type, value=0, value_float=None):
    resp = set_anticogging_config(
        bus, args.node_id, item_id, req_type, value=value, value_float=value_float,
        extended_id=args.extended_id,
    )
    if resp is None:
        raise RuntimeError(f"SET_ANTICOGGING_CONFIG 0x{item_id:02X}: no response")
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"  anticog item=0x{item_id:02X}: {status} raw={resp['raw']}")
    if resp["status"] != 0:
        raise RuntimeError(f"SET_ANTICOGGING_CONFIG 0x{item_id:02X} failed with {status}")


def monitor_anticogging(bus, args):
    start = time.monotonic()
    last_print = 0.0
    max_abs_iq = 0.0
    sample_count = 0
    max_index = 0
    last_index = None
    last_index_change = start

    while time.monotonic() - start < args.max_duration:
        now = time.monotonic()
        elapsed = now - start
        snapshot = read_anticogging_snapshot(bus, args)
        _, pos, vel = get_encoder(bus, args, timeout=0.5)
        _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=0.2)
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.2)
        sample_count += 1

        iq_set = iq[0] if iq else float("nan")
        iq_meas = iq[1] if iq else float("nan")
        if math.isfinite(iq_set):
            max_abs_iq = max(max_abs_iq, abs(iq_set))

        index = snapshot["index"]
        if index != last_index:
            last_index = index
            last_index_change = now
        elif now - last_index_change > args.stall_timeout:
            raise RuntimeError(
                f"Anticogging stalled at index {index} for "
                f"{now - last_index_change:.1f}s; pos={pos:.6f}, vel={vel:.6f}, "
                f"Iq_set={iq_set:.3f}, Iq_meas={iq_meas:.3f}"
            )
        max_index = max(max_index, index)
        progress = min(100.0, index / 3600.0 * 100.0)

        if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
            raise RuntimeError(
                f"Error observed: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} "
                f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X} "
                f"odrv_error={snapshot['odrv_error']}"
            )

        if elapsed - last_print >= args.print_period:
            last_print = elapsed
            state_tail = ""
            if hb:
                state_tail = f" state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) err=0x{hb['axis_error']:08X}"
            flags = snapshot["flags"]
            print(
                f"  t={elapsed:6.1f}s idx={index:4d}/3600 progress={progress:5.1f}% "
                f"calib={int(flags['calib_anticogging'])} valid={int(flags['anticogging_valid'])} "
                f"pos={pos: .5f} turn vel={vel: .5f} turn/s "
                f"Iq_set={iq_set: .3f} Iq_meas={iq_meas: .3f} odrv_error={snapshot['odrv_error']}{state_tail}"
            )

        if snapshot["flags"]["anticogging_valid"] and not snapshot["flags"]["calib_anticogging"]:
            print(
                f"Anticogging complete: max_index={max_index}, pos={pos:.5f}, vel={vel:.5f}, "
                f"after {elapsed:.1f}s, samples={sample_count}, max|Iq_set|={max_abs_iq:.3f}A"
            )
            return True

        time.sleep(args.sample_period)

    print(f"Timed out before firmware reported completion. max_index={max_index}/3600.")
    return False


def main():
    parser = argparse.ArgumentParser(description="Trigger ODrive anticogging calibration over CAN for M0 ABZ.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--vel-limit", type=float, default=20.0)
    parser.add_argument("--current-limit", type=float, default=5.0)
    parser.add_argument("--pos-gain", type=float, default=5.0, help="Optional position gain to set before calibration.")
    parser.add_argument("--vel-gain", type=float, default=0.02, help="Optional velocity gain to set before calibration.")
    parser.add_argument("--vel-integrator-gain", type=float, default=0.05, help="Optional velocity integrator gain to set before calibration.")
    parser.add_argument("--calib-pos-threshold", type=float, default=10.0, help="Anticogging settle position threshold in encoder counts.")
    parser.add_argument("--calib-vel-threshold", type=float, default=200.0, help="Anticogging settle velocity threshold in encoder counts/s.")
    parser.add_argument("--enter-timeout", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=900.0)
    parser.add_argument("--sample-period", type=float, default=0.10)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--stall-timeout", type=float, default=15.0)
    parser.add_argument("--zero-linear-count", action="store_true", help="Set encoder linear count to 0 before entering position control.")
    parser.add_argument("--set-precalibrated-if-needed", action="store_true")
    parser.add_argument("--leave-closed-loop", action="store_true")
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

        bus_vi = request_two_floats(bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, args.extended_id, timeout=1.0)
        if bus_vi[1] is not None:
            print(f"Bus: voltage={bus_vi[1][0]:.3f} V current={bus_vi[1][1]:.3f} A raw={bus_vi[0].hex(' ')}")

        pos, vel = get_stable_encoder(bus, args, "Initial encoder")
        if args.zero_linear_count:
            print("Setting encoder linear count to 0 before position-control entry...")
            set_linear_count(bus, args.node_id, 0, args.extended_id)
            time.sleep(0.2)
            pos, vel = get_stable_encoder(bus, args, "Encoder after zero")

        print("Configuring anticogging thresholds and resetting calibration state...")
        set_anticogging_param(bus, args, 0x03, EXT_TYPE_FLOAT32, value_float=args.calib_pos_threshold)
        set_anticogging_param(bus, args, 0x04, EXT_TYPE_FLOAT32, value_float=args.calib_vel_threshold)
        set_anticogging_param(bus, args, 0x05, EXT_TYPE_UINT32, value=1)
        snapshot = read_anticogging_snapshot(bus, args)
        print(
            f"Anticogging before start: flags=0x{snapshot['flags_raw']:02X} "
            f"index={snapshot['index']} odrv_error={snapshot['odrv_error']}"
        )

        print("Preparing position control...")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.1)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        if args.pos_gain is not None:
            set_pos_gain(bus, args.node_id, args.pos_gain, args.extended_id)
        if args.vel_gain is not None or args.vel_integrator_gain is not None:
            if args.vel_gain is None or args.vel_integrator_gain is None:
                raise RuntimeError("--vel-gain and --vel-integrator-gain must be provided together")
            set_vel_gains(bus, args.node_id, args.vel_gain, args.vel_integrator_gain, args.extended_id)
        set_input_pos(bus, args.node_id, pos, 0.0, 0.0, args.extended_id)
        set_controller_modes(bus, args.node_id, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
        time.sleep(0.1)

        print("Requesting CLOSED_LOOP_CONTROL...")
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, args.enter_timeout)
        pos, vel = get_stable_encoder(bus, args, "Closed-loop hold")
        set_input_pos(bus, args.node_id, pos, 0.0, 0.0, args.extended_id)
        wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL, 1.5)

        print("Sending START_ANTICOGGING and monitoring calibration sweep...")
        start_anticogging(bus, args.node_id, args.extended_id)
        complete = monitor_anticogging(bus, args)

        if not args.leave_closed_loop:
            print("Returning to IDLE...")
            set_input_pos(bus, args.node_id, 0.0, 0.0, 0.0, args.extended_id)
            time.sleep(0.3)
            set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            wait_for_state(bus, args, AXIS_STATE_IDLE, 3.0)

        if complete:
            print("PASS: anticogging calibration was triggered and completed by external motion heuristic.")
            print("Note: save/reset of the anticogging map is not exposed by the current CAN extension.")
            return 0
        print("WARN: anticogging calibration was triggered, but completion could not be confirmed over CAN.")
        return 2
    except BaseException:
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
