#!/usr/bin/env python3
"""
Velocity loop test with main/aux encoder angle tracing.

This script adds per-sample main/aux angle deltas to the standard velocity
loop output so you can see how each sensor's phase evolves during motion
and how d(aux-main) correlates with the Vernier output position.
"""

import argparse
import math
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_CONTROLLER_ERROR,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CMD_GET_MOTOR_ERROR,
    CONTROL_MODE_VELOCITY_CONTROL,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    ext_request,
    get_anticogging_status,
    get_basic_config,
    get_status_ex,
    open_bus,
    request_u32,
    request_u64,
    request_two_floats,
    set_basic_config,
    set_controller_modes,
    set_input_vel,
    set_limits,
    set_requested_state,
    set_vel_gains,
    wait_heartbeat,
)


PARAM_DC_MAX_NEGATIVE_CURRENT = 0x40
PARAM_DC_MAX_POSITIVE_CURRENT = 0x41
SUB_CMD_VERNIER_DIAG = 0x0A
CPR = 32768


MOTOR_ERROR_BITS = [
    (0x0000000000000001, "PHASE_RESISTANCE_OUT_OF_RANGE"),
    (0x0000000000000002, "PHASE_INDUCTANCE_OUT_OF_RANGE"),
    (0x0000000000000008, "DRV_FAULT"),
    (0x0000000000000010, "CONTROL_DEADLINE_MISSED"),
    (0x0000000000000080, "MODULATION_MAGNITUDE"),
    (0x0000000000000400, "CURRENT_SENSE_SATURATION"),
    (0x0000000000001000, "CURRENT_LIMIT_VIOLATION"),
    (0x0000000000010000, "MODULATION_IS_NAN"),
    (0x0000000000020000, "MOTOR_THERMISTOR_OVER_TEMP"),
    (0x0000000000040000, "FET_THERMISTOR_OVER_TEMP"),
    (0x0000000000080000, "TIMER_UPDATE_MISSED"),
    (0x0000000000100000, "CURRENT_MEASUREMENT_UNAVAILABLE"),
    (0x0000000000200000, "CONTROLLER_FAILED"),
    (0x0000000000400000, "I_BUS_OUT_OF_RANGE"),
    (0x0000000000800000, "BRAKE_RESISTOR_DISARMED"),
    (0x0000000001000000, "SYSTEM_LEVEL"),
    (0x0000000002000000, "BAD_TIMING"),
    (0x0000000004000000, "UNKNOWN_PHASE_ESTIMATE"),
    (0x0000000008000000, "UNKNOWN_PHASE_VEL"),
    (0x0000000010000000, "UNKNOWN_TORQUE"),
    (0x0000000020000000, "UNKNOWN_CURRENT_COMMAND"),
    (0x0000000040000000, "UNKNOWN_CURRENT_MEASUREMENT"),
    (0x0000000080000000, "UNKNOWN_VBUS_VOLTAGE"),
    (0x0000000100000000, "UNKNOWN_VOLTAGE_COMMAND"),
    (0x0000000200000000, "UNKNOWN_GAINS"),
    (0x0000000400000000, "CONTROLLER_INITIALIZING"),
    (0x0000000800000000, "UNBALANCED_PHASES"),
]

ENCODER_ERROR_BITS = [
    (0x00000001, "UNSTABLE_GAIN"),
    (0x00000002, "CPR_POLEPAIRS_MISMATCH"),
    (0x00000004, "NO_RESPONSE"),
    (0x00000008, "UNSUPPORTED_ENCODER_MODE"),
    (0x00000040, "ABS_SPI_TIMEOUT"),
    (0x00000080, "ABS_SPI_COM_FAIL"),
    (0x00000100, "ABS_SPI_NOT_READY"),
    (0x00000200, "VERNIER_RESOLVER_FAIL"),
]

CONTROLLER_ERROR_BITS = [
    (0x00000001, "OVERSPEED"),
    (0x00000002, "INVALID_INPUT_MODE"),
    (0x00000004, "UNSTABLE_GAIN"),
    (0x00000008, "INVALID_MIRROR_AXIS"),
    (0x00000010, "INVALID_LOAD_ENCODER"),
    (0x00000020, "INVALID_ESTIMATE"),
    (0x00000040, "INVALID_CIRCULAR_RANGE"),
    (0x00000080, "SPINOUT_DETECTED"),
]

ODRIVE_ERROR_BITS = [
    (0x00000001, "CONTROL_ITERATION_MISSED"),
    (0x00000002, "DC_BUS_UNDER_VOLTAGE"),
    (0x00000004, "DC_BUS_OVER_VOLTAGE"),
    (0x00000008, "DC_BUS_OVER_REGEN_CURRENT"),
    (0x00000010, "DC_BUS_OVER_CURRENT"),
    (0x00000020, "BRAKE_DEADTIME_VIOLATION"),
    (0x00000040, "BRAKE_DUTY_CYCLE_NAN"),
    (0x00000080, "INVALID_BRAKE_RESISTANCE"),
]

AXIS_ERROR_BITS = [
    (0x00000001, "INVALID_STATE"),
    (0x00000040, "MOTOR_FAILED"),
    (0x00000100, "ENCODER_FAILED"),
    (0x00000200, "CONTROLLER_FAILED"),
    (0x00000800, "WATCHDOG_TIMER_EXPIRED"),
    (0x00001000, "ESTOP_REQUESTED"),
    (0x00002000, "OVER_TEMP"),
    (0x00004000, "UNKNOWN_POSITION"),
]


def decode_bits(value, table):
    names = [name for bit, name in table if value and value & bit]
    return ",".join(names) if names else "NONE"


def latest_heartbeat(bus, args, timeout=0.8):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.15)
        if hb is not None:
            last = hb
    return last


def read_float_config(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"{name}: TIMEOUT")
        return None
    status = EXT_STATUS.get(resp["status"], resp["status"])
    value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_i"]
    print(f"{name}: {value} status={status} raw={resp['raw']}")
    return value if resp["status"] == 0 else None


def read_vernier_diag_item(bus, args, item_id, want_float=False, timeout=0.08):
    resp = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item_id, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=timeout,
    )
    if resp is None or resp["status"] != 0:
        return None
    return resp["value_f"] if want_float else resp["value_u"]


def read_vernier_diag(bus, args):
    return {
        "main_angle": read_vernier_diag_item(bus, args, 0x00),
        "aux_angle": read_vernier_diag_item(bus, args, 0x01),
        "main_valid": read_vernier_diag_item(bus, args, 0x02),
        "aux_valid": read_vernier_diag_item(bus, args, 0x03),
        "pair_seq": read_vernier_diag_item(bus, args, 0x04),
        "pair_valid": read_vernier_diag_item(bus, args, 0x05),
        "resolver_state": read_vernier_diag_item(bus, args, 0x0B),
        "vernier_pos": read_vernier_diag_item(bus, args, 0x08, want_float=True),
        "diag_pos": read_vernier_diag_item(bus, args, 0x20, want_float=True),
        "diag_vel": read_vernier_diag_item(bus, args, 0x21, want_float=True),
        "pair_busy": read_vernier_diag_item(bus, args, 0x37),
        "pair_ok": read_vernier_diag_item(bus, args, 0x38),
    }


def fmt_diag_value(value, fmt):
    return "NA" if value is None else format(value, fmt)


def write_float_config(bus, args, param_id, name, value):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_FLOAT32,
        value_float=value, extended_id=args.extended_id,
    )
    if resp is None:
        raise RuntimeError(f"{name}: no response")
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"{name} <- {value}: {status} raw={resp['raw']}")
    if resp["status"] != 0:
        raise RuntimeError(f"{name}: set failed with {status}")


def read_error_summary(bus, args):
    odrv_resp = get_anticogging_status(
        bus, args.node_id, 0x06, extended_id=args.extended_id, timeout=0.5
    )
    odrv_error = odrv_resp["value_u"] if odrv_resp else None

    _, motor_error = request_u64(
        bus, args.node_id, CMD_GET_MOTOR_ERROR, args.extended_id, timeout=0.5
    )
    _, encoder_error = request_u32(
        bus, args.node_id, CMD_GET_ENCODER_ERROR, args.extended_id, timeout=0.5
    )
    _, controller_error = request_u32(
        bus, args.node_id, CMD_GET_CONTROLLER_ERROR, args.extended_id, timeout=0.5
    )

    return odrv_error, motor_error, encoder_error, controller_error


def print_error_summary(label, errors):
    odrv_error, motor_error, encoder_error, controller_error = errors
    print(label)
    print(f"  odrv_error:      {fmt_error(odrv_error, 32)} {decode_bits(odrv_error, ODRIVE_ERROR_BITS)}")
    print(f"  motor_error:     {fmt_error(motor_error, 64)} {decode_bits(motor_error, MOTOR_ERROR_BITS)}")
    print(f"  encoder_error:   {fmt_error(encoder_error, 32)} {decode_bits(encoder_error, ENCODER_ERROR_BITS)}")
    print(f"  controller_error:{fmt_error(controller_error, 32)} {decode_bits(controller_error, CONTROLLER_ERROR_BITS)}")


def format_axis_error(value):
    return f"0x{value:08X} {decode_bits(value, AXIS_ERROR_BITS)}"


def fmt_error(value, bits):
    if value is None:
        return "TIMEOUT"
    width = 16 if bits == 64 else 8
    return f"0x{value:0{width}X}"


def phase(angle):
    """Convert raw encoder angle to phase in turns [0, 1)."""
    if angle is None:
        return float("nan")
    return (angle % CPR) / float(CPR)


def wrap_pm_half(value):
    """Wrap a phase difference to [-0.5, 0.5)."""
    wrapped = (value + 0.5) % 1.0 - 0.5
    if wrapped >= 0.5:
        wrapped -= 1.0
    return wrapped


def turns_to_deg(turns):
    return turns * 360.0


def require_ready(bus, args):
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
    if status is None:
        raise RuntimeError("GET_AXIS_STATUS_EX timeout")
    print(
        f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
        f"motor_calibrated={status['motor_calibrated']} "
        f"encoder_ready={status['encoder_ready']} axis_error={format_axis_error(status['axis_error'])}"
    )
    if status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
        print_error_summary("Errors before abort:", read_error_summary(bus, args))
        raise RuntimeError("Axis is not calibrated/ready or has an error")


def enter_closed_loop(bus, args):
    print("Requesting CLOSED_LOOP_CONTROL...")
    set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
    start = time.monotonic()
    while time.monotonic() - start < args.enter_timeout:
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.3)
        if hb is None:
            continue
        print(
            f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error={format_axis_error(hb['axis_error'])} flags="
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
            print_error_summary("Errors while entering closed loop:", read_error_summary(bus, args))
            raise RuntimeError("Error while entering closed loop")
        time.sleep(0.1)
    raise RuntimeError("Timed out waiting for CLOSED_LOOP_CONTROL")


def main():
    parser = argparse.ArgumentParser(description="Run MT6826S velocity loop with angle tracing over CANSimple.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--velocity", type=float, default=0.3, help="Output-shaft velocity in turns/s.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--ramp-time", type=float, default=1.5)
    parser.add_argument("--sample-period", type=float, default=0.25)
    parser.add_argument("--vel-limit", type=float, default=0.3)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--dc-max-negative-current", type=float, default=-0.5)
    parser.add_argument("--stop-negative-ibus", type=float, default=-0.35)
    parser.add_argument("--vel-gain", type=float, default=1.0)
    parser.add_argument("--vel-integrator-gain", type=float, default=0.05)
    parser.add_argument("--enter-timeout", type=float, default=3.0)
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    if args.dc_max_negative_current > 0:
        raise ValueError("--dc-max-negative-current must be non-positive")
    if args.stop_negative_ibus > 0:
        raise ValueError("--stop-negative-ibus must be non-positive")

    bus = open_bus(args.channel, args.bitrate)
    samples = []
    motion_started = False
    try:
        print(f"Opened {bus.channel_info}")
        hb = latest_heartbeat(bus, args, timeout=2.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(
            f"Heartbeat: raw={hb['raw']} state={hb['axis_state']} "
            f"axis_error={format_axis_error(hb['axis_error'])}"
        )

        read_float_config(bus, args, PARAM_DC_MAX_NEGATIVE_CURRENT, "dc_max_negative_current")
        write_float_config(
            bus, args, PARAM_DC_MAX_NEGATIVE_CURRENT,
            "dc_max_negative_current", args.dc_max_negative_current,
        )
        read_float_config(bus, args, PARAM_DC_MAX_NEGATIVE_CURRENT, "dc_max_negative_current")

        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.3)
        require_ready(bus, args)

        print("Preparing velocity loop...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        set_controller_modes(
            bus, args.node_id,
            CONTROL_MODE_VELOCITY_CONTROL,
            INPUT_MODE_PASSTHROUGH,
            args.extended_id,
        )
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_vel_gains(bus, args.node_id, args.vel_gain, args.vel_integrator_gain, args.extended_id)
        time.sleep(0.1)

        enter_closed_loop(bus, args)
        motion_started = True

        print(
            f"Commanding velocity {args.velocity:.4f} turns/s for {args.duration:.2f}s "
            f"(vel_gain={args.vel_gain:.2f}, vel_integrator_gain={args.vel_integrator_gain:.3f}, "
            f"current_limit={args.current_limit:.3f}A)..."
        )
        print(
            f"{'':>6s} {'t':>6s} {'cmd':>7s} {'pos':>10s} {'vel':>8s} "
            f"{'main':>6s} {'dmain':>8s} {'aux':>6s} {'daux':>8s} "
            f"{'A-M':>8s} {'d(A-M)':>8s} {'vern':>10s} {'enc':>10s} "
            f"{'Iq_set':>7s} {'Iq_meas':>7s} {'state':>5s}"
        )

        start = time.monotonic()
        first_diag = None
        last_diag = None
        missed_telemetry = 0
        while time.monotonic() - start < args.duration:
            t = time.monotonic() - start
            ramp = min(1.0, t / args.ramp_time) if args.ramp_time > 0 else 1.0
            cmd = args.velocity * ramp
            set_input_vel(bus, args.node_id, cmd, 0.0, args.extended_id)

            _, enc = request_two_floats(
                bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, args.extended_id, timeout=0.2
            )
            _, iq = request_two_floats(
                bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=0.2
            )
            _, bus_vi = request_two_floats(
                bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, args.extended_id, timeout=0.2
            )
            diag = read_vernier_diag(bus, args)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.2)

            pos = enc[0] if enc else float("nan")
            vel = enc[1] if enc else float("nan")
            iq_set = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            vbus = bus_vi[0] if bus_vi else float("nan")
            ibus = bus_vi[1] if bus_vi else float("nan")
            samples.append((t, pos, vel, vbus, ibus, iq_set, iq_meas))

            # --- angle trace computation ---
            if first_diag is None:
                first_diag = diag
            last_diag = diag

            def _delta_phase(now_val, base_val):
                if now_val is None or base_val is None:
                    return float("nan")
                return wrap_pm_half(phase(now_val) - phase(base_val))

            if first_diag is not None:
                dmain = _delta_phase(diag["main_angle"], first_diag["main_angle"])
                daux = _delta_phase(diag["aux_angle"], first_diag["aux_angle"])
                d_am = turns_to_deg(wrap_pm_half(
                    wrap_pm_half(phase(diag["aux_angle"]) - phase(diag["main_angle"]))
                    - wrap_pm_half(phase(first_diag["aux_angle"]) - phase(first_diag["main_angle"]))
                )) if (diag["main_angle"] is not None and diag["aux_angle"] is not None
                       and first_diag["main_angle"] is not None and first_diag["aux_angle"] is not None) else float("nan")
            else:
                dmain = daux = d_am = float("nan")

            main_phase_val = phase(diag["main_angle"])
            aux_phase_val = phase(diag["aux_angle"])
            am = wrap_pm_half(aux_phase_val - main_phase_val) if (
                diag["main_angle"] is not None and diag["aux_angle"] is not None
            ) else float("nan")

            if enc is None and iq is None and bus_vi is None and hb is None:
                missed_telemetry += 1
            else:
                missed_telemetry = 0

            hb_state = hb["axis_state"] if hb else "?"

            print(
                f"{'':>6s} {t:5.2f}s {cmd: .4f} {pos: 10.5f} {vel: 8.5f} "
                f"{fmt_diag_value(diag['main_angle'], '5d'):>6s} {turns_to_deg(dmain): 7.1f}° "
                f"{fmt_diag_value(diag['aux_angle'], '5d'):>6s} {turns_to_deg(daux): 7.1f}° "
                f"{turns_to_deg(am): 7.2f}° {d_am: 7.2f}° "
                f"{fmt_diag_value(diag['vernier_pos'], '10.5f'):>10s} "
                f"{fmt_diag_value(diag['diag_pos'], '10.5f'):>10s} "
                f"{iq_set: 7.3f} {iq_meas: 7.3f} {str(hb_state):>5s}"
            )

            if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
                print("Vernier diag at error:")
                print(f"  main={diag['main_angle']} aux={diag['aux_angle']} "
                      f"pair={diag['pair_seq']} valid={diag['pair_valid']} "
                      f"state={diag['resolver_state']} "
                      f"vern={diag['vernier_pos']} enc={diag['diag_pos']} "
                      f"v={diag['diag_vel']} busy/ok={diag['pair_busy']}/{diag['pair_ok']}")
                print_error_summary("Errors:", read_error_summary(bus, args))
                raise RuntimeError("Error observed during velocity loop")
            if missed_telemetry >= 3:
                raise RuntimeError("Lost CAN telemetry for 3 consecutive samples")
            if math.isfinite(ibus) and ibus < args.stop_negative_ibus:
                raise RuntimeError(f"Stopping due to ibus={ibus:.4f}A below threshold")
            time.sleep(args.sample_period)

        # --- summary ---
        print("\n=== Angle Trace Summary ===")
        if first_diag and last_diag:
            def _fmt_phase_change(diag, key, first):
                n = diag[key]
                f = first[key]
                if n is None or f is None:
                    return "NA"
                delta = wrap_pm_half(phase(n) - phase(f))
                return f"{turns_to_deg(delta):+.2f}° ({delta:+.6f} turns)"

            print(f"  main angle delta:       {_fmt_phase_change(last_diag, 'main_angle', first_diag)}")
            print(f"  aux angle delta:        {_fmt_phase_change(last_diag, 'aux_angle', first_diag)}")

            first_am = wrap_pm_half(phase(first_diag["aux_angle"]) - phase(first_diag["main_angle"]))
            last_am = wrap_pm_half(phase(last_diag["aux_angle"]) - phase(last_diag["main_angle"]))
            am_delta = wrap_pm_half(last_am - first_am)
            print(f"  d(aux-main) delta:      {turns_to_deg(am_delta):+.2f}° ({am_delta:+.6f} turns)")

            if last_diag["vernier_pos"] is not None and first_diag["vernier_pos"] is not None:
                vd = last_diag["vernier_pos"] - first_diag["vernier_pos"]
                print(f"  vernier pos delta:      {turns_to_deg(vd):+.2f}° ({vd:+.6f} turns)")
            if last_diag["diag_pos"] is not None and first_diag["diag_pos"] is not None:
                ed = last_diag["diag_pos"] - first_diag["diag_pos"]
                print(f"  encoder pos delta:      {turns_to_deg(ed):+.2f}° ({ed:+.6f} turns)")

            print(f"  first main_angle={first_diag['main_angle']} aux_angle={first_diag['aux_angle']}")
            print(f"  last  main_angle={last_diag['main_angle']} aux_angle={last_diag['aux_angle']}")

        print("\nStopping motor and returning to IDLE...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        time.sleep(0.6)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        time.sleep(0.5)

        final = latest_heartbeat(bus, args, timeout=1.0)
        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
        print(f"Final heartbeat: {final}")
        print(f"Final status: {status}")
        print_error_summary("Final errors:", read_error_summary(bus, args))

        if samples:
            output_turns = samples[-1][1] - samples[0][1]
            finite_vel = [s[2] for s in samples if math.isfinite(s[2])]
            finite_vbus = [s[3] for s in samples if math.isfinite(s[3])]
            finite_ibus = [s[4] for s in samples if math.isfinite(s[4])]
            print(
                f"Output position delta (controller): {output_turns:.5f} turns = {output_turns * 360.0:.2f} deg"
            )
            if finite_vel:
                print(f"Max |velocity|: {max(abs(v) for v in finite_vel):.5f} turns/s")
            if finite_ibus and finite_vbus:
                print(
                    f"Bus current: {min(finite_ibus):.5f} .. {max(finite_ibus):.5f} A; "
                    f"Bus voltage: {min(finite_vbus):.3f} .. {max(finite_vbus):.3f} V"
                )

        if final is None or final["axis_state"] != AXIS_STATE_IDLE or final["axis_error"]:
            raise RuntimeError("Axis did not return cleanly to IDLE")
        print("PASS: velocity loop with angle trace completed")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        try:
            for _ in range(20):
                set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
                time.sleep(0.02)
            set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            if args.clear_at_end:
                time.sleep(0.2)
                clear_errors(bus, args.node_id, args.extended_id)
        except Exception as ex:
            print(f"WARNING: failed to send stop/IDLE during cleanup: {ex}")
        raise
    finally:
        if motion_started:
            try:
                set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
                set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            except Exception:
                pass
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
