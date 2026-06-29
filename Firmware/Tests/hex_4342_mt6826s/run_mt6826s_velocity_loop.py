#!/usr/bin/env python3
import argparse
import math
import time

from common import (
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
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
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
    wait_heartbeat,
)


PARAM_DC_MAX_NEGATIVE_CURRENT = 0x40
PARAM_DC_MAX_POSITIVE_CURRENT = 0x41


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
    (0x00000010, "ILLEGAL_HALL_STATE"),
    (0x00000020, "INDEX_NOT_FOUND_YET"),
    (0x00000040, "ABS_SPI_TIMEOUT"),
    (0x00000080, "ABS_SPI_COM_FAIL"),
    (0x00000100, "ABS_SPI_NOT_READY"),
    (0x00000200, "HALL_NOT_CALIBRATED_YET"),
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


def fmt_error(value, bits):
    if value is None:
        return "TIMEOUT"
    width = 16 if bits == 64 else 8
    return f"0x{value:0{width}X}"


def require_ready(bus, args):
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=2.0)
    if status is None:
        raise RuntimeError("GET_AXIS_STATUS_EX timeout")
    print(
        f"Status: raw={status['raw']} flags=0x{status['flags']:02X} "
        f"motor_calibrated={status['motor_calibrated']} "
        f"encoder_ready={status['encoder_ready']} axis_error=0x{status['axis_error']:08X}"
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
            print_error_summary("Errors while entering closed loop:", read_error_summary(bus, args))
            raise RuntimeError("Error while entering closed loop")
        time.sleep(0.1)
    raise RuntimeError("Timed out waiting for CLOSED_LOOP_CONTROL")


def main():
    parser = argparse.ArgumentParser(description="Run MT6826S velocity loop over CANSimple.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--velocity", type=float, default=0.3, help="Motor rotor velocity in turns/s.")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--ramp-time", type=float, default=1.5)
    parser.add_argument("--sample-period", type=float, default=0.25)
    parser.add_argument("--vel-limit", type=float, default=20.0)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--dc-max-negative-current", type=float, default=-0.5)
    parser.add_argument("--stop-negative-ibus", type=float, default=-0.35)
    parser.add_argument("--gear-ratio", type=float, default=42.0)
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
            f"axis_error=0x{hb['axis_error']:08X}"
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
        time.sleep(0.1)

        enter_closed_loop(bus, args)
        motion_started = True

        print(
            f"Commanding velocity {args.velocity:.4f} turns/s for {args.duration:.2f}s "
            f"(current_limit={args.current_limit:.3f}A)..."
        )
        start = time.monotonic()
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
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.2)

            pos = enc[0] if enc else float("nan")
            vel = enc[1] if enc else float("nan")
            iq_set = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            vbus = bus_vi[0] if bus_vi else float("nan")
            ibus = bus_vi[1] if bus_vi else float("nan")
            samples.append((t, pos, vel, vbus, ibus, iq_set, iq_meas))

            if enc is None and iq is None and bus_vi is None and hb is None:
                missed_telemetry += 1
            else:
                missed_telemetry = 0

            hb_tail = ""
            if hb is not None:
                hb_tail = (
                    f" state={hb['axis_state']} err=0x{hb['axis_error']:08X} "
                    f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
                )
            print(
                f"  t={t:5.2f}s cmd={cmd: .4f} pos={pos: .5f} vel={vel: .5f} "
                f"vbus={vbus: .2f} ibus={ibus: .4f} Iq={iq_set: .3f}/{iq_meas: .3f}{hb_tail}"
            )

            if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
                print_error_summary("Errors:", read_error_summary(bus, args))
                raise RuntimeError("Error observed during velocity loop")
            if missed_telemetry >= 3:
                raise RuntimeError("Lost CAN telemetry for 3 consecutive samples")
            if math.isfinite(ibus) and ibus < args.stop_negative_ibus:
                raise RuntimeError(f"Stopping due to ibus={ibus:.4f}A below threshold")
            time.sleep(args.sample_period)

        print("Stopping motor and returning to IDLE...")
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
            delta_motor = samples[-1][1] - samples[0][1]
            output_turns = delta_motor / args.gear_ratio
            finite_vel = [s[2] for s in samples if math.isfinite(s[2])]
            finite_vbus = [s[3] for s in samples if math.isfinite(s[3])]
            finite_ibus = [s[4] for s in samples if math.isfinite(s[4])]
            print(
                f"Motor position delta: {delta_motor:.5f} turns; "
                f"output estimate: {output_turns:.5f} turns = {output_turns * 360.0:.2f} deg"
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
        print("PASS: velocity loop completed")
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
