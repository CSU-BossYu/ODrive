#!/usr/bin/env python3
import argparse
import struct
import time

from common import (
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_CONTROLLER_ERROR,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_IQ,
    CMD_GET_MOTOR_ERROR,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    clear_errors,
    get_basic_config,
    get_calib_result,
    get_status_ex,
    open_bus,
    request_two_floats,
    request_u32,
    send,
    set_basic_config,
    set_requested_state,
    wait_heartbeat,
)


CMD_GET_ENCODER_COUNT = 0x00A

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
    (0x0000000000000001, "UNSTABLE_GAIN"),
    (0x0000000000000002, "CPR_POLEPAIRS_MISMATCH"),
    (0x0000000000000004, "NO_RESPONSE"),
    (0x0000000000000008, "UNSUPPORTED_ENCODER_MODE"),
    (0x0000000000000010, "ILLEGAL_HALL_STATE"),
    (0x0000000000000020, "INDEX_NOT_FOUND_YET"),
    (0x0000000000000040, "ABS_SPI_TIMEOUT"),
    (0x0000000000000080, "ABS_SPI_COM_FAIL"),
    (0x0000000000000100, "ABS_SPI_NOT_READY"),
    (0x0000000000000200, "HALL_NOT_CALIBRATED_YET"),
]

CONTROLLER_ERROR_BITS = [
    (0x0000000000000001, "UNSTABLE_GAIN"),
    (0x0000000000000002, "INVALID_INPUT_MODE"),
    (0x0000000000000004, "UNSTABLE_GAIN"),
    (0x0000000000000008, "INVALID_MIRROR_AXIS"),
    (0x0000000000000010, "INVALID_LOAD_ENCODER"),
    (0x0000000000000020, "INVALID_ESTIMATE"),
    (0x0000000000000040, "INVALID_CIRCULAR_RANGE"),
    (0x0000000000000080, "SPINOUT_DETECTED"),
]


def request_u64(bus, node_id, cmd_id, extended_id=False, timeout=1.0):
    send(bus, node_id, cmd_id, b"", extended_id)
    expected_id = (node_id << 5) | cmd_id
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(deadline - time.monotonic())
        if msg is None:
            continue
        if msg.arbitration_id != expected_id or msg.is_extended_id != extended_id or msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        if len(data) >= 8:
            return data, struct.unpack("<Q", data[:8])[0]
    return None, None


def read_encoder_count(bus, node_id, extended_id=False, timeout=1.0):
    send(bus, node_id, CMD_GET_ENCODER_COUNT, b"", extended_id)
    expected_id = (node_id << 5) | CMD_GET_ENCODER_COUNT
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(deadline - time.monotonic())
        if msg is None:
            continue
        if msg.arbitration_id != expected_id or msg.is_extended_id != extended_id or msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        if len(data) >= 8:
            return data, struct.unpack("<ii", data[:8])
    return None, None


def decode_bits(value, table):
    names = [name for bit, name in table if value & bit]
    return names if names else ["NONE"]


def read_basic_float(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"{name}: TIMEOUT")
        return None
    value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_i"]
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"{name}: {value} status={status} raw={resp['raw']}")
    return value if resp["status"] == 0 else None


def set_basic_float(bus, args, param_id, value, name):
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


def print_status(bus, args, label):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=args.timeout)
    print(f"{label}:")
    if hb is None:
        print("  heartbeat: TIMEOUT")
    else:
        state = AXIS_STATES.get(hb["axis_state"], "?")
        print(
            f"  heartbeat: state={hb['axis_state']}({state}) "
            f"axis_error=0x{hb['axis_error']:08X} "
            f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X} "
            f"raw={hb['raw']}"
        )

    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
    print(f"  status_ex: {status}")

    for cmd_id, name in [
        (CMD_GET_MOTOR_ERROR, "motor_error"),
        (CMD_GET_ENCODER_ERROR, "encoder_error"),
        (CMD_GET_CONTROLLER_ERROR, "controller_error"),
    ]:
        data, value = request_u64(bus, args.node_id, cmd_id, args.extended_id, timeout=args.timeout)
        if data is None:
            print(f"  {name}: TIMEOUT")
        else:
            suffix = ""
            if cmd_id == CMD_GET_MOTOR_ERROR:
                suffix = " " + ",".join(decode_bits(value, MOTOR_ERROR_BITS))
            elif cmd_id == CMD_GET_ENCODER_ERROR:
                suffix = " " + ",".join(decode_bits(value, ENCODER_ERROR_BITS))
            elif cmd_id == CMD_GET_CONTROLLER_ERROR:
                suffix = " " + ",".join(decode_bits(value, CONTROLLER_ERROR_BITS))
            print(f"  {name}: 0x{value:016X}{suffix} raw={data.hex(' ')}")

    _, bus_vi = request_two_floats(
        bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, args.extended_id, timeout=args.timeout
    )
    print(f"  bus_voltage/current: {bus_vi}")

    _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=args.timeout)
    print(f"  iq_setpoint/measured: {iq}")

    _, count = read_encoder_count(bus, args.node_id, args.extended_id, timeout=args.timeout)
    print(f"  encoder_count: {count}")


def print_calibration_values(bus, args, label):
    print(label)
    for item_id, name in [
        (0x01, "phase_resistance"),
        (0x02, "phase_inductance"),
        (0x03, "phase_offset"),
        (0x04, "direction"),
    ]:
        result = get_calib_result(bus, args.node_id, item_id, args.extended_id)
        if result is None:
            print(f"  {name}: TIMEOUT")
            continue
        resp, value = result
        status = EXT_STATUS.get(resp["status"], resp["status"])
        print(f"  {name}: {value} status={status} raw={resp['raw']}")


def read_error_values(bus, args):
    values = {}
    for cmd_id, name in [
        (CMD_GET_MOTOR_ERROR, "motor_error"),
        (CMD_GET_ENCODER_ERROR, "encoder_error"),
        (CMD_GET_CONTROLLER_ERROR, "controller_error"),
    ]:
        _, value = request_u64(bus, args.node_id, cmd_id, args.extended_id, timeout=args.timeout)
        values[name] = value
    return values


def preflight_ready_for_motor_test(bus, args):
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)
    errors = read_error_values(bus, args)
    _, count = read_encoder_count(bus, args.node_id, args.extended_id, timeout=args.timeout)

    if status is None:
        print("Preflight failed: status_ex timed out")
        return False

    ok = True
    if status["axis_error"] != 0:
        print(f"Preflight failed: axis_error=0x{status['axis_error']:08X}")
        ok = False

    encoder_error = errors.get("encoder_error")
    if encoder_error:
        names = ",".join(decode_bits(encoder_error, ENCODER_ERROR_BITS))
        print(f"Preflight failed: encoder_error=0x{encoder_error:016X} {names}")
        ok = False

    controller_error = errors.get("controller_error")
    if controller_error:
        names = ",".join(decode_bits(controller_error, CONTROLLER_ERROR_BITS))
        print(f"Preflight failed: controller_error=0x{controller_error:016X} {names}")
        ok = False

    if count is None:
        print("Preflight failed: encoder_count timed out")
        ok = False

    return ok


def main():
    parser = argparse.ArgumentParser(description="Safely diagnose ODrive motor phase wiring over CANSimple.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--test-current", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--run-timeout", type=float, default=12.0)
    parser.add_argument("--no-clear-start", action="store_true")
    parser.add_argument("--leave-errors", action="store_true")
    parser.add_argument("--force", action="store_true", help="Run motor calibration even if preflight sees existing errors.")
    args = parser.parse_args()

    if args.test_current <= 0.0:
        raise ValueError("--test-current must be positive")

    bus = open_bus(args.channel, args.bitrate)
    orig_calibration_current = None
    orig_current_lim = None

    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")

        print_status(bus, args, "Before")
        print("Motor config:")
        read_basic_float(bus, args, 0x10, "  motor_type")
        read_basic_float(bus, args, 0x11, "  pole_pairs")
        orig_calibration_current = read_basic_float(bus, args, 0x12, "  calibration_current")
        read_basic_float(bus, args, 0x13, "  resistance_calib_max_voltage")
        orig_current_lim = read_basic_float(bus, args, 0x14, "  current_lim")
        read_basic_float(bus, args, 0x15, "  torque_constant")
        print_calibration_values(bus, args, "Calibration values before:")

        test_current = args.test_current
        if orig_calibration_current is not None:
            test_current = min(test_current, float(orig_calibration_current))
        print(f"Using temporary calibration_current={test_current} A")
        set_basic_float(bus, args, 0x12, test_current, "  calibration_current")
        if orig_current_lim is not None and float(orig_current_lim) < test_current:
            set_basic_float(bus, args, 0x14, test_current + 1.0, "  current_lim")

        if not args.no_clear_start:
            print("Clearing errors...")
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.3)

        if not args.force and not preflight_ready_for_motor_test(bus, args):
            print("Skipping motor calibration because preflight failed. Fix the listed error first, or pass --force.")
            return 2

        print("Requesting AXIS_STATE_MOTOR_CALIBRATION...")
        set_requested_state(bus, args.node_id, AXIS_STATE_MOTOR_CALIBRATION, args.extended_id)

        start = time.monotonic()
        last_state = None
        last_print = 0.0
        final_hb = None
        while time.monotonic() - start < args.run_timeout:
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=1.0)
            now = time.monotonic() - start
            if hb is None:
                print(f"  {now:5.1f}s heartbeat timeout")
                continue

            final_hb = hb
            state = hb["axis_state"]
            if state != last_state or now - last_print > 2.0 or hb["axis_error"]:
                _, bus_vi = request_two_floats(
                    bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT,
                    args.extended_id, timeout=0.3,
                )
                print(
                    f"  {now:5.1f}s state={state}({AXIS_STATES.get(state, '?')}) "
                    f"axis_error=0x{hb['axis_error']:08X} "
                    f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X} "
                    f"bus={bus_vi}"
                )
                last_state = state
                last_print = now

            if state == AXIS_STATE_IDLE and now > 1.0:
                break

        print(f"Final heartbeat object: {final_hb}")
        print_status(bus, args, "After")
        print_calibration_values(bus, args, "Calibration values after:")

    finally:
        if orig_calibration_current is not None:
            try:
                set_basic_float(bus, args, 0x12, float(orig_calibration_current), "  restore calibration_current")
            except Exception as ex:
                print(f"WARNING: failed to restore calibration_current: {ex}")

        if orig_current_lim is not None:
            try:
                set_basic_float(bus, args, 0x14, float(orig_current_lim), "  restore current_lim")
            except Exception as ex:
                print(f"WARNING: failed to restore current_lim: {ex}")

        if not args.leave_errors:
            try:
                print("Clearing errors after diagnostic...")
                clear_errors(bus, args.node_id, args.extended_id)
                time.sleep(0.2)
            except Exception as ex:
                print(f"WARNING: failed to clear errors: {ex}")

        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
