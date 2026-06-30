#!/usr/bin/env python3
import argparse
import math
import struct
import time

from common import (
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_CONTROLLER_ERROR,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_IQ,
    CMD_GET_MOTOR_ERROR,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_INT32,
    clear_errors,
    get_basic_config,
    get_calib_result,
    get_status_ex,
    open_bus,
    request_two_floats,
    request_u32,
    request_u64,
    send,
    set_basic_config,
    set_requested_state,
    wait_heartbeat,
)


CMD_GET_ENCODER_COUNT = 0x00A
ENCODER_MODE_SPI_ABS_MT6826S = 0x105
ENCODER_MODE_SPI_ABS_MT6826S_VERNIER = 0x106


ENCODER_ERROR_BITS = [
    (0x00000001, "UNSTABLE_GAIN"),
    (0x00000002, "CPR_POLEPAIRS_MISMATCH"),
    (0x00000004, "NO_RESPONSE"),
    (0x00000008, "UNSUPPORTED_ENCODER_MODE"),
    (0x00000020, "INDEX_NOT_FOUND_YET"),
    (0x00000040, "ABS_SPI_TIMEOUT"),
    (0x00000080, "ABS_SPI_COM_FAIL"),
    (0x00000100, "ABS_SPI_NOT_READY"),
]


def decode_bits(value, table):
    names = [name for bit, name in table if value and value & bit]
    return ",".join(names) if names else "NONE"


def read_encoder_count(bus, node_id, extended_id=False, timeout=0.2):
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
            return struct.unpack("<ii", data[:8])
    return None


def read_config_int(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"{name}: TIMEOUT")
        return None
    status = EXT_STATUS.get(resp["status"], resp["status"])
    value = resp["value_i"]
    print(f"{name}: {value} status={status} raw={resp['raw']}")
    return value if resp["status"] == 0 else None


def set_config_int(bus, args, param_id, value, name):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_INT32,
        value=value, extended_id=args.extended_id,
    )
    if resp is None:
        raise RuntimeError(f"{name}: no response")
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"{name} <- {value}: {status} raw={resp['raw']}")
    if resp["status"] != 0:
        raise RuntimeError(f"{name}: set failed with {status}")


def read_config_float(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        raise RuntimeError(f"{name}: no response")
    value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else float(resp["value_i"])
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"{name}: {value} status={status} raw={resp['raw']}")
    if resp["status"] != 0:
        raise RuntimeError(f"{name}: read failed with {status}")
    return value


def set_config_float(bus, args, param_id, value, name):
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


def sample_snapshot(bus, args, timeout=0.2):
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=timeout)
    count = read_encoder_count(bus, args.node_id, args.extended_id, timeout=timeout)
    _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, args.extended_id, timeout=timeout)
    _, bus_vi = request_two_floats(
        bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, args.extended_id, timeout=timeout
    )
    _, encoder_error = request_u32(
        bus, args.node_id, CMD_GET_ENCODER_ERROR, args.extended_id, timeout=timeout
    )

    return {
        "status": status,
        "count": count,
        "iq": iq,
        "bus_vi": bus_vi,
        "encoder_error": encoder_error,
    }


def count_delta(a, b):
    if a is None or b is None:
        return None
    return b - a


def wait_heartbeat_retry(bus, args, timeout=5.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.5)
        if hb is not None:
            last = hb
            if hb["axis_state"] != 0:
                return hb
    return last


def main():
    parser = argparse.ArgumentParser(
        description="Observe MT6826S encoder offset calibration over CANSimple."
    )
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--pole-pairs", type=int, default=14)
    parser.add_argument("--encoder-cpr", type=int, default=32768)
    parser.add_argument("--period", type=float, default=0.12)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--calibration-current", type=float)
    parser.add_argument("--current-lim", type=float)
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    if args.calibration_current is not None and args.calibration_current <= 0.0:
        raise ValueError("--calibration-current must be positive")
    if args.current_lim is not None and args.current_lim <= 0.0:
        raise ValueError("--current-lim must be positive")

    expected_delta = 8.0 * args.encoder_cpr / args.pole_pairs

    bus = open_bus(args.channel, args.bitrate)
    original_calibration_current = None
    original_current_lim = None
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat_retry(bus, args, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(
            f"Heartbeat: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}"
        )

        print("Config before:")
        mode = read_config_int(bus, args, 0x20, "encoder.mode")
        cpr = read_config_int(bus, args, 0x21, "encoder.cpr")
        pole_pairs = read_config_int(bus, args, 0x11, "motor.pole_pairs")
        read_config_int(bus, args, 0x22, "encoder.abs_spi_cs_gpio_pin")

        if mode is not None and mode not in (ENCODER_MODE_SPI_ABS_MT6826S, ENCODER_MODE_SPI_ABS_MT6826S_VERNIER):
            print(
                f"WARN: encoder.mode={mode}, expected "
                f"{ENCODER_MODE_SPI_ABS_MT6826S} or {ENCODER_MODE_SPI_ABS_MT6826S_VERNIER}"
            )
        if cpr != args.encoder_cpr:
            set_config_int(bus, args, 0x21, args.encoder_cpr, "encoder.cpr")
        if pole_pairs != args.pole_pairs:
            set_config_int(bus, args, 0x11, args.pole_pairs, "motor.pole_pairs")

        if args.calibration_current is not None:
            original_calibration_current = read_config_float(
                bus, args, 0x12, "motor.calibration_current"
            )
            set_config_float(
                bus, args, 0x12, args.calibration_current,
                "temporary motor.calibration_current",
            )
        if args.current_lim is not None:
            original_current_lim = read_config_float(bus, args, 0x14, "motor.current_lim")
            set_config_float(
                bus, args, 0x14, args.current_lim, "temporary motor.current_lim"
            )

        print_calibration_values(bus, args, "Calibration values before:")
        print(
            f"Expected forward scan response: {expected_delta:.1f} counts "
            f"({args.encoder_cpr} CPR, {args.pole_pairs} pole pairs)"
        )

        print("Clearing errors and requesting ENCODER_OFFSET_CALIBRATION...")
        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.2)
        start_snapshot = sample_snapshot(bus, args, timeout=0.5)
        start_count = start_snapshot["count"]
        print(f"Start count: {start_count}")

        set_requested_state(bus, args.node_id, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, args.extended_id)

        start_time = time.monotonic()
        last_state = None
        first_cal_count = None
        last_count = start_count
        max_excursion = 0
        max_abs_iq = 0.0
        rows = 0
        final_snapshot = None

        while time.monotonic() - start_time < args.timeout:
            snap = sample_snapshot(bus, args, timeout=0.25)
            final_snapshot = snap
            now = time.monotonic() - start_time
            status = snap["status"]
            count = snap["count"]
            iq = snap["iq"]
            bus_vi = snap["bus_vi"]
            encoder_error = snap["encoder_error"]

            state = status["axis_state"] if status else None
            axis_error = status["axis_error"] if status else None
            if state != last_state:
                print(
                    f"  {now:5.2f}s state={state}({AXIS_STATES.get(state, '?')}) "
                    f"axis_error=0x{axis_error or 0:08X}"
                )
                last_state = state

            if count is not None:
                if first_cal_count is None and state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
                    first_cal_count = count
                last_count = count
                if first_cal_count is not None:
                    max_excursion = max(max_excursion, abs(count[0] - first_cal_count[0]))

            if iq is not None:
                max_abs_iq = max(max_abs_iq, abs(iq[0]), abs(iq[1]))

            shadow = count[0] if count else None
            cpr_count = count[1] if count else None
            iq_text = "TIMEOUT" if iq is None else f"{iq[0]: .3f}/{iq[1]: .3f}"
            bus_text = "TIMEOUT" if bus_vi is None else f"{bus_vi[0]:.2f}V/{bus_vi[1]: .3f}A"
            err_text = "TIMEOUT" if encoder_error is None else f"0x{encoder_error:08X}"
            delta_text = ""
            if first_cal_count is not None and count is not None:
                delta_text = f" delta={count[0] - first_cal_count[0]:7d}"

            print(
                f"    t={now:5.2f}s shadow={shadow} cpr={cpr_count}{delta_text} "
                f"Iq={iq_text} bus={bus_text} enc_err={err_text}"
            )
            rows += 1

            if status and state == AXIS_STATE_IDLE and now > 1.0:
                break
            if encoder_error:
                # Keep one extra sample after the error becomes visible.
                time.sleep(args.period)
                final_snapshot = sample_snapshot(bus, args, timeout=0.25)
                break
            time.sleep(args.period)

        print("Summary:")
        print(f"  samples: {rows}")
        print(f"  max |Iq| observed: {max_abs_iq:.3f} A")
        print(f"  start_count: {start_count}")
        print(f"  first_cal_count: {first_cal_count}")
        print(f"  last_count: {last_count}")

        if first_cal_count is not None and last_count is not None:
            actual = max_excursion
            ratio = actual / expected_delta if expected_delta else math.nan
            inferred_pole_pairs = 8.0 * args.encoder_cpr / actual if actual else math.nan
            print(f"  measured max scan excursion: {actual} counts")
            print(f"  final delta after return scan: {abs(count_delta(first_cal_count[0], last_count[0]))} counts")
            print(f"  expected scan response: {expected_delta:.1f} counts")
            print(f"  measured/expected: {ratio:.3f}")
            print(f"  inferred pole_pairs from response: {inferred_pole_pairs:.2f}")

        if final_snapshot and final_snapshot["status"]:
            status = final_snapshot["status"]
            print(
                f"  final state={status['axis_state']}({AXIS_STATES.get(status['axis_state'], '?')}) "
                f"axis_error=0x{status['axis_error']:08X}"
            )

        for cmd_id, name in [
            (CMD_GET_MOTOR_ERROR, "motor_error"),
            (CMD_GET_ENCODER_ERROR, "encoder_error"),
            (CMD_GET_CONTROLLER_ERROR, "controller_error"),
        ]:
            _, value = request_u64(bus, args.node_id, cmd_id, args.extended_id, timeout=1.0)
            if value is None:
                print(f"  {name}: TIMEOUT")
            elif name == "encoder_error":
                print(f"  {name}: 0x{value:016X} {decode_bits(value, ENCODER_ERROR_BITS)}")
            else:
                print(f"  {name}: 0x{value:016X}")

        print_calibration_values(bus, args, "Calibration values after:")

        if args.clear_at_end:
            print("Clearing errors at end...")
            clear_errors(bus, args.node_id, args.extended_id)

        return 0
    finally:
        try:
            set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            time.sleep(0.1)
            if original_calibration_current is not None:
                set_config_float(
                    bus, args, 0x12, original_calibration_current,
                    "restore motor.calibration_current",
                )
            if original_current_lim is not None:
                set_config_float(
                    bus, args, 0x14, original_current_lim, "restore motor.current_lim"
                )
        except Exception as ex:
            print(f"WARNING: failed to restore temporary motor limits: {ex}")
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
