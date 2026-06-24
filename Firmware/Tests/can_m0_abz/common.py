#!/usr/bin/env python3
import math
import struct
import time

import can


CMD_HEARTBEAT = 0x001
CMD_GET_MOTOR_ERROR = 0x003
CMD_GET_ENCODER_ERROR = 0x004
CMD_SET_REQUESTED_STATE = 0x007
CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODES = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_SET_INPUT_VEL = 0x00D
CMD_SET_LIMITS = 0x00F
CMD_START_ANTICOGGING = 0x010
CMD_GET_IQ = 0x014
CMD_GET_BUS_VOLTAGE_CURRENT = 0x017
CMD_CLEAR_ERRORS = 0x018
CMD_SET_LINEAR_COUNT = 0x019
CMD_SET_POS_GAIN = 0x01A
CMD_SET_VEL_GAINS = 0x01B
CMD_GET_CONTROLLER_ERROR = 0x01D
CMD_EXTENDED = 0x01E

AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1

EXT_TYPE_FLOAT32 = 1
EXT_TYPE_INT32 = 2
EXT_TYPE_UINT32 = 3

EXT_STATUS = {
    0: "OK",
    1: "UNKNOWN",
    2: "READONLY",
    3: "INVALID_TYPE",
    4: "INVALID_VALUE",
    5: "BUSY_ARMED",
}

AXIS_STATES = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
    9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
}


def arb_id(node_id, cmd_id):
    return (node_id << 5) | cmd_id


def open_bus(channel, bitrate):
    return can.Bus(interface="pcan", channel=channel, bitrate=bitrate)


def send(bus, node_id, cmd_id, data=b"", extended_id=False):
    bus.send(can.Message(
        arbitration_id=arb_id(node_id, cmd_id),
        is_extended_id=extended_id,
        data=data,
        dlc=len(data),
    ))


def recv_matching(bus, node_id, cmd_id, extended_id=False, timeout=1.0, first_byte=None, not_data=None):
    expected_id = arb_id(node_id, cmd_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(deadline - time.monotonic())
        if msg is None:
            continue
        if msg.arbitration_id != expected_id or msg.is_extended_id != extended_id or msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        if first_byte is not None and (not data or data[0] != first_byte):
            continue
        if not_data is not None and data == not_data:
            continue
        return data
    return None


def wait_heartbeat(bus, node_id, extended_id=False, timeout=5.0):
    data = recv_matching(bus, node_id, CMD_HEARTBEAT, extended_id, timeout)
    return decode_heartbeat(data) if data else None


def decode_heartbeat(data):
    axis_error, axis_state, motor_flags, encoder_flags, controller_flags = struct.unpack("<IBBBB", data[:8])
    return {
        "raw": data.hex(" "),
        "axis_error": axis_error,
        "axis_state": axis_state,
        "motor_error_flag": bool(motor_flags & 0x01),
        "encoder_error_flag": bool(encoder_flags & 0x01),
        "controller_error_flag": bool(controller_flags & 0x01),
        "trajectory_done": bool(controller_flags & 0x80),
        "motor_flags": motor_flags,
        "encoder_flags": encoder_flags,
        "controller_flags": controller_flags,
    }


def request_u32(bus, node_id, cmd_id, extended_id=False, timeout=1.0):
    send(bus, node_id, cmd_id, b"", extended_id)
    data = recv_matching(bus, node_id, cmd_id, extended_id, timeout)
    return (data, struct.unpack("<I", data[:4])[0]) if data else (None, None)


def request_u64(bus, node_id, cmd_id, extended_id=False, timeout=1.0):
    send(bus, node_id, cmd_id, b"", extended_id)
    data = recv_matching(bus, node_id, cmd_id, extended_id, timeout)
    return (data, struct.unpack("<Q", data[:8])[0]) if data else (None, None)


def request_two_floats(bus, node_id, cmd_id, extended_id=False, timeout=1.0):
    send(bus, node_id, cmd_id, b"", extended_id)
    data = recv_matching(bus, node_id, cmd_id, extended_id, timeout)
    return (data, struct.unpack("<ff", data[:8])) if data else (None, None)


def ext_request(bus, node_id, sub_cmd, item=0, req_type=0, value=0, value_float=None,
                extended_id=False, timeout=1.0):
    if value_float is None:
        payload = struct.pack("<BBBBi", sub_cmd, item, req_type, 0, int(value))
    else:
        payload = struct.pack("<BBBBf", sub_cmd, item, req_type, 0, float(value_float))

    send(bus, node_id, CMD_EXTENDED, payload, extended_id)
    data = recv_matching(bus, node_id, CMD_EXTENDED, extended_id, timeout, first_byte=sub_cmd, not_data=payload)
    if data is None:
        return None

    sub_cmd_r, b1, b2, b3, value_i = struct.unpack("<BBBBi", data[:8])
    value_u = struct.unpack("<BBBBI", data[:8])[4]
    value_f = struct.unpack("<f", data[4:8])[0]
    return {
        "raw": data.hex(" "),
        "sub_cmd": sub_cmd_r,
        "byte1": b1,
        "status": b2,
        "type": b3,
        "value_i": value_i,
        "value_u": value_u,
        "value_f": value_f,
    }


def get_status_ex(bus, node_id, extended_id=False, timeout=1.0):
    resp = ext_request(bus, node_id, 0x01, extended_id=extended_id, timeout=timeout)
    if resp is None:
        return None
    flags = resp["type"]
    return {
        "raw": resp["raw"],
        "status": resp["byte1"],
        "axis_state": resp["status"],
        "flags": flags,
        "axis_error": resp["value_u"],
        "motor_calibrated": bool(flags & 0x01),
        "encoder_ready": bool(flags & 0x02),
        "trajectory_done": bool(flags & 0x20),
    }


def set_basic_config(bus, node_id, param_id, req_type, value=0, value_float=None, extended_id=False):
    return ext_request(bus, node_id, 0x07, item=param_id, req_type=req_type,
                       value=value, value_float=value_float, extended_id=extended_id)


def get_basic_config(bus, node_id, param_id, extended_id=False):
    return ext_request(bus, node_id, 0x06, item=param_id, extended_id=extended_id)


def get_calib_result(bus, node_id, item_id, extended_id=False):
    resp = ext_request(bus, node_id, 0x04, item=item_id, extended_id=extended_id)
    if resp is None:
        return None
    value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_i"]
    return resp, value


def set_precalibrated(bus, node_id, flags=0x03, extended_id=False):
    return ext_request(bus, node_id, 0x02, item=flags, extended_id=extended_id)


def save_configuration(bus, node_id, extended_id=False, timeout=2.0):
    return ext_request(bus, node_id, 0x03, extended_id=extended_id, timeout=timeout)


def get_anticogging_status(bus, node_id, item_id, value=0, extended_id=False, timeout=1.0):
    return ext_request(bus, node_id, 0x08, item=item_id, req_type=EXT_TYPE_UINT32,
                       value=value, extended_id=extended_id, timeout=timeout)


def set_anticogging_config(bus, node_id, item_id, req_type, value=0, value_float=None, extended_id=False):
    return ext_request(bus, node_id, 0x09, item=item_id, req_type=req_type,
                       value=value, value_float=value_float, extended_id=extended_id)


def decode_anticogging_flags(flags):
    return {
        "calib_anticogging": bool(flags & 0x01),
        "anticogging_valid": bool(flags & 0x02),
        "pre_calibrated": bool(flags & 0x04),
        "anticogging_enabled": bool(flags & 0x08),
    }


def clear_errors(bus, node_id, extended_id=False):
    send(bus, node_id, CMD_CLEAR_ERRORS, b"\x00" * 8, extended_id)


def set_linear_count(bus, node_id, count, extended_id=False):
    send(bus, node_id, CMD_SET_LINEAR_COUNT, struct.pack("<i", int(count)), extended_id)


def set_requested_state(bus, node_id, state, extended_id=False):
    send(bus, node_id, CMD_SET_REQUESTED_STATE, struct.pack("<I", state), extended_id)


def set_controller_modes(bus, node_id, control_mode, input_mode, extended_id=False):
    send(bus, node_id, CMD_SET_CONTROLLER_MODES, struct.pack("<ii", control_mode, input_mode), extended_id)


def set_input_vel(bus, node_id, vel_turns_per_s, torque_ff=0.0, extended_id=False):
    send(bus, node_id, CMD_SET_INPUT_VEL, struct.pack("<ff", vel_turns_per_s, torque_ff), extended_id)


def set_input_pos(bus, node_id, pos_turns, vel_ff_turns_per_s=0.0, torque_ff=0.0, extended_id=False):
    vel_raw = int(round(vel_ff_turns_per_s / 0.001))
    torque_raw = int(round(torque_ff / 0.001))
    vel_raw = max(-32768, min(32767, vel_raw))
    torque_raw = max(-32768, min(32767, torque_raw))
    send(bus, node_id, CMD_SET_INPUT_POS, struct.pack("<fhh", pos_turns, vel_raw, torque_raw), extended_id)


def set_limits(bus, node_id, vel_limit_turns_per_s, current_lim_amps, extended_id=False):
    send(bus, node_id, CMD_SET_LIMITS, struct.pack("<ff", vel_limit_turns_per_s, current_lim_amps), extended_id)


def start_anticogging(bus, node_id, extended_id=False):
    send(bus, node_id, CMD_START_ANTICOGGING, b"", extended_id)


def set_pos_gain(bus, node_id, pos_gain, extended_id=False):
    send(bus, node_id, CMD_SET_POS_GAIN, struct.pack("<f", pos_gain), extended_id)


def set_vel_gains(bus, node_id, vel_gain, vel_integrator_gain, extended_id=False):
    send(bus, node_id, CMD_SET_VEL_GAINS, struct.pack("<ff", vel_gain, vel_integrator_gain), extended_id)


def check_finite(value, name):
    if not math.isfinite(value):
        raise RuntimeError(f"{name} is not finite: {value}")
