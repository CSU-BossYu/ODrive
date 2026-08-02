"""Shared CANSimple product codec and quarantined calibration diagnostics.

Product message IDs come from the generated Phase 7 contract.  Legacy 0x1e
items are retained only for the deferred calibration/estimator experiments;
new tests must use product management and USB scope/fault diagnostics.
"""

from __future__ import annotations

import math
import struct
import time

from .can_product_schema_generated import ProductCanMessageId


CMD_HEARTBEAT = int(ProductCanMessageId.HEARTBEAT)
CMD_GET_MOTOR_ERROR = int(ProductCanMessageId.GET_MOTOR_ERROR)
CMD_GET_ENCODER_ERROR = int(ProductCanMessageId.GET_ENCODER_ERROR)
CMD_SET_REQUESTED_STATE = int(ProductCanMessageId.SET_AXIS_STATE)
CMD_GET_ENCODER_ESTIMATES = int(ProductCanMessageId.ENCODER_ESTIMATES)
CMD_SET_CONTROLLER_MODES = int(ProductCanMessageId.SET_CONTROLLER_MODE)
CMD_SET_INPUT_POS = int(ProductCanMessageId.SET_INPUT_POS)
CMD_SET_INPUT_VEL = int(ProductCanMessageId.SET_INPUT_VEL)
CMD_SET_INPUT_TORQUE = int(ProductCanMessageId.SET_INPUT_TORQUE)
CMD_SET_LIMITS = int(ProductCanMessageId.SET_LIMITS)
CMD_GET_IQ = int(ProductCanMessageId.IQ)
CMD_GET_BUS_VOLTAGE_CURRENT = int(ProductCanMessageId.BUS_VOLTAGE_CURRENT)
CMD_CLEAR_ERRORS = int(ProductCanMessageId.CLEAR_ERRORS)
CMD_SET_LINEAR_COUNT = int(ProductCanMessageId.SET_LINEAR_COUNT)
CMD_SET_POS_GAIN = int(ProductCanMessageId.SET_POS_GAIN)
CMD_SET_VEL_GAINS = int(ProductCanMessageId.SET_VEL_GAINS)
CMD_GET_CONTROLLER_ERROR = int(ProductCanMessageId.GET_CONTROLLER_ERROR)
CMD_EXTENDED = int(ProductCanMessageId.LEGACY_EXTENDED)

AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CONTROL_MODE_TORQUE_CONTROL = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1

EXT_TYPE_FLOAT32 = 1
EXT_TYPE_INT32 = 2
EXT_TYPE_UINT32 = 3

EXT_STATUS = {0: "OK", 1: "UNKNOWN", 2: "READONLY", 3: "INVALID_TYPE",
              4: "INVALID_VALUE", 5: "BUSY_ARMED"}
AXIS_STATES = {0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
               3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
               7: "ENCODER_OFFSET_CALIBRATION", 8: "CLOSED_LOOP_CONTROL",
               9: "LOCKIN_SPIN"}


def arb_id(node_id, cmd_id):
    return (node_id << 5) | cmd_id


def open_bus(channel, bitrate):
    import can
    return can.Bus(interface="pcan", channel=channel, bitrate=bitrate)


def send(bus, node_id, cmd_id, data=b"", extended_id=False):
    import can
    bus.send(can.Message(arbitration_id=arb_id(node_id, cmd_id),
                         is_extended_id=extended_id, data=data, dlc=len(data)))


def recv_matching(bus, node_id, cmd_id, extended_id=False, timeout=1.0,
                  first_byte=None, not_data=None):
    expected_id = arb_id(node_id, cmd_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(max(0.0, deadline - time.monotonic()))
        if msg is None or msg.arbitration_id != expected_id:
            continue
        if msg.is_extended_id != extended_id or msg.is_remote_frame:
            continue
        data = bytes(msg.data)
        if first_byte is not None and (not data or data[0] != first_byte):
            continue
        if not_data is not None and data == not_data:
            continue
        return data
    return None


def decode_heartbeat(data):
    axis_error, axis_state, motor_flags, encoder_flags, controller_flags = \
        struct.unpack("<IBBBB", data[:8])
    return {"raw": data.hex(" "), "axis_error": axis_error,
            "axis_state": axis_state, "motor_error_flag": bool(motor_flags & 1),
            "encoder_error_flag": bool(encoder_flags & 1),
            "controller_error_flag": bool(controller_flags & 1),
            "trajectory_done": bool(controller_flags & 0x80),
            "motor_flags": motor_flags, "encoder_flags": encoder_flags,
            "controller_flags": controller_flags}


def wait_heartbeat(bus, node_id, extended_id=False, timeout=5.0):
    data = recv_matching(bus, node_id, CMD_HEARTBEAT, extended_id, timeout)
    return decode_heartbeat(data) if data else None


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


def ext_request(bus, node_id, sub_cmd, item=0, req_type=0, value=0,
                value_float=None, extended_id=False, timeout=1.0):
    payload = (struct.pack("<BBBBi", sub_cmd, item, req_type, 0, int(value))
               if value_float is None else
               struct.pack("<BBBBf", sub_cmd, item, req_type, 0,
                           float(value_float)))
    send(bus, node_id, CMD_EXTENDED, payload, extended_id)
    data = recv_matching(bus, node_id, CMD_EXTENDED, extended_id, timeout,
                         first_byte=sub_cmd, not_data=payload)
    if data is None:
        return None
    sub_cmd_r, b1, b2, b3, value_i = struct.unpack("<BBBBi", data[:8])
    return {"raw": data.hex(" "), "sub_cmd": sub_cmd_r, "byte1": b1,
            "status": b2, "type": b3, "value_i": value_i,
            "value_u": struct.unpack("<BBBBI", data[:8])[4],
            "value_f": struct.unpack("<f", data[4:8])[0]}


def get_status_ex(bus, node_id, extended_id=False, timeout=1.0):
    resp = ext_request(bus, node_id, 0x01, extended_id=extended_id,
                       timeout=timeout)
    if resp is None:
        return None
    flags = resp["type"]
    return {"raw": resp["raw"], "status": resp["byte1"],
            "axis_state": resp["status"], "flags": flags,
            "axis_error": resp["value_u"],
            "motor_calibrated": bool(flags & 1),
            "encoder_ready": bool(flags & 2),
            "trajectory_done": bool(flags & 0x20)}


def set_basic_config(bus, node_id, param_id, req_type, value=0,
                     value_float=None, extended_id=False):
    return ext_request(bus, node_id, 0x07, item=param_id, req_type=req_type,
                       value=value, value_float=value_float,
                       extended_id=extended_id)


def get_basic_config(bus, node_id, param_id, extended_id=False):
    return ext_request(bus, node_id, 0x06, item=param_id,
                       extended_id=extended_id)


def get_calib_result(bus, node_id, item_id, extended_id=False):
    resp = ext_request(bus, node_id, 0x04, item=item_id,
                       extended_id=extended_id)
    if resp is None:
        return None
    value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_i"]
    return resp, value


def set_precalibrated(bus, node_id, flags=0x03, extended_id=False):
    return ext_request(bus, node_id, 0x02, item=flags,
                       extended_id=extended_id)


def save_configuration(bus, node_id, extended_id=False, timeout=2.0):
    return ext_request(bus, node_id, 0x03, extended_id=extended_id,
                       timeout=timeout)


def get_anticogging_status(bus, node_id, item_id, value=0,
                           extended_id=False, timeout=1.0):
    return ext_request(bus, node_id, 0x08, item=item_id,
                       req_type=EXT_TYPE_UINT32, value=value,
                       extended_id=extended_id, timeout=timeout)


def set_anticogging_config(bus, node_id, item_id, req_type, value=0,
                           value_float=None, extended_id=False):
    return ext_request(bus, node_id, 0x09, item=item_id, req_type=req_type,
                       value=value, value_float=value_float,
                       extended_id=extended_id)


def decode_anticogging_flags(flags):
    return {"calib_anticogging": bool(flags & 1),
            "anticogging_valid": bool(flags & 2),
            "pre_calibrated": bool(flags & 4),
            "anticogging_enabled": bool(flags & 8)}


def clear_errors(bus, node_id, extended_id=False):
    send(bus, node_id, CMD_CLEAR_ERRORS, b"\x00" * 8, extended_id)


def set_linear_count(bus, node_id, count, extended_id=False):
    send(bus, node_id, CMD_SET_LINEAR_COUNT, struct.pack("<i", int(count)),
         extended_id)


def set_requested_state(bus, node_id, state, extended_id=False):
    send(bus, node_id, CMD_SET_REQUESTED_STATE, struct.pack("<I", state),
         extended_id)


def set_controller_modes(bus, node_id, control_mode, input_mode,
                         extended_id=False):
    send(bus, node_id, CMD_SET_CONTROLLER_MODES,
         struct.pack("<ii", control_mode, input_mode), extended_id)


def set_input_vel(bus, node_id, vel_turns_per_s, torque_ff=0.0,
                  extended_id=False):
    send(bus, node_id, CMD_SET_INPUT_VEL,
         struct.pack("<ff", vel_turns_per_s, torque_ff), extended_id)


def set_input_torque(bus, node_id, torque_nm, extended_id=False):
    send(bus, node_id, CMD_SET_INPUT_TORQUE, struct.pack("<f", torque_nm),
         extended_id)


def set_input_pos(bus, node_id, pos_turns, vel_ff_turns_per_s=0.0,
                  torque_ff=0.0, extended_id=False):
    vel_raw = max(-32768, min(32767, int(round(vel_ff_turns_per_s / 0.001))))
    torque_raw = max(-32768, min(32767, int(round(torque_ff / 0.001))))
    send(bus, node_id, CMD_SET_INPUT_POS,
         struct.pack("<fhh", pos_turns, vel_raw, torque_raw), extended_id)


def set_limits(bus, node_id, vel_limit_turns_per_s, current_lim_amps,
               extended_id=False):
    send(bus, node_id, CMD_SET_LIMITS,
         struct.pack("<ff", vel_limit_turns_per_s, current_lim_amps),
         extended_id)


def set_pos_gain(bus, node_id, pos_gain, extended_id=False):
    send(bus, node_id, CMD_SET_POS_GAIN, struct.pack("<f", pos_gain), extended_id)


def set_vel_gains(bus, node_id, vel_gain, vel_integrator_gain,
                  extended_id=False):
    send(bus, node_id, CMD_SET_VEL_GAINS,
         struct.pack("<ff", vel_gain, vel_integrator_gain), extended_id)


def check_finite(value, name):
    if not math.isfinite(value):
        raise RuntimeError(f"{name} is not finite: {value}")
