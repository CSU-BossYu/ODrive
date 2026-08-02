"""Generated from docs/can_product_protocol_schema.json."""

from enum import IntEnum
import struct

PROTOCOL_VERSION = 0x0000010C

class ProductCanMessageId(IntEnum):
    NMT = 0x00
    HEARTBEAT = 0x01
    ESTOP = 0x02
    GET_MOTOR_ERROR = 0x03
    GET_ENCODER_ERROR = 0x04
    COMMAND_ACK = 0x05
    SET_AXIS_NODE_ID = 0x06
    SET_AXIS_STATE = 0x07
    MANAGEMENT_COMMAND = 0x08
    ENCODER_ESTIMATES = 0x09
    GET_ENCODER_COUNT = 0x0A
    SET_CONTROLLER_MODE = 0x0B
    SET_INPUT_POS = 0x0C
    SET_INPUT_VEL = 0x0D
    SET_INPUT_TORQUE = 0x0E
    SET_LIMITS = 0x0F
    PRODUCT_STATUS = 0x10
    SET_TRAJ_VEL_LIMIT = 0x11
    SET_TRAJ_ACCEL_LIMITS = 0x12
    SET_TRAJ_INERTIA = 0x13
    IQ = 0x14
    REBOOT = 0x16
    BUS_VOLTAGE_CURRENT = 0x17
    CLEAR_ERRORS = 0x18
    SET_LINEAR_COUNT = 0x19
    SET_POS_GAIN = 0x1A
    SET_VEL_GAINS = 0x1B
    GET_ADC_VOLTAGE = 0x1C
    GET_CONTROLLER_ERROR = 0x1D
    LEGACY_EXTENDED = 0x1E
    SET_MIT_CONTROL = 0x1F

def encode_management_command(request_id: int, command_type: int,
                              operation: int = 0, arg0: int = 0) -> bytes:
    if not 0 < request_id <= 0xFFFF:
        raise ValueError("CAN management request_id must be 1..65535")
    return struct.pack("<HBBI", request_id, command_type, operation,
                       arg0 & 0xFFFFFFFF)

def decode_command_ack(data: bytes) -> dict[str, int]:
    if len(data) != 8:
        raise ValueError("CAN command ACK must be 8 bytes")
    request_id, status, reserved, epoch, reason = struct.unpack("<HBBHH", data)
    if reserved != 0:
        raise ValueError("CAN command ACK reserved byte is non-zero")
    return {"request_id": request_id, "status": status,
            "state_epoch_low": epoch, "reason": reason}
