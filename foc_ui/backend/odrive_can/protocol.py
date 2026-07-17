"""ODrive CAN Simple protocol constants and codec.

Single source of truth for all CAN frame IDs, enums, error tables, and
encode/decode functions. Hand-written from:
  - Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md
  - tools/odrive-cansimple.dbc

Frame ID format: ``(node_id << 5) | command_id``
  - Node ID: 6 bits (0..63)
  - Command ID: 5 bits (0..31)

Data encoding:
  - Normal CAN Simple scalar fields: little-endian
  - float32: IEEE-754 little-endian
  - MIT packed control (0x01F): big-endian AK/T-Motor bit layout
"""

from __future__ import annotations

import struct
from enum import IntEnum
from typing import Any

# --------------------------------------------------------------------------- #
# Node ID
# --------------------------------------------------------------------------- #

NODE_ID_DEFAULT = 0

# --------------------------------------------------------------------------- #
# Command IDs  (CAN_PROTOCOL_PRODUCTION.md command map, lines 28-62)
# --------------------------------------------------------------------------- #


class CmdId(IntEnum):
    """CAN command ID (5-bit, 0x00..0x1F)."""
    NMT                     = 0x000
    HEARTBEAT               = 0x001
    ESTOP                   = 0x002
    GET_MOTOR_ERROR         = 0x003
    GET_ENCODER_ERROR       = 0x004
    SET_AXIS_NODE_ID        = 0x006
    SET_AXIS_STATE          = 0x007
    GET_ENCODER_ESTIMATES   = 0x009
    GET_ENCODER_COUNT       = 0x00A
    SET_CONTROLLER_MODE     = 0x00B
    SET_INPUT_POS           = 0x00C
    SET_INPUT_VEL           = 0x00D
    SET_INPUT_TORQUE        = 0x00E
    SET_LIMITS              = 0x00F
    SET_TRAJ_VEL_LIMIT      = 0x011
    SET_TRAJ_ACCEL_LIMITS   = 0x012
    SET_TRAJ_INERTIA        = 0x013
    GET_IQ                  = 0x014
    REBOOT                  = 0x016
    GET_BUS_VOLTAGE_CURRENT = 0x017
    CLEAR_ERRORS            = 0x018
    SET_LINEAR_COUNT        = 0x019
    SET_POS_GAIN            = 0x01A
    SET_VEL_GAINS           = 0x01B
    GET_ADC_VOLTAGE         = 0x01C
    GET_CONTROLLER_ERROR    = 0x01D
    EXTENDED_COMMAND        = 0x01E
    SET_MIT_CONTROL         = 0x01F


def make_frame_id(node_id: int, cmd_id: CmdId | int) -> int:
    """Build an 11-bit CAN arbitration ID from node_id and command_id."""
    return (node_id << 5) | (int(cmd_id) & 0x1F)


def parse_frame_id(arb_id: int) -> tuple[int, int]:
    """Extract (node_id, cmd_id) from an 11-bit arbitration ID."""
    return (arb_id >> 5, arb_id & 0x1F)


# --------------------------------------------------------------------------- #
# Axis State  (CAN_PROTOCOL_PRODUCTION.md / DBC VAL_ 1, line 154)
# --------------------------------------------------------------------------- #


class AxisState(IntEnum):
    UNDEFINED                  = 0
    IDLE                       = 1
    STARTUP_SEQUENCE           = 2
    FULL_CALIBRATION_SEQUENCE  = 3
    MOTOR_CALIBRATION          = 4
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL        = 8
    LOCKIN_SPIN                = 9


# --------------------------------------------------------------------------- #
# Control Mode  (CAN_PROTOCOL_PRODUCTION.md lines 68-75, DBC VAL_ 11)
# --------------------------------------------------------------------------- #


class ControlMode(IntEnum):
    VOLTAGE_CONTROL  = 0
    TORQUE_CONTROL   = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3


# --------------------------------------------------------------------------- #
# Input Mode  (CAN_PROTOCOL_PRODUCTION.md lines 77-89, DBC VAL_ 11)
# --------------------------------------------------------------------------- #


class InputMode(IntEnum):
    INACTIVE     = 0
    PASSTHROUGH  = 1
    VEL_RAMP     = 2
    POS_FILTER   = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ    = 5
    TORQUE_RAMP  = 6
    TUNING       = 8
    MIT          = 9


# --------------------------------------------------------------------------- #
# Error Flag Tables  (sparse bitmask: mask -> name)
# --------------------------------------------------------------------------- #

# Axis error (DBC VAL_ 1 Axis_Error, line 155)
AXIS_ERROR_BITS: dict[int, str] = {
    0x00001: 'INVALID_STATE',
    0x00040: 'MOTOR_FAILED',
    0x00100: 'ENCODER_FAILED',
    0x00200: 'CONTROLLER_FAILED',
    0x00800: 'WATCHDOG_TIMER_EXPIRED',
    0x01000: 'ESTOP_REQUESTED',
    0x02000: 'OVER_TEMP',
    0x04000: 'UNKNOWN_POSITION',
}

# Motor error (DBC VAL_ 3 Motor_Error, line 156)
MOTOR_ERROR_BITS: dict[int, str] = {
    0x00000001: 'PHASE_RESISTANCE_OUT_OF_RANGE',
    0x00000002: 'PHASE_INDUCTANCE_OUT_OF_RANGE',
    0x00000008: 'DRV_FAULT',
    0x00000010: 'CONTROL_DEADLINE_MISSED',
    0x00000080: 'MODULATION_MAGNITUDE',
    0x00000400: 'CURRENT_SENSE_SATURATION',
    0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00010000: 'MODULATION_IS_NAN',
    0x00020000: 'MOTOR_THERMISTOR_OVER_TEMP',
    0x00040000: 'FET_THERMISTOR_OVER_TEMP',
    0x00080000: 'TIMER_UPDATE_MISSED',
    0x00100000: 'CURRENT_MEASUREMENT_UNAVAILABLE',
    0x00200000: 'CONTROLLER_FAILED',
    0x00400000: 'I_BUS_OUT_OF_RANGE',
    0x00800000: 'BRAKE_RESISTOR_DISARMED',
    0x01000000: 'SYSTEM_LEVEL',
    0x02000000: 'BAD_TIMING',
    0x04000000: 'UNKNOWN_PHASE_ESTIMATE',
    0x08000000: 'UNKNOWN_PHASE_VEL',
    0x10000000: 'UNKNOWN_TORQUE',
    0x20000000: 'UNKNOWN_CURRENT_COMMAND',
    0x40000000: 'UNKNOWN_CURRENT_MEASUREMENT',
    0x80000000: 'UNKNOWN_VBUS_VOLTAGE',
    0x100000000: 'UNKNOWN_VOLTAGE_COMMAND',
    0x200000000: 'UNKNOWN_GAINS',
    0x400000000: 'CONTROLLER_INITIALIZING',
    0x800000000: 'UNBALANCED_PHASES',
}

# Encoder error (DBC VAL_ 4 Encoder_Error, line 157)
ENCODER_ERROR_BITS: dict[int, str] = {
    0x01: 'UNSTABLE_GAIN',
    0x02: 'CPR_POLEPAIRS_MISMATCH',
    0x04: 'NO_RESPONSE',
    0x08: 'UNSUPPORTED_ENCODER_MODE',
    0x40: 'ABS_SPI_TIMEOUT',
    0x80: 'ABS_SPI_COM_FAIL',
    0x100: 'ABS_SPI_NOT_READY',
    0x200: 'VERNIER_RESOLVER_FAIL',
}

# Controller error (DBC VAL_ 29 Controller_Error, line 161)
CONTROLLER_ERROR_BITS: dict[int, str] = {
    0x01: 'OVERSPEED',
    0x02: 'INVALID_INPUT_MODE',
    0x04: 'UNSTABLE_GAIN',
    0x20: 'INVALID_ESTIMATE',
    0x40: 'INVALID_CIRCULAR_RANGE',
    0x80: 'SPINOUT_DETECTED',
}

ODRIVE_ERROR_BITS: dict[int, str] = {
    0x00000001: 'CONTROL_ITERATION_MISSED',
    0x00000002: 'DC_BUS_UNDER_VOLTAGE',
    0x00000004: 'DC_BUS_OVER_VOLTAGE',
    0x00000008: 'DC_BUS_OVER_REGEN_CURRENT',
    0x00000010: 'DC_BUS_OVER_CURRENT',
    0x00000020: 'BRAKE_DEADTIME_VIOLATION',
    0x00000040: 'BRAKE_DUTY_CYCLE_NAN',
    0x00000080: 'INVALID_BRAKE_RESISTANCE',
}


# --------------------------------------------------------------------------- #
# Extended Command Constants  (CAN_PROTOCOL_PRODUCTION.md lines 127-231)
# --------------------------------------------------------------------------- #


class ExtSubCmd(IntEnum):
    GET_AXIS_STATUS_EX      = 0x01
    SET_PRECALIBRATED       = 0x02
    SAVE_CONFIGURATION      = 0x03
    GET_CALIB_RESULT        = 0x04
    GET_DEVICE_INFO         = 0x05
    GET_BASIC_CONFIG        = 0x06
    SET_BASIC_CONFIG        = 0x07
    GET_VERNIER_DIAGNOSTICS = 0x0A
    GET_CONTROL_CONFIG      = 0x0B
    SET_CONTROL_CONFIG      = 0x0C
    VERNIER_CALIBRATION     = 0x0D
    GET_FAULT_SNAPSHOT      = 0x0E
    CALIBRATION_SESSION     = 0x0F


class CalibrationSessionState(IntEnum):
    EMPTY      = 0
    COLLECTING = 1
    COLLECTED  = 2
    FITTING    = 3
    IDENTIFIED = 4
    VALIDATING = 5
    VALIDATED  = 6
    STAGED     = 7
    COMMITTED  = 8
    FAILED     = 9
    ABORTED    = 10
    STALE      = 11


class CalibrationProfile(IntEnum):
    FULL          = 0
    ELECTRICAL    = 1
    MECHANICAL    = 2
    VALIDATE_ONLY = 3


class CalibrationSessionItem(IntEnum):
    SCHEMA_VERSION   = 0x00
    SESSION_ID       = 0x01
    STATE            = 0x02
    STAGE            = 0x03
    FAILURE_CODE     = 0x04
    FLAGS            = 0x05
    TRANSITION_COUNT = 0x06
    REQUEST_OPTIONS  = 0x07
    PROGRESS_PERMILLE = 0x08
    BUFFERED_RECORDS = 0x09
    DROPPED_RECORDS  = 0x0A
    BUFFER_HIGH_WATERMARK = 0x0B
    BUFFER_CAPACITY  = 0x0C
    TRANSPORT_FRAMES_SENT = 0x0D
    TRANSPORT_QUEUE_RETRIES = 0x0E
    TRANSPORT_DISCONNECT_WAITS = 0x0F
    START            = 0x10
    ABORT            = 0x11
    RESULT_VALIDITY  = 0x20
    PHASE_RESISTANCE = 0x21
    PHASE_INDUCTANCE = 0x22
    ENCODER_DIRECTION = 0x23
    PHASE_OFFSET     = 0x24
    PHASE_OFFSET_FLOAT = 0x25
    EFFECTIVE_RATIO_SCALE = 0x26
    GEOMETRY_RAW_RMS = 0x27
    GEOMETRY_CORRECTED_RMS = 0x28
    GEOMETRY_DIRECTION_PEAK_TO_PEAK = 0x29
    GEOMETRY_USED_SAMPLES = 0x2A
    FLUX_LINKAGE = 0x2B
    TORQUE_CONSTANT = 0x2C
    FLUX_SAMPLE_STDDEV = 0x2D
    FLUX_USED_SAMPLES = 0x2E
    POLE_PAIRS = 0x2F
    OUTPUT_INERTIA = 0x30
    FRICTION_COULOMB_POS = 0x31
    FRICTION_COULOMB_NEG = 0x32
    FRICTION_VISCOUS_POS = 0x33
    FRICTION_VISCOUS_NEG = 0x34
    MECHANICAL_RESIDUAL_RMS_TORQUE = 0x35
    MECHANICAL_USED_SAMPLES = 0x36
    ELECTRICAL_DELAY = 0x37
    DELAY_RESIDUAL_PHASE_OFFSET = 0x38
    DELAY_RESIDUAL_RMS = 0x39
    DELAY_USED_SAMPLES = 0x3A
    MECHANICAL_ATTEMPTED_SAMPLES = 0x3B
    MECHANICAL_REJECTED_INVALID = 0x3C
    MECHANICAL_REJECTED_SATURATED = 0x3D
    MECHANICAL_REJECTED_LOW_VELOCITY = 0x3E
    MECHANICAL_MAX_ABS_VELOCITY = 0x3F
    MECHANICAL_REJECTED_TIMING = 0x40
    DELAY_ATTEMPTED_SAMPLES = 0x41
    DELAY_REJECTED_INVALID = 0x42
    DELAY_REJECTED_SATURATED = 0x43
    DELAY_REJECTED_SPEED = 0x44
    DELAY_REJECTED_EMF = 0x45
    DELAY_REJECTED_PHASE = 0x46
    DELAY_MAX_ABS_ELECTRICAL_SPEED = 0x47


class ExtStatus(IntEnum):
    OK            = 0
    UNKNOWN       = 1
    READONLY      = 2
    INVALID_TYPE  = 3
    INVALID_VALUE = 4
    BUSY_ARMED    = 5
    STORAGE_ERROR = 6


class ExtType(IntEnum):
    FLOAT32 = 1
    INT32   = 2
    UINT32  = 3


# ServoControlMode (ext item 0x5B) — business-layer mode mapping.
class ServoControlMode(IntEnum):
    TORQUE          = 0
    VELOCITY        = 1
    PROFILE_POSITION = 2
    MIT_REALTIME    = 3
    UNKNOWN         = 0xFF


# TimeoutAction (ext item 0x54).
class TimeoutAction(IntEnum):
    HOLD_LAST_POSITION   = 0
    QUICK_STOP           = 1
    QUICK_STOP_AND_HOLD  = 2
    TORQUE_ZERO          = 3
    FAULT_DISABLE        = 4


# Vernier diagnostic item IDs (sub_cmd 0x0A)
VERNIER_FOC_BAD_TIMING = 0x30
VERNIER_ADC_PRE        = 0x34
VERNIER_ADC_POST       = 0x35
VERNIER_DEADLINE_MISS  = 0x36
VERNIER_SPI_PAIR_BUSY  = 0x37
VERNIER_SPI_PAIR_OK    = 0x38
VERNIER_OUTPUT_PAIR_VEL_ESTIMATE = 0x2C
VERNIER_OUTPUT_LAST_AUX_CORRECTION = 0x2D

# OverspeedSnapshot item IDs (ext sub_cmd 0x0A, items 0x40-0x5F).
# Captured by firmware at the instant of ERROR_OVERSPEED and held until
# clear_errors. (item, field_name, is_float). Mirrors can_simple.cpp.
OVERSPEED_SNAPSHOT_ITEMS: list[tuple[int, str, bool]] = [
    (0x40, 'valid',                False),
    (0x41, 'control_loop_count',   False),
    (0x42, 'timestamp',            True),
    (0x43, 'vel_estimate',         True),
    (0x44, 'vel_limit',            True),
    (0x45, 'vel_limit_tolerance',  True),
    (0x46, 'pos_estimate_linear',  True),
    (0x47, 'pos_setpoint',         True),
    (0x48, 'vel_setpoint',         True),
    (0x49, 'input_pos',            True),
    (0x4A, 'input_vel',            True),
    (0x4B, 'input_mode',           False),
    (0x4C, 'control_mode',         False),
    (0x4D, 'resolver_state',       False),
    (0x4E, 'resolver_valid',       False),
    (0x4F, 'resolver_locked',      False),
    (0x50, 'resolver_accepted_aux',False),
    (0x51, 'resolver_degraded',    False),
    (0x52, 'resolver_position_turns', True),
    (0x53, 'resolver_residual',    True),
    (0x54, 'encoder_vel_estimate', True),
    (0x55, 'pair_sequence',        False),
    (0x56, 'pair_valid',           False),
    (0x57, 'output_estimate_valid',False),
    (0x58, 'output_pos_estimate',  True),
    (0x59, 'output_vel_estimate',  True),
    (0x5A, 'output_sample_dt',     True),
    (0x5B, 'output_pair_sequence', False),
    (0x5C, 'encoder_pos_estimate', True),
    (0x5D, 'pos_estimate_circular',True),
    (0x5E, 'torque_setpoint',      True),
    (0x5F, 'input_torque',         True),
]


# Control configuration item IDs (ext sub_cmd 0x0B Get / 0x0C Set).
# (item, name, is_float). 0x58/0x59/0x5A are readonly; 0x5B setter maps to
# (ControlMode, InputMode, TimeoutAction); 0x5C is the heartbeat watchdog.
# 0x5D exposes the position-loop integrator gain. 0x60-0x63 expose the
# controller velocity limit and enable gates. Items 0x6D-0x6F expose bounded
# ADRC trim. Items 0x70-0x7C expose friction compensation (0x70 pos/vel
# enable, 0x7C MIT enable, 0x71-0x7B Stribeck params; output-shaft
# Nm/turn/turn-s, vernier mode). All items 0x50-0x7C are persisted to NVM via
# save_configuration (cmd 0x03) and guarded by the armed guard (disarm to edit),
# except live-tunable gains explicitly allowed by the firmware.
CONTROL_CONFIG_ITEMS: list[tuple[int, str, bool]] = [
    (0x50, 'velocity_accel_limit',    True),
    (0x51, 'velocity_decel_limit',    True),
    (0x52, 'quick_stop_decel_limit',  True),
    (0x53, 'can_watchdog_timeout_ms', False),
    (0x54, 'timeout_action',          False),
    (0x55, 'profile_vel_limit',       True),
    (0x56, 'profile_accel_limit',     True),
    (0x57, 'profile_decel_limit',     True),
    (0x58, 'control_runtime_state',   False),  # readonly
    (0x59, 'last_timeout_reason',     False),  # readonly
    (0x5A, 'trajectory_done',         False),  # readonly
    (0x5B, 'servo_mode',              False),
    (0x5C, 'heartbeat_timeout_ms',    False),
    (0x5D, 'pos_integrator_gain',      True),
    (0x60, 'vel_limit',                True),
    (0x61, 'vel_limit_tolerance',      True),
    (0x62, 'enable_vel_limit',         False),
    (0x63, 'enable_torque_mode_vel_limit', False),
    # ADRC trim (bounded additive disturbance compensation).
    (0x6D, 'adrc_trim_slew_rate',      True),
    (0x6E, 'enable_adrc',              False),
    (0x6F, 'adrc_trim_torque_limit',   True),
    # Friction compensation (output-shaft Nm/turn/turn-s; vernier mode).
    (0x70, 'enable_friction_compensation',     False),
    (0x7C, 'enable_mit_friction_compensation', False),
    (0x71, 'friction_pos_deadband',      True),
    (0x72, 'friction_vel_deadband',      True),
    (0x73, 'friction_stribeck_vel',      True),
    (0x74, 'friction_static_pos',        True),
    (0x75, 'friction_static_neg',        True),
    (0x76, 'friction_coulomb_pos',       True),
    (0x77, 'friction_coulomb_neg',       True),
    (0x78, 'friction_viscous_pos',       True),
    (0x79, 'friction_viscous_neg',       True),
    (0x7A, 'friction_max_torque',        True),
    (0x7B, 'friction_torque_slew_rate',  True),
]

# control_runtime_state (0x58) flag bits.
CTRL_FLAG_ENABLED              = 1 << 0
CTRL_FLAG_RUNNING              = 1 << 1
CTRL_FLAG_HOLDING              = 1 << 2
CTRL_FLAG_QUICK_STOP_ACTIVE    = 1 << 3
CTRL_FLAG_COMM_TIMEOUT         = 1 << 4
CTRL_FLAG_CMD_WATCHDOG_EXPIRED = 1 << 5
CTRL_FLAG_MIT_FRAME_STALE      = 1 << 6
CTRL_FLAG_TRAJECTORY_DONE      = 1 << 7
CTRL_FLAG_HEARTBEAT_EXPIRED    = 1 << 8


# --------------------------------------------------------------------------- #
# Decode Functions  (CAN payload -> Python dict)
# --------------------------------------------------------------------------- #

def decode_heartbeat(data: bytes) -> dict[str, Any]:
    """Decode 0x001 Heartbeat (8 bytes, little-endian).

    DBC BO_ 1 (lines 39-45):
      byte0-3: Axis_Error (uint32 LE)
      byte4:   Axis_State (uint8)
      byte5:   Motor_Error_Flag (bit 0)
      byte6:   Encoder_Error_Flag (bit 0)
      byte7:   controller runtime flags:
               bit0 controller error, bit1 comm timeout,
               bit2 quick stop active, bit3 holding,
               bit4 command watchdog expired, bit5 MIT frame stale,
               bit6 running, bit7 trajectory done.
    """
    if len(data) < 8:
        return {}
    axis_error = struct.unpack_from('<I', data, 0)[0]
    controller_flags = data[7]
    return {
        'axis_error': axis_error,
        'axis_state': data[4],
        'motor_err_flag': bool(data[5] & 1),
        'encoder_err_flag': bool(data[6] & 1),
        'controller_err_flag': bool(controller_flags & 0x01),
        'comm_timeout': bool(controller_flags & 0x02),
        'quick_stop_active': bool(controller_flags & 0x04),
        'holding': bool(controller_flags & 0x08),
        'cmd_watchdog_expired': bool(controller_flags & 0x10),
        'mit_frame_stale': bool(controller_flags & 0x20),
        'running': bool(controller_flags & 0x40),
        'traj_done': bool(controller_flags & 0x80),
        'controller_flags': controller_flags,
    }


def decode_encoder_estimates(data: bytes) -> dict[str, float]:
    """Decode 0x009 Get_Encoder_Estimates (8 bytes).
    DBC BO_ 9 (lines 62-63): pos(float32 LE rev), vel(float32 LE rev/s)."""
    if len(data) < 8:
        return {}
    pos, vel = struct.unpack('<ff', data[:8])
    return {'pos_estimate': pos, 'vel_estimate': vel}


def decode_encoder_count(data: bytes) -> dict[str, int]:
    """Decode 0x00A Get_Encoder_Count (8 bytes).
    DBC BO_ 10: shadow_count(int32 LE), count_in_cpr(int32 LE)."""
    if len(data) < 8:
        return {}
    shadow, cpr = struct.unpack('<ii', data[:8])
    return {'shadow_count': shadow, 'count_in_cpr': cpr}


def decode_iq(data: bytes) -> dict[str, float]:
    """Decode 0x014 Get_Iq (8 bytes).
    DBC BO_ 20 (lines 102-103): iq_setpoint(float32 LE A), iq_measured(float32 LE A)."""
    if len(data) < 8:
        return {}
    iq_sp, iq_meas = struct.unpack('<ff', data[:8])
    return {'iq_setpoint': iq_sp, 'iq_measured': iq_meas}


def decode_bus_vi(data: bytes) -> dict[str, float]:
    """Decode 0x017 Get_Bus_Voltage_Current (8 bytes).
    DBC BO_ 23 (lines 110-111): vbus(float32 LE V), ibus(float32 LE A)."""
    if len(data) < 8:
        return {}
    vbus, ibus = struct.unpack('<ff', data[:8])
    return {'vbus': vbus, 'ibus': ibus}


def decode_motor_error(data: bytes) -> int:
    """Decode 0x003 Get_Motor_Error. Firmware sends a 64-bit LE error word
    (can_setSignal ... 64). The high bits include UNKNOWN_VOLTAGE_COMMAND,
    UNKNOWN_GAINS, CONTROLLER_INITIALIZING, UNBALANCED_PHASES — decoding as
    uint32 would silently drop those."""
    if len(data) >= 8:
        return struct.unpack_from('<Q', data, 0)[0]
    if len(data) >= 4:
        return struct.unpack_from('<I', data, 0)[0]
    return 0


def decode_encoder_error(data: bytes) -> int:
    """Decode 0x004 Get_Encoder_Error (4+ bytes). Returns uint32 LE error word."""
    if len(data) < 4:
        return 0
    return struct.unpack_from('<I', data, 0)[0]


def decode_controller_error(data: bytes) -> int:
    """Decode 0x01D Get_Controller_Error (4+ bytes). Returns uint32 LE error word."""
    if len(data) < 4:
        return 0
    return struct.unpack_from('<I', data, 0)[0]


def decode_extended_response(data: bytes) -> dict[str, Any]:
    """Decode 0x01E Extended_Command response (8 bytes).

    Layout:
      byte0: sub_cmd (uint8)
      byte1: item (uint8)
      byte2: status (uint8)
      byte3: type/aux (uint8)
      byte4-7: value (interpreted by type)
    """
    if len(data) < 8:
        return {'status': ExtStatus.UNKNOWN, 'error': 'short frame'}
    sub_cmd = data[0]
    item = data[1]
    status = data[2]
    ext_type = data[3]
    val_bytes = data[4:8]

    if ext_type == ExtType.FLOAT32:
        value = struct.unpack('<f', val_bytes)[0]
    elif ext_type == ExtType.INT32:
        value = struct.unpack('<i', val_bytes)[0]
    else:
        value = struct.unpack('<I', val_bytes)[0]

    return {
        'sub_cmd': sub_cmd,
        'item': item,
        'status': status,
        'type': ext_type,
        'value': value,
    }


# --------------------------------------------------------------------------- #
# Frame meaning decoder (for the live CAN monitor pane)
# --------------------------------------------------------------------------- #

def _mit_uint_to_float(raw: int, low: float, high: float, bits: int) -> float:
    """Inverse of _mit_float_to_uint: raw int -> physical float."""
    return low + (high - low) * raw / ((1 << bits) - 1)


def decode_frame_meaning(cmd_id: int, data: bytes, direction: str) -> str:
    """One-line human-readable decode of a CAN frame's payload.

    direction is 'rx' (firmware -> host) or 'tx' (host -> firmware). Returns
    '' for empty payloads or cmds without a decoder (the pane falls back to
    showing just the cmd name + raw hex on the left).
    """
    cmd = int(cmd_id)
    n = len(data)
    try:
        if cmd == CmdId.HEARTBEAT and direction == 'rx':
            hb = decode_heartbeat(data)
            if not hb:
                return ''
            return (f"state={hb.get('axis_state')} "
                    f"err=0x{hb.get('axis_error', 0):x}")
        if cmd == CmdId.GET_ENCODER_ESTIMATES:
            if n < 8:
                return 'request'
            d = decode_encoder_estimates(data)
            return (f"pos={d['pos_estimate']:.4f}rev "
                    f"vel={d['vel_estimate']:.4f}rev/s") if d else ''
        if cmd == CmdId.GET_ENCODER_COUNT:
            if n < 8:
                return 'request'
            d = decode_encoder_count(data)
            return f"shadow={d['shadow_count']} cpr={d['count_in_cpr']}" if d else ''
        if cmd == CmdId.GET_IQ:
            if n < 8:
                return 'request'
            d = decode_iq(data)
            return (f"iq_sp={d['iq_setpoint']:.3f} "
                    f"iq_meas={d['iq_measured']:.3f}A") if d else ''
        if cmd == CmdId.GET_BUS_VOLTAGE_CURRENT:
            if n < 8:
                return 'request'
            d = decode_bus_vi(data)
            return f"vbus={d['vbus']:.2f}V ibus={d['ibus']:.3f}A" if d else ''
        if cmd == CmdId.GET_MOTOR_ERROR and direction == 'rx':
            return f"motor_err=0x{decode_motor_error(data):x}"
        if cmd == CmdId.GET_ENCODER_ERROR and direction == 'rx':
            return f"enc_err=0x{decode_encoder_error(data):x}"
        if cmd == CmdId.GET_CONTROLLER_ERROR and direction == 'rx':
            return f"ctrl_err=0x{decode_controller_error(data):x}"
        if cmd == CmdId.EXTENDED_COMMAND:
            if direction == 'rx' and n >= 8:
                r = decode_extended_response(data)
                return (f"sub=0x{r.get('sub_cmd', 0):02x} "
                        f"item=0x{r.get('item', 0):02x} "
                        f"status={r.get('status')} val={r.get('value')}")
            if direction == 'tx' and n >= 4:
                sub, item, typ = data[0], data[1], data[2]
                val: Any = ''
                if n >= 8:
                    if typ == ExtType.FLOAT32:
                        val = struct.unpack_from('<f', data, 4)[0]
                    elif typ == ExtType.INT32:
                        val = struct.unpack_from('<i', data, 4)[0]
                    else:
                        val = struct.unpack_from('<I', data, 4)[0]
                return f"sub=0x{sub:02x} item=0x{item:02x} type={typ} val={val}"
        if cmd == CmdId.SET_INPUT_POS and direction == 'tx' and n >= 8:
            pos = struct.unpack_from('<f', data, 0)[0]
            vel_ff = struct.unpack_from('<h', data, 4)[0] * 0.001
            tq_ff = struct.unpack_from('<h', data, 6)[0] * 0.001
            return f"pos={pos:.4f}rev vel_ff={vel_ff:.3f} tq_ff={tq_ff:.3f}Nm"
        if cmd == CmdId.SET_INPUT_VEL and direction == 'tx' and n >= 8:
            vel, tq = struct.unpack_from('<ff', data, 0)
            return f"vel={vel:.4f}rev/s tq_ff={tq:.3f}Nm"
        if cmd == CmdId.SET_INPUT_TORQUE and direction == 'tx' and n >= 4:
            return f"torque={struct.unpack_from('<f', data, 0)[0]:.3f}Nm"
        if cmd == CmdId.SET_LIMITS and direction == 'tx' and n >= 8:
            vl, cl = struct.unpack_from('<ff', data, 0)
            return f"vel_limit={vl:.3f} cur_limit={cl:.3f}A"
        if cmd == CmdId.SET_AXIS_STATE and direction == 'tx' and n >= 4:
            s = struct.unpack_from('<I', data, 0)[0]
            try:
                return f"state={AxisState(s).name}"
            except ValueError:
                return f"state={s}"
        if cmd == CmdId.SET_CONTROLLER_MODE and direction == 'tx' and n >= 8:
            c, i = struct.unpack_from('<II', data, 0)
            return f"ctrl={c} input={i}"
        if cmd == CmdId.SET_MIT_CONTROL and direction == 'tx' and n >= 8:
            b = data
            p = _mit_uint_to_float((b[0] << 8) | b[1], MIT_P_MIN, MIT_P_MAX, 16)
            v = _mit_uint_to_float((b[2] << 4) | (b[3] >> 4), MIT_V_MIN, MIT_V_MAX, 12)
            kp = _mit_uint_to_float(((b[3] & 0x0F) << 8) | b[4], MIT_KP_MIN, MIT_KP_MAX, 12)
            kd = _mit_uint_to_float((b[5] << 4) | (b[6] >> 4), MIT_KD_MIN, MIT_KD_MAX, 12)
            t = _mit_uint_to_float(((b[6] & 0x0F) << 8) | b[7], MIT_T_MIN, MIT_T_MAX, 12)
            return f"p={p:.3f} v={v:.3f} kp={kp:.2f} kd={kd:.3f} t_ff={t:.3f}Nm"
        if cmd == CmdId.SET_POS_GAIN and direction == 'tx' and n >= 4:
            return f"pos_gain={struct.unpack_from('<f', data, 0)[0]:.4f}"
        if cmd == CmdId.SET_VEL_GAINS and direction == 'tx' and n >= 8:
            g, ig = struct.unpack_from('<ff', data, 0)
            return f"vel_gain={g:.4f} vel_int_gain={ig:.4f}"
        if cmd == CmdId.SET_LINEAR_COUNT and direction == 'tx' and n >= 4:
            return f"count={struct.unpack_from('<i', data, 0)[0]}"
        if cmd == CmdId.CLEAR_ERRORS and direction == 'tx':
            return 'clear_errors'
        if cmd == CmdId.ESTOP and direction == 'tx':
            return 'estop'
        if cmd == CmdId.REBOOT and direction == 'tx':
            return 'reboot'
        if cmd == CmdId.NMT and direction == 'tx':
            return 'nmt heartbeat feed'
    except Exception:
        return ''
    return ''


# --------------------------------------------------------------------------- #
# Encode Functions  (Python values -> CAN payload bytes)
# --------------------------------------------------------------------------- #

def encode_set_axis_state(state: AxisState | int) -> bytes:
    """Encode 0x007 Set_Axis_State: requested_state(uint32 LE)."""
    return struct.pack('<I', int(state))


def encode_set_controller_mode(ctrl: ControlMode | int,
                               inp: InputMode | int) -> bytes:
    """Encode 0x00B Set_Controller_Mode: control_mode(uint32 LE), input_mode(uint32 LE)."""
    return struct.pack('<II', int(ctrl), int(inp))


def encode_set_input_pos(pos: float, vel_ff: float = 0.0,
                         torque_ff: float = 0.0) -> bytes:
    """Encode 0x00C Set_Input_Pos (8 bytes).

    DBC BO_ 12 (lines 74-76):
      byte0-3: Input_Pos (float32 LE, rev)
      byte4-5: Vel_FF (int16 LE, scale=0.001 rev/s)
      byte6-7: Torque_FF (int16 LE, scale=0.001 Nm)
    """
    pos_bytes = struct.pack('<f', pos)
    vel_raw = max(-32768, min(32767, int(round(vel_ff / 0.001))))
    t_raw = max(-32768, min(32767, int(round(torque_ff / 0.001))))
    return pos_bytes + struct.pack('<hh', vel_raw, t_raw)


def encode_set_input_vel(vel: float, torque_ff: float = 0.0) -> bytes:
    """Encode 0x00D Set_Input_Vel (8 bytes).
    DBC BO_ 13 (lines 79-80): vel(float32 LE rev/s), torque_ff(float32 LE Nm)."""
    return struct.pack('<ff', vel, torque_ff)


def encode_set_input_torque(torque: float) -> bytes:
    """Encode 0x00E Set_Input_Torque (4 bytes).
    DBC BO_ 14 (line 83): torque(float32 LE Nm)."""
    return struct.pack('<f', torque)


def encode_set_limits(vel_limit: float, current_limit: float) -> bytes:
    """Encode 0x00F Set_Limits (8 bytes).
    DBC BO_ 15 (lines 86-87): vel_limit(float32 LE rev/s), current_limit(float32 LE A)."""
    return struct.pack('<ff', vel_limit, current_limit)


def encode_set_pos_gain(gain: float) -> bytes:
    """Encode 0x01A Set_Pos_Gain (4 bytes).
    DBC BO_ 26 (line 119): pos_gain(float32 LE (rev/s)/rev)."""
    return struct.pack('<f', gain)


def encode_set_vel_gains(gain: float, integrator_gain: float) -> bytes:
    """Encode 0x01B Set_Vel_Gains (8 bytes).
    DBC BO_ 27 (lines 122-123):
      vel_gain(float32 LE Nm/(rev/s))
      vel_integrator_gain(float32 LE (Nm/(rev/s))/s)"""
    return struct.pack('<ff', gain, integrator_gain)


def encode_set_traj_vel_limit(vel_limit: float) -> bytes:
    """Encode 0x011 Set_Traj_Vel_Limit (4 bytes)."""
    return struct.pack('<f', vel_limit)


def encode_set_traj_accel_limits(accel: float, decel: float) -> bytes:
    """Encode 0x012 Set_Traj_Accel_Limits (8 bytes)."""
    return struct.pack('<ff', accel, decel)


def encode_set_traj_inertia(inertia: float) -> bytes:
    """Encode 0x013 Set_Traj_Inertia (4 bytes)."""
    return struct.pack('<f', inertia)


def encode_set_linear_count(count: int) -> bytes:
    """Encode 0x019 Set_Linear_Count (4 bytes, int32 LE counts)."""
    return struct.pack('<i', count)


# --------------------------------------------------------------------------- #
# MIT Control Encoding  (0x01F, big-endian packed, AK/T-Motor compatible)
# --------------------------------------------------------------------------- #

# MIT field ranges
MIT_P_MIN = -12.5
MIT_P_MAX = 12.5
MIT_V_MIN = -45.0
MIT_V_MAX = 45.0
MIT_KP_MIN = 0.0
MIT_KP_MAX = 500.0
MIT_KD_MIN = 0.0
MIT_KD_MAX = 5.0
MIT_T_MIN = -18.0
MIT_T_MAX = 18.0


def _mit_float_to_uint(value: float, vmin: float, vmax: float,
                       bits: int) -> int:
    """Quantize a float to an unsigned integer in [0, 2^bits - 1]."""
    max_int = (1 << bits) - 1
    clamped = max(vmin, min(vmax, value))
    return int(round((clamped - vmin) / (vmax - vmin) * max_int)) & max_int


def encode_mit_control(p_des: float, v_des: float, kp: float,
                       kd: float, t_ff: float) -> bytes:
    """Encode 0x01F Set_MIT_Control (8 bytes, big-endian packed).

    Layout (CAN_PROTOCOL_PRODUCTION.md lines 107-116):
      byte0: p[15:8]
      byte1: p[7:0]
      byte2: v[11:4]
      byte3: v[3:0] | kp[11:8]
      byte4: kp[7:0]
      byte5: kd[11:4]
      byte6: kd[3:0] | t[11:8]
      byte7: t[7:0]
    """
    p_int = _mit_float_to_uint(p_des, MIT_P_MIN, MIT_P_MAX, 16)
    v_int = _mit_float_to_uint(v_des, MIT_V_MIN, MIT_V_MAX, 12)
    kp_int = _mit_float_to_uint(kp, MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_int = _mit_float_to_uint(kd, MIT_KD_MIN, MIT_KD_MAX, 12)
    t_int = _mit_float_to_uint(t_ff, MIT_T_MIN, MIT_T_MAX, 12)

    buf = bytearray(8)
    buf[0] = (p_int >> 8) & 0xFF
    buf[1] = p_int & 0xFF
    buf[2] = (v_int >> 4) & 0xFF
    buf[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    buf[4] = kp_int & 0xFF
    buf[5] = (kd_int >> 4) & 0xFF
    buf[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    buf[7] = t_int & 0xFF
    return bytes(buf)


def encode_mit_neutral() -> bytes:
    """Encode a neutral MIT frame.

    Firmware decodes both adjacent center raw codes for signed MIT fields as
    exact zero, so the normal rounded encoding is neutral despite the even
    12/16-bit quantization.
    """
    return encode_mit_control(0.0, 0.0, 0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Extended Command Encoding  (0x01E)
# --------------------------------------------------------------------------- #

def encode_extended_request(sub_cmd: int, item: int,
                            ext_type: int = 0, value: Any = 0) -> bytes:
    """Encode 0x01E Extended_Command request (8 bytes).

    Layout:
      byte0: sub_cmd (uint8)
      byte1: item (uint8)
      byte2: type (uint8, 0 for reads)
      byte3: reserved (0)
      byte4-7: value (interpreted by type for setters)
    """
    if ext_type == ExtType.FLOAT32:
        # JSON may deliver an integer literal (e.g. {"pos_threshold": 1});
        # coerce to float so it is not packed as uint32 and read back by the
        # firmware as a denormal float (~1.4e-45).
        val_bytes = struct.pack('<f', float(value))
    elif ext_type == ExtType.INT32:
        val_bytes = struct.pack('<i', int(value))
    else:
        val_bytes = struct.pack('<I', int(value))
    return bytes([sub_cmd & 0xFF, item & 0xFF,
                  ext_type & 0xFF, 0]) + val_bytes


# --------------------------------------------------------------------------- #
# Utility Functions
# --------------------------------------------------------------------------- #

def decode_flags(word: int, table: dict[int, str]) -> list[str]:
    """Return list of flag names that are set in word."""
    return [name for mask, name in sorted(table.items())
            if (word & mask) != 0]


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]."""
    return max(lo, min(hi, value))


# --------------------------------------------------------------------------- #
# JSON Serializers for /api/can/* static table endpoints
# --------------------------------------------------------------------------- #

def axis_states_as_json() -> list[dict]:
    return [{'value': s.value, 'name': s.name} for s in AxisState]


def control_modes_as_json() -> list[dict]:
    return [{'value': m.value, 'name': m.name} for m in ControlMode]


def input_modes_as_json() -> list[dict]:
    return [{'value': m.value, 'name': m.name} for m in InputMode]


def error_bits_as_json(table: dict[int, str], category: str) -> list[dict]:
    return [{'mask': m, 'name': n, 'category': category}
            for m, n in sorted(table.items())]
