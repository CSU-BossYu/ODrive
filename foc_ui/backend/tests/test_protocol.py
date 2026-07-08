"""Unit tests for odrive_can.protocol.

Tests all encode/decode functions with known vectors.
Run: pytest tests/test_protocol.py -v
"""

import struct
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.protocol import (
    CmdId, AxisState, ControlMode, InputMode,
    ExtSubCmd, ExtStatus, ExtType,
    AXIS_ERROR_BITS, MOTOR_ERROR_BITS, ENCODER_ERROR_BITS, CONTROLLER_ERROR_BITS,
    make_frame_id, parse_frame_id,
    decode_heartbeat, decode_encoder_estimates, decode_iq, decode_bus_vi,
    decode_motor_error, decode_encoder_error, decode_controller_error,
    decode_extended_response, decode_encoder_count,
    encode_set_axis_state, encode_set_controller_mode,
    encode_set_input_pos, encode_set_input_vel, encode_set_input_torque,
    encode_set_limits, encode_set_pos_gain, encode_set_vel_gains,
    encode_mit_control, encode_mit_neutral,
    encode_extended_request,
    decode_flags, clamp,
    axis_states_as_json, control_modes_as_json, input_modes_as_json,
    error_bits_as_json,
    MIT_P_MIN, MIT_P_MAX, MIT_V_MIN, MIT_V_MAX,
    MIT_KP_MIN, MIT_KP_MAX, MIT_KD_MIN, MIT_KD_MAX,
    MIT_T_MIN, MIT_T_MAX,
)


# --------------------------------------------------------------------------- #
# Frame ID encoding/decoding
# --------------------------------------------------------------------------- #

class TestFrameId:
    def test_make_frame_id_node0_heartbeat(self):
        assert make_frame_id(0, CmdId.HEARTBEAT) == 0x001

    def test_make_frame_id_node0_encoder(self):
        assert make_frame_id(0, CmdId.GET_ENCODER_ESTIMATES) == 0x009

    def test_make_frame_id_node1_heartbeat(self):
        assert make_frame_id(1, CmdId.HEARTBEAT) == 0x021

    def test_make_frame_id_node3_clear_errors(self):
        # node_id=3, cmd=0x018 -> (3 << 5) | 0x18 = 0x60 | 0x18 = 0x78
        assert make_frame_id(3, CmdId.CLEAR_ERRORS) == 0x78

    def test_parse_frame_id_roundtrip(self):
        for nid in range(0, 4):
            for cmd in [CmdId.HEARTBEAT, CmdId.GET_ENCODER_ESTIMATES,
                        CmdId.SET_INPUT_TORQUE, CmdId.SET_MIT_CONTROL]:
                arb = make_frame_id(nid, cmd)
                got_nid, got_cmd = parse_frame_id(arb)
                assert got_nid == nid
                assert got_cmd == int(cmd)


# --------------------------------------------------------------------------- #
# Heartbeat decode
# --------------------------------------------------------------------------- #

class TestDecodeHeartbeat:
    def test_basic(self):
        # axis_error=0x41, axis_state=8 (CLOSED_LOOP), motor=1, enc=0, ctrl=1, traj=1
        data = struct.pack('<I', 0x41) + bytes([8, 1, 0, 0x81])
        d = decode_heartbeat(data)
        assert d['axis_error'] == 0x41
        assert d['axis_state'] == 8
        assert d['motor_err_flag'] is True
        assert d['encoder_err_flag'] is False
        assert d['controller_err_flag'] is True
        assert d['traj_done'] is True

    def test_no_error(self):
        data = struct.pack('<I', 0) + bytes([1, 0, 0, 0])
        d = decode_heartbeat(data)
        assert d['axis_error'] == 0
        assert d['axis_state'] == 1  # IDLE
        assert d['motor_err_flag'] is False
        assert d['traj_done'] is False

    def test_short_frame(self):
        assert decode_heartbeat(b'\x00\x00') == {}


# --------------------------------------------------------------------------- #
# Encoder estimates decode
# --------------------------------------------------------------------------- #

class TestDecodeEncoderEstimates:
    def test_basic(self):
        pos, vel = 1.5, 10.25
        data = struct.pack('<ff', pos, vel)
        d = decode_encoder_estimates(data)
        assert d['pos_estimate'] == pytest.approx(pos)
        assert d['vel_estimate'] == pytest.approx(vel)

    def test_negative(self):
        data = struct.pack('<ff', -3.14, -50.0)
        d = decode_encoder_estimates(data)
        assert d['pos_estimate'] == pytest.approx(-3.14)
        assert d['vel_estimate'] == pytest.approx(-50.0)


# --------------------------------------------------------------------------- #
# Iq decode
# --------------------------------------------------------------------------- #

class TestDecodeIq:
    def test_basic(self):
        data = struct.pack('<ff', 2.5, 2.3)
        d = decode_iq(data)
        assert d['iq_setpoint'] == pytest.approx(2.5)
        assert d['iq_measured'] == pytest.approx(2.3)


# --------------------------------------------------------------------------- #
# Bus voltage/current decode
# --------------------------------------------------------------------------- #

class TestDecodeBusVI:
    def test_basic(self):
        data = struct.pack('<ff', 48.0, 1.5)
        d = decode_bus_vi(data)
        assert d['vbus'] == pytest.approx(48.0)
        assert d['ibus'] == pytest.approx(1.5)


# --------------------------------------------------------------------------- #
# Error decode
# --------------------------------------------------------------------------- #

class TestDecodeErrors:
    def test_motor_error(self):
        data = struct.pack('<I', 0x00200010)
        assert decode_motor_error(data) == 0x00200010

    def test_encoder_error(self):
        data = struct.pack('<I', 0x04)
        assert decode_encoder_error(data) == 0x04

    def test_controller_error(self):
        data = struct.pack('<I', 0x80)
        assert decode_controller_error(data) == 0x80


# --------------------------------------------------------------------------- #
# Set axis state encode
# --------------------------------------------------------------------------- #

class TestEncodeSetAxisState:
    def test_idle(self):
        assert encode_set_axis_state(AxisState.IDLE) == struct.pack('<I', 1)

    def test_closed_loop(self):
        assert encode_set_axis_state(AxisState.CLOSED_LOOP_CONTROL) == struct.pack('<I', 8)


# --------------------------------------------------------------------------- #
# Set controller mode encode
# --------------------------------------------------------------------------- #

class TestEncodeSetControllerMode:
    def test_velocity_ramp(self):
        data = encode_set_controller_mode(ControlMode.VELOCITY_CONTROL, InputMode.VEL_RAMP)
        assert data == struct.pack('<II', 2, 2)

    def test_torque_passthrough(self):
        data = encode_set_controller_mode(ControlMode.TORQUE_CONTROL, InputMode.PASSTHROUGH)
        assert data == struct.pack('<II', 1, 1)

    def test_torque_mit(self):
        data = encode_set_controller_mode(ControlMode.TORQUE_CONTROL, InputMode.MIT)
        assert data == struct.pack('<II', 1, 9)


# --------------------------------------------------------------------------- #
# Set input pos encode
# --------------------------------------------------------------------------- #

class TestEncodeSetInputPos:
    def test_basic(self):
        data = encode_set_input_pos(1.0, 0.5, 0.1)
        pos = struct.unpack_from('<f', data, 0)[0]
        vel_ff = struct.unpack_from('<h', data, 4)[0]
        t_ff = struct.unpack_from('<h', data, 6)[0]
        assert pos == pytest.approx(1.0)
        assert vel_ff == 500       # 0.5 / 0.001 = 500
        assert t_ff == 100         # 0.1 / 0.001 = 100

    def test_zero(self):
        data = encode_set_input_pos(0.0)
        assert len(data) == 8
        assert struct.unpack_from('<f', data, 0)[0] == 0.0

    def test_negative_vel_ff(self):
        data = encode_set_input_pos(0.0, -1.0)
        vel_ff = struct.unpack_from('<h', data, 4)[0]
        assert vel_ff == -1000     # -1.0 / 0.001 = -1000


# --------------------------------------------------------------------------- #
# Set input vel encode
# --------------------------------------------------------------------------- #

class TestEncodeSetInputVel:
    def test_basic(self):
        data = encode_set_input_vel(10.0, 0.5)
        vel, t_ff = struct.unpack('<ff', data)
        assert vel == pytest.approx(10.0)
        assert t_ff == pytest.approx(0.5)


# --------------------------------------------------------------------------- #
# Set input torque encode
# --------------------------------------------------------------------------- #

class TestEncodeSetInputTorque:
    def test_basic(self):
        data = encode_set_input_torque(0.5)
        val = struct.unpack('<f', data)[0]
        assert val == pytest.approx(0.5)


# --------------------------------------------------------------------------- #
# Set limits encode
# --------------------------------------------------------------------------- #

class TestEncodeSetLimits:
    def test_basic(self):
        data = encode_set_limits(50.0, 20.0)
        vl, cl = struct.unpack('<ff', data)
        assert vl == pytest.approx(50.0)
        assert cl == pytest.approx(20.0)


# --------------------------------------------------------------------------- #
# Set gains encode
# --------------------------------------------------------------------------- #

class TestEncodeGains:
    def test_pos_gain(self):
        data = encode_set_pos_gain(25.0)
        assert struct.unpack('<f', data)[0] == pytest.approx(25.0)

    def test_vel_gains(self):
        data = encode_set_vel_gains(0.5, 10.0)
        g, ig = struct.unpack('<ff', data)
        assert g == pytest.approx(0.5)
        assert ig == pytest.approx(10.0)


# --------------------------------------------------------------------------- #
# MIT control encode
# --------------------------------------------------------------------------- #

class TestEncodeMitControl:
    def test_all_zeros_neutral(self):
        data = encode_mit_control(0.0, 0.0, 0.0, 0.0, 0.0)
        assert len(data) == 8
        # Neutral = center of range for signed fields, 0 for unsigned
        # p: (0 + 12.5) / 25.0 * 65535 = 32767.5 -> 32768 = 0x8000
        # v: (0 + 45) / 90 * 4095 = 2047.5 -> 2048 = 0x800
        # kp: 0 -> 0x000
        # kd: 0 -> 0x000
        # t: (0 + 18) / 36 * 4095 = 2047.5 -> 2048 = 0x800
        assert data[0] == 0x80  # p[15:8]
        assert data[1] == 0x00  # p[7:0]

    def test_neutral_helper(self):
        data = encode_mit_neutral()
        assert data == encode_mit_control(0.0, 0.0, 0.0, 0.0, 0.0)

    def test_boundary_p_min(self):
        data = encode_mit_control(MIT_P_MIN, 0, 0, 0, 0)
        p_int = (data[0] << 8) | data[1]
        assert p_int == 0  # min value maps to 0

    def test_boundary_p_max(self):
        data = encode_mit_control(MIT_P_MAX, 0, 0, 0, 0)
        p_int = (data[0] << 8) | data[1]
        assert p_int == 0xFFFF  # max value maps to 65535

    def test_clamping_p_over(self):
        # p=20.0 exceeds MIT_P_MAX=12.5, should clamp
        data = encode_mit_control(20.0, 0, 0, 0, 0)
        p_int = (data[0] << 8) | data[1]
        assert p_int == 0xFFFF

    def test_clamping_t_over(self):
        # t_ff=25.0 exceeds MIT_T_MAX=18.0, should clamp
        data = encode_mit_control(0, 0, 0, 0, 25.0)
        t_int = ((data[6] & 0xF) << 8) | data[7]
        assert t_int == 0xFFF

    def test_known_vector(self):
        # p_des=0, v_des=0, kp=100, kd=1.0, t_ff=0.5
        data = encode_mit_control(0.0, 0.0, 100.0, 1.0, 0.5)
        # p: (0+12.5)/25*65535 = 32768 = 0x8000
        assert data[0] == 0x80
        assert data[1] == 0x00
        # kp: 100/500*4095 = 819 = 0x333
        kp_int = ((data[3] & 0xF) << 8) | data[4]
        assert kp_int == 819
        # kd: 1.0/5.0*4095 = 819 = 0x333
        kd_int = (data[5] << 4) | ((data[6] >> 4) & 0xF)
        assert kd_int == 819

    def test_frame_length(self):
        data = encode_mit_control(1.0, 2.0, 3.0, 4.0, 5.0)
        assert len(data) == 8


# --------------------------------------------------------------------------- #
# Extended command encode/decode
# --------------------------------------------------------------------------- #

class TestExtendedCommand:
    def test_encode_read_request(self):
        data = encode_extended_request(ExtSubCmd.GET_AXIS_STATUS_EX, 0x00)
        assert data[0] == 0x01  # sub_cmd
        assert data[1] == 0x00  # item
        assert data[2] == 0x00  # type (read)
        assert data[3] == 0x00  # reserved
        assert len(data) == 8

    def test_encode_float_write(self):
        data = encode_extended_request(ExtSubCmd.SET_BASIC_CONFIG,
                                       0x03, ExtType.FLOAT32, 1.5)
        assert data[0] == 0x07
        assert data[1] == 0x03
        assert data[2] == ExtType.FLOAT32
        val = struct.unpack_from('<f', data, 4)[0]
        assert val == pytest.approx(1.5)

    def test_encode_uint_write(self):
        data = encode_extended_request(ExtSubCmd.SET_BASIC_CONFIG,
                                       0x01, ExtType.UINT32, 1)
        val = struct.unpack_from('<I', data, 4)[0]
        assert val == 1

    def test_decode_response(self):
        data = bytes([0x01, 0x00, 0x00, ExtType.UINT32]) + struct.pack('<I', 8)
        resp = decode_extended_response(data)
        assert resp['sub_cmd'] == 0x01
        assert resp['item'] == 0x00
        assert resp['status'] == ExtStatus.OK
        assert resp['type'] == ExtType.UINT32
        assert resp['value'] == 8

    def test_decode_float_response(self):
        data = bytes([0x08, 0x03, 0x00, ExtType.FLOAT32]) + struct.pack('<f', 2.5)
        resp = decode_extended_response(data)
        assert resp['value'] == pytest.approx(2.5)


# --------------------------------------------------------------------------- #
# Error flag decoders
# --------------------------------------------------------------------------- #

class TestDecodeFlags:
    def test_single_flag(self):
        assert decode_flags(0x01, AXIS_ERROR_BITS) == ['INVALID_STATE']

    def test_multiple_flags(self):
        result = decode_flags(0x41, AXIS_ERROR_BITS)
        assert 'INVALID_STATE' in result
        assert 'MOTOR_FAILED' in result

    def test_no_flags(self):
        assert decode_flags(0, AXIS_ERROR_BITS) == []

    def test_motor_error(self):
        result = decode_flags(0x00200010, MOTOR_ERROR_BITS)
        assert 'CONTROL_DEADLINE_MISSED' in result
        assert 'CONTROLLER_FAILED' in result


# --------------------------------------------------------------------------- #
# Utility functions
# --------------------------------------------------------------------------- #

class TestClamp:
    def test_within_range(self):
        assert clamp(5.0, 0.0, 10.0) == 5.0

    def test_below_min(self):
        assert clamp(-1.0, 0.0, 10.0) == 0.0

    def test_above_max(self):
        assert clamp(15.0, 0.0, 10.0) == 10.0


# --------------------------------------------------------------------------- #
# JSON serializers
# --------------------------------------------------------------------------- #

class TestJsonSerializers:
    def test_axis_states_count(self):
        j = axis_states_as_json()
        assert len(j) == len(AxisState)
        names = [e['name'] for e in j]
        assert 'IDLE' in names
        assert 'CLOSED_LOOP_CONTROL' in names

    def test_control_modes_count(self):
        j = control_modes_as_json()
        assert len(j) == len(ControlMode)

    def test_input_modes_count(self):
        j = input_modes_as_json()
        assert len(j) == len(InputMode)
        names = [e['name'] for e in j]
        assert 'MIT' in names

    def test_error_bits_axis(self):
        j = error_bits_as_json(AXIS_ERROR_BITS, 'axis')
        assert all(e['category'] == 'axis' for e in j)
        assert len(j) == len(AXIS_ERROR_BITS)

    def test_error_bits_motor(self):
        j = error_bits_as_json(MOTOR_ERROR_BITS, 'motor')
        assert len(j) == len(MOTOR_ERROR_BITS)
