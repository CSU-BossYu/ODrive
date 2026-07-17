"""Unit tests for odrive_can.service with virtual CAN bus.

Run: pytest tests/test_service.py -v
Requires: python-can installed
"""

import asyncio
import math
import struct
import time
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import can as _can
    _HAS_CAN = True
except ImportError:
    _HAS_CAN = False

from odrive_can.protocol import (
    CmdId, AxisState, ControlMode, InputMode,
    ExtSubCmd, ExtType, ExtStatus,
    make_frame_id, encode_extended_request,
    MIT_P_MIN, MIT_P_MAX, MIT_V_MIN, MIT_V_MAX,
)
from odrive_can.transport import CanTransport
from odrive_can.service import (
    ODriveService,
    VERNIER_AUTO_SWEEP_DEFAULT, build_vernier_auto_offsets,
)


def _decode_mit_raw(data):
    return {
        'p': (data[0] << 8) | data[1],
        'v': (data[2] << 4) | (data[3] >> 4),
        'kp': ((data[3] & 0x0F) << 8) | data[4],
        'kd': (data[5] << 4) | (data[6] >> 4),
        't': ((data[6] & 0x0F) << 8) | data[7],
    }


def _mit_float_to_uint(value, low, high, bits):
    value = max(low, min(high, value))
    return int(round((value - low) / (high - low) * ((1 << bits) - 1)))


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


def _make_service(event_loop, channel='test_svc'):
    """Create a transport + service pair on a virtual CAN channel."""
    transport = CanTransport(event_loop, node_id=0)
    transport.open(interface='virtual', channel=channel)
    service = ODriveService(transport, node_id=0)
    # Wire the transport's frame callback to the service's on_frame
    transport._on_frame = service.on_frame
    return transport, service


def _make_rx_bus(channel):
    """Create a receiver bus to inspect sent frames."""
    return _can.Bus(interface='virtual', channel=channel)


def _make_tx_bus(channel):
    """Create a transmitter bus to inject frames."""
    return _can.Bus(interface='virtual', channel=channel)


# --------------------------------------------------------------------------- #
# Frame Dispatch
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestFrameDispatch:
    def test_heartbeat_updates_cache(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_hb')
        try:
            hb_data = struct.pack('<I', 0x41) + bytes([8, 1, 0, 0x81])
            event_loop.run_until_complete(
                svc.on_frame(CmdId.HEARTBEAT, hb_data, time.monotonic()))
            assert svc.cache.heartbeat.axis_error == 0x41
            assert svc.cache.heartbeat.axis_state == 8
            assert svc.cache.heartbeat.motor_err_flag is True
            assert svc.cache.heartbeat.trajectory_done is True
        finally:
            transport.close()

    def test_encoder_updates_cache(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_enc')
        try:
            data = struct.pack('<ff', 1.5, 10.25)
            event_loop.run_until_complete(
                svc.on_frame(CmdId.GET_ENCODER_ESTIMATES, data, time.monotonic()))
            assert svc.cache.encoder.pos_estimate == pytest.approx(1.5)
            assert svc.cache.encoder.vel_estimate == pytest.approx(10.25)
        finally:
            transport.close()

    def test_iq_updates_cache(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_iq')
        try:
            data = struct.pack('<ff', 2.5, 2.3)
            event_loop.run_until_complete(
                svc.on_frame(CmdId.GET_IQ, data, time.monotonic()))
            assert svc.cache.iq.iq_setpoint == pytest.approx(2.5)
            assert svc.cache.iq.iq_measured == pytest.approx(2.3)
        finally:
            transport.close()

    def test_bus_vi_updates_cache(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_bus')
        try:
            data = struct.pack('<ff', 48.0, 1.5)
            event_loop.run_until_complete(
                svc.on_frame(CmdId.GET_BUS_VOLTAGE_CURRENT, data, time.monotonic()))
            assert svc.cache.bus.vbus == pytest.approx(48.0)
            assert svc.cache.bus.ibus == pytest.approx(1.5)
        finally:
            transport.close()

    def test_error_detail_updates(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_err')
        try:
            motor_err = struct.pack('<I', 0x00200010)
            event_loop.run_until_complete(
                svc.on_frame(CmdId.GET_MOTOR_ERROR, motor_err, time.monotonic()))
            assert svc.cache.errors.motor_error == 0x00200010

            enc_err = struct.pack('<I', 0x04)
            event_loop.run_until_complete(
                svc.on_frame(CmdId.GET_ENCODER_ERROR, enc_err, time.monotonic()))
            assert svc.cache.errors.encoder_error == 0x04

            ctrl_err = struct.pack('<I', 0x80)
            event_loop.run_until_complete(
                svc.on_frame(CmdId.GET_CONTROLLER_ERROR, ctrl_err, time.monotonic()))
            assert svc.cache.errors.controller_error == 0x80
        finally:
            transport.close()


# --------------------------------------------------------------------------- #
# Command Methods
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestCommands:
    def test_set_axis_state(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_cmd_state')
        rx = _make_rx_bus('test_cmd_state')
        try:
            event_loop.run_until_complete(
                svc.set_axis_state(AxisState.CLOSED_LOOP_CONTROL))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            assert msg.arbitration_id == make_frame_id(0, CmdId.SET_AXIS_STATE)
            state = struct.unpack_from('<I', msg.data, 0)[0]
            assert state == 8
        finally:
            rx.shutdown()
            transport.close()

    def test_set_controller_mode(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_cmd_mode')
        rx = _make_rx_bus('test_cmd_mode')
        try:
            event_loop.run_until_complete(
                svc.set_controller_mode(ControlMode.VELOCITY_CONTROL, InputMode.VEL_RAMP))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            ctrl, inp = struct.unpack('<II', msg.data[:8])
            assert ctrl == 2
            assert inp == 2
        finally:
            rx.shutdown()
            transport.close()

    def test_set_input_torque_passthrough(self, event_loop):
        """Torque is passed through unscaled (firmware current_lim is the
        safety net); NaN/Inf are rejected."""
        transport, svc = _make_service(event_loop, 'test_cmd_torque')
        rx = _make_rx_bus('test_cmd_torque')
        try:
            # 1.0 Nm passes through unchanged (no host-side clamp).
            event_loop.run_until_complete(svc.set_input_torque(1.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            torque = struct.unpack('<f', msg.data[:4])[0]
            assert torque == pytest.approx(1.0)

            # -1.0 Nm passes through unchanged.
            event_loop.run_until_complete(svc.set_input_torque(-1.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            torque = struct.unpack('<f', msg.data[:4])[0]
            assert torque == pytest.approx(-1.0)

            # NaN is rejected: no frame sent.
            event_loop.run_until_complete(svc.set_input_torque(float('nan')))
            assert rx.recv(timeout=0.2) is None
        finally:
            rx.shutdown()
            transport.close()

    def test_set_mit_torque_passthrough(self, event_loop):
        """MIT t_ff passes through (saturates at ±18 Nm in the encoding, not
        clamped to a bench limit); NaN t_ff is rejected."""
        transport, svc = _make_service(event_loop, 'test_cmd_mit')
        rx = _make_rx_bus('test_cmd_mit')
        try:
            # t_ff=1.0 Nm must pass through: the raw 12-bit field encodes
            # ~1.0 Nm, NOT the one-step (~0.013 Nm) value the old bench clamp
            # would have produced.
            event_loop.run_until_complete(
                svc.set_mit_control(0, 0, 0, 0, 1.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            assert len(msg.data) == 8
            raw = _decode_mit_raw(msg.data)
            expected_t = _mit_float_to_uint(1.0, -18.0, 18.0, 12)
            assert raw['t'] == expected_t
            assert svc._mit_active is True

            # NaN t_ff is rejected: no frame sent.
            event_loop.run_until_complete(
                svc.set_mit_control(0, 0, 0, 0, float('nan')))
            assert rx.recv(timeout=0.2) is None
        finally:
            rx.shutdown()
            transport.close()

    def test_set_mit_uses_rad_units(self, event_loop):
        """MIT p/v are protocol rad and rad/s, not ODrive turns/turn-s."""
        transport, svc = _make_service(event_loop, 'test_cmd_mit_units')
        rx = _make_rx_bus('test_cmd_mit_units')
        try:
            event_loop.run_until_complete(
                svc.set_mit_control(math.pi, 2.0 * math.pi, 1.0, 0.5, 0.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            raw = _decode_mit_raw(bytes(msg.data))
            assert raw['p'] == _mit_float_to_uint(math.pi, MIT_P_MIN, MIT_P_MAX, 16)
            assert raw['v'] == _mit_float_to_uint(2.0 * math.pi, MIT_V_MIN, MIT_V_MAX, 12)
        finally:
            rx.shutdown()
            transport.close()

    def test_set_mit_position_velocity_clamped_to_protocol_range(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_cmd_mit_clamp_units')
        rx = _make_rx_bus('test_cmd_mit_clamp_units')
        try:
            event_loop.run_until_complete(
                svc.set_mit_control(100.0, 100.0, 0.0, 0.0, 0.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            raw = _decode_mit_raw(bytes(msg.data))
            assert raw['p'] == 0xFFFF
            assert raw['v'] == 0xFFF
        finally:
            rx.shutdown()
            transport.close()

    def test_clear_errors(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_cmd_clear')
        rx = _make_rx_bus('test_cmd_clear')
        try:
            event_loop.run_until_complete(svc.clear_errors())
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            assert msg.arbitration_id == make_frame_id(0, CmdId.CLEAR_ERRORS)
        finally:
            rx.shutdown()
            transport.close()

    def test_estop(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_cmd_estop')
        rx = _make_rx_bus('test_cmd_estop')
        try:
            event_loop.run_until_complete(svc.estop())
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            assert msg.arbitration_id == make_frame_id(0, CmdId.ESTOP)
        finally:
            rx.shutdown()
            transport.close()


# --------------------------------------------------------------------------- #
# Extended Command Correlation
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestExtendedCommand:
    def test_correlated_response(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_ext_corr')
        tx = _make_tx_bus('test_ext_corr')
        try:
            async def run_test():
                # Start ext command in background
                cmd_task = asyncio.ensure_future(
                    svc.ext_command(ExtSubCmd.GET_AXIS_STATUS_EX, 0x00,
                                   timeout=2.0))

                # Wait a bit, then inject a response
                await asyncio.sleep(0.1)
                resp_data = (bytes([0x01, 0x00, ExtStatus.OK, ExtType.UINT32])
                             + struct.pack('<I', 8))
                msg = _can.Message(
                    arbitration_id=make_frame_id(0, CmdId.EXTENDED_COMMAND),
                    data=resp_data,
                    is_extended_id=False,
                )
                tx.send(msg)

                # Process the frame through the transport pump
                pump_task = asyncio.ensure_future(transport.pump())
                result = await cmd_task
                pump_task.cancel()
                try:
                    await pump_task
                except asyncio.CancelledError:
                    pass
                return result

            result = event_loop.run_until_complete(run_test())
            assert result['status'] == ExtStatus.OK
            assert result['value'] == 8
        finally:
            tx.shutdown()
            transport.close()

    def test_timeout(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_ext_timeout')
        try:
            async def run_test():
                return await svc.ext_command(
                    ExtSubCmd.GET_AXIS_STATUS_EX, 0x00, timeout=0.1)

            result = event_loop.run_until_complete(run_test())
            assert result['status'] == -1
            assert result['error'] == 'timeout'
        finally:
            transport.close()

# --------------------------------------------------------------------------- #
# Safety Monitor
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestSafetyMonitor:
    def test_ws_disconnect_triggers_safety_stop(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_ws_safety')
        rx = _make_rx_bus('test_ws_safety')
        try:
            async def run_test():
                svc.notify_ws_connected()
                svc.notify_ws_activity()

                # Start safety monitor
                await svc.start_safety_monitor()

                # Wait for watchdog (WS_WATCHDOG_S=10.0s, check every 0.5s)
                # We need to expire the watchdog: set last_ws_msg_ts far back
                svc._last_ws_msg_ts = time.monotonic() - 15.0

                await asyncio.sleep(1.0)
                await svc.stop_safety_monitor()

            event_loop.run_until_complete(run_test())

            # Should have received set_input_torque(0) and set_axis_state(IDLE)
            msgs = []
            while True:
                msg = rx.recv(timeout=0.1)
                if msg is None:
                    break
                msgs.append(msg)

            # At minimum, we expect a torque=0 command and an IDLE state command
            arb_ids = [m.arbitration_id for m in msgs]
            assert make_frame_id(0, CmdId.SET_INPUT_TORQUE) in arb_ids
            assert make_frame_id(0, CmdId.SET_AXIS_STATE) in arb_ids
        finally:
            rx.shutdown()
            transport.close()

    def test_mit_watchdog_sends_neutral(self, event_loop):
        transport, svc = _make_service(event_loop, 'test_mit_safety')
        rx = _make_rx_bus('test_mit_safety')
        try:
            async def run_test():
                # Send MIT to activate
                await svc.set_mit_control(0, 0, 0, 0, 0)
                assert svc._mit_active is True

                # Start safety monitor
                await svc.start_safety_monitor()

                # Force MIT last_ts to be old
                svc._mit_last_ts = time.monotonic() - 1.0

                await asyncio.sleep(1.0)
                await svc.stop_safety_monitor()

            event_loop.run_until_complete(run_test())

            # Should have received at least one MIT neutral frame
            msgs = []
            while True:
                msg = rx.recv(timeout=0.1)
                if msg is None:
                    break
                msgs.append(msg)

            mit_arb = make_frame_id(0, CmdId.SET_MIT_CONTROL)
            mit_msgs = [m for m in msgs if m.arbitration_id == mit_arb]
            assert len(mit_msgs) >= 1  # At least the initial + neutral
        finally:
            rx.shutdown()
            transport.close()


# --------------------------------------------------------------------------- #
# Friction-Compensation Calibration
# --------------------------------------------------------------------------- #

def test_compute_friction_params():
    """Pure helper: breakaway -> static/coulomb/max/slew (no CAN)."""
    from odrive_can.service import compute_friction_params
    p = compute_friction_params(1.0, 0.5)
    assert p['friction_static_pos'] == pytest.approx(0.8)        # 0.8 * 1.0
    assert p['friction_static_neg'] == pytest.approx(0.4)        # 0.8 * 0.5
    assert p['friction_coulomb_pos'] == pytest.approx(0.48)      # 0.6 * 0.8
    assert p['friction_coulomb_neg'] == pytest.approx(0.24)      # 0.6 * 0.4
    assert p['friction_max_torque'] == pytest.approx(0.96)       # 1.2 * 0.8
    assert p['friction_torque_slew_rate'] == pytest.approx(24.0)  # 30 * 0.8


def test_build_vernier_auto_offsets_are_symmetric_and_bounded():
    offsets = build_vernier_auto_offsets(8, VERNIER_AUTO_SWEEP_DEFAULT, 2)
    assert len(offsets) == 8
    assert max(offsets) <= VERNIER_AUTO_SWEEP_DEFAULT / 2.0 + 1e-9
    assert min(offsets) >= -VERNIER_AUTO_SWEEP_DEFAULT / 2.0 - 1e-9
    assert offsets[0] == pytest.approx(-VERNIER_AUTO_SWEEP_DEFAULT / 2.0)
    assert offsets[-1] == pytest.approx(-VERNIER_AUTO_SWEEP_DEFAULT / 2.0)
    assert max(abs(offsets[i + 1] - offsets[i]) for i in range(len(offsets) - 1)) < VERNIER_AUTO_SWEEP_DEFAULT


def test_build_vernier_auto_offsets_clamps_full_turn_request():
    offsets = build_vernier_auto_offsets(5, 1.0, 1)
    assert max(offsets) <= 0.125 + 1e-9
    assert min(offsets) >= -0.125 - 1e-9


def test_build_vernier_auto_offsets_supports_multiple_cycles():
    offsets = build_vernier_auto_offsets(9, 0.04, 3)
    assert offsets[0] == pytest.approx(-0.02)
    assert offsets[-1] == pytest.approx(0.02)
    assert max(offsets) == pytest.approx(0.02)
    assert min(offsets) == pytest.approx(-0.02)


def test_control_config_items_include_velocity_adrc_and_friction():
    """CONTROL_CONFIG_ITEMS exposes velocity-limit, ADRC trim and friction params."""
    from odrive_can.protocol import CONTROL_CONFIG_ITEMS
    names = {name for _item, name, _is_float in CONTROL_CONFIG_ITEMS}
    for n in ('vel_limit',
              'vel_limit_tolerance',
              'enable_vel_limit',
              'enable_torque_mode_vel_limit',
              'adrc_trim_slew_rate',
              'enable_adrc',
              'adrc_trim_torque_limit',
              'enable_friction_compensation',
              'enable_mit_friction_compensation',
              'friction_pos_deadband', 'friction_vel_deadband',
              'friction_stribeck_vel',
              'friction_static_pos', 'friction_static_neg',
              'friction_coulomb_pos', 'friction_coulomb_neg',
              'friction_viscous_pos', 'friction_viscous_neg',
              'friction_max_torque', 'friction_torque_slew_rate'):
        assert n in names, f'{n} missing from CONTROL_CONFIG_ITEMS'


def test_set_control_config_allows_inf_velocity_limit_sentinels(event_loop):
    """Low-level config API preserves +inf sentinels for 0x60/0x61."""
    class DummyTransport:
        def send(self, cmd_id, data):
            raise AssertionError('send should be mocked by fake_ext')

    svc = ODriveService(DummyTransport(), node_id=0)
    calls = []

    async def fake_ext(sub_cmd, item, ext_type=0, value=0, timeout=1.0):
        calls.append((sub_cmd, item, ext_type, value))
        return {'status': ExtStatus.OK, 'sub_cmd': int(sub_cmd), 'item': item}

    svc.ext_command = fake_ext
    result = event_loop.run_until_complete(
        svc.set_control_config(0x60, float('inf'), True))
    assert result['status'] == ExtStatus.OK
    assert calls[-1] == (ExtSubCmd.SET_CONTROL_CONFIG, 0x60,
                         ExtType.FLOAT32, float('inf'))

    result = event_loop.run_until_complete(
        svc.set_control_config(0x61, float('inf'), True))
    assert result['status'] == ExtStatus.OK
    assert calls[-1] == (ExtSubCmd.SET_CONTROL_CONFIG, 0x61,
                         ExtType.FLOAT32, float('inf'))


def test_set_limits_returns_two_ext_responses_and_updates_cache_on_ack(event_loop):
    """set_limits compatibility path emits standard ext responses only."""
    class DummyTransport:
        def send(self, cmd_id, data):
            raise AssertionError('send should be mocked by fake_ext')

    svc = ODriveService(DummyTransport(), node_id=0)
    calls = []

    async def fake_ext(sub_cmd, item, ext_type=0, value=0, timeout=1.0):
        calls.append((sub_cmd, item, ext_type, value))
        status = ExtStatus.BUSY_ARMED if item == 0x60 else ExtStatus.OK
        return {'status': status, 'sub_cmd': int(sub_cmd), 'item': item}

    svc.ext_command = fake_ext
    responses = event_loop.run_until_complete(svc.set_limits(3.0, 4.0))
    assert len(responses) == 2
    assert responses[0]['sub_cmd'] == int(ExtSubCmd.SET_CONTROL_CONFIG)
    assert responses[0]['item'] == 0x60
    assert responses[1]['sub_cmd'] == int(ExtSubCmd.SET_BASIC_CONFIG)
    assert responses[1]['item'] == 0x14
    assert svc._last_vel_limit == 0.0

    async def fake_ext_ok(sub_cmd, item, ext_type=0, value=0, timeout=1.0):
        return {'status': ExtStatus.OK, 'sub_cmd': int(sub_cmd), 'item': item}

    svc.ext_command = fake_ext_ok
    responses = event_loop.run_until_complete(svc.set_limits(3.0, 4.0))
    assert responses[0]['status'] == ExtStatus.OK
    assert svc._last_vel_limit == pytest.approx(3.0)


@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
def test_read_friction_static_pos_correlated(event_loop):
    """ext_command GET_CONTROL_CONFIG 0x74 correlates with the 0x0B/0x74
    response (mirrors TestExtendedCommand.test_correlated_response)."""
    transport, svc = _make_service(event_loop, 'test_friction_ext')
    tx = _make_tx_bus('test_friction_ext')
    try:
        async def run_test():
            cmd_task = asyncio.ensure_future(
                svc.ext_command(ExtSubCmd.GET_CONTROL_CONFIG, 0x74,
                                timeout=2.0))
            await asyncio.sleep(0.1)
            resp_data = (bytes([0x0B, 0x74, ExtStatus.OK, ExtType.FLOAT32])
                         + struct.pack('<f', 1.5))
            tx.send(_can.Message(
                arbitration_id=make_frame_id(0, CmdId.EXTENDED_COMMAND),
                data=resp_data, is_extended_id=False))
            pump_task = asyncio.ensure_future(transport.pump())
            result = await cmd_task
            pump_task.cancel()
            try:
                await pump_task
            except asyncio.CancelledError:
                pass
            return result

        result = event_loop.run_until_complete(run_test())
        assert result['status'] == ExtStatus.OK
        assert result['value'] == pytest.approx(1.5)
    finally:
        tx.shutdown()
        transport.close()
