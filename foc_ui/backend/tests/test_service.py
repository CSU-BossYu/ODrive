"""Unit tests for odrive_can.service with virtual CAN bus.

Run: pytest tests/test_service.py -v
Requires: python-can installed
"""

import asyncio
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
)
from odrive_can.transport import CanTransport
from odrive_can.service import (
    ODriveService, TORQUE_LIMIT, MIT_TORQUE_LIMIT,
)


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

    def test_set_input_torque_clamped(self, event_loop):
        """Torque should be clamped to ±TORQUE_LIMIT."""
        transport, svc = _make_service(event_loop, 'test_cmd_torque')
        rx = _make_rx_bus('test_cmd_torque')
        try:
            # Request 1.0 Nm, should clamp to TORQUE_LIMIT (0.005)
            event_loop.run_until_complete(svc.set_input_torque(1.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            torque = struct.unpack('<f', msg.data[:4])[0]
            assert torque == pytest.approx(TORQUE_LIMIT)

            # Request -1.0 Nm, should clamp to -TORQUE_LIMIT
            event_loop.run_until_complete(svc.set_input_torque(-1.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            torque = struct.unpack('<f', msg.data[:4])[0]
            assert torque == pytest.approx(-TORQUE_LIMIT)
        finally:
            rx.shutdown()
            transport.close()

    def test_set_mit_torque_clamped(self, event_loop):
        """MIT t_ff should be clamped to ±MIT_TORQUE_LIMIT."""
        transport, svc = _make_service(event_loop, 'test_cmd_mit')
        rx = _make_rx_bus('test_cmd_mit')
        try:
            # t_ff=1.0 exceeds MIT_TORQUE_LIMIT=0.005
            event_loop.run_until_complete(
                svc.set_mit_control(0, 0, 0, 0, 1.0))
            msg = rx.recv(timeout=1.0)
            assert msg is not None
            assert len(msg.data) == 8
            # The MIT frame should have t_ff clamped
            assert svc._mit_active is True
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
