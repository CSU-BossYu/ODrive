"""Unit tests for odrive_can.transport using python-can virtual bus.

Run: pytest tests/test_transport.py -v
Requires: python-can installed (pip install python-can==4.4.2)
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

from odrive_can.protocol import CmdId, make_frame_id
from odrive_can.transport import CanTransport


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def transport(event_loop):
    t = CanTransport(event_loop, node_id=0)
    yield t
    if t.is_open:
        t.close()


# --------------------------------------------------------------------------- #
# Lifecycle
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestLifecycle:
    def test_open_close(self, transport):
        transport.open(interface='virtual', channel='test_open_close')
        assert transport.is_open
        assert transport.interface == 'virtual'
        assert transport.channel == 'test_open_close'
        transport.close()
        assert not transport.is_open

    def test_double_open_raises(self, transport):
        transport.open(interface='virtual', channel='test_double_open')
        with pytest.raises(RuntimeError, match='already open'):
            transport.open(interface='virtual', channel='test_double_open2')


# --------------------------------------------------------------------------- #
# Send
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestSend:
    def test_send_frame(self, event_loop):
        """Send a frame and verify it arrives on a second virtual bus."""
        transport = CanTransport(event_loop, node_id=0)
        transport.open(interface='virtual', channel='test_send')

        # Create a second bus on the same channel to receive
        rx_bus = _can.Bus(interface='virtual', channel='test_send')
        try:
            data = struct.pack('<I', 8)  # CLOSED_LOOP_CONTROL
            transport.send(CmdId.SET_AXIS_STATE, data)
            assert transport.frames_tx == 1

            # Receive on second bus
            msg = rx_bus.recv(timeout=1.0)
            assert msg is not None
            assert msg.arbitration_id == make_frame_id(0, CmdId.SET_AXIS_STATE)
            assert bytes(msg.data) == data
        finally:
            rx_bus.shutdown()
            transport.close()


# --------------------------------------------------------------------------- #
# Receive
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestReceive:
    def test_receive_with_callback(self, event_loop):
        """Verify that frames from the bus are dispatched via callback."""
        received = []

        async def on_frame(cmd_id, data, ts):
            received.append((cmd_id, data))

        transport = CanTransport(event_loop, node_id=0, on_frame=on_frame)
        transport.open(interface='virtual', channel='test_recv')

        # Send a heartbeat from a second bus
        tx_bus = _can.Bus(interface='virtual', channel='test_recv')
        try:
            hb_data = struct.pack('<I', 0) + bytes([1, 0, 0, 0])
            msg = _can.Message(
                arbitration_id=make_frame_id(0, CmdId.HEARTBEAT),
                data=hb_data,
                is_extended_id=False,
            )
            tx_bus.send(msg)

            # Run pump briefly to process the frame
            async def run_pump():
                task = asyncio.ensure_future(transport.pump())
                await asyncio.sleep(0.3)
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

            event_loop.run_until_complete(run_pump())

            assert transport.frames_rx >= 1
            assert len(received) >= 1
            assert received[0][0] == CmdId.HEARTBEAT
        finally:
            tx_bus.shutdown()
            transport.close()

    def test_node_id_filter(self, event_loop):
        """Frames for a different node_id should be filtered out."""
        received = []

        async def on_frame(cmd_id, data, ts):
            received.append(cmd_id)

        # Transport listening for node_id=0
        transport = CanTransport(event_loop, node_id=0, on_frame=on_frame)
        transport.open(interface='virtual', channel='test_filter')

        tx_bus = _can.Bus(interface='virtual', channel='test_filter')
        try:
            # Send a frame for node_id=1 (should be filtered)
            msg = _can.Message(
                arbitration_id=make_frame_id(1, CmdId.HEARTBEAT),
                data=struct.pack('<I', 0) + bytes([1, 0, 0, 0]),
                is_extended_id=False,
            )
            tx_bus.send(msg)

            async def run_pump():
                task = asyncio.ensure_future(transport.pump())
                await asyncio.sleep(0.3)
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

            event_loop.run_until_complete(run_pump())

            # Should have filtered out the node_id=1 frame
            assert len(received) == 0
            assert transport.frames_rx == 0
        finally:
            tx_bus.shutdown()
            transport.close()


# --------------------------------------------------------------------------- #
# Request helper
# --------------------------------------------------------------------------- #

@pytest.mark.skipif(not _HAS_CAN, reason='python-can not installed')
class TestSendRequest:
    def test_request_sends_zero_dlc(self, event_loop):
        # GET_* requests must be DLC=0 (empty data). This ODrive firmware
        # branch only responds to zero-length GET requests; sending 8 zero
        # bytes (DLC=8) gets no response.
        transport = CanTransport(event_loop, node_id=0)
        transport.open(interface='virtual', channel='test_request')

        rx_bus = _can.Bus(interface='virtual', channel='test_request')
        try:
            transport.send_request(CmdId.GET_ENCODER_ESTIMATES)
            msg = rx_bus.recv(timeout=1.0)
            assert msg is not None
            assert len(msg.data) == 0
            assert msg.dlc == 0
        finally:
            rx_bus.shutdown()
            transport.close()
