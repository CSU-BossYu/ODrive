"""Unit tests for odrive_can.ws_hub and odrive_can.recorder.

Run: pytest tests/test_ws_hub.py tests/test_recorder.py -v
"""

import asyncio
import json
import os
import sys
import tempfile
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.ws_hub import ODriveWsHub
from odrive_can.recorder import CanRecorder, CAN_COLUMNS


# --------------------------------------------------------------------------- #
# Mock WebSocket
# --------------------------------------------------------------------------- #

class MockWebSocket:
    """Minimal mock for FastAPI WebSocket."""

    def __init__(self):
        self.sent: list[str] = []
        self.closed = False

    async def accept(self):
        pass

    async def send_text(self, text: str):
        if self.closed:
            raise ConnectionError('closed')
        self.sent.append(text)

    async def receive_text(self):
        # Simulate a long pause then close
        await asyncio.sleep(10)
        raise Exception('disconnected')

    def close(self):
        self.closed = True


# --------------------------------------------------------------------------- #
# WsHub Tests
# --------------------------------------------------------------------------- #

class TestWsHubLifecycle:
    def test_attach_detach(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))
        assert hub.has_clients()
        assert hub.client_count == 1
        loop.run_until_complete(hub.detach(ws))
        assert not hub.has_clients()
        loop.close()

    def test_multiple_clients(self):
        hub = ODriveWsHub()
        ws1 = MockWebSocket()
        ws2 = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws1))
        loop.run_until_complete(hub.attach(ws2))
        assert hub.client_count == 2
        loop.run_until_complete(hub.detach(ws1))
        assert hub.client_count == 1
        loop.close()


class TestWsHubBroadcast:
    def test_broadcast_telemetry(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        ch = {'pos': 1.5, 'vel': 10.0, 'iq_sp': 2.0, 'iq_meas': 1.8,
              'vbus': 48.0, 'ibus': 1.0, 'axis_error': 0, 'axis_state': 8,
              'ctrl_mode': 2, 'input_mode': 2, 'motor_err': 0,
              'enc_err': 0, 'ctrl_err': 0, 'traj_done': 0.0}
        loop.run_until_complete(hub.broadcast_telemetry(ch, time.monotonic()))

        assert len(ws.sent) == 1
        msg = json.loads(ws.sent[0])
        assert msg['type'] == 'telemetry'
        assert msg['ch']['pos'] == 1.5
        assert len(msg['ch']) == 14
        loop.close()

    def test_broadcast_heartbeat(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        hb = {'axis_error': 0, 'axis_state': 1, 'motor_err_flag': False,
              'encoder_err_flag': False, 'controller_err_flag': False,
              'traj_done': False}
        loop.run_until_complete(hub.broadcast_heartbeat(hb, time.monotonic()))

        assert len(ws.sent) == 1
        msg = json.loads(ws.sent[0])
        assert msg['type'] == 'heartbeat'
        assert msg['axis_state'] == 1
        loop.close()

    def test_broadcast_status(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        loop.run_until_complete(hub.broadcast_status(
            connected=True, interface='pcan', channel='PCAN_USBBUS1',
            node_id=0, frames_rx=100, frames_tx=50, bus_errors=0,
            poll_hz=20.0))

        msg = json.loads(ws.sent[0])
        assert msg['type'] == 'status'
        assert msg['connected'] is True
        assert msg['interface'] == 'pcan'
        assert msg['frames_rx'] == 100
        loop.close()

    def test_broadcast_log(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        loop.run_until_complete(hub.broadcast_log('[SAFETY] WS watchdog'))
        msg = json.loads(ws.sent[0])
        assert msg['type'] == 'log'
        assert msg['tag'] == 'SAFETY'
        assert 'watchdog' in msg['text']
        loop.close()

    def test_broadcast_ext_resp(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        resp = {'sub_cmd': 1, 'item': 0, 'status': 0, 'value': 8}
        loop.run_until_complete(hub.broadcast_ext_resp(resp))
        msg = json.loads(ws.sent[0])
        assert msg['type'] == 'ext_resp'
        assert msg['value'] == 8
        loop.close()

    def test_dead_client_cleanup(self):
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        # Close the mock so sends fail
        ws.close()
        loop.run_until_complete(hub.broadcast_log('[TEST] fail'))

        assert not hub.has_clients()
        loop.close()

    def test_downsampling(self):
        """Rapid telemetry broadcasts should be downsampled to ~60Hz."""
        hub = ODriveWsHub()
        ws = MockWebSocket()
        loop = asyncio.new_event_loop()
        loop.run_until_complete(hub.attach(ws))

        ch = {'pos': 0, 'vel': 0, 'iq_sp': 0, 'iq_meas': 0,
              'vbus': 0, 'ibus': 0, 'axis_error': 0, 'axis_state': 0,
              'ctrl_mode': 0, 'input_mode': 0, 'motor_err': 0,
              'enc_err': 0, 'ctrl_err': 0, 'traj_done': 0.0}

        async def rapid_broadcast():
            for i in range(100):
                await hub.broadcast_telemetry(ch, time.monotonic())
                await asyncio.sleep(0.001)  # 1ms between calls = 1000Hz

        loop.run_until_complete(rapid_broadcast())

        # 100 calls at 1ms intervals (~100ms total), 60Hz cap = ~6 sends
        assert len(ws.sent) < 20  # Well below 100
        assert len(ws.sent) >= 1  # At least one got through
        loop.close()


# --------------------------------------------------------------------------- #
# Recorder Tests
# --------------------------------------------------------------------------- #

class TestRecorder:
    def test_start_stop(self):
        rec = CanRecorder()
        path = os.path.join(tempfile.gettempdir(), 'test_rec.csv')
        try:
            rec.start(path)
            assert rec.is_recording
            assert rec.path == os.path.abspath(path)
            result = rec.stop()
            assert not rec.is_recording
            assert result == os.path.abspath(path)
            assert os.path.exists(path)
        finally:
            if os.path.exists(path):
                os.remove(path)

    def test_record_rows(self):
        rec = CanRecorder()
        path = os.path.join(tempfile.gettempdir(), 'test_rec_rows.csv')
        try:
            rec.start(path)
            ch = {'pos': 1.5, 'vel': 10.0, 'iq_sp': 2.0, 'iq_meas': 1.8,
                  'vbus': 48.0, 'ibus': 1.0, 'axis_error': 0, 'axis_state': 8,
                  'ctrl_mode': 2, 'input_mode': 2, 'motor_err': 0,
                  'enc_err': 0, 'ctrl_err': 0, 'traj_done': 0.0}
            for i in range(10):
                rec.record(ch, time.monotonic() + i * 0.05)
            rec.stop()

            with open(path, 'r') as f:
                lines = f.readlines()
            # 1 header + 10 data rows
            assert len(lines) == 11
            assert rec.rows_written == 10
        finally:
            if os.path.exists(path):
                os.remove(path)

    def test_columns(self):
        rec = CanRecorder()
        path = os.path.join(tempfile.gettempdir(), 'test_rec_cols.csv')
        try:
            rec.start(path)
            rec.stop()
            with open(path, 'r') as f:
                header = f.readline().strip()
            assert header == ','.join(CAN_COLUMNS)
        finally:
            if os.path.exists(path):
                os.remove(path)

    def test_float_rounding(self):
        rec = CanRecorder()
        path = os.path.join(tempfile.gettempdir(), 'test_rec_round.csv')
        try:
            rec.start(path)
            ch = {'pos': 1.123456789, 'vel': 0, 'iq_sp': 0, 'iq_meas': 0,
                  'vbus': 0, 'ibus': 0, 'axis_error': 0, 'axis_state': 0,
                  'ctrl_mode': 0, 'input_mode': 0, 'motor_err': 0,
                  'enc_err': 0, 'ctrl_err': 0, 'traj_done': 0.0}
            rec.record(ch, 1000.0)
            rec.stop()
            with open(path, 'r') as f:
                lines = f.readlines()
            # Second line (data row) should have rounded float
            assert '1.123457' in lines[1]
        finally:
            if os.path.exists(path):
                os.remove(path)

    def test_double_start_raises(self):
        rec = CanRecorder()
        path = os.path.join(tempfile.gettempdir(), 'test_rec_dbl.csv')
        try:
            rec.start(path)
            with pytest.raises(RuntimeError, match='already recording'):
                rec.start(path + '.2')
            rec.stop()
        finally:
            if os.path.exists(path):
                os.remove(path)
