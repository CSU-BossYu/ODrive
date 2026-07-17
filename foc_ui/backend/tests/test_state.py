"""Unit tests for odrive_can.state (AxisCache + telemetry synthesis).

Run: pytest tests/test_state.py -v
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.state import (
    AxisCache, HeartbeatState, EncoderState, IqState,
    BusState, ErrorDetail, ControlState,
    ODRIVE_CHANNELS, ODRIVE_CHANNEL_BY_KEY, ODRIVE_DEFAULT_VISIBLE,
    channels_as_json,
)


class TestAxisCacheFresh:
    def test_all_zeros(self):
        cache = AxisCache()
        assert cache.heartbeat.axis_error == 0
        assert cache.heartbeat.axis_state == 0
        assert cache.encoder.pos_estimate == 0.0
        assert cache.iq.iq_setpoint == 0.0
        assert cache.bus.vbus == 0.0

    def test_synthesize_keys_match_channel_table(self):
        cache = AxisCache()
        t = cache.synthesize_telemetry()
        expected_keys = {c.key for c in ODRIVE_CHANNELS}
        assert len(t) == len(expected_keys)
        assert set(t.keys()) == expected_keys

    def test_synthesize_zeros(self):
        cache = AxisCache()
        t = cache.synthesize_telemetry()
        assert t['pos'] == 0.0
        assert t['vel'] == 0.0
        assert t['axis_error'] == 0
        assert t['traj_done'] == 0.0


class TestAxisCacheUpdates:
    def test_heartbeat_update(self):
        cache = AxisCache()
        cache.heartbeat.axis_error = 0x41
        cache.heartbeat.axis_state = 8
        t = cache.synthesize_telemetry()
        assert t['axis_error'] == 0x41
        assert t['axis_state'] == 8

    def test_encoder_update(self):
        cache = AxisCache()
        cache.encoder.pos_estimate = 1.5
        cache.encoder.vel_estimate = 10.25
        t = cache.synthesize_telemetry()
        assert t['pos'] == 1.5
        assert t['vel'] == 10.25

    def test_iq_update(self):
        cache = AxisCache()
        cache.iq.iq_setpoint = 2.5
        cache.iq.iq_measured = 2.3
        t = cache.synthesize_telemetry()
        assert t['iq_sp'] == 2.5
        assert t['iq_meas'] == 2.3

    def test_bus_update(self):
        cache = AxisCache()
        cache.bus.vbus = 48.0
        cache.bus.ibus = 1.5
        t = cache.synthesize_telemetry()
        assert t['vbus'] == 48.0
        assert t['ibus'] == 1.5

    def test_error_update(self):
        cache = AxisCache()
        cache.errors.motor_error = 0x1000
        cache.errors.odrive_error = 0x10
        cache.errors.encoder_error = 0x04
        cache.errors.controller_error = 0x80
        t = cache.synthesize_telemetry()
        assert t['motor_err'] == 0x1000
        assert t['odrv_err'] == 0x10
        assert t['enc_err'] == 0x04
        assert t['ctrl_err'] == 0x80

    def test_control_update(self):
        cache = AxisCache()
        cache.control.control_mode = 2
        cache.control.input_mode = 2
        t = cache.synthesize_telemetry()
        assert t['ctrl_mode'] == 2
        assert t['input_mode'] == 2

    def test_traj_done(self):
        cache = AxisCache()
        cache.heartbeat.trajectory_done = True
        t = cache.synthesize_telemetry()
        assert t['traj_done'] == 1.0

    def test_traj_done_false(self):
        cache = AxisCache()
        t = cache.synthesize_telemetry()
        assert t['traj_done'] == 0.0

    def test_control_runtime_flags(self):
        cache = AxisCache()
        cache.heartbeat.comm_timeout = True
        cache.heartbeat.quick_stop_active = True
        cache.heartbeat.holding = True
        cache.heartbeat.cmd_watchdog_expired = True
        cache.heartbeat.mit_frame_stale = True
        cache.heartbeat.running = True
        cache.heartbeat.controller_flags = 0x7E
        t = cache.synthesize_telemetry()
        assert t['comm_timeout'] == 1.0
        assert t['quick_stop'] == 1.0
        assert t['holding'] == 1.0
        assert t['cmd_wdog'] == 1.0
        assert t['mit_stale'] == 1.0
        assert t['running'] == 1.0
        assert t['ctrl_flags'] == 0x7E


class TestSnapshot:
    def test_snapshot_keys(self):
        cache = AxisCache()
        s = cache.snapshot()
        assert 'heartbeat_alive' in s
        assert 'axis_state' in s
        assert 'pos_turns' in s
        assert 'vel_turns_per_s' in s
        assert 'iq_setpoint' in s
        assert 'vbus' in s
        assert 'control_mode' in s

    def test_heartbeat_alive(self):
        cache = AxisCache()
        assert cache.snapshot()['heartbeat_alive'] is False
        cache.heartbeat.last_ts = time.monotonic()
        assert cache.snapshot()['heartbeat_alive'] is True
        cache.heartbeat.last_ts = time.monotonic() - 2.0
        assert cache.snapshot()['heartbeat_alive'] is False


class TestChannelTable:
    def test_channel_count(self):
        assert len(ODRIVE_CHANNELS) == 24

    def test_channel_by_key(self):
        assert ODRIVE_CHANNEL_BY_KEY['pos'].label == 'Position'
        assert ODRIVE_CHANNEL_BY_KEY['vbus'].unit == 'V'

    def test_default_visible(self):
        assert 'pos' in ODRIVE_DEFAULT_VISIBLE
        assert 'vel' in ODRIVE_DEFAULT_VISIBLE
        assert len(ODRIVE_DEFAULT_VISIBLE) == 6

    def test_channels_as_json(self):
        j = channels_as_json()
        assert len(j) == len(ODRIVE_CHANNELS)
        assert j[0]['key'] == 'pos'
        assert all('index' in c and 'key' in c and 'label' in c for c in j)
