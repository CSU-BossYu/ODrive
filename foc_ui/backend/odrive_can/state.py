"""Axis state cache and telemetry synthesizer.

Maintains a live cache of the ODrive axis state, synthesized from multiple
CAN messages into a unified telemetry dict. This is the key bridge between
CAN's distributed message model and the waveform panel's flat-channel model.

Serial firmware sends one 36-byte frame with all 16 channels at once.
CAN gives separate messages for heartbeat, encoder estimates, Iq, and bus
voltage/current. The AxisCache merges them into one flat dict.

Also defines the ODrive CAN channel table and default visible
set for the waveform panel.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any

HEARTBEAT_STALE_S = 1.0

# --------------------------------------------------------------------------- #
# Sub-cache dataclasses
# --------------------------------------------------------------------------- #


@dataclass
class HeartbeatState:
    axis_error: int = 0
    axis_state: int = 0
    motor_err_flag: bool = False
    encoder_err_flag: bool = False
    controller_err_flag: bool = False
    comm_timeout: bool = False
    quick_stop_active: bool = False
    holding: bool = False
    cmd_watchdog_expired: bool = False
    mit_frame_stale: bool = False
    running: bool = False
    trajectory_done: bool = False
    controller_flags: int = 0
    safety_state: int = 0
    operation: int = 0
    readiness_flags: int = 0
    product_flags: int = 0
    last_ts: float = 0.0

    @property
    def is_alive(self) -> bool:
        return self.last_ts > 0.0 and (time.monotonic() - self.last_ts) <= HEARTBEAT_STALE_S


@dataclass
class EncoderState:
    pos_estimate: float = 0.0   # rev
    vel_estimate: float = 0.0   # rev/s
    shadow_count: int = 0       # cumulative encoder count (linear, from Get_Encoder_Count)
    count_in_cpr: int = 0       # encoder count modulo cpr (0..cpr-1)
    last_ts: float = 0.0


@dataclass
class IqState:
    iq_setpoint: float = 0.0    # A
    iq_measured: float = 0.0    # A
    last_ts: float = 0.0


@dataclass
class BusState:
    vbus: float = 0.0           # V
    ibus: float = 0.0           # A
    last_ts: float = 0.0


@dataclass
class ErrorDetail:
    odrive_error: int = 0
    motor_error: int = 0
    encoder_error: int = 0
    controller_error: int = 0
    last_ts: float = 0.0


@dataclass
class ControlState:
    control_mode: int = 0
    input_mode: int = 0
    last_ts: float = 0.0


# --------------------------------------------------------------------------- #
# Axis Cache
# --------------------------------------------------------------------------- #


@dataclass
class AxisCache:
    """Unified cache for all ODrive axis state, updated from CAN frames."""
    heartbeat: HeartbeatState = field(default_factory=HeartbeatState)
    encoder: EncoderState = field(default_factory=EncoderState)
    iq: IqState = field(default_factory=IqState)
    bus: BusState = field(default_factory=BusState)
    errors: ErrorDetail = field(default_factory=ErrorDetail)
    control: ControlState = field(default_factory=ControlState)

    def synthesize_telemetry(self) -> dict[str, Any]:
        """Produce a flat dict for the waveform panel.

        Keys match ODRIVE_CHANNELS below. Error and state channels are
        kept as raw integers; everything else is a float.
        """
        return {
            'pos':        self.encoder.pos_estimate,
            'vel':        self.encoder.vel_estimate,
            'shadow_count': self.encoder.shadow_count,
            'count_in_cpr': self.encoder.count_in_cpr,
            'iq_sp':      self.iq.iq_setpoint,
            'iq_meas':    self.iq.iq_measured,
            'vbus':       self.bus.vbus,
            'ibus':       self.bus.ibus,
            'axis_error': self.heartbeat.axis_error,
            'axis_state': self.heartbeat.axis_state,
            'ctrl_mode':  self.control.control_mode,
            'input_mode': self.control.input_mode,
            'motor_err':  self.errors.motor_error,
            'enc_err':    self.errors.encoder_error,
            'ctrl_err':   self.errors.controller_error,
            'odrv_err':   self.errors.odrive_error,
            'traj_done':  1.0 if self.heartbeat.trajectory_done else 0.0,
            'comm_timeout': 1.0 if self.heartbeat.comm_timeout else 0.0,
            'quick_stop': 1.0 if self.heartbeat.quick_stop_active else 0.0,
            'holding':    1.0 if self.heartbeat.holding else 0.0,
            'cmd_wdog':   1.0 if self.heartbeat.cmd_watchdog_expired else 0.0,
            'mit_stale':  1.0 if self.heartbeat.mit_frame_stale else 0.0,
            'running':    1.0 if self.heartbeat.running else 0.0,
            'ctrl_flags': self.heartbeat.controller_flags,
        }

    def snapshot(self) -> dict[str, Any]:
        """Full state snapshot for WebSocket status push (not just waveform)."""
        return {
            'heartbeat_alive': self.heartbeat.is_alive,
            'axis_state': self.heartbeat.axis_state,
            'axis_error': self.heartbeat.axis_error,
            'motor_error': self.errors.motor_error,
            'odrive_error': self.errors.odrive_error,
            'encoder_error': self.errors.encoder_error,
            'controller_error': self.errors.controller_error,
            'motor_err_flag': self.heartbeat.motor_err_flag,
            'encoder_err_flag': self.heartbeat.encoder_err_flag,
            'controller_err_flag': self.heartbeat.controller_err_flag,
            'comm_timeout': self.heartbeat.comm_timeout,
            'quick_stop_active': self.heartbeat.quick_stop_active,
            'holding': self.heartbeat.holding,
            'cmd_watchdog_expired': self.heartbeat.cmd_watchdog_expired,
            'mit_frame_stale': self.heartbeat.mit_frame_stale,
            'running': self.heartbeat.running,
            'trajectory_done': self.heartbeat.trajectory_done,
            'controller_flags': self.heartbeat.controller_flags,
            'safety_state': self.heartbeat.safety_state,
            'operation': self.heartbeat.operation,
            'readiness_flags': self.heartbeat.readiness_flags,
            'product_flags': self.heartbeat.product_flags,
            'pos_turns': self.encoder.pos_estimate,
            'vel_turns_per_s': self.encoder.vel_estimate,
            'shadow_count': self.encoder.shadow_count,
            'count_in_cpr': self.encoder.count_in_cpr,
            'iq_setpoint': self.iq.iq_setpoint,
            'iq_measured': self.iq.iq_measured,
            'vbus': self.bus.vbus,
            'ibus': self.bus.ibus,
            'control_mode': self.control.control_mode,
            'input_mode': self.control.input_mode,
        }


# --------------------------------------------------------------------------- #
# ODrive CAN Channel Table
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ODriveChannelDef:
    """One telemetry channel for the CAN waveform display."""
    index: int
    key: str
    label: str
    unit: str
    group: str  # 'pos' | 'vel' | 'cur' | 'bus' | 'state'


ODRIVE_CHANNELS: list[ODriveChannelDef] = [
    ODriveChannelDef(0,  'pos',        'Position',       'rev',   'pos'),
    ODriveChannelDef(1,  'vel',        'Velocity',       'rev/s', 'vel'),
    ODriveChannelDef(2,  'iq_sp',      'Iq setpoint',    'A',     'cur'),
    ODriveChannelDef(3,  'iq_meas',    'Iq measured',    'A',     'cur'),
    ODriveChannelDef(4,  'vbus',       'Bus voltage',    'V',     'bus'),
    ODriveChannelDef(5,  'ibus',       'Bus current',    'A',     'bus'),
    ODriveChannelDef(6,  'axis_error', 'Axis error',     'bits',  'state'),
    ODriveChannelDef(7,  'axis_state', 'Axis state',     '',      'state'),
    ODriveChannelDef(8,  'ctrl_mode',  'Control mode',   '',      'state'),
    ODriveChannelDef(9,  'input_mode', 'Input mode',     '',      'state'),
    ODriveChannelDef(10, 'motor_err',  'Motor error',    'bits',  'state'),
    ODriveChannelDef(11, 'enc_err',    'Encoder error',  'bits',  'state'),
    ODriveChannelDef(12, 'ctrl_err',   'Controller err', 'bits',  'state'),
    ODriveChannelDef(13, 'odrv_err',   'ODrive error',   'bits',  'state'),
    ODriveChannelDef(14, 'traj_done',  'Traj done',      '',      'state'),
    ODriveChannelDef(15, 'comm_timeout', 'Comm timeout', '',      'state'),
    ODriveChannelDef(16, 'quick_stop', 'Quick stop',     '',      'state'),
    ODriveChannelDef(17, 'holding',    'Holding',        '',      'state'),
    ODriveChannelDef(18, 'cmd_wdog',   'Cmd watchdog',   '',      'state'),
    ODriveChannelDef(19, 'mit_stale',  'MIT stale',      '',      'state'),
    ODriveChannelDef(20, 'running',    'Running',        '',      'state'),
    ODriveChannelDef(21, 'ctrl_flags', 'Control flags',  'bits',  'state'),
    ODriveChannelDef(22, 'shadow_count', 'Encoder count', 'count', 'pos'),
    ODriveChannelDef(23, 'count_in_cpr', 'Count in CPR',  'count', 'pos'),
]

ODRIVE_CHANNEL_BY_KEY: dict[str, ODriveChannelDef] = {
    c.key: c for c in ODRIVE_CHANNELS
}

ODRIVE_DEFAULT_VISIBLE: list[str] = [
    'pos', 'vel', 'iq_sp', 'iq_meas', 'vbus', 'ibus',
]


def channels_as_json() -> list[dict]:
    """Serialize ODRIVE_CHANNELS for the /api/can/channels endpoint."""
    return [
        {'index': c.index, 'key': c.key, 'label': c.label,
         'unit': c.unit, 'group': c.group}
        for c in ODRIVE_CHANNELS
    ]
