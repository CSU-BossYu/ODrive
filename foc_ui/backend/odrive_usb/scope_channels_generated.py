"""Generated from docs/scope_channel_schema.json."""

from dataclasses import dataclass

@dataclass(frozen=True)
class ScopeChannel:
    id: int
    name: str
    wire_type: str
    unit: str
    display_min: float
    display_max: float
    max_sample_rate_hz: int
    threshold_allowed: bool
    source: str

MAX_CAPTURE_CHANNELS = 8
MAX_PRE_SAMPLES = 64
MAX_POST_SAMPLES = 128
MAX_BATCH_SAMPLES = 6

SCOPE_CHANNELS = (
    ScopeChannel(1, 'control_sequence', 'u32', 'count', 0, 100000, 10000, False, 'control_sequence'),
    ScopeChannel(2, 'timestamp_cycles', 'u32', 'cycles', 0, 4294967295, 10000, False, 'timestamp_cycles'),
    ScopeChannel(3, 'state_epoch', 'u32', 'epoch', 0, 4294967295, 1000, False, 'state_epoch'),
    ScopeChannel(4, 'SafetyState', 'u8', 'enum', 0, 7, 1000, True, 'safety_state'),
    ScopeChannel(5, 'Operation', 'u8', 'enum', 0, 3, 1000, True, 'operation'),
    ScopeChannel(6, 'phase', 'f32', 'rad', -3.1415927, 3.1415927, 10000, True, 'phase'),
    ScopeChannel(7, 'phase_velocity', 'f32', 'rad/s', -10000, 10000, 10000, True, 'phase_velocity'),
    ScopeChannel(8, 'position', 'f32', 'rev', -1000, 1000, 10000, True, 'position'),
    ScopeChannel(9, 'velocity', 'f32', 'rev/s', -1000, 1000, 10000, True, 'velocity'),
    ScopeChannel(10, 'Id measured', 'f32', 'A', -50, 50, 10000, True, 'id_measured'),
    ScopeChannel(11, 'Iq measured', 'f32', 'A', -50, 50, 10000, True, 'iq_measured'),
    ScopeChannel(12, 'Id setpoint', 'f32', 'A', -50, 50, 10000, True, 'id_setpoint'),
    ScopeChannel(13, 'Iq setpoint', 'f32', 'A', -50, 50, 10000, True, 'iq_setpoint'),
    ScopeChannel(14, 'torque setpoint', 'f32', 'Nm', -50, 50, 10000, True, 'torque_setpoint'),
    ScopeChannel(15, 'controller output', 'f32', 'command', -1, 1, 10000, True, 'controller_output'),
)
CHANNEL_BY_ID = {item.id: item for item in SCOPE_CHANNELS}
