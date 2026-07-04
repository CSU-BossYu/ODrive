"""Binary telemetry frame definition and parser.

Mirrors the firmware in F:\\souce_code\\foc\\Users\\Motor\\motor_stream.c.
Frame layout (36 bytes, little-endian):
    [0]     0x55 (sync byte 1)
    [1]     0xAA (sync byte 2)
    [2]     sequence number (wraps 0-255, detect dropped frames)
    [3..34] 16 x int16 data
    [35]    CRC-8 (poly=0x07, init=0x00, no reflect, covers bytes 2..34)

The 16 channels, their scale factors and physical units are kept in the
CHANNELS list below -- this is the single source of truth shared with the
frontend via the /api/channels endpoint.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional

# --------------------------------------------------------------------------- #
# Frame constants (must match motor_stream.h)
# --------------------------------------------------------------------------- #

SYNC0: int = 0x55
SYNC1: int = 0xAA
FRAME_SIZE: int = 36
NUM_CHANNELS: int = 16
HEADER_SIZE: int = 3   # sync0 + sync1 + seq
DATA_SIZE: int = 32    # 16 x int16
CRC_OFFSET: int = 35
CRC_COVER_OFFSET: int = 2
CRC_COVER_LEN: int = 33   # seq + 32 data bytes


# --------------------------------------------------------------------------- #
# Channel table (must match motor_stream.c lines 113-134)
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class ChannelDef:
    """One telemetry channel: how to decode the raw int16 into engineering units."""
    index: int          # 0..15
    key: str            # machine name, used in JSON
    label: str          # human label for the UI
    unit: str           # physical unit
    scale: float        # value = raw / scale  (raw is int16)
    group: str          # 'vel' / 'cur' / 'vol' / 'state' for UI grouping

    def decode(self, raw: int) -> float:
        # raw already sign-extended by struct.unpack('<16h')
        return raw / self.scale


# Order MUST match firmware packing order (data[0]..data[15]).
CHANNELS: List[ChannelDef] = [
    ChannelDef(0,  'pos',        'Position',         'rad',   1000.0, 'pos'),
    ChannelDef(1,  'vel_ref',    'Velocity ref',     'rad/s', 100.0,  'vel'),
    ChannelDef(2,  'vel_fb',     'Velocity fb',      'rad/s', 100.0,  'vel'),
    ChannelDef(3,  'iq_ref',     'Iq ref',           'A',     1000.0, 'cur'),
    ChannelDef(4,  'iq_meas',    'Iq meas',          'A',     1000.0, 'cur'),
    ChannelDef(5,  'id_meas',    'Id meas',          'A',     1000.0, 'cur'),
    ChannelDef(6,  'vq',         'Vq',               'V',     100.0,  'vol'),
    ChannelDef(7,  'vd',         'Vd',               'V',     100.0,  'vol'),
    ChannelDef(8,  'vbus',       'Vbus',             'V',     100.0,  'vol'),
    ChannelDef(9,  'phase',      'Phase',            'rad',   1000.0, 'pos'),
    ChannelDef(10, 'vel_err',    'Vel error',        'rad/s', 100.0,  'vel'),
    ChannelDef(11, 'integrator', 'Vel integrator',   'A',     1000.0, 'cur'),
    ChannelDef(12, 'vq_ff',      'Vq feedforward',   'V',     100.0,  'vol'),
    ChannelDef(13, 'vd_ff',      'Vd feedforward',   'V',     100.0,  'vol'),
    ChannelDef(14, 'errors',     'Error flags',      'bits',  1.0,    'state'),
    ChannelDef(15, 'state',      'State flags',      'bits',  1.0,    'state'),
]

_CHANNEL_BY_INDEX = {c.index: c for c in CHANNELS}


# --------------------------------------------------------------------------- #
# Error bits (low 16 bits transmitted over the stream; see motor_errors.h)
# --------------------------------------------------------------------------- #

# Full 28-bit table is used by the frontend to decode the complete error word
# obtained from the periodic `status` ASCII poll. Here we expose the names.
ERROR_BITS: dict = {
    0:  ('DRV_FAULT',              True),   # FATAL
    1:  ('CURRENT_SENSE',          True),   # FATAL
    2:  ('OVERCURRENT',            False),
    3:  ('DC_BUS_UNDERVOLTAGE',    False),
    4:  ('ENCODER',                False),
    5:  ('ENCODER_SPI',            False),
    6:  ('ENCODER_NO_RESPONSE',    False),
    7:  ('PHASE_RESISTANCE',       True),   # FATAL
    8:  ('PHASE_INDUCTANCE',       True),   # FATAL
    9:  ('CONTROL_DEADLINE',       False),
    10: ('SPINOUT',                False),
    11: ('INVALID_STATE',          False),
    12: ('PWM_START',              False),
    13: ('ADC_START',              False),
    14: ('TIM_FAULT',              False),
    15: ('ENCODER_NOT_READY',      False),
    16: ('ENCODER_OFFSET_INVALID', False),
    17: ('ENCODER_DIRECTION_INVALID', False),
    18: ('ENCODER_CPR_MISMATCH',   False),
    19: ('ENCODER_SCAN_TIMEOUT',   False),
    20: ('CONFIG_INVALID',         True),   # FATAL
    23: ('CALIBRATION_MISMATCH',   True),   # FATAL
    24: ('OVERSPEED',              False),
    25: ('DC_BUS_OVERVOLTAGE',     False),
    26: ('ENCODER_TIMEOUT',        False),
    27: ('STALL',                  False),
}

FATAL_MASK: int = 0
for _bit, (_name, fatal) in ERROR_BITS.items():
    if fatal:
        FATAL_MASK |= (1 << _bit)


# --------------------------------------------------------------------------- #
# State-flags bit decoder (channel 15, see motor_stream.c:128-134)
# --------------------------------------------------------------------------- #

@dataclass
class StateFlags:
    running: bool
    control_mode: int        # 0=current, 1=velocity, 2=position
    current_limited: bool
    voltage_limited: bool
    overspeed_error: bool
    trajectory_done: bool

    @classmethod
    def decode(cls, raw: int) -> 'StateFlags':
        # NOTE: firmware also writes overspeed_error to bit 8 (duplicate of bit 6,
        # looks like a bug); we only read bit 6 to match the documented protocol.
        return cls(
            running=bool(raw & (1 << 0)),
            control_mode=(raw >> 1) & 0x07,
            current_limited=bool(raw & (1 << 4)),
            voltage_limited=bool(raw & (1 << 5)),
            overspeed_error=bool(raw & (1 << 6)),
            trajectory_done=bool(raw & (1 << 7)),
        )

    @property
    def mode_name(self) -> str:
        return {0: 'current', 1: 'velocity', 2: 'position'}.get(self.control_mode, f'?{self.control_mode}')


# --------------------------------------------------------------------------- #
# CRC-8 (poly 0x07, init 0x00) -- tableless, matches firmware motor_stream.c:29
# --------------------------------------------------------------------------- #

def crc8(data: bytes, start: int = 0, end: Optional[int] = None) -> int:
    if end is None:
        end = len(data)
    crc = 0x00
    for i in range(start, end):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# --------------------------------------------------------------------------- #
# Frame parsing
# --------------------------------------------------------------------------- #

@dataclass
class TelemetryFrame:
    """One decoded telemetry frame."""
    seq: int
    values: dict       # {channel_key: float}  (errors/state kept as raw int)
    error_bits: int    # channel 14 raw (low 16 bits of firmware error word)
    state: StateFlags
    crc_ok: bool

    def engineering_values(self) -> dict:
        """Values with current/voltage/velocity in physical units; errors/state as int."""
        return dict(self.values)


def _decode_frame_payload(payload: bytes) -> dict:
    """Decode the 32-byte data region into {channel_key: value}.
    The error and state channels keep their raw integer form; everything else
    uses the engineering scale."""
    raw16 = struct.unpack('<16h', payload)
    out = {}
    for ch, raw in zip(CHANNELS, raw16):
        if ch.key in ('errors', 'state'):
            out[ch.key] = raw   # keep as signed int; bitfield semantics
        else:
            out[ch.key] = ch.decode(raw)
    return out


def parse_frame(buf: bytes) -> Optional[TelemetryFrame]:
    """Attempt to parse a 36-byte frame at buf[0..36).
    Returns None if sync/CRC is bad (caller should resync)."""
    if len(buf) < FRAME_SIZE:
        return None
    if buf[0] != SYNC0 or buf[1] != SYNC1:
        return None
    seq = buf[2]
    expected_crc = crc8(buf, CRC_COVER_OFFSET, CRC_COVER_OFFSET + CRC_COVER_LEN)
    crc_ok = (expected_crc == buf[CRC_OFFSET])
    # Parse regardless of CRC -- caller decides whether to drop on bad CRC.
    payload = buf[HEADER_SIZE:HEADER_SIZE + DATA_SIZE]
    values = _decode_frame_payload(payload)
    error_bits = int(values['errors']) & 0xFFFF
    state = StateFlags.decode(int(values['state']))
    return TelemetryFrame(seq=seq, values=values, error_bits=error_bits,
                          state=state, crc_ok=crc_ok)


def channels_as_json() -> list:
    """Serialize CHANNELS for the frontend /api/channels endpoint."""
    return [
        {'index': c.index, 'key': c.key, 'label': c.label,
         'unit': c.unit, 'scale': c.scale, 'group': c.group}
        for c in CHANNELS
    ]


def errors_as_json() -> list:
    """Serialize ERROR_BITS for the frontend error decoder."""
    return [{'bit': b, 'name': n, 'fatal': f} for b, (n, f) in sorted(ERROR_BITS.items())]
