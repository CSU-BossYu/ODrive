"""Self-test: exercises telemetry parsing, CRC, and the serial-link demux
without any real hardware. Run via:

    .venv\\Scripts\\python.exe -m foc_backend._selftest

It builds synthetic frames + ASCII lines in-memory and feeds them through the
same SerialLink._drain_buffer() the real reader thread uses, asserting that
the right events come out the other side.
"""

from __future__ import annotations

import asyncio
import struct
import sys

from .telemetry import (
    CHANNELS,
    FRAME_SIZE,
    SYNC0,
    SYNC1,
    crc8,
    parse_frame,
)


def build_frame(seq: int, values: dict[str, float]) -> bytes:
    """Build a 36-byte frame mirroring the firmware packing."""
    data = []
    for ch in CHANNELS:
        if ch.key in ('errors', 'state'):
            data.append(int(values.get(ch.key, 0)))
        else:
            data.append(int(values.get(ch.key, 0) * ch.scale))
    payload = struct.pack('<16h', *data)
    frame = bytearray(FRAME_SIZE)
    frame[0] = SYNC0
    frame[1] = SYNC1
    frame[2] = seq & 0xFF
    frame[3:3 + 32] = payload
    frame[35] = crc8(frame, 2, 2 + 33)
    return bytes(frame)


def test_frame_round_trip():
    print('== test_frame_round_trip ==')
    values = {
        'pos': 1.234, 'vel_ref': 2.0, 'vel_fb': 1.95,
        'iq_ref': 0.5, 'iq_meas': 0.48, 'id_meas': 0.01,
        'vq': 2.5, 'vd': 0.1, 'vbus': 24.4, 'phase': 1.57,
        'vel_err': 0.05, 'integrator': 0.3, 'vq_ff': 0.04, 'vd_ff': 0.01,
        'errors': 0x0004,   # bit 2 OVERCURRENT
        'state': 0b00010011,  # running | velocity mode | current_limited
    }
    raw = build_frame(seq=7, values=values)
    assert len(raw) == FRAME_SIZE, f'frame size {len(raw)}'
    frame = parse_frame(raw)
    assert frame is not None
    assert frame.crc_ok, 'CRC should pass'
    assert frame.seq == 7
    # Tolerance: quantization to int16 then back.
    for k, expected in values.items():
        if k in ('errors', 'state'):
            continue
        got = frame.values[k]
        tol = abs(expected) * 0.01 + 1e-3
        assert abs(got - expected) <= tol, f'{k}: expected {expected}, got {got}'
    assert frame.error_bits == 0x0004
    assert frame.state.running
    assert frame.state.control_mode == 1   # velocity
    assert frame.state.current_limited
    print(f'  decoded {len(frame.values)} channels OK; mode={frame.state.mode_name}')


def test_crc_corruption_detected():
    print('== test_crc_corruption_detected ==')
    raw = bytearray(build_frame(seq=1, values={'iq_meas': 0.5}))
    raw[10] ^= 0xFF   # flip a data byte
    frame = parse_frame(bytes(raw))
    assert frame is not None
    assert not frame.crc_ok, 'CRC should fail'
    print('  bad CRC correctly flagged')


def test_seq_wrap():
    print('== test_seq_wrap ==')
    raw = build_frame(seq=255, values={})
    f1 = parse_frame(raw)
    raw2 = build_frame(seq=0, values={})  # next frame wraps to 0
    f2 = parse_frame(raw2)
    assert f1.seq == 255 and f2.seq == 0
    print('  seq 255 -> 0 wrap OK')


def test_serial_link_demux():
    print('== test_serial_link_demux ==')
    from .serial_link import SerialLink

    received = []

    async def on_telemetry(frame, ts, gap):
        received.append(('t', frame.seq, gap))

    async def on_ascii(line):
        received.append(('a', line))

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    link = SerialLink(loop, on_telemetry=on_telemetry, on_ascii_line=on_ascii)

    # Simulate interleaved: ASCII log line + binary frame + CLI response + partial frame.
    frame_a = build_frame(seq=10, values={'iq_meas': 0.3})
    frame_b = build_frame(seq=12, values={'iq_meas': 0.4})   # gap of 1 (seq 11 missing)
    blob = (
        b'[BOOT] motor init starting\r\n'
        + frame_a
        + b'[CLI] stream_hz=100\r\n'
        + frame_b
        + b'[FOC] state=0\r\n'
        + build_frame(seq=13, values={})[:20]   # truncated; should be retained
    )
    remaining = link._drain_buffer(bytearray(blob))
    print(f'  consumed {len(blob) - len(remaining)} bytes, '
          f'{len(remaining)} remain (truncated frame tail)')

    # Drive the pump to dispatch queued events.
    async def run():
        task = asyncio.create_task(link.pump())
        await asyncio.sleep(0.1)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    loop.run_until_complete(run())

    print(f'  received {len(received)} events:')
    for ev in received:
        print(f'    {ev}')

    # Assertions: 2 telemetry frames (with 1 gap on the second), 3 ascii lines.
    telem = [e for e in received if e[0] == 't']
    asciis = [e for e in received if e[0] == 'a']
    assert len(telem) == 2, f'expected 2 frames, got {len(telem)}'
    assert telem[0][1] == 10 and telem[0][2] == 0
    assert telem[1][1] == 12 and telem[1][2] == 1, 'should detect gap of 1'
    assert any('[BOOT]' in e[1] for e in asciis)
    assert any('[CLI]' in e[1] for e in asciis)
    assert any('[FOC]' in e[1] for e in asciis)
    print('  demux OK')


def main():
    test_frame_round_trip()
    test_crc_corruption_detected()
    test_seq_wrap()
    test_serial_link_demux()
    print('\nALL TESTS PASSED')


if __name__ == '__main__':
    try:
        main()
    except AssertionError as e:
        print(f'FAIL: {e}', file=sys.stderr)
        sys.exit(1)
