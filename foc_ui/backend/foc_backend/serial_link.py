"""Serial port reader and 3-way byte-stream demultiplexer.

The firmware sends three kinds of data over a single USB CDC endpoint:

  1. Binary telemetry frames: ``0x55 0xAA <seq> <32 data> <crc8>``, fixed 36
     bytes, sent at up to 500 Hz. Frames are atomic on the wire (USB CDC
     guarantees this at the frame level) but the host may see them interleaved
     with ASCII text at any byte offset, because pyserial is a byte stream.

  2. CLI responses: ASCII lines prefixed ``[CLI] `` ending in ``\\r\\n``.

  3. Log lines: ASCII lines prefixed with other bracket tags (``[FOC]``,
     ``[BOOT]``, ``[CAN]``, ``[DRV8301]``, ``[FAULT]``, ...), ending in
     ``\\r\\n``.

This module runs a dedicated reader thread and feeds parsed events into an
asyncio queue. The asyncio side dispatches them to:

  - telemetry callbacks (one per decoded frame, with CRC pass/fail + seq gap)
  - the CLI responder (for [CLI] lines)
  - log callbacks (for any other ASCII line)

Demux strategy
--------------
Bytes are accumulated in a single buffer. At each iteration:

  * If the buffer head is ``0x55 0xAA`` and we have 36 bytes, try to parse a
    frame. On CRC pass, emit it and consume 36 bytes. On CRC fail, consume 2
    bytes (the sync header) and resync -- this discards a corrupt frame but
    keeps ASCII bytes that may have followed from being misread.

  * Otherwise, scan the buffer for the next ``\n``. Everything up to and
    including it is one ASCII line (CR is stripped). The line is dispatched
    by its leading tag.

  * If a byte happens to equal 0x55 inside ASCII text but is not followed by
    0xAA, it is treated as ASCII and consumed normally. The 0x55+0xAA pair
    inside ASCII text is vanishingly unlikely (would require '[' = 0x5B etc.).
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Awaitable, Callable, Optional

import serial

from .telemetry import (
    FRAME_SIZE,
    SYNC0,
    SYNC1,
    TelemetryFrame,
    parse_frame,
)

# Async callbacks invoked on the event-loop thread.
TelemetryCallback = Callable[[TelemetryFrame, float, int], Awaitable[None]]
# args: (frame, timestamp_monotonic, seq_gap)   seq_gap>0 means dropped frames
AsciiLineCallback = Callable[[str], Awaitable[None]]
# args: (raw_line,)   raw_line includes the leading tag, e.g. "[FOC] ..."

LOG_TAGS = ('[FOC]', '[BOOT]', '[CAN]', '[DRV8301]', '[FAULT]',
            '[CAL_CFG]', '[MOTOR_CAL]', '[MOTOR_CFG]', '[CLI]')


class SerialLink:
    """Owns the serial port and a reader thread; emits async events."""

    def __init__(self,
                 loop: asyncio.AbstractEventLoop,
                 *,
                 on_telemetry: Optional[TelemetryCallback] = None,
                 on_ascii_line: Optional[AsciiLineCallback] = None):
        self._loop = loop
        self._on_telemetry = on_telemetry
        self._on_ascii_line = on_ascii_line

        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        # Cross-thread handoff: the reader thread puts (kind, payload) tuples,
        # the event loop drains them. This avoids calling asyncio callables
        # directly from the thread.
        self._q: asyncio.Queue = asyncio.Queue()

        # Stats (read from either side; ints are atomic enough for stats).
        self.frames_total = 0
        self.frames_bad_crc = 0
        self.frames_dropped = 0   # by seq-gap detection
        self.last_seq: Optional[int] = None
        self.port_name: Optional[str] = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def open(self, port: str, baudrate: int = 115200) -> None:
        """Open the port and start the reader thread. Raises on failure."""
        if self._serial is not None:
            raise RuntimeError('serial link already open')
        # CDC virtual serial ports ignore baudrate/parity but pyserial still
        # requires a non-zero value. 115200 is a harmless convention.
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,           # short read timeout so the thread can poll _stop_evt
            write_timeout=1.0,
        )
        self.port_name = port
        self.frames_total = 0
        self.frames_bad_crc = 0
        self.frames_dropped = 0
        self.last_seq = None
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._reader_main, name='serial-reader', daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        t = self._thread
        s = self._serial
        self._thread = None
        self._serial = None
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        if s is not None and s.is_open:
            try:
                s.close()
            except Exception:
                pass
        self.port_name = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def write(self, data: bytes) -> None:
        """Synchronous write (called from event loop, blocks briefly)."""
        if self._serial is None:
            raise RuntimeError('serial link not open')
        # pyserial write is blocking up to write_timeout; for short commands
        # this is effectively instant.
        self._serial.write(data)

    # ------------------------------------------------------------------ #
    # Reader thread
    # ------------------------------------------------------------------ #
    def _reader_main(self) -> None:
        buf = bytearray()
        s = self._serial
        assert s is not None
        try:
            while not self._stop_evt.is_set():
                # Read whatever is available, up to a generous chunk.
                chunk = s.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                buf = self._drain_buffer(buf)
        except Exception as exc:  # noqa: BLE001
            # Push the error to the event loop as a synthetic log line.
            self._put_nowait(('log', f'[serial-reader] error: {exc!r}'))
        finally:
            pass

    def _drain_buffer(self, buf: bytearray) -> bytearray:
        """Process as many complete units as possible from ``buf``.

        Returns the remaining bytes (may be empty)."""
        while buf:
            # Case 1: binary frame at head.
            if len(buf) >= 2 and buf[0] == SYNC0 and buf[1] == SYNC1:
                if len(buf) < FRAME_SIZE:
                    break  # need more bytes
                frame_bytes = bytes(buf[:FRAME_SIZE])
                self._handle_frame_bytes(frame_bytes)
                del buf[:FRAME_SIZE]
                continue

            # Case 2: ASCII line. Find the next newline.
            nl = buf.find(b'\n')
            if nl < 0:
                # No newline yet. But we might be sitting on a stray 0x55
                # that isn't a sync header -- consume one byte at a time only
                # if we're sure it can't start a frame. Since we already
                # checked case 1, the current head is not a sync header; if
                # it's 0x55 alone (followed by non-0xAA), fall through to
                # accumulate. To avoid unbounded growth, cap the ASCII
                # backlog; if exceeded, flush it as a malformed log line.
                if len(buf) > 512:
                    self._handle_ascii(bytes(buf))
                    buf.clear()
                break

            line_bytes = bytes(buf[:nl])
            del buf[:nl + 1]
            # Strip a trailing CR.
            if line_bytes.endswith(b'\r'):
                line_bytes = line_bytes[:-1]
            self._handle_ascii(line_bytes)
        return buf

    def _handle_frame_bytes(self, data: bytes) -> None:
        frame = parse_frame(data)
        if frame is None:
            return  # shouldn't happen (sync verified)
        self.frames_total += 1
        if not frame.crc_ok:
            self.frames_bad_crc += 1
            # Drop silently; do not update last_seq so the gap is attributed
            # to the next good frame.
            return
        gap = 0
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFF
            if frame.seq != expected:
                # Counts frames between last_seq and frame.seq (exclusive).
                gap = (frame.seq - expected) & 0xFF
                self.frames_dropped += gap
        self.last_seq = frame.seq
        self._put_nowait(('telemetry', frame, time.monotonic(), gap))

    def _handle_ascii(self, line_bytes: bytes) -> None:
        # Decode leniently -- the firmware prints ASCII but a stray binary
        # byte should not crash the reader.
        try:
            line = line_bytes.decode('ascii', errors='replace')
        except Exception:
            line = repr(line_bytes)
        # Drop empty lines (firmware emits them between commands).
        if not line.strip():
            return
        self._put_nowait(('ascii', line))

    # ------------------------------------------------------------------ #
    # Cross-thread queue helpers
    # ------------------------------------------------------------------ #
    def _put_nowait(self, item) -> None:
        # Called from the reader thread. schedule_future is not needed; we
        # use call_soon_threadsafe to enqueue into the asyncio.Queue.
        try:
            self._loop.call_soon_threadsafe(self._q.put_nowait, item)
        except RuntimeError:
            # Event loop closed; reader is shutting down. Ignore.
            pass

    async def pump(self) -> None:
        """Event-loop coroutine: drain the queue and dispatch to callbacks.

        Runs forever; cancel it when shutting down.
        """
        while True:
            item = await self._q.get()
            kind = item[0]
            try:
                if kind == 'telemetry':
                    _, frame, ts, gap = item
                    if self._on_telemetry is not None:
                        await self._on_telemetry(frame, ts, gap)
                elif kind == 'ascii':
                    _, line = item
                    if self._on_ascii_line is not None:
                        await self._on_ascii_line(line)
                elif kind == 'log':
                    _, text = item
                    if self._on_ascii_line is not None:
                        await self._on_ascii_line(text)
            except Exception as exc:  # noqa: BLE001
                # Don't let a bad callback kill the pump.
                print(f'[serial-link] dispatch error: {exc!r}', flush=True)
