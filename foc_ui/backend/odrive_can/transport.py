"""python-can Bus wrapper with reader thread and async dispatch.

Mirrors the pattern of ``foc_backend.serial_link.SerialLink``:
  - A dedicated reader thread polls ``bus.recv()`` for incoming CAN frames
  - Frames are filtered by node_id and dispatched to an asyncio callback
    via a cross-thread queue
  - Synchronous ``send()`` for outbound frames (blocking pyserial-style,
    run in executor by the caller when needed)

Default configuration:
  - interface = 'pcan'
  - channel = 'PCAN_USBBUS1'
  - bitrate = 1_000_000
  - node_id = 0

Supports all python-can interfaces: pcan, socketcan, kvaser, vector,
ixxat, virtual (for testing).
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Awaitable, Callable, Optional

from .protocol import CmdId, make_frame_id, parse_frame_id

# python-can import -- deferred to allow the package to be imported even
# when python-can is not installed (for static analysis / documentation).
try:
    import can as _can
    _HAS_CAN = True
except ImportError:
    _can = None  # type: ignore[assignment]
    _HAS_CAN = False

# Async callback type: (cmd_id, data_bytes, timestamp_monotonic) -> None
FrameCallback = Callable[[int, bytes, float], Awaitable[None]]


class CanTransport:
    """Owns the CAN bus and a reader thread; emits async frame events."""

    def __init__(self,
                 loop: asyncio.AbstractEventLoop,
                 *,
                 on_frame: Optional[FrameCallback] = None,
                 node_id: int = 0):
        self._loop = loop
        self._on_frame = on_frame
        self._node_id = node_id

        self._bus: Optional[object] = None  # can.BusABC
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        # Lock for thread-safe bus.send() and counter updates
        self._send_lock = threading.Lock()

        # Cross-thread handoff queue
        self._q: asyncio.Queue = asyncio.Queue()

        # Stats
        self.frames_rx: int = 0
        self.frames_tx: int = 0
        self.bus_errors: int = 0

        # Connection info
        self.interface: Optional[str] = None
        self.channel: Optional[str] = None
        self.bitrate: int = 1_000_000

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def open(self, interface: str = 'pcan', channel: str = 'PCAN_USBBUS1',
             bitrate: int = 1_000_000) -> None:
        """Open the CAN bus and start the reader thread.

        Raises RuntimeError if python-can is not installed or bus is
        already open.
        """
        if not _HAS_CAN:
            raise RuntimeError(
                'python-can is not installed. '
                'Run: pip install python-can==4.4.2')
        if self._bus is not None:
            raise RuntimeError('CAN transport already open')

        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.frames_rx = 0
        self.frames_tx = 0
        self.bus_errors = 0
        self._stop_evt.clear()

        self._bus = _can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
        )
        self._thread = threading.Thread(
            target=self._reader_main, name='can-reader', daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stop the reader thread and shut down the CAN bus."""
        self._stop_evt.set()
        t = self._thread
        bus = self._bus
        self._thread = None
        self._bus = None
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        # Acquire _send_lock so we don't shutdown the bus while an executor
        # thread is mid-send (PCAN releases the GIL during I/O, so a
        # concurrent shutdown() and send() on the same handle is undefined
        # behavior and can crash the process).
        if bus is not None:
            with self._send_lock:
                try:
                    bus.shutdown()  # type: ignore[attr-defined]
                except Exception:
                    pass
        self.interface = None
        self.channel = None

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    @property
    def node_id(self) -> int:
        return self._node_id

    # ------------------------------------------------------------------ #
    # Send
    # ------------------------------------------------------------------ #

    def send(self, cmd_id: CmdId | int, data: bytes,
             node_id: Optional[int] = None) -> None:
        """Send a CAN frame synchronously. Thread-safe via send lock.

        Args:
            cmd_id: Command ID (0x00..0x1F)
            data: Payload (0-8 bytes for classic CAN)
            node_id: Override node_id (default: self._node_id)
        """
        bus = self._bus
        if bus is None:
            raise RuntimeError('CAN transport not open')

        nid = node_id if node_id is not None else self._node_id
        arb_id = make_frame_id(nid, int(cmd_id))
        msg = _can.Message(
            arbitration_id=arb_id,
            data=data[:8],  # classic CAN: max 8 bytes
            is_extended_id=False,
        )
        with self._send_lock:
            bus.send(msg)  # type: ignore[attr-defined]
            self.frames_tx += 1

    def send_request(self, cmd_id: CmdId | int,
                     node_id: Optional[int] = None) -> None:
        """Send a zero-length request frame (RTR-like, data=b'').

        This ODrive firmware branch only responds to GET_* requests with
        DLC=0; sending 8 zero bytes (DLC=8) gets no response. The test
        scripts send empty data for the same reason.
        """
        self.send(cmd_id, b'', node_id=node_id)

    # ------------------------------------------------------------------ #
    # Reader Thread
    # ------------------------------------------------------------------ #

    def _reader_main(self) -> None:
        bus = self._bus
        assert bus is not None
        try:
            while not self._stop_evt.is_set():
                try:
                    msg = bus.recv(timeout=0.1)  # type: ignore[attr-defined]
                except Exception:
                    self.bus_errors += 1
                    continue

                if msg is None:
                    continue

                # Filter: only process frames for our node_id
                if msg.is_extended_id:
                    continue
                nid, cmd_id = parse_frame_id(msg.arbitration_id)
                if nid != self._node_id:
                    continue

                self.frames_rx += 1
                ts = time.monotonic()
                data = bytes(msg.data)
                self._put_nowait(('frame', cmd_id, data, ts))

        except Exception as exc:
            self._put_nowait(('error', str(exc)))

    # ------------------------------------------------------------------ #
    # Cross-thread Queue
    # ------------------------------------------------------------------ #

    def _put_nowait(self, item) -> None:
        try:
            self._loop.call_soon_threadsafe(self._q.put_nowait, item)
        except RuntimeError:
            pass  # event loop closed

    async def pump(self) -> None:
        """Event-loop coroutine: drain the queue and dispatch to callback.

        Runs forever; cancel it when shutting down.
        """
        while True:
            item = await self._q.get()
            kind = item[0]
            try:
                if kind == 'frame':
                    _, cmd_id, data, ts = item
                    if self._on_frame is not None:
                        await self._on_frame(cmd_id, data, ts)
                elif kind == 'error':
                    _, text = item
                    # Surface as a log event via the frame callback
                    # (service layer handles it)
                    if self._on_frame is not None:
                        await self._on_frame(-1, text.encode(), time.monotonic())
            except Exception as exc:
                print(f'[can-transport] dispatch error: {exc!r}', flush=True)
