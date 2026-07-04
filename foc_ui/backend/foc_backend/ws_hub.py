"""WebSocket hub: broadcasts parsed events to all connected browser clients.

A single websocket connection per browser tab is the norm. The hub keeps a
weak set of clients and broadcasts JSON messages. Message schema (text frame
JSON, one per message):

  Telemetry:
    {"type":"telemetry","t":<ms_mono>,"seq":<int>,"gap":<int>,
     "ch":{ "<key>":<float|int>, ... 16 keys ... }}

  Log (any non-[CLI] ASCII line):
    {"type":"log","t":<ms_mono>,"tag":"<FOC|BOOT|...>","text":"<line>"}

  CLI response:
    {"type":"cli","t":<ms_mono>,"line":"<text without [CLI] prefix>","done":<bool>}
    (done=true is sent when the response stream goes idle)

  Status (connection lifecycle):
    {"type":"status","t":<ms_mono>,"connected":<bool>,"port":"<COMx>",
     "frames":<int>,"bad_crc":<int>,"dropped":<int>}

Inbound messages from the browser are passed to an ``on_client_msg`` callback
which the main app wires to the CLI responder.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Awaitable, Callable, Optional

from fastapi import WebSocket

from .telemetry import TelemetryFrame

# Browsers struggle to redraw 16 channels at 500 Hz. The hub downsamples the
# telemetry stream by sending at most one frame per SEND_MIN_GAP_S to each
# client. Front-end buffering handles the in-between smoothing. 60 Hz is a
# good balance: smooth on a 60 Hz monitor, light on the WebSocket.
SEND_MIN_GAP_S: float = 1.0 / 60.0


ClientMessageHandler = Callable[[dict, WebSocket], Awaitable[None]]


class WsHub:
    def __init__(self):
        # Each client is a (websocket, last_send_ts) entry.
        self._clients: dict[WebSocket, float] = {}
        self._on_client_msg: Optional[ClientMessageHandler] = None
        self._lock = asyncio.Lock()

    def set_client_msg_handler(self, handler: ClientMessageHandler) -> None:
        self._on_client_msg = handler

    # ------------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------------ #
    async def attach(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._clients[ws] = 0.0

    async def detach(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.pop(ws, None)

    # ------------------------------------------------------------------ #
    # Broadcast helpers (event-loop thread only)
    # ------------------------------------------------------------------ #
    async def broadcast_telemetry(self, frame: TelemetryFrame, ts: float, gap: int) -> None:
        now = time.monotonic()
        # Downsample: each client independently. We still build the payload
        # once (the dict is the expensive part).
        payload = self._build_telemetry_payload(frame, int(ts * 1000), gap)
        text = json.dumps(payload, separators=(',', ':'))
        dead = []
        async with self._lock:
            items = list(self._clients.items())
        for ws, last_ts in items:
            if (now - last_ts) < SEND_MIN_GAP_S:
                continue
            try:
                await ws.send_text(text)
                self._clients[ws] = now
            except Exception:
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.pop(ws, None)

    async def broadcast_log(self, line: str) -> None:
        tag, text = _split_tag(line)
        payload = {
            'type': 'log',
            't': int(time.monotonic() * 1000),
            'tag': tag,
            'text': text,
        }
        await self._broadcast_json(payload)

    async def broadcast_cli(self, line_body: str, done: bool) -> None:
        payload = {
            'type': 'cli',
            't': int(time.monotonic() * 1000),
            'line': line_body,
            'done': done,
        }
        await self._broadcast_json(payload)

    async def broadcast_status(self, connected: bool, port: Optional[str],
                               frames: int, bad_crc: int, dropped: int) -> None:
        payload = {
            'type': 'status',
            't': int(time.monotonic() * 1000),
            'connected': connected,
            'port': port or '',
            'frames': frames,
            'bad_crc': bad_crc,
            'dropped': dropped,
        }
        await self._broadcast_json(payload)

    async def _broadcast_json(self, payload: dict) -> None:
        text = json.dumps(payload, separators=(',', ':'))
        dead = []
        async with self._lock:
            clients = list(self._clients.keys())
        for ws in clients:
            try:
                await ws.send_text(text)
            except Exception:
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.pop(ws, None)

    # ------------------------------------------------------------------ #
    # Inbound (browser -> backend)
    # ------------------------------------------------------------------ #
    async def receive_loop(self, ws: WebSocket) -> None:
        """Read inbound messages from one client until disconnect."""
        try:
            while True:
                msg = await ws.receive_text()
                try:
                    obj = json.loads(msg)
                except json.JSONDecodeError:
                    await ws.send_text(json.dumps({'type': 'error', 'msg': 'bad json'}))
                    continue
                if self._on_client_msg is not None:
                    await self._on_client_msg(obj, ws)
        except Exception:
            # Normal close / network drop -- detach.
            pass

    # ------------------------------------------------------------------ #
    # Internal payload builders
    # ------------------------------------------------------------------ #
    @staticmethod
    def _build_telemetry_payload(frame: TelemetryFrame, t_ms: int, gap: int) -> dict:
        return {
            'type': 'telemetry',
            't': t_ms,
            'seq': frame.seq,
            'gap': gap,
            'ch': frame.values,   # already {key: value} dict
        }


def _split_tag(line: str) -> tuple[str, str]:
    """Split '[FOC] hello' -> ('FOC', 'hello'). Unknown format -> ('?', line)."""
    if line.startswith('['):
        end = line.find(']')
        if end > 0:
            return line[1:end], line[end + 1:].strip()
    return '?', line
