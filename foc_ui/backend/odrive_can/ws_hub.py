"""CAN-aware WebSocket hub for ODrive telemetry broadcast.

Mirrors the serial ``foc_backend.ws_hub.WsHub`` pattern:
  - Singleton hub with per-client downsample (60 Hz)
  - Broadcast methods for telemetry, heartbeat, status, log, ext_resp
  - Dead client cleanup on send failure

Message schema (backend -> browser):
  telemetry:  {"type":"telemetry","t":<ms>,"ch":{...14 keys...}}
  heartbeat:  {"type":"heartbeat","t":<ms>,"axis_error":<u32>,...}
  status:     {"type":"status","t":<ms>,"connected":<bool>,...}
  log:        {"type":"log","t":<ms>,"tag":"CAN|SAFETY|...","text":"..."}
  ext_resp:   {"type":"ext_resp","sub_cmd":<int>,"item":<int>,...}
  rec_state:  {"type":"rec_state","recording":<bool>,"path":<str|null>}
  error:      {"type":"error","msg":<str>}
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Any, Awaitable, Callable, Optional

from fastapi import WebSocket

# Browsers struggle to render 60+ updates/s. Downsample to 60 Hz per client.
SEND_MIN_GAP_S: float = 1.0 / 60.0

ClientMessageHandler = Callable[[dict, WebSocket], Awaitable[None]]


class ODriveWsHub:
    """WebSocket broadcast hub for ODrive CAN telemetry."""

    def __init__(self):
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

    def has_clients(self) -> bool:
        return len(self._clients) > 0

    @property
    def client_count(self) -> int:
        return len(self._clients)

    # ------------------------------------------------------------------ #
    # Broadcast methods
    # ------------------------------------------------------------------ #

    async def broadcast_telemetry(self, ch: dict, ts: float) -> None:
        """Broadcast synthesized telemetry (downsampled to 60Hz per client)."""
        now = time.monotonic()
        payload = {
            'type': 'telemetry',
            't': int(ts * 1000),
            'ch': ch,
        }
        text = json.dumps(payload, separators=(',', ':'))
        dead = []
        async with self._lock:
            items = list(self._clients.items())
        for ws, last_ts in items:
            if (now - last_ts) < SEND_MIN_GAP_S:
                continue
            try:
                await ws.send_text(text)
                # Update timestamp under the lock and only if the client is
                # still attached. Otherwise a client that detached during the
                # await (its finally ran detach()) would be re-added here,
                # leaving a dead WebSocket in _clients that keeps
                # has_clients() true and delays the safety watchdog.
                async with self._lock:
                    if ws in self._clients:
                        self._clients[ws] = now
            except Exception:
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.pop(ws, None)

    async def broadcast_heartbeat(self, hb: dict, ts: float) -> None:
        """Broadcast heartbeat (push-based, not downsampled)."""
        payload = {
            'type': 'heartbeat',
            't': int(ts * 1000),
            **hb,
        }
        await self._broadcast_json(payload)

    async def broadcast_status(self, connected: bool, interface: str,
                               channel: str, node_id: int,
                               frames_rx: int, frames_tx: int,
                               bus_errors: int, poll_hz: float) -> None:
        """Broadcast connection status (2Hz heartbeat from main.py)."""
        payload = {
            'type': 'status',
            't': int(time.monotonic() * 1000),
            'connected': connected,
            'interface': interface,
            'channel': channel,
            'node_id': node_id,
            'frames_rx': frames_rx,
            'frames_tx': frames_tx,
            'bus_errors': bus_errors,
            'poll_hz': poll_hz,
        }
        await self._broadcast_json(payload)

    async def broadcast_log(self, line: str) -> None:
        """Broadcast a log line, splitting [TAG] text into tag + text."""
        tag, text = _split_tag(line)
        payload = {
            'type': 'log',
            't': int(time.monotonic() * 1000),
            'tag': tag,
            'text': text,
        }
        await self._broadcast_json(payload)

    async def broadcast_ext_resp(self, resp: dict) -> None:
        """Broadcast an extended command response."""
        payload = {
            'type': 'ext_resp',
            't': int(time.monotonic() * 1000),
            **resp,
        }
        await self._broadcast_json(payload)

    async def broadcast_error(self, msg: str) -> None:
        """Broadcast an error message."""
        await self._broadcast_json({
            'type': 'error',
            'msg': msg,
        })

    async def _broadcast_json(self, payload: dict) -> None:
        """Send a JSON payload to all clients, removing dead ones."""
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
                    await ws.send_text(json.dumps(
                        {'type': 'error', 'msg': 'bad json'}))
                    continue
                if self._on_client_msg is not None:
                    await self._on_client_msg(obj, ws)
        except Exception:
            pass  # Normal close / network drop


def _split_tag(line: str) -> tuple[str, str]:
    """Split '[CAN] hello' -> ('CAN', 'hello'). Unknown -> ('?', line)."""
    if line.startswith('['):
        end = line.find(']')
        if end > 0:
            return line[1:end], line[end + 1:].strip()
    return '?', line
