"""FOC upper-computer backend: FastAPI + WebSocket + serial bridge.

Run:
    python -m foc_backend.main
    (or:  uvicorn foc_backend.main:app --host 127.0.0.1 --port 8000)

Endpoints:
    GET  /                  -> serves the built frontend (../frontend/dist) if present,
                               otherwise a small placeholder page.
    GET  /api/ports         -> list available serial ports ([{name, desc}])
    POST /api/connect       -> body {port, baudrate?} -> opens the serial link
    POST /api/disconnect    -> closes the serial link
    GET  /api/status        -> {connected, port, frames, bad_crc, dropped,
                               recording, record_path, record_rows, stream_hz}
    GET  /api/channels      -> static channel table (mirrors firmware)
    GET  /api/errors        -> static 28-bit error-bit table
    WS   /ws                -> bidirectional control + telemetry channel

Browser -> backend WS messages:
    {"type":"cmd",     "text":"start velocity 2.0"}    -> send raw CLI line, await
    {"type":"set",     "key":"ilimit","value":2.5}     -> sugar for 'set <key> <value>'
    {"type":"rec",     "action":"start","path":"..."}  -> start recording
    {"type":"rec",     "action":"stop"}                -> stop recording
    {"type":"rawcli",  "text":"status"}                -> fire-and-forget CLI (no wait)
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager
from typing import Any, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .cli_responder import CliResponder
from .recorder import Recorder
from .serial_link import SerialLink
from .telemetry import channels_as_json, errors_as_json
from .ws_hub import WsHub

# Optional ODrive CAN backend (sibling package)
try:
    from odrive_can.main import can_router as _can_router
    _HAS_CAN = True
except ImportError:
    _HAS_CAN = False

logger = logging.getLogger('foc_backend')

# Default CLI parameter ranges (must match firmware motor_cli.c limits). Used
# only for cheap pre-validation; the firmware is the authoritative enforcer.
SET_RANGES: dict[str, tuple] = {
    'vel_limit': (0.0, 60.0),
    'ilimit':    (0.0, 3.0),
    'ramp':      (0.1, 100.0),
    'vel_kp':    (0.0, 1e6),
    'vel_ki':    (0.0, 1e6),
    'pos_kp':    (0.0, 1000.0),
    'flux':      (0.0, 0.05),
    'watchdog':  (0.0, 60.0),
    'stream_hz': (0, 500),
}

# Keys that take a runtime target value (handled differently from tunables).
SET_LIVE_KEYS = {'velocity', 'position', 'current'}


# --------------------------------------------------------------------------- #
# App state container
# --------------------------------------------------------------------------- #

class AppState:
    def __init__(self):
        self.hub = WsHub()
        self.recorder = Recorder()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.link: Optional[SerialLink] = None
        self.cli: Optional[CliResponder] = None
        self._pump_task: Optional[asyncio.Task] = None
        self._status_task: Optional[asyncio.Task] = None

    # -- serial event callbacks (run on the event loop) ---------------------
    async def _on_telemetry(self, frame, ts, gap):
        # 1) record (if active)
        if self.recorder.is_recording:
            self.recorder.record(frame, ts)
        # 2) broadcast to browsers (downsampled inside the hub)
        await self.hub.broadcast_telemetry(frame, ts, gap)

    async def _on_ascii_line(self, line: str):
        # First offer to the CLI responder (it claims [CLI] lines).
        if self.cli is not None:
            self.cli.on_ascii_line(line)
            # The responder's on_ascii_line returns None for non-CLI lines,
            # but it always consumes [CLI] lines internally. We still want to
            # forward them to the browser log console for visibility, so we
            # broadcast every line as a log event and additionally emit CLI
            # line events for [CLI]-prefixed lines.
        await self.hub.broadcast_log(line)
        # If this is a CLI response line, also forward as a 'cli' event so
        # the console can highlight it. We don't know "done" here; the cli
        # responder drives done-ness separately (best-effort).
        if line.startswith('[CLI] '):
            await self.hub.broadcast_cli(line[len('[CLI] '):], done=False)


STATE = AppState()


# --------------------------------------------------------------------------- #
# Lifespan: capture the running event loop so cross-thread code can reach it.
# --------------------------------------------------------------------------- #

@asynccontextmanager
async def lifespan(app: FastAPI):
    STATE.loop = asyncio.get_running_loop()
    # Status broadcast heartbeat (~2 Hz) -- lets the UI show drop counters.
    STATE._status_task = asyncio.create_task(_status_heartbeat())
    logger.info('FOC backend up at http://127.0.0.1:8000')
    try:
        yield
    finally:
        if STATE._status_task:
            STATE._status_task.cancel()
        if STATE._pump_task:
            STATE._pump_task.cancel()
        if STATE.link is not None:
            STATE.link.close()
        if STATE.recorder.is_recording:
            STATE.recorder.stop()
        logger.info('FOC backend down')


app = FastAPI(title='FOC upper-computer', lifespan=lifespan)

# Mount ODrive CAN routes if the odrive_can package is available
if _HAS_CAN:
    app.include_router(_can_router)


# --------------------------------------------------------------------------- #
# HTTP routes
# --------------------------------------------------------------------------- #

class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 115200


@app.get('/api/ports')
async def list_ports():
    """Enumerate available serial ports (filters to COM* on Windows)."""
    try:
        from serial.tools import list_ports
        ports = []
        for p in list_ports.comports():
            ports.append({'name': p.device, 'desc': p.description})
        return {'ports': ports}
    except Exception as exc:  # noqa: BLE001
        return JSONResponse({'error': str(exc)}, status_code=500)


@app.post('/api/connect')
async def connect(req: ConnectRequest):
    if STATE.link is not None and STATE.link.is_open:
        return JSONResponse({'error': 'already connected'}, status_code=409)
    try:
        link = SerialLink(
            loop=STATE.loop,
            on_telemetry=STATE._on_telemetry,
            on_ascii_line=STATE._on_ascii_line,
        )
        link.open(req.port, baudrate=req.baudrate)
    except Exception as exc:  # noqa: BLE001
        logger.exception('connect failed')
        return JSONResponse({'error': str(exc)}, status_code=500)

    STATE.link = link
    STATE.cli = CliResponder(send_fn=_send_bytes_async)
    STATE._pump_task = asyncio.create_task(link.pump())
    await STATE.hub.broadcast_status(True, link.port_name, 0, 0, 0)
    return {'ok': True, 'port': req.port}


@app.post('/api/disconnect')
async def disconnect():
    link = STATE.link
    if link is None:
        return {'ok': True, 'was_connected': False}
    if STATE._pump_task:
        STATE._pump_task.cancel()
        STATE._pump_task = None
    if STATE.recorder.is_recording:
        STATE.recorder.stop()
    port = link.port_name
    link.close()
    STATE.link = None
    STATE.cli = None
    await STATE.hub.broadcast_status(False, port, 0, 0, 0)
    return {'ok': True, 'was_connected': True}


@app.get('/api/status')
async def get_status():
    link = STATE.link
    return {
        'connected': link is not None and link.is_open,
        'port': link.port_name if link else None,
        'frames': link.frames_total if link else 0,
        'bad_crc': link.frames_bad_crc if link else 0,
        'dropped': link.frames_dropped if link else 0,
        'recording': STATE.recorder.is_recording,
        'record_path': STATE.recorder.path,
        'record_rows': STATE.recorder.rows_written,
    }


@app.get('/api/channels')
async def get_channels():
    return {'channels': channels_as_json()}


@app.get('/api/errors')
async def get_errors():
    return {'errors': errors_as_json()}


# --------------------------------------------------------------------------- #
# WebSocket route
# --------------------------------------------------------------------------- #

@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await STATE.hub.attach(ws)
    # Push an immediate status snapshot so the new client knows the state.
    link = STATE.link
    await STATE.hub.broadcast_status(
        link is not None and link.is_open,
        link.port_name if link else None,
        link.frames_total if link else 0,
        link.frames_bad_crc if link else 0,
        link.frames_dropped if link else 0,
    )
    try:
        await STATE.hub.receive_loop(ws)
    finally:
        await STATE.hub.detach(ws)


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

async def _send_bytes_async(data: bytes) -> None:
    """Adapter so CliResponder (async) can drive the blocking serial write."""
    link = STATE.link
    if link is None or not link.is_open:
        raise RuntimeError('serial link not open')
    # Serial write is fast for short commands; run in default executor to
    # avoid blocking the loop on a slow CDC stack.
    await asyncio.get_running_loop().run_in_executor(None, link.write, data)


async def _status_heartbeat() -> None:
    """Periodically push connection stats so the UI shows live drop counters."""
    while True:
        try:
            await asyncio.sleep(0.5)
            link = STATE.link
            connected = link is not None and link.is_open
            await STATE.hub.broadcast_status(
                connected,
                link.port_name if link else None,
                link.frames_total if link else 0,
                link.frames_bad_crc if link else 0,
                link.frames_dropped if link else 0,
            )
        except asyncio.CancelledError:
            raise
        except Exception:  # noqa: BLE001
            logger.exception('status heartbeat error')


# Wire the hub's inbound handler after STATE exists.
def _on_client_msg(obj: dict, ws: WebSocket) -> Any:
    return _dispatch_client_msg(obj, ws)


async def _dispatch_client_msg(obj: dict, ws: WebSocket) -> None:
    kind = obj.get('type')
    if kind == 'cmd':
        await _handle_cmd(obj.get('text', ''), ws)
    elif kind == 'set':
        await _handle_set(obj.get('key', ''), obj.get('value'), ws)
    elif kind == 'rawcli':
        await _handle_rawcli(obj.get('text', ''), ws)
    elif kind == 'rec':
        await _handle_rec(obj, ws)
    else:
        await ws.send_text(json.dumps({'type': 'error', 'msg': f'unknown type {kind!r}'}))


STATE.hub.set_client_msg_handler(_on_client_msg)


async def _handle_cmd(text: str, ws: WebSocket) -> None:
    cli = STATE.cli
    if cli is None:
        await ws.send_text(json.dumps({'type': 'error', 'msg': 'not connected'}))
        return
    try:
        result = await cli.send(text)
    except Exception as exc:  # noqa: BLE001
        await ws.send_text(json.dumps({'type': 'error', 'msg': str(exc)}))
        return
    # The CLI responder already fed each [CLI] line to broadcast_cli via the
    # ASCII callback. Here we send a final "done" marker so the console knows
    # the response is complete.
    await ws.send_text(json.dumps({
        'type': 'cmd_done',
        'is_error': result.is_error,
        'lines': result.lines,
        'text': result.text,
    }))
    # Also push a synthetic 'cli' done event to all clients.
    await STATE.hub.broadcast_cli('', done=True)


async def _handle_set(key: str, value, ws: WebSocket) -> None:
    if not key:
        await ws.send_text(json.dumps({'type': 'error', 'msg': 'missing key'}))
        return
    # Cheap range check for tunables.
    if key in SET_RANGES:
        lo, hi = SET_RANGES[key]
        try:
            v = float(value)
        except (TypeError, ValueError):
            await ws.send_text(json.dumps({'type': 'error', 'msg': f'bad value for {key}'}))
            return
        if v < lo or v > hi:
            await ws.send_text(json.dumps({
                'type': 'error',
                'msg': f'{key}={v} out of range [{lo}, {hi}]',
            }))
            return
        cmd = f'set {key} {v:g}'
    elif key == 'current':
        # current takes id and iq.
        try:
            idv, iq = float(value[0]), float(value[1])
        except Exception:
            await ws.send_text(json.dumps({'type': 'error', 'msg': "set current needs [id, iq]"}))
            return
        cmd = f'set current {idv:g} {iq:g}'
    elif key in SET_LIVE_KEYS:
        try:
            v = float(value)
        except (TypeError, ValueError):
            await ws.send_text(json.dumps({'type': 'error', 'msg': f'bad value for {key}'}))
            return
        cmd = f'set {key} {v:g}'
    else:
        await ws.send_text(json.dumps({'type': 'error', 'msg': f'unknown set key {key!r}'}))
        return
    await _handle_cmd(cmd, ws)


async def _handle_rawcli(text: str, ws: WebSocket) -> None:
    """Fire-and-forget CLI send -- no response collection."""
    cli = STATE.cli
    if cli is None:
        await ws.send_text(json.dumps({'type': 'error', 'msg': 'not connected'}))
        return
    try:
        await cli.send_nowait(text)
    except Exception as exc:  # noqa: BLE001
        await ws.send_text(json.dumps({'type': 'error', 'msg': str(exc)}))


async def _handle_rec(obj: dict, ws: WebSocket) -> None:
    action = obj.get('action')
    if action == 'start':
        path = obj.get('path') or 'foc_capture.csv'
        try:
            STATE.recorder.start(path)
        except Exception as exc:  # noqa: BLE001
            await ws.send_text(json.dumps({'type': 'error', 'msg': str(exc)}))
            return
        await ws.send_text(json.dumps({
            'type': 'rec_state', 'recording': True, 'path': STATE.recorder.path,
        }))
    elif action == 'stop':
        p = STATE.recorder.stop()
        await ws.send_text(json.dumps({
            'type': 'rec_state', 'recording': False, 'path': p,
        }))
    else:
        await ws.send_text(json.dumps({'type': 'error', 'msg': 'unknown rec action'}))


# --------------------------------------------------------------------------- #
# Frontend static hosting (production). Dev mode uses Vite's proxy.
# --------------------------------------------------------------------------- #

_FRONTEND_DIST = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', 'frontend', 'dist'))


@app.get('/', response_class=HTMLResponse)
async def index():
    idx = os.path.join(_FRONTEND_DIST, 'index.html')
    if os.path.exists(idx):
        with open(idx, 'r', encoding='utf-8') as f:
            return HTMLResponse(f.read())
    # Placeholder shown until the frontend is built.
    return HTMLResponse(_PLACEHOLDER_HTML, status_code=200)


if os.path.isdir(_FRONTEND_DIST):
    # Mount static asset directories if they exist (built frontend).
    for sub in ('assets', 'static'):
        p = os.path.join(_FRONTEND_DIST, sub)
        if os.path.isdir(p):
            app.mount(f'/{sub}', StaticFiles(directory=p), name=f'frontend-{sub}')


_PLACEHOLDER_HTML = """<!doctype html>
<html><head><meta charset='utf-8'><title>FOC backend</title>
<style>body{font:14px/1.5 system-ui;margin:2rem;color:#222}
code{background:#eee;padding:1px 4px;border-radius:3px}</style></head>
<body>
<h2>FOC upper-computer backend</h2>
<p>The backend is running. The frontend is not built yet.</p>
<p>To run the dev frontend (hot reload):</p>
<pre><code>cd frontend
npm install
npm run dev     # then open the URL Vite prints</code></pre>
<p>Or build and serve from here:</p>
<pre><code>cd frontend
npm install
npm run build   # outputs to frontend/dist
# then reload this page</code></pre>
<p>API: <a href="/api/ports">/api/ports</a> |
<a href="/api/channels">/api/channels</a> |
<a href="/api/status">/api/status</a> |
<a href="/docs">/docs</a> (Swagger UI)</p>
</body></html>
"""


# --------------------------------------------------------------------------- #
# Entry point for `python -m foc_backend.main`
# --------------------------------------------------------------------------- #

def main():
    import uvicorn
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(name)s %(levelname)s %(message)s',
    )
    uvicorn.run(app, host='127.0.0.1', port=8000, log_level='info')


if __name__ == '__main__':
    main()
