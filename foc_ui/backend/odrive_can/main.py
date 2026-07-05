"""ODrive CAN FastAPI routes and WebSocket endpoint.

Provides:
  - HTTP routes at /api/can/* for CAN interface management
  - WebSocket at /ws/can for bidirectional CAN control
  - Static table endpoints for axis states, modes, error definitions

Designed as an APIRouter that gets mounted into the existing serial
FastAPI app via ``app.include_router(can_router)``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from contextlib import asynccontextmanager
from typing import Any, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from .protocol import (
    AxisState, ControlMode, InputMode,
    ExtSubCmd, ExtType, ServoControlMode,
    AXIS_ERROR_BITS, MOTOR_ERROR_BITS, ENCODER_ERROR_BITS, CONTROLLER_ERROR_BITS,
    axis_states_as_json, control_modes_as_json, input_modes_as_json,
    error_bits_as_json,
)
from .state import channels_as_json
from .transport import CanTransport
from .service import (
    ODriveService, POLL_HZ_DEFAULT, POLL_HZ_MIN, POLL_HZ_MAX,
)
from .ws_hub import ODriveWsHub
from .recorder import CanRecorder

logger = logging.getLogger('odrive_can.main')

# --------------------------------------------------------------------------- #
# CAN App State
# --------------------------------------------------------------------------- #


class CanAppState:
    """CAN-specific app state container."""

    def __init__(self):
        self.hub = ODriveWsHub()
        self.recorder = CanRecorder()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.transport: Optional[CanTransport] = None
        self.service: Optional[ODriveService] = None
        self._pump_task: Optional[asyncio.Task] = None
        self._status_task: Optional[asyncio.Task] = None


CAN_STATE = CanAppState()


# --------------------------------------------------------------------------- #
# API Router
# --------------------------------------------------------------------------- #

can_router = APIRouter(tags=['odrive-can'])


# --------------------------------------------------------------------------- #
# HTTP Routes
# --------------------------------------------------------------------------- #


class CanConnectRequest(BaseModel):
    interface: str = 'pcan'
    channel: str = 'PCAN_USBBUS1'
    bitrate: int = 1_000_000
    node_id: int = 0


@can_router.get('/api/can/interfaces')
async def list_can_interfaces():
    """Enumerate available CAN interfaces using python-can."""
    try:
        from can.interfaces import VALID_INTERFACES
        interfaces = []
        for iface in sorted(VALID_INTERFACES):
            interfaces.append({
                'interface': iface,
                'desc': f'{iface} CAN interface',
            })
        # Always include 'virtual' for testing
        if 'virtual' not in [i['interface'] for i in interfaces]:
            interfaces.append({
                'interface': 'virtual',
                'desc': 'Virtual CAN bus (testing)',
            })
        return {'interfaces': interfaces}
    except ImportError:
        return {'interfaces': [{'interface': 'virtual',
                                 'desc': 'Virtual CAN bus (python-can not installed)'}]}
    except Exception as exc:
        return JSONResponse({'error': str(exc)}, status_code=500)


@can_router.post('/api/can/connect')
async def can_connect(req: CanConnectRequest):
    """Open a CAN bus connection and start polling."""
    if CAN_STATE.transport is not None and CAN_STATE.transport.is_open:
        return JSONResponse({'error': 'already connected'}, status_code=409)

    # H3 fix: Clean up any stale transport from a previous failed disconnect
    if CAN_STATE.transport is not None:
        try:
            if CAN_STATE._pump_task:
                CAN_STATE._pump_task.cancel()
                CAN_STATE._pump_task = None
            CAN_STATE.transport.close()
        except Exception:
            pass
        CAN_STATE.transport = None
        CAN_STATE.service = None

    try:
        transport = None
        service = None
        loop = asyncio.get_running_loop()
        if CAN_STATE.loop is None:
            CAN_STATE.loop = loop

        transport = CanTransport(loop, node_id=req.node_id)
        service = ODriveService(transport, node_id=req.node_id)

        # Wire callbacks
        service.on_telemetry_update = _on_telemetry
        service.on_heartbeat_update = _on_heartbeat
        service.on_log = _on_log
        transport._on_frame = service.on_frame

        # Open the bus
        transport.open(
            interface=req.interface,
            channel=req.channel,
            bitrate=req.bitrate,
        )

        CAN_STATE.transport = transport
        CAN_STATE.service = service

        # Start pump, polling, and safety monitor
        CAN_STATE._pump_task = asyncio.create_task(transport.pump())
        await service.start_polling(POLL_HZ_DEFAULT)
        await service.start_safety_monitor()

        # Start status heartbeat
        if CAN_STATE._status_task is None:
            CAN_STATE._status_task = asyncio.create_task(_status_heartbeat())

        # Wire WS message handler
        CAN_STATE.hub.set_client_msg_handler(_on_client_msg)

        await CAN_STATE.hub.broadcast_status(
            connected=True,
            interface=req.interface,
            channel=req.channel,
            node_id=req.node_id,
            frames_rx=0, frames_tx=0, bus_errors=0,
            poll_hz=POLL_HZ_DEFAULT,
        )

        logger.info('CAN connected: %s:%s @ %d bps, node_id=%d',
                     req.interface, req.channel, req.bitrate, req.node_id)
        return {'ok': True, 'interface': req.interface,
                'channel': req.channel, 'node_id': req.node_id}

    except Exception as exc:
        logger.exception('CAN connect failed')
        # Clean up any partial state so a retry doesn't hit 409 "already
        # connected" with a leaked bus / pump / poll task.
        if CAN_STATE._pump_task:
            CAN_STATE._pump_task.cancel()
            try:
                await CAN_STATE._pump_task
            except (asyncio.CancelledError, Exception):
                pass
            CAN_STATE._pump_task = None
        if service is not None:
            try:
                await service.shutdown()
            except Exception:
                pass
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        CAN_STATE.transport = None
        CAN_STATE.service = None
        return JSONResponse({'error': str(exc)}, status_code=500)


@can_router.post('/api/can/disconnect')
async def can_disconnect():
    """Close the CAN bus connection."""
    transport = CAN_STATE.transport
    service = CAN_STATE.service
    if transport is None or not transport.is_open:
        return {'ok': True, 'was_connected': False}

    # Stop polling and safety monitor
    if service:
        await service.shutdown()

    # Stop pump
    if CAN_STATE._pump_task:
        CAN_STATE._pump_task.cancel()
        try:
            await CAN_STATE._pump_task
        except asyncio.CancelledError:
            pass
        except Exception:
            logger.exception('pump task shutdown error')
        CAN_STATE._pump_task = None

    # Stop status heartbeat task (M1 fix)
    if CAN_STATE._status_task:
        CAN_STATE._status_task.cancel()
        try:
            await CAN_STATE._status_task
        except asyncio.CancelledError:
            pass
        CAN_STATE._status_task = None

    # Stop recording
    if CAN_STATE.recorder.is_recording:
        CAN_STATE.recorder.stop()

    interface = transport.interface or ''
    channel = transport.channel or ''
    transport.close()
    CAN_STATE.transport = None
    CAN_STATE.service = None

    await CAN_STATE.hub.broadcast_status(
        connected=False,
        interface=interface, channel=channel, node_id=0,
        frames_rx=0, frames_tx=0, bus_errors=0, poll_hz=0,
    )

    logger.info('CAN disconnected: %s:%s', interface, channel)
    return {'ok': True, 'was_connected': True}


@can_router.get('/api/can/status')
async def can_status():
    transport = CAN_STATE.transport
    service = CAN_STATE.service
    return {
        'connected': transport is not None and transport.is_open,
        'interface': transport.interface if transport else None,
        'channel': transport.channel if transport else None,
        'node_id': service.node_id if service else 0,
        'frames_rx': transport.frames_rx if transport else 0,
        'frames_tx': transport.frames_tx if transport else 0,
        'bus_errors': transport.bus_errors if transport else 0,
        'poll_hz': service._poll_hz if service else 0,
        'recording': CAN_STATE.recorder.is_recording,
        'record_path': CAN_STATE.recorder.path,
        'record_rows': CAN_STATE.recorder.rows_written,
    }


@can_router.get('/api/can/channels')
async def can_channels():
    return {'channels': channels_as_json()}


@can_router.get('/api/can/axis-states')
async def can_axis_states():
    return {'states': axis_states_as_json()}


@can_router.get('/api/can/control-modes')
async def can_control_modes():
    return {'modes': control_modes_as_json()}


@can_router.get('/api/can/input-modes')
async def can_input_modes():
    return {'modes': input_modes_as_json()}


@can_router.get('/api/can/axis-errors')
async def can_axis_errors():
    return {'errors': error_bits_as_json(AXIS_ERROR_BITS, 'axis')}


@can_router.get('/api/can/motor-errors')
async def can_motor_errors():
    return {'errors': error_bits_as_json(MOTOR_ERROR_BITS, 'motor')}


@can_router.get('/api/can/encoder-errors')
async def can_encoder_errors():
    return {'errors': error_bits_as_json(ENCODER_ERROR_BITS, 'encoder')}


@can_router.get('/api/can/controller-errors')
async def can_controller_errors():
    return {'errors': error_bits_as_json(CONTROLLER_ERROR_BITS, 'controller')}


# --------------------------------------------------------------------------- #
# WebSocket Endpoint
# --------------------------------------------------------------------------- #


@can_router.websocket('/ws/can')
async def can_ws_endpoint(ws: WebSocket):
    await CAN_STATE.hub.attach(ws)

    # Push immediate status snapshot
    transport = CAN_STATE.transport
    service = CAN_STATE.service
    await CAN_STATE.hub.broadcast_status(
        connected=transport is not None and transport.is_open,
        interface=transport.interface if transport else '',
        channel=transport.channel if transport else '',
        node_id=service.node_id if service else 0,
        frames_rx=transport.frames_rx if transport else 0,
        frames_tx=transport.frames_tx if transport else 0,
        bus_errors=transport.bus_errors if transport else 0,
        poll_hz=service._poll_hz if service else 0,
    )

    # Notify safety monitor
    if service:
        service.notify_ws_connected()

    try:
        await CAN_STATE.hub.receive_loop(ws)
    finally:
        await CAN_STATE.hub.detach(ws)
        # If no more clients, notify safety monitor
        if not CAN_STATE.hub.has_clients():
            if service:
                service.notify_ws_disconnected()


# --------------------------------------------------------------------------- #
# Callbacks (wired to service)
# --------------------------------------------------------------------------- #


async def _on_telemetry(ch: dict, ts: float) -> None:
    """Called by service on each poll cycle."""
    # Record if active
    if CAN_STATE.recorder.is_recording:
        CAN_STATE.recorder.record(ch, ts)
    # Broadcast to browsers (downsampled in hub)
    await CAN_STATE.hub.broadcast_telemetry(ch, ts)


async def _on_heartbeat(hb: dict, ts: float) -> None:
    """Called by service on each heartbeat frame."""
    await CAN_STATE.hub.broadcast_heartbeat(hb, ts)


def _on_log(line: str) -> None:
    """Called by service for log events (sync, schedules broadcast).

    Safe from any thread (command handlers run in the event loop; future
    callers might not). Uses run_coroutine_threadsafe + a done callback so a
    failed broadcast_log surfaces to the log instead of becoming an
    unretrieved-future warning at GC time.
    """
    loop = CAN_STATE.loop
    if not (loop and loop.is_running()):
        return
    fut = asyncio.run_coroutine_threadsafe(
        CAN_STATE.hub.broadcast_log(line), loop)

    def _done(f) -> None:
        try:
            f.result()
        except Exception:
            logger.exception('broadcast_log failed')

    fut.add_done_callback(_done)


# --------------------------------------------------------------------------- #
# WS Message Dispatch
# --------------------------------------------------------------------------- #


async def _on_client_msg(obj: dict, ws: WebSocket) -> None:
    """Handle an inbound WebSocket message from a browser client."""
    kind = obj.get('type')
    svc = CAN_STATE.service

    if svc is None:
        await ws.send_text(json.dumps(
            {'type': 'error', 'msg': 'CAN not connected'}))
        return

    # Reset WS activity watchdog
    svc.notify_ws_activity()

    try:
        if kind == 'ping':
            # Frontend keepalive; notify_ws_activity() above already fed the
            # watchdog. Pong lets the frontend measure RTT / liveness.
            await ws.send_text(json.dumps({'type': 'pong'}))

        elif kind == 'set_state':
            await svc.set_axis_state(obj['state'])

        elif kind == 'set_mode':
            await svc.set_controller_mode(
                obj['control_mode'], obj['input_mode'])

        elif kind == 'set_pos':
            await svc.set_input_pos(
                obj['pos'],
                obj.get('vel_ff', 0),
                obj.get('torque_ff', 0))

        elif kind == 'set_vel':
            await svc.set_input_vel(
                obj['vel'], obj.get('torque_ff', 0))

        elif kind == 'set_torque':
            await svc.set_input_torque(obj['torque'])

        elif kind == 'mit':
            await svc.set_mit_control(
                obj['p_des'], obj['v_des'],
                obj['kp'], obj['kd'], obj['t_ff'])

        elif kind == 'set_gain':
            name = obj.get('name')
            if not name:
                await ws.send_text(json.dumps(
                    {'type': 'error', 'msg': 'missing name in set_gain'}))
            elif name == 'pos_gain':
                await svc.set_pos_gain(float(obj['value']))
            elif name == 'pos_integrator_gain':
                await svc.set_pos_integrator_gain(float(obj['value']))
            elif name == 'vel_gain':
                # Use cached integrator gain if not provided
                ig = float(obj.get('integrator', svc._last_vel_integrator_gain))
                await svc.set_vel_gains(float(obj['value']), ig)
            elif name == 'vel_integrator_gain':
                # Use cached vel_gain if not provided (prevents zeroing)
                vg = float(obj.get('gain', svc._last_vel_gain))
                await svc.set_vel_gains(vg, float(obj['value']))
            else:
                await ws.send_text(json.dumps(
                    {'type': 'error', 'msg': f'unknown gain: {name}'}))

        elif kind == 'set_limits':
            await svc.set_limits(obj['vel_limit'], obj['current_limit'])

        elif kind == 'clear_errors':
            await svc.clear_errors()

        elif kind == 'estop':
            await svc.estop()

        elif kind == 'reboot':
            await svc.reboot()

        elif kind == 'ext_cmd':
            result = await svc.ext_command(
                obj['sub_cmd'], obj['item'],
                obj.get('ext_type', 0), obj.get('value', 0),
                timeout=obj.get('timeout', 1.0))
            # result carries an ext 'type' byte (FLOAT32/INT32/UINT32) that
            # would clobber 'type': 'ext_resp' via **result, so the frontend
            # dispatch (case 'ext_resp') never matched. Rename it.
            ext_type = result.pop('type', 0)
            await ws.send_text(json.dumps(
                {'type': 'ext_resp', 'ext_type': ext_type, **result}))

        elif kind == 'anticogging_start':
            await svc.start_anticogging()

        elif kind == 'get_overspeed_snapshot':
            # Read the fault-instant snapshot captured by firmware on OVERSPEED.
            # Held until clear_errors. ~32 ext round-trips (~100-200ms).
            snap = await svc.read_overspeed_snapshot()
            await ws.send_text(json.dumps(
                {'type': 'overspeed_snapshot', 'snapshot': snap}))

        elif kind == 'get_control_config':
            cfg = await svc.read_control_config()
            await ws.send_text(json.dumps(
                {'type': 'control_config', 'config': cfg}))

        elif kind == 'set_control_config':
            result = await svc.set_control_config(
                obj['item'], obj['value'], obj.get('is_float', False))
            ext_type = result.pop('type', 0)
            await ws.send_text(json.dumps(
                {'type': 'ext_resp', 'ext_type': ext_type, **result}))

        elif kind == 'set_servo_mode':
            result = await svc.set_servo_mode(ServoControlMode(obj['mode']))
            ext_type = result.pop('type', 0)
            await ws.send_text(json.dumps(
                {'type': 'ext_resp', 'ext_type': ext_type, **result}))

        elif kind == 'anticogging_status':
            # Request all status items
            for item in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]:
                result = await svc.ext_command(
                    ExtSubCmd.GET_ANTICOGGING_STATUS, item)
                ext_type = result.pop('type', 0)
                await ws.send_text(json.dumps(
                    {'type': 'ext_resp', 'ext_type': ext_type, **result}))

        elif kind == 'anticogging_config':
            if 'enabled' in obj:
                await svc.ext_command(
                    ExtSubCmd.SET_ANTICOGGING_CONFIG,
                    0x01, ExtType.UINT32,
                    1 if obj['enabled'] else 0)
            if 'pre_calibrated' in obj:
                await svc.ext_command(
                    ExtSubCmd.SET_ANTICOGGING_CONFIG,
                    0x02, ExtType.UINT32,
                    1 if obj['pre_calibrated'] else 0)
            if 'pos_threshold' in obj:
                await svc.ext_command(
                    ExtSubCmd.SET_ANTICOGGING_CONFIG,
                    0x03, ExtType.FLOAT32,
                    obj['pos_threshold'])
            if 'vel_threshold' in obj:
                await svc.ext_command(
                    ExtSubCmd.SET_ANTICOGGING_CONFIG,
                    0x04, ExtType.FLOAT32,
                    obj['vel_threshold'])
            if obj.get('reset'):
                await svc.ext_command(
                    ExtSubCmd.SET_ANTICOGGING_CONFIG,
                    0x05, ExtType.UINT32, 1)

        elif kind == 'set_poll_hz':
            hz = max(POLL_HZ_MIN, min(POLL_HZ_MAX, float(obj['hz'])))
            await svc.stop_polling()
            await svc.start_polling(hz)

        elif kind == 'rec':
            action = obj.get('action')
            if action == 'start':
                path = obj.get('path', 'odrive_capture.csv')
                try:
                    CAN_STATE.recorder.start(path)
                except Exception as exc:
                    await ws.send_text(json.dumps(
                        {'type': 'error', 'msg': str(exc)}))
                    return
                await ws.send_text(json.dumps({
                    'type': 'rec_state', 'recording': True,
                    'path': CAN_STATE.recorder.path}))
            elif action == 'stop':
                p = CAN_STATE.recorder.stop()
                await ws.send_text(json.dumps({
                    'type': 'rec_state', 'recording': False,
                    'path': p}))

        else:
            await ws.send_text(json.dumps(
                {'type': 'error', 'msg': f'unknown type {kind!r}'}))

    except Exception as exc:
        logger.exception('WS command error: %s', kind)
        await ws.send_text(json.dumps(
            {'type': 'error', 'msg': str(exc)}))


# --------------------------------------------------------------------------- #
# Status Heartbeat
# --------------------------------------------------------------------------- #


async def _status_heartbeat() -> None:
    """Periodically push connection stats (2 Hz)."""
    while True:
        try:
            await asyncio.sleep(0.5)
            transport = CAN_STATE.transport
            service = CAN_STATE.service
            connected = transport is not None and transport.is_open
            await CAN_STATE.hub.broadcast_status(
                connected=connected,
                interface=transport.interface if transport else '',
                channel=transport.channel if transport else '',
                node_id=service.node_id if service else 0,
                frames_rx=transport.frames_rx if transport else 0,
                frames_tx=transport.frames_tx if transport else 0,
                bus_errors=transport.bus_errors if transport else 0,
                poll_hz=service._poll_hz if service else 0,
            )
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception('status heartbeat error')
