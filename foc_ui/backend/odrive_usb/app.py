"""FastAPI routes and bounded WebSocket fan-out for USB diagnostics."""

from __future__ import annotations

import asyncio
from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, Field

from .protocol import PROTOCOL_RELEASE, PROTOCOL_VERSION, SCHEMA_VERSION
from .scope_channels_generated import SCOPE_CHANNELS
from .symbolizer import symbolize_crash
from .transport import FakeSerialTransport, UsbDebugTransport


def _load_trusted_firmware_identities() -> set[tuple[bytes, bytes]]:
    """Load identities backed by an existing local firmware artifact."""
    repo = Path(__file__).resolve().parents[3]
    identities: set[tuple[bytes, bytes]] = set()
    build_root = repo / "Firmware" / "build"
    for manifest_path in build_root.rglob("build_manifest.json"):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            build_id = bytes.fromhex(str(manifest["build_id"])[:16])
            manifest_identity = bytes.fromhex(str(manifest["dirty_sha256"])[:32])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if len(build_id) != 8 or len(manifest_identity) != 16:
            continue
        artifact_path = manifest_path.parent / "ODriveFirmware.bin"
        if not artifact_path.is_file():
            artifact_path = manifest_path.parent / "ODriveFirmware.elf"
        try:
            artifact = artifact_path.read_bytes()
        except OSError:
            continue
        if build_id in artifact and manifest_identity in artifact:
            identities.add((build_id, manifest_identity))
    return identities


class UsbConnectRequest(BaseModel):
    port: str | None = None
    fake: bool = False


class ScopeConfigRequest(BaseModel):
    mode: int = Field(default=0, ge=0, le=1)
    trigger_type: int = Field(default=0, ge=0, le=4)
    trigger_channel: int = Field(default=0xFFFF, ge=0, le=0xFFFF)
    trigger_edge: int = Field(default=0, ge=0, le=1)
    trigger_state: int = Field(default=0, ge=0, le=255)
    sample_rate_hz: int = Field(default=1000, ge=1, le=10000)
    decimation: int = Field(default=10, ge=1, le=65535)
    pre_samples: int = Field(default=0, ge=0, le=64)
    post_samples: int = Field(default=0, ge=0, le=128)
    channel_ids: list[int] = Field(min_length=1, max_length=8)
    threshold: float = 0.0


class ScopeArmRequest(BaseModel):
    # 0=stop, 1=arm, 2=manual trigger; all are diagnostic-only actions.
    arm: int = Field(default=1, ge=0, le=2)


class TestSessionBeginRequest(BaseModel):
    motion: bool = False
    hardware_estop_confirmed: bool = False
    lease_ms: int = Field(default=2000, ge=250, le=10000)
    velocity_limit: float = Field(default=0.5, gt=0, le=2.0)
    current_limit: float = Field(default=0.5, gt=0, le=1.0)
    torque_limit: float = Field(default=0.1, gt=0, le=0.2)


class TestSessionCommandRequest(BaseModel):
    command_type: int = Field(ge=1, le=6)
    operation: int = Field(default=0, ge=0, le=3)
    arg0: int = Field(default=0, ge=0, le=0xFFFFFFFF)


class _UsbClient:
    def __init__(self, ws: WebSocket) -> None:
        self.ws = ws
        self.queue: asyncio.Queue[str] = asyncio.Queue(maxsize=64)
        self.sender: asyncio.Task | None = None


class UsbWsHub:
    """Non-blocking fan-out; a slow browser drops old waveform events only."""

    def __init__(self) -> None:
        self.clients: set[_UsbClient] = set()

    async def attach(self, ws: WebSocket) -> _UsbClient:
        await ws.accept()
        client = _UsbClient(ws)
        self.clients.add(client)
        client.sender = asyncio.create_task(self._sender(client))
        return client

    async def detach(self, client: _UsbClient) -> None:
        self.clients.discard(client)
        if client.sender:
            client.sender.cancel()
            try:
                await client.sender
            except asyncio.CancelledError:
                pass

    async def _sender(self, client: _UsbClient) -> None:
        while True:
            text = await client.queue.get()
            await client.ws.send_text(text)

    def publish(self, event: dict[str, Any]) -> None:
        text = json.dumps(event, separators=(",", ":"))
        for client in list(self.clients):
            try:
                client.queue.put_nowait(text)
            except asyncio.QueueFull:
                # Preserve status/fault/command events. Waveform batches are
                # explicitly lossy so the serial reader is never backpressured.
                retained: list[str] = []
                while not client.queue.empty():
                    item = client.queue.get_nowait()
                    if '"type":"scope_data"' not in item:
                        retained.append(item)
                for item in retained[-63:]:
                    try:
                        client.queue.put_nowait(item)
                    except asyncio.QueueFull:
                        break
                try:
                    client.queue.put_nowait(text)
                except asyncio.QueueFull:
                    pass


class UsbAppState:
    def __init__(self) -> None:
        self.transport = UsbDebugTransport(
            trusted_identities=_load_trusted_firmware_identities())
        self.hub = UsbWsHub()
        self.poll_task: asyncio.Task | None = None

    def status(self) -> dict[str, Any]:
        result = asdict(self.transport.status)
        result["parser"] = asdict(self.transport.decoder.stats)
        result["events_buffered"] = len(self.transport.events)
        return result

    async def poll_loop(self) -> None:
        while True:
            try:
                for event in self.transport.poll():
                    if event.get("type") == "crash_report":
                        event["crash"]["symbolization"] = symbolize_crash(
                            event["crash"])
                    self.hub.publish(event)
                await asyncio.sleep(0.005)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.hub.publish({"type": "error", "error": str(exc)})
                await asyncio.sleep(0.05)

    async def start_polling(self) -> None:
        if self.poll_task is None or self.poll_task.done():
            self.poll_task = asyncio.create_task(self.poll_loop())

    async def disconnect(self) -> None:
        if self.poll_task:
            self.poll_task.cancel()
            try:
                await self.poll_task
            except asyncio.CancelledError:
                pass
            self.poll_task = None
        self.transport.disconnect()


USB_STATE = UsbAppState()
usb_router = APIRouter(tags=["odrive-usb"])


@usb_router.get("/api/usb/ports")
async def usb_ports() -> dict[str, Any]:
    return {"ports": UsbDebugTransport.enumerate_ports()}


@usb_router.get("/api/usb/schema")
async def usb_schema() -> dict[str, Any]:
    return {"protocol_release": PROTOCOL_RELEASE, "protocol_version": PROTOCOL_VERSION,
            "schema_version": SCHEMA_VERSION,
            "channels": [asdict(channel) for channel in SCOPE_CHANNELS]}


@usb_router.get("/api/usb/status")
async def usb_status() -> dict[str, Any]:
    return USB_STATE.status()


@usb_router.post("/api/usb/connect")
async def usb_connect(req: UsbConnectRequest):
    if USB_STATE.transport.status.connected:
        return JSONResponse({"error": "already connected"}, status_code=409)
    try:
        # Refresh at connect time so a firmware built after backend startup is
        # accepted without weakening identity validation.
        USB_STATE.transport.trusted_identities = _load_trusted_firmware_identities()
        attempts = 1 if req.fake else 2
        last_error: Exception | None = None
        for attempt in range(attempts):
            try:
                USB_STATE.transport.connect(
                    port=req.port,
                    serial_device=FakeSerialTransport() if req.fake else None)
                last_error = None
                break
            except Exception as exc:
                last_error = exc
                USB_STATE.transport.disconnect()
                if attempt + 1 < attempts:
                    # Windows can expose the CDC COM port slightly before the
                    # post-reset endpoint accepts its first WriteFile.
                    await asyncio.sleep(0.2)
        if last_error is not None:
            raise last_error
        await USB_STATE.start_polling()
        return {"ok": True, "port": req.port, "fake": req.fake}
    except Exception as exc:
        USB_STATE.transport.disconnect()
        return JSONResponse({"error": str(exc)}, status_code=500)


@usb_router.post("/api/usb/disconnect")
async def usb_disconnect() -> dict[str, Any]:
    was_connected = USB_STATE.transport.status.connected
    await USB_STATE.disconnect()
    return {"ok": True, "was_connected": was_connected}


@usb_router.post("/api/usb/session/begin")
async def usb_session_begin(req: TestSessionBeginRequest):
    if req.motion and not req.hardware_estop_confirmed:
        return JSONResponse(
            {"error": "motion session requires independent hardware estop"},
            status_code=409)
    try:
        request_id = USB_STATE.transport.begin_test_session(**req.model_dump())
        return {"ok": True, "request_id": request_id}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/session/command")
async def usb_session_command(req: TestSessionCommandRequest):
    try:
        request_id = USB_STATE.transport.send_command(
            req.command_type, req.operation, req.arg0)
        return {"ok": True, "request_id": request_id}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/session/keepalive")
async def usb_session_keepalive():
    try:
        return {"ok": True,
                "request_id": USB_STATE.transport.keepalive_test_session()}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/session/stop")
async def usb_session_stop():
    try:
        return {"ok": True,
                "request_id": USB_STATE.transport.stop_test_session()}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/session/end")
async def usb_session_end():
    try:
        return {"ok": True,
                "request_id": USB_STATE.transport.end_test_session()}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/scope/config")
async def usb_scope_config(req: ScopeConfigRequest):
    try:
        request_id = USB_STATE.transport.configure_scope(req.model_dump())
        return {"ok": True, "request_id": request_id}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/scope/arm")
async def usb_scope_arm(req: ScopeArmRequest):
    try:
        request_id = USB_STATE.transport.arm_scope(req.arm)
        return {"ok": True, "request_id": request_id, "arm": req.arm}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.post("/api/usb/scope/stop")
async def usb_scope_stop():
    try:
        request_id = USB_STATE.transport.stop_scope()
        return {"ok": True, "request_id": request_id}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.get("/api/usb/scope/status")
async def usb_scope_status():
    try:
        request_id = USB_STATE.transport.request_scope_status()
        return {"ok": True, "request_id": request_id}
    except Exception as exc:
        return JSONResponse({"error": str(exc)}, status_code=409)


@usb_router.get("/api/usb/export/raw")
async def usb_export_raw() -> Response:
    return Response(bytes(USB_STATE.transport.raw_capture),
                    media_type="application/octet-stream",
                    headers={"Content-Disposition": "attachment; filename=odrive_usb_capture.bin"})


@usb_router.get("/api/usb/export/replay")
async def usb_export_replay() -> Response:
    return Response(USB_STATE.transport.export_replay_bytes(),
                    media_type="application/vnd.odrive.usb-replay+json",
                    headers={"Content-Disposition":
                             "attachment; filename=odrive_usb_replay.json"})


@usb_router.websocket("/ws/usb")
async def usb_ws_endpoint(ws: WebSocket):
    client = await USB_STATE.hub.attach(ws)
    USB_STATE.hub.publish({"type": "usb_status", **USB_STATE.status()})
    try:
        while True:
            raw = await ws.receive_text()
            try:
                message = json.loads(raw)
            except json.JSONDecodeError:
                USB_STATE.hub.publish({"type": "error", "error": "bad json"})
                continue
            kind = message.get("type")
            if kind == "scope_config":
                await usb_scope_config(ScopeConfigRequest(**message))
            elif kind == "scope_arm":
                await usb_scope_arm(ScopeArmRequest(arm=int(message.get("arm", 1))))
            elif kind == "scope_stop":
                await usb_scope_stop()
            elif kind == "scope_status":
                await usb_scope_status()
            elif kind == "session_begin":
                await usb_session_begin(TestSessionBeginRequest(**message))
            elif kind == "session_command":
                await usb_session_command(TestSessionCommandRequest(**message))
            elif kind == "session_keepalive":
                await usb_session_keepalive()
            elif kind == "session_stop":
                await usb_session_stop()
            elif kind == "session_end":
                await usb_session_end()
            elif kind == "ping":
                await ws.send_text(json.dumps({"type": "pong"}))
            else:
                USB_STATE.hub.publish({"type": "error", "error": f"unknown type {kind!r}"})
    except WebSocketDisconnect:
        pass
    finally:
        await USB_STATE.hub.detach(client)
