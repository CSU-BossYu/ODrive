"""Bounded USB serial transport and hardware-free fake serial implementation."""

from __future__ import annotations

from collections import deque
import csv
from dataclasses import dataclass
import importlib
import os
import time
from typing import Any, Optional

from .protocol import (
    Frame, MessageType, ProtocolError, StreamDecoder, decode_crash_report,
    decode_scope_batch, decode_scope_status, decode_test_session_status,
    encode_frame, hello_frame, scope_arm_frame, scope_config_frame,
    scope_stop_frame, command_frame, session_begin_frame,
    session_id_frame,
    PROTOCOL_RELEASE, PROTOCOL_VERSION, SCHEMA_VERSION,
    BATCH_FLAG_DROPPED, FRAME_ERROR,
)

MAX_EVENTS = 256
MAX_RAW_CAPTURE_BYTES = 4 * 1024 * 1024
HANDSHAKE_TIMEOUT_S = 2.0
TEST_SESSION_INACTIVE = 0
TEST_SESSION_ACTIVE = 1
TEST_SESSION_STOPPING = 2


class FakeSerialTransport:
    """In-memory serial endpoint; device bytes can be fragmented arbitrarily."""

    def __init__(self) -> None:
        self.is_open = True
        self._device_to_host = bytearray()
        self.host_writes: deque[bytes] = deque(maxlen=256)

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("fake serial is closed")
        self.host_writes.append(bytes(data))
        return len(data)

    def read(self, size: int = 4096) -> bytes:
        if not self.is_open:
            return b""
        result = bytes(self._device_to_host[:size])
        del self._device_to_host[:size]
        return result

    def reset_input_buffer(self) -> None:
        self._device_to_host.clear()

    def inject_device_bytes(self, data: bytes, fragment_sizes: Optional[list[int]] = None) -> None:
        if fragment_sizes:
            offset = 0
            for size in fragment_sizes:
                self._device_to_host.extend(data[offset:offset + size])
                offset += size
            self._device_to_host.extend(data[offset:])
        else:
            self._device_to_host.extend(data)

    def take_host_bytes(self) -> bytes:
        data = b"".join(self.host_writes)
        self.host_writes.clear()
        return data

    def close(self) -> None:
        self.is_open = False


@dataclass
class UsbStatus:
    connected: bool = False
    handshake_complete: bool = False
    handshake_error: str | None = None
    port: str | None = None
    protocol_release: str | None = None
    protocol_version: int | None = None
    schema_version: int | None = None
    build_id: str | None = None
    manifest_identity: str | None = None
    capabilities: dict[str, Any] | None = None
    parser_errors: int = 0
    sequence_gaps: int = 0
    timestamp_wraps: int = 0
    dropped_samples: int = 0
    test_session: dict[str, Any] | None = None


class UsbDebugTransport:
    """Owns only USB bytes and diagnostic records; it never calls CAN APIs."""

    def __init__(self, serial: Any | None = None, *, expected_build_id: bytes | None = None,
                 expected_manifest_identity: bytes | None = None,
                 trusted_identities: set[tuple[bytes, bytes]] | None = None) -> None:
        self.serial = serial
        self.status = UsbStatus()
        self.decoder = StreamDecoder()
        self.events: deque[dict[str, Any]] = deque(maxlen=MAX_EVENTS)
        self.raw_capture = bytearray()
        self._sequence = 0
        self._request_id = 0
        self._last_scope_sequence: int | None = None
        self._last_scope_timestamp: int | None = None
        self._scope_decimation = 1
        self._pending_scope_configs: dict[int, dict[str, Any]] = {}
        self._pending_session_begin: dict[int, int] = {}
        self._pending_session_stop: set[int] = set()
        self._next_session_keepalive_at: float | None = None
        self._protocol_errors = 0
        self._handshake_started_at: float | None = None
        self.expected_build_id = expected_build_id
        self.expected_manifest_identity = expected_manifest_identity
        self.trusted_identities = trusted_identities

    @staticmethod
    def enumerate_ports() -> list[dict[str, str]]:
        try:
            list_ports = importlib.import_module("serial.tools.list_ports")
            return [{"device": item.device, "description": item.description or "USB serial"}
                    for item in list_ports.comports()]
        except (ImportError, AttributeError):
            return []

    def connect(self, port: str | None = None, serial_device: Any | None = None) -> None:
        if self.status.connected:
            raise RuntimeError("USB already connected")
        if serial_device is not None:
            self.serial = serial_device
        elif self.serial is None:
            try:
                serial_module = importlib.import_module("serial")
                self.serial = serial_module.Serial(port=port, baudrate=115200,
                                                   timeout=0)
            except ImportError as exc:
                raise RuntimeError("pyserial is not installed") from exc
        self.status = UsbStatus(connected=True, port=port)
        self.decoder.reset()
        self.events.clear()
        self._sequence = 0
        self._request_id = 0
        self._last_scope_sequence = None
        self._last_scope_timestamp = None
        self._scope_decimation = 1
        self._pending_scope_configs.clear()
        self._pending_session_begin.clear()
        self._pending_session_stop.clear()
        self._next_session_keepalive_at = None
        self._protocol_errors = 0
        # The device stream is binary from USB configuration onward. Discard
        # bytes retained by the OS from an earlier COM handle, then establish a
        # fresh protocol session with HELLO. DTR is deliberately irrelevant.
        reset_input = getattr(self.serial, "reset_input_buffer", None)
        if callable(reset_input):
            reset_input()
        self._handshake_started_at = time.monotonic()
        self._send(hello_frame(self._next_sequence(), self._next_request()))

    def disconnect(self) -> None:
        # Disconnect is a transport operation only. It deliberately does not
        # send DISARM, CLEAR_FAULTS or any other device-state command.
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.status = UsbStatus(connected=False)
        self.decoder.reset()
        self.events.clear()
        self._pending_scope_configs.clear()
        self._pending_session_begin.clear()
        self._pending_session_stop.clear()
        self._next_session_keepalive_at = None
        self._handshake_started_at = None

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        return self._sequence or 1

    def _next_request(self) -> int:
        self._request_id = (self._request_id + 1) & 0xFFFFFFFF
        return self._request_id or 1

    def _send(self, frame: Frame) -> int:
        if self.serial is None or not getattr(self.serial, "is_open", True):
            raise RuntimeError("USB is not connected")
        data = encode_frame(frame)
        try:
            written = self.serial.write(data)
        except Exception:
            self.status.connected = False
            self.status.handshake_complete = False
            raise
        if written != len(data):
            self.status.connected = False
            self.status.handshake_complete = False
            raise OSError(f"short USB serial write: {written}/{len(data)}")
        return frame.request_id

    def poll(self, max_bytes: int = 4096) -> list[dict[str, Any]]:
        if self.serial is None or not getattr(self.serial, "is_open", True):
            return []
        try:
            incoming = self.serial.read(max_bytes)
        except Exception:
            self.status.connected = False
            self.status.handshake_complete = False
            raise
        if not incoming:
            self._maintain_test_session()
            return self._check_handshake_timeout()
        self.raw_capture.extend(incoming)
        if len(self.raw_capture) > MAX_RAW_CAPTURE_BYTES:
            del self.raw_capture[:len(self.raw_capture) - MAX_RAW_CAPTURE_BYTES]
        results: list[dict[str, Any]] = []
        for frame in self.decoder.feed(incoming):
            try:
                event = self._handle_frame(frame)
            except ProtocolError as exc:
                self._protocol_errors += 1
                event = {"type": "error", "error": str(exc)}
            if event is not None:
                results.append(event)
                self.events.append(event)
        self.status.parser_errors = (self.decoder.stats.crc_errors +
                                     self.decoder.stats.length_errors +
                                     self.decoder.stats.unknown_frames +
                                     self._protocol_errors)
        results.extend(self._check_handshake_timeout())
        self._maintain_test_session()
        return results

    def _check_handshake_timeout(self) -> list[dict[str, Any]]:
        if (self.status.handshake_complete or
                self.status.handshake_error is not None or
                self._handshake_started_at is None or
                time.monotonic() - self._handshake_started_at < HANDSHAKE_TIMEOUT_S):
            return []
        message = (
            "timed out waiting for USB CAPABILITIES; device did not complete "
            "the binary HELLO handshake")
        self.status.handshake_error = message
        event = {"type": "error", "error": message, "stage": "handshake"}
        self.events.append(event)
        return [event]

    def _handle_frame(self, frame: Frame) -> dict[str, Any] | None:
        if self.status.handshake_complete:
            if self.status.build_id is not None and frame.build_id.hex() != self.status.build_id:
                raise ProtocolError("frame build identity changed")
            if self.status.manifest_identity is not None and frame.manifest_identity.hex() != self.status.manifest_identity:
                raise ProtocolError("frame manifest identity changed")
        if frame.message_type == MessageType.CAPABILITIES:
            if len(frame.payload) != 20:
                raise ProtocolError("unknown CAPABILITIES schema")
            if self.expected_build_id is not None and frame.build_id != self.expected_build_id:
                raise ProtocolError("unknown build identity; refusing decode")
            if self.expected_manifest_identity is not None and frame.manifest_identity != self.expected_manifest_identity:
                raise ProtocolError("unknown manifest identity; refusing decode")
            if (self.trusted_identities is not None and
                    (frame.build_id, frame.manifest_identity) not in self.trusted_identities):
                raise ProtocolError("unknown build/manifest identity; refusing decode")
            proto, schema, max_payload, bits, rx, critical, scope, max_channels, reserved, pre, post = struct_unpack_capabilities(frame.payload)
            if proto != PROTOCOL_VERSION or schema != SCHEMA_VERSION or reserved != 0:
                raise ProtocolError("unknown protocol/schema capability")
            self.status.handshake_complete = True
            self.status.handshake_error = None
            self._handshake_started_at = None
            self.status.protocol_release = PROTOCOL_RELEASE
            self.status.protocol_version = proto
            self.status.schema_version = schema
            self.status.build_id = frame.build_id.hex()
            self.status.manifest_identity = frame.manifest_identity.hex()
            self.status.capabilities = {"max_payload": max_payload, "capability_bits": bits,
                                        "rx_byte_capacity": rx, "critical_tx_capacity": critical,
                                        "scope_tx_capacity": scope, "max_scope_channels": max_channels,
                                        "max_pre_samples": pre, "max_post_samples": post}
            return {"type": "capabilities", **self.status.capabilities,
                    "build_id": self.status.build_id,
                    "manifest_identity": self.status.manifest_identity}
        if not self.status.handshake_complete:
            raise ProtocolError("binary session not established")
        if frame.message_type == MessageType.SCOPE_STATUS:
            event = {"type": "scope_status", **decode_scope_status(frame.payload),
                     "request_id": frame.request_id,
                     "ok": (frame.flags & FRAME_ERROR) == 0}
            pending_config = self._pending_scope_configs.pop(frame.request_id, None)
            if pending_config is not None:
                if event["ok"]:
                    self._scope_decimation = max(
                        1, int(pending_config.get("decimation", 1)))
                    event["applied_config"] = pending_config
                else:
                    event["rejected_config"] = pending_config
            return event
        if frame.message_type == MessageType.TEST_SESSION_STATUS:
            event = {"type": "test_session_status",
                     **decode_test_session_status(frame.payload),
                     "request_id": frame.request_id,
                     "ok": (frame.flags & FRAME_ERROR) == 0}
            self.status.test_session = dict(event)
            lease_ms = self._pending_session_begin.pop(frame.request_id, None)
            if event["ok"] and event["state"] == TEST_SESSION_ACTIVE:
                interval_ms = lease_ms or event["lease_remaining_ms"]
                self._next_session_keepalive_at = (
                    time.monotonic() + max(0.1, interval_ms / 3000.0))
            elif event["state"] == TEST_SESSION_INACTIVE:
                self._next_session_keepalive_at = None
            return event
        if frame.message_type == MessageType.SCOPE_DATA_BATCH:
            event = {"type": "scope_data", **decode_scope_batch(frame.payload)}
            self._annotate_scope_integrity(event)
            return event
        if frame.message_type == MessageType.CALIBRATION_DATA:
            # Calibration records are retained byte-for-byte in raw_capture for
            # export/replay. Validate their small envelope here, but do not push
            # every PWM-rate record through the JSON/WebSocket diagnostics UI.
            # Besides unnecessary UI load, a raw bytes value is not JSON-safe.
            if len(frame.payload) < 8:
                raise ProtocolError("invalid CALIBRATION_DATA prefix")
            import struct
            record_type, payload_size, sequence = struct.unpack(
                "<HHI", frame.payload[:8])
            if record_type not in (1, 2) or payload_size != len(frame.payload) - 8:
                raise ProtocolError("invalid CALIBRATION_DATA envelope")
            del sequence
            return None
        if frame.message_type == MessageType.COMMAND_RESULT:
            if len(frame.payload) != 8:
                raise ProtocolError("invalid COMMAND_RESULT length")
            status, reserved, epoch, reason = struct_unpack_command_result(frame.payload)
            if reserved != 0:
                raise ProtocolError("non-zero COMMAND_RESULT reserved field")
            event = {"type": "command_result", "request_id": frame.request_id,
                     "status": status, "state_epoch": epoch, "reason": reason}
            if frame.request_id in self._pending_session_stop and status != 0:
                self._pending_session_stop.discard(frame.request_id)
                if self.status.test_session is not None:
                    self.status.test_session["state"] = TEST_SESSION_ACTIVE
                    self.status.test_session["stop_confirmed"] = status == 2
                    event["test_session"] = dict(self.status.test_session)
            return event
        if frame.message_type == MessageType.STATE_EVENT:
            if len(frame.payload) != 22:
                raise ProtocolError("invalid STATE_EVENT length")
            import struct
            sequence, epoch, timestamp, request_id, state, operation, axis_state, event_type, reason = struct.unpack(
                "<IIIIBBBBH", frame.payload)
            return {"type": "state_event", "request_id": request_id,
                    "event": {"sequence": sequence, "state_epoch": epoch,
                               "timestamp_cycles": timestamp,
                               "request_id": request_id, "state": state,
                               "operation": operation, "axis_state": axis_state,
                               "event_type": event_type, "reason": reason}}
        if frame.message_type == MessageType.FAULT_EVENT:
            if len(frame.payload) != 44:
                raise ProtocolError("invalid FAULT_EVENT length")
            import struct
            sequence, control_sequence, epoch, timestamp, fault_sequence, source, code, site, severity, reserved, parent, arg0, arg1, arg2 = struct.unpack(
                "<IIIIIHHHBBIIII", frame.payload)
            if reserved != 0:
                raise ProtocolError("non-zero FAULT_EVENT reserved field")
            return {"type": "fault_event", "request_id": frame.request_id,
                    "event": {"sequence": sequence,
                               "control_sequence": control_sequence,
                               "state_epoch": epoch,
                               "timestamp_cycles": timestamp,
                               "fault_sequence": fault_sequence,
                               "source": source, "code": code, "site": site,
                               "severity": severity,
                               "parent_fault_sequence": parent,
                               "arg0": arg0, "arg1": arg1, "arg2": arg2}}
        if frame.message_type == MessageType.CRASH_REPORT:
            crash = decode_crash_report(frame.payload)
            request_id = self._next_request()
            self._send(Frame(
                MessageType.CRASH_ACK,
                payload=__import__("struct").pack("<I", crash["record_crc"]),
                sequence=self._next_sequence(), request_id=request_id))
            return {"type": "crash_report", "request_id": request_id,
                    "crash": crash}
        if frame.message_type == MessageType.CRASH_ACK:
            if len(frame.payload) != 4:
                raise ProtocolError("invalid CRASH_ACK length")
            record_crc, = __import__("struct").unpack("<I", frame.payload)
            return {"type": "crash_ack", "request_id": frame.request_id,
                    "record_crc": record_crc,
                    "ok": (frame.flags & FRAME_ERROR) == 0}
        # Keep fallback events JSON-safe. Raw bytes remain available in the
        # bounded raw capture/export path; the diagnostics WebSocket receives a
        # compact hexadecimal representation instead of a Python bytes object.
        return {"type": frame.message_type.name.lower(),
                "payload_hex": frame.payload.hex(),
                "payload_size": len(frame.payload),
                "request_id": frame.request_id}

    def _annotate_scope_integrity(self, event: dict[str, Any]) -> None:
        previous = self._last_scope_sequence
        samples = event["samples"]
        if previous is not None and samples:
            expected = max(1, self._scope_decimation)
            if ((samples[0]["control_sequence"] - previous) & 0xFFFFFFFF) != expected:
                self.status.sequence_gaps += 1
                event["sequence_gap"] = True
        previous_timestamp = self._last_scope_timestamp
        if previous_timestamp is not None and samples:
            timestamp = samples[0]["timestamp_cycles"]
            if timestamp < previous_timestamp and previous_timestamp - timestamp > 0x80000000:
                self.status.timestamp_wraps += 1
                event["timestamp_wrap"] = True
        if event["flags"] & BATCH_FLAG_DROPPED:
            self.status.dropped_samples += event["dropped_samples"]
        if samples:
            self._last_scope_sequence = samples[-1]["control_sequence"]
            self._last_scope_timestamp = samples[-1]["timestamp_cycles"]

    def configure_scope(self, config: dict[str, Any]) -> int:
        request_id = self._next_request()
        result = self._send(scope_config_frame(
            config, self._next_sequence(), request_id))
        self._pending_scope_configs[request_id] = dict(config)
        return result

    def arm_scope(self, arm: bool = True) -> int:
        return self._send(scope_arm_frame(arm, self._next_sequence(), self._next_request()))

    def stop_scope(self) -> int:
        return self._send(scope_stop_frame(self._next_sequence(), self._next_request()))

    def request_scope_status(self) -> int:
        return self._send(Frame(MessageType.SCOPE_STATUS, sequence=self._next_sequence(),
                                request_id=self._next_request()))

    def begin_test_session(self, *, motion: bool,
                           hardware_estop_confirmed: bool,
                           lease_ms: int = 2000,
                           velocity_limit: float = 0.5,
                           current_limit: float = 0.5,
                           torque_limit: float = 0.1) -> int:
        if not self.status.handshake_complete:
            raise RuntimeError("USB HELLO handshake is not complete")
        request_id = self._next_request()
        frame = session_begin_frame(
            motion=motion,
            hardware_estop_confirmed=hardware_estop_confirmed,
            lease_ms=lease_ms, velocity_limit=velocity_limit,
            current_limit=current_limit, torque_limit=torque_limit,
            sequence=self._next_sequence(), request_id=request_id)
        result = self._send(frame)
        self._pending_session_begin[request_id] = lease_ms
        return result

    def keepalive_test_session(self) -> int:
        session_id = self._active_test_session_id()
        return self._send(session_id_frame(
            MessageType.TEST_SESSION_KEEPALIVE, session_id,
            self._next_sequence(), self._next_request()))

    def stop_test_session(self) -> int:
        session_id = self._active_test_session_id()
        request_id = self._next_request()
        result = self._send(session_id_frame(
            MessageType.TEST_SESSION_STOP, session_id,
            self._next_sequence(), request_id))
        self._pending_session_stop.add(request_id)
        self._next_session_keepalive_at = None
        return result

    def end_test_session(self) -> int:
        session_id = self._active_test_session_id()
        return self._send(session_id_frame(
            MessageType.TEST_SESSION_END, session_id,
            self._next_sequence(), self._next_request()))

    def send_command(self, command_type: int, operation: int = 0,
                     arg0: int = 0) -> int:
        self._active_test_session_id()
        return self._send(command_frame(
            command_type, operation, arg0, self._next_sequence(),
            self._next_request()))

    def _active_test_session_id(self) -> int:
        session = self.status.test_session
        if not session or session.get("state") != TEST_SESSION_ACTIVE:
            raise RuntimeError("USB test session is not active")
        return int(session["session_id"])

    def _maintain_test_session(self) -> None:
        if (self._next_session_keepalive_at is None or
                time.monotonic() < self._next_session_keepalive_at):
            return
        try:
            self.keepalive_test_session()
            session = self.status.test_session or {}
            remaining = max(300, int(session.get("lease_remaining_ms", 1000)))
            self._next_session_keepalive_at = (
                time.monotonic() + max(0.1, remaining / 3000.0))
        except Exception:
            self._next_session_keepalive_at = None

    def drain_events(self, limit: int = 64) -> list[dict[str, Any]]:
        result = []
        for _ in range(max(0, limit)):
            if not self.events:
                break
            result.append(self.events.popleft())
        return result

    def export_raw(self, path: str) -> str:
        with open(path, "wb") as output:
            output.write(self.raw_capture)
        return os.path.abspath(path)

    def export_replay_bytes(self) -> bytes:
        from .replay import encode_bundle, make_bundle
        bundle = make_bundle(status=self.status,
                             raw_capture=bytes(self.raw_capture),
                             events=list(self.events))
        return encode_bundle(bundle)

    def export_csv(self, path: str, batches: list[dict[str, Any]]) -> str:
        rows = []
        channel_ids: list[int] = []
        for batch in batches:
            channel_ids = batch.get("channel_ids", channel_ids)
            for sample in batch.get("samples", []):
                rows.append(sample)
        columns = ["control_sequence", "timestamp_cycles"] + [str(item) for item in channel_ids]
        with open(path, "w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(columns)
            for sample in rows:
                writer.writerow([sample["control_sequence"], sample["timestamp_cycles"]] +
                                [sample["values"].get(item, "") for item in channel_ids])
        return os.path.abspath(path)


def struct_unpack_capabilities(payload: bytes) -> tuple[int, ...]:
    import struct
    return struct.unpack("<BBHIHHHBBHH", payload)


def struct_unpack_command_result(payload: bytes) -> tuple[int, ...]:
    import struct
    return struct.unpack("<BBIH", payload)
