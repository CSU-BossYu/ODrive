"""Python codec for the versioned ODrive USB Debug/Test wire protocol."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import struct
from typing import Any

from .scope_channels_generated import CHANNEL_BY_ID, MAX_CAPTURE_CHANNELS

MAGIC = 0x4F44
PROTOCOL_VERSION = 4
SCHEMA_VERSION = 4
PROTOCOL_RELEASE = "4.5"
HEADER_SIZE = 40
TRAILER_SIZE = 4
MAX_PAYLOAD = 512
MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + TRAILER_SIZE
SCOPE_DATA_HEADER_SIZE = 36
CRASH_REPORT_PAYLOAD_SIZE = 204
SAMPLE_FLAG_SEQUENCE_GAP = 1
SAMPLE_FLAG_DROPPED = 2
BATCH_FLAG_SEQUENCE_GAP = 1
BATCH_FLAG_DROPPED = 2
BATCH_FLAG_COMPLETE = 4
FRAME_RESPONSE = 1
FRAME_ERROR = 2


class MessageType(IntEnum):
    HELLO = 1
    CAPABILITIES = 2
    COMMAND = 3
    COMMAND_RESULT = 4
    FAULT_EVENT = 5
    STATE_EVENT = 6
    LOG_RECORD = 7
    SCOPE_RECORD = 8
    STATS = 9
    CALIBRATION_DATA = 10
    SCOPE_CONFIG = 11
    SCOPE_ARM = 12
    SCOPE_STATUS = 13
    SCOPE_DATA_BATCH = 14
    SCOPE_STOP = 15
    CRASH_REPORT = 16
    CRASH_ACK = 17
    TEST_SESSION_BEGIN = 18
    TEST_SESSION_STATUS = 19
    TEST_SESSION_KEEPALIVE = 20
    TEST_SESSION_STOP = 21
    TEST_SESSION_END = 22


@dataclass
class Frame:
    message_type: MessageType
    payload: bytes = b""
    flags: int = 0
    sequence: int = 0
    request_id: int = 0
    protocol_version: int = PROTOCOL_VERSION
    schema_version: int = SCHEMA_VERSION
    build_id: bytes = b"\0" * 8
    manifest_identity: bytes = b"\0" * 16


class ProtocolError(ValueError):
    pass


def crc32(data: bytes) -> int:
    return __import__("binascii").crc32(data) & 0xFFFFFFFF


def encode_frame(frame: Frame) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 512 bytes")
    if len(frame.build_id) != 8 or len(frame.manifest_identity) != 16:
        raise ProtocolError("invalid identity length")
    header = struct.pack(
        "<HBBBBIIH8s16s",
        MAGIC, frame.protocol_version, frame.schema_version,
        int(frame.message_type), frame.flags, frame.sequence & 0xFFFFFFFF,
        frame.request_id & 0xFFFFFFFF, len(frame.payload), frame.build_id,
        frame.manifest_identity,
    )
    return header + frame.payload + struct.pack("<I", crc32(header + frame.payload))


def decode_frame(data: bytes, *, accept_unknown_identity: bool = True) -> Frame:
    if len(data) < HEADER_SIZE + TRAILER_SIZE:
        raise ProtocolError("frame too short")
    fields = struct.unpack_from("<HBBBBIIH8s16s", data, 0)
    magic, protocol, schema, message, flags, sequence, request_id, length, build_id, manifest = fields
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if length > MAX_PAYLOAD or len(data) != HEADER_SIZE + length + TRAILER_SIZE:
        raise ProtocolError("bad payload length")
    expected = struct.unpack_from("<I", data, HEADER_SIZE + length)[0]
    if expected != crc32(data[:HEADER_SIZE + length]):
        raise ProtocolError("CRC mismatch")
    if protocol != PROTOCOL_VERSION or schema != SCHEMA_VERSION:
        raise ProtocolError(f"unsupported protocol/schema {protocol}.{schema}")
    try:
        message_type = MessageType(message)
    except ValueError as exc:
        raise ProtocolError(f"unknown message type {message}") from exc
    return Frame(message_type, data[HEADER_SIZE:HEADER_SIZE + length], flags,
                 sequence, request_id, protocol, schema, build_id, manifest)


@dataclass
class ParserStats:
    bytes_seen: int = 0
    frames_decoded: int = 0
    noise_bytes: int = 0
    resynchronizations: int = 0
    length_errors: int = 0
    crc_errors: int = 0
    unknown_frames: int = 0
    dropped_frames: int = 0


class StreamDecoder:
    """Incremental parser that never retains more than a bounded byte buffer."""

    def __init__(self, max_buffer: int = MAX_FRAME * 4):
        self.max_buffer = max_buffer
        self.buffer = bytearray()
        self.stats = ParserStats()

    def reset(self) -> None:
        self.buffer.clear()
        self.stats = ParserStats()

    def feed(self, data: bytes) -> list[Frame]:
        self.stats.bytes_seen += len(data)
        self.buffer.extend(data)
        frames: list[Frame] = []
        magic = struct.pack("<H", MAGIC)
        while True:
            start = self.buffer.find(magic)
            if start < 0:
                keep = 1 if self.buffer[-1:] == magic[:1] else 0
                self.stats.noise_bytes += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                self.stats.noise_bytes += start
                self.stats.resynchronizations += 1
                del self.buffer[:start]
            if len(self.buffer) < HEADER_SIZE:
                break
            length = struct.unpack_from("<H", self.buffer, 14)[0]
            if length > MAX_PAYLOAD:
                self.stats.length_errors += 1
                del self.buffer[:2]
                continue
            total = HEADER_SIZE + length + TRAILER_SIZE
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            try:
                frame = decode_frame(raw)
            except ProtocolError as exc:
                if "CRC" in str(exc):
                    self.stats.crc_errors += 1
                elif "protocol/schema" in str(exc):
                    self.stats.unknown_frames += 1
                else:
                    self.stats.dropped_frames += 1
                del self.buffer[:2]
                continue
            frames.append(frame)
            self.stats.frames_decoded += 1
            del self.buffer[:total]
        # Bound only undecoded residue. A single serial read can legitimately
        # contain many complete frames and can therefore be much larger than
        # max_buffer. Truncating before parsing destroyed valid frame prefixes
        # during high-rate Scope streaming. After the loop, valid incomplete
        # residue is always smaller than one maximum-size frame.
        if len(self.buffer) > self.max_buffer:
            self.stats.dropped_frames += 1
            del self.buffer[:len(self.buffer) - self.max_buffer]
        return frames


def hello_frame(sequence: int = 1, request_id: int = 1) -> Frame:
    return Frame(MessageType.HELLO, sequence=sequence, request_id=request_id)


def command_frame(command_type: int, operation: int = 0, arg0: int = 0,
                  sequence: int = 1, request_id: int = 1) -> Frame:
    return Frame(MessageType.COMMAND, struct.pack("<BBHI", command_type,
                                                    operation, 0, arg0),
                 sequence=sequence, request_id=request_id)


def session_begin_frame(*, motion: bool, hardware_estop_confirmed: bool,
                        lease_ms: int, velocity_limit: float,
                        current_limit: float, torque_limit: float,
                        sequence: int, request_id: int) -> Frame:
    flags = int(motion) | (int(hardware_estop_confirmed) << 1)
    payload = struct.pack("<BBHfff", flags, 0, lease_ms, velocity_limit,
                          current_limit, torque_limit)
    return Frame(MessageType.TEST_SESSION_BEGIN, payload, sequence=sequence,
                 request_id=request_id)


def session_id_frame(message_type: MessageType, session_id: int,
                     sequence: int, request_id: int) -> Frame:
    if message_type not in (MessageType.TEST_SESSION_KEEPALIVE,
                            MessageType.TEST_SESSION_STOP,
                            MessageType.TEST_SESSION_END):
        raise ProtocolError("invalid test-session request type")
    return Frame(message_type, struct.pack("<I", session_id),
                 sequence=sequence, request_id=request_id)


def decode_test_session_status(payload: bytes) -> dict[str, Any]:
    if len(payload) != 24:
        raise ProtocolError("invalid TEST_SESSION_STATUS length")
    state, flags, reason, session_id, remaining, velocity, current, torque = \
        struct.unpack("<BBHIIfff", payload)
    return {"state": state, "flags": flags, "reason": reason,
            "session_id": session_id, "lease_remaining_ms": remaining,
            "velocity_limit": velocity, "current_limit": current,
            "torque_limit": torque, "motion": bool(flags & 1),
            "hardware_estop_confirmed": bool(flags & 2)}


def scope_config_payload(config: dict[str, Any]) -> bytes:
    channels = [int(value) for value in config.get("channel_ids", [])]
    if not 0 < len(channels) <= MAX_CAPTURE_CHANNELS:
        raise ProtocolError("scope channel count out of range")
    channels += [0xFFFF] * (MAX_CAPTURE_CHANNELS - len(channels))
    return struct.pack(
        "<BBHBBIHHHBBf8H",
        int(config.get("mode", 0)), int(config.get("trigger_type", 0)),
        int(config.get("trigger_channel", 0xFFFF)),
        int(config.get("trigger_edge", 0)), int(config.get("trigger_state", 0)),
        int(config.get("sample_rate_hz", 1000)), int(config.get("decimation", 10)),
        int(config.get("pre_samples", 0)), int(config.get("post_samples", 0)),
        len([item for item in channels if item != 0xFFFF]), 0,
        float(config.get("threshold", 0.0)), *channels)


def scope_config_frame(config: dict[str, Any], sequence: int, request_id: int) -> Frame:
    return Frame(MessageType.SCOPE_CONFIG, scope_config_payload(config),
                 sequence=sequence, request_id=request_id)


def scope_arm_frame(arm: bool | int, sequence: int, request_id: int) -> Frame:
    action = int(arm) if isinstance(arm, int) else int(arm)
    return Frame(MessageType.SCOPE_ARM, struct.pack("<BB", action, 0),
                 sequence=sequence, request_id=request_id)


def scope_stop_frame(sequence: int, request_id: int) -> Frame:
    return Frame(MessageType.SCOPE_STOP, sequence=sequence, request_id=request_id)


def _decode_raw_value(channel_id: int, raw: int) -> int | float:
    channel = CHANNEL_BY_ID[channel_id]
    if channel.wire_type == "f32":
        return struct.unpack("<f", struct.pack("<I", raw))[0]
    if channel.wire_type == "u8":
        return raw & 0xFF
    return raw


def decode_scope_status(payload: bytes) -> dict[str, Any]:
    if len(payload) != 32:
        raise ProtocolError("invalid SCOPE_STATUS length")
    capture_id, state, reason, mode, trigger, count, dropped, gaps, channels, reserved, decimation, first_ts, last_ts = struct.unpack(
        "<IBBBBIII BBHII", payload)
    if reserved != 0:
        raise ProtocolError("non-zero SCOPE_STATUS reserved field")
    return {"capture_id": capture_id, "state": state, "final_reason": reason,
            "mode": mode, "trigger_type": trigger, "sample_count": count,
            "dropped_samples": dropped, "sequence_gaps": gaps,
            "channel_count": channels, "decimation": decimation,
            "first_timestamp_cycles": first_ts, "last_timestamp_cycles": last_ts}


def decode_scope_batch(payload: bytes) -> dict[str, Any]:
    if len(payload) < SCOPE_DATA_HEADER_SIZE:
        raise ProtocolError("SCOPE_DATA_BATCH too short")
    capture_id, sample_count, channel_count, flags, dropped, reserved, first_seq, first_ts = struct.unpack_from(
        "<I H B B H H I I", payload, 0)
    if not 0 < channel_count <= MAX_CAPTURE_CHANNELS or reserved != 0:
        raise ProtocolError("invalid SCOPE_DATA_BATCH header")
    ids = list(struct.unpack_from("<8H", payload, 20)[:channel_count])
    if any(channel_id not in CHANNEL_BY_ID for channel_id in ids):
        raise ProtocolError("unknown ScopeChannel in data batch")
    sample_size = 12 + 4 * channel_count
    expected = SCOPE_DATA_HEADER_SIZE + sample_count * sample_size
    if sample_count == 0 or expected != len(payload):
        raise ProtocolError("invalid SCOPE_DATA_BATCH sample length")
    samples = []
    offset = SCOPE_DATA_HEADER_SIZE
    for _ in range(sample_count):
        sequence, timestamp, sample_flags, sample_reserved = struct.unpack_from(
            "<IIHH", payload, offset)
        if sample_reserved != 0:
            raise ProtocolError("non-zero sample reserved field")
        offset += 12
        raw_values = struct.unpack_from(f"<{channel_count}I", payload, offset)
        offset += 4 * channel_count
        samples.append({"control_sequence": sequence,
                        "timestamp_cycles": timestamp,
                        "flags": sample_flags,
                        "values": {channel_id: _decode_raw_value(channel_id, raw)
                                   for channel_id, raw in zip(ids, raw_values)}})
    return {"capture_id": capture_id, "sample_count": sample_count,
            "channel_ids": ids, "flags": flags, "dropped_samples": dropped,
            "first_sequence": first_seq, "first_timestamp_cycles": first_ts,
            "samples": samples}


def decode_crash_report(payload: bytes) -> dict[str, Any]:
    if len(payload) != CRASH_REPORT_PAYLOAD_SIZE:
        raise ProtocolError("invalid CRASH_REPORT length")
    version, encoded_size = struct.unpack_from("<HH", payload, 0)
    if version != 1 or encoded_size != CRASH_REPORT_PAYLOAD_SIZE:
        raise ProtocolError("unknown CRASH_REPORT format")
    record_crc, generation, fault_kind, flags = struct.unpack_from(
        "<IIII", payload, 4)
    offset = 20
    record_build_id = payload[offset:offset + 8].hex(); offset += 8
    record_manifest_identity = payload[offset:offset + 16].hex(); offset += 16
    stacked_values = struct.unpack_from("<8I", payload, offset); offset += 32
    stacked = dict(zip(("r0", "r1", "r2", "r3", "r12", "lr", "pc", "xpsr"),
                       stacked_values))
    exception_return, msp, psp = struct.unpack_from("<III", payload, offset)
    offset += 12
    fault_values = struct.unpack_from("<8I", payload, offset); offset += 32
    fault_status = dict(zip(("cfsr", "hfsr", "dfsr", "afsr", "mmfar",
                             "bfar", "icsr", "shcsr"), fault_values))
    control_sequence, state_epoch, safety_operation, active_irq = struct.unpack_from(
        "<IIII", payload, offset)
    offset += 16
    critical_count, = struct.unpack_from("<I", payload, offset)
    offset += 4
    if critical_count > 2:
        raise ProtocolError("invalid CRASH_REPORT critical event count")
    critical_events = []
    for index in range(2):
        values = struct.unpack_from("<8I", payload, offset)
        offset += 32
        if index >= critical_count:
            continue
        source_code, site_severity = values[3], values[4]
        critical_events.append({
            "sequence": values[0], "control_sequence": values[1],
            "timestamp_cycles": values[2],
            "source": source_code & 0xFFFF, "code": source_code >> 16,
            "site": site_severity & 0xFFFF,
            "severity": (site_severity >> 16) & 0xFF,
            "arg0": values[5], "arg1": values[6], "arg2": values[7],
        })
    return {
        "format_version": version, "record_crc": record_crc,
        "generation": generation, "fault_kind": fault_kind, "flags": flags,
        "stack_frame_valid": bool(flags & 1),
        "context_valid": bool(flags & 2),
        "critical_events_valid": bool(flags & 4),
        "extended_fp_frame": bool(flags & 8),
        "record_build_id": record_build_id,
        "record_manifest_identity": record_manifest_identity,
        "stacked": stacked, "exception_return": exception_return,
        "msp": msp, "psp": psp, "fault_status": fault_status,
        "control_sequence": control_sequence, "state_epoch": state_epoch,
        "safety_state": safety_operation & 0xFF,
        "operation": (safety_operation >> 8) & 0xFF,
        "active_irq": active_irq, "critical_events": critical_events,
    }
