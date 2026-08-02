"""Versioned, deterministic USB capture export and mutation helpers."""

from __future__ import annotations

import base64
from copy import deepcopy
import json
from typing import Any

from .protocol import HEADER_SIZE, MAGIC, MAX_PAYLOAD, TRAILER_SIZE


FORMAT = "odrive-usb-replay"
SCHEMA_VERSION = 1


class ReplayError(ValueError):
    pass


def make_bundle(*, status: Any, raw_capture: bytes,
                events: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "format": FORMAT,
        "schema_version": SCHEMA_VERSION,
        "identity": {
            "build_id": status.build_id or "",
            "manifest_identity": status.manifest_identity or "",
        },
        "protocol": {
            "release": status.protocol_release or "",
            "version": status.protocol_version or 0,
            "schema_version": status.schema_version or 0,
        },
        "capture": {
            "raw_frames_base64": base64.b64encode(raw_capture).decode("ascii"),
            "events": _json_safe(events),
        },
        "mutations": [],
    }


def encode_bundle(bundle: dict[str, Any]) -> bytes:
    validate_bundle(bundle)
    return (json.dumps(bundle, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def decode_bundle(data: bytes | str) -> dict[str, Any]:
    try:
        bundle = json.loads(data.decode("utf-8") if isinstance(data, bytes) else data)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReplayError("invalid replay JSON") from exc
    validate_bundle(bundle)
    return bundle


def validate_bundle(bundle: dict[str, Any]) -> None:
    if bundle.get("format") != FORMAT or bundle.get("schema_version") != SCHEMA_VERSION:
        raise ReplayError("unsupported replay format/schema")
    for key in ("identity", "protocol", "capture", "mutations"):
        if key not in bundle:
            raise ReplayError(f"replay missing {key}")
    capture = bundle["capture"]
    if not isinstance(capture.get("events"), list):
        raise ReplayError("replay events must be an array")
    try:
        base64.b64decode(capture.get("raw_frames_base64", ""), validate=True)
    except (ValueError, TypeError) as exc:
        raise ReplayError("invalid replay raw frame encoding") from exc


def drop_control_sequences(bundle: dict[str, Any], sequences: set[int]) -> dict[str, Any]:
    result = deepcopy(bundle)
    for event in result["capture"]["events"]:
        if event.get("type") != "scope_data":
            continue
        event["samples"] = [sample for sample in event.get("samples", [])
                            if int(sample.get("control_sequence", -1)) not in sequences]
    result["mutations"].append({"type": "drop_control_sequence",
                                "sequences": sorted(sequences)})
    return result


def corrupt_frame_crc(bundle: dict[str, Any], frame_index: int) -> dict[str, Any]:
    result = deepcopy(bundle)
    raw = bytearray(base64.b64decode(result["capture"]["raw_frames_base64"]))
    frames = _frame_ranges(raw)
    if not 0 <= frame_index < len(frames):
        raise ReplayError("frame index out of range")
    _, end = frames[frame_index]
    raw[end - 1] ^= 0x01
    result["capture"]["raw_frames_base64"] = base64.b64encode(raw).decode("ascii")
    result["mutations"].append({"type": "corrupt_frame_crc",
                                "frame_index": frame_index})
    return result


def mark_stale_feedback(bundle: dict[str, Any], *, now_cycles: int,
                        maximum_age_cycles: int) -> dict[str, Any]:
    result = deepcopy(bundle)
    for event in result["capture"]["events"]:
        for sample in event.get("samples", []):
            age = (now_cycles - int(sample.get("timestamp_cycles", 0))) & 0xFFFFFFFF
            sample["feedback_age_cycles"] = age
            sample["feedback_stale"] = age < 0x80000000 and age > maximum_age_cycles
    result["mutations"].append({"type": "mark_stale_feedback",
                                "now_cycles": now_cycles & 0xFFFFFFFF,
                                "maximum_age_cycles": maximum_age_cycles})
    return result


def _frame_ranges(raw: bytearray) -> list[tuple[int, int]]:
    magic = int(MAGIC).to_bytes(2, "little")
    ranges = []
    offset = 0
    while offset + HEADER_SIZE + TRAILER_SIZE <= len(raw):
        start = raw.find(magic, offset)
        if start < 0 or start + HEADER_SIZE > len(raw):
            break
        length = int.from_bytes(raw[start + 14:start + 16], "little")
        if length > MAX_PAYLOAD:
            offset = start + 2
            continue
        end = start + HEADER_SIZE + length + TRAILER_SIZE
        if end > len(raw):
            break
        ranges.append((start, end))
        offset = end
    return ranges


def _json_safe(value: Any) -> Any:
    """Normalize decoder output without losing opaque binary payloads."""
    if isinstance(value, bytes):
        return {
            "encoding": "base64",
            "data": base64.b64encode(value).decode("ascii"),
        }
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return deepcopy(value)
