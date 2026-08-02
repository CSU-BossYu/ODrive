"""Fake-serial, protocol, bounded-buffer and generated-schema coverage."""

from __future__ import annotations

import json
import base64
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from odrive_usb.protocol import (  # noqa: E402
    Frame, MessageType, ProtocolError, StreamDecoder, decode_crash_report,
    decode_scope_batch, decode_test_session_status, encode_frame, FRAME_ERROR,
    session_begin_frame, session_id_frame,
)
from odrive_usb.transport import (  # noqa: E402
    HANDSHAKE_TIMEOUT_S, FakeSerialTransport, UsbDebugTransport,
)
from odrive_usb import symbolizer  # noqa: E402
from odrive_usb.replay import (  # noqa: E402
    corrupt_frame_crc, decode_bundle, drop_control_sequences, encode_bundle,
    make_bundle, mark_stale_feedback,
)


ROOT = Path(__file__).parents[3]


def capabilities(build_id=b"BUILD123", manifest=b"M" * 16):
    payload = struct.pack("<BBHIHHHBBHH", 4, 4, 512, 0x3FF, 1024,
                          8, 8, 8, 0, 64, 128)
    return Frame(MessageType.CAPABILITIES, payload=payload, sequence=1,
                 request_id=1, build_id=build_id, manifest_identity=manifest)


def establish(fake: FakeSerialTransport) -> UsbDebugTransport:
    transport = UsbDebugTransport(serial=fake)
    transport.connect(serial_device=fake)
    fake.take_host_bytes()  # HELLO
    fake.inject_device_bytes(encode_frame(capabilities()))
    assert any(event["type"] == "capabilities" for event in transport.poll())
    assert transport.status.handshake_complete
    return transport


def batch_frame(sequence=1, timestamp=0xFFFFFFF0, values=(1.0, 2.5)):
    payload = bytearray()
    payload.extend(struct.pack("<IHBBHHII", 7, 1, 2, 0, 0, 0, sequence, timestamp))
    payload.extend(struct.pack("<8H", 6, 11, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF))
    payload.extend(struct.pack("<IIHHff", sequence, timestamp, 0, 0, *values))
    return Frame(MessageType.SCOPE_DATA_BATCH, payload=bytes(payload), sequence=2,
                 build_id=b"BUILD123", manifest_identity=b"M" * 16)


def crash_payload(record_crc=0x12345678):
    payload = bytearray(struct.pack("<HHIIII", 1, 204, record_crc, 3, 1, 7))
    payload.extend(b"OLDCRASH")
    payload.extend(b"R" * 16)
    payload.extend(struct.pack("<8I", 1, 2, 3, 4, 12, 0x08001235,
                               0x08004567, 0x21000000))
    payload.extend(struct.pack("<3I", 0xFFFFFFFD, 0x20010000, 0x10001000))
    payload.extend(struct.pack("<8I", 0x200, 0x40000000, 0, 0, 0,
                               0xDEADBEEF, 3, 0))
    payload.extend(struct.pack("<4I", 99, 7, 0x0205, 3))
    payload.extend(struct.pack("<I", 1))
    payload.extend(struct.pack("<8I", 8, 9, 10, 0x00020001,
                               0x00030004, 11, 12, 13))
    payload.extend(bytes(32))
    assert len(payload) == 204
    return bytes(payload)


class TestUsbProtocol:
    def test_fragmentation_noise_sticky_frames_and_crc_recovery(self):
        decoder = StreamDecoder()
        first = encode_frame(capabilities())
        second = encode_frame(Frame(MessageType.SCOPE_STATUS, payload=bytes(32),
                                    build_id=b"BUILD123", manifest_identity=b"M" * 16))
        corrupted = bytearray(second); corrupted[-1] ^= 0x55
        result = []
        stream = b"noise" + first + bytes(corrupted) + second
        for byte in stream:
            result.extend(decoder.feed(bytes([byte])))
        assert [frame.message_type for frame in result] == [MessageType.CAPABILITIES, MessageType.SCOPE_STATUS]
        assert decoder.stats.crc_errors == 1
        assert decoder.stats.noise_bytes >= 5

    def test_large_serial_read_is_parsed_before_residue_is_bounded(self):
        decoder = StreamDecoder()
        raw = encode_frame(Frame(
            MessageType.SCOPE_STATUS, payload=bytes(32),
            build_id=b"BUILD123", manifest_identity=b"M" * 16))
        stream = raw * 64
        assert len(stream) > decoder.max_buffer

        decoded = decoder.feed(stream)

        assert len(decoded) == 64
        assert decoder.stats.frames_decoded == 64
        assert decoder.stats.dropped_frames == 0
        assert decoder.stats.noise_bytes == 0
        assert decoder.buffer == b""

    def test_unknown_schema_and_bad_length_are_rejected(self):
        raw = bytearray(encode_frame(capabilities()))
        raw[3] = 99
        with pytest.raises(ProtocolError):
            from odrive_usb.protocol import decode_frame
            decode_frame(bytes(raw))
        decoder = StreamDecoder()
        malformed = bytearray(encode_frame(capabilities()))
        malformed[14:16] = struct.pack("<H", 513)
        assert decoder.feed(bytes(malformed)) == []
        assert decoder.stats.length_errors >= 1

    def test_batch_decodes_f32_and_records_firmware_time(self):
        decoded = decode_scope_batch(batch_frame().payload)
        assert decoded["capture_id"] == 7
        assert decoded["samples"][0]["values"][11] == pytest.approx(2.5)
        assert decoded["samples"][0]["timestamp_cycles"] == 0xFFFFFFF0

    def test_crash_report_decodes_core_context_and_critical_event(self):
        decoded = decode_crash_report(crash_payload())
        assert decoded["record_build_id"] == b"OLDCRASH".hex()
        assert decoded["stacked"]["pc"] == 0x08004567
        assert decoded["fault_status"]["cfsr"] == 0x200
        assert decoded["safety_state"] == 5
        assert decoded["operation"] == 2
        assert decoded["critical_events"][0]["code"] == 2

    def test_test_session_codecs_are_fixed_and_bounded(self):
        begin = session_begin_frame(
            motion=True, hardware_estop_confirmed=True, lease_ms=2000,
            velocity_limit=0.5, current_limit=0.5, torque_limit=0.1,
            sequence=7, request_id=8)
        assert begin.message_type == MessageType.TEST_SESSION_BEGIN
        assert len(begin.payload) == 16
        assert struct.unpack("<BBHfff", begin.payload)[:3] == (3, 0, 2000)
        keepalive = session_id_frame(
            MessageType.TEST_SESSION_KEEPALIVE, 42, 9, 10)
        assert keepalive.payload == struct.pack("<I", 42)
        status = decode_test_session_status(struct.pack(
            "<BBHIIfff", 1, 3, 0, 42, 1800, 0.5, 0.5, 0.1))
        assert status["motion"] and status["hardware_estop_confirmed"]
        assert status["session_id"] == 42

    def test_symbolizer_rejects_missing_malformed_and_mismatched_identity(
            self, monkeypatch):
        tmp_path = ROOT / "Firmware" / "build" / "symbolizer-test"
        if tmp_path.exists():
            shutil.rmtree(tmp_path)
        build_id = "1122334455667788"
        manifest_identity = "00112233445566778899aabbccddeeff"
        artifact = tmp_path / "Firmware" / "build" / "artifacts" / build_id
        artifact.mkdir(parents=True)
        (artifact / "build_manifest.json").write_text(json.dumps({
            "build_id": build_id + "99",
            "dirty_sha256": manifest_identity + "00" * 16,
            "compiler": {"path": "missing-g++.exe"},
        }), encoding="utf-8")
        (artifact / "ODriveFirmware.elf").write_bytes(
            b"ELF" + bytes.fromhex(build_id) + bytes.fromhex(manifest_identity))
        try:
            monkeypatch.setattr(symbolizer, "ROOT", tmp_path)
            assert symbolizer._matching_artifact("", "") is None
            assert symbolizer._matching_artifact(
                "not-hex-identity", manifest_identity) is None
            assert symbolizer._matching_artifact(build_id, "ff" * 16) is None
            match = symbolizer._matching_artifact(build_id, manifest_identity)
            assert match is not None
            assert match[0] == artifact / "ODriveFirmware.elf"
            result = symbolizer.symbolize_crash(
                {"stacked": {"pc": 0, "lr": 0}})
            assert not result["reliable"]
            assert "no exact build/manifest" in result["error"]
        finally:
            shutil.rmtree(tmp_path)


class TestUsbTransport:
    def test_test_session_begin_keepalive_stop_and_end_are_correlated(self):
        fake = FakeSerialTransport()
        transport = establish(fake)
        request_id = transport.begin_test_session(
            motion=True, hardware_estop_confirmed=True, lease_ms=2000,
            velocity_limit=0.5, current_limit=0.5, torque_limit=0.1)
        requests = StreamDecoder().feed(fake.take_host_bytes())
        assert requests[-1].message_type == MessageType.TEST_SESSION_BEGIN
        payload = struct.pack("<BBHIIfff", 1, 3, 0, 77, 2000,
                              0.5, 0.5, 0.1)
        fake.inject_device_bytes(encode_frame(Frame(
            MessageType.TEST_SESSION_STATUS, payload=payload,
            request_id=request_id, build_id=b"BUILD123",
            manifest_identity=b"M" * 16)))
        event = transport.poll()[0]
        assert event["ok"] and event["session_id"] == 77

        transport.keepalive_test_session()
        transport.stop_test_session()
        requests = StreamDecoder().feed(fake.take_host_bytes())
        assert [item.message_type for item in requests] == [
            MessageType.TEST_SESSION_KEEPALIVE,
            MessageType.TEST_SESSION_STOP]
    def test_crash_report_is_acknowledged_by_record_crc(self):
        fake = FakeSerialTransport()
        transport = establish(fake)
        fake.inject_device_bytes(encode_frame(Frame(
            MessageType.CRASH_REPORT, payload=crash_payload(), sequence=2,
            build_id=b"BUILD123", manifest_identity=b"M" * 16)))

        event = transport.poll()[0]

        assert event["type"] == "crash_report"
        acknowledgements = StreamDecoder().feed(fake.take_host_bytes())
        assert len(acknowledgements) == 1
        assert acknowledgements[0].message_type == MessageType.CRASH_ACK
        assert acknowledgements[0].payload == struct.pack("<I", 0x12345678)

    def test_connect_discards_stale_device_bytes_before_binary_hello(self):
        fake = FakeSerialTransport()
        fake.inject_device_bytes(b"stale legacy bytes")
        transport = UsbDebugTransport(serial=fake)

        transport.connect(serial_device=fake)

        assert fake.read() == b""
        frames = StreamDecoder().feed(fake.take_host_bytes())
        assert len(frames) == 1
        assert frames[0].message_type == MessageType.HELLO

    def test_handshake_timeout_is_reported_once_and_remains_non_destructive(self):
        fake = FakeSerialTransport()
        transport = UsbDebugTransport(serial=fake)
        transport.connect(serial_device=fake)
        transport._handshake_started_at -= HANDSHAKE_TIMEOUT_S + 1.0

        events = transport.poll()

        assert events[0]["stage"] == "handshake"
        assert "CAPABILITIES" in transport.status.handshake_error
        assert transport.poll() == []
        assert transport.status.connected

    def test_fake_serial_handshake_scope_and_disconnect_are_bounded(self):
        fake = FakeSerialTransport()
        transport = establish(fake)
        transport.configure_scope({"channel_ids": [6, 11], "mode": 1,
                                   "trigger_type": 1, "sample_rate_hz": 10000,
                                   "decimation": 1, "pre_samples": 64,
                                   "post_samples": 128})
        transport.arm_scope(True)
        host_bytes = fake.take_host_bytes()
        assert len(host_bytes) > 100  # includes frames larger than one CDC packet
        fake.inject_device_bytes(encode_frame(batch_frame()), fragment_sizes=[1] * 40)
        events = []
        for _ in range(200):
            events.extend(transport.poll(max_bytes=1))
        assert any(event["type"] == "scope_data" for event in events)
        assert transport.status.timestamp_wraps == 0
        transport.disconnect()
        assert not transport.status.connected
        assert not any(b"DISARM" in data for data in fake.host_writes)

    def test_unknown_build_identity_is_explicitly_refused(self):
        fake = FakeSerialTransport()
        transport = UsbDebugTransport(serial=fake, expected_build_id=b"EXPECTED")
        transport.connect(serial_device=fake)
        fake.take_host_bytes()
        fake.inject_device_bytes(encode_frame(capabilities()))
        events = transport.poll()
        assert events and events[0]["type"] == "error"
        assert "build identity" in events[0]["error"]
        assert not transport.status.handshake_complete

    def test_trusted_identity_pair_is_required_when_whitelist_is_enabled(self):
        fake = FakeSerialTransport()
        transport = UsbDebugTransport(
            serial=fake,
            trusted_identities={(b"TRUSTED!", b"T" * 16)})
        transport.connect(serial_device=fake)
        fake.take_host_bytes()
        fake.inject_device_bytes(encode_frame(capabilities()))
        events = transport.poll()
        assert events and events[0]["type"] == "error"
        assert "build/manifest identity" in events[0]["error"]
        assert not transport.status.handshake_complete

    def test_scope_config_is_committed_only_after_matching_success_ack(self):
        fake = FakeSerialTransport()
        transport = establish(fake)
        request_id = transport.configure_scope({"channel_ids": [6],
                                                "decimation": 8})
        assert transport._scope_decimation == 1

        rejected = Frame(MessageType.SCOPE_STATUS, payload=bytes(32),
                         flags=FRAME_ERROR, request_id=request_id,
                         build_id=b"BUILD123", manifest_identity=b"M" * 16)
        fake.inject_device_bytes(encode_frame(rejected))
        event = transport.poll()[0]
        assert not event["ok"]
        assert "rejected_config" in event
        assert transport._scope_decimation == 1

        request_id = transport.configure_scope({"channel_ids": [6],
                                                "decimation": 8})
        accepted = Frame(MessageType.SCOPE_STATUS, payload=bytes(32),
                         request_id=request_id, build_id=b"BUILD123",
                         manifest_identity=b"M" * 16)
        fake.inject_device_bytes(encode_frame(accepted))
        event = transport.poll()[0]
        assert event["applied_config"]["decimation"] == 8
        assert transport._scope_decimation == 8

    def test_sequence_gap_timestamp_wrap_and_slow_consumer_are_bounded(self):
        fake = FakeSerialTransport()
        transport = establish(fake)
        first = encode_frame(batch_frame(sequence=0xFFFFFFFE, timestamp=0xFFFFFFF0))
        second = encode_frame(batch_frame(sequence=3, timestamp=4))
        fake.inject_device_bytes(first + second)
        events = transport.poll()
        assert len([e for e in events if e["type"] == "scope_data"]) == 2
        assert transport.status.timestamp_wraps == 1
        for index in range(1000):
            transport.events.append({"type": "scope_data", "index": index})
        assert len(transport.events) <= 256


class TestUsbReplay:
    def bundle(self):
        status = SimpleNamespace(
            build_id="4255494c44313233",
            manifest_identity="4d" * 16,
            protocol_release="4.5",
            protocol_version=4,
            schema_version=4,
        )
        raw = encode_frame(capabilities()) + encode_frame(batch_frame())
        events = [{
            "type": "scope_data",
            "opaque": b"\x00\xff",
            "samples": [
                {"control_sequence": 10, "timestamp_cycles": 0xFFFFFFF0,
                 "values": {6: 1.0}},
                {"control_sequence": 11, "timestamp_cycles": 4,
                 "values": {6: 2.0}},
            ],
        }]
        return make_bundle(status=status, raw_capture=raw, events=events)

    def test_export_is_deterministic_json_and_preserves_opaque_payload(self):
        bundle = self.bundle()
        encoded = encode_bundle(bundle)
        decoded = decode_bundle(encoded)

        assert encode_bundle(decoded) == encoded
        assert decoded["capture"]["events"][0]["opaque"] == {
            "encoding": "base64", "data": "AP8="}
        assert decoded["capture"]["events"][0]["samples"][0]["values"] == {
            "6": 1.0}

    def test_sequence_loss_crc_corruption_and_stale_feedback_mutations(self):
        bundle = self.bundle()
        dropped = drop_control_sequences(bundle, {10})
        assert [sample["control_sequence"] for sample in
                dropped["capture"]["events"][0]["samples"]] == [11]

        corrupted = corrupt_frame_crc(bundle, 0)
        raw = base64.b64decode(corrupted["capture"]["raw_frames_base64"])
        decoder = StreamDecoder()
        frames = decoder.feed(raw)
        assert [frame.message_type for frame in frames] == [
            MessageType.SCOPE_DATA_BATCH]
        assert decoder.stats.crc_errors == 1

        stale = mark_stale_feedback(
            bundle, now_cycles=5, maximum_age_cycles=20)
        samples = stale["capture"]["events"][0]["samples"]
        assert samples[0]["feedback_age_cycles"] == 21
        assert samples[0]["feedback_stale"] is True
        assert samples[1]["feedback_age_cycles"] == 1
        assert samples[1]["feedback_stale"] is False


def test_scope_generator_outputs_match_schema():
    generator = ROOT / "tools" / "generate_scope_schema.py"
    schema = ROOT / "docs" / "scope_channel_schema.json"
    out = ROOT / "Firmware" / "build" / "scope-schema-test"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    try:
        subprocess.run([
            sys.executable, str(generator), "--schema", str(schema),
            "--cpp-output", str(out / "scope.hpp"),
            "--python-output", str(out / "scope.py"),
            "--typescript-output", str(out / "scope.ts"),
        ], check=True)
        assert (out / "scope.hpp").read_text(encoding="utf-8") == (ROOT / "Firmware/MotorControl/scope_channels_generated.hpp").read_text(encoding="utf-8")
        assert (out / "scope.py").read_text(encoding="utf-8") == (ROOT / "foc_ui/backend/odrive_usb/scope_channels_generated.py").read_text(encoding="utf-8")
        assert (out / "scope.ts").read_text(encoding="utf-8") == (ROOT / "foc_ui/frontend/src/scope_channels_generated.ts").read_text(encoding="utf-8")
    finally:
        shutil.rmtree(out)
