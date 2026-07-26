"""Versioned raw calibration sample and .odcal container format."""

from __future__ import annotations

from dataclasses import astuple, dataclass
import io
import json
import struct
from typing import BinaryIO, Iterator, Sequence
import zlib


FILE_MAGIC = b'ODCAL1\0\0'
CHUNK_MAGIC = b'ODCK'
STREAM_MAGIC = b'ODCR'
SCHEMA_VERSION = 1

FILE_HEADER = struct.Struct('<8sHHII')
CHUNK_HEADER = struct.Struct('<4sHHIIII')
STREAM_HEADER = struct.Struct('<4sHHHHHHIII')
SAMPLE_V1 = struct.Struct('<IIIIIIIIHHHH18f')
ELECTRICAL_SAMPLE_V1 = struct.Struct('<IIIIHHHH10f')
assert SAMPLE_V1.size == 112
assert ELECTRICAL_SAMPLE_V1.size == 64
assert STREAM_HEADER.size == 28

RECORD_FULL_V1 = 1
RECORD_ELECTRICAL_FAST_V1 = 2


@dataclass(frozen=True)
class CalibrationSampleV1:
    record_type = RECORD_FULL_V1
    sample_ticks: int
    control_ticks: int
    sequence: int
    session_id: int
    main_sequence: int
    aux_sequence: int
    pair_sequence: int
    pair_sample_skew_cycles: int
    main_raw: int
    aux_raw: int
    axis_state: int
    flags: int
    ia: float
    ib: float
    ic: float
    id: float
    iq: float
    vd_applied: float
    vq_applied: float
    vbus: float
    duty_a: float
    duty_b: float
    duty_c: float
    position_turns: float
    velocity_turns_per_s: float
    electrical_phase: float
    electrical_velocity: float
    output_position_turns: float
    board_temperature: float
    motor_temperature: float

    def pack(self) -> bytes:
        return SAMPLE_V1.pack(*astuple(self))

    @classmethod
    def unpack(cls, data: bytes) -> 'CalibrationSampleV1':
        if len(data) != SAMPLE_V1.size:
            raise ValueError(f'sample must be {SAMPLE_V1.size} bytes')
        return cls(*SAMPLE_V1.unpack(data))


@dataclass(frozen=True)
class CalibrationElectricalSampleV1:
    record_type = RECORD_ELECTRICAL_FAST_V1

    sample_ticks: int
    control_ticks: int
    sequence: int
    session_id: int
    main_raw: int
    aux_raw: int
    axis_state: int
    flags: int
    ia: float
    ib: float
    ic: float
    electrical_phase: float
    vd_applied: float
    vq_applied: float
    vbus: float
    duty_a: float
    duty_b: float
    duty_c: float

    def pack(self) -> bytes:
        return ELECTRICAL_SAMPLE_V1.pack(*astuple(self))

    @classmethod
    def unpack(cls, data: bytes) -> 'CalibrationElectricalSampleV1':
        if len(data) != ELECTRICAL_SAMPLE_V1.size:
            raise ValueError(
                f'electrical sample must be {ELECTRICAL_SAMPLE_V1.size} bytes')
        return cls(*ELECTRICAL_SAMPLE_V1.unpack(data))


RECORD_LAYOUTS = {
    RECORD_FULL_V1: (SAMPLE_V1, CalibrationSampleV1),
    RECORD_ELECTRICAL_FAST_V1:
        (ELECTRICAL_SAMPLE_V1, CalibrationElectricalSampleV1),
}


@dataclass(frozen=True)
class CalibrationChunk:
    sequence_start: int
    record_type: int
    samples: tuple[CalibrationSampleV1 | CalibrationElectricalSampleV1, ...]


@dataclass(frozen=True)
class CalibrationStreamRecord:
    axis: int
    flags: int
    sample: CalibrationSampleV1 | CalibrationElectricalSampleV1


class CalibrationStreamDecoder:
    """Incrementally extracts CRC-protected ODCR frames from USB CDC bytes.

    The CDC stream can also contain normal ASCII stdout. The decoder scans for
    frame magic and retains at most the possible magic prefix when no frame is
    present, so unrelated logging cannot grow memory without bound.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.bad_headers = 0
        self.bad_payloads = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[CalibrationStreamRecord]:
        self.buffer.extend(data)
        records: list[CalibrationStreamRecord] = []

        while True:
            magic_at = self.buffer.find(STREAM_MAGIC)
            if magic_at < 0:
                keep = min(len(self.buffer), len(STREAM_MAGIC) - 1)
                self.discarded_bytes += len(self.buffer) - keep
                if keep:
                    del self.buffer[:-keep]
                else:
                    self.buffer.clear()
                break
            if magic_at:
                self.discarded_bytes += magic_at
                del self.buffer[:magic_at]
            if len(self.buffer) < STREAM_HEADER.size:
                break

            values = STREAM_HEADER.unpack_from(self.buffer)
            (magic, schema, header_size, axis, record_type, payload_size,
             flags, sequence, payload_crc, header_crc) = values
            layout = RECORD_LAYOUTS.get(record_type)
            header_valid = (
                magic == STREAM_MAGIC and schema == SCHEMA_VERSION and
                header_size == STREAM_HEADER.size and layout is not None and
                payload_size == layout[0].size and
                header_crc == (zlib.crc32(
                    self.buffer[:STREAM_HEADER.size - 4]) & 0xFFFFFFFF)
            )
            if not header_valid:
                self.bad_headers += 1
                del self.buffer[0]
                continue

            frame_size = STREAM_HEADER.size + payload_size
            if len(self.buffer) < frame_size:
                break
            payload = bytes(self.buffer[STREAM_HEADER.size:frame_size])
            if (zlib.crc32(payload) & 0xFFFFFFFF) != payload_crc:
                self.bad_payloads += 1
                del self.buffer[0]
                continue

            sample = layout[1].unpack(payload)
            if sample.sequence != sequence:
                self.bad_headers += 1
                del self.buffer[0]
                continue
            records.append(CalibrationStreamRecord(axis, flags, sample))
            del self.buffer[:frame_size]

        return records


class CalibrationRecordWriter:
    def __init__(self, stream: BinaryIO, metadata: dict):
        self.stream = stream
        self.next_sequence: int | None = None
        metadata_bytes = json.dumps(
            metadata, ensure_ascii=False, sort_keys=True,
            separators=(',', ':')).encode('utf-8')
        metadata_crc = zlib.crc32(metadata_bytes) & 0xFFFFFFFF
        stream.write(FILE_HEADER.pack(
            FILE_MAGIC, SCHEMA_VERSION, 0, len(metadata_bytes), metadata_crc))
        stream.write(metadata_bytes)

    def append(self, samples: Sequence[
            CalibrationSampleV1 | CalibrationElectricalSampleV1]) -> None:
        if not samples:
            return
        sequence_start = samples[0].sequence
        record_type = samples[0].record_type
        if any(sample.record_type != record_type for sample in samples):
            raise ValueError('a chunk cannot mix calibration record types')
        record_struct, _ = RECORD_LAYOUTS[record_type]
        for offset, sample in enumerate(samples):
            expected = sequence_start + offset
            if sample.sequence != expected:
                raise ValueError(
                    f'non-contiguous sample sequence: expected {expected}, '
                    f'got {sample.sequence}')
        if self.next_sequence is not None and sequence_start != self.next_sequence:
            raise ValueError(
                f'chunk sequence gap: expected {self.next_sequence}, '
                f'got {sequence_start}')

        payload = b''.join(sample.pack() for sample in samples)
        payload_crc = zlib.crc32(payload) & 0xFFFFFFFF
        self.stream.write(CHUNK_HEADER.pack(
            CHUNK_MAGIC, SCHEMA_VERSION, record_type, sequence_start,
            len(samples), record_struct.size, payload_crc))
        self.stream.write(payload)
        self.next_sequence = sequence_start + len(samples)


class CalibrationRecordReader:
    def __init__(self, stream: BinaryIO):
        self.stream = stream
        header = self._read_exact(FILE_HEADER.size)
        magic, version, flags, metadata_len, metadata_crc = \
            FILE_HEADER.unpack(header)
        if magic != FILE_MAGIC:
            raise ValueError('not an ODCAL file')
        if version != SCHEMA_VERSION:
            raise ValueError(f'unsupported ODCAL schema {version}')
        self.flags = flags
        metadata_bytes = self._read_exact(metadata_len)
        if (zlib.crc32(metadata_bytes) & 0xFFFFFFFF) != metadata_crc:
            raise ValueError('metadata CRC mismatch')
        self.metadata = json.loads(metadata_bytes.decode('utf-8'))

    def chunks(self) -> Iterator[CalibrationChunk]:
        expected_sequence: int | None = None
        while True:
            header = self.stream.read(CHUNK_HEADER.size)
            if header == b'':
                return
            if len(header) != CHUNK_HEADER.size:
                raise ValueError('truncated chunk header')
            magic, version, record_type, sequence_start, sample_count, \
                record_size, payload_crc = CHUNK_HEADER.unpack(header)
            if magic != CHUNK_MAGIC:
                raise ValueError('invalid chunk magic')
            if version != SCHEMA_VERSION or record_type not in RECORD_LAYOUTS:
                raise ValueError('unsupported calibration chunk layout')
            record_struct, record_class = RECORD_LAYOUTS[record_type]
            if record_size != record_struct.size:
                raise ValueError('calibration chunk record size mismatch')
            if expected_sequence is not None and sequence_start != expected_sequence:
                raise ValueError(
                    f'chunk sequence gap: expected {expected_sequence}, '
                    f'got {sequence_start}')
            payload = self._read_exact(sample_count * record_size)
            if (zlib.crc32(payload) & 0xFFFFFFFF) != payload_crc:
                raise ValueError('chunk payload CRC mismatch')
            samples = tuple(
                record_class.unpack(payload[i:i + record_size])
                for i in range(0, len(payload), record_size)
            )
            yield CalibrationChunk(sequence_start, record_type, samples)
            expected_sequence = sequence_start + sample_count

    def _read_exact(self, size: int) -> bytes:
        data = self.stream.read(size)
        if len(data) != size:
            raise ValueError(f'truncated ODCAL data: wanted {size}, got {len(data)}')
        return data


def encode_record(metadata: dict,
                  chunks: Sequence[Sequence[
                      CalibrationSampleV1 | CalibrationElectricalSampleV1]]) -> bytes:
    stream = io.BytesIO()
    writer = CalibrationRecordWriter(stream, metadata)
    for samples in chunks:
        writer.append(samples)
    return stream.getvalue()
