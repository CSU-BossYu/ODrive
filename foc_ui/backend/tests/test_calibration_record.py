import io
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.calibration_record import (
    CalibrationRecordReader,
    CalibrationStreamDecoder,
    CalibrationElectricalSampleV1,
    CalibrationSampleV1,
    ELECTRICAL_SAMPLE_V1,
    SAMPLE_V1,
    encode_record,
    STREAM_HEADER,
    STREAM_MAGIC,
)
import struct
import zlib


def sample(sequence: int) -> CalibrationSampleV1:
    return CalibrationSampleV1(
        100 + sequence, 90 + sequence, sequence, 42,
        1000 + sequence, 2000 + sequence, 3000 + sequence, 400,
        1000 + sequence, 2000 + sequence, 8, 0x37,
        1.0, 2.0, -3.0, 0.1, 1.2, 0.3, 2.4, 24.0,
        0.4, 0.5, 0.6, 1.25, 0.75, 2.1, 3.2, 1.25,
        35.0, 40.0,
    )


def electrical_sample(sequence: int) -> CalibrationElectricalSampleV1:
    return CalibrationElectricalSampleV1(
        100 + sequence, 90 + sequence, sequence, 42,
        1000 + sequence, 2000 + sequence, 8, 0x37,
        1.0, 2.0, -3.0, 2.1, 0.3, 2.4, 24.0, 0.4, 0.5, 0.6,
    )


def stream_frame(value: CalibrationElectricalSampleV1, axis: int = 0) -> bytes:
    payload = value.pack()
    prefix = STREAM_HEADER.pack(
        STREAM_MAGIC, 1, STREAM_HEADER.size, axis, value.record_type,
        len(payload), 0, value.sequence,
        zlib.crc32(payload) & 0xFFFFFFFF, 0)
    header_crc = zlib.crc32(prefix[:-4]) & 0xFFFFFFFF
    return prefix[:-4] + struct.pack('<I', header_crc) + payload


class CalibrationRecordTests(unittest.TestCase):
    def test_wire_size_is_stable(self):
        self.assertEqual(SAMPLE_V1.size, 112)
        self.assertEqual(len(sample(0).pack()), 112)
        self.assertEqual(ELECTRICAL_SAMPLE_V1.size, 64)
        self.assertEqual(len(electrical_sample(0).pack()), 64)

    def test_round_trip_multiple_chunks(self):
        encoded = encode_record(
            {'session_id': 42, 'firmware': 'test'},
            ([sample(10), sample(11)], [sample(12)]),
        )
        reader = CalibrationRecordReader(io.BytesIO(encoded))
        chunks = list(reader.chunks())
        self.assertEqual(reader.metadata['session_id'], 42)
        self.assertEqual(len(chunks), 2)
        self.assertEqual(chunks[1].samples[0].sequence, 12)
        self.assertEqual(chunks[0].samples[0].pair_sequence, 3010)
        self.assertEqual(chunks[0].samples[0].pair_sample_skew_cycles, 400)
        self.assertAlmostEqual(chunks[0].samples[0].vbus, 24.0)

    def test_round_trip_electrical_fast_chunk(self):
        encoded = encode_record(
            {'session_id': 42},
            ([electrical_sample(0), electrical_sample(1)],),
        )
        chunks = list(CalibrationRecordReader(io.BytesIO(encoded)).chunks())
        self.assertEqual(chunks[0].record_type, 2)
        self.assertEqual(chunks[0].samples[1].sequence, 1)
        self.assertAlmostEqual(
            chunks[0].samples[1].electrical_phase, 2.1, places=6)

    def test_mixed_record_type_chunk_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'cannot mix'):
            encode_record(
                {'session_id': 42},
                ([sample(0), electrical_sample(1)],),
            )

    def test_payload_corruption_is_detected(self):
        encoded = bytearray(encode_record({'session_id': 42}, ([sample(0)],)))
        encoded[-1] ^= 0x01
        reader = CalibrationRecordReader(io.BytesIO(encoded))
        with self.assertRaisesRegex(ValueError, 'CRC mismatch'):
            list(reader.chunks())

    def test_sequence_gap_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'sequence gap'):
            encode_record({'session_id': 42}, ([sample(0)], [sample(2)]))

    def test_stream_decoder_handles_fragmentation_and_stdout(self):
        decoder = CalibrationStreamDecoder()
        frame = stream_frame(electrical_sample(7))
        self.assertEqual(decoder.feed(b'normal log line\n' + frame[:13]), [])
        records = decoder.feed(frame[13:])
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0].sample.sequence, 7)
        self.assertGreater(decoder.discarded_bytes, 0)

    def test_stream_decoder_recovers_after_bad_payload(self):
        decoder = CalibrationStreamDecoder()
        bad = bytearray(stream_frame(electrical_sample(1)))
        bad[-1] ^= 0x80
        good = stream_frame(electrical_sample(2))
        records = decoder.feed(bytes(bad) + b'noise' + good)
        self.assertEqual([record.sample.sequence for record in records], [2])
        self.assertEqual(decoder.bad_payloads, 1)


if __name__ == '__main__':
    unittest.main()
