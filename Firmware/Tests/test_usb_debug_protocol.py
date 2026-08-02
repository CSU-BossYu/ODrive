import binascii
import json
import random
import unittest
from pathlib import Path


MAGIC = b"\x44\x4f"
HEADER_SIZE = 40
MAX_PAYLOAD = 512
COMMAND = 3


def crc32(data):
    return binascii.crc32(data) & 0xFFFFFFFF


def decode_frame(data):
    if len(data) < HEADER_SIZE + 4:
        raise ValueError("short")
    if data[:2] != MAGIC:
        raise ValueError("magic")
    payload_length = int.from_bytes(data[14:16], "little")
    if payload_length > MAX_PAYLOAD:
        raise ValueError("length")
    expected = HEADER_SIZE + payload_length + 4
    if len(data) != expected:
        raise ValueError("length")
    if int.from_bytes(data[-4:], "little") != crc32(data[:-4]):
        raise ValueError("crc")
    if data[2] != 4 or data[3] != 4:
        raise ValueError("version")
    if not 1 <= data[4] <= 22:
        raise ValueError("message")
    return {
        "type": data[4],
        "flags": data[5],
        "sequence": int.from_bytes(data[6:10], "little"),
        "request_id": int.from_bytes(data[10:14], "little"),
        "payload": data[40:-4],
        "build_id": data[16:24],
        "manifest": data[24:40],
    }


def stream_frames(chunks):
    buffer = bytearray()
    frames = []
    for chunk in chunks:
        buffer.extend(chunk)
        while True:
            start = buffer.find(MAGIC)
            if start < 0:
                buffer[:] = buffer[-1:] if buffer[-1:] == MAGIC[:1] else b""
                break
            if start:
                del buffer[:start]
            if len(buffer) < HEADER_SIZE:
                break
            length = int.from_bytes(buffer[14:16], "little")
            if length > MAX_PAYLOAD:
                del buffer[0:1]
                continue
            total = HEADER_SIZE + length + 4
            if len(buffer) < total:
                break
            candidate = bytes(buffer[:total])
            del buffer[:total]
            try:
                frames.append(decode_frame(candidate))
            except ValueError:
                continue
    return frames


GOLDEN_HEX = (
    "444f0404030004030201d4c3b2a10800"
    "1011121314151617"
    "202122232425262728292a2b2c2d2e2f"
    "0101000044332211"
    "ba079694"
)


class UsbDebugProtocolTest(unittest.TestCase):
    def test_authoritative_schema_covers_every_message_payload(self):
        schema_path = Path(__file__).resolve().parents[2] / "docs" / \
            "usb_debug_protocol_schema.json"
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        self.assertEqual(schema["crc"]["polynomial_reflected"], "0xEDB88320")
        self.assertIn("usb_packet_fragmentation", schema["transport"])
        self.assertEqual(set(schema["message_types"]), set(schema["payloads"]))
        self.assertEqual(schema["payloads"]["SCOPE_RECORD"]["length"], 56)
        self.assertEqual(schema["payloads"]["FAULT_EVENT"]["length"], 44)
        self.assertEqual(schema["payloads"]["STATS"]["response_length"], 44)

    def test_cpp_golden_vector_is_python_decodable(self):
        frame = decode_frame(bytes.fromhex(GOLDEN_HEX))
        self.assertEqual(frame["type"], COMMAND)
        self.assertEqual(frame["sequence"], 0x01020304)
        self.assertEqual(frame["request_id"], 0xA1B2C3D4)
        self.assertEqual(frame["payload"], bytes.fromhex("0101000044332211"))

    def test_single_byte_arbitrary_sticky_and_noise(self):
        raw = bytes.fromhex(GOLDEN_HEX)
        second = bytearray(raw)
        second[6:10] = (9).to_bytes(4, "little")
        second[10:14] = (10).to_bytes(4, "little")
        second[-4:] = crc32(second[:-4]).to_bytes(4, "little")
        joined = b"noise" + raw + bytes(second)
        self.assertEqual(len(stream_frames(joined[i:i + 1]
                                           for i in range(len(joined)))), 2)
        chunks = []
        rng = random.Random(0x504834)
        index = 0
        while index < len(joined):
            size = min(rng.randint(1, 11), len(joined) - index)
            chunks.append(joined[index:index + size])
            index += size
        self.assertEqual(len(stream_frames(chunks)), 2)

    def test_crc_error_and_oversize_recover(self):
        raw = bytearray.fromhex(GOLDEN_HEX)
        corrupt = bytearray(raw)
        corrupt[20] ^= 0x80
        oversize = bytearray(raw[:HEADER_SIZE])
        oversize[14:16] = (MAX_PAYLOAD + 1).to_bytes(2, "little")
        self.assertEqual(len(stream_frames([bytes(corrupt) + bytes(oversize) + raw])), 1)

    def test_unknown_version_and_type_are_not_accepted(self):
        raw = bytearray.fromhex(GOLDEN_HEX)
        raw[2] = 99
        raw[-4:] = crc32(raw[:-4]).to_bytes(4, "little")
        with self.assertRaises(ValueError):
            decode_frame(bytes(raw))
        raw = bytearray.fromhex(GOLDEN_HEX)
        raw[4] = 200
        raw[-4:] = crc32(raw[:-4]).to_bytes(4, "little")
        with self.assertRaises(ValueError):
            decode_frame(bytes(raw))

    def test_deterministic_fuzz_has_a_bound(self):
        rng = random.Random(0x504834)
        chunks = []
        for _ in range(2000):
            size = rng.randint(0, 17)
            chunks.append(bytes(rng.getrandbits(8) for _ in range(size)))
        # The assertion is deliberately about termination and bounded input;
        # random bytes are not required to form a valid command frame.
        frames = stream_frames(chunks)
        self.assertLessEqual(len(frames), 2000)


if __name__ == "__main__":
    unittest.main()
