#!/usr/bin/env python3
"""Generate product CAN IDs and management codecs from one JSON schema."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    messages = data["messages"]
    ids = [int(item["id"]) for item in messages]
    names = [str(item["name"]) for item in messages]
    if len(ids) != len(set(ids)) or len(names) != len(set(names)):
        raise ValueError("CAN message IDs and names must be unique")
    if any(not 0 <= value <= 31 for value in ids):
        raise ValueError("CAN command IDs must fit five bits")
    required = {"id", "name", "direction", "dlc", "class", "status"}
    if any(not required <= set(item) for item in messages):
        raise ValueError("CAN message entry is incomplete")
    return data


def cpp(data: dict) -> str:
    rows = "\n".join(
        f"    {item['name']} = 0x{int(item['id']):02X}u,"
        for item in data["messages"])
    version = int(data["protocol_version_u32"])
    return f'''#ifndef ODRIVE_CAN_PRODUCT_SCHEMA_GENERATED_HPP
#define ODRIVE_CAN_PRODUCT_SCHEMA_GENERATED_HPP

#include <cstdint>

namespace odrive::can::product {{

inline constexpr uint32_t kProtocolVersion = 0x{version:08X}u;
inline constexpr uint8_t kNodeIdBits = {int(data['node_id_bits'])}u;
inline constexpr uint8_t kCommandIdBits = {int(data['command_id_bits'])}u;

enum class MessageId : uint8_t {{
{rows}
}};

struct ManagementCommand {{
    uint16_t request_id = 0u;
    uint8_t command_type = 0u;
    uint8_t operation = 0u;
    uint32_t arg0 = 0u;
}};

struct CommandAck {{
    uint16_t request_id = 0u;
    uint8_t status = 0u;
    uint16_t state_epoch_low = 0u;
    uint16_t reason = 0u;
}};

inline ManagementCommand decode_management_command(const uint8_t* data) {{
    return {{
        static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8u)),
        data[2], data[3],
        static_cast<uint32_t>(data[4]) |
            (static_cast<uint32_t>(data[5]) << 8u) |
            (static_cast<uint32_t>(data[6]) << 16u) |
            (static_cast<uint32_t>(data[7]) << 24u)}};
}}

inline void encode_command_ack(const CommandAck& value, uint8_t* data) {{
    data[0] = static_cast<uint8_t>(value.request_id);
    data[1] = static_cast<uint8_t>(value.request_id >> 8u);
    data[2] = value.status;
    data[3] = 0u;
    data[4] = static_cast<uint8_t>(value.state_epoch_low);
    data[5] = static_cast<uint8_t>(value.state_epoch_low >> 8u);
    data[6] = static_cast<uint8_t>(value.reason);
    data[7] = static_cast<uint8_t>(value.reason >> 8u);
}}

}}  // namespace odrive::can::product

#endif
'''


def python(data: dict) -> str:
    rows = "\n".join(
        f"    {item['name']} = 0x{int(item['id']):02X}"
        for item in data["messages"])
    version = int(data["protocol_version_u32"])
    return f'''"""Generated from docs/can_product_protocol_schema.json."""

from enum import IntEnum
import struct

PROTOCOL_VERSION = 0x{version:08X}

class ProductCanMessageId(IntEnum):
{rows}

def encode_management_command(request_id: int, command_type: int,
                              operation: int = 0, arg0: int = 0) -> bytes:
    if not 0 < request_id <= 0xFFFF:
        raise ValueError("CAN management request_id must be 1..65535")
    return struct.pack("<HBBI", request_id, command_type, operation,
                       arg0 & 0xFFFFFFFF)

def decode_command_ack(data: bytes) -> dict[str, int]:
    if len(data) != 8:
        raise ValueError("CAN command ACK must be 8 bytes")
    request_id, status, reserved, epoch, reason = struct.unpack("<HBBHH", data)
    if reserved != 0:
        raise ValueError("CAN command ACK reserved byte is non-zero")
    return {{"request_id": request_id, "status": status,
            "state_epoch_low": epoch, "reason": reason}}
'''


def typescript(data: dict) -> str:
    rows = "\n".join(
        f"  {item['name']} = 0x{int(item['id']):02X},"
        for item in data["messages"])
    return f'''// Generated from docs/can_product_protocol_schema.json; do not edit.
export const CAN_PRODUCT_PROTOCOL_VERSION = 0x{int(data['protocol_version_u32']):08X} as const
export enum ProductCanMessageId {{
{rows}
}}
export interface CanCommandAck {{
  requestId: number
  status: number
  stateEpochLow: number
  reason: number
}}
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--schema", required=True)
    parser.add_argument("--cpp-output", required=True)
    parser.add_argument("--python-output", required=True)
    parser.add_argument("--test-python-output")
    parser.add_argument("--typescript-output", required=True)
    args = parser.parse_args()
    data = load(Path(args.schema))
    outputs = [(Path(args.cpp_output), cpp(data)),
               (Path(args.python_output), python(data)),
               (Path(args.typescript_output), typescript(data))]
    if args.test_python_output:
        outputs.append((Path(args.test_python_output), python(data)))
    for path, text in outputs:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
