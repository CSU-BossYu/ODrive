#!/usr/bin/env python3
"""Generate the C++, Python and TypeScript ScopeChannel definitions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


CPP_TYPES = {"u8": "U8", "u32": "U32", "f32": "F32"}
TS_TYPES = {"u8": "number", "u32": "number", "f32": "number"}


def cpp_float(value: object) -> str:
    text = format(float(value), ".9g")
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def load(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    ids = [item["id"] for item in data["channels"]]
    if ids != sorted(ids) or len(ids) != len(set(ids)):
        raise ValueError("Scope channel IDs must be unique and sorted")
    required = {"id", "name", "wire_type", "unit", "display_min",
                "display_max", "max_sample_rate_hz", "threshold_allowed",
                "source"}
    for item in data["channels"]:
        if set(item) < required or item["wire_type"] not in CPP_TYPES:
            raise ValueError(f"invalid channel definition: {item}")
    return data


def cpp(data: dict) -> str:
    lines = [
        "#ifndef ODRIVE_SCOPE_CHANNELS_GENERATED_HPP",
        "#define ODRIVE_SCOPE_CHANNELS_GENERATED_HPP",
        "",
        "#include <array>",
        "#include <cstdint>",
        "#include <cstring>",
        "",
        "namespace odrive::scope::generated {",
        "",
        "enum class ScopeWireDataType : uint8_t { U8 = 0, U32 = 1, F32 = 2 };",
        "struct ScopeChannelDefinition { uint16_t id; const char* name; ScopeWireDataType wire_type; const char* unit; float display_min; float display_max; uint32_t max_sample_rate_hz; bool threshold_allowed; const char* source; };",
        "struct ScopeSampleContext {",
    ]
    for item in data["channels"]:
        typ = {"u8": "uint8_t", "u32": "uint32_t", "f32": "float"}[item["wire_type"]]
        lines.append(f"    {typ} {item['source']} = {{}};")
    lines += [
        "};",
        "struct ScopeChannelValue { uint32_t raw = 0; };",
        f"inline constexpr size_t kChannelCount = {len(data['channels'])}u;",
        f"inline constexpr size_t kMaxCaptureChannels = {data['max_capture_channels']}u;",
        f"inline constexpr size_t kMaxPreSamples = {data['max_pre_samples']}u;",
        f"inline constexpr size_t kMaxPostSamples = {data['max_post_samples']}u;",
        f"inline constexpr size_t kMaxBatchSamples = {data['max_batch_samples']}u;",
        "inline constexpr std::array<ScopeChannelDefinition, kChannelCount> kChannels = {{",
    ]
    for item in data["channels"]:
        lines.append(
            f"    {{{item['id']}u, \"{item['name']}\", ScopeWireDataType::{CPP_TYPES[item['wire_type']]}, \"{item['unit']}\", {cpp_float(item['display_min'])}, {cpp_float(item['display_max'])}, {item['max_sample_rate_hz']}u, {'true' if item['threshold_allowed'] else 'false'}, \"{item['source']}\"}},"
        )
    lines += ["}};", "", "inline const ScopeChannelDefinition* channel(uint16_t id) {",
              "    for (const auto& item : kChannels) if (item.id == id) return &item;",
              "    return nullptr;", "}", "",
              "inline bool read_channel(const ScopeSampleContext& sample, uint16_t id, ScopeChannelValue* output) {"]
    lines += ["    if (output == nullptr) return false;", "    switch (id) {"]
    for item in data["channels"]:
        source = f"sample.{item['source']}"
        if item["wire_type"] == "f32":
            value = f"std::memcpy(&output->raw, &{source}, sizeof(output->raw));"
        else:
            value = f"output->raw = static_cast<uint32_t>({source});"
        lines += [f"        case {item['id']}u: {value} return true;"]
    lines += ["        default: return false;", "    }", "}", "",
              "}  // namespace odrive::scope::generated", "", "#endif", ""]
    return "\n".join(lines)


def python(data: dict) -> str:
    lines = ['"""Generated from docs/scope_channel_schema.json."""', "",
             "from dataclasses import dataclass", "", "@dataclass(frozen=True)",
             "class ScopeChannel:", "    id: int", "    name: str", "    wire_type: str",
             "    unit: str", "    display_min: float", "    display_max: float",
             "    max_sample_rate_hz: int", "    threshold_allowed: bool", "    source: str", "",
             f"MAX_CAPTURE_CHANNELS = {data['max_capture_channels']}",
             f"MAX_PRE_SAMPLES = {data['max_pre_samples']}",
             f"MAX_POST_SAMPLES = {data['max_post_samples']}",
             f"MAX_BATCH_SAMPLES = {data['max_batch_samples']}", "",
             "SCOPE_CHANNELS = ("]
    for item in data["channels"]:
        lines.append(f"    ScopeChannel({item['id']}, {item['name']!r}, {item['wire_type']!r}, {item['unit']!r}, {item['display_min']!r}, {item['display_max']!r}, {item['max_sample_rate_hz']}, {item['threshold_allowed']}, {item['source']!r}),")
    lines += [")", "CHANNEL_BY_ID = {item.id: item for item in SCOPE_CHANNELS}", ""]
    return "\n".join(lines)


def typescript(data: dict) -> str:
    lines = ["// Generated from docs/scope_channel_schema.json; do not edit by hand.",
             "export type ScopeWireDataType = 'u8' | 'u32' | 'f32'", "export interface ScopeChannel {",
             "  id: number; name: string; wireType: ScopeWireDataType; unit: string;",
             "  displayMin: number; displayMax: number; maxSampleRateHz: number;",
             "  thresholdAllowed: boolean; source: string;", "}",
             f"export const MAX_CAPTURE_CHANNELS = {data['max_capture_channels']} as const",
             f"export const MAX_PRE_SAMPLES = {data['max_pre_samples']} as const",
             f"export const MAX_POST_SAMPLES = {data['max_post_samples']} as const",
             f"export const MAX_BATCH_SAMPLES = {data['max_batch_samples']} as const", "",
             "export const SCOPE_CHANNELS: readonly ScopeChannel[] = ["]
    for item in data["channels"]:
        lines.append(f"  {{ id: {item['id']}, name: {item['name']!r}, wireType: {item['wire_type']!r}, unit: {item['unit']!r}, displayMin: {item['display_min']}, displayMax: {item['display_max']}, maxSampleRateHz: {item['max_sample_rate_hz']}, thresholdAllowed: {str(item['threshold_allowed']).lower()}, source: {item['source']!r} }},")
    lines += ["]", "export const SCOPE_CHANNEL_BY_ID = Object.fromEntries(SCOPE_CHANNELS.map((item) => [item.id, item])) as Record<number, ScopeChannel>", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--schema", required=True)
    parser.add_argument("--cpp-output", required=True)
    parser.add_argument("--python-output", required=True)
    parser.add_argument("--typescript-output", required=True)
    args = parser.parse_args()
    data = load(Path(args.schema))
    outputs = [(Path(args.cpp_output), cpp(data)), (Path(args.python_output), python(data)), (Path(args.typescript_output), typescript(data))]
    for path, content in outputs:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
