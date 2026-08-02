#ifndef ODRIVE_SCOPE_CHANNELS_GENERATED_HPP
#define ODRIVE_SCOPE_CHANNELS_GENERATED_HPP

#include <array>
#include <cstdint>
#include <cstring>

namespace odrive::scope::generated {

enum class ScopeWireDataType : uint8_t { U8 = 0, U32 = 1, F32 = 2 };
struct ScopeChannelDefinition { uint16_t id; const char* name; ScopeWireDataType wire_type; const char* unit; float display_min; float display_max; uint32_t max_sample_rate_hz; bool threshold_allowed; const char* source; };
struct ScopeSampleContext {
    uint32_t control_sequence = {};
    uint32_t timestamp_cycles = {};
    uint32_t state_epoch = {};
    uint8_t safety_state = {};
    uint8_t operation = {};
    float phase = {};
    float phase_velocity = {};
    float position = {};
    float velocity = {};
    float id_measured = {};
    float iq_measured = {};
    float id_setpoint = {};
    float iq_setpoint = {};
    float torque_setpoint = {};
    float controller_output = {};
};
struct ScopeChannelValue { uint32_t raw = 0; };
inline constexpr size_t kChannelCount = 15u;
inline constexpr size_t kMaxCaptureChannels = 8u;
inline constexpr size_t kMaxPreSamples = 64u;
inline constexpr size_t kMaxPostSamples = 128u;
inline constexpr size_t kMaxBatchSamples = 6u;
inline constexpr std::array<ScopeChannelDefinition, kChannelCount> kChannels = {{
    {1u, "control_sequence", ScopeWireDataType::U32, "count", 0.0f, 100000.0f, 10000u, false, "control_sequence"},
    {2u, "timestamp_cycles", ScopeWireDataType::U32, "cycles", 0.0f, 4.2949673e+09f, 10000u, false, "timestamp_cycles"},
    {3u, "state_epoch", ScopeWireDataType::U32, "epoch", 0.0f, 4.2949673e+09f, 1000u, false, "state_epoch"},
    {4u, "SafetyState", ScopeWireDataType::U8, "enum", 0.0f, 7.0f, 1000u, true, "safety_state"},
    {5u, "Operation", ScopeWireDataType::U8, "enum", 0.0f, 3.0f, 1000u, true, "operation"},
    {6u, "phase", ScopeWireDataType::F32, "rad", -3.1415927f, 3.1415927f, 10000u, true, "phase"},
    {7u, "phase_velocity", ScopeWireDataType::F32, "rad/s", -10000.0f, 10000.0f, 10000u, true, "phase_velocity"},
    {8u, "position", ScopeWireDataType::F32, "rev", -1000.0f, 1000.0f, 10000u, true, "position"},
    {9u, "velocity", ScopeWireDataType::F32, "rev/s", -1000.0f, 1000.0f, 10000u, true, "velocity"},
    {10u, "Id measured", ScopeWireDataType::F32, "A", -50.0f, 50.0f, 10000u, true, "id_measured"},
    {11u, "Iq measured", ScopeWireDataType::F32, "A", -50.0f, 50.0f, 10000u, true, "iq_measured"},
    {12u, "Id setpoint", ScopeWireDataType::F32, "A", -50.0f, 50.0f, 10000u, true, "id_setpoint"},
    {13u, "Iq setpoint", ScopeWireDataType::F32, "A", -50.0f, 50.0f, 10000u, true, "iq_setpoint"},
    {14u, "torque setpoint", ScopeWireDataType::F32, "Nm", -50.0f, 50.0f, 10000u, true, "torque_setpoint"},
    {15u, "controller output", ScopeWireDataType::F32, "command", -1.0f, 1.0f, 10000u, true, "controller_output"},
}};

inline const ScopeChannelDefinition* channel(uint16_t id) {
    for (const auto& item : kChannels) if (item.id == id) return &item;
    return nullptr;
}

inline bool read_channel(const ScopeSampleContext& sample, uint16_t id, ScopeChannelValue* output) {
    if (output == nullptr) return false;
    switch (id) {
        case 1u: output->raw = static_cast<uint32_t>(sample.control_sequence); return true;
        case 2u: output->raw = static_cast<uint32_t>(sample.timestamp_cycles); return true;
        case 3u: output->raw = static_cast<uint32_t>(sample.state_epoch); return true;
        case 4u: output->raw = static_cast<uint32_t>(sample.safety_state); return true;
        case 5u: output->raw = static_cast<uint32_t>(sample.operation); return true;
        case 6u: std::memcpy(&output->raw, &sample.phase, sizeof(output->raw)); return true;
        case 7u: std::memcpy(&output->raw, &sample.phase_velocity, sizeof(output->raw)); return true;
        case 8u: std::memcpy(&output->raw, &sample.position, sizeof(output->raw)); return true;
        case 9u: std::memcpy(&output->raw, &sample.velocity, sizeof(output->raw)); return true;
        case 10u: std::memcpy(&output->raw, &sample.id_measured, sizeof(output->raw)); return true;
        case 11u: std::memcpy(&output->raw, &sample.iq_measured, sizeof(output->raw)); return true;
        case 12u: std::memcpy(&output->raw, &sample.id_setpoint, sizeof(output->raw)); return true;
        case 13u: std::memcpy(&output->raw, &sample.iq_setpoint, sizeof(output->raw)); return true;
        case 14u: std::memcpy(&output->raw, &sample.torque_setpoint, sizeof(output->raw)); return true;
        case 15u: std::memcpy(&output->raw, &sample.controller_output, sizeof(output->raw)); return true;
        default: return false;
    }
}

}  // namespace odrive::scope::generated

#endif
