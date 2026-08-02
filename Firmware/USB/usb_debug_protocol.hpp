#ifndef ODRIVE_USB_DEBUG_PROTOCOL_HPP
#define ODRIVE_USB_DEBUG_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "MotorControl/scope_capture.hpp"

namespace odrive::usb {

constexpr uint16_t kFrameMagic = 0x4F44u;
constexpr uint8_t kProtocolVersion = 4u;
constexpr uint8_t kSchemaVersion = 4u;
constexpr size_t kHeaderSize = 40u;
constexpr size_t kTrailerSize = 4u;
constexpr size_t kMaxPayloadSize = 512u;
constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kTrailerSize;

enum class MessageType : uint8_t {
    HELLO = 1u,
    CAPABILITIES = 2u,
    COMMAND = 3u,
    COMMAND_RESULT = 4u,
    FAULT_EVENT = 5u,
    STATE_EVENT = 6u,
    LOG_RECORD = 7u,
    SCOPE_RECORD = 8u,
    STATS = 9u,
    CALIBRATION_DATA = 10u,
    SCOPE_CONFIG = 11u,
    SCOPE_ARM = 12u,
    SCOPE_STATUS = 13u,
    SCOPE_DATA_BATCH = 14u,
    SCOPE_STOP = 15u,
    CRASH_REPORT = 16u,
    CRASH_ACK = 17u,
    TEST_SESSION_BEGIN = 18u,
    TEST_SESSION_STATUS = 19u,
    TEST_SESSION_KEEPALIVE = 20u,
    TEST_SESSION_STOP = 21u,
    TEST_SESSION_END = 22u,
};

enum FrameFlags : uint8_t {
    FRAME_RESPONSE = 1u << 0,
    FRAME_ERROR = 1u << 1,
    FRAME_DROPPED = 1u << 2,
};

struct Frame {
    uint8_t protocol_version = kProtocolVersion;
    uint8_t schema_version = kSchemaVersion;
    MessageType message_type = MessageType::HELLO;
    uint8_t flags = 0u;
    uint32_t sequence = 0u;
    uint32_t request_id = 0u;
    uint16_t payload_length = 0u;
    std::array<uint8_t, 8> build_id = {};
    std::array<uint8_t, 16> manifest_identity = {};
    std::array<uint8_t, kMaxPayloadSize> payload = {};
};

enum class DecodeStatus : uint8_t {
    OK = 0u,
    NULL_ARGUMENT = 1u,
    TOO_SHORT = 2u,
    BAD_MAGIC = 3u,
    BAD_LENGTH = 4u,
    CRC_ERROR = 5u,
    UNKNOWN_VERSION = 6u,
    UNKNOWN_MESSAGE = 7u,
};

struct ParserStats {
    uint32_t bytes_seen = 0u;
    uint32_t frames_decoded = 0u;
    uint32_t noise_bytes = 0u;
    uint32_t resynchronizations = 0u;
    uint32_t length_errors = 0u;
    uint32_t crc_errors = 0u;
    uint32_t unknown_versions = 0u;
    uint32_t unknown_messages = 0u;
    uint32_t dropped_frames = 0u;
};

using FrameCallback = void (*)(void* context, const Frame& frame);

uint32_t crc32(const uint8_t* data, size_t length);
size_t encode_frame(const Frame& frame, uint8_t* output, size_t capacity);
size_t encode_device_frame(MessageType message_type, uint8_t flags,
                           uint32_t sequence, uint32_t request_id,
                           const uint8_t* payload, size_t payload_length,
                           uint8_t* output, size_t capacity);
size_t encode_device_frame_parts(MessageType message_type, uint8_t flags,
                                 uint32_t sequence, uint32_t request_id,
                                 const uint8_t* prefix, size_t prefix_length,
                                 const uint8_t* payload, size_t payload_length,
                                 uint8_t* output, size_t capacity);
DecodeStatus decode_frame(const uint8_t* data, size_t length, Frame* frame);
bool is_known_message_type(uint8_t value);
void fill_device_identity(Frame* frame);

class StreamParser {
public:
    StreamParser() = default;

    size_t feed(const uint8_t* data, size_t length,
                void* callback_context, FrameCallback callback);
    void reset();
    const ParserStats& stats() const { return stats_; }

private:
    void consume_byte(uint8_t value, void* callback_context,
                      FrameCallback callback);
    void restart_scan();
    bool has_magic_prefix() const;

    std::array<uint8_t, kMaxFrameSize> frame_ = {};
    size_t frame_size_ = 0u;
    size_t expected_size_ = 0u;
    ParserStats stats_{};
};

struct CommandPayload {
    uint8_t command_type = 0u;
    uint8_t operation = 0u;
    uint32_t arg0 = 0u;
};

struct CommandResultPayload {
    uint8_t status = 0u;
    uint32_t state_epoch = 0u;
    uint16_t reason = 0u;
};

bool encode_scope_config_payload(const odrive::scope::ScopeConfig& config,
                                 uint8_t* output, size_t capacity,
                                 size_t* written);
bool decode_scope_config_payload(const uint8_t* data, size_t length,
                                 odrive::scope::ScopeConfig* config);
bool encode_scope_arm_payload(uint8_t action, uint8_t* output, size_t capacity,
                              size_t* written);
bool decode_scope_arm_payload(const uint8_t* data, size_t length,
                              uint8_t* action);
bool encode_scope_status_payload(const odrive::scope::ScopeStatus& status,
                                 uint8_t* output, size_t capacity,
                                 size_t* written);
bool encode_scope_batch_payload(const odrive::scope::ScopeBatch& batch,
                                uint8_t* output, size_t capacity,
                                size_t* written);

bool encode_command_payload(const CommandPayload& command,
                            uint8_t* output, size_t capacity,
                            size_t* written);
bool decode_command_payload(const uint8_t* data, size_t length,
                            CommandPayload* command);
bool encode_command_result_payload(const CommandResultPayload& result,
                                   uint8_t* output, size_t capacity,
                                   size_t* written);
bool decode_command_result_payload(const uint8_t* data, size_t length,
                                   CommandResultPayload* result);

}  // namespace odrive::usb

#endif
