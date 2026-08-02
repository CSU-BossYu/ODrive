#include "usb_debug_protocol.hpp"

#include <algorithm>
#include <cstring>

#if __has_include("autogen/usb_debug_identity.hpp")
#include "autogen/usb_debug_identity.hpp"
#endif

namespace odrive::usb {

namespace {

constexpr uint8_t kMagicLow = static_cast<uint8_t>(kFrameMagic & 0xffu);
constexpr uint8_t kMagicHigh = static_cast<uint8_t>(kFrameMagic >> 8u);

void put_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8u);
}

void put_u32(uint8_t* output, uint32_t value) {
    for (size_t index = 0; index < 4u; ++index) {
        output[index] = static_cast<uint8_t>(value >> (8u * index));
    }
}

uint16_t get_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1]) << 8u;
}

uint32_t get_u32(const uint8_t* data) {
    uint32_t value = 0u;
    for (size_t index = 0; index < 4u; ++index) {
        value |= static_cast<uint32_t>(data[index]) << (8u * index);
    }
    return value;
}

bool is_valid_header_length(size_t length) {
    return length >= kHeaderSize + kTrailerSize;
}

}  // namespace

uint32_t crc32(const uint8_t* data, size_t length) {
    if (data == nullptr && length != 0u) {
        return 0u;
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint32_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1u) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

bool is_known_message_type(uint8_t value) {
    return value >= static_cast<uint8_t>(MessageType::HELLO) &&
           value <= static_cast<uint8_t>(MessageType::TEST_SESSION_END);
}

void fill_device_identity(Frame* frame) {
    if (frame == nullptr) {
        return;
    }
#if __has_include("autogen/usb_debug_identity.hpp")
    frame->build_id = odrive::usb::identity::kBuildId;
    frame->manifest_identity = odrive::usb::identity::kManifestIdentity;
#else
    frame->build_id = {0x50u, 0x48u, 0x34u, 0x2Du,
                       0x44u, 0x45u, 0x56u, 0x00u};
    frame->manifest_identity = {};
#endif
}

size_t encode_frame(const Frame& frame, uint8_t* output, size_t capacity) {
    if (output == nullptr || frame.payload_length > kMaxPayloadSize) {
        return 0u;
    }
    const size_t total = kHeaderSize + frame.payload_length + kTrailerSize;
    if (capacity < total) {
        return 0u;
    }

    put_u16(output, kFrameMagic);
    output[2] = frame.protocol_version;
    output[3] = frame.schema_version;
    output[4] = static_cast<uint8_t>(frame.message_type);
    output[5] = frame.flags;
    put_u32(output + 6u, frame.sequence);
    put_u32(output + 10u, frame.request_id);
    put_u16(output + 14u, frame.payload_length);
    std::memcpy(output + 16u, frame.build_id.data(), frame.build_id.size());
    std::memcpy(output + 24u, frame.manifest_identity.data(),
                frame.manifest_identity.size());
    if (frame.payload_length != 0u) {
        std::memcpy(output + kHeaderSize, frame.payload.data(),
                    frame.payload_length);
    }
    put_u32(output + kHeaderSize + frame.payload_length,
            crc32(output, kHeaderSize + frame.payload_length));
    return total;
}

size_t encode_device_frame(MessageType message_type, uint8_t flags,
                           uint32_t sequence, uint32_t request_id,
                           const uint8_t* payload, size_t payload_length,
                           uint8_t* output, size_t capacity) {
    return encode_device_frame_parts(message_type, flags, sequence, request_id,
                                     nullptr, 0u, payload, payload_length,
                                     output, capacity);
}

size_t encode_device_frame_parts(MessageType message_type, uint8_t flags,
                                 uint32_t sequence, uint32_t request_id,
                                 const uint8_t* prefix, size_t prefix_length,
                                 const uint8_t* payload, size_t payload_length,
                                 uint8_t* output, size_t capacity) {
    const size_t combined_length = prefix_length + payload_length;
    if (output == nullptr || combined_length > kMaxPayloadSize ||
        (prefix == nullptr && prefix_length != 0u) ||
        (payload == nullptr && payload_length != 0u)) {
        return 0u;
    }
    const size_t total = kHeaderSize + combined_length + kTrailerSize;
    if (capacity < total) {
        return 0u;
    }

    put_u16(output, kFrameMagic);
    output[2] = kProtocolVersion;
    output[3] = kSchemaVersion;
    output[4] = static_cast<uint8_t>(message_type);
    output[5] = flags;
    put_u32(output + 6u, sequence);
    put_u32(output + 10u, request_id);
    put_u16(output + 14u, static_cast<uint16_t>(combined_length));
#if __has_include("autogen/usb_debug_identity.hpp")
    std::memcpy(output + 16u, odrive::usb::identity::kBuildId.data(),
                odrive::usb::identity::kBuildId.size());
    std::memcpy(output + 24u,
                odrive::usb::identity::kManifestIdentity.data(),
                odrive::usb::identity::kManifestIdentity.size());
#else
    static constexpr std::array<uint8_t, 8> kDevelopmentBuildId = {
        0x50u, 0x48u, 0x34u, 0x2Du, 0x44u, 0x45u, 0x56u, 0x00u};
    std::memcpy(output + 16u, kDevelopmentBuildId.data(),
                kDevelopmentBuildId.size());
    std::memset(output + 24u, 0, 16u);
#endif
    if (prefix_length != 0u) {
        std::memcpy(output + kHeaderSize, prefix, prefix_length);
    }
    if (payload_length != 0u) {
        std::memcpy(output + kHeaderSize + prefix_length, payload,
                    payload_length);
    }
    put_u32(output + kHeaderSize + combined_length,
            crc32(output, kHeaderSize + combined_length));
    return total;
}

DecodeStatus decode_frame(const uint8_t* data, size_t length, Frame* frame) {
    if (data == nullptr || frame == nullptr) {
        return DecodeStatus::NULL_ARGUMENT;
    }
    if (!is_valid_header_length(length)) {
        return DecodeStatus::TOO_SHORT;
    }
    if (get_u16(data) != kFrameMagic) {
        return DecodeStatus::BAD_MAGIC;
    }
    const uint16_t payload_length = get_u16(data + 14u);
    if (payload_length > kMaxPayloadSize ||
        length != kHeaderSize + payload_length + kTrailerSize) {
        return DecodeStatus::BAD_LENGTH;
    }
    if (get_u32(data + kHeaderSize + payload_length) !=
        crc32(data, kHeaderSize + payload_length)) {
        return DecodeStatus::CRC_ERROR;
    }
    if (data[2] != kProtocolVersion || data[3] != kSchemaVersion) {
        return DecodeStatus::UNKNOWN_VERSION;
    }
    if (!is_known_message_type(data[4])) {
        return DecodeStatus::UNKNOWN_MESSAGE;
    }

    frame->protocol_version = data[2];
    frame->schema_version = data[3];
    frame->message_type = static_cast<MessageType>(data[4]);
    frame->flags = data[5];
    frame->sequence = get_u32(data + 6u);
    frame->request_id = get_u32(data + 10u);
    frame->payload_length = payload_length;
    std::memcpy(frame->build_id.data(), data + 16u, frame->build_id.size());
    std::memcpy(frame->manifest_identity.data(), data + 24u,
                frame->manifest_identity.size());
    if (payload_length != 0u) {
        std::memcpy(frame->payload.data(), data + kHeaderSize,
                    payload_length);
    }
    return DecodeStatus::OK;
}

bool StreamParser::has_magic_prefix() const {
    return frame_size_ >= 2u && frame_[0] == kMagicLow &&
           frame_[1] == kMagicHigh;
}

void StreamParser::restart_scan() {
    ++stats_.resynchronizations;
    size_t start = 1u;
    while (start + 1u < frame_size_ &&
           !(frame_[start] == kMagicLow && frame_[start + 1u] == kMagicHigh)) {
        ++start;
    }
    if (start + 1u < frame_size_) {
        const size_t remaining = frame_size_ - start;
        std::memmove(frame_.data(), frame_.data() + start, remaining);
        frame_size_ = remaining;
        expected_size_ = 0u;
    } else if (frame_size_ != 0u && frame_[frame_size_ - 1u] == kMagicLow) {
        frame_[0] = kMagicLow;
        frame_size_ = 1u;
        expected_size_ = 0u;
    } else {
        frame_size_ = 0u;
        expected_size_ = 0u;
    }
}

void StreamParser::consume_byte(uint8_t value, void* callback_context,
                                FrameCallback callback) {
    ++stats_.bytes_seen;
    if (frame_size_ == 0u) {
        if (value == kMagicLow) {
            frame_[frame_size_++] = value;
        } else {
            ++stats_.noise_bytes;
        }
        return;
    }
    if (frame_size_ == 1u && value != kMagicHigh) {
        ++stats_.noise_bytes;
        if (value == kMagicLow) {
            frame_[0] = value;
            return;
        }
        frame_size_ = 0u;
        return;
    }
    if (frame_size_ >= frame_.size()) {
        ++stats_.dropped_frames;
        restart_scan();
        return;
    }
    frame_[frame_size_++] = value;
    if (frame_size_ == kHeaderSize) {
        const uint16_t payload_length = get_u16(frame_.data() + 14u);
        if (payload_length > kMaxPayloadSize) {
            ++stats_.length_errors;
            restart_scan();
            return;
        }
        expected_size_ = kHeaderSize + payload_length + kTrailerSize;
    }
    if (expected_size_ != 0u && frame_size_ == expected_size_) {
        Frame decoded;
        const DecodeStatus status = decode_frame(
            frame_.data(), frame_size_, &decoded);
        if (status == DecodeStatus::OK) {
            ++stats_.frames_decoded;
            if (callback != nullptr) {
                callback(callback_context, decoded);
            }
            frame_size_ = 0u;
            expected_size_ = 0u;
        } else {
            switch (status) {
                case DecodeStatus::BAD_LENGTH: ++stats_.length_errors; break;
                case DecodeStatus::CRC_ERROR: ++stats_.crc_errors; break;
                case DecodeStatus::UNKNOWN_VERSION:
                    ++stats_.unknown_versions;
                    break;
                case DecodeStatus::UNKNOWN_MESSAGE:
                    ++stats_.unknown_messages;
                    break;
                default: ++stats_.dropped_frames; break;
            }
            restart_scan();
        }
    }
}

size_t StreamParser::feed(const uint8_t* data, size_t length,
                          void* callback_context, FrameCallback callback) {
    if (data == nullptr && length != 0u) {
        return 0u;
    }
    for (size_t index = 0; index < length; ++index) {
        consume_byte(data[index], callback_context, callback);
    }
    return length;
}

void StreamParser::reset() {
    frame_size_ = 0u;
    expected_size_ = 0u;
    stats_ = {};
}

bool encode_command_payload(const CommandPayload& command,
                            uint8_t* output, size_t capacity,
                            size_t* written) {
    if (output == nullptr || written == nullptr || capacity < 8u) {
        return false;
    }
    output[0] = command.command_type;
    output[1] = command.operation;
    put_u16(output + 2u, 0u);
    put_u32(output + 4u, command.arg0);
    *written = 8u;
    return true;
}

bool decode_command_payload(const uint8_t* data, size_t length,
                            CommandPayload* command) {
    if (data == nullptr || command == nullptr || length != 8u ||
        data[2] != 0u || data[3] != 0u) {
        return false;
    }
    command->command_type = data[0];
    command->operation = data[1];
    command->arg0 = get_u32(data + 4u);
    return true;
}

bool encode_command_result_payload(const CommandResultPayload& result,
                                   uint8_t* output, size_t capacity,
                                   size_t* written) {
    if (output == nullptr || written == nullptr || capacity < 8u) {
        return false;
    }
    output[0] = result.status;
    output[1] = 0u;
    put_u32(output + 2u, result.state_epoch);
    put_u16(output + 6u, result.reason);
    *written = 8u;
    return true;
}

bool decode_command_result_payload(const uint8_t* data, size_t length,
                                   CommandResultPayload* result) {
    if (data == nullptr || result == nullptr || length != 8u ||
        data[1] != 0u) {
        return false;
    }
    result->status = data[0];
    result->state_epoch = get_u32(data + 2u);
    result->reason = get_u16(data + 6u);
    return true;
}

bool encode_scope_config_payload(const odrive::scope::ScopeConfig& config,
                                 uint8_t* output, size_t capacity,
                                 size_t* written) {
    constexpr size_t kLength = 38u;
    if (output == nullptr || written == nullptr || capacity < kLength ||
        config.channel_count > odrive::scope::generated::kMaxCaptureChannels) {
        return false;
    }
    output[0] = static_cast<uint8_t>(config.mode);
    output[1] = static_cast<uint8_t>(config.trigger_type);
    put_u16(output + 2u, config.trigger_channel);
    output[4] = static_cast<uint8_t>(config.trigger_edge);
    output[5] = config.trigger_state;
    put_u32(output + 6u, config.sample_rate_hz);
    put_u16(output + 10u, config.decimation);
    put_u16(output + 12u, config.pre_samples);
    put_u16(output + 14u, config.post_samples);
    output[16] = config.channel_count;
    output[17] = 0u;
    uint32_t threshold_bits = 0u;
    std::memcpy(&threshold_bits, &config.threshold, sizeof(threshold_bits));
    put_u32(output + 18u, threshold_bits);
    for (size_t index = 0; index < odrive::scope::generated::kMaxCaptureChannels;
         ++index) {
        put_u16(output + 22u + index * 2u, config.channel_ids[index]);
    }
    *written = kLength;
    return true;
}

bool decode_scope_config_payload(const uint8_t* data, size_t length,
                                 odrive::scope::ScopeConfig* config) {
    constexpr size_t kLength = 38u;
    if (data == nullptr || config == nullptr || length != kLength ||
        data[17] != 0u || data[16] == 0u ||
        data[16] > odrive::scope::generated::kMaxCaptureChannels) {
        return false;
    }
    config->mode = static_cast<odrive::scope::CaptureMode>(data[0]);
    config->trigger_type = static_cast<odrive::scope::TriggerType>(data[1]);
    config->trigger_channel = get_u16(data + 2u);
    config->trigger_edge = static_cast<odrive::scope::TriggerEdge>(data[4]);
    config->trigger_state = data[5];
    config->sample_rate_hz = get_u32(data + 6u);
    config->decimation = get_u16(data + 10u);
    config->pre_samples = get_u16(data + 12u);
    config->post_samples = get_u16(data + 14u);
    config->channel_count = data[16];
    const uint32_t threshold_bits = get_u32(data + 18u);
    std::memcpy(&config->threshold, &threshold_bits, sizeof(threshold_bits));
    for (size_t index = 0; index < odrive::scope::generated::kMaxCaptureChannels;
         ++index) {
        config->channel_ids[index] = get_u16(data + 22u + index * 2u);
    }
    return true;
}

bool encode_scope_arm_payload(uint8_t action, uint8_t* output, size_t capacity,
                              size_t* written) {
    if (output == nullptr || written == nullptr || capacity < 2u || action > 2u) {
        return false;
    }
    output[0] = action;
    output[1] = 0u;
    *written = 2u;
    return true;
}

bool decode_scope_arm_payload(const uint8_t* data, size_t length,
                              uint8_t* action) {
    if (data == nullptr || action == nullptr || length != 2u ||
        data[0] > 2u || data[1] != 0u) return false;
    *action = data[0];
    return true;
}

bool encode_scope_status_payload(const odrive::scope::ScopeStatus& status,
                                 uint8_t* output, size_t capacity,
                                 size_t* written) {
    constexpr size_t kLength = 32u;
    if (output == nullptr || written == nullptr || capacity < kLength) return false;
    put_u32(output, status.capture_id);
    output[4] = static_cast<uint8_t>(status.state);
    output[5] = static_cast<uint8_t>(status.final_reason);
    output[6] = static_cast<uint8_t>(status.mode);
    output[7] = static_cast<uint8_t>(status.trigger_type);
    put_u32(output + 8u, status.sample_count);
    put_u32(output + 12u, status.dropped_samples);
    put_u32(output + 16u, status.sequence_gaps);
    output[20] = status.channel_count;
    output[21] = 0u;
    put_u16(output + 22u, status.decimation);
    put_u32(output + 24u, status.first_timestamp_cycles);
    put_u32(output + 28u, status.last_timestamp_cycles);
    *written = kLength;
    return true;
}

bool encode_scope_batch_payload(const odrive::scope::ScopeBatch& batch,
                                uint8_t* output, size_t capacity,
                                size_t* written) {
    if (output == nullptr || written == nullptr ||
        batch.sample_count > odrive::scope::generated::kMaxBatchSamples ||
        batch.channel_count == 0u ||
        batch.channel_count > odrive::scope::generated::kMaxCaptureChannels) {
        return false;
    }
    const size_t sample_size = 12u + 4u * batch.channel_count;
    const size_t total = 36u + sample_size * batch.sample_count;
    if (capacity < total || total > kMaxPayloadSize) return false;
    put_u32(output, batch.capture_id);
    put_u16(output + 4u, batch.sample_count);
    output[6] = batch.channel_count;
    output[7] = batch.flags;
    put_u16(output + 8u, batch.dropped_samples);
    put_u16(output + 10u, 0u);
    put_u32(output + 12u, batch.first_sequence);
    put_u32(output + 16u, batch.first_timestamp_cycles);
    for (size_t index = 0; index < odrive::scope::generated::kMaxCaptureChannels;
         ++index) {
        put_u16(output + 20u + index * 2u, batch.channel_ids[index]);
    }
    size_t offset = 36u;
    for (size_t sample_index = 0; sample_index < batch.sample_count;
         ++sample_index) {
        const auto& sample = batch.samples[sample_index];
        put_u32(output + offset, sample.control_sequence);
        put_u32(output + offset + 4u, sample.timestamp_cycles);
        put_u16(output + offset + 8u, sample.flags);
        put_u16(output + offset + 10u, 0u);
        offset += 12u;
        for (size_t channel = 0; channel < batch.channel_count; ++channel) {
            put_u32(output + offset, sample.values[channel]);
            offset += 4u;
        }
    }
    *written = offset;
    return true;
}

}  // namespace odrive::usb
