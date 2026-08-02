#include <array>
#include <cstring>

#include <doctest.h>

#include "USB/usb_debug_protocol.hpp"

using namespace odrive::usb;
using namespace odrive::scope;

TEST_SUITE("UsbScopeProtocol") {
    TEST_CASE("all scope message types pass complete frame decoding") {
        constexpr std::array<MessageType, 5u> message_types = {
            MessageType::SCOPE_CONFIG, MessageType::SCOPE_ARM,
            MessageType::SCOPE_STATUS, MessageType::SCOPE_DATA_BATCH,
            MessageType::SCOPE_STOP};
        for (const MessageType message_type : message_types) {
            Frame input;
            input.message_type = message_type;
            input.sequence = 7u;
            input.request_id = 9u;
            std::array<uint8_t, kMaxFrameSize> bytes{};
            const size_t length = encode_frame(input, bytes.data(), bytes.size());
            REQUIRE(length != 0u);
            Frame output;
            CHECK(decode_frame(bytes.data(), length, &output) == DecodeStatus::OK);
            CHECK(output.message_type == message_type);
        }
    }

    TEST_CASE("scope config round trips the generated channel whitelist") {
        ScopeConfig input;
        input.mode = CaptureMode::TRIGGERED;
        input.trigger_type = TriggerType::THRESHOLD;
        input.trigger_channel = 11u;
        input.sample_rate_hz = 10000u;
        input.decimation = 1u;
        input.pre_samples = 64u;
        input.post_samples = 128u;
        input.channel_count = 3u;
        input.threshold = 1.25f;
        input.channel_ids[0] = 6u;
        input.channel_ids[1] = 11u;
        input.channel_ids[2] = 13u;
        std::array<uint8_t, 64u> bytes{};
        size_t written = 0u;
        REQUIRE(encode_scope_config_payload(input, bytes.data(), bytes.size(),
                                            &written));
        CHECK(written == 38u);
        ScopeConfig output;
        REQUIRE(decode_scope_config_payload(bytes.data(), written, &output));
        CHECK(output.trigger_channel == 11u);
        CHECK(output.threshold == doctest::Approx(1.25f));
        CHECK(output.channel_ids[2] == 13u);
    }

    TEST_CASE("scope batch preserves fixed timestamp and sequence fields") {
        ScopeBatch batch;
        batch.capture_id = 4u;
        batch.channel_count = 2u;
        batch.channel_ids[0] = 6u;
        batch.channel_ids[1] = 11u;
        batch.sample_count = 1u;
        batch.first_sequence = 100u;
        batch.first_timestamp_cycles = 0xfffffff0u;
        batch.samples[0].control_sequence = 100u;
        batch.samples[0].timestamp_cycles = 0xfffffff0u;
        float iq = 2.5f;
        std::memcpy(&batch.samples[0].values[1], &iq, sizeof(iq));
        std::array<uint8_t, kMaxPayloadSize> bytes{};
        size_t written = 0u;
        REQUIRE(encode_scope_batch_payload(batch, bytes.data(), bytes.size(),
                                           &written));
        CHECK(written == 36u + 12u + 8u);
        CHECK(bytes[0] == 4u);
        CHECK(bytes[16] == 0xf0u);
    }

    TEST_CASE("scope arm payload preserves manual trigger action") {
        std::array<uint8_t, 2u> bytes{};
        size_t written = 0u;
        REQUIRE(encode_scope_arm_payload(2u, bytes.data(), bytes.size(),
                                         &written));
        CHECK(written == 2u);
        uint8_t action = 0u;
        REQUIRE(decode_scope_arm_payload(bytes.data(), written, &action));
        CHECK(action == 2u);
    }
}
