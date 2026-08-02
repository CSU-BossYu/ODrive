#include <array>
#include <algorithm>
#include <cstring>
#include <doctest.h>
#include <string>

#include "USB/usb_debug_protocol.hpp"
#include "USB/usb_debug_transport.hpp"

using namespace odrive::usb;

namespace {

Frame golden_frame() {
    Frame frame;
    frame.message_type = MessageType::COMMAND;
    frame.sequence = 0x01020304u;
    frame.request_id = 0xA1B2C3D4u;
    for (size_t index = 0; index < frame.build_id.size(); ++index) {
        frame.build_id[index] = static_cast<uint8_t>(0x10u + index);
    }
    for (size_t index = 0; index < frame.manifest_identity.size(); ++index) {
        frame.manifest_identity[index] = static_cast<uint8_t>(0x20u + index);
    }
    const CommandPayload command{1u, 1u, 0x11223344u};
    size_t written = 0u;
    REQUIRE(encode_command_payload(command, frame.payload.data(),
                                   frame.payload.size(), &written));
    frame.payload_length = static_cast<uint16_t>(written);
    return frame;
}

struct CapturedFrames {
    std::array<Frame, 8> frames{};
    size_t count = 0u;
};

void capture_frame(void* context, const Frame& frame) {
    auto* capture = static_cast<CapturedFrames*>(context);
    if (capture->count < capture->frames.size()) {
        capture->frames[capture->count++] = frame;
    }
}

bool submit_command(void* context, const odrive::safety::Command& command) {
    auto* count = static_cast<size_t*>(context);
    ++*count;
    return command.source == odrive::safety::CommandSource::USB;
}

bool apply_test_limits(void*, float velocity, float current, float torque) {
    return velocity <= 2.0f && current <= 1.0f && torque <= 0.2f;
}

uint32_t fake_clock(void* context) {
    return *static_cast<uint32_t*>(context);
}

void feed_frame(UsbDebugTransport& transport, const Frame& frame) {
    std::array<uint8_t, kMaxFrameSize> bytes{};
    const size_t length = encode_frame(frame, bytes.data(), bytes.size());
    REQUIRE(length != 0u);
    transport.receive_bytes(bytes.data(), length);
    transport.service();
}

Frame test_session_begin(uint32_t request_id, bool motion = true) {
    Frame begin;
    begin.message_type = MessageType::TEST_SESSION_BEGIN;
    begin.sequence = request_id;
    begin.request_id = request_id;
    begin.payload_length = 16u;
    begin.payload[0] = motion ? 0x03u : 0u;
    begin.payload[2] = 0xd0u;
    begin.payload[3] = 0x07u;
    const float limits[] = {0.5f, 0.5f, 0.1f};
    std::memcpy(begin.payload.data() + 4u, limits, sizeof(limits));
    return begin;
}

void establish_session(UsbDebugTransport& transport, uint32_t request_id = 1u) {
    transport.notify_connected(true);
    Frame hello;
    hello.message_type = MessageType::HELLO;
    hello.request_id = request_id;
    hello.sequence = request_id;
    std::array<uint8_t, kMaxFrameSize> bytes{};
    const size_t length = encode_frame(hello, bytes.data(), bytes.size());
    transport.receive_bytes(bytes.data(), length);
    transport.service();
}

std::string drain_tx(UsbDebugTransport& transport, uint16_t chunk_size) {
    std::string bytes;
    const uint8_t* data = nullptr;
    uint16_t length = 0u;
    while (transport.begin_tx_chunk(chunk_size, &data, &length)) {
        REQUIRE(length != 0u);
        REQUIRE(length <= chunk_size);
        bytes.append(reinterpret_cast<const char*>(data), length);
        transport.complete_tx_chunk(length);
    }
    return bytes;
}

}  // namespace

TEST_SUITE("UsbDebugProtocol") {
    TEST_CASE("golden vector encodes explicit little-endian fields") {
        const Frame frame = golden_frame();
        std::array<uint8_t, kMaxFrameSize> encoded{};
        const size_t length = encode_frame(frame, encoded.data(),
                                           encoded.size());
        REQUIRE(length == kHeaderSize + 8u + kTrailerSize);

        // The Python regression uses the same vector and checks this exact
        // byte stream, including CRC32.
        const char* expected_hex =
            "444f0404030004030201d4c3b2a10800"
            "1011121314151617"
            "202122232425262728292a2b2c2d2e2f"
            "0101000044332211"
            "ba079694";
        std::string actual;
        actual.reserve(length * 2u);
        static constexpr char digits[] = "0123456789abcdef";
        for (size_t index = 0; index < length; ++index) {
            actual.push_back(digits[encoded[index] >> 4u]);
            actual.push_back(digits[encoded[index] & 0x0fu]);
        }
        CHECK(actual == expected_hex);

        Frame decoded;
        REQUIRE(decode_frame(encoded.data(), length, &decoded) ==
                DecodeStatus::OK);
        CHECK(decoded.sequence == frame.sequence);
        CHECK(decoded.request_id == frame.request_id);
        CommandPayload command;
        REQUIRE(decode_command_payload(decoded.payload.data(),
                                       decoded.payload_length, &command));
        CHECK(command.arg0 == 0x11223344u);
        decoded.payload[2] = 1u;
        CHECK_FALSE(decode_command_payload(decoded.payload.data(),
                                           decoded.payload_length, &command));
    }

    TEST_CASE("single byte, arbitrary fragmentation, and sticky frames") {
        const Frame first = golden_frame();
        Frame second = first;
        second.sequence = 9u;
        second.request_id = 10u;
        std::array<uint8_t, kMaxFrameSize * 2u> bytes{};
        const size_t first_size = encode_frame(first, bytes.data(),
                                               bytes.size());
        const size_t second_size = encode_frame(
            second, bytes.data() + first_size, bytes.size() - first_size);
        StreamParser parser;
        CapturedFrames capture;
        for (size_t index = 0; index < first_size + second_size; ++index) {
            parser.feed(bytes.data() + index, 1u, &capture, capture_frame);
        }
        CHECK(capture.count == 2u);

        parser.reset();
        capture = {};
        const size_t total = first_size + second_size;
        for (size_t offset = 0u; offset < total;) {
            const size_t chunk = std::min<size_t>(7u, total - offset);
            parser.feed(bytes.data() + offset, chunk, &capture, capture_frame);
            offset += chunk;
        }
        CHECK(capture.count == 2u);
    }

    TEST_CASE("noise, CRC failure, and malformed length resynchronize") {
        const Frame valid = golden_frame();
        std::array<uint8_t, kMaxFrameSize> encoded{};
        const size_t length = encode_frame(valid, encoded.data(),
                                           encoded.size());
        std::array<uint8_t, kMaxFrameSize * 2u> input{};
        input[0] = 0x00u;
        input[1] = 0x7fu;
        input[2] = 0x44u;
        std::memcpy(input.data() + 3u, encoded.data(), length);
        input[3u + 20u] ^= 0x80u;
        std::memcpy(input.data() + 3u + length, encoded.data(), length);

        StreamParser parser;
        CapturedFrames capture;
        parser.feed(input.data(), 3u + 2u * length, &capture, capture_frame);
        CHECK(capture.count == 1u);
        CHECK(parser.stats().noise_bytes >= 2u);
        CHECK(parser.stats().crc_errors == 1u);

        parser.reset();
        std::array<uint8_t, kMaxFrameSize + 4u> malformed{};
        malformed[0] = static_cast<uint8_t>(kFrameMagic);
        malformed[1] = static_cast<uint8_t>(kFrameMagic >> 8u);
        std::memcpy(malformed.data() + 2u, encoded.data() + 2u,
                    kHeaderSize - 2u);
        malformed[14] = 0xffu;
        malformed[15] = 0xffu;
        parser.feed(malformed.data(), kHeaderSize, &capture, capture_frame);
        parser.feed(encoded.data(), length, &capture, capture_frame);
        CHECK(parser.stats().length_errors >= 1u);
        CHECK(capture.count >= 2u);
    }

    TEST_CASE("unknown version and message are rejected without desync") {
        Frame frame = golden_frame();
        std::array<uint8_t, kMaxFrameSize> bytes{};
        frame.protocol_version = 99u;
        size_t length = encode_frame(frame, bytes.data(), bytes.size());
        StreamParser parser;
        CapturedFrames capture;
        parser.feed(bytes.data(), length, &capture, capture_frame);
        CHECK(parser.stats().unknown_versions == 1u);

        frame = golden_frame();
        frame.message_type = static_cast<MessageType>(200u);
        length = encode_frame(frame, bytes.data(), bytes.size());
        parser.feed(bytes.data(), length, &capture, capture_frame);
        CHECK(parser.stats().unknown_messages == 1u);
        CHECK(capture.count == 0u);
    }

    TEST_CASE("request tracker rejects duplicate and expired requests") {
        RequestTracker tracker;
        CHECK(tracker.accept(1u, 100u) == RequestDecision::ACCEPT);
        CHECK(tracker.accept(1u, 101u) == RequestDecision::DUPLICATE);
        CHECK(tracker.accept(2u, 200u) == RequestDecision::ACCEPT);
        CHECK(tracker.accept(3u, 50u) == RequestDecision::EXPIRED);
    }

    TEST_CASE("transport keeps command and TX boundaries task-owned") {
        UsbDebugTransport transport;
        size_t submitted = 0u;
        transport.bind_command_sink(&submitted, submit_command);
        transport.bind_test_limits(nullptr, apply_test_limits, nullptr);
        establish_session(transport);
        std::array<uint8_t, kMaxFrameSize> bytes{};
        const uint8_t* tx_data = nullptr;
        uint16_t tx_length = 0u;
        REQUIRE(transport.begin_tx_chunk(64u, &tx_data, &tx_length));
        CHECK(transport.begin_tx_chunk(64u, &tx_data, &tx_length));
        transport.complete_tx_chunk(tx_length);

        Frame begin = test_session_begin(2u, false);
        size_t begin_length = encode_frame(begin, bytes.data(), bytes.size());
        transport.receive_bytes(bytes.data(), begin_length);
        transport.service();
        (void)drain_tx(transport, 64u);

        Frame command = golden_frame();
        command.sequence = 3u;
        command.request_id = 3u;
        const size_t length = encode_frame(command, bytes.data(), bytes.size());
        transport.receive_bytes(bytes.data(), length);
        transport.service();
        CHECK(submitted == 1u);

        transport.notify_connected(false);
        transport.service();
        // Disconnect owns one fail-safe DISARM submission. Bytes received
        // after disconnect must not submit any further command.
        CHECK(submitted == 2u);
        transport.receive_bytes(bytes.data(), length);
        transport.service();
        CHECK(submitted == 2u);
        CHECK_FALSE(transport.begin_tx_chunk(64u, &tx_data, &tx_length));
    }

    TEST_CASE("test session lease expiry deterministically submits DISARM") {
        UsbDebugTransport transport;
        size_t submitted = 0u;
        uint32_t now = 100u;
        transport.bind_command_sink(&submitted, submit_command);
        transport.bind_clock(&now, fake_clock);
        transport.bind_test_limits(nullptr, apply_test_limits, nullptr);
        establish_session(transport);
        (void)drain_tx(transport, 64u);
        feed_frame(transport, test_session_begin(2u));
        CHECK(transport.test_session_status().state == TestSessionState::ACTIVE);
        now += 2001u;
        transport.service();
        CHECK(submitted == 1u);
        CHECK(transport.test_session_status().state == TestSessionState::EXPIRED);
    }

    TEST_CASE("motion session without estop is rejected before command ownership") {
        UsbDebugTransport transport;
        transport.bind_test_limits(nullptr, apply_test_limits, nullptr);
        establish_session(transport);
        (void)drain_tx(transport, 64u);
        Frame begin = test_session_begin(2u);
        begin.payload[0] = 0x01u;
        feed_frame(transport, begin);
        CHECK(transport.test_session_status().state == TestSessionState::INACTIVE);
    }

    TEST_CASE("HELLO gates telemetry and oversized frames are chunked") {
        UsbDebugTransport transport;
        transport.notify_connected(true);
        odrive::trace::TraceDispatchRecord scope;
        scope.kind = odrive::trace::TraceEventKind::SCOPE;
        transport.publish_trace(scope);
        const uint8_t* data = nullptr;
        uint16_t length = 0u;
        CHECK_FALSE(transport.begin_tx_chunk(64u, &data, &length));

        establish_session(transport, 7u);
        CHECK_FALSE(transport.binary_session_active());
        const std::string capabilities = drain_tx(transport, 64u);
        REQUIRE_FALSE(capabilities.empty());
        CHECK(transport.binary_session_active());

        transport.publish_trace(scope);
        const std::string encoded_scope = drain_tx(transport, 64u);
        REQUIRE(encoded_scope.size() == kHeaderSize + 56u + kTrailerSize);
        Frame decoded;
        REQUIRE(decode_frame(
                    reinterpret_cast<const uint8_t*>(encoded_scope.data()),
                    encoded_scope.size(), &decoded) == DecodeStatus::OK);
        CHECK(decoded.message_type == MessageType::SCOPE_RECORD);
    }

    TEST_CASE("only USB command results are correlated onto USB") {
        UsbDebugTransport transport;
        establish_session(transport);
        (void)drain_tx(transport, 64u);

        odrive::safety::CommandResult can_result;
        can_result.request_id = 42u;
        can_result.status = odrive::safety::CommandStatus::COMPLETED;
        can_result.source = odrive::safety::CommandSource::CAN;
        transport.publish_command_result(can_result);
        CHECK(drain_tx(transport, 64u).empty());

        can_result.source = odrive::safety::CommandSource::USB;
        transport.publish_command_result(can_result);
        CHECK_FALSE(drain_tx(transport, 64u).empty());
    }

    TEST_CASE("RX and critical TX overflow are independent and counted") {
        UsbDebugTransport transport;
        establish_session(transport);
        (void)drain_tx(transport, 64u);
        std::array<uint8_t, UsbDebugTransport::kRxByteCapacity + 32u> noise{};
        transport.receive_bytes(noise.data(), noise.size());
        CHECK(transport.stats().rx_overflow != 0u);

        odrive::trace::TraceDispatchRecord record;
        record.kind = odrive::trace::TraceEventKind::CRITICAL;
        record.payload.critical.code = odrive::fault::FaultCode::TIMEOUT;
        for (size_t index = 0; index < UsbDebugTransport::kCriticalTxCapacity + 2u;
             ++index) {
            record.sequence = static_cast<uint32_t>(index + 1u);
            transport.publish_trace(record);
        }
        CHECK(transport.stats().tx_overflow_critical != 0u);
    }
}
