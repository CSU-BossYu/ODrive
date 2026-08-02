#include "usb_debug_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "MotorControl/crash_recorder.hpp"

namespace odrive::usb {

namespace {

constexpr uint8_t kPriorityCritical = 0u;
constexpr uint8_t kPriorityResultState = 1u;
constexpr uint8_t kPriorityCalibration = 2u;
constexpr uint8_t kPriorityScope = 3u;
constexpr uint8_t kPriorityLog = 4u;

constexpr uint16_t kReasonRequestDuplicate = 0xFF01u;
constexpr uint16_t kReasonRequestExpired = 0xFF02u;
constexpr uint16_t kReasonQueueFull = 0xFF03u;
constexpr uint16_t kReasonBadPayload = 0xFF04u;
constexpr uint16_t kReasonSessionRequired = 0xFF10u;
constexpr uint16_t kReasonSessionBusy = 0xFF11u;
constexpr uint16_t kReasonSessionMismatch = 0xFF12u;
constexpr uint16_t kReasonSessionUnsafe = 0xFF13u;
constexpr uint16_t kReasonSessionExpired = 0xFF14u;

void put_u16(uint8_t* output, size_t* offset, uint16_t value) {
    output[(*offset)++] = static_cast<uint8_t>(value);
    output[(*offset)++] = static_cast<uint8_t>(value >> 8u);
}

void put_u32(uint8_t* output, size_t* offset, uint32_t value) {
    for (size_t index = 0; index < 4u; ++index) {
        output[(*offset)++] = static_cast<uint8_t>(value >> (8u * index));
    }
}

void put_u8(uint8_t* output, size_t* offset, uint8_t value) {
    output[(*offset)++] = value;
}

void put_f32(uint8_t* output, size_t* offset, float value) {
    static_assert(sizeof(float) == sizeof(uint32_t),
                  "protocol requires IEEE-754 binary32");
    uint32_t bits = 0u;
    std::memcpy(&bits, &value, sizeof(bits));
    put_u32(output, offset, bits);
}

uint16_t get_u16(const uint8_t* input, size_t offset) {
    return static_cast<uint16_t>(input[offset]) |
        (static_cast<uint16_t>(input[offset + 1u]) << 8u);
}

uint32_t get_u32(const uint8_t* input, size_t offset) {
    return static_cast<uint32_t>(input[offset]) |
        (static_cast<uint32_t>(input[offset + 1u]) << 8u) |
        (static_cast<uint32_t>(input[offset + 2u]) << 16u) |
        (static_cast<uint32_t>(input[offset + 3u]) << 24u);
}

float get_f32(const uint8_t* input, size_t offset) {
    const uint32_t bits = get_u32(input, offset);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void put_fault_fields(uint8_t* output, size_t* offset,
                      const odrive::fault::FaultSource source,
                      const odrive::fault::FaultCode code,
                      const odrive::fault::FaultSite site,
                      const odrive::fault::FaultSeverity severity) {
    put_u16(output, offset, static_cast<uint16_t>(source));
    put_u16(output, offset, static_cast<uint16_t>(code));
    put_u16(output, offset, static_cast<uint16_t>(site));
    put_u8(output, offset, static_cast<uint8_t>(severity));
    put_u8(output, offset, 0u);
}

}  // namespace

RequestDecision RequestTracker::accept(uint32_t request_id,
                                       uint32_t sequence) {
    if (request_id == 0u) {
        return RequestDecision::EXPIRED;
    }
    for (const Entry& entry : entries_) {
        if (entry.valid && entry.request_id == request_id) {
            return RequestDecision::DUPLICATE;
        }
    }
    if (have_sequence_ &&
        static_cast<int32_t>(sequence - highest_sequence_) <
            -static_cast<int32_t>(kSequenceWindow)) {
        return RequestDecision::EXPIRED;
    }

    size_t slot = 0u;
    bool found_empty = false;
    uint32_t oldest_sequence = 0u;
    for (size_t index = 0; index < entries_.size(); ++index) {
        if (!entries_[index].valid) {
            slot = index;
            found_empty = true;
            break;
        }
        if (index == 0u || static_cast<int32_t>(
                entries_[index].sequence - oldest_sequence) < 0) {
            oldest_sequence = entries_[index].sequence;
            slot = index;
        }
    }
    (void)found_empty;
    entries_[slot] = {request_id, sequence, true};
    if (!have_sequence_ ||
        static_cast<int32_t>(sequence - highest_sequence_) > 0) {
        highest_sequence_ = sequence;
        have_sequence_ = true;
    }
    return RequestDecision::ACCEPT;
}

void RequestTracker::reset() {
    entries_ = {};
    highest_sequence_ = 0u;
    have_sequence_ = false;
}

UsbDebugTransport::UsbDebugTransport() = default;

void UsbDebugTransport::bind_command_sink(void* context,
                                          CommandSubmitFn callback) {
    command_context_ = context;
    command_submit_ = callback;
}

void UsbDebugTransport::receive_bytes(const uint8_t* data, size_t length) {
    if (data == nullptr && length != 0u) {
        return;
    }
    for (size_t index = 0; index < length; ++index) {
        if (!rx_ring_.push(data[index])) {
            ++stats_.rx_overflow;
        } else {
            ++stats_.rx_bytes;
        }
    }
}

void UsbDebugTransport::notify_connected(bool connected) {
    if (!connected && test_session_.state != TestSessionState::INACTIVE) {
        force_session_disarm();
        test_session_ = {};
        pending_stop_request_id_ = 0u;
        test_session_stop_confirmed_ = true;
    }
    connected_.store(connected, std::memory_order_release);
}

uint32_t UsbDebugTransport::now_ms() const {
    return clock_ms_ == nullptr ? 0u : clock_ms_(clock_context_);
}

TestSessionStatus UsbDebugTransport::test_session_status() const {
    TestSessionStatus status = test_session_;
    if (status.state == TestSessionState::ACTIVE ||
        status.state == TestSessionState::STOPPING) {
        const uint32_t now = now_ms();
        status.lease_remaining_ms =
            static_cast<int32_t>(test_session_deadline_ms_ - now) > 0
                ? test_session_deadline_ms_ - now : 0u;
    }
    return status;
}

void UsbDebugTransport::force_session_disarm() {
    if (command_submit_ == nullptr) return;
    odrive::safety::Command command;
    command.request_id = 0xF0000000u | (++test_session_generation_ & 0x0FFFFFFFu);
    command.source = odrive::safety::CommandSource::USB;
    command.type = odrive::safety::CommandType::DISARM;
    command.operation = odrive::safety::Operation::NONE;
    (void)command_submit_(command_context_, command);
}

void UsbDebugTransport::expire_test_session() {
    force_session_disarm();
    test_session_.state = TestSessionState::EXPIRED;
    test_session_.reason = kReasonSessionExpired;
    test_session_.lease_remaining_ms = 0u;
    pending_stop_request_id_ = 0u;
    test_session_stop_confirmed_ = true;
    (void)queue_test_session_status(0u, FRAME_ERROR, kReasonSessionExpired);
}

void UsbDebugTransport::parsed_frame(void* context, const Frame& frame) {
    if (context != nullptr) {
        static_cast<UsbDebugTransport*>(context)->handle_frame(frame);
    }
}

bool UsbDebugTransport::is_usb_command(uint8_t value) {
    return value >= static_cast<uint8_t>(odrive::safety::CommandType::SET_OPERATION) &&
           value <= static_cast<uint8_t>(odrive::safety::CommandType::RESTART_CALIBRATION);
}

uint16_t UsbDebugTransport::reason_for_decision(RequestDecision decision) {
    return decision == RequestDecision::DUPLICATE
        ? kReasonRequestDuplicate : kReasonRequestExpired;
}

uint16_t UsbDebugTransport::reason_for_queue_failure() {
    return kReasonQueueFull;
}

uint16_t UsbDebugTransport::reason_for_bad_payload() {
    return kReasonBadPayload;
}

bool UsbDebugTransport::queue_test_session_status(uint32_t request_id,
                                                  uint8_t flags,
                                                  uint16_t reason) {
    TestSessionStatus status = test_session_status();
    if (reason != 0u) status.reason = reason;
    std::array<uint8_t, 24u> payload{};
    size_t offset = 0u;
    put_u8(payload.data(), &offset, static_cast<uint8_t>(status.state));
    put_u8(payload.data(), &offset, status.flags);
    put_u16(payload.data(), &offset, status.reason);
    put_u32(payload.data(), &offset, status.session_id);
    put_u32(payload.data(), &offset, status.lease_remaining_ms);
    put_f32(payload.data(), &offset, status.velocity_limit);
    put_f32(payload.data(), &offset, status.current_limit);
    put_f32(payload.data(), &offset, status.torque_limit);
    return queue_frame(MessageType::TEST_SESSION_STATUS, flags | FRAME_RESPONSE,
                       request_id, payload.data(), offset,
                       kPriorityResultState);
}

bool UsbDebugTransport::begin_test_session(const Frame& frame) {
    if (frame.payload_length != 16u || frame.request_id == 0u ||
        frame.payload[1] != 0u) {
        return queue_test_session_status(frame.request_id, FRAME_ERROR,
                                         kReasonBadPayload);
    }
    if (test_session_.state == TestSessionState::ACTIVE ||
        test_session_.state == TestSessionState::STOPPING) {
        return queue_test_session_status(frame.request_id, FRAME_ERROR,
                                         kReasonSessionBusy);
    }
    const uint8_t flags = frame.payload[0];
    const uint16_t lease_ms = get_u16(frame.payload.data(), 2u);
    const float velocity = get_f32(frame.payload.data(), 4u);
    const float current = get_f32(frame.payload.data(), 8u);
    const float torque = get_f32(frame.payload.data(), 12u);
    const bool motion = (flags & 0x01u) != 0u;
    const bool estop = (flags & 0x02u) != 0u;
    const bool valid = (flags & ~0x03u) == 0u && lease_ms >= 250u &&
        lease_ms <= 10000u && std::isfinite(velocity) && velocity > 0.0f &&
        velocity <= 2.0f && std::isfinite(current) && current > 0.0f &&
        current <= 1.0f && std::isfinite(torque) && torque > 0.0f &&
        torque <= 0.2f && (!motion || estop);
    if (!valid) {
        return queue_test_session_status(frame.request_id, FRAME_ERROR,
                                         kReasonSessionUnsafe);
    }
    if (test_limits_apply_ == nullptr ||
        !test_limits_apply_(test_limits_context_, velocity, current, torque)) {
        return queue_test_session_status(frame.request_id, FRAME_ERROR,
                                         kReasonSessionUnsafe);
    }
    test_session_ = {};
    test_session_.state = TestSessionState::ACTIVE;
    test_session_.flags = flags;
    test_session_.session_id = ++test_session_generation_;
    if (test_session_.session_id == 0u) {
        test_session_.session_id = ++test_session_generation_;
    }
    test_session_.velocity_limit = velocity;
    test_session_.current_limit = current;
    test_session_.torque_limit = torque;
    test_session_lease_ms_ = lease_ms;
    test_session_deadline_ms_ = now_ms() + lease_ms;
    test_session_stop_confirmed_ = !motion;
    pending_stop_request_id_ = 0u;
    return queue_test_session_status(frame.request_id);
}

bool UsbDebugTransport::refresh_test_session(const Frame& frame) {
    if (frame.payload_length != 4u ||
        test_session_.state != TestSessionState::ACTIVE ||
        get_u32(frame.payload.data(), 0u) != test_session_.session_id) {
        return queue_test_session_status(frame.request_id, FRAME_ERROR,
                                         kReasonSessionMismatch);
    }
    test_session_deadline_ms_ = now_ms() + test_session_lease_ms_;
    return queue_test_session_status(frame.request_id);
}

bool UsbDebugTransport::stop_test_session(const Frame& frame) {
    if (frame.payload_length != 4u || frame.request_id == 0u ||
        test_session_.state != TestSessionState::ACTIVE ||
        get_u32(frame.payload.data(), 0u) != test_session_.session_id) {
        return queue_command_result(
            frame.request_id, frame.sequence,
            static_cast<uint8_t>(odrive::safety::CommandStatus::REJECTED),
            0u, kReasonSessionMismatch);
    }
    CommandEnvelope envelope;
    envelope.request_id = frame.request_id;
    envelope.sequence = frame.sequence;
    envelope.payload.command_type = static_cast<uint8_t>(
        odrive::safety::CommandType::DISARM);
    envelope.payload.operation = static_cast<uint8_t>(
        odrive::safety::Operation::NONE);
    if (!command_ring_.push(envelope)) {
        return queue_command_result(
            frame.request_id, frame.sequence,
            static_cast<uint8_t>(odrive::safety::CommandStatus::REJECTED),
            0u, reason_for_queue_failure());
    }
    test_session_.state = TestSessionState::STOPPING;
    pending_stop_request_id_ = frame.request_id;
    return true;
}

bool UsbDebugTransport::end_test_session(const Frame& frame) {
    if (frame.payload_length != 4u ||
        get_u32(frame.payload.data(), 0u) != test_session_.session_id ||
        test_session_.state != TestSessionState::ACTIVE ||
        !test_session_stop_confirmed_) {
        return queue_test_session_status(frame.request_id, FRAME_ERROR,
                                         kReasonSessionUnsafe);
    }
    test_session_ = {};
    test_session_lease_ms_ = 0u;
    test_session_deadline_ms_ = 0u;
    pending_stop_request_id_ = 0u;
    test_session_stop_confirmed_ = true;
    if (test_limits_restore_ != nullptr) {
        test_limits_restore_(test_limits_context_);
    }
    return queue_test_session_status(frame.request_id);
}

void UsbDebugTransport::handle_frame(const Frame& frame) {
    if (frame.message_type == MessageType::HELLO) {
        if (frame.payload_length != 0u) {
            ++stats_.rejected_commands;
            return;
        }
        if (handshake_in_progress_.load(std::memory_order_acquire)) {
            ++stats_.duplicate_requests;
            return;
        }
        // Queue the handshake response before publishing the active session.
        // Otherwise fault/state producers can observe `active`, preempt this
        // task, fill the result queue, and leave the host in a half-open
        // session where legacy text is suppressed but CAPABILITIES was lost.
        if (queue_capabilities(frame.request_id, frame.sequence)) {
            // NEGOTIATING is intentionally not BINARY. Runtime telemetry stays
            // gated until the complete CAPABILITIES frame reaches the host.
            binary_session_active_.store(false, std::memory_order_release);
            handshake_in_progress_.store(true, std::memory_order_release);
        } else {
            ++stats_.rejected_commands;
        }
        return;
    }
    if (!binary_session_active_.load(std::memory_order_acquire)) {
        ++stats_.rejected_commands;
        return;
    }
    switch (frame.message_type) {
        case MessageType::STATS:
            if (frame.payload_length == 0u) {
                queue_stats(frame.request_id, frame.sequence);
            } else {
                ++stats_.rejected_commands;
            }
            break;
        case MessageType::COMMAND: {
            CommandPayload payload;
            if (!decode_command_payload(frame.payload.data(),
                                        frame.payload_length, &payload) ||
                !is_usb_command(payload.command_type)) {
                ++stats_.rejected_commands;
                queue_command_result(frame.request_id, frame.sequence,
                                     static_cast<uint8_t>(
                                         odrive::safety::CommandStatus::REJECTED),
                                     0u,
                                     reason_for_bad_payload());
                break;
            }
            if (test_session_.state != TestSessionState::ACTIVE) {
                ++stats_.rejected_commands;
                queue_command_result(frame.request_id, frame.sequence,
                                     static_cast<uint8_t>(
                                         odrive::safety::CommandStatus::REJECTED),
                                     0u, kReasonSessionRequired);
                break;
            }
            if (payload.command_type == static_cast<uint8_t>(
                    odrive::safety::CommandType::ARM) &&
                (test_session_.flags & 0x03u) != 0x03u) {
                ++stats_.rejected_commands;
                queue_command_result(frame.request_id, frame.sequence,
                                     static_cast<uint8_t>(
                                         odrive::safety::CommandStatus::REJECTED),
                                     0u, kReasonSessionUnsafe);
                break;
            }
            const RequestDecision decision = request_tracker_.accept(
                frame.request_id, frame.sequence);
            if (decision != RequestDecision::ACCEPT) {
                ++stats_.rejected_commands;
                if (decision == RequestDecision::DUPLICATE) {
                    ++stats_.duplicate_requests;
                } else {
                    ++stats_.expired_requests;
                }
                queue_command_result(frame.request_id, frame.sequence,
                                     static_cast<uint8_t>(
                                         odrive::safety::CommandStatus::REJECTED),
                                     0u,
                                     reason_for_decision(decision));
                break;
            }
            CommandEnvelope envelope;
            envelope.request_id = frame.request_id;
            envelope.sequence = frame.sequence;
            envelope.payload = payload;
            if (!command_ring_.push(envelope)) {
                ++stats_.rejected_commands;
                queue_command_result(frame.request_id, frame.sequence,
                                     static_cast<uint8_t>(
                                         odrive::safety::CommandStatus::REJECTED),
                                     0u,
                                     reason_for_queue_failure());
            } else if (payload.command_type == static_cast<uint8_t>(
                           odrive::safety::CommandType::DISARM)) {
                test_session_.state = TestSessionState::STOPPING;
                pending_stop_request_id_ = frame.request_id;
            } else if (payload.command_type == static_cast<uint8_t>(
                           odrive::safety::CommandType::ARM)) {
                test_session_stop_confirmed_ = false;
            }
            break;
        }
        case MessageType::TEST_SESSION_BEGIN:
            (void)begin_test_session(frame);
            break;
        case MessageType::TEST_SESSION_KEEPALIVE:
            (void)refresh_test_session(frame);
            break;
        case MessageType::TEST_SESSION_STOP:
            (void)stop_test_session(frame);
            break;
        case MessageType::TEST_SESSION_END:
            (void)end_test_session(frame);
            break;
        case MessageType::SCOPE_CONFIG: {
            odrive::scope::ScopeConfig config;
            if (scope_engine_ == nullptr ||
                !decode_scope_config_payload(frame.payload.data(),
                                             frame.payload_length, &config)) {
                ++stats_.rejected_commands;
                odrive::scope::ScopeStatus error;
                error.final_reason = odrive::scope::CaptureFinalReason::INVALID_CONFIG;
                queue_scope_status(frame.request_id, error,
                                   FRAME_RESPONSE | FRAME_ERROR);
                break;
            }
            uint32_t capture_id = 0u;
            const auto validation = scope_engine_->configure_from_task(
                config, &capture_id);
            if (validation != odrive::scope::ValidationError::NONE) {
                ++stats_.rejected_commands;
                odrive::scope::ScopeStatus error;
                error.capture_id = capture_id;
                error.final_reason = odrive::scope::CaptureFinalReason::INVALID_CONFIG;
                queue_scope_status(frame.request_id, error,
                                   FRAME_RESPONSE | FRAME_ERROR);
            } else {
                odrive::scope::ScopeStatus scope_status;
                scope_engine_->read_status(&scope_status);
                queue_scope_status(frame.request_id, scope_status);
            }
            break;
        }
        case MessageType::SCOPE_ARM: {
            uint8_t action = 0u;
            if (scope_engine_ == nullptr ||
                !decode_scope_arm_payload(frame.payload.data(),
                                          frame.payload_length, &action) ||
                (action == 1u && !scope_engine_->arm_from_task()) ||
                (action == 0u && !scope_engine_->stop_from_task()) ||
                (action == 2u && !scope_engine_->trigger_manual_from_task())) {
                ++stats_.rejected_commands;
                odrive::scope::ScopeStatus error;
                error.final_reason = odrive::scope::CaptureFinalReason::INVALID_CONFIG;
                queue_scope_status(frame.request_id, error,
                                   FRAME_RESPONSE | FRAME_ERROR);
            } else {
                odrive::scope::ScopeStatus scope_status;
                scope_engine_->read_status(&scope_status);
                queue_scope_status(frame.request_id, scope_status);
            }
            break;
        }
        case MessageType::SCOPE_STOP: {
            if (frame.payload_length != 0u || scope_engine_ == nullptr) {
                ++stats_.rejected_commands;
                break;
            }
            scope_engine_->stop_from_task();
            odrive::scope::ScopeStatus scope_status;
            scope_engine_->read_status(&scope_status);
            queue_scope_status(frame.request_id, scope_status);
            break;
        }
        case MessageType::SCOPE_STATUS: {
            if (frame.payload_length != 0u || scope_engine_ == nullptr) {
                ++stats_.rejected_commands;
                break;
            }
            odrive::scope::ScopeStatus scope_status;
            scope_engine_->read_status(&scope_status);
            queue_scope_status(frame.request_id, scope_status);
            break;
        }
        case MessageType::CRASH_ACK: {
            if (frame.payload_length != 4u || frame.request_id == 0u) {
                ++stats_.rejected_commands;
                break;
            }
            const uint32_t record_crc =
                static_cast<uint32_t>(frame.payload[0]) |
                (static_cast<uint32_t>(frame.payload[1]) << 8u) |
                (static_cast<uint32_t>(frame.payload[2]) << 16u) |
                (static_cast<uint32_t>(frame.payload[3]) << 24u);
            std::array<uint8_t, 4u> response{{
                frame.payload[0], frame.payload[1],
                frame.payload[2], frame.payload[3]}};
            const bool valid = odrive::crash::pending_crc(record_crc);
            const bool queued = queue_frame(
                MessageType::CRASH_ACK,
                static_cast<uint8_t>(FRAME_RESPONSE |
                    (valid ? 0u : static_cast<uint8_t>(FRAME_ERROR))),
                frame.request_id, response.data(), response.size(),
                kPriorityCritical);
            if (!queued) {
                ++stats_.rejected_commands;
            }
            if (valid && queued && odrive::crash::clear_if_crc(record_crc)) {
                crash_report_queued_ = false;
            }
            break;
        }
        default:
            // Device-originated records are not accepted as host commands.
            break;
    }
}

void UsbDebugTransport::drain_commands() {
    CommandEnvelope envelope;
    for (size_t count = 0; count < kCommandCapacity &&
                            command_ring_.pop(&envelope); ++count) {
        odrive::safety::Command command;
        command.request_id = envelope.request_id;
        command.source = odrive::safety::CommandSource::USB;
        command.type = static_cast<odrive::safety::CommandType>(
            envelope.payload.command_type);
        command.operation = static_cast<odrive::safety::Operation>(
            envelope.payload.operation);
        command.arg0 = envelope.payload.arg0;
        if (command_submit_ == nullptr ||
            !command_submit_(command_context_, command)) {
            ++stats_.rejected_commands;
            queue_command_result(envelope.request_id, envelope.sequence,
                                 static_cast<uint8_t>(
                                     odrive::safety::CommandStatus::REJECTED),
                                 0u,
                                 reason_for_queue_failure());
        }
    }
}

void UsbDebugTransport::service() {
    if (!connected_.load(std::memory_order_acquire)) {
        parser_.reset();
        request_tracker_.reset();
        drop_disconnected_queues();
        uint8_t ignored = 0u;
        while (rx_ring_.pop(&ignored)) {}
        binary_session_active_.store(false, std::memory_order_release);
        return;
    }

    if (clock_ms_ != nullptr &&
        (test_session_.state == TestSessionState::ACTIVE ||
         test_session_.state == TestSessionState::STOPPING) &&
        static_cast<int32_t>(now_ms() - test_session_deadline_ms_) >= 0) {
        expire_test_session();
    }

    uint8_t value = 0u;
    while (rx_ring_.pop(&value)) {
        parser_.feed(&value, 1u, this, parsed_frame);
    }
    drain_commands();
    service_crash_report();
    service_scope_stream();
}

void UsbDebugTransport::service_crash_report() {
    if (crash_report_queued_ ||
        !binary_session_active_.load(std::memory_order_acquire)) {
        return;
    }
    std::array<uint8_t, odrive::crash::kEncodedPayloadSize> payload{};
    const size_t length = odrive::crash::encode_pending(
        payload.data(), payload.size());
    if (length != 0u &&
        queue_frame(MessageType::CRASH_REPORT, 0u, 0u,
                    payload.data(), length, kPriorityCritical)) {
        crash_report_queued_ = true;
    }
}

bool UsbDebugTransport::queue_frame(MessageType message_type, uint8_t flags,
                                    uint32_t request_id,
                                    const uint8_t* payload,
                                    size_t payload_length,
                                    uint8_t priority) {
    return queue_frame_parts(message_type, flags, request_id, nullptr, 0u,
                             payload, payload_length, priority);
}

bool UsbDebugTransport::queue_frame_parts(
        MessageType message_type, uint8_t flags, uint32_t request_id,
        const uint8_t* prefix, size_t prefix_length,
        const uint8_t* payload, size_t payload_length, uint8_t priority) {
    TxPacket packet;
    const size_t length = encode_device_frame_parts(
        message_type, flags, next_tx_sequence(), request_id,
        prefix, prefix_length, payload, payload_length,
        packet.bytes.data(), packet.bytes.size());
    if (length == 0u) {
        return false;
    }
    packet.length = static_cast<uint16_t>(length);
    bool queued = false;
    switch (priority) {
        case kPriorityCritical: queued = critical_tx_ring_.push(packet); break;
        case kPriorityResultState:
            queued = result_state_tx_ring_.push(packet);
            break;
        case kPriorityCalibration:
            queued = calibration_tx_ring_.push(packet);
            break;
        case kPriorityScope: queued = scope_tx_ring_.push(packet); break;
        case kPriorityLog: queued = log_tx_ring_.push(packet); break;
        default: return false;
    }
    if (!queued) {
        switch (priority) {
            case kPriorityCritical: ++stats_.tx_overflow_critical; break;
            case kPriorityResultState: ++stats_.tx_overflow_result_state; break;
            case kPriorityCalibration: ++stats_.tx_overflow_calibration; break;
            case kPriorityScope: ++stats_.tx_overflow_scope; break;
            case kPriorityLog: ++stats_.tx_overflow_log; break;
            default: break;
        }
    }
    return queued;
}

bool UsbDebugTransport::queue_command_result(uint32_t request_id,
                                             uint32_t sequence,
                                             uint8_t status,
                                             uint32_t state_epoch,
                                             uint16_t reason) {
    const uint8_t flags = static_cast<uint8_t>(FRAME_RESPONSE |
        (status == static_cast<uint8_t>(odrive::safety::CommandStatus::REJECTED) ||
         status == static_cast<uint8_t>(odrive::safety::CommandStatus::FAILED)
             ? static_cast<uint8_t>(FRAME_ERROR) : 0u));
    CommandResultPayload payload{status, state_epoch, reason};
    std::array<uint8_t, 8u> encoded_payload{};
    size_t written = 0u;
    if (!encode_command_result_payload(payload, encoded_payload.data(),
                                        encoded_payload.size(), &written)) {
        return false;
    }
    (void)sequence;
    return queue_frame(MessageType::COMMAND_RESULT, flags, request_id,
                       encoded_payload.data(), written, kPriorityResultState);
}

bool UsbDebugTransport::queue_capabilities(uint32_t request_id,
                                           uint32_t sequence) {
    std::array<uint8_t, 20u> payload{};
    size_t offset = 0u;
    put_u8(payload.data(), &offset, kProtocolVersion);
    put_u8(payload.data(), &offset, kSchemaVersion);
    put_u16(payload.data(), &offset,
            static_cast<uint16_t>(kMaxPayloadSize));
    put_u32(payload.data(), &offset, 0x000003FFu);
    put_u16(payload.data(), &offset,
            static_cast<uint16_t>(kRxByteCapacity));
    put_u16(payload.data(), &offset,
            static_cast<uint16_t>(kCriticalTxCapacity));
    put_u16(payload.data(), &offset,
            static_cast<uint16_t>(kScopeTxCapacity));
    put_u8(payload.data(), &offset,
           static_cast<uint8_t>(odrive::scope::generated::kMaxCaptureChannels));
    put_u8(payload.data(), &offset, 0u);
    put_u16(payload.data(), &offset,
            static_cast<uint16_t>(odrive::scope::generated::kMaxPreSamples));
    put_u16(payload.data(), &offset,
            static_cast<uint16_t>(odrive::scope::generated::kMaxPostSamples));
    if (handshake_tx_pending_) {
        return false;
    }
    const size_t length = encode_device_frame(
        MessageType::CAPABILITIES, FRAME_RESPONSE, next_tx_sequence(),
        request_id, payload.data(), offset, handshake_tx_.bytes.data(),
        handshake_tx_.bytes.size());
    (void)sequence;
    if (length == 0u) {
        return false;
    }
    handshake_tx_.length = static_cast<uint16_t>(length);
    handshake_tx_pending_ = true;
    return true;
}

bool UsbDebugTransport::queue_stats(uint32_t request_id, uint32_t sequence) {
    std::array<uint8_t, 48u> payload{};
    size_t offset = 0u;
    put_u32(payload.data(), &offset, stats_.rx_bytes.load());
    put_u32(payload.data(), &offset, stats_.rx_overflow.load());
    put_u32(payload.data(), &offset, stats_.tx_frames.load());
    put_u32(payload.data(), &offset, stats_.tx_overflow_critical.load());
    put_u32(payload.data(), &offset, stats_.tx_overflow_result_state.load());
    put_u32(payload.data(), &offset, stats_.tx_overflow_calibration.load());
    put_u32(payload.data(), &offset, stats_.tx_overflow_scope.load());
    put_u32(payload.data(), &offset, stats_.tx_overflow_log.load());
    put_u32(payload.data(), &offset, parser_.stats().crc_errors);
    put_u32(payload.data(), &offset, parser_.stats().length_errors);
    put_u32(payload.data(), &offset, parser_.stats().dropped_frames);
    (void)sequence;
    return queue_frame(MessageType::STATS, FRAME_RESPONSE, request_id,
                       payload.data(), offset, kPriorityResultState);
}

bool UsbDebugTransport::queue_scope_status(
        uint32_t request_id, const odrive::scope::ScopeStatus& status,
        uint8_t flags) {
    std::array<uint8_t, 32u> payload{};
    size_t written = 0u;
    if (!encode_scope_status_payload(status, payload.data(), payload.size(),
                                     &written)) {
        return false;
    }
    return queue_frame(MessageType::SCOPE_STATUS, flags, request_id,
                       payload.data(), written, kPriorityResultState);
}

void UsbDebugTransport::service_scope_stream() {
    if (scope_engine_ == nullptr ||
        !binary_session_active_.load(std::memory_order_acquire)) {
        return;
    }
    for (size_t count = 0; count < 2u; ++count) {
        if (!pending_scope_batch_valid_) {
            if (!scope_engine_->read_batch(&pending_scope_batch_)) break;
            pending_scope_batch_valid_ = true;
        }
        std::array<uint8_t, kMaxPayloadSize> payload{};
        size_t written = 0u;
        if (!encode_scope_batch_payload(pending_scope_batch_, payload.data(),
                                        payload.size(), &written)) {
            break;
        }
        if (!queue_frame(MessageType::SCOPE_DATA_BATCH, 0u, 0u,
                         payload.data(), written, kPriorityScope)) {
            break;
        }
        pending_scope_batch_valid_ = false;
    }
}

bool UsbDebugTransport::make_trace_payload(
        const odrive::trace::TraceDispatchRecord& record,
        MessageType* message_type, uint8_t* payload, size_t capacity,
        size_t* written) {
    if (message_type == nullptr || payload == nullptr || written == nullptr ||
        capacity < 64u) {
        return false;
    }
    size_t offset = 0u;
    switch (record.kind) {
        case odrive::trace::TraceEventKind::CRITICAL: {
            *message_type = MessageType::FAULT_EVENT;
            const auto& event = record.payload.critical;
            put_u32(payload, &offset, event.sequence);
            put_u32(payload, &offset, event.control_sequence);
            put_u32(payload, &offset, event.state_epoch);
            put_u32(payload, &offset, event.timestamp_cycles);
            put_u32(payload, &offset, event.fault_sequence);
            put_fault_fields(payload, &offset, event.source,
                             event.code, event.site, event.severity);
            put_u32(payload, &offset, event.parent_fault_sequence);
            put_u32(payload, &offset, event.arg0);
            put_u32(payload, &offset, event.arg1);
            put_u32(payload, &offset, event.arg2);
            break;
        }
        case odrive::trace::TraceEventKind::STATE: {
            *message_type = MessageType::STATE_EVENT;
            const auto& event = record.payload.state;
            put_u32(payload, &offset, event.sequence);
            put_u32(payload, &offset, event.state_epoch);
            put_u32(payload, &offset, event.timestamp_cycles);
            put_u32(payload, &offset, event.request_id);
            put_u8(payload, &offset, event.state);
            put_u8(payload, &offset, event.operation);
            put_u8(payload, &offset, event.axis_state);
            put_u8(payload, &offset,
                   static_cast<uint8_t>(event.type));
            put_u16(payload, &offset,
                    static_cast<uint16_t>(event.reason));
            break;
        }
        case odrive::trace::TraceEventKind::LOG: {
            *message_type = MessageType::LOG_RECORD;
            const auto& event = record.payload.log;
            put_u32(payload, &offset, event.sequence);
            put_u32(payload, &offset, event.timestamp_cycles);
            put_u32(payload, &offset, event.arg0);
            put_u32(payload, &offset, event.arg1);
            put_u16(payload, &offset, event.code);
            put_u8(payload, &offset,
                   static_cast<uint8_t>(event.level));
            put_u8(payload, &offset, event.category);
            break;
        }
        case odrive::trace::TraceEventKind::SCOPE: {
            *message_type = MessageType::SCOPE_RECORD;
            const auto& event = record.payload.scope;
            put_u32(payload, &offset, event.sequence);
            put_u32(payload, &offset, event.control_sequence);
            put_u32(payload, &offset, event.state_epoch);
            put_u32(payload, &offset, event.timestamp_cycles);
            put_f32(payload, &offset, event.phase);
            put_f32(payload, &offset, event.phase_velocity);
            put_f32(payload, &offset, event.position);
            put_f32(payload, &offset, event.velocity);
            put_f32(payload, &offset, event.id_measured);
            put_f32(payload, &offset, event.iq_measured);
            put_f32(payload, &offset, event.id_setpoint);
            put_f32(payload, &offset, event.iq_setpoint);
            put_f32(payload, &offset, event.torque_setpoint);
            put_f32(payload, &offset, event.controller_output);
            break;
        }
    }
    *written = offset;
    return offset <= capacity;
}

void UsbDebugTransport::publish_command_result(
        const odrive::safety::CommandResult& result) {
    if (result.source != odrive::safety::CommandSource::USB ||
        !binary_session_active_.load(std::memory_order_acquire)) {
        return;
    }
    if (result.request_id == pending_stop_request_id_ &&
        result.status != odrive::safety::CommandStatus::ACCEPTED) {
        test_session_.state = TestSessionState::ACTIVE;
        test_session_stop_confirmed_ =
            result.status == odrive::safety::CommandStatus::COMPLETED;
        pending_stop_request_id_ = 0u;
    }
    (void)queue_command_result(
        result.request_id, 0u,
        static_cast<uint8_t>(result.status),
        result.state_epoch,
        static_cast<uint16_t>(result.reason));
}

void UsbDebugTransport::publish_trace(
        const odrive::trace::TraceDispatchRecord& record) {
    if (!binary_session_active_.load(std::memory_order_acquire)) {
        return;
    }
    std::array<uint8_t, 64u> payload{};
    MessageType message_type = MessageType::LOG_RECORD;
    size_t written = 0u;
    if (!make_trace_payload(record, &message_type, payload.data(),
                            payload.size(), &written)) {
        return;
    }
    const uint8_t priority = record.kind == odrive::trace::TraceEventKind::CRITICAL
        ? kPriorityCritical
        : record.kind == odrive::trace::TraceEventKind::STATE
            ? kPriorityResultState
            : record.kind == odrive::trace::TraceEventKind::LOG
                ? kPriorityLog : kPriorityScope;
    (void)queue_frame(message_type, 0u, 0u, payload.data(), written, priority);
}

bool UsbDebugTransport::publish_calibration(uint16_t record_type,
                                            uint32_t sequence,
                                            const uint8_t* payload,
                                            size_t length) {
    if (payload == nullptr || length > kMaxPayloadSize - 8u) {
        ++stats_.tx_overflow_calibration;
        return false;
    }
    if (!binary_session_active_.load(std::memory_order_acquire)) {
        return false;
    }
    std::array<uint8_t, 8u> prefix{};
    size_t offset = 0u;
    put_u16(prefix.data(), &offset, record_type);
    put_u16(prefix.data(), &offset, static_cast<uint16_t>(length));
    put_u32(prefix.data(), &offset, sequence);
    return queue_frame_parts(MessageType::CALIBRATION_DATA, 0u, 0u,
                             prefix.data(), offset, payload, length,
                             kPriorityCalibration);
}

bool UsbDebugTransport::take_next_packet(TxPacket* packet) {
    if (packet == nullptr) {
        return false;
    }
    return critical_tx_ring_.pop(packet) ||
           result_state_tx_ring_.pop(packet) ||
           calibration_tx_ring_.pop(packet) ||
           scope_tx_ring_.pop(packet) ||
           log_tx_ring_.pop(packet);
}

bool UsbDebugTransport::begin_tx_chunk(uint16_t maximum_length,
                                       const uint8_t** data,
                                       uint16_t* length) {
    if (data == nullptr || length == nullptr || maximum_length == 0u) {
        return false;
    }
    if (!connected_.load(std::memory_order_acquire)) {
        ++stats_.tx_disconnected;
        return false;
    }
    if (!binary_session_active_.load(std::memory_order_acquire) &&
        !handshake_in_progress_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!active_tx_valid_) {
        if (handshake_tx_pending_) {
            // The handshake mailbox is already a stable task-owned buffer.
            // Transmit directly from it instead of copying a maximum-sized
            // TxPacket into the runtime active slot on the embedded stack path.
            active_tx_is_handshake_ = true;
        } else {
            if (!take_next_packet(&active_tx_)) {
                return false;
            }
            active_tx_is_handshake_ = false;
        }
        active_tx_offset_ = 0u;
        active_tx_valid_ = true;
    }
    const TxPacket& packet = active_tx_is_handshake_
        ? handshake_tx_ : active_tx_;
    const uint16_t remaining = packet.length - active_tx_offset_;
    *data = packet.bytes.data() + active_tx_offset_;
    *length = std::min(maximum_length, remaining);
    return true;
}

void UsbDebugTransport::complete_tx_chunk(uint16_t transmitted_length) {
    const TxPacket& packet = active_tx_is_handshake_
        ? handshake_tx_ : active_tx_;
    if (!active_tx_valid_ || transmitted_length == 0u ||
        transmitted_length > packet.length - active_tx_offset_) {
        return;
    }
    active_tx_offset_ += transmitted_length;
    if (active_tx_offset_ == packet.length) {
        const bool completed_handshake = active_tx_is_handshake_;
        active_tx_valid_ = false;
        active_tx_offset_ = 0u;
        active_tx_is_handshake_ = false;
        ++stats_.tx_frames;
        if (completed_handshake) {
            handshake_tx_pending_ = false;
            handshake_in_progress_.store(false, std::memory_order_release);
            binary_session_active_.store(true, std::memory_order_release);
        }
    }
}

void UsbDebugTransport::drop_disconnected_queues() {
    TxPacket packet;
    while (critical_tx_ring_.pop(&packet)) {}
    while (result_state_tx_ring_.pop(&packet)) {}
    while (calibration_tx_ring_.pop(&packet)) {}
    while (scope_tx_ring_.pop(&packet)) {}
    while (log_tx_ring_.pop(&packet)) {}
    handshake_tx_pending_ = false;
    handshake_in_progress_.store(false, std::memory_order_release);
    active_tx_valid_ = false;
    active_tx_offset_ = 0u;
    active_tx_is_handshake_ = false;
    crash_report_queued_ = false;
}

UsbDebugTransport& usb_debug_transport() {
    static UsbDebugTransport transport;
    return transport;
}

}  // namespace odrive::usb

extern "C" {

void usb_debug_bind_command_sink(void* context,
                                 odrive::usb::CommandSubmitFn callback) {
    odrive::usb::usb_debug_transport().bind_command_sink(context, callback);
}

void usb_debug_bind_clock(void* context, odrive::usb::ClockMsFn callback) {
    odrive::usb::usb_debug_transport().bind_clock(context, callback);
}

void usb_debug_bind_test_limits(void* context,
                                odrive::usb::TestLimitsApplyFn apply,
                                odrive::usb::TestLimitsRestoreFn restore) {
    odrive::usb::usb_debug_transport().bind_test_limits(
        context, apply, restore);
}

void usb_debug_bind_scope_engine(odrive::scope::CaptureEngine* engine) {
    odrive::usb::usb_debug_transport().bind_scope_engine(engine);
}

void usb_debug_receive_bytes(const uint8_t* data, size_t length) {
    odrive::usb::usb_debug_transport().receive_bytes(data, length);
}

void usb_debug_notify_connected(bool connected) {
    odrive::usb::usb_debug_transport().notify_connected(connected);
}

void usb_debug_service(void) {
    odrive::usb::usb_debug_transport().service();
}

void usb_debug_publish_command_result(
        const odrive::safety::CommandResult* result) {
    if (result != nullptr) {
        odrive::usb::usb_debug_transport().publish_command_result(*result);
    }
}

void usb_debug_publish_trace(
        const odrive::trace::TraceDispatchRecord* record) {
    if (record != nullptr) {
        odrive::usb::usb_debug_transport().publish_trace(*record);
    }
}

bool usb_debug_publish_calibration(uint16_t record_type, uint32_t sequence,
                                   const uint8_t* payload, size_t length) {
    return odrive::usb::usb_debug_transport().publish_calibration(
        record_type, sequence, payload, length);
}

bool usb_debug_begin_tx_chunk(uint16_t maximum_length, const uint8_t** data,
                              uint16_t* length) {
    return odrive::usb::usb_debug_transport().begin_tx_chunk(
        maximum_length, data, length);
}

void usb_debug_complete_tx_chunk(uint16_t transmitted_length) {
    odrive::usb::usb_debug_transport().complete_tx_chunk(transmitted_length);
}

bool usb_debug_binary_session_active(void) {
    return odrive::usb::usb_debug_transport().binary_session_active();
}

}  // extern "C"
