#ifndef ODRIVE_USB_DEBUG_TRANSPORT_HPP
#define ODRIVE_USB_DEBUG_TRANSPORT_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "MotorControl/realtime_event_ring.hpp"
#include "MotorControl/safety_supervisor.hpp"
#include "MotorControl/trace_rings.hpp"
#include "usb_debug_protocol.hpp"

namespace odrive::usb {

enum class RequestDecision : uint8_t {
    ACCEPT = 0u,
    DUPLICATE = 1u,
    EXPIRED = 2u,
};

class RequestTracker {
public:
    static constexpr size_t kCapacity = 16u;
    static constexpr uint32_t kSequenceWindow = 128u;

    RequestDecision accept(uint32_t request_id, uint32_t sequence);
    void reset();

private:
    struct Entry {
        uint32_t request_id = 0u;
        uint32_t sequence = 0u;
        bool valid = false;
    };

    std::array<Entry, kCapacity> entries_ = {};
    uint32_t highest_sequence_ = 0u;
    bool have_sequence_ = false;
};

struct UsbDebugStats {
    std::atomic<uint32_t> rx_bytes{0u};
    std::atomic<uint32_t> rx_overflow{0u};
    std::atomic<uint32_t> tx_frames{0u};
    std::atomic<uint32_t> tx_busy{0u};
    std::atomic<uint32_t> tx_disconnected{0u};
    std::atomic<uint32_t> tx_overflow_critical{0u};
    std::atomic<uint32_t> tx_overflow_result_state{0u};
    std::atomic<uint32_t> tx_overflow_calibration{0u};
    std::atomic<uint32_t> tx_overflow_scope{0u};
    std::atomic<uint32_t> tx_overflow_log{0u};
    std::atomic<uint32_t> rejected_commands{0u};
    std::atomic<uint32_t> duplicate_requests{0u};
    std::atomic<uint32_t> expired_requests{0u};
};

using CommandSubmitFn = bool (*)(void* context,
                                  const odrive::safety::Command& command);
using ClockMsFn = uint32_t (*)(void* context);
using TestLimitsApplyFn = bool (*)(void* context, float velocity_limit,
                                   float current_limit, float torque_limit);
using TestLimitsRestoreFn = void (*)(void* context);

enum class TestSessionState : uint8_t {
    INACTIVE = 0u,
    ACTIVE = 1u,
    STOPPING = 2u,
    EXPIRED = 3u,
};

struct TestSessionStatus {
    TestSessionState state = TestSessionState::INACTIVE;
    uint8_t flags = 0u;
    uint16_t reason = 0u;
    uint32_t session_id = 0u;
    uint32_t lease_remaining_ms = 0u;
    float velocity_limit = 0.0f;
    float current_limit = 0.0f;
    float torque_limit = 0.0f;
};

class UsbDebugTransport {
public:
    static constexpr size_t kRxByteCapacity = 1024u;
    static constexpr size_t kCommandCapacity = 8u;
    static constexpr size_t kCriticalTxCapacity = 8u;
    static constexpr size_t kResultStateTxCapacity = 16u;
    static constexpr size_t kCalibrationTxCapacity = 8u;
    static constexpr size_t kScopeTxCapacity = 8u;
    static constexpr size_t kLogTxCapacity = 16u;

    UsbDebugTransport();

    void bind_command_sink(void* context, CommandSubmitFn callback);
    void bind_clock(void* context, ClockMsFn callback) {
        clock_context_ = context;
        clock_ms_ = callback;
    }
    void bind_test_limits(void* context, TestLimitsApplyFn apply,
                          TestLimitsRestoreFn restore) {
        test_limits_context_ = context;
        test_limits_apply_ = apply;
        test_limits_restore_ = restore;
    }
    void bind_scope_engine(odrive::scope::CaptureEngine* engine) {
        scope_engine_ = engine;
    }
    void receive_bytes(const uint8_t* data, size_t length);
    void notify_connected(bool connected);
    void service();

    void publish_command_result(
        const odrive::safety::CommandResult& result);
    void publish_trace(const odrive::trace::TraceDispatchRecord& record);
    bool publish_calibration(uint16_t record_type, uint32_t sequence,
                             const uint8_t* payload, size_t length);

    bool begin_tx_chunk(uint16_t maximum_length, const uint8_t** data,
                        uint16_t* length);
    void complete_tx_chunk(uint16_t transmitted_length);
    bool binary_session_active() const {
        return binary_session_active_.load(std::memory_order_acquire);
    }
    const UsbDebugStats& stats() const { return stats_; }
    const ParserStats& parser_stats() const { return parser_.stats(); }
    TestSessionStatus test_session_status() const;

private:
    struct CommandEnvelope {
        uint32_t request_id = 0u;
        uint32_t sequence = 0u;
        CommandPayload payload{};
    };

    struct TxPacket {
        uint16_t length = 0u;
        std::array<uint8_t, kMaxFrameSize> bytes = {};
    };

    using RxRing = odrive::safety::FixedSpscRing<uint8_t,
                                                  kRxByteCapacity + 1u>;
    using CommandRing = odrive::safety::FixedSpscRing<CommandEnvelope,
                                                       kCommandCapacity + 1u>;
    using CriticalTxRing = odrive::trace::FixedMpscRing<
        TxPacket, kCriticalTxCapacity>;
    using ResultStateTxRing = odrive::trace::FixedMpscRing<
        TxPacket, kResultStateTxCapacity>;
    using CalibrationTxRing = odrive::trace::FixedMpscRing<
        TxPacket, kCalibrationTxCapacity>;
    using ScopeTxRing = odrive::trace::FixedMpscRing<TxPacket,
                                                     kScopeTxCapacity>;
    using LogTxRing = odrive::trace::FixedMpscRing<TxPacket, kLogTxCapacity>;

    static void parsed_frame(void* context, const Frame& frame);
    void handle_frame(const Frame& frame);
    void drain_commands();
    bool queue_frame(MessageType message_type, uint8_t flags,
                     uint32_t request_id, const uint8_t* payload,
                     size_t payload_length, uint8_t priority);
    bool queue_frame_parts(MessageType message_type, uint8_t flags,
                           uint32_t request_id, const uint8_t* prefix,
                           size_t prefix_length, const uint8_t* payload,
                           size_t payload_length, uint8_t priority);
    bool queue_command_result(uint32_t request_id, uint32_t sequence,
                              uint8_t status, uint32_t state_epoch,
                              uint16_t reason);
    bool queue_capabilities(uint32_t request_id, uint32_t sequence);
    bool queue_stats(uint32_t request_id, uint32_t sequence);
    bool queue_scope_status(uint32_t request_id,
                            const odrive::scope::ScopeStatus& status,
                            uint8_t flags = FRAME_RESPONSE);
    bool queue_test_session_status(uint32_t request_id,
                                   uint8_t flags = FRAME_RESPONSE,
                                   uint16_t reason = 0u);
    bool begin_test_session(const Frame& frame);
    bool refresh_test_session(const Frame& frame);
    bool stop_test_session(const Frame& frame);
    bool end_test_session(const Frame& frame);
    void expire_test_session();
    void force_session_disarm();
    uint32_t now_ms() const;
    void service_scope_stream();
    void service_crash_report();
    bool make_trace_payload(const odrive::trace::TraceDispatchRecord& record,
                            MessageType* message_type, uint8_t* payload,
                            size_t capacity, size_t* written);
    static uint16_t reason_for_decision(RequestDecision decision);
    static uint16_t reason_for_queue_failure();
    static uint16_t reason_for_bad_payload();
    static bool is_usb_command(uint8_t value);
    void drop_disconnected_queues();
    bool take_next_packet(TxPacket* packet);

    RxRing rx_ring_;
    CommandRing command_ring_;
    CriticalTxRing critical_tx_ring_;
    ResultStateTxRing result_state_tx_ring_;
    CalibrationTxRing calibration_tx_ring_;
    ScopeTxRing scope_tx_ring_;
    LogTxRing log_tx_ring_;
    // HELLO has exactly one producer and must not contend with runtime
    // telemetry. Reserve a task-owned mailbox so CAPABILITIES cannot be
    // starved by or coupled to the MPSC event rings.
    TxPacket handshake_tx_{};
    bool handshake_tx_pending_ = false;
    TxPacket active_tx_{};
    uint16_t active_tx_offset_ = 0u;
    bool active_tx_valid_ = false;
    bool active_tx_is_handshake_ = false;
    std::atomic<bool> connected_{false};
    std::atomic<bool> binary_session_active_{false};
    std::atomic<bool> handshake_in_progress_{false};
    StreamParser parser_;
    RequestTracker request_tracker_;
    void* command_context_ = nullptr;
    CommandSubmitFn command_submit_ = nullptr;
    void* clock_context_ = nullptr;
    ClockMsFn clock_ms_ = nullptr;
    void* test_limits_context_ = nullptr;
    TestLimitsApplyFn test_limits_apply_ = nullptr;
    TestLimitsRestoreFn test_limits_restore_ = nullptr;
    odrive::scope::CaptureEngine* scope_engine_ = nullptr;
    odrive::scope::ScopeBatch pending_scope_batch_{};
    bool pending_scope_batch_valid_ = false;
    bool crash_report_queued_ = false;
    TestSessionStatus test_session_{};
    uint32_t test_session_generation_ = 0u;
    uint32_t test_session_lease_ms_ = 0u;
    uint32_t test_session_deadline_ms_ = 0u;
    uint32_t pending_stop_request_id_ = 0u;
    bool test_session_stop_confirmed_ = true;
    std::atomic<uint32_t> tx_sequence_{0u};
    UsbDebugStats stats_{};

    uint32_t next_tx_sequence() {
        return tx_sequence_.fetch_add(1u, std::memory_order_relaxed) + 1u;
    }
};

UsbDebugTransport& usb_debug_transport();

}  // namespace odrive::usb

extern "C" {
void usb_debug_bind_command_sink(void* context,
                                 odrive::usb::CommandSubmitFn callback);
void usb_debug_bind_clock(void* context, odrive::usb::ClockMsFn callback);
void usb_debug_bind_test_limits(void* context,
                                odrive::usb::TestLimitsApplyFn apply,
                                odrive::usb::TestLimitsRestoreFn restore);
void usb_debug_bind_scope_engine(odrive::scope::CaptureEngine* engine);
void usb_debug_receive_bytes(const uint8_t* data, size_t length);
void usb_debug_notify_connected(bool connected);
void usb_debug_service(void);
void usb_debug_publish_command_result(
    const odrive::safety::CommandResult* result);
void usb_debug_publish_trace(
    const odrive::trace::TraceDispatchRecord* record);
bool usb_debug_publish_calibration(uint16_t record_type, uint32_t sequence,
                                   const uint8_t* payload, size_t length);
bool usb_debug_begin_tx_chunk(uint16_t maximum_length, const uint8_t** data,
                              uint16_t* length);
void usb_debug_complete_tx_chunk(uint16_t transmitted_length);
bool usb_debug_binary_session_active(void);
}

#endif
