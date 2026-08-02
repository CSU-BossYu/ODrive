#ifndef ODRIVE_SCOPE_CAPTURE_HPP
#define ODRIVE_SCOPE_CAPTURE_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "realtime_snapshot.hpp"
#include "scope_channels_generated.hpp"

namespace odrive::scope {

constexpr uint32_t kControlLoopRateHz = 10000u;
constexpr size_t kCaptureBufferCapacity =
    generated::kMaxPreSamples + generated::kMaxPostSamples;

enum class CaptureMode : uint8_t { CONTINUOUS = 0, TRIGGERED = 1 };
enum class TriggerType : uint8_t {
    NONE = 0, MANUAL = 1, FAULT = 2, STATE = 3, THRESHOLD = 4
};
enum class TriggerEdge : uint8_t { RISING = 0, FALLING = 1 };
enum class CaptureState : uint8_t {
    IDLE = 0, CONFIGURED = 1, ARMED = 2, COMPLETE = 3, STOPPED = 4, ERROR = 5
};
enum class CaptureFinalReason : uint8_t {
    NONE = 0, MANUAL = 1, FAULT = 2, STATE = 3, THRESHOLD = 4,
    STOPPED = 5, OVERFLOW = 6, INVALID_CONFIG = 7
};

enum ScopeSampleFlags : uint16_t {
    SAMPLE_SEQUENCE_GAP = 1u << 0,
    SAMPLE_DROPPED = 1u << 1,
};
enum ScopeBatchFlags : uint8_t {
    BATCH_SEQUENCE_GAP = 1u << 0,
    BATCH_DROPPED = 1u << 1,
    BATCH_COMPLETE = 1u << 2,
};

struct ScopeConfig {
    CaptureMode mode = CaptureMode::CONTINUOUS;
    TriggerType trigger_type = TriggerType::NONE;
    uint16_t trigger_channel = 0xffffu;
    TriggerEdge trigger_edge = TriggerEdge::RISING;
    uint8_t trigger_state = 0u;
    uint32_t sample_rate_hz = 1000u;
    uint16_t decimation = 10u;
    uint16_t pre_samples = 0u;
    uint16_t post_samples = 0u;
    uint8_t channel_count = 0u;
    float threshold = 0.0f;
    std::array<uint16_t, generated::kMaxCaptureChannels> channel_ids{};
};

struct CapturePlan {
    ScopeConfig config{};
    uint32_t capture_id = 0u;
};

enum class ValidationError : uint8_t {
    NONE = 0, INVALID_MODE, INVALID_TRIGGER, INVALID_RATE,
    INVALID_DECIMATION, INVALID_PRE_SAMPLES, INVALID_POST_SAMPLES,
    TOO_MANY_CHANNELS, UNKNOWN_CHANNEL, DUPLICATE_CHANNEL,
    THRESHOLD_NOT_ALLOWED, TRIGGER_CHANNEL_NOT_SELECTED,
    CAPTURE_TOO_LARGE, ARMED
};

bool build_capture_plan(const ScopeConfig& config, uint32_t capture_id,
                        CapturePlan* plan, ValidationError* error);

struct ScopeSample {
    uint32_t control_sequence = 0u;
    uint32_t timestamp_cycles = 0u;
    uint16_t flags = 0u;
    std::array<uint32_t, generated::kMaxCaptureChannels> values{};
};

struct ScopeBatch {
    uint32_t capture_id = 0u;
    uint16_t sample_count = 0u;
    uint8_t channel_count = 0u;
    uint8_t flags = 0u;
    uint16_t dropped_samples = 0u;
    uint32_t first_sequence = 0u;
    uint32_t first_timestamp_cycles = 0u;
    std::array<uint16_t, generated::kMaxCaptureChannels> channel_ids{};
    std::array<ScopeSample, generated::kMaxBatchSamples> samples{};
};

struct ScopeStatus {
    uint32_t capture_id = 0u;
    CaptureState state = CaptureState::IDLE;
    CaptureFinalReason final_reason = CaptureFinalReason::NONE;
    CaptureMode mode = CaptureMode::CONTINUOUS;
    TriggerType trigger_type = TriggerType::NONE;
    uint32_t sample_count = 0u;
    uint32_t dropped_samples = 0u;
    uint32_t sequence_gaps = 0u;
    uint8_t channel_count = 0u;
    uint16_t decimation = 0u;
    uint32_t first_timestamp_cycles = 0u;
    uint32_t last_timestamp_cycles = 0u;
};

class CaptureEngine {
public:
    CaptureEngine();

    // These methods are task-context only. Validation completes before the
    // immutable plan enters the triple-buffer mailbox consumed by the ISR.
    ValidationError configure_from_task(const ScopeConfig& config,
                                         uint32_t* capture_id);
    bool arm_from_task();
    bool stop_from_task();
    bool trigger_manual_from_task();
    bool read_status(ScopeStatus* status) const;
    bool read_batch(ScopeBatch* batch);

    // The control ISR calls this once per control cycle. It only reads the
    // validated plan and fixed whitelist readers generated from the schema.
    void sample_from_isr(const generated::ScopeSampleContext& sample);
    void trigger_fault_from_isr();
    void notify_state_from_task(uint8_t state);

    static constexpr size_t capture_buffer_bytes() {
        return sizeof(ScopeSample) * kCaptureBufferCapacity;
    }

private:
    using PlanBuffer = odrive::trace::FixedTripleBuffer<CapturePlan>;

    bool trigger_now_from_isr(const generated::ScopeSampleContext& sample,
                              uint32_t* reason);
    void reset_capture_from_task();

    PlanBuffer plan_mailbox_;
    std::atomic<uint32_t> plan_generation_{0u};
    std::atomic<bool> armed_{false};
    std::atomic<CaptureState> state_{CaptureState::IDLE};
    std::atomic<CaptureFinalReason> final_reason_{CaptureFinalReason::NONE};
    std::atomic<uint32_t> capture_id_{0u};
    std::atomic<uint32_t> write_count_{0u};
    std::atomic<uint32_t> read_cursor_{0u};
    std::atomic<uint32_t> capture_start_ordinal_{0u};
    std::atomic<uint32_t> capture_sample_count_{0u};
    std::atomic<uint32_t> dropped_samples_{0u};
    std::atomic<uint32_t> sequence_gaps_{0u};
    std::atomic<uint32_t> first_timestamp_cycles_{0u};
    std::atomic<uint32_t> last_timestamp_cycles_{0u};
    std::array<ScopeSample, kCaptureBufferCapacity> samples_{};
    std::array<std::atomic<uint32_t>, kCaptureBufferCapacity> committed_{};

    // Pending trigger flags are atomic because manual/state requests arrive
    // from task context while fault requests may arrive in the control ISR.
    std::atomic<bool> manual_trigger_{false};
    std::atomic<bool> fault_trigger_{false};
    std::atomic<bool> state_trigger_{false};

    // The following fields are single-writer ISR state or task-only state.
    CapturePlan active_plan_{};
    CapturePlan configured_plan_{};
    uint32_t active_generation_ = 0u;
    uint32_t last_stored_control_sequence_ = 0u;
    bool have_last_stored_sequence_ = false;
    bool triggered_ = false;
    uint32_t post_remaining_ = 0u;
    uint8_t last_task_state_ = 0xffu;
    bool batch_cursor_initialized_ = false;
    uint32_t next_capture_id_ = 0u;
    float threshold_previous_value_ = 0.0f;
    bool threshold_have_previous_ = false;
};

}  // namespace odrive::scope

#endif
