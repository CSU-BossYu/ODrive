#include "scope_capture.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace odrive::scope {

namespace {

uint32_t ceil_div(uint32_t numerator, uint32_t denominator) {
    return (numerator + denominator - 1u) / denominator;
}

float channel_as_float(const generated::ScopeChannelDefinition& definition,
                       const generated::ScopeChannelValue& value) {
    if (definition.wire_type == generated::ScopeWireDataType::F32) {
        float result = 0.0f;
        static_assert(sizeof(result) == sizeof(value.raw), "f32 wire size");
        std::memcpy(&result, &value.raw, sizeof(result));
        return result;
    }
    return static_cast<float>(value.raw);
}

}  // namespace

bool build_capture_plan(const ScopeConfig& config, uint32_t capture_id,
                        CapturePlan* plan, ValidationError* error) {
    if (plan == nullptr || error == nullptr) {
        return false;
    }
    *error = ValidationError::NONE;
    if (config.mode != CaptureMode::CONTINUOUS &&
        config.mode != CaptureMode::TRIGGERED) {
        *error = ValidationError::INVALID_MODE;
        return false;
    }
    if (config.trigger_type > TriggerType::THRESHOLD ||
        (config.mode == CaptureMode::CONTINUOUS &&
         config.trigger_type != TriggerType::NONE)) {
        *error = ValidationError::INVALID_TRIGGER;
        return false;
    }
    if (config.mode == CaptureMode::TRIGGERED &&
        (config.trigger_type == TriggerType::NONE || config.post_samples == 0u)) {
        *error = ValidationError::INVALID_TRIGGER;
        return false;
    }
    if (config.sample_rate_hz == 0u ||
        config.sample_rate_hz > kControlLoopRateHz) {
        *error = ValidationError::INVALID_RATE;
        return false;
    }
    if (config.decimation == 0u ||
        config.decimation < ceil_div(kControlLoopRateHz, config.sample_rate_hz)) {
        *error = ValidationError::INVALID_DECIMATION;
        return false;
    }
    if (config.channel_count == 0u ||
        config.channel_count > generated::kMaxCaptureChannels) {
        *error = ValidationError::TOO_MANY_CHANNELS;
        return false;
    }
    if (config.pre_samples > generated::kMaxPreSamples) {
        *error = ValidationError::INVALID_PRE_SAMPLES;
        return false;
    }
    if (config.post_samples > generated::kMaxPostSamples) {
        *error = ValidationError::INVALID_POST_SAMPLES;
        return false;
    }
    if (config.mode == CaptureMode::CONTINUOUS &&
        (config.pre_samples != 0u || config.post_samples != 0u)) {
        *error = ValidationError::INVALID_PRE_SAMPLES;
        return false;
    }
    if (config.mode == CaptureMode::TRIGGERED &&
        static_cast<size_t>(config.pre_samples) + config.post_samples >
            kCaptureBufferCapacity) {
        *error = ValidationError::CAPTURE_TOO_LARGE;
        return false;
    }

    bool trigger_channel_selected = false;
    for (size_t index = 0; index < config.channel_count; ++index) {
        const uint16_t id = config.channel_ids[index];
        const auto* definition = generated::channel(id);
        if (definition == nullptr) {
            *error = ValidationError::UNKNOWN_CHANNEL;
            return false;
        }
        if (config.sample_rate_hz > definition->max_sample_rate_hz) {
            *error = ValidationError::INVALID_RATE;
            return false;
        }
        for (size_t prior = 0; prior < index; ++prior) {
            if (config.channel_ids[prior] == id) {
                *error = ValidationError::DUPLICATE_CHANNEL;
                return false;
            }
        }
        if (id == config.trigger_channel) {
            trigger_channel_selected = true;
            if (!definition->threshold_allowed &&
                config.trigger_type == TriggerType::THRESHOLD) {
                *error = ValidationError::THRESHOLD_NOT_ALLOWED;
                return false;
            }
        }
    }
    if (config.trigger_type == TriggerType::THRESHOLD &&
        (!trigger_channel_selected || config.trigger_channel == 0xffffu)) {
        *error = ValidationError::TRIGGER_CHANNEL_NOT_SELECTED;
        return false;
    }
    *plan = {config, capture_id};
    return true;
}

CaptureEngine::CaptureEngine() {
    for (auto& item : committed_) {
        item.store(0u, std::memory_order_relaxed);
    }
}

void CaptureEngine::reset_capture_from_task() {
    write_count_.store(0u, std::memory_order_release);
    read_cursor_.store(0u, std::memory_order_release);
    capture_start_ordinal_.store(0u, std::memory_order_release);
    capture_sample_count_.store(0u, std::memory_order_release);
    dropped_samples_.store(0u, std::memory_order_release);
    sequence_gaps_.store(0u, std::memory_order_release);
    first_timestamp_cycles_.store(0u, std::memory_order_release);
    last_timestamp_cycles_.store(0u, std::memory_order_release);
    for (auto& item : committed_) item.store(0u, std::memory_order_release);
    manual_trigger_.store(false, std::memory_order_release);
    fault_trigger_.store(false, std::memory_order_release);
    state_trigger_.store(false, std::memory_order_release);
    triggered_ = false;
    post_remaining_ = 0u;
    last_stored_control_sequence_ = 0u;
    have_last_stored_sequence_ = false;
    batch_cursor_initialized_ = false;
}

ValidationError CaptureEngine::configure_from_task(const ScopeConfig& config,
                                                   uint32_t* capture_id) {
    if (armed_.load(std::memory_order_acquire)) {
        return ValidationError::ARMED;
    }
    CapturePlan plan;
    ValidationError error = ValidationError::NONE;
    ++next_capture_id_;
    if (next_capture_id_ == 0u) ++next_capture_id_;
    if (!build_capture_plan(config, next_capture_id_, &plan, &error)) {
        state_.store(CaptureState::ERROR, std::memory_order_release);
        final_reason_.store(CaptureFinalReason::INVALID_CONFIG,
                            std::memory_order_release);
        return error;
    }
    configured_plan_ = plan;
    reset_capture_from_task();
    capture_id_.store(plan.capture_id, std::memory_order_release);
    plan_mailbox_.publish(plan);
    plan_generation_.fetch_add(1u, std::memory_order_release);
    state_.store(CaptureState::CONFIGURED, std::memory_order_release);
    final_reason_.store(CaptureFinalReason::NONE, std::memory_order_release);
    if (capture_id != nullptr) *capture_id = plan.capture_id;
    return ValidationError::NONE;
}

bool CaptureEngine::arm_from_task() {
    if (configured_plan_.capture_id == 0u ||
        state_.load(std::memory_order_acquire) == CaptureState::ERROR) {
        return false;
    }
    reset_capture_from_task();
    armed_.store(true, std::memory_order_release);
    state_.store(CaptureState::ARMED, std::memory_order_release);
    final_reason_.store(CaptureFinalReason::NONE, std::memory_order_release);
    return true;
}

bool CaptureEngine::stop_from_task() {
    const bool was_armed = armed_.exchange(false, std::memory_order_acq_rel);
    state_.store(CaptureState::STOPPED, std::memory_order_release);
    final_reason_.store(CaptureFinalReason::STOPPED, std::memory_order_release);
    return was_armed || configured_plan_.capture_id != 0u;
}

bool CaptureEngine::trigger_manual_from_task() {
    if (!armed_.load(std::memory_order_acquire) ||
        configured_plan_.config.trigger_type != TriggerType::MANUAL) {
        return false;
    }
    manual_trigger_.store(true, std::memory_order_release);
    return true;
}

void CaptureEngine::trigger_fault_from_isr() {
    // The immutable active plan is copied on the first sample after ARM. A
    // fault can arrive in that small window, so only gate on ARM here; the
    // sampling path consumes this flag only for a FAULT-triggered plan.
    if (armed_.load(std::memory_order_acquire)) {
        fault_trigger_.store(true, std::memory_order_release);
    }
}

void CaptureEngine::notify_state_from_task(uint8_t state) {
    if (state != last_task_state_ &&
        configured_plan_.config.trigger_type == TriggerType::STATE &&
        state == configured_plan_.config.trigger_state) {
        state_trigger_.store(true, std::memory_order_release);
    }
    last_task_state_ = state;
}

bool CaptureEngine::trigger_now_from_isr(
        const generated::ScopeSampleContext& sample, uint32_t* reason) {
    if (reason == nullptr) return false;
    *reason = static_cast<uint32_t>(CaptureFinalReason::NONE);
    switch (active_plan_.config.trigger_type) {
        case TriggerType::MANUAL:
            if (manual_trigger_.load(std::memory_order_acquire)) {
                *reason = static_cast<uint32_t>(CaptureFinalReason::MANUAL);
                return true;
            }
            break;
        case TriggerType::FAULT:
            if (fault_trigger_.load(std::memory_order_acquire)) {
                *reason = static_cast<uint32_t>(CaptureFinalReason::FAULT);
                return true;
            }
            break;
        case TriggerType::STATE:
            if (state_trigger_.load(std::memory_order_acquire)) {
                *reason = static_cast<uint32_t>(CaptureFinalReason::STATE);
                return true;
            }
            break;
        case TriggerType::THRESHOLD: {
            generated::ScopeChannelValue raw;
            const auto* definition = generated::channel(
                active_plan_.config.trigger_channel);
            if (definition != nullptr && generated::read_channel(
                    sample, active_plan_.config.trigger_channel, &raw)) {
                const float current = channel_as_float(*definition, raw);
                const float threshold = active_plan_.config.threshold;
                const float previous = threshold_previous_value_;
                const bool had_previous = threshold_have_previous_;
                const bool crossed = active_plan_.config.trigger_edge ==
                    TriggerEdge::RISING
                    ? had_previous && previous < threshold && current >= threshold
                    : had_previous && previous > threshold && current <= threshold;
                threshold_previous_value_ = current;
                threshold_have_previous_ = true;
                if (threshold_have_previous_ && crossed) {
                    *reason = static_cast<uint32_t>(CaptureFinalReason::THRESHOLD);
                    return true;
                }
            }
            break;
        }
        case TriggerType::NONE:
            break;
    }
    return false;
}

void CaptureEngine::sample_from_isr(
        const generated::ScopeSampleContext& sample) {
    if (!armed_.load(std::memory_order_acquire)) return;
    const uint32_t generation = plan_generation_.load(std::memory_order_acquire);
    if (generation != active_generation_) {
        CapturePlan plan;
        if (!plan_mailbox_.read(&plan)) return;
        active_plan_ = plan;
        active_generation_ = generation;
        threshold_have_previous_ = false;
        threshold_previous_value_ = 0.0f;
    }
    if (active_plan_.config.decimation == 0u ||
        (sample.control_sequence % active_plan_.config.decimation) != 0u) {
        return;
    }

    const uint32_t ordinal = write_count_.load(std::memory_order_relaxed);
    const uint32_t cursor = read_cursor_.load(std::memory_order_acquire);
    // Overwriting old pre-trigger history is the intended rolling-snapshot
    // behavior. Only a continuous consumer that falls behind has lost data.
    if (active_plan_.config.mode == CaptureMode::CONTINUOUS &&
        ordinal - cursor >= kCaptureBufferCapacity) {
        dropped_samples_.fetch_add(1u, std::memory_order_relaxed);
    }

    ScopeSample captured;
    captured.control_sequence = sample.control_sequence;
    captured.timestamp_cycles = sample.timestamp_cycles;
    if (have_last_stored_sequence_ &&
        static_cast<uint32_t>(sample.control_sequence -
                              last_stored_control_sequence_) !=
            active_plan_.config.decimation) {
        captured.flags |= SAMPLE_SEQUENCE_GAP;
        sequence_gaps_.fetch_add(1u, std::memory_order_relaxed);
    }
    for (size_t index = 0; index < active_plan_.config.channel_count; ++index) {
        generated::ScopeChannelValue value;
        if (generated::read_channel(sample,
                                    active_plan_.config.channel_ids[index],
                                    &value)) {
            captured.values[index] = value.raw;
        }
    }
    samples_[ordinal % kCaptureBufferCapacity] = captured;
    committed_[ordinal % kCaptureBufferCapacity].store(
        ordinal + 1u, std::memory_order_release);
    write_count_.store(ordinal + 1u, std::memory_order_release);
    if (ordinal == 0u) {
        first_timestamp_cycles_.store(sample.timestamp_cycles,
                                      std::memory_order_release);
    }
    last_timestamp_cycles_.store(sample.timestamp_cycles,
                                  std::memory_order_release);
    last_stored_control_sequence_ = sample.control_sequence;
    have_last_stored_sequence_ = true;

    uint32_t trigger_reason = static_cast<uint32_t>(CaptureFinalReason::NONE);
    const bool trigger = !triggered_ &&
        active_plan_.config.mode == CaptureMode::TRIGGERED &&
        trigger_now_from_isr(sample, &trigger_reason);
    if (trigger) {
        triggered_ = true;
        manual_trigger_.store(false, std::memory_order_release);
        fault_trigger_.store(false, std::memory_order_release);
        state_trigger_.store(false, std::memory_order_release);
        const uint32_t before = ordinal;
        const uint32_t pre = std::min<uint32_t>(
            before, active_plan_.config.pre_samples);
        const uint32_t capture_start = ordinal - pre;
        capture_start_ordinal_.store(capture_start,
                                     std::memory_order_release);
        first_timestamp_cycles_.store(
            samples_[capture_start % kCaptureBufferCapacity].timestamp_cycles,
            std::memory_order_release);
        post_remaining_ = active_plan_.config.post_samples;
        final_reason_.store(static_cast<CaptureFinalReason>(trigger_reason),
                            std::memory_order_release);
    }
    if (active_plan_.config.mode == CaptureMode::CONTINUOUS) {
        capture_sample_count_.store(
            std::min<uint32_t>(ordinal + 1u, kCaptureBufferCapacity),
            std::memory_order_release);
    } else if (triggered_) {
        const uint32_t start = capture_start_ordinal_.load(
            std::memory_order_acquire);
        capture_sample_count_.store(ordinal + 1u - start,
                                    std::memory_order_release);
        if (post_remaining_ > 0u) --post_remaining_;
        if (post_remaining_ == 0u) {
            armed_.store(false, std::memory_order_release);
            state_.store(CaptureState::COMPLETE, std::memory_order_release);
            capture_sample_count_.store(ordinal + 1u - start,
                                        std::memory_order_release);
        }
    }
}

bool CaptureEngine::read_status(ScopeStatus* status) const {
    if (status == nullptr) return false;
    status->capture_id = capture_id_.load(std::memory_order_acquire);
    status->state = state_.load(std::memory_order_acquire);
    status->final_reason = final_reason_.load(std::memory_order_acquire);
    status->mode = configured_plan_.config.mode;
    status->trigger_type = configured_plan_.config.trigger_type;
    status->sample_count = capture_sample_count_.load(std::memory_order_acquire);
    status->dropped_samples = dropped_samples_.load(std::memory_order_acquire);
    status->sequence_gaps = sequence_gaps_.load(std::memory_order_acquire);
    status->channel_count = configured_plan_.config.channel_count;
    status->decimation = configured_plan_.config.decimation;
    status->first_timestamp_cycles = first_timestamp_cycles_.load(
        std::memory_order_acquire);
    status->last_timestamp_cycles = last_timestamp_cycles_.load(
        std::memory_order_acquire);
    return true;
}

bool CaptureEngine::read_batch(ScopeBatch* batch) {
    if (batch == nullptr) return false;
    const CaptureState current_state = state_.load(std::memory_order_acquire);
    if (current_state == CaptureState::IDLE ||
        current_state == CaptureState::ERROR) return false;
    // Triggered captures are snapshots. Exposing the live pre-trigger ring
    // before completion advances read_cursor_, then COMPLETE rewinds it to the
    // snapshot start and sends duplicate, out-of-order samples. Continuous
    // mode remains streamable while armed.
    if (configured_plan_.config.mode == CaptureMode::TRIGGERED &&
        current_state != CaptureState::COMPLETE) {
        return false;
    }
    if (current_state == CaptureState::COMPLETE && !batch_cursor_initialized_) {
        read_cursor_.store(capture_start_ordinal_.load(std::memory_order_acquire),
                           std::memory_order_release);
        batch_cursor_initialized_ = true;
    }
    batch->capture_id = capture_id_.load(std::memory_order_acquire);
    batch->channel_count = configured_plan_.config.channel_count;
    batch->channel_ids = configured_plan_.config.channel_ids;
    batch->sample_count = 0u;
    batch->flags = current_state == CaptureState::COMPLETE
        ? static_cast<uint8_t>(BATCH_COMPLETE) : 0u;
    batch->dropped_samples = 0u;
    const uint32_t newest = write_count_.load(std::memory_order_acquire);
    uint32_t cursor = read_cursor_.load(std::memory_order_acquire);
    if (newest - cursor > kCaptureBufferCapacity) {
        const uint32_t skipped = newest - kCaptureBufferCapacity - cursor;
        cursor = newest - kCaptureBufferCapacity;
        batch->flags |= BATCH_DROPPED;
        batch->dropped_samples = static_cast<uint16_t>(
            std::min<uint32_t>(skipped, 0xffffu));
    }
    const uint32_t start_cursor = cursor;
    while (batch->sample_count < generated::kMaxBatchSamples &&
           cursor < newest) {
        const size_t slot = cursor % kCaptureBufferCapacity;
        if (committed_[slot].load(std::memory_order_acquire) != cursor + 1u) {
            batch->flags |= BATCH_DROPPED;
            ++batch->dropped_samples;
            ++cursor;
            continue;
        }
        batch->samples[batch->sample_count++] = samples_[slot];
        ++cursor;
    }
    if (batch->sample_count == 0u) return false;
    batch->first_sequence = batch->samples[0].control_sequence;
    batch->first_timestamp_cycles = batch->samples[0].timestamp_cycles;
    for (size_t index = 0; index < batch->sample_count; ++index) {
        if ((batch->samples[index].flags & SAMPLE_SEQUENCE_GAP) != 0u) {
            batch->flags |= BATCH_SEQUENCE_GAP;
        }
    }
    read_cursor_.store(cursor, std::memory_order_release);
    (void)start_cursor;
    return true;
}

}  // namespace odrive::scope
