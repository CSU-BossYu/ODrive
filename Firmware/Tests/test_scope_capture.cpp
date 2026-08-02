#include <doctest.h>

#include "MotorControl/scope_capture.hpp"

using namespace odrive::scope;

namespace {

ScopeConfig base_config(CaptureMode mode, TriggerType trigger) {
    ScopeConfig config;
    config.mode = mode;
    config.trigger_type = trigger;
    config.sample_rate_hz = 10000u;
    config.decimation = 1u;
    config.channel_count = 3u;
    config.channel_ids[0] = 6u;
    config.channel_ids[1] = 11u;
    config.channel_ids[2] = 7u;
    return config;
}

generated::ScopeSampleContext sample(uint32_t sequence, float phase,
                                     uint8_t state = 1u) {
    generated::ScopeSampleContext value;
    value.control_sequence = sequence;
    value.timestamp_cycles = sequence * 100u;
    value.state_epoch = 2u;
    value.safety_state = state;
    value.phase = phase;
    value.iq_measured = phase * 2.0f;
    return value;
}

}  // namespace

TEST_SUITE("ScopeCapture") {
    TEST_CASE("plan validation enforces whitelist and fixed memory limits") {
        ScopeConfig config = base_config(CaptureMode::TRIGGERED,
                                          TriggerType::MANUAL);
        config.pre_samples = 64u;
        config.post_samples = 128u;
        CapturePlan plan;
        ValidationError error = ValidationError::NONE;
        CHECK(build_capture_plan(config, 9u, &plan, &error));
        CHECK(error == ValidationError::NONE);
        config.channel_ids[0] = 0x1234u;
        CHECK_FALSE(build_capture_plan(config, 9u, &plan, &error));
        CHECK(error == ValidationError::UNKNOWN_CHANNEL);
    }

    TEST_CASE("manual trigger retains pre and post samples") {
        CaptureEngine engine;
        ScopeConfig config = base_config(CaptureMode::TRIGGERED,
                                          TriggerType::MANUAL);
        config.pre_samples = 2u;
        config.post_samples = 3u;
        uint32_t capture_id = 0u;
        CHECK(engine.configure_from_task(config, &capture_id) ==
              ValidationError::NONE);
        CHECK(engine.arm_from_task());
        engine.sample_from_isr(sample(1u, 1.0f));
        engine.sample_from_isr(sample(2u, 2.0f));
        ScopeBatch batch;
        CHECK_FALSE(engine.read_batch(&batch));
        CHECK(engine.trigger_manual_from_task());
        engine.sample_from_isr(sample(3u, 3.0f));
        engine.sample_from_isr(sample(4u, 4.0f));
        engine.sample_from_isr(sample(5u, 5.0f));
        ScopeStatus status;
        REQUIRE(engine.read_status(&status));
        CHECK(status.state == CaptureState::COMPLETE);
        CHECK(status.sample_count == 5u);
        size_t count = 0u;
        while (engine.read_batch(&batch)) count += batch.sample_count;
        CHECK(count == 5u);
        CHECK(batch.samples[0].control_sequence == 1u);
    }

    TEST_CASE("threshold and state triggers are edge/transition based") {
        CaptureEngine threshold_engine;
        ScopeConfig threshold = base_config(CaptureMode::TRIGGERED,
                                             TriggerType::THRESHOLD);
        threshold.trigger_channel = 6u;
        threshold.threshold = 1.0f;
        threshold.pre_samples = 1u;
        threshold.post_samples = 1u;
        uint32_t ignored = 0u;
        CHECK(threshold_engine.configure_from_task(threshold, &ignored) ==
              ValidationError::NONE);
        CHECK(threshold_engine.arm_from_task());
        threshold_engine.sample_from_isr(sample(1u, 0.0f));
        threshold_engine.sample_from_isr(sample(2u, 2.0f));
        ScopeStatus threshold_status;
        threshold_engine.read_status(&threshold_status);
        CHECK(threshold_status.final_reason == CaptureFinalReason::THRESHOLD);

        CaptureEngine state_engine;
        ScopeConfig state = base_config(CaptureMode::TRIGGERED,
                                         TriggerType::STATE);
        state.trigger_state = 8u;
        state.pre_samples = 0u;
        state.post_samples = 1u;
        CHECK(state_engine.configure_from_task(state, &ignored) ==
              ValidationError::NONE);
        CHECK(state_engine.arm_from_task());
        state_engine.notify_state_from_task(1u);
        state_engine.notify_state_from_task(8u);
        state_engine.sample_from_isr(sample(1u, 0.0f, 8u));
        ScopeStatus state_status;
        state_engine.read_status(&state_status);
        CHECK(state_status.final_reason == CaptureFinalReason::STATE);
    }

    TEST_CASE("rolling pre-trigger history is not reported as dropped data") {
        CaptureEngine engine;
        ScopeConfig config = base_config(CaptureMode::TRIGGERED,
                                          TriggerType::MANUAL);
        config.pre_samples = 2u;
        config.post_samples = 1u;
        uint32_t ignored = 0u;
        CHECK(engine.configure_from_task(config, &ignored) ==
              ValidationError::NONE);
        CHECK(engine.arm_from_task());
        for (uint32_t sequence = 1u; sequence <=
             static_cast<uint32_t>(kCaptureBufferCapacity + 20u); ++sequence) {
            engine.sample_from_isr(sample(sequence, 0.0f));
        }
        CHECK(engine.trigger_manual_from_task());
        engine.sample_from_isr(sample(
            static_cast<uint32_t>(kCaptureBufferCapacity + 21u), 0.0f));

        ScopeStatus status;
        REQUIRE(engine.read_status(&status));
        CHECK(status.state == CaptureState::COMPLETE);
        CHECK(status.sample_count == 3u);
        CHECK(status.dropped_samples == 0u);
        CHECK(status.first_timestamp_cycles ==
              static_cast<uint32_t>(kCaptureBufferCapacity + 19u) * 100u);

        ScopeBatch batch;
        size_t count = 0u;
        while (engine.read_batch(&batch)) count += batch.sample_count;
        CHECK(count == 3u);
    }

    TEST_CASE("continuous overflow is bounded and independently counted") {
        CaptureEngine engine;
        ScopeConfig config = base_config(CaptureMode::CONTINUOUS,
                                          TriggerType::NONE);
        uint32_t ignored = 0u;
        CHECK(engine.configure_from_task(config, &ignored) ==
              ValidationError::NONE);
        CHECK(engine.arm_from_task());
        for (uint32_t sequence = 1u; sequence <
             static_cast<uint32_t>(kCaptureBufferCapacity + 8u); ++sequence) {
            engine.sample_from_isr(sample(sequence, 0.0f));
        }
        ScopeStatus status;
        engine.read_status(&status);
        CHECK(status.dropped_samples > 0u);
        CHECK(status.sample_count <= kCaptureBufferCapacity);
    }

    TEST_CASE("continuous capture applies decimation and never replays a batch") {
        CaptureEngine engine;
        ScopeConfig config = base_config(CaptureMode::CONTINUOUS,
                                          TriggerType::NONE);
        config.sample_rate_hz = 1000u;
        config.decimation = 10u;
        uint32_t ignored = 0u;
        CHECK(engine.configure_from_task(config, &ignored) ==
              ValidationError::NONE);
        CHECK(engine.arm_from_task());
        for (uint32_t sequence = 1u; sequence <= 100u; ++sequence) {
            engine.sample_from_isr(sample(sequence, 0.0f));
        }

        ScopeStatus status;
        REQUIRE(engine.read_status(&status));
        CHECK(status.decimation == 10u);
        CHECK(status.sample_count == 10u);

        ScopeBatch batch;
        size_t count = 0u;
        uint32_t expected_sequence = 10u;
        while (engine.read_batch(&batch)) {
            for (size_t index = 0u; index < batch.sample_count; ++index) {
                CHECK(batch.samples[index].control_sequence == expected_sequence);
                expected_sequence += 10u;
            }
            count += batch.sample_count;
        }
        CHECK(count == 10u);
        CHECK_FALSE(engine.read_batch(&batch));
    }
}
