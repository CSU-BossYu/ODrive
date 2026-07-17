#ifndef __CALIBRATION_DELAY_FITTER_HPP
#define __CALIBRATION_DELAY_FITTER_HPP

#include <algorithm>
#include <cmath>
#include <stdint.h>

#include "calibration_record.hpp"

class CalibrationDelayFitter {
public:
    enum FailureReason : uint32_t {
        FAILURE_NONE = 0,
        FAILURE_INSUFFICIENT_SAMPLES = 1,
        FAILURE_UNOBSERVABLE_SPEED = 2,
        FAILURE_NONPHYSICAL_RESULT = 3,
    };

    struct Result {
        float electrical_delay = 0.0f;
        float residual_phase_offset = 0.0f;
        float residual_rms = 0.0f;
        uint32_t used_samples = 0;
    };

    bool init(float resistance, float inductance) {
        resistance_ = resistance;
        inductance_ = inductance;
        count_ = positive_count_ = negative_count_ = regression_count_ = 0;
        attempted_count_ = rejected_invalid_ = rejected_saturated_ = 0;
        rejected_speed_ = rejected_emf_ = rejected_phase_ = 0;
        max_abs_electrical_speed_ = 0.0f;
        failure_reason_ = FAILURE_NONE;
        sum_x_ = sum_y_ = sum_xx_ = sum_xy_ = sum_yy_ = 0.0f;
        return std::isfinite(resistance_) && resistance_ > 0.0f &&
               std::isfinite(inductance_) && inductance_ > 0.0f;
    }

    void begin_segment() {
        block_count_ = 0;
        block_sum_x_ = 0.0f;
        block_sum_y_ = 0.0f;
    }

    void finish_segment() {
        // Deliberately discard a partial block so samples from different
        // commanded speeds/directions are never averaged together.
        block_count_ = 0;
        block_sum_x_ = 0.0f;
        block_sum_y_ = 0.0f;
    }

    bool add_sample(const CalibrationSampleV1& sample) {
        ++attempted_count_;
        constexpr float kMinimumElectricalSpeed = 30.0f;
        const uint32_t required = CAL_SAMPLE_CURRENT_VALID |
                                  CAL_SAMPLE_APPLIED_VOLTAGE_VALID;
        if ((sample.flags & required) != required) {
            ++rejected_invalid_;
            return false;
        }
        if (sample.flags & CAL_SAMPLE_PWM_SATURATED) {
            ++rejected_saturated_;
            return false;
        }
        max_abs_electrical_speed_ = std::max(
            max_abs_electrical_speed_, std::abs(sample.electrical_velocity));
        if (std::abs(sample.electrical_velocity) < kMinimumElectricalSpeed) {
            ++rejected_speed_;
            return false;
        }
        const float omega = sample.electrical_velocity;
        const float back_emf_d = sample.vd_applied - resistance_ * sample.id +
                                 omega * inductance_ * sample.iq;
        const float back_emf_q = sample.vq_applied - resistance_ * sample.iq -
                                 omega * inductance_ * sample.id;
        const float magnitude = std::hypot(back_emf_d, back_emf_q);
        if (!std::isfinite(magnitude) || magnitude < 0.05f) {
            ++rejected_emf_;
            return false;
        }
        // Reverse rotation reverses the complete PM back-EMF vector. Normalize
        // both axes by the speed sign before measuring its small phase error;
        // otherwise every reverse sample appears close to +/-pi and is
        // incorrectly rejected by the 0.5 rad quality gate.
        const float direction = omega > 0.0f ? 1.0f : -1.0f;
        const float phase_error = std::atan2(
            -direction * back_emf_d, direction * back_emf_q);
        if (!std::isfinite(phase_error) || std::abs(phase_error) > 0.5f) {
            ++rejected_phase_;
            return false;
        }
        ++count_;
        if (omega > 0.0f) ++positive_count_;
        else ++negative_count_;
        block_sum_x_ += omega;
        block_sum_y_ += phase_error;
        if (++block_count_ == kBlockSamples) {
            const float block_x = block_sum_x_ / static_cast<float>(kBlockSamples);
            const float block_y = block_sum_y_ / static_cast<float>(kBlockSamples);
            ++regression_count_;
            sum_x_ += block_x;
            sum_y_ += block_y;
            sum_xx_ += block_x * block_x;
            sum_xy_ += block_x * block_y;
            sum_yy_ += block_y * block_y;
            block_count_ = 0;
            block_sum_x_ = 0.0f;
            block_sum_y_ = 0.0f;
        }
        return true;
    }

    bool finish(Result* result) {
        constexpr uint32_t kMinimumSamplesPerDirection = 500;
        failure_reason_ = FAILURE_NONE;
        if (!result) return false;
        *result = {};
        result->used_samples = count_;
        if (positive_count_ < kMinimumSamplesPerDirection ||
            negative_count_ < kMinimumSamplesPerDirection) {
            failure_reason_ = FAILURE_INSUFFICIENT_SAMPLES;
            return false;
        }
        constexpr uint32_t kMinimumRegressionBlocks = 16;
        if (regression_count_ < kMinimumRegressionBlocks) {
            failure_reason_ = FAILURE_INSUFFICIENT_SAMPLES;
            return false;
        }
        const float n = static_cast<float>(regression_count_);
        const float denominator = n * sum_xx_ - sum_x_ * sum_x_;
        if (denominator <= 1.0e-6f) {
            failure_reason_ = FAILURE_UNOBSERVABLE_SPEED;
            return false;
        }
        const float delay = (n * sum_xy_ - sum_x_ * sum_y_) / denominator;
        const float intercept = (sum_y_ - delay * sum_x_) / n;
        const float sse = sum_yy_ + delay * delay * sum_xx_ +
            n * intercept * intercept - 2.0f * delay * sum_xy_ -
            2.0f * intercept * sum_y_ + 2.0f * delay * intercept * sum_x_;
        const float rms = std::sqrt(std::max(sse, 0.0f) / n);
        result->electrical_delay = delay;
        result->residual_phase_offset = intercept;
        result->residual_rms = rms;
        if (!std::isfinite(delay) || delay < 0.0f || delay > 500.0e-6f ||
            !std::isfinite(intercept) || std::abs(intercept) > 0.20f ||
            !std::isfinite(rms) || rms > 0.10f) {
            failure_reason_ = FAILURE_NONPHYSICAL_RESULT;
            return false;
        }
        return true;
    }

    FailureReason failure_reason() const { return failure_reason_; }
    uint32_t attempted_count() const { return attempted_count_; }
    uint32_t rejected_invalid() const { return rejected_invalid_; }
    uint32_t rejected_saturated() const { return rejected_saturated_; }
    uint32_t rejected_speed() const { return rejected_speed_; }
    uint32_t rejected_emf() const { return rejected_emf_; }
    uint32_t rejected_phase() const { return rejected_phase_; }
    float max_abs_electrical_speed() const { return max_abs_electrical_speed_; }

private:
    static constexpr uint32_t kBlockSamples = 32;
    float resistance_ = 0.0f;
    float inductance_ = 0.0f;
    uint32_t count_ = 0;
    uint32_t positive_count_ = 0;
    uint32_t negative_count_ = 0;
    uint32_t regression_count_ = 0;
    uint32_t block_count_ = 0;
    float block_sum_x_ = 0.0f;
    float block_sum_y_ = 0.0f;
    uint32_t attempted_count_ = 0;
    uint32_t rejected_invalid_ = 0;
    uint32_t rejected_saturated_ = 0;
    uint32_t rejected_speed_ = 0;
    uint32_t rejected_emf_ = 0;
    uint32_t rejected_phase_ = 0;
    float max_abs_electrical_speed_ = 0.0f;
    FailureReason failure_reason_ = FAILURE_NONE;
    float sum_x_ = 0.0f;
    float sum_y_ = 0.0f;
    float sum_xx_ = 0.0f;
    float sum_xy_ = 0.0f;
    float sum_yy_ = 0.0f;
};

#endif // __CALIBRATION_DELAY_FITTER_HPP
