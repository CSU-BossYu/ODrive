#ifndef __CALIBRATION_MECHANICAL_FITTER_HPP
#define __CALIBRATION_MECHANICAL_FITTER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdint.h>

#include "calibration_record.hpp"

class CalibrationMechanicalFitter {
public:
    static constexpr size_t kParameters = 5;

    enum FailureReason : uint32_t {
        FAILURE_NONE = 0,
        FAILURE_INSUFFICIENT_EXCITATION = 1,
        FAILURE_SINGULAR_REGRESSION = 2,
        FAILURE_NONPHYSICAL_PARAMETERS = 3,
    };

    struct Result {
        float output_inertia = 0.0f;
        float coulomb_pos = 0.0f;
        float coulomb_neg = 0.0f;
        float viscous_pos = 0.0f;
        float viscous_neg = 0.0f;
        float residual_rms_torque = 0.0f;
        uint32_t used_samples = 0;
    };

    bool init(float torque_constant, float motor_turns_per_output_turn,
              float effective_ratio_scale, float cpu_hz,
              float iq_to_output_direction) {
        torque_scale_ = torque_constant *
                        std::abs(motor_turns_per_output_turn);
        ratio_scale_ = effective_ratio_scale;
        cpu_hz_ = cpu_hz;
        iq_to_output_direction_ = iq_to_output_direction;
        xtx_ = {};
        xty_ = {};
        yty_ = 0.0f;
        sample_count_ = 0;
        positive_count_ = 0;
        negative_count_ = 0;
        max_abs_acceleration_ = 0.0f;
        max_abs_velocity_ = 0.0f;
        attempted_samples_ = 0;
        rejected_invalid_ = 0;
        rejected_saturated_ = 0;
        rejected_low_velocity_ = 0;
        rejected_timing_ = 0;
        segment_active_ = false;
        failure_reason_ = FAILURE_NONE;
        return std::isfinite(torque_scale_) && torque_scale_ > 0.0f &&
            std::isfinite(ratio_scale_) && std::abs(ratio_scale_) > 1.0e-6f &&
            cpu_hz_ > 0.0f &&
            (iq_to_output_direction_ == 1.0f || iq_to_output_direction_ == -1.0f);
    }

    void begin_segment() {
        previous_valid_ = false;
        filtered_acceleration_ = 0.0f;
        segment_active_ = true;
    }

    bool add_sample(const CalibrationSampleV1& sample) {
        ++attempted_samples_;
        const uint32_t required = CAL_SAMPLE_CURRENT_VALID |
                                  CAL_SAMPLE_MAIN_VALID;
        if (!segment_active_ || (sample.flags & required) != required) {
            ++rejected_invalid_;
            return false;
        }
        if (sample.flags & CAL_SAMPLE_PWM_SATURATED) {
            ++rejected_saturated_;
            return false;
        }
        const float velocity = sample.velocity_turns_per_s / ratio_scale_;
        if (std::isfinite(velocity)) {
            max_abs_velocity_ = std::max(max_abs_velocity_, std::abs(velocity));
        }
        if (!previous_valid_) {
            previous_ticks_ = sample.sample_ticks;
            previous_velocity_ = velocity;
            previous_valid_ = true;
            return false;
        }
        const uint32_t elapsed_ticks = sample.sample_ticks - previous_ticks_;
        previous_ticks_ = sample.sample_ticks;
        const float dt = elapsed_ticks / cpu_hz_;
        if (!(dt > 0.0f && dt < 0.02f)) {
            ++rejected_timing_;
            previous_velocity_ = velocity;
            return false;
        }
        const float raw_acceleration = (velocity - previous_velocity_) / dt;
        previous_velocity_ = velocity;
        constexpr float kAccelerationFilterTau = 0.02f;
        const float filter_gain = dt / (kAccelerationFilterTau + dt);
        filtered_acceleration_ += filter_gain *
            (raw_acceleration - filtered_acceleration_);

        constexpr float kMinimumVelocity = 0.005f;
        if (std::abs(velocity) < kMinimumVelocity ||
            !std::isfinite(sample.iq)) {
            ++rejected_low_velocity_;
            return false;
        }

        // Convert electrical q-current into the controller/output-shaft sign
        // convention explicitly. Inferring this sign from fitted friction can
        // fail under gravity loads or strongly asymmetric gearbox friction.
        const float output_iq = iq_to_output_direction_ * sample.iq;

        // output_iq = Ja*alpha + Tc+/- + B+/-*velocity. Coefficients are first
        // identified in A-based units, then converted to output Nm.
        std::array<float, kParameters> x = {};
        x[0] = filtered_acceleration_;
        if (velocity > 0.0f) {
            x[1] = 1.0f;
            x[3] = velocity;
            ++positive_count_;
        } else {
            x[2] = -1.0f;
            x[4] = velocity;
            ++negative_count_;
        }
        for (size_t row = 0; row < kParameters; ++row) {
            xty_[row] += x[row] * output_iq;
            for (size_t col = 0; col < kParameters; ++col) {
                xtx_[row][col] += x[row] * x[col];
            }
        }
        yty_ += output_iq * output_iq;
        max_abs_acceleration_ = std::max(
            max_abs_acceleration_, std::abs(filtered_acceleration_));
        ++sample_count_;
        return true;
    }

    void finish_segment() { segment_active_ = false; }

    bool finish(Result* result) {
        constexpr uint32_t kMinimumSamplesPerDirection = 500;
        failure_reason_ = FAILURE_NONE;
        if (!result) return false;
        *result = {};
        result->used_samples = sample_count_;
        if (positive_count_ < kMinimumSamplesPerDirection ||
            negative_count_ < kMinimumSamplesPerDirection ||
            max_abs_acceleration_ < 0.02f) {
            failure_reason_ = FAILURE_INSUFFICIENT_EXCITATION;
            return false;
        }
        std::array<std::array<float, kParameters + 1>, kParameters> matrix = {};
        for (size_t row = 0; row < kParameters; ++row) {
            for (size_t col = 0; col < kParameters; ++col) {
                matrix[row][col] = xtx_[row][col];
            }
            matrix[row][kParameters] = xty_[row];
        }
        if (!solve(matrix, kParameters)) {
            failure_reason_ = FAILURE_SINGULAR_REGRESSION;
            return false;
        }

        std::array<float, kParameters> beta = {};
        for (size_t i = 0; i < kParameters; ++i) {
            beta[i] = matrix[i][kParameters];
        }
        // Measurement noise, gravity loading and acceleration-estimator phase
        // lag can push an unconstrained least-squares coefficient below zero
        // even though inertia and friction are physically nonnegative. Refit
        // over every active parameter subset and retain the minimum-residual
        // nonnegative solution instead of rejecting the complete calibration.
        bool has_negative_coefficient = false;
        for (float value : beta) {
            has_negative_coefficient |= value < 0.0f;
        }
        if (has_negative_coefficient && !solve_nonnegative(&beta)) {
            failure_reason_ = FAILURE_NONPHYSICAL_PARAMETERS;
            return false;
        }
        float explained = 0.0f;
        for (size_t i = 0; i < kParameters; ++i) {
            explained += beta[i] * xty_[i];
        }
        const float residual_current_rms = std::sqrt(
            std::max(yty_ - explained, 0.0f) /
            static_cast<float>(sample_count_));
        result->output_inertia = beta[0] * torque_scale_;
        result->coulomb_pos = beta[1] * torque_scale_;
        result->coulomb_neg = beta[2] * torque_scale_;
        result->viscous_pos = beta[3] * torque_scale_;
        result->viscous_neg = beta[4] * torque_scale_;
        result->residual_rms_torque = residual_current_rms * torque_scale_;
        if (!std::isfinite(result->output_inertia) ||
            !std::isfinite(result->residual_rms_torque) ||
            beta[0] < 0.0f || beta[1] < 0.0f || beta[2] < 0.0f ||
            beta[3] < 0.0f || beta[4] < 0.0f) {
            failure_reason_ = FAILURE_NONPHYSICAL_PARAMETERS;
            return false;
        }
        return true;
    }

    FailureReason failure_reason() const { return failure_reason_; }
    uint32_t sample_count() const { return sample_count_; }
    uint32_t positive_count() const { return positive_count_; }
    uint32_t negative_count() const { return negative_count_; }
    float max_abs_acceleration() const { return max_abs_acceleration_; }
    float max_abs_velocity() const { return max_abs_velocity_; }
    uint32_t attempted_samples() const { return attempted_samples_; }
    uint32_t rejected_invalid() const { return rejected_invalid_; }
    uint32_t rejected_saturated() const { return rejected_saturated_; }
    uint32_t rejected_low_velocity() const { return rejected_low_velocity_; }
    uint32_t rejected_timing() const { return rejected_timing_; }

private:
    bool solve_nonnegative(std::array<float, kParameters>* result) const {
        if (!result) return false;
        bool found = false;
        float best_error = INFINITY;
        std::array<float, kParameters> best = {};
        constexpr uint32_t kAllParameters = (1u << kParameters) - 1u;

        for (uint32_t mask = 1u; mask <= kAllParameters; ++mask) {
            std::array<size_t, kParameters> active = {};
            size_t active_count = 0;
            for (size_t index = 0; index < kParameters; ++index) {
                if (mask & (1u << index)) active[active_count++] = index;
            }

            std::array<std::array<float, kParameters + 1>, kParameters> matrix = {};
            for (size_t row = 0; row < active_count; ++row) {
                for (size_t col = 0; col < active_count; ++col) {
                    matrix[row][col] = xtx_[active[row]][active[col]];
                }
                matrix[row][active_count] = xty_[active[row]];
            }
            if (!solve(matrix, active_count)) continue;

            std::array<float, kParameters> candidate = {};
            bool feasible = true;
            for (size_t row = 0; row < active_count; ++row) {
                const float value = matrix[row][active_count];
                if (!std::isfinite(value) || value < -1.0e-6f) {
                    feasible = false;
                    break;
                }
                candidate[active[row]] = std::max(value, 0.0f);
            }
            if (!feasible) continue;

            float error = yty_;
            for (size_t row = 0; row < kParameters; ++row) {
                error -= 2.0f * candidate[row] * xty_[row];
                for (size_t col = 0; col < kParameters; ++col) {
                    error += candidate[row] * xtx_[row][col] * candidate[col];
                }
            }
            if (std::isfinite(error) && error < best_error) {
                best_error = error;
                best = candidate;
                found = true;
            }
        }
        if (found) *result = best;
        return found;
    }

    static bool solve(
            std::array<std::array<float, kParameters + 1>, kParameters>& a,
            size_t size) {
        for (size_t pivot = 0; pivot < size; ++pivot) {
            size_t best = pivot;
            for (size_t row = pivot + 1; row < size; ++row) {
                if (std::abs(a[row][pivot]) > std::abs(a[best][pivot])) best = row;
            }
            if (std::abs(a[best][pivot]) < 1.0e-7f) return false;
            if (best != pivot) std::swap(a[best], a[pivot]);
            const float divisor = a[pivot][pivot];
            for (size_t col = pivot; col <= size; ++col) {
                a[pivot][col] /= divisor;
            }
            for (size_t row = 0; row < size; ++row) {
                if (row == pivot) continue;
                const float factor = a[row][pivot];
                for (size_t col = pivot; col <= size; ++col) {
                    a[row][col] -= factor * a[pivot][col];
                }
            }
        }
        return true;
    }

    float torque_scale_ = 0.0f;
    float ratio_scale_ = 1.0f;
    float cpu_hz_ = 0.0f;
    float iq_to_output_direction_ = 1.0f;
    std::array<std::array<float, kParameters>, kParameters> xtx_ = {};
    std::array<float, kParameters> xty_ = {};
    float yty_ = 0.0f;
    uint32_t sample_count_ = 0;
    uint32_t positive_count_ = 0;
    uint32_t negative_count_ = 0;
    float max_abs_acceleration_ = 0.0f;
    float max_abs_velocity_ = 0.0f;
    uint32_t attempted_samples_ = 0;
    uint32_t rejected_invalid_ = 0;
    uint32_t rejected_saturated_ = 0;
    uint32_t rejected_low_velocity_ = 0;
    uint32_t rejected_timing_ = 0;
    bool segment_active_ = false;
    bool previous_valid_ = false;
    FailureReason failure_reason_ = FAILURE_NONE;
    uint32_t previous_ticks_ = 0;
    float previous_velocity_ = 0.0f;
    float filtered_acceleration_ = 0.0f;
};

#endif // __CALIBRATION_MECHANICAL_FITTER_HPP
