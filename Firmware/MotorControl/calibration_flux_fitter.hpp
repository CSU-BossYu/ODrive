#ifndef __CALIBRATION_FLUX_FITTER_HPP
#define __CALIBRATION_FLUX_FITTER_HPP

#include <algorithm>
#include <cmath>
#include <stdint.h>

#include "calibration_record.hpp"

class CalibrationFluxFitter {
public:
    enum FailureReason : uint32_t {
        FAILURE_NONE = 0,
        FAILURE_INSUFFICIENT_BLOCKS = 1,
        FAILURE_NONPHYSICAL_MEAN = 2,
        FAILURE_EXCESSIVE_DISPERSION = 3,
    };

    struct Result {
        float flux_linkage = 0.0f;       // [V/(electrical rad/s)]
        float torque_constant = 0.0f;    // [motor Nm/Aq]
        float sample_stddev = 0.0f;
        float mean_std_error = 0.0f;
        uint32_t used_samples = 0;
    };

    bool init(float phase_resistance, float phase_inductance,
              int32_t pole_pairs) {
        resistance_ = phase_resistance;
        inductance_ = phase_inductance;
        pole_pairs_ = pole_pairs;
        positive_ = {};
        negative_ = {};
        positive_block_ = {};
        negative_block_ = {};
        used_samples_ = 0;
        failure_reason_ = FAILURE_NONE;
        return std::isfinite(resistance_) && resistance_ > 0.0f &&
            std::isfinite(inductance_) && inductance_ > 0.0f &&
            pole_pairs_ > 0;
    }

    bool add_sample(const CalibrationSampleV1& sample) {
        constexpr float kMinimumElectricalSpeed = 20.0f;
        const uint32_t required = CAL_SAMPLE_CURRENT_VALID |
                                  CAL_SAMPLE_APPLIED_VOLTAGE_VALID;
        if ((sample.flags & required) != required ||
            (sample.flags & CAL_SAMPLE_PWM_SATURATED) ||
            std::abs(sample.electrical_velocity) < kMinimumElectricalSpeed) {
            return false;
        }

        // vq = R*iq + omega_e*L*id + omega_e*psi. Using both directions
        // cancels much of the inverter/dead-time voltage bias.
        const float flux =
            (sample.vq_applied - resistance_ * sample.iq) /
                sample.electrical_velocity -
            inductance_ * sample.id;
        if (!std::isfinite(flux) || flux <= 1.0e-6f || flux > 1.0f) {
            return false;
        }
        ++used_samples_;
        add_block(sample.electrical_velocity > 0.0f ? positive_ : negative_,
                  sample.electrical_velocity > 0.0f ? positive_block_ : negative_block_,
                  flux);
        return true;
    }

    bool finish(Result* result) {
        constexpr uint32_t kMinimumBlocksPerDirection = 4;
        failure_reason_ = FAILURE_NONE;
        if (!result) return false;
        *result = {};
        result->used_samples = used_samples_;
        if (positive_.count < kMinimumBlocksPerDirection ||
            negative_.count < kMinimumBlocksPerDirection) {
            failure_reason_ = FAILURE_INSUFFICIENT_BLOCKS;
            return false;
        }
        const float mean = 0.5f * (positive_.mean + negative_.mean);
        const uint32_t count = positive_.count + negative_.count;
        // A direction-independent inverter/dead-time voltage offset produces
        // opposite flux-estimate offsets after division by signed speed. The
        // bidirectional mean cancels that systematic term. Do not count the
        // separation of the two direction means again as random dispersion.
        const float variance = (positive_.m2 + negative_.m2) /
            static_cast<float>(std::max<uint32_t>(count - 2, 1));
        const float stddev = std::sqrt(std::max(variance, 0.0f));
        const float positive_variance = positive_.m2 /
            static_cast<float>(std::max<uint32_t>(positive_.count - 1, 1));
        const float negative_variance = negative_.m2 /
            static_cast<float>(std::max<uint32_t>(negative_.count - 1, 1));
        const float mean_std_error = 0.5f * std::sqrt(std::max(
            positive_variance / static_cast<float>(positive_.count) +
            negative_variance / static_cast<float>(negative_.count),
            0.0f));
        result->flux_linkage = mean;
        result->torque_constant = 1.5f * pole_pairs_ * mean;
        result->sample_stddev = stddev;
        result->mean_std_error = mean_std_error;
        if (!std::isfinite(mean) || mean <= 1.0e-6f) {
            failure_reason_ = FAILURE_NONPHYSICAL_MEAN;
            return false;
        }
        // Block ripple may be appreciable while its bidirectional mean is
        // precise. Reject pathological ripple and an uncertain mean
        // independently.
        if (!std::isfinite(stddev) || stddev > mean ||
            !std::isfinite(mean_std_error) ||
            mean_std_error > mean * 0.05f) {
            failure_reason_ = FAILURE_EXCESSIVE_DISPERSION;
            return false;
        }
        return true;
    }

    FailureReason failure_reason() const { return failure_reason_; }

private:
    struct Accumulator {
        uint32_t count = 0;
        float mean = 0.0f;
        float m2 = 0.0f;
    };

    struct BlockAccumulator {
        uint32_t count = 0;
        float sum = 0.0f;
    };

    static constexpr uint32_t kBlockSamples = 32;

    static void add(Accumulator& accumulator, float value) {
        ++accumulator.count;
        const float delta = value - accumulator.mean;
        accumulator.mean += delta / static_cast<float>(accumulator.count);
        accumulator.m2 += delta * (value - accumulator.mean);
    }

    static void add_block(Accumulator& accumulator,
                          BlockAccumulator& block, float value) {
        block.sum += value;
        if (++block.count == kBlockSamples) {
            add(accumulator, block.sum / static_cast<float>(kBlockSamples));
            block = {};
        }
    }

    float resistance_ = 0.0f;
    float inductance_ = 0.0f;
    int32_t pole_pairs_ = 0;
    Accumulator positive_ = {};
    Accumulator negative_ = {};
    BlockAccumulator positive_block_ = {};
    BlockAccumulator negative_block_ = {};
    uint32_t used_samples_ = 0;
    FailureReason failure_reason_ = FAILURE_NONE;
};

#endif // __CALIBRATION_FLUX_FITTER_HPP
