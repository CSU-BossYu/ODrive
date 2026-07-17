#ifndef __CALIBRATION_GEOMETRY_FITTER_HPP
#define __CALIBRATION_GEOMETRY_FITTER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdint.h>

#include "calibration_record.hpp"

class CalibrationGeometryFitter {
public:
    static constexpr size_t kBins = 64;

    struct Config {
        int32_t pole_pairs = 0;
        float main_ratio = 0.0f;
        float aux_ratio = 0.0f;
        float main_offset = 0.0f;
        float aux_offset = 0.0f;
        bool main_reversed = false;
        bool aux_reversed = false;
        bool output_reversed = false;
        float cpu_hz = 0.0f;
    };

    struct Result {
        float effective_ratio_scale = 1.0f;
        std::array<float, kBins> common_correction = {};
        std::array<float, kBins> direction_correction = {};
        float raw_rms_turns = 0.0f;
        float corrected_rms_turns = 0.0f;
        float direction_peak_to_peak_turns = 0.0f;
        uint32_t used_samples = 0;
    };

    bool init(const Config& config) {
        config_ = config;
        reset();
        return config_.pole_pairs > 0 && std::isfinite(config_.main_ratio) &&
            std::isfinite(config_.aux_ratio) && config_.cpu_hz > 0.0f &&
            std::abs(config_.main_ratio) > 1.0e-6f &&
            std::abs(config_.aux_ratio - config_.main_ratio) > 1.0e-6f;
    }

    void reset() {
        scale_sxx_ = 0.0;
        scale_sxy_ = 0.0;
        total_samples_ = 0;
        effective_ratio_scale_ = 1.0f;
        global_sum_.fill({});
        global_sumsq_.fill({});
        global_count_.fill({});
        segment_active_ = false;
    }

    bool begin_scale_segment(int direction) {
        return begin_segment(Pass::Scale, direction);
    }

    bool begin_lut_segment(int direction) {
        return begin_segment(Pass::Lut, direction);
    }

    bool finish_segment() {
        if (!segment_active_ || segment_samples_ < kBins / 2) {
            segment_active_ = false;
            return false;
        }
        if (pass_ == Pass::Scale) {
            scale_sxx_ += segment_sxx_;
            scale_sxy_ += segment_sxy_;
        } else {
            const float residual_mean = segment_residual_sum_ /
                                        static_cast<float>(segment_samples_);
            for (size_t bin = 0; bin < kBins; ++bin) {
                const uint32_t count = segment_count_[bin];
                if (!count) continue;
                const float sum = segment_sum_[bin];
                global_sum_[direction_index_][bin] +=
                    sum - residual_mean * static_cast<float>(count);
                global_sumsq_[direction_index_][bin] +=
                    segment_sumsq_[bin] - 2.0f * residual_mean * sum +
                    residual_mean * residual_mean * static_cast<float>(count);
                global_count_[direction_index_][bin] += count;
            }
        }
        total_samples_ += segment_samples_;
        segment_active_ = false;
        return true;
    }

    bool finish_scale_pass() {
        if (segment_active_ || scale_sxx_ <= 1.0e-12f) return false;
        const float scale = scale_sxy_ / scale_sxx_;
        if (!std::isfinite(scale) || scale < 0.8f || scale > 1.2f) return false;
        effective_ratio_scale_ = scale;
        return true;
    }

    bool add_sample(const CalibrationSampleV1& sample) {
        if (!segment_active_ ||
            (sample.flags & (CAL_SAMPLE_MAIN_VALID | CAL_SAMPLE_AUX_VALID)) !=
                (CAL_SAMPLE_MAIN_VALID | CAL_SAMPLE_AUX_VALID) ||
            std::abs(sample.electrical_velocity) < 1.0f) {
            return false;
        }

        const float electrical = sample.electrical_phase;
        const float output = resolve_output(sample);
        if (!unwrap_valid_) {
            previous_electrical_ = electrical;
            previous_output_ = output;
            electrical_unwrapped_ = electrical;
            output_unwrapped_ = output;
            unwrap_valid_ = true;
        } else {
            electrical_unwrapped_ += wrap_pm_pi(electrical - previous_electrical_);
            output_unwrapped_ += wrap_pm_half(output - previous_output_);
            previous_electrical_ = electrical;
            previous_output_ = output;
        }

        const float output_sign = config_.output_reversed ? -1.0f : 1.0f;
        const float reference = output_sign * electrical_unwrapped_ /
            (2.0f * M_PI * config_.pole_pairs * config_.main_ratio);
        const float observed = output_unwrapped_;
        ++segment_samples_;

        if (pass_ == Pass::Scale) {
            const float dx = reference - segment_ref_mean_;
            segment_ref_mean_ += dx / segment_samples_;
            const float dy = observed - segment_obs_mean_;
            segment_obs_mean_ += dy / segment_samples_;
            segment_sxx_ += dx * (reference - segment_ref_mean_);
            segment_sxy_ += dx * (observed - segment_obs_mean_);
        } else {
            const float residual = observed -
                                   effective_ratio_scale_ * reference;
            const size_t bin = std::min(
                static_cast<size_t>(wrap01(output) * kBins), kBins - 1);
            segment_sum_[bin] += residual;
            segment_sumsq_[bin] += residual * residual;
            ++segment_count_[bin];
            segment_residual_sum_ += residual;
        }
        return true;
    }

    bool finish(Result* result) const {
        if (!result || segment_active_) return false;
        Result candidate = {};
        candidate.effective_ratio_scale = effective_ratio_scale_;
        float raw_sse = 0.0f;
        float corrected_sse = 0.0f;
        uint32_t count_total = 0;
        float max_direction = 0.0f;

        for (size_t bin = 0; bin < kBins; ++bin) {
            if (!global_count_[0][bin] || !global_count_[1][bin]) return false;
            const float forward = global_sum_[0][bin] / global_count_[0][bin];
            const float reverse = global_sum_[1][bin] / global_count_[1][bin];
            const float common = 0.5f * (forward + reverse);
            const float directional = 0.5f * (forward - reverse);
            candidate.common_correction[bin] = static_cast<float>(-common);
            candidate.direction_correction[bin] =
                static_cast<float>(-directional);
            max_direction = std::max(max_direction,
                                     static_cast<float>(std::abs(directional)));

            for (size_t direction = 0; direction < 2; ++direction) {
                const float correction = -common +
                    (direction == 0 ? -directional : directional);
                const float sum = global_sum_[direction][bin];
                const float sumsq = global_sumsq_[direction][bin];
                const uint32_t count = global_count_[direction][bin];
                raw_sse += sumsq;
                corrected_sse += sumsq + 2.0f * correction * sum +
                    correction * correction * static_cast<float>(count);
                count_total += count;
            }
        }
        if (!count_total) return false;
        candidate.raw_rms_turns = static_cast<float>(
            std::sqrt(std::max(raw_sse, 0.0f) / count_total));
        candidate.corrected_rms_turns = static_cast<float>(
            std::sqrt(std::max(corrected_sse, 0.0f) / count_total));
        candidate.direction_peak_to_peak_turns = 2.0f * max_direction;
        candidate.used_samples = total_samples_;
        if (!std::isfinite(candidate.corrected_rms_turns) ||
            candidate.corrected_rms_turns > candidate.raw_rms_turns * 1.05f) {
            return false;
        }
        *result = candidate;
        return true;
    }

private:
    enum class Pass { Scale, Lut };

    static float wrap01(float value) { return value - std::floor(value); }
    static float wrap_pm_half(float value) {
        return value - std::floor(value + 0.5f);
    }
    static float wrap_pm_pi(float value) {
        return value - 2.0f * M_PI *
            std::floor(value / (2.0f * M_PI) + 0.5f);
    }

    bool begin_segment(Pass pass, int direction) {
        if (segment_active_ || (direction != 1 && direction != -1)) return false;
        pass_ = pass;
        direction_index_ = direction > 0 ? 0 : 1;
        segment_samples_ = 0;
        unwrap_valid_ = false;
        segment_ref_mean_ = 0.0;
        segment_obs_mean_ = 0.0;
        segment_sxx_ = 0.0;
        segment_sxy_ = 0.0;
        segment_residual_sum_ = 0.0;
        segment_sum_.fill(0.0);
        segment_sumsq_.fill(0.0);
        segment_count_.fill(0);
        segment_active_ = true;
        return true;
    }

    float normalize_raw(uint16_t raw, bool reversed) const {
        const float phase = static_cast<float>(raw) / 32768.0f;
        return wrap01(reversed ? -phase : phase);
    }

    float resolve_output(const CalibrationSampleV1& sample) const {
        const float main = normalize_raw(sample.main_raw, config_.main_reversed);
        float aux = normalize_raw(sample.aux_raw, config_.aux_reversed);
        const float skew_seconds = sample.pair_sample_skew_cycles / config_.cpu_hz;
        const float aux_velocity = sample.electrical_velocity /
            (2.0f * M_PI * config_.pole_pairs * config_.main_ratio) *
            config_.aux_ratio;
        aux = wrap01(aux - aux_velocity * skew_seconds);
        const float main_corr = wrap01(main - config_.main_offset);
        const float aux_corr = wrap01(aux - config_.aux_offset);
        const float ratio_delta = config_.aux_ratio - config_.main_ratio;
        const float coarse = wrap01(wrap_pm_half(aux_corr - main_corr) /
                                    ratio_delta);
        const float predicted_main = wrap01(config_.main_ratio * coarse);
        const float refined = wrap01(
            coarse + wrap_pm_half(main_corr - predicted_main) /
                         config_.main_ratio);
        return wrap01(config_.output_reversed ? -refined : refined);
    }

    Config config_ = {};
    Pass pass_ = Pass::Scale;
    bool segment_active_ = false;
    bool unwrap_valid_ = false;
    size_t direction_index_ = 0;
    uint32_t segment_samples_ = 0;
    uint32_t total_samples_ = 0;
    float previous_electrical_ = 0.0f;
    float previous_output_ = 0.0f;
    float electrical_unwrapped_ = 0.0f;
    float output_unwrapped_ = 0.0f;
    float segment_ref_mean_ = 0.0f;
    float segment_obs_mean_ = 0.0f;
    float segment_sxx_ = 0.0f;
    float segment_sxy_ = 0.0f;
    float scale_sxx_ = 0.0f;
    float scale_sxy_ = 0.0f;
    float effective_ratio_scale_ = 1.0f;
    float segment_residual_sum_ = 0.0f;
    std::array<float, kBins> segment_sum_ = {};
    std::array<float, kBins> segment_sumsq_ = {};
    std::array<uint32_t, kBins> segment_count_ = {};
    std::array<std::array<float, kBins>, 2> global_sum_ = {};
    std::array<std::array<float, kBins>, 2> global_sumsq_ = {};
    std::array<std::array<uint32_t, kBins>, 2> global_count_ = {};
};

#endif // __CALIBRATION_GEOMETRY_FITTER_HPP
