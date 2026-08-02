#include "lz5710_position_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace lz5710 {

float wrap_0_2pi(float value) {
    value -= kTwoPi * floorf(value / kTwoPi);
    return value < 0.0f ? value + kTwoPi : value;
}

float wrap_pm_pi(float value) {
    value = wrap_0_2pi(value + 0.5f * kTwoPi) - 0.5f * kTwoPi;
    return value <= -0.5f * kTwoPi ? value + kTwoPi : value;
}

void Resolver::init(const ResolverConfig& config) {
    config_ = config;
    reset();
}

void Resolver::reset() {
    result_ = {};
    pending_branch_ = -1;
    pending_count_ = 0;
    mismatch_count_ = 0;
    accepted_position_valid_ = false;
    accepted_unique_position_rad_ = 0.0f;
    accepted_main_phase_rad_ = 0.0f;
}

float Resolver::corrected_phase(uint16_t count, bool reversed,
                                float offset_rad) const {
    float phase = kTwoPi * static_cast<float>(
        count % config_.counts_per_rev) /
        static_cast<float>(config_.counts_per_rev);
    if (reversed) {
        phase = -phase;
    }
    return wrap_0_2pi(phase - offset_rad);
}

ResolverResult Resolver::update(uint16_t main_count, bool main_valid,
                                uint16_t aux_count, bool aux_valid) {
    ResolverResult candidate = result_;
    candidate.fault = RESOLVER_FAULT_NONE;
    candidate.ambiguous = false;
    // This resolver intentionally implements the confirmed LZ5710 10:1,
    // 22:21 geometry. Reject a mismatched persisted/configured model instead
    // of silently searching the wrong fixed set of 21 branches.
    if (config_.counts_per_rev == 0u ||
        !std::isfinite(config_.main_ratio) ||
        !std::isfinite(config_.aux_ratio) ||
        std::abs(config_.main_ratio) < 1.0e-6f ||
        std::abs(config_.aux_ratio) < 1.0e-6f ||
        std::abs(config_.main_ratio - kMainRatio) > 1.0e-4f ||
        std::abs(config_.aux_ratio - kAuxRatio) > 1.0e-4f ||
        !(config_.residual_accept_rad >= 0.0f) ||
        !(config_.residual_reject_rad >=
          config_.residual_accept_rad)) {
        candidate.valid = false;
        candidate.locked = false;
        candidate.fault = RESOLVER_FAULT_NO_SOLUTION;
        result_ = candidate;
        return result_;
    }
    if (!main_valid) {
        candidate.valid = false;
        candidate.locked = false;
        candidate.fault = RESOLVER_FAULT_MAIN_INVALID;
        result_ = candidate;
        return result_;
    }
    if (!aux_valid) {
        candidate.locked = false;
        candidate.fault = RESOLVER_FAULT_AUX_INVALID;
        result_ = candidate;
        return result_;
    }

    candidate.raw_main_phase_rad = corrected_phase(
        main_count, config_.main_reversed, config_.main_offset_rad);
    candidate.raw_aux_phase_rad = corrected_phase(
        aux_count, config_.aux_reversed, config_.aux_offset_rad);
    const float main_fraction = candidate.raw_main_phase_rad / kTwoPi;
    float best_abs = INFINITY;
    float second_abs = INFINITY;
    float best_residual = 0.0f;
    float best_position = 0.0f;
    int32_t best_branch = -1;

    for (int32_t branch = 0; branch < kBranchCount; ++branch) {
        const float output_position =
            kTwoPi * (static_cast<float>(branch) + main_fraction) /
            config_.main_ratio;
        const float predicted_aux = wrap_0_2pi(
            config_.aux_ratio * output_position);
        const float residual = wrap_pm_pi(
            candidate.raw_aux_phase_rad - predicted_aux);
        const float abs_residual = std::abs(residual);
        if (abs_residual < best_abs) {
            second_abs = best_abs;
            best_abs = abs_residual;
            best_residual = residual;
            best_position = output_position;
            best_branch = branch;
        } else if (abs_residual < second_abs) {
            second_abs = abs_residual;
        }
    }

    if (best_branch < 0 || !std::isfinite(second_abs)) {
        candidate.valid = false;
        candidate.locked = false;
        candidate.fault = RESOLVER_FAULT_NO_SOLUTION;
        result_ = candidate;
        return result_;
    }

    candidate.vernier_branch_index = best_branch;
    candidate.unique_position_rad = config_.output_reversed
        ? kUniqueRangeRad - best_position
        : best_position;
    if (candidate.unique_position_rad >= kUniqueRangeRad) {
        candidate.unique_position_rad -= kUniqueRangeRad;
    }
    candidate.wrapped_output_phase_rad =
        wrap_0_2pi(candidate.unique_position_rad);
    candidate.residual_rad = best_residual;
    candidate.second_best_margin_rad = second_abs - best_abs;
    candidate.ambiguous =
        candidate.second_best_margin_rad < config_.ambiguity_margin_rad;

    bool accepted = false;
    // Use the tighter threshold only while acquiring a branch.  Once a
    // branch is locked, retain it up to the reject threshold and rely on the
    // independent ambiguity and continuity checks.  Treating the hysteresis
    // band as an invalid sample made normal stationary settling repeatedly
    // drop the resolver and starve the downstream PLL.
    const float active_residual_limit = accepted_position_valid_
        ? config_.residual_reject_rad
        : config_.residual_accept_rad;
    if (candidate.ambiguous) {
        candidate.locked = false;
        candidate.fault = RESOLVER_FAULT_AMBIGUOUS;
    } else if (best_abs > config_.residual_reject_rad) {
        candidate.locked = false;
        candidate.fault = RESOLVER_FAULT_RESIDUAL;
    } else if (best_abs <= active_residual_limit) {
        if (!accepted_position_valid_) {
            if (pending_branch_ == best_branch) {
                pending_count_ = static_cast<uint8_t>(pending_count_ + 1u);
            } else {
                pending_branch_ = best_branch;
                pending_count_ = 1u;
            }
            candidate.valid =
                pending_count_ >= config_.startup_confirm_samples;
            candidate.locked = candidate.valid;
            accepted = candidate.locked;
        } else {
            const float output_direction =
                config_.output_reversed ? -1.0f : 1.0f;
            const float main_delta_rad = wrap_pm_pi(
                candidate.raw_main_phase_rad - accepted_main_phase_rad_);
            const float expected_unique_position_rad = wrap_0_2pi(
                (accepted_unique_position_rad_ +
                 output_direction * main_delta_rad / config_.main_ratio) *
                kTwoPi / kUniqueRangeRad) *
                kUniqueRangeRad / kTwoPi;
            float position_difference_rad =
                candidate.unique_position_rad - expected_unique_position_rad;
            position_difference_rad -= kUniqueRangeRad * floorf(
                position_difference_rad / kUniqueRangeRad + 0.5f);
            if (std::abs(position_difference_rad) <=
                config_.branch_continuity_tolerance_rad) {
                candidate.valid = true;
                candidate.locked = true;
                accepted = true;
            } else {
                candidate.valid = result_.valid;
                candidate.locked = false;
                candidate.fault = RESOLVER_FAULT_BRANCH_JUMP;
            }
        }
    } else {
        candidate.valid = result_.valid;
        candidate.locked = false;
    }

    if (accepted) {
        accepted_position_valid_ = true;
        accepted_unique_position_rad_ = candidate.unique_position_rad;
        accepted_main_phase_rad_ = candidate.raw_main_phase_rad;
        mismatch_count_ = 0;
        pending_branch_ = -1;
        pending_count_ = 0;
    } else {
        mismatch_count_ = static_cast<uint8_t>(mismatch_count_ + 1u);
        if (mismatch_count_ > config_.max_consecutive_mismatch) {
            candidate.valid = false;
            accepted_position_valid_ = false;
            pending_branch_ = -1;
            pending_count_ = 0;
        }
    }
    result_ = candidate;
    return result_;
}

void ContinuousPositionTracker::init(const TrackerConfig& config) {
    config_ = config;
    reset();
}

void ContinuousPositionTracker::reset() {
    valid_ = false;
    previous_main_count_ = 0;
    main_unwrapped_count_ = 0;
    anchor_main_unwrapped_count_ = 0;
    anchor_position_rad_ = 0.0f;
    output_position_rad_ = 0.0f;
    runtime_unique_range_index_ = 0;
    fault_ = TRACKER_FAULT_NONE;
}

bool ContinuousPositionTracker::initialize(
        uint16_t main_count, const ResolverResult& absolute) {
    if (!absolute.valid || !absolute.locked ||
        config_.counts_per_rev == 0u ||
        !std::isfinite(config_.main_ratio) ||
        std::abs(config_.main_ratio) < 1.0e-6f) {
        return false;
    }
    previous_main_count_ = main_count % config_.counts_per_rev;
    main_unwrapped_count_ = previous_main_count_;
    anchor_main_unwrapped_count_ = main_unwrapped_count_;
    anchor_position_rad_ = absolute.unique_position_rad;
    output_position_rad_ = anchor_position_rad_;
    runtime_unique_range_index_ = 0;
    fault_ = TRACKER_FAULT_NONE;
    valid_ = true;
    return true;
}

bool ContinuousPositionTracker::update(uint16_t main_count, float dt) {
    if (!valid_ || !(dt > 0.0f) ||
        dt > config_.maximum_sample_interval_s) {
        fault_ |= TRACKER_FAULT_INVALID_DT;
        return false;
    }
    const int32_t cpr = config_.counts_per_rev;
    int32_t delta = static_cast<int32_t>(main_count % cpr) -
                    static_cast<int32_t>(previous_main_count_);
    if (delta > cpr / 2) delta -= cpr;
    if (delta <= -cpr / 2) delta += cpr;

    const float direction = (config_.main_reversed ? -1.0f : 1.0f) *
                            (config_.output_reversed ? -1.0f : 1.0f);
    const float delta_position_rad =
        direction * kTwoPi * static_cast<float>(delta) /
        (static_cast<float>(cpr) * config_.main_ratio);
    const float max_delta = config_.maximum_output_speed_rpm *
                            kTwoPi / 60.0f * dt +
                            kTwoPi /
                            (static_cast<float>(cpr) * config_.main_ratio);
    if (std::abs(delta_position_rad) > max_delta) {
        fault_ |= TRACKER_FAULT_ALIASED_JUMP;
        return false;
    }

    previous_main_count_ = main_count % cpr;
    main_unwrapped_count_ += delta;
    output_position_rad_ = anchor_position_rad_ +
        direction * kTwoPi * static_cast<float>(
            main_unwrapped_count_ - anchor_main_unwrapped_count_) /
        (static_cast<float>(cpr) * config_.main_ratio);
    runtime_unique_range_index_ = static_cast<int32_t>(
        floorf(output_position_rad_ / kUniqueRangeRad));
    return true;
}

bool ContinuousPositionTracker::check_vernier(
        const ResolverResult& absolute, float tolerance_rad) {
    if (!valid_ || !absolute.valid || !absolute.locked) {
        return false;
    }
    float current_unique = fmodf(output_position_rad_, kUniqueRangeRad);
    if (current_unique < 0.0f) current_unique += kUniqueRangeRad;
    float difference = absolute.unique_position_rad - current_unique;
    difference -= kUniqueRangeRad *
        floorf(difference / kUniqueRangeRad + 0.5f);
    if (std::abs(difference) > tolerance_rad) {
        fault_ |= TRACKER_FAULT_VERNIER_JUMP;
        return false;
    }
    return true;
}

void ContinuousPositionPll::init(const PllConfig& config) {
    config_ = config;
    reset();
}

void ContinuousPositionPll::reset() {
    valid_ = false;
    position_estimate_rad_ = 0.0f;
    velocity_estimate_rpm_ = 0.0f;
    phase_error_rad_ = 0.0f;
    time_since_observation_s_ = 0.0f;
    fault_ = PLL_FAULT_NONE;
}

bool ContinuousPositionPll::update(float observation_position_rad, float dt) {
    if (!(dt > 0.0f) || dt > config_.maximum_sample_interval_s ||
        !std::isfinite(observation_position_rad)) {
        fault_ |= PLL_FAULT_INVALID_DT;
        return false;
    }
    if (!valid_) {
        position_estimate_rad_ = observation_position_rad;
        velocity_estimate_rpm_ = 0.0f;
        phase_error_rad_ = 0.0f;
        time_since_observation_s_ = 0.0f;
        valid_ = true;
        return true;
    }

    const float velocity_rad_per_s =
        velocity_estimate_rpm_ * kTwoPi / 60.0f;
    const float predicted_position =
        position_estimate_rad_ + velocity_rad_per_s * dt;
    const float error = observation_position_rad - predicted_position;
    if (std::abs(error) > config_.maximum_position_step_rad) {
        fault_ |= PLL_FAULT_POSITION_JUMP;
        valid_ = false;
        return false;
    }

    const float kp = 2.0f * config_.damping *
                     config_.bandwidth_rad_per_s;
    const float ki = config_.bandwidth_rad_per_s *
                     config_.bandwidth_rad_per_s;
    position_estimate_rad_ = predicted_position + kp * dt * error;
    const float corrected_velocity_rad_per_s =
        velocity_rad_per_s + ki * dt * error;
    velocity_estimate_rpm_ =
        corrected_velocity_rad_per_s * 60.0f / kTwoPi;
    if (std::abs(velocity_estimate_rpm_) <
        config_.zero_speed_threshold_rpm) {
        velocity_estimate_rpm_ = 0.0f;
    }
    phase_error_rad_ = error;
    time_since_observation_s_ = 0.0f;
    return true;
}

void ContinuousPositionPll::note_missing_sample(float dt) {
    if (!(dt > 0.0f)) return;
    time_since_observation_s_ += dt;
    if (time_since_observation_s_ > config_.maximum_sample_interval_s) {
        fault_ |= PLL_FAULT_TIMEOUT;
        valid_ = false;
    }
}

}  // namespace lz5710
