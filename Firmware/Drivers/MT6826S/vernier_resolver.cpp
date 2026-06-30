#include "vernier_resolver.hpp"

#include <math.h>

void VernierResolver::init(const Config& config) {
    config_ = config;
    reset();

    if (!validate_config()) {
        error_ |= ERROR_BAD_CONFIG;
        state_ = STATE_ERROR;
    }
}

void VernierResolver::reset() {
    state_ = STATE_UNINITIALIZED;
    error_ = ERROR_NONE;

    has_last_phase_ = false;
    last_main_phase_corr_ = 0.0f;
    has_last_output_phase_ = false;
    last_output_phase_ = 0.0f;
    main_cycle_index_ = 0;
    position_turns_ = 0.0f;

    pending_k_ = 0;
    pending_count_ = 0;

    consecutive_good_ = 0;
    consecutive_mismatch_ = 0;
    consecutive_aux_miss_ = 0;

    last_result_ = {};
}

bool VernierResolver::validate_config() const {
    if (config_.angle_counts_per_rev == 0) {
        return false;
    }

    if (absf(config_.main_ratio) < 1.0e-6f) {
        return false;
    }

    if (absf(config_.aux_ratio) < 1.0e-6f) {
        return false;
    }

    if (config_.max_main_cycle_index < 0) {
        return false;
    }

    if (config_.runtime_search_radius < 0) {
        return false;
    }

    if (config_.err_accept < 0.0f || config_.err_accept > 0.5f) {
        return false;
    }

    if (config_.err_reject < config_.err_accept || config_.err_reject > 0.5f) {
        return false;
    }

    if (config_.startup_confirm_frames < 1 || config_.relock_confirm_frames < 1) {
        return false;
    }

    if (config_.max_mismatch_frames < 1 || config_.max_aux_miss_frames < 1) {
        return false;
    }

    return true;
}

float VernierResolver::wrap01(float x) {
    float y = x - floorf(x);
    if (y >= 1.0f) {
        y -= 1.0f;
    }
    if (y < 0.0f) {
        y += 1.0f;
    }
    return y;
}

float VernierResolver::wrap_pm_half(float x) {
    float y = wrap01(x + 0.5f) - 0.5f;
    // Keep the interval half-open: [-0.5, 0.5).
    if (y >= 0.5f) {
        y -= 1.0f;
    }
    return y;
}

float VernierResolver::normalize_raw_angle(uint16_t angle, bool reversed) const {
    const float denom = static_cast<float>(config_.angle_counts_per_rev);
    float phase = static_cast<float>(angle % config_.angle_counts_per_rev) / denom;

    if (reversed) {
        phase = wrap01(-phase);
    }

    return phase;
}

float VernierResolver::corrected_phase(uint16_t angle, bool reversed, float offset) const {
    return wrap01(normalize_raw_angle(angle, reversed) - offset);
}

int32_t VernierResolver::predict_main_cycle_from_phase(float main_phase_corr) const {
    if (!has_last_phase_) {
        return main_cycle_index_;
    }

    int32_t k = main_cycle_index_;
    const float delta = main_phase_corr - last_main_phase_corr_;

    // Forward wrap: 0.99 -> 0.01.
    if (delta < -0.5f) {
        ++k;
    }

    // Reverse wrap: 0.01 -> 0.99.
    if (delta > 0.5f) {
        --k;
    }

    return k;
}

VernierResolver::Candidate VernierResolver::evaluate_candidate(int32_t k,
                                                               float main_phase_corr,
                                                               float aux_phase_corr) const {
    Candidate candidate = {};
    candidate.k = k;
    candidate.position_turns = (static_cast<float>(k) + main_phase_corr) / config_.main_ratio;
    candidate.aux_pred_phase = wrap01(config_.aux_ratio * candidate.position_turns);
    candidate.residual = wrap_pm_half(aux_phase_corr - candidate.aux_pred_phase);
    candidate.abs_residual = absf(candidate.residual);
    candidate.valid = position_within_limit(candidate.position_turns);
    return candidate;
}

VernierResolver::Candidate VernierResolver::search_candidates(int32_t k_min,
                                                              int32_t k_max,
                                                              float main_phase_corr,
                                                              float aux_phase_corr) const {
    Candidate best = {};
    best.abs_residual = 1.0e30f;
    best.valid = false;

    if (k_max < k_min) {
        return best;
    }

    for (int32_t k = k_min; k <= k_max; ++k) {
        const Candidate candidate = evaluate_candidate(k, main_phase_corr, aux_phase_corr);
        if (!candidate.valid) {
            continue;
        }

        if (!best.valid || candidate.abs_residual < best.abs_residual) {
            best = candidate;
        }
    }

    return best;
}

VernierResolver::Result VernierResolver::make_result_base(uint16_t main_angle,
                                                          uint16_t aux_angle,
                                                          bool aux_angle_present,
                                                          float main_phase,
                                                          float aux_phase,
                                                          float main_phase_corr,
                                                          float aux_phase_corr) const {
    Result result = {};
    result.state = state_;
    result.error = error_;

    result.main_angle = main_angle;
    result.aux_angle = aux_angle_present ? aux_angle : 0;

    result.main_phase = main_phase;
    result.aux_phase = aux_angle_present ? aux_phase : 0.0f;
    result.main_phase_corr = main_phase_corr;
    result.aux_phase_corr = aux_angle_present ? aux_phase_corr : 0.0f;

    result.main_cycle_index = main_cycle_index_;
    result.main_unwrapped = static_cast<float>(main_cycle_index_) + main_phase_corr;
    result.position_turns = position_turns_;

    result.consecutive_good = consecutive_good_;
    result.consecutive_mismatch = consecutive_mismatch_;
    result.consecutive_aux_miss = consecutive_aux_miss_;

    return result;
}

float VernierResolver::confidence_from_residual(float abs_residual) const {
    if (config_.err_reject <= 0.0f) {
        return 0.0f;
    }

    float confidence = 1.0f - (abs_residual / config_.err_reject);
    if (confidence < 0.0f) {
        confidence = 0.0f;
    }
    if (confidence > 1.0f) {
        confidence = 1.0f;
    }
    return confidence;
}

bool VernierResolver::position_within_limit(float position_turns) const {
    if (config_.max_abs_position_turns <= 0.0f) {
        return true;
    }

    return absf(position_turns) <= config_.max_abs_position_turns;
}

void VernierResolver::update_pending_lock(int32_t candidate_k, bool startup_mode) {
    if (pending_count_ == 0 || pending_k_ != candidate_k) {
        pending_k_ = candidate_k;
        pending_count_ = 1;
    } else {
        ++pending_count_;
    }

    const int32_t required = startup_mode ? config_.startup_confirm_frames
                                          : config_.relock_confirm_frames;

    if (pending_count_ >= required) {
        state_ = STATE_LOCKED;
    } else {
        state_ = STATE_ACQUIRING;
    }
}

VernierResolver::Result VernierResolver::accept_candidate(const Candidate& candidate,
                                                          uint16_t main_angle,
                                                          uint16_t aux_angle,
                                                          float main_phase,
                                                          float aux_phase,
                                                          float main_phase_corr,
                                                          float aux_phase_corr,
                                                          bool accepted_aux) {
    const State previous_state = state_;
    const bool startup_mode = (previous_state == STATE_UNINITIALIZED || previous_state == STATE_ACQUIRING);

    main_cycle_index_ = candidate.k;
    position_turns_ = candidate.position_turns;
    last_main_phase_corr_ = main_phase_corr;
    has_last_phase_ = true;

    if (accepted_aux) {
        if (previous_state == STATE_LOCKED) {
            state_ = STATE_LOCKED;
            pending_count_ = 0;
        } else {
            update_pending_lock(candidate.k, startup_mode);
            if (previous_state == STATE_SUSPECT && state_ == STATE_ACQUIRING) {
                state_ = STATE_SUSPECT;
            }
        }

        consecutive_aux_miss_ = 0;
        consecutive_mismatch_ = 0;
        ++consecutive_good_;
    }

    if (state_ == STATE_LOCKED) {
        pending_count_ = 0;
    }

    Result result = make_result_base(main_angle, aux_angle, true,
                                     main_phase, aux_phase,
                                     main_phase_corr, aux_phase_corr);

    result.state = state_;
    result.error = error_;
    result.valid = (state_ == STATE_LOCKED || state_ == STATE_SUSPECT);
    result.locked = (state_ == STATE_LOCKED);
    result.accepted_aux = accepted_aux;
    result.degraded = (state_ == STATE_SUSPECT);

    result.main_cycle_index = main_cycle_index_;
    result.main_unwrapped = static_cast<float>(main_cycle_index_) + main_phase_corr;
    result.position_turns = position_turns_;
    result.aux_pred_phase = candidate.aux_pred_phase;
    result.residual_turns = candidate.residual;
    result.abs_residual_turns = candidate.abs_residual;
    result.confidence = confidence_from_residual(candidate.abs_residual);
    result.consecutive_good = consecutive_good_;
    result.consecutive_mismatch = consecutive_mismatch_;
    result.consecutive_aux_miss = consecutive_aux_miss_;

    last_result_ = result;
    return result;
}

VernierResolver::Result VernierResolver::reject_aux_and_propagate(uint16_t main_angle,
                                                                  uint16_t aux_angle,
                                                                  bool aux_angle_present,
                                                                  float main_phase,
                                                                  float aux_phase,
                                                                  float main_phase_corr,
                                                                  float aux_phase_corr,
                                                                  Error error_to_set) {
    error_ |= error_to_set;

    if (state_ == STATE_LOCKED || state_ == STATE_SUSPECT) {
        const int32_t predicted_k = predict_main_cycle_from_phase(main_phase_corr);
        main_cycle_index_ = predicted_k;
        position_turns_ = (static_cast<float>(main_cycle_index_) + main_phase_corr) / config_.main_ratio;
        last_main_phase_corr_ = main_phase_corr;
        has_last_phase_ = true;

        if (!position_within_limit(position_turns_)) {
            error_ |= ERROR_OUTPUT_LIMIT;
            state_ = STATE_ERROR;
        } else {
            state_ = STATE_SUSPECT;
        }
    } else {
        state_ = STATE_ACQUIRING;
        error_ |= ERROR_NO_LOCK;
    }

    if (error_to_set == ERROR_AUX_INVALID) {
        ++consecutive_aux_miss_;
        if (consecutive_aux_miss_ > config_.max_aux_miss_frames) {
            error_ |= ERROR_EXCESSIVE_MISSES;
            state_ = STATE_ERROR;
        }
    } else {
        ++consecutive_mismatch_;
        if (consecutive_mismatch_ > config_.max_mismatch_frames) {
            state_ = STATE_ERROR;
        }
    }

    consecutive_good_ = 0;
    pending_count_ = 0;

    Result result = make_result_base(main_angle, aux_angle, aux_angle_present,
                                     main_phase, aux_phase,
                                     main_phase_corr, aux_phase_corr);
    result.state = state_;
    result.error = error_;
    result.valid = (state_ == STATE_LOCKED || state_ == STATE_SUSPECT);
    result.locked = false;
    result.accepted_aux = false;
    result.degraded = result.valid;
    result.main_cycle_index = main_cycle_index_;
    result.main_unwrapped = static_cast<float>(main_cycle_index_) + main_phase_corr;
    result.position_turns = position_turns_;
    result.confidence = 0.0f;
    result.consecutive_good = consecutive_good_;
    result.consecutive_mismatch = consecutive_mismatch_;
    result.consecutive_aux_miss = consecutive_aux_miss_;

    last_result_ = result;
    return result;
}

VernierResolver::Result VernierResolver::update(uint16_t main_angle,
                                                bool main_valid,
                                                uint16_t aux_angle,
                                                bool aux_valid) {
    if (config_.use_phase_difference) {
        return update_phase_difference(main_angle, main_valid, aux_angle, aux_valid);
    }

    if (!validate_config()) {
        error_ |= ERROR_BAD_CONFIG;
        state_ = STATE_ERROR;
        last_result_.state = state_;
        last_result_.error = error_;
        last_result_.valid = false;
        return last_result_;
    }

    if (!main_valid) {
        error_ |= ERROR_MAIN_INVALID;
        state_ = STATE_ERROR;
        last_result_.state = state_;
        last_result_.error = error_;
        last_result_.valid = false;
        return last_result_;
    }

    const float main_phase = normalize_raw_angle(main_angle, config_.main_reversed);
    const float main_phase_corr = wrap01(main_phase - config_.main_offset);

    if (!aux_valid) {
        return reject_aux_and_propagate(main_angle, 0, false,
                                        main_phase, 0.0f,
                                        main_phase_corr, 0.0f,
                                        ERROR_AUX_INVALID);
    }

    const float aux_phase = normalize_raw_angle(aux_angle, config_.aux_reversed);
    const float aux_phase_corr = wrap01(aux_phase - config_.aux_offset);

    Candidate best = {};

    if (state_ == STATE_UNINITIALIZED || state_ == STATE_ACQUIRING || !has_last_phase_) {
        best = search_candidates(-config_.max_main_cycle_index,
                                  config_.max_main_cycle_index,
                                  main_phase_corr,
                                  aux_phase_corr);
    } else {
        const int32_t predicted_k = predict_main_cycle_from_phase(main_phase_corr);
        best = search_candidates(predicted_k - config_.runtime_search_radius,
                                  predicted_k + config_.runtime_search_radius,
                                  main_phase_corr,
                                  aux_phase_corr);
    }

    if (!best.valid) {
        return reject_aux_and_propagate(main_angle, aux_angle, true,
                                        main_phase, aux_phase,
                                        main_phase_corr, aux_phase_corr,
                                        ERROR_OUTPUT_LIMIT);
    }

    if (best.abs_residual <= config_.err_accept) {
        return accept_candidate(best, main_angle, aux_angle,
                                main_phase, aux_phase,
                                main_phase_corr, aux_phase_corr,
                                true);
    }

    // In LOCKED state, tolerate residuals between accept and reject as long as
    // the branch remains plausible.  This keeps the resolver from chattering
    // due to eccentricity/noise while still rejecting gross branch errors.
    if ((state_ == STATE_LOCKED || state_ == STATE_SUSPECT) &&
        best.abs_residual <= config_.err_reject) {
        state_ = STATE_SUSPECT;
        pending_count_ = 0;
        return accept_candidate(best, main_angle, aux_angle,
                                main_phase, aux_phase,
                                main_phase_corr, aux_phase_corr,
                                false);
    }

    return reject_aux_and_propagate(main_angle, aux_angle, true,
                                    main_phase, aux_phase,
                                    main_phase_corr, aux_phase_corr,
                                    ERROR_RESIDUAL_TOO_HIGH);
}

VernierResolver::Result VernierResolver::update_phase_difference(uint16_t main_angle,
                                                                 bool main_valid,
                                                                 uint16_t aux_angle,
                                                                 bool aux_valid) {
    if (!validate_config()) {
        error_ |= ERROR_BAD_CONFIG;
        state_ = STATE_ERROR;
        last_result_.state = state_;
        last_result_.error = error_;
        last_result_.valid = false;
        return last_result_;
    }

    if (!main_valid) {
        error_ |= ERROR_MAIN_INVALID;
        state_ = STATE_ERROR;
        last_result_.state = state_;
        last_result_.error = error_;
        last_result_.valid = false;
        return last_result_;
    }

    const float main_phase = normalize_raw_angle(main_angle, config_.main_reversed);
    const float main_phase_corr = wrap01(main_phase - config_.main_offset);

    if (!aux_valid) {
        error_ |= ERROR_AUX_INVALID;

        if ((state_ == STATE_LOCKED || state_ == STATE_SUSPECT) && has_last_phase_) {
            const float delta_main = wrap_pm_half(main_phase_corr - last_main_phase_corr_);
            position_turns_ += delta_main / config_.main_ratio;
            last_main_phase_corr_ = main_phase_corr;
            state_ = STATE_SUSPECT;

            ++consecutive_aux_miss_;
            consecutive_good_ = 0;
            if (consecutive_aux_miss_ > config_.max_aux_miss_frames) {
                error_ |= ERROR_EXCESSIVE_MISSES;
                state_ = STATE_ERROR;
            }
        } else {
            state_ = STATE_ACQUIRING;
            error_ |= ERROR_NO_LOCK;
            ++consecutive_aux_miss_;
            consecutive_good_ = 0;
        }

        Result result = make_result_base(main_angle, 0, false,
                                         main_phase, 0.0f,
                                         main_phase_corr, 0.0f);
        result.state = state_;
        result.error = error_;
        result.valid = (state_ == STATE_LOCKED || state_ == STATE_SUSPECT);
        result.locked = false;
        result.accepted_aux = false;
        result.degraded = result.valid;
        result.main_unwrapped = config_.main_ratio * position_turns_;
        result.main_cycle_index = static_cast<int32_t>(floorf(result.main_unwrapped));
        result.position_turns = position_turns_;
        result.confidence = 0.0f;
        result.consecutive_good = consecutive_good_;
        result.consecutive_mismatch = consecutive_mismatch_;
        result.consecutive_aux_miss = consecutive_aux_miss_;
        last_result_ = result;
        return result;
    }

    const float aux_phase = normalize_raw_angle(aux_angle, config_.aux_reversed);
    const float aux_phase_corr = wrap01(aux_phase - config_.aux_offset);
    const float output_phase = config_.output_reversed
        ? wrap01(main_phase_corr - aux_phase_corr)
        : wrap01(aux_phase_corr - main_phase_corr);

    if (!has_last_output_phase_) {
        position_turns_ = output_phase;
    } else {
        position_turns_ += wrap_pm_half(output_phase - last_output_phase_);
    }

    if (!position_within_limit(position_turns_)) {
        error_ |= ERROR_OUTPUT_LIMIT;
        state_ = STATE_ERROR;
    } else {
        state_ = STATE_LOCKED;
    }

    has_last_output_phase_ = true;
    last_output_phase_ = output_phase;
    has_last_phase_ = true;
    last_main_phase_corr_ = main_phase_corr;
    main_cycle_index_ = static_cast<int32_t>(floorf(config_.main_ratio * position_turns_));

    consecutive_aux_miss_ = 0;
    consecutive_mismatch_ = 0;
    ++consecutive_good_;
    pending_count_ = 0;

    Result result = make_result_base(main_angle, aux_angle, true,
                                     main_phase, aux_phase,
                                     main_phase_corr, aux_phase_corr);
    result.state = state_;
    result.error = error_;
    result.valid = (state_ == STATE_LOCKED);
    result.locked = (state_ == STATE_LOCKED);
    result.accepted_aux = (state_ == STATE_LOCKED);
    result.degraded = false;
    result.main_cycle_index = main_cycle_index_;
    result.main_unwrapped = config_.main_ratio * position_turns_;
    result.position_turns = position_turns_;
    result.aux_pred_phase = aux_phase_corr;
    result.residual_turns = 0.0f;
    result.abs_residual_turns = 0.0f;
    result.confidence = result.valid ? 1.0f : 0.0f;
    result.consecutive_good = consecutive_good_;
    result.consecutive_mismatch = consecutive_mismatch_;
    result.consecutive_aux_miss = consecutive_aux_miss_;

    last_result_ = result;
    return result;
}

VernierResolver::Result VernierResolver::update_main_only(uint16_t main_angle, bool main_valid) {
    return update(main_angle, main_valid, 0, false);
}

bool VernierResolver::get_virtual_count(int32_t virtual_cpr, int32_t* out) const {
    if (!out || virtual_cpr <= 0 || !last_result_.valid) {
        return false;
    }

    const float count_f = last_result_.position_turns * static_cast<float>(virtual_cpr);

    // Avoid pulling in lroundf for portability across embedded libm builds.
    if (count_f >= 0.0f) {
        *out = static_cast<int32_t>(count_f + 0.5f);
    } else {
        *out = static_cast<int32_t>(count_f - 0.5f);
    }

    return true;
}
