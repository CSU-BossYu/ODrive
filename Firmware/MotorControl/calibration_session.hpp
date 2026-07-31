#ifndef __CALIBRATION_SESSION_HPP
#define __CALIBRATION_SESSION_HPP

#include <stdint.h>

// Runtime transaction state for one firmware-orchestrated calibration session.
// This deliberately does not persist results. A validated result will later be
// staged into the versioned A/B Calibration Blob; until then the existing
// configuration remains authoritative.
class CalibrationSession {
public:
    static constexpr uint32_t kSchemaVersion = 2;

    enum Profile : uint32_t {
        PROFILE_FULL = 0,
        PROFILE_ELECTRICAL = 1,
        PROFILE_MECHANICAL = 2,
        PROFILE_VALIDATE_ONLY = 3,
    };

    // Stable coarse stage IDs. Fine-grained experiment steps remain internal
    // and can evolve without changing the CAN contract.
    enum Stage : uint32_t {
        STAGE_NONE = 0,
        STAGE_PRECHECK = 1,
        STAGE_ELECTRICAL_CAPTURE = 10,
        STAGE_ENCODER_GEOMETRY = 20,
        STAGE_MECHANICAL_CAPTURE = 30,
        STAGE_ELECTRICAL_DELAY = 35,
        STAGE_FITTING = 40,
        STAGE_VALIDATION = 50,
        STAGE_COMMIT = 60,
    };

    enum State : uint32_t {
        STATE_EMPTY = 0,
        STATE_COLLECTING = 1,
        STATE_COLLECTED = 2,
        STATE_FITTING = 3,
        STATE_IDENTIFIED = 4,
        STATE_VALIDATING = 5,
        STATE_VALIDATED = 6,
        STATE_STAGED = 7,
        STATE_COMMITTED = 8,
        STATE_FAILED = 9,
        STATE_ABORTED = 10,
        STATE_STALE = 11,
    };

    enum Flag : uint32_t {
        FLAG_ACTIVE = 1u << 0,
        FLAG_IDENTIFIED = 1u << 1,
        FLAG_VALIDATED = 1u << 2,
        FLAG_APPLICABLE = 1u << 3,
        FLAG_STAGED = 1u << 4,
        FLAG_COMMITTED = 1u << 5,
        FLAG_STALE = 1u << 6,
        FLAG_TERMINAL = 1u << 7,
    };

    enum FailureCode : uint32_t {
        FAILURE_NONE = 0,
        FAILURE_PROFILE_NOT_IMPLEMENTED = 1,
        FAILURE_MOTOR_CALIBRATION = 2,
        FAILURE_ENCODER_CALIBRATION = 3,
        FAILURE_GEOMETRY_SCAN = 4,
        FAILURE_SAMPLE_LOSS = 5,
        FAILURE_MECHANICAL_SCAN = 6,
        FAILURE_VALIDATION = 7,
        FAILURE_STORAGE = 8,
        FAILURE_ELECTRICAL_DELAY = 9,
        FAILURE_FLUX_INSUFFICIENT_SAMPLES = 10,
        FAILURE_FLUX_NONPHYSICAL_MEAN = 11,
        FAILURE_FLUX_EXCESSIVE_DISPERSION = 12,
        FAILURE_MECHANICAL_INSUFFICIENT_EXCITATION = 13,
        FAILURE_MECHANICAL_SINGULAR_REGRESSION = 14,
        FAILURE_MECHANICAL_NONPHYSICAL_PARAMETERS = 15,
        FAILURE_MECHANICAL_CLOSED_LOOP_START = 16,
        FAILURE_MECHANICAL_TRACKING_TIMEOUT = 17,
        FAILURE_MECHANICAL_ALL_INVALID = 18,
        FAILURE_MECHANICAL_ALL_SATURATED = 19,
        FAILURE_MECHANICAL_NO_MOTION = 20,
        FAILURE_MECHANICAL_ALL_TIMING_INVALID = 21,
        FAILURE_DELAY_CLOSED_LOOP_START = 22,
        FAILURE_DELAY_INSUFFICIENT_SAMPLES = 23,
        FAILURE_DELAY_UNOBSERVABLE_SPEED = 24,
        FAILURE_DELAY_NONPHYSICAL_RESULT = 25,
        FAILURE_VERNIER_INSUFFICIENT_SAMPLES = 26,
        FAILURE_VERNIER_FIT = 27,
        FAILURE_VERNIER_AMBIGUOUS = 28,
        FAILURE_VERNIER_RESIDUAL = 29,
        FAILURE_VERNIER_READY_TIMEOUT = 30,
    };

    bool begin(uint32_t request_options) {
        if (is_active()) {
            return false;
        }
        if ((request_options & 0xFFu) > PROFILE_VALIDATE_ONLY) {
            return false;
        }
        uint32_t next_id = session_id_ + 1;
        if (next_id == 0) {
            next_id = 1;
        }
        session_id_ = next_id;
        request_options_ = request_options;
        state_ = STATE_COLLECTING;
        stage_ = 0;
        progress_permille_ = 0;
        failure_code_ = 0;
        ++transition_count_;
        return true;
    }

    bool transition(State next) {
        if (!is_forward_transition(state_, next)) {
            return false;
        }
        state_ = next;
        ++transition_count_;
        return true;
    }

    bool set_stage(uint32_t stage) {
        if (!is_active()) {
            return false;
        }
        stage_ = stage;
        return true;
    }

    bool set_progress(uint32_t progress_permille) {
        if (!is_active() || progress_permille > 1000) {
            return false;
        }
        progress_permille_ = progress_permille;
        return true;
    }

    bool fail(uint32_t code) {
        if (!is_active() || code == 0) {
            return false;
        }
        failure_code_ = code;
        state_ = STATE_FAILED;
        ++transition_count_;
        return true;
    }

    bool abort() {
        if (!is_active()) {
            return false;
        }
        state_ = STATE_ABORTED;
        ++transition_count_;
        return true;
    }

    bool mark_stale() {
        if (state_ < STATE_IDENTIFIED || state_ == STATE_FAILED ||
            state_ == STATE_ABORTED || state_ == STATE_STALE) {
            return false;
        }
        state_ = STATE_STALE;
        ++transition_count_;
        return true;
    }

    bool reset() {
        if (is_active()) {
            return false;
        }
        state_ = STATE_EMPTY;
        request_options_ = 0;
        stage_ = 0;
        progress_permille_ = 0;
        failure_code_ = 0;
        ++transition_count_;
        return true;
    }

    uint32_t state() const { return static_cast<uint32_t>(state_); }
    uint32_t session_id() const { return session_id_; }
    uint32_t stage() const { return stage_; }
    uint32_t failure_code() const { return failure_code_; }
    uint32_t transition_count() const { return transition_count_; }
    uint32_t request_options() const { return request_options_; }
    Profile profile() const {
        return static_cast<Profile>(request_options_ & 0xFFu);
    }
    uint32_t progress_permille() const { return progress_permille_; }

    uint32_t flags() const {
        uint32_t value = 0;
        if (is_active()) value |= FLAG_ACTIVE;
        if ((state_ >= STATE_IDENTIFIED && state_ <= STATE_COMMITTED) ||
            state_ == STATE_STALE) value |= FLAG_IDENTIFIED;
        if (state_ >= STATE_VALIDATED && state_ <= STATE_COMMITTED) value |= FLAG_VALIDATED | FLAG_APPLICABLE;
        if (state_ == STATE_STAGED) value |= FLAG_STAGED;
        if (state_ == STATE_COMMITTED) value |= FLAG_COMMITTED;
        if (state_ == STATE_STALE) value |= FLAG_STALE;
        if (state_ == STATE_FAILED || state_ == STATE_ABORTED ||
            state_ == STATE_COMMITTED || state_ == STATE_STALE) value |= FLAG_TERMINAL;
        return value;
    }

    bool active() const { return is_active(); }

private:
    bool is_active() const {
        return state_ >= STATE_COLLECTING && state_ <= STATE_STAGED;
    }

    static bool is_forward_transition(State current, State next) {
        switch (current) {
            case STATE_COLLECTING: return next == STATE_COLLECTED;
            case STATE_COLLECTED: return next == STATE_FITTING;
            case STATE_FITTING: return next == STATE_IDENTIFIED;
            case STATE_IDENTIFIED: return next == STATE_VALIDATING;
            case STATE_VALIDATING: return next == STATE_VALIDATED;
            case STATE_VALIDATED: return next == STATE_STAGED;
            case STATE_STAGED: return next == STATE_COMMITTED;
            default: return false;
        }
    }

    State state_ = STATE_EMPTY;
    uint32_t session_id_ = 0;
    uint32_t request_options_ = 0;
    uint32_t stage_ = 0;
    uint32_t progress_permille_ = 0;
    uint32_t failure_code_ = 0;
    uint32_t transition_count_ = 0;
};

#endif // __CALIBRATION_SESSION_HPP
