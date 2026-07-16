#ifndef __CALIBRATION_SESSION_HPP
#define __CALIBRATION_SESSION_HPP

#include <stdint.h>

// Runtime transaction state for one host-orchestrated calibration session.
// This deliberately does not persist results. A validated result will later be
// staged into the versioned A/B Calibration Blob; until then the existing
// configuration remains authoritative.
class CalibrationSession {
public:
    static constexpr uint32_t kSchemaVersion = 1;

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

    bool begin(uint32_t requested_id) {
        if (is_active()) {
            return false;
        }
        if (requested_id == 0) {
            requested_id = session_id_ + 1;
            if (requested_id == 0) {
                requested_id = 1;
            }
        }
        session_id_ = requested_id;
        state_ = STATE_COLLECTING;
        stage_ = 0;
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
        stage_ = 0;
        failure_code_ = 0;
        ++transition_count_;
        return true;
    }

    uint32_t state() const { return static_cast<uint32_t>(state_); }
    uint32_t session_id() const { return session_id_; }
    uint32_t stage() const { return stage_; }
    uint32_t failure_code() const { return failure_code_; }
    uint32_t transition_count() const { return transition_count_; }

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
    uint32_t stage_ = 0;
    uint32_t failure_code_ = 0;
    uint32_t transition_count_ = 0;
};

#endif // __CALIBRATION_SESSION_HPP
