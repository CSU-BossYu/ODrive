#pragma once

#include <stdint.h>

// VernierResolver
// ----------------
// A small, deterministic resolver for two single-turn absolute encoders whose
// mechanical ratios are close but not equal.  It resolves the integer unwrap of
// the main encoder by checking the auxiliary encoder phase residual.
//
// Coordinate convention:
//   x = controlled mechanical position, in turns
//   main_phase ~= frac(main_ratio * x + main_offset)
//   aux_phase  ~= frac(aux_ratio  * x + aux_offset)
//
// The resolver subtracts offsets internally, so it solves:
//   main_corr = frac(main_phase - main_offset) = frac(main_ratio * x)
//   aux_corr  = frac(aux_phase  - aux_offset)  = frac(aux_ratio  * x)
//
// For gear pairs where abs(aux_ratio - main_ratio) = 1, the direct phase-difference
// path can be used:
//   output_phase = frac((aux_corr - main_corr) / (aux_ratio - main_ratio)) = frac(x)
// This is the preferred path for the 41:42 / 42:1 reduction MT6826S layout.
//
// Output:
//   position_turns = x
//   main_cycle_index = integer unwrap index of main_corr
//   main_unwrapped = main_cycle_index + main_corr = main_ratio * x
//
// Intended ODrive use:
//   shadow_count_ = round(position_turns * virtual_cpr)
//   count_in_cpr_ = main_angle converted to the main encoder CPR
//   phase_ remains derived from the main encoder angle, not from aux.

class VernierResolver {
public:
    static constexpr uint16_t kDefaultAngleCounts = 32768; // MT6826S ANGLE[14:0]

    enum State : uint8_t {
        STATE_UNINITIALIZED = 0,
        STATE_ACQUIRING,
        STATE_LOCKED,
        STATE_SUSPECT,
        STATE_ERROR,
    };

    enum Error : uint32_t {
        ERROR_NONE              = 0,
        ERROR_BAD_CONFIG        = 1u << 0,
        ERROR_MAIN_INVALID      = 1u << 1,
        ERROR_AUX_INVALID       = 1u << 2,
        ERROR_NO_LOCK           = 1u << 3,
        ERROR_RESIDUAL_TOO_HIGH = 1u << 4,
        ERROR_EXCESSIVE_MISSES  = 1u << 5,
        ERROR_OUTPUT_LIMIT      = 1u << 6,
    };

    struct Config {
        // Encoder raw angle modulus.  For MT6826S SPI this is 32768.
        uint16_t angle_counts_per_rev = kDefaultAngleCounts;

        // Ratios from output coordinate x to encoder revolutions.
        // Example: if main advances 50 turns while x advances 1 turn,
        // main_ratio = 50.0f.
        float main_ratio = 1.0f;
        float aux_ratio = 1.0f;

        // Phase offsets in turns, applied after optional reversal.
        // Range may be any real value; it is wrapped to [0, 1).
        float main_offset = 0.0f;
        float aux_offset = 0.0f;

        // If true, raw phase is inverted before offset subtraction.
        bool main_reversed = false;
        bool aux_reversed = false;

        // Startup full-search bounds for main_cycle_index.  This is measured in
        // main encoder cycles, not output turns.
        int32_t max_main_cycle_index = 128;

        // Runtime candidate window around the main-only predicted cycle index.
        // 1 means {pred-1, pred, pred+1}.
        int32_t runtime_search_radius = 1;

        // Residual thresholds are in auxiliary phase turns, range 0..0.5.
        // err_accept: clean lock / relock threshold.
        // err_reject: above this, the auxiliary observation is rejected.
        float err_accept = 0.020f;
        float err_reject = 0.080f;

        // Confirmation frames for startup/relock.  Use 1 for immediate lock.
        int32_t startup_confirm_frames = 3;
        int32_t relock_confirm_frames = 2;

        // How many consecutive rejected aux observations before entering ERROR.
        int32_t max_mismatch_frames = 8;

        // How many consecutive aux-invalid frames can be tolerated while locked.
        int32_t max_aux_miss_frames = 8;

        // Optional output limit, in output turns.  Set <=0 to disable.
        // This is a sanity guard against choosing an impossible Vernier branch.
        float max_abs_position_turns = 0.0f;

        // If true, use the encoder phase difference to select the absolute
        // output branch, then refine within that branch with the main phase.
        // Runtime updates select the main-compatible branch nearest the
        // main-phase prediction, preventing one-motor-turn branch jumps.
        bool use_phase_difference = false;

        // Reverse only the resolved output coordinate in phase-difference mode.
        // This is distinct from main_reversed/aux_reversed, which describe the
        // raw sensor phase direction before offsets.
        bool output_reversed = false;
    };

    struct Result {
        State state = STATE_UNINITIALIZED;
        uint32_t error = ERROR_NONE;

        bool valid = false;       // position_turns can be used; false while acquiring initial lock
        bool locked = false;      // auxiliary residual is currently accepted
        bool accepted_aux = false;// this update accepted an aux observation
        bool degraded = false;    // main-only propagation / suspect state

        uint16_t main_angle = 0;
        uint16_t aux_angle = 0;

        float main_phase = 0.0f;      // raw normalized phase after optional reverse, before offset
        float aux_phase = 0.0f;
        float main_phase_corr = 0.0f; // offset-corrected phase, [0, 1)
        float aux_phase_corr = 0.0f;

        int32_t main_cycle_index = 0;
        float main_unwrapped = 0.0f;  // main_cycle_index + main_phase_corr
        float position_turns = 0.0f;  // output coordinate x

        float aux_pred_phase = 0.0f;
        float residual_turns = 0.0f;  // wrap(aux_phase_corr - aux_pred_phase), [-0.5, 0.5)
        float abs_residual_turns = 0.0f;
        float confidence = 0.0f;      // 1 best, 0 rejected/invalid

        int32_t consecutive_good = 0;
        int32_t consecutive_mismatch = 0;
        int32_t consecutive_aux_miss = 0;
    };

    VernierResolver() = default;

    void init(const Config& config);
    void reset();

    const Config& config() const { return config_; }
    State state() const { return state_; }
    uint32_t error() const { return error_; }
    void clear_error() { error_ = ERROR_NONE; }

    // Main update path for a complete pair sample.  main_valid is strict: if it
    // is false, no position update is produced.  aux_valid can be false after
    // the resolver has locked; in that case the resolver propagates using the
    // main encoder only for a limited number of frames.
    Result update(uint16_t main_angle, bool main_valid, uint16_t aux_angle, bool aux_valid);

    // Explicit main-only propagation.  Equivalent to update(main, true, 0, false)
    // but clearer at call sites.
    Result update_main_only(uint16_t main_angle, bool main_valid);

    Result last_result() const { return last_result_; }

    // Helper for ODrive integration.  Converts output-position turns to integer
    // counts.  Returns false if latest result is not valid or cpr <= 0.
    bool get_virtual_count(int32_t virtual_cpr, int32_t* out) const;

    // Exposed small math helpers for tests / integration code.
    static float wrap01(float x);
    static float wrap_pm_half(float x);
    static float absf(float x) { return x < 0.0f ? -x : x; }

private:
    struct Candidate {
        int32_t k = 0;
        float position_turns = 0.0f;
        float aux_pred_phase = 0.0f;
        float residual = 0.0f;
        float abs_residual = 0.0f;
        bool valid = false;
    };

    bool validate_config() const;

    float normalize_raw_angle(uint16_t angle, bool reversed) const;
    float corrected_phase(uint16_t angle, bool reversed, float offset) const;

    int32_t predict_main_cycle_from_phase(float main_phase_corr) const;
    Candidate evaluate_candidate(int32_t k, float main_phase_corr, float aux_phase_corr) const;
    Candidate search_candidates(int32_t k_min, int32_t k_max,
                                float main_phase_corr,
                                float aux_phase_corr) const;

    Result make_result_base(uint16_t main_angle,
                            uint16_t aux_angle,
                            bool aux_angle_present,
                            float main_phase,
                            float aux_phase,
                            float main_phase_corr,
                            float aux_phase_corr) const;

    Result update_phase_difference(uint16_t main_angle,
                                   bool main_valid,
                                   uint16_t aux_angle,
                                   bool aux_valid);

    Result accept_candidate(const Candidate& candidate,
                            uint16_t main_angle,
                            uint16_t aux_angle,
                            float main_phase,
                            float aux_phase,
                            float main_phase_corr,
                            float aux_phase_corr,
                            bool accepted_aux);

    Result reject_aux_and_propagate(uint16_t main_angle,
                                    uint16_t aux_angle,
                                    bool aux_angle_present,
                                    float main_phase,
                                    float aux_phase,
                                    float main_phase_corr,
                                    float aux_phase_corr,
                                    Error error_to_set);

    void update_pending_lock(int32_t candidate_k, bool startup_mode);
    float confidence_from_residual(float abs_residual) const;
    bool position_within_limit(float position_turns) const;

    Config config_ = {};
    State state_ = STATE_UNINITIALIZED;
    uint32_t error_ = ERROR_NONE;

    bool has_last_phase_ = false;
    float last_main_phase_corr_ = 0.0f;
    bool has_last_output_phase_ = false;
    float last_output_phase_ = 0.0f;
    int32_t main_cycle_index_ = 0;
    float position_turns_ = 0.0f;

    int32_t pending_k_ = 0;
    int32_t pending_count_ = 0;

    int32_t consecutive_good_ = 0;
    int32_t consecutive_mismatch_ = 0;
    int32_t consecutive_aux_miss_ = 0;

    Result last_result_ = {};
};
