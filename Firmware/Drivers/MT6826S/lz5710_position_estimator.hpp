#pragma once

#include <stdint.h>

namespace lz5710 {

constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kMainRatio = 10.0f;
constexpr float kAuxRatio = 220.0f / 21.0f;
constexpr int32_t kBranchCount = 21;
constexpr float kUniqueRangeRad = kTwoPi * 21.0f / 10.0f;

float wrap_0_2pi(float value);
float wrap_pm_pi(float value);

struct ResolverConfig {
    uint16_t counts_per_rev = 32768;
    float main_ratio = kMainRatio;
    float aux_ratio = kAuxRatio;
    float main_offset_rad = 0.0f;
    float aux_offset_rad = 0.0f;
    bool main_reversed = false;
    bool aux_reversed = true;
    bool output_reversed = false;
    float residual_accept_rad = 0.03f;
    float residual_reject_rad = 0.08f;
    float ambiguity_margin_rad = 0.04f;
    float branch_continuity_tolerance_rad = 0.10f;
    uint8_t startup_confirm_samples = 3;
    uint8_t max_consecutive_mismatch = 3;
};

enum ResolverFault : uint32_t {
    RESOLVER_FAULT_NONE = 0,
    RESOLVER_FAULT_MAIN_INVALID = 1u << 0,
    RESOLVER_FAULT_AUX_INVALID = 1u << 1,
    RESOLVER_FAULT_NO_SOLUTION = 1u << 2,
    RESOLVER_FAULT_AMBIGUOUS = 1u << 3,
    RESOLVER_FAULT_RESIDUAL = 1u << 4,
    RESOLVER_FAULT_BRANCH_JUMP = 1u << 5,
};

struct ResolverResult {
    bool valid = false;
    bool locked = false;
    bool ambiguous = false;
    int32_t vernier_branch_index = -1;
    float raw_main_phase_rad = 0.0f;
    float raw_aux_phase_rad = 0.0f;
    float unique_position_rad = 0.0f;
    float wrapped_output_phase_rad = 0.0f;
    float residual_rad = 0.0f;
    float second_best_margin_rad = 0.0f;
    uint32_t fault = RESOLVER_FAULT_NONE;
};

class Resolver {
public:
    void init(const ResolverConfig& config);
    void reset();
    ResolverResult update(uint16_t main_count, bool main_valid,
                          uint16_t aux_count, bool aux_valid);
    const ResolverResult& result() const { return result_; }

private:
    float corrected_phase(uint16_t count, bool reversed, float offset_rad) const;

    ResolverConfig config_ = {};
    ResolverResult result_ = {};
    int32_t pending_branch_ = -1;
    uint8_t pending_count_ = 0;
    uint8_t mismatch_count_ = 0;
    bool accepted_position_valid_ = false;
    float accepted_unique_position_rad_ = 0.0f;
    float accepted_main_phase_rad_ = 0.0f;
};

enum TrackerFault : uint32_t {
    TRACKER_FAULT_NONE = 0,
    TRACKER_FAULT_INVALID_DT = 1u << 0,
    TRACKER_FAULT_ALIASED_JUMP = 1u << 1,
    TRACKER_FAULT_VERNIER_JUMP = 1u << 2,
};

struct TrackerConfig {
    uint16_t counts_per_rev = 32768;
    float main_ratio = kMainRatio;
    bool main_reversed = false;
    bool output_reversed = false;
    float maximum_output_speed_rpm = 600.0f;
    float maximum_sample_interval_s = 0.002f;
};

class ContinuousPositionTracker {
public:
    void init(const TrackerConfig& config);
    void reset();
    bool initialize(uint16_t main_count, const ResolverResult& absolute);
    bool update(uint16_t main_count, float dt);
    bool check_vernier(const ResolverResult& absolute, float tolerance_rad);

    bool valid() const { return valid_; }
    float output_position_rad() const { return output_position_rad_; }
    float wrapped_output_phase_rad() const {
        return wrap_0_2pi(output_position_rad_);
    }
    int64_t main_unwrapped_count() const { return main_unwrapped_count_; }
    int32_t runtime_unique_range_index() const {
        return runtime_unique_range_index_;
    }
    uint32_t fault() const { return fault_; }

private:
    TrackerConfig config_ = {};
    bool valid_ = false;
    uint16_t previous_main_count_ = 0;
    int64_t main_unwrapped_count_ = 0;
    int64_t anchor_main_unwrapped_count_ = 0;
    float anchor_position_rad_ = 0.0f;
    float output_position_rad_ = 0.0f;
    int32_t runtime_unique_range_index_ = 0;
    uint32_t fault_ = TRACKER_FAULT_NONE;
};

enum PllFault : uint32_t {
    PLL_FAULT_NONE = 0,
    PLL_FAULT_INVALID_DT = 1u << 0,
    PLL_FAULT_TIMEOUT = 1u << 1,
    PLL_FAULT_POSITION_JUMP = 1u << 2,
};

struct PllConfig {
    float bandwidth_rad_per_s = 120.0f;
    float damping = 0.70710678f;
    float maximum_sample_interval_s = 0.002f;
    float maximum_position_step_rad = 0.25f;
    float zero_speed_threshold_rpm = 0.01f;
};

class ContinuousPositionPll {
public:
    void init(const PllConfig& config);
    void reset();
    bool update(float observation_position_rad, float dt);
    void note_missing_sample(float dt);

    bool valid() const { return valid_; }
    float position_estimate_rad() const { return position_estimate_rad_; }
    float velocity_estimate_rpm() const { return velocity_estimate_rpm_; }
    float phase_error_rad() const { return phase_error_rad_; }
    float time_since_observation_s() const { return time_since_observation_s_; }
    uint32_t fault() const { return fault_; }

private:
    PllConfig config_ = {};
    bool valid_ = false;
    float position_estimate_rad_ = 0.0f;
    float velocity_estimate_rpm_ = 0.0f;
    float phase_error_rad_ = 0.0f;
    float time_since_observation_s_ = 0.0f;
    uint32_t fault_ = PLL_FAULT_NONE;
};

}  // namespace lz5710
