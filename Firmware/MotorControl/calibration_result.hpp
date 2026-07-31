#ifndef __CALIBRATION_RESULT_HPP
#define __CALIBRATION_RESULT_HPP

#include <stdint.h>
#include <array>

// Runtime-only candidate values. Nothing in this structure is active or
// persistent until the later validation and transactional commit stages.
struct CalibrationPendingResult {
    enum Validity : uint32_t {
        VALID_PHASE_RESISTANCE = 1u << 0,
        VALID_PHASE_INDUCTANCE = 1u << 1,
        VALID_ENCODER_DIRECTION = 1u << 2,
        VALID_ELECTRICAL_OFFSET = 1u << 3,
        VALID_GEOMETRY_MODEL = 1u << 4,
        VALID_FLUX_LINKAGE = 1u << 5,
        VALID_POLE_PAIRS = 1u << 6,
        VALID_MECHANICAL_MODEL = 1u << 7,
        VALID_ELECTRICAL_DELAY = 1u << 8,
        VALID_VERNIER_OFFSETS = 1u << 9,
    };

    uint32_t session_id = 0;
    uint32_t validity = 0;
    float phase_resistance = 0.0f;
    float phase_inductance = 0.0f;
    int32_t encoder_direction = 0;
    int32_t phase_offset = 0;
    float phase_offset_float = 0.0f;
    float vernier_main_offset_rad = 0.0f;
    float vernier_aux_offset_rad = 0.0f;
    float vernier_fit_rms_rad = 0.0f;
    float vernier_worst_residual_rad = 0.0f;
    float vernier_minimum_margin_rad = 0.0f;
    uint32_t vernier_used_samples = 0;
    float effective_ratio_scale = 1.0f;
    std::array<float, 64> common_geometry_correction = {};
    std::array<float, 64> direction_geometry_correction = {};
    float geometry_raw_rms = 0.0f;
    float geometry_corrected_rms = 0.0f;
    float geometry_direction_peak_to_peak = 0.0f;
    uint32_t geometry_used_samples = 0;
    float flux_linkage = 0.0f;
    float torque_constant = 0.0f;
    float flux_sample_stddev = 0.0f;
    float flux_mean_std_error = 0.0f;
    uint32_t flux_used_samples = 0;
    int32_t pole_pairs = 0;
    float output_inertia = 0.0f;
    float friction_coulomb_pos = 0.0f;
    float friction_coulomb_neg = 0.0f;
    float friction_viscous_pos = 0.0f;
    float friction_viscous_neg = 0.0f;
    float mechanical_residual_rms_torque = 0.0f;
    uint32_t mechanical_used_samples = 0;
    uint32_t mechanical_attempted_samples = 0;
    uint32_t mechanical_rejected_invalid = 0;
    uint32_t mechanical_rejected_saturated = 0;
    uint32_t mechanical_rejected_low_velocity = 0;
    float mechanical_max_abs_velocity = 0.0f;
    uint32_t mechanical_rejected_timing = 0;
    float electrical_delay = 0.0f;
    float delay_residual_phase_offset = 0.0f;
    float delay_residual_rms = 0.0f;
    uint32_t delay_used_samples = 0;
    uint32_t delay_attempted_samples = 0;
    uint32_t delay_rejected_invalid = 0;
    uint32_t delay_rejected_saturated = 0;
    uint32_t delay_rejected_speed = 0;
    uint32_t delay_rejected_emf = 0;
    uint32_t delay_rejected_phase = 0;
    float delay_max_abs_electrical_speed = 0.0f;
};

#endif // __CALIBRATION_RESULT_HPP
