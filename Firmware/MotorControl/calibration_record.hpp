#ifndef __CALIBRATION_RECORD_HPP
#define __CALIBRATION_RECORD_HPP

#include <stdint.h>

enum CalibrationRecordType : uint16_t {
    CAL_RECORD_FULL_V1 = 1,
    CAL_RECORD_ELECTRICAL_FAST_V1 = 2,
};

// Wire-stable complete sample for long/decimated scans.
struct CalibrationSampleV1 {
    uint32_t sample_ticks;
    uint32_t control_ticks;
    uint32_t sequence;
    uint32_t session_id;
    uint32_t main_sequence;
    uint32_t aux_sequence;
    uint32_t pair_sequence;
    uint32_t pair_sample_skew_cycles;

    uint16_t main_raw;
    uint16_t aux_raw;
    uint16_t axis_state;
    uint16_t flags;

    float ia;
    float ib;
    float ic;
    float id;
    float iq;
    float vd_applied;
    float vq_applied;
    float vbus;
    float duty_a;
    float duty_b;
    float duty_c;
    float position_turns;
    float velocity_turns_per_s;
    float electrical_phase;
    float electrical_velocity;
    float output_position_turns;
    float board_temperature;
    float motor_temperature;
};

static_assert(sizeof(CalibrationSampleV1) == 112,
              "CalibrationSampleV1 wire layout changed");

// Compact 8 kHz electrical sample. Id/Iq are intentionally reconstructed
// offline from phase currents and electrical_phase, avoiding redundant fields.
struct CalibrationElectricalSampleV1 {
    uint32_t sample_ticks;
    uint32_t control_ticks;
    uint32_t sequence;
    uint32_t session_id;

    uint16_t main_raw;
    uint16_t aux_raw;
    uint16_t axis_state;
    uint16_t flags;

    float ia;
    float ib;
    float ic;
    float electrical_phase;
    float vd_applied;
    float vq_applied;
    float vbus;
    float duty_a;
    float duty_b;
    float duty_c;
};

static_assert(sizeof(CalibrationElectricalSampleV1) == 64,
              "CalibrationElectricalSampleV1 wire layout changed");

enum CalibrationSampleFlag : uint16_t {
    CAL_SAMPLE_CURRENT_VALID = 1u << 0,
    CAL_SAMPLE_MAIN_VALID = 1u << 1,
    CAL_SAMPLE_AUX_VALID = 1u << 2,
    CAL_SAMPLE_PWM_SATURATED = 1u << 3,
    CAL_SAMPLE_MOTOR_ARMED = 1u << 4,
    CAL_SAMPLE_RESOLVER_VALID = 1u << 5,
    CAL_SAMPLE_RESOLVER_DEGRADED = 1u << 6,
    CAL_SAMPLE_DROPPED_BEFORE = 1u << 7,
    CAL_SAMPLE_APPLIED_VOLTAGE_VALID = 1u << 8,
    CAL_SAMPLE_DUTY_VALID = 1u << 9,
    CAL_SAMPLE_VBUS_VALID = 1u << 10,
};

#endif // __CALIBRATION_RECORD_HPP
