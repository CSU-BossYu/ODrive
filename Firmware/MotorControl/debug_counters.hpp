#pragma once

#include <stdint.h>

// Global volatile debug counters for ISR-safe diagnostics.
// All counters are monotonic and read atomically by CAN diagnostics.

struct DebugCounters {
    volatile uint32_t foc_bad_timing_cnt;       // FOC BAD_TIMING occurrences
    volatile uint32_t foc_bad_timing_delta;      // last delta value
    volatile uint32_t foc_bad_timing_i_ts;       // last i_timestamp_
    volatile uint32_t foc_bad_timing_ctrl_ts;    // last ctrl_timestamp_

    volatile uint32_t cl_adc_fail_pre_cnt;       // ControlLoop ADC fail PRE
    volatile uint32_t cl_adc_fail_post_cnt;      // ControlLoop ADC fail POST
    volatile uint32_t cl_deadline_miss_cnt;      // ControlLoop deadline missed

    volatile uint32_t enc_pair_busy_cnt;         // SPI pair busy (start_sample_async returned false)
    volatile uint32_t enc_pair_ok_cnt;           // SPI pair completions
    volatile uint32_t enc_pair_ok_main;          // last main angle from pair_ok
    volatile uint32_t enc_pair_ok_aux;           // last aux angle from pair_ok

    volatile uint32_t loop_alive_cnt;            // sampling_cb invocation count

    // Per-ADC / per-flag breakdown of fetch_and_reset_adcs() failures
    volatile uint32_t adc1_jeoc_fail;            // ADC1 JEOC (VBUS injected) not ready
    volatile uint32_t adc2_eoc_fail;             // ADC2 EOC (M1 PhB regular) not ready
    volatile uint32_t adc2_jeoc_fail;            // ADC2 JEOC (M0 PhB injected) not ready
    volatile uint32_t adc3_eoc_fail;             // ADC3 EOC (M1 PhC regular) not ready
    volatile uint32_t adc3_jeoc_fail;            // ADC3 JEOC (M0 PhC injected) not ready

    // Last M0 current-sense sample and motor calibration internals
    volatile uint32_t m0_adc1_jdr;
    volatile uint32_t m0_adc2_jdr;
    volatile uint32_t m0_adc3_jdr;
    volatile uint32_t m0_current_sample_valid;
    volatile float m0_current_phA;
    volatile float m0_current_phB;
    volatile float m0_current_phC;
    volatile uint32_t tim1_bdtr;
    volatile uint32_t tim1_ccr1;
    volatile uint32_t tim1_ccr2;
    volatile uint32_t tim1_ccr3;
    volatile uint32_t m0_is_armed;
    volatile float resistance_actual_current;
    volatile float resistance_test_voltage;
    volatile float resistance_i_beta;
    volatile float resistance_test_mod;

    // Last Motor::current_meas_cb()/dc_calib_cb() decision state for M0.
    volatile uint32_t m0_cm_current_present;
    volatile uint32_t m0_cm_dc_calib_valid;
    volatile uint32_t m0_cm_current_meas_valid;
    volatile uint32_t m0_cm_armed_state;
    volatile float m0_dc_calib_running_since;
    volatile float m0_dc_calib_phA;
    volatile float m0_dc_calib_phB;
    volatile float m0_dc_calib_phC;
    volatile float m0_cm_phA;
    volatile float m0_cm_phB;
    volatile float m0_cm_phC;

    // M0 diagnostic phase-voltage scan state.
    volatile uint32_t m0_phase_scan_active;
    volatile uint32_t m0_phase_scan_vector;
    volatile float m0_phase_scan_v_alpha;
    volatile float m0_phase_scan_v_beta;
    volatile float m0_phase_scan_i_alpha;
    volatile float m0_phase_scan_i_beta;
    volatile uint32_t m0_phase_scan_missing_current;
    volatile uint32_t m0_phase_scan_sample_count;
    volatile float m0_phase_scan_avg_phA;
    volatile float m0_phase_scan_avg_phB;
    volatile float m0_phase_scan_avg_phC;
    volatile float m0_phase_scan_avg_i_alpha;
    volatile float m0_phase_scan_avg_i_beta;
};

extern DebugCounters g_debug;

void init_debug_counters();
