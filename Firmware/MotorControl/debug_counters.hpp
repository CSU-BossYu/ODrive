#pragma once

#include <stdint.h>

// Global volatile debug counters for ISR-safe diagnostics.
// All counters are monotonic and read atomically by CAN diagnostics.

struct DebugCounters {
    volatile uint32_t foc_bad_timing_cnt;       // FOC BAD_TIMING occurrences

    volatile uint32_t cl_adc_fail_pre_cnt;       // ControlLoop ADC fail PRE
    volatile uint32_t cl_adc_fail_post_cnt;      // ControlLoop ADC fail POST
    volatile uint32_t cl_deadline_miss_cnt;      // ControlLoop deadline missed

    volatile uint32_t enc_pair_busy_cnt;         // SPI pair busy (start_sample_async returned false)
    volatile uint32_t enc_pair_ok_cnt;           // SPI pair completions
};

extern DebugCounters g_debug;

void init_debug_counters();
