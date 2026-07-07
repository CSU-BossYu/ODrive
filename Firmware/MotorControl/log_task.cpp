#include "log_task.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>

#include "odrive_main.h"
#include <cmsis_os.h>

// Runtime log configuration (RAM only, not persisted). See log_task.hpp.
volatile uint32_t log_channel_mask = 0x3f; // Id/Iq/Inorm/Itrip/vel/ibus
volatile float log_rate_hz = 100.0f;

namespace {

const char* const kChannelNames[LOG_CHANNEL_COUNT] = {
    "Id",         // LOG_ID
    "Iq",         // LOG_IQ
    "Inorm",      // LOG_INORM
    "Itrip",      // LOG_ITRIP
    "vel",        // LOG_VEL
    "ibus",       // LOG_IBUS
    "vbus",       // LOG_VBUS
    "pos",        // LOG_POS
    "Iq_sp",      // LOG_IQ_SETPOINT
    "motor_err",  // LOG_MOTOR_ERR
    "axis_state", // LOG_AXIS_STATE
};

// Returns the current value of a channel as a float. Integer-valued channels
// (motor_err, axis_state) are cast through their integer type so small values
// stay exact; high error bits may lose precision but the low bits relevant to
// CURRENT_LIMIT_VIOLATION (bit 12) are preserved.
float get_channel(LogChannel ch) {
    Axis& axis = odrv.get_axis(0);
    Motor& motor = axis.motor_;
    switch (ch) {
        case LOG_ID:
            return motor.current_control_.Id_measured_;
        case LOG_IQ:
            return motor.current_control_.Iq_measured_;
        case LOG_INORM: {
            float id = motor.current_control_.Id_measured_;
            float iq = motor.current_control_.Iq_measured_;
            return sqrtf(id * id + iq * iq);
        }
        case LOG_ITRIP:
            return motor.effective_current_lim_ + motor.config_.current_lim_margin;
        case LOG_VEL:
            return axis.encoder_.vel_estimate_.present().value_or(0.0f);
        case LOG_IBUS:
            return motor.I_bus_;
        case LOG_VBUS:
            return odrv.vbus_voltage_;
        case LOG_POS:
            return axis.encoder_.pos_estimate_.present().value_or(0.0f);
        case LOG_IQ_SETPOINT:
            return motor.current_control_.Idq_setpoint_.value_or(float2D{0.0f, 0.0f}).second;
        case LOG_MOTOR_ERR:
            return static_cast<float>(static_cast<uint64_t>(motor.error_));
        case LOG_AXIS_STATE:
            return static_cast<float>(static_cast<uint32_t>(axis.current_state_));
        default:
            return 0.0f;
    }
}

void emit_header(uint32_t mask) {
    // t_ms is always the first column.
    printf("@log_hdr,t_ms");
    for (uint8_t i = 0; i < LOG_CHANNEL_COUNT; ++i) {
        if (mask & (1u << i)) {
            printf(",%s", kChannelNames[i]);
        }
    }
    printf("\n");
}

void log_task(void*) {
    // Force a header on the first valid iteration.
    uint32_t prev_mask = 0xFFFFFFFFu;
    float prev_rate = -1.0f;
    uint32_t t_ms = 0;

    for (;;) {
        uint32_t mask = log_channel_mask;
        float rate = log_rate_hz;

        if (mask == 0 || !(rate > 0.0f) || !std::isfinite(rate)) {
            osDelay(100);
            continue;
        }

        uint32_t period_ms = (uint32_t)(1000.0f / rate);
        if (period_ms < 1) period_ms = 1;

        if (mask != prev_mask || rate != prev_rate) {
            emit_header(mask);
            prev_mask = mask;
            prev_rate = rate;
        }

        // Build the whole line in one buffer so it goes out as a single _write
        // (no interleaving with other stdout).
        char buf[192];
        int n = snprintf(buf, sizeof(buf), "@log,%lu", (unsigned long)t_ms);
        if (n < 0) n = 0;
        for (uint8_t i = 0; i < LOG_CHANNEL_COUNT && (size_t)n + 16 < sizeof(buf); ++i) {
            if (mask & (1u << i)) {
                int m = snprintf(buf + n, sizeof(buf) - n, ",%.5g",
                                 (double)get_channel((LogChannel)i));
                if (m < 0) break;
                n += m;
            }
        }
        printf("%s\n", buf);

        t_ms += period_ms;
        osDelay(period_ms);
    }
}

// Stack size in bytes. snprintf with %.5g (float formatting, _printf_float) is
// stack-hungry -- match the USB thread (4096) which does the same kind of
// printf work. 2048 overflowed and hard-faulted at boot.
const uint32_t stack_size_log_thread = 4096;

} // namespace

void log_task_create() {
    osThreadDef(log_task_def, log_task, osPriorityLow, 0,
                stack_size_log_thread / sizeof(StackType_t));
    osThreadCreate(osThread(log_task_def), NULL);
}
