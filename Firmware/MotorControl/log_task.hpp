#ifndef __LOG_TASK_HPP
#define __LOG_TASK_HPP

#include <cstdint>

// USB CDC log stream.
//
// A low-priority FreeRTOS task prints a CSV line of user-selected channels at
// `config.log_rate_hz` over USB CDC (via printf -> _write -> usb_cdc_stdout_sink,
// see communication.cpp). Channels are chosen by `config.log_channel_mask`
// (bitmask of LogChannel, bit i == enum value i). A `@log_hdr` line listing the
// active column names is emitted whenever the mask or rate changes; each sample
// is a `@log` line. The host filters on these prefixes.
//
// `t_ms` (a task tick counter) is always the first column of both lines and is
// not controlled by the mask.
//
// To add a channel: add an enum entry below (keep LOG_CHANNEL_COUNT last) and a
// case in get_channel() in log_task.cpp.

enum LogChannel : uint8_t {
    LOG_ID = 0,         // Id_measured [A]
    LOG_IQ,             // Iq_measured [A]
    LOG_INORM,          // sqrt(Id^2 + Iq^2) [A]  (the quantity checked vs Itrip)
    LOG_ITRIP,          // effective_current_lim + current_lim_margin [A]
    LOG_VEL,            // encoder vel_estimate [turn/s]
    LOG_IBUS,           // motor I_bus_ [A]
    LOG_VBUS,           // vbus_voltage [V]
    LOG_POS,            // encoder pos_estimate [turn]
    LOG_IQ_SETPOINT,    // Iq setpoint [A]
    LOG_MOTOR_ERR,      // motor error bits (decimal)
    LOG_AXIS_STATE,     // current axis state (decimal)
    LOG_CHANNEL_COUNT,
};

// Create and start the log task. Call once during init (before osKernelStart).
void log_task_create();

// Runtime configuration for the log task. These are NOT part of BoardConfig_t
// and are NOT persisted to flash -- they live in RAM and reset to their
// defaults on every boot. Set at runtime via the ASCII `l` command
// (see ascii_protocol.cpp cmd_log_config) or by editing the defaults below.
// `volatile` because they are written by the ASCII/USB thread and read by the
// log task.
extern volatile uint32_t log_channel_mask;
extern volatile float log_rate_hz;

#endif // __LOG_TASK_HPP
