/* Includes ------------------------------------------------------------------*/

#include "communication.h"
#include "interface_usb.h"
#include "interface_can.hpp"
#include "calibration_transport.hpp"
#include "odrive_main.h"
#include "MotorControl/axis.hpp"
#include "USB/usb_debug_transport.hpp"
#include "stm32f4xx_hal.h"

#include <algorithm>
#include <cmath>

/* Global variables ----------------------------------------------------------*/

uint64_t serial_number;
char serial_number_str[13]; // 12 digits + null termination

namespace {

bool submit_usb_command(void* context,
                        const odrive::safety::Command& command) {
    return context != nullptr &&
        static_cast<Axis*>(context)->submit_command(command);
}

void publish_usb_command_result(
        void*, const odrive::safety::CommandResult& result) {
    usb_debug_publish_command_result(&result);
}

void publish_usb_trace(void*, const odrive::trace::TraceDispatchRecord& record) {
    usb_debug_publish_trace(&record);
}

uint32_t usb_clock_ms(void*) {
    return HAL_GetTick();
}

struct UsbTestLimitBackup {
    bool valid = false;
    float velocity = 0.0f;
    float trajectory_velocity = 0.0f;
    float current = 0.0f;
    float torque = 0.0f;
};

UsbTestLimitBackup usb_test_limit_backup;

bool apply_usb_test_limits(void* context, float velocity, float current,
                           float torque) {
    if (context == nullptr || !std::isfinite(velocity) ||
        !std::isfinite(current) || !std::isfinite(torque)) return false;
    Axis& axis = *static_cast<Axis*>(context);
    if (!usb_test_limit_backup.valid) {
        usb_test_limit_backup = {
            true, axis.controller_.config_.vel_limit,
            axis.trap_traj_.config_.vel_limit,
            axis.motor_.config_.current_lim,
            axis.motor_.config_.torque_lim};
    }
    axis.controller_.config_.vel_limit =
        std::min(axis.controller_.config_.vel_limit, velocity);
    axis.trap_traj_.config_.vel_limit =
        std::min(axis.trap_traj_.config_.vel_limit, velocity);
    axis.motor_.config_.current_lim =
        std::min(axis.motor_.config_.current_lim, current);
    axis.motor_.config_.torque_lim =
        std::min(axis.motor_.config_.torque_lim, torque);
    return true;
}

void restore_usb_test_limits(void* context) {
    if (context == nullptr || !usb_test_limit_backup.valid) return;
    Axis& axis = *static_cast<Axis*>(context);
    axis.controller_.config_.vel_limit = usb_test_limit_backup.velocity;
    axis.trap_traj_.config_.vel_limit =
        usb_test_limit_backup.trajectory_velocity;
    axis.motor_.config_.current_lim = usb_test_limit_backup.current;
    axis.motor_.config_.torque_lim = usb_test_limit_backup.torque;
    usb_test_limit_backup = {};
}

}  // namespace

void init_communication(void) {
    Axis& axis = odrv.get_axis(0);
    usb_debug_bind_command_sink(&axis, submit_usb_command);
    usb_debug_bind_clock(nullptr, usb_clock_ms);
    usb_debug_bind_test_limits(&axis, apply_usb_test_limits,
                               restore_usb_test_limits);
    usb_debug_bind_scope_engine(&axis.scope_capture());
    (void)axis.add_command_result_sink(nullptr, publish_usb_command_result);
    axis.set_trace_dispatch_sink(nullptr, publish_usb_trace);
    start_usb_server();
    start_calibration_transport();

    if (odrv.config_.enable_can_a) {
        odrv.can_.start_server(&hcan1);
    }
}

extern "C" {
int _write(int file, const char* data, int len) __attribute__((used));
}

// @brief This is what printf calls internally
int _write(int file, const char* data, int len) {
    // USB CDC is a production stdout-only transport with no command parser.
    usb_stdout_write(reinterpret_cast<const uint8_t*>(data), static_cast<size_t>(len));
    return len; // Always pretend that we processed everything
}
