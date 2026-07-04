#ifndef __CONTROL_TIMEOUT_HPP
#define __CONTROL_TIMEOUT_HPP

#include "controller.hpp"

class Axis;

namespace ControlTimeout {

static constexpr uint32_t FLAG_ENABLED              = 1u << 0;
static constexpr uint32_t FLAG_RUNNING              = 1u << 1;
static constexpr uint32_t FLAG_HOLDING              = 1u << 2;
static constexpr uint32_t FLAG_QUICK_STOP_ACTIVE    = 1u << 3;
static constexpr uint32_t FLAG_COMM_TIMEOUT         = 1u << 4;
static constexpr uint32_t FLAG_CMD_WATCHDOG_EXPIRED = 1u << 5;
static constexpr uint32_t FLAG_MIT_FRAME_STALE      = 1u << 6;
static constexpr uint32_t FLAG_TRAJECTORY_DONE      = 1u << 7;
static constexpr uint32_t FLAG_HEARTBEAT_EXPIRED    = 1u << 8;

struct Config {
    float velocity_accel_limit;
    float velocity_decel_limit;
    float quick_stop_decel_limit;
    uint32_t can_watchdog_timeout_ms;
    Controller::TimeoutAction timeout_action;
    uint32_t heartbeat_timeout_ms;
};

struct State {
    uint32_t command_deadline_ms = 0;
    uint32_t heartbeat_deadline_ms = 0;
    uint32_t flags = 0;
    uint32_t last_timeout_reason = 0;
    bool command_watchdog_expired = false;
    bool quick_stop_active = false;
    bool torque_zero_active = false;
};

Config& config(Axis& axis);
const Config& config(const Axis& axis);
State& state(Axis& axis);
const State& state(const Axis& axis);

void feed_command(Axis& axis);
void feed_heartbeat(Axis& axis);
bool check_command(Axis& axis);
bool check_heartbeat(Axis& axis);
void apply_timeout_action(Axis& axis, uint32_t reason);
void clear(Axis& axis);

Controller::TimeoutAction effective_timeout_action(const Axis& axis);
Controller::ServoControlMode derive_servo_mode(const Axis& axis);

void mark_running(Axis& axis);
void clear_running(Axis& axis);
void mark_trajectory_active(Axis& axis);
void mark_trajectory_done(Axis& axis);
void mark_holding(Axis& axis);
bool quick_stop_active(const Axis& axis);

} // namespace ControlTimeout

#endif // __CONTROL_TIMEOUT_HPP
