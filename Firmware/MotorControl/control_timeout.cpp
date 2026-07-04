#include <odrive_main.h>
#include "control_timeout.hpp"
#include "axis.hpp"
#include <algorithm>

namespace ControlTimeout {

static Config configs[AXIS_COUNT];
static State states[AXIS_COUNT];
static bool configs_initialized[AXIS_COUNT];

static size_t axis_index(const Axis& axis) {
    return static_cast<size_t>(std::clamp(axis.axis_num_, 0, AXIS_COUNT - 1));
}

static void ensure_config_initialized(size_t index) {
    if (configs_initialized[index]) {
        return;
    }
    configs[index].velocity_accel_limit = 0.5f;
    configs[index].velocity_decel_limit = 0.5f;
    configs[index].quick_stop_decel_limit = 1.0f;
    configs[index].can_watchdog_timeout_ms = 0;
    configs[index].timeout_action = Controller::TIMEOUT_ACTION_QUICK_STOP_AND_HOLD;
    configs[index].heartbeat_timeout_ms = 0;
    configs_initialized[index] = true;
}

Config& config(Axis& axis) {
    size_t index = axis_index(axis);
    ensure_config_initialized(index);
    return configs[index];
}

const Config& config(const Axis& axis) {
    size_t index = axis_index(axis);
    ensure_config_initialized(index);
    return configs[index];
}

State& state(Axis& axis) {
    return states[axis_index(axis)];
}

const State& state(const Axis& axis) {
    return states[axis_index(axis)];
}

Controller::TimeoutAction effective_timeout_action(const Axis& axis) {
    const Config& cfg = config(axis);
    if (cfg.timeout_action != Controller::TIMEOUT_ACTION_QUICK_STOP_AND_HOLD) {
        return cfg.timeout_action;
    }
    const Controller& controller = axis.controller_;
    if (controller.config_.input_mode == Controller::INPUT_MODE_MIT) {
        return Controller::TIMEOUT_ACTION_TORQUE_ZERO;
    }
    if (controller.config_.control_mode == Controller::CONTROL_MODE_TORQUE_CONTROL) {
        return Controller::TIMEOUT_ACTION_TORQUE_ZERO;
    }
    if (controller.config_.control_mode >= Controller::CONTROL_MODE_POSITION_CONTROL) {
        return Controller::TIMEOUT_ACTION_HOLD_LAST_POSITION;
    }
    return Controller::TIMEOUT_ACTION_QUICK_STOP_AND_HOLD;
}

Controller::ServoControlMode derive_servo_mode(const Axis& axis) {
    const Controller& controller = axis.controller_;
    if (controller.config_.control_mode == Controller::CONTROL_MODE_TORQUE_CONTROL &&
        controller.config_.input_mode == Controller::INPUT_MODE_MIT) {
        return Controller::SERVO_CONTROL_MODE_MIT_REALTIME;
    }
    if (controller.config_.control_mode == Controller::CONTROL_MODE_TORQUE_CONTROL &&
        controller.config_.input_mode == Controller::INPUT_MODE_PASSTHROUGH) {
        return Controller::SERVO_CONTROL_MODE_TORQUE;
    }
    if (controller.config_.control_mode == Controller::CONTROL_MODE_VELOCITY_CONTROL &&
        controller.config_.input_mode == Controller::INPUT_MODE_VEL_RAMP) {
        return Controller::SERVO_CONTROL_MODE_VELOCITY;
    }
    if (controller.config_.control_mode == Controller::CONTROL_MODE_POSITION_CONTROL &&
        controller.config_.input_mode == Controller::INPUT_MODE_TRAP_TRAJ) {
        return Controller::SERVO_CONTROL_MODE_PROFILE_POSITION;
    }
    return static_cast<Controller::ServoControlMode>(0xFFu);
}

void clear(Axis& axis) {
    State& st = state(axis);
    st.command_watchdog_expired = false;
    st.quick_stop_active = false;
    st.torque_zero_active = false;
    st.last_timeout_reason = 0;
    st.flags &= ~(FLAG_COMM_TIMEOUT | FLAG_CMD_WATCHDOG_EXPIRED |
                  FLAG_QUICK_STOP_ACTIVE | FLAG_MIT_FRAME_STALE |
                  FLAG_HEARTBEAT_EXPIRED);
}

void feed_command(Axis& axis) {
    State& st = state(axis);
    const uint32_t timeout_ms = config(axis).can_watchdog_timeout_ms;
    if (timeout_ms > 0) {
        st.command_deadline_ms = HAL_GetTick() + timeout_ms;
        if (st.command_watchdog_expired || st.quick_stop_active || st.torque_zero_active) {
            clear(axis);
        }
    } else {
        st.command_deadline_ms = 0;
    }
}

void feed_heartbeat(Axis& axis) {
    State& st = state(axis);
    const uint32_t timeout_ms = config(axis).heartbeat_timeout_ms;
    st.heartbeat_deadline_ms = (timeout_ms > 0) ? (HAL_GetTick() + timeout_ms) : 0;
}

static bool deadline_expired(uint32_t deadline_ms) {
    uint32_t now = HAL_GetTick();
    return (now - deadline_ms) < 0x80000000u;
}

bool check_command(Axis& axis) {
    State& st = state(axis);
    if (st.command_deadline_ms == 0 || st.command_watchdog_expired) {
        return true;
    }
    if (deadline_expired(st.command_deadline_ms)) {
        apply_timeout_action(axis, 1);
    }
    return true;
}

bool check_heartbeat(Axis& axis) {
    State& st = state(axis);
    if (st.heartbeat_deadline_ms == 0 || st.command_watchdog_expired) {
        return true;
    }
    if (deadline_expired(st.heartbeat_deadline_ms)) {
        apply_timeout_action(axis, 2);
    }
    return true;
}

void apply_timeout_action(Axis& axis, uint32_t reason) {
    Controller& controller = axis.controller_;
    State& st = state(axis);

    st.command_watchdog_expired = true;
    st.last_timeout_reason = reason;
    st.flags |= FLAG_COMM_TIMEOUT;
    st.flags |= (reason == 2) ? FLAG_HEARTBEAT_EXPIRED : FLAG_CMD_WATCHDOG_EXPIRED;

    if (controller.config_.input_mode == Controller::INPUT_MODE_MIT) {
        controller.mit_kp_ = 0.0f;
        controller.mit_kd_ = 0.0f;
        controller.mit_torque_ff_ = 0.0f;
        st.flags |= FLAG_MIT_FRAME_STALE;
    }

    switch (effective_timeout_action(axis)) {
        case Controller::TIMEOUT_ACTION_HOLD_LAST_POSITION:
            controller.input_pos_ = controller.pos_setpoint_;
            controller.input_pos_updated_ = false;
            st.flags |= FLAG_HOLDING;
            break;

        case Controller::TIMEOUT_ACTION_QUICK_STOP:
        case Controller::TIMEOUT_ACTION_QUICK_STOP_AND_HOLD:
            controller.input_vel_ = 0.0f;
            st.quick_stop_active = true;
            st.flags |= FLAG_QUICK_STOP_ACTIVE;
            break;

        case Controller::TIMEOUT_ACTION_TORQUE_ZERO:
            controller.input_torque_ = 0.0f;
            controller.torque_setpoint_ = 0.0f;
            st.torque_zero_active = true;
            break;

        case Controller::TIMEOUT_ACTION_FAULT_DISABLE:
            axis.error_ |= Axis::ERROR_CONTROLLER_FAILED;
            break;
    }
}

void mark_running(Axis& axis) {
    state(axis).flags |= FLAG_ENABLED | FLAG_RUNNING;
}

void clear_running(Axis& axis) {
    state(axis).flags &= ~(FLAG_ENABLED | FLAG_RUNNING);
}

void mark_trajectory_active(Axis& axis) {
    state(axis).flags &= ~(FLAG_TRAJECTORY_DONE | FLAG_HOLDING);
}

void mark_trajectory_done(Axis& axis) {
    state(axis).flags |= FLAG_TRAJECTORY_DONE | FLAG_HOLDING;
}

void mark_holding(Axis& axis) {
    state(axis).flags |= FLAG_HOLDING;
}

bool quick_stop_active(const Axis& axis) {
    return state(axis).quick_stop_active;
}

} // namespace ControlTimeout
