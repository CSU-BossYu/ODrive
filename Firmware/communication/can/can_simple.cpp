
#include "can_simple.hpp"

#include "debug_counters.hpp"
#include <odrive_main.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

bool CANSimple::init() {
    return renew_subscription(0);
}

bool CANSimple::renew_subscription(size_t i) {
    Axis& axis = axes[i];

    // TODO: remove these two lines (see comment in header)
    node_ids_[i] = axis.config_.can.node_id;
    extended_node_ids_[i] = axis.config_.can.is_extended;

    MsgIdFilterSpecs filter = {
        .id = {},
        .mask = (uint32_t)(0xffffffff << NUM_CMD_ID_BITS)};
    if (axis.config_.can.is_extended) {
        filter.id = (uint32_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    } else {
        filter.id = (uint16_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    }

    if (subscription_handles_[i]) {
        canbus_->unsubscribe(subscription_handles_[i]);
    }

    return canbus_->subscribe(
        filter, [](void* ctx, const can_Message_t& msg) {
            ((CANSimple*)ctx)->handle_can_message(msg);
        },
        this, &subscription_handles_[i]);
}

void CANSimple::handle_can_message(const can_Message_t& msg) {
    //     Frame
    // nodeID | CMD
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(msg.id);

    Axis& axis = axes[0];
    if ((axis.config_.can.node_id == nodeID) && (axis.config_.can.is_extended == msg.isExt)) {
        do_command(axis, msg);
    }
}

void CANSimple::do_command(Axis& axis, const can_Message_t& msg) {
    const uint32_t cmd = get_cmd_id(msg.id);
    axis.watchdog_feed();
    switch (cmd) {
        case MSG_CO_NMT_CTRL:
            break;
        case MSG_CO_HEARTBEAT_CMD:
            break;
        case MSG_ODRIVE_HEARTBEAT:
            // We don't currently do anything to respond to ODrive heartbeat messages
            break;
        case MSG_ODRIVE_ESTOP:
            estop_callback(axis, msg);
            break;
        case MSG_GET_MOTOR_ERROR:
            if (msg.rtr || msg.len == 0)
                get_motor_error_callback(axis);
            break;
        case MSG_GET_ENCODER_ERROR:
            if (msg.rtr || msg.len == 0)
                get_encoder_error_callback(axis);
            break;
        case MSG_SET_AXIS_NODE_ID:
            set_axis_nodeid_callback(axis, msg);
            break;
        case MSG_SET_AXIS_REQUESTED_STATE:
            set_axis_requested_state_callback(axis, msg);
            break;
        case MSG_SET_AXIS_STARTUP_CONFIG:
            set_axis_startup_config_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ESTIMATES:
            if (msg.rtr || msg.len == 0)
                get_encoder_estimates_callback(axis);
            break;
        case MSG_GET_ENCODER_COUNT:
            if (msg.rtr || msg.len == 0)
                get_encoder_count_callback(axis);
            break;
        case MSG_SET_INPUT_POS:
            set_input_pos_callback(axis, msg);
            break;
        case MSG_SET_INPUT_VEL:
            set_input_vel_callback(axis, msg);
            break;
        case MSG_SET_INPUT_TORQUE:
            set_input_torque_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_MODES:
            set_controller_modes_callback(axis, msg);
            break;
        case MSG_SET_LIMITS:
            set_limits_callback(axis, msg);
            break;
        case MSG_START_ANTICOGGING:
            start_anticogging_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_INERTIA:
            set_traj_inertia_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_ACCEL_LIMITS:
            set_traj_accel_limits_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_VEL_LIMIT:
            set_traj_vel_limit_callback(axis, msg);
            break;
        case MSG_GET_IQ:
            if (msg.rtr || msg.len == 0)
                get_iq_callback(axis);
            break;
        case MSG_RESET_ODRIVE:
            NVIC_SystemReset();
            break;
        case MSG_GET_BUS_VOLTAGE_CURRENT:
            if (msg.rtr || msg.len == 0)
                get_bus_voltage_current_callback(axis);
            break;
        case MSG_CLEAR_ERRORS:
            clear_errors_callback(axis, msg);
            break;
        case MSG_SET_LINEAR_COUNT:
            set_linear_count_callback(axis, msg);
            break;
        case MSG_SET_POS_GAIN:
            set_pos_gain_callback(axis, msg);
            break;
        case MSG_SET_VEL_GAINS:
            set_vel_gains_callback(axis, msg);
            break;
        case MSG_GET_ADC_VOLTAGE:
            get_adc_voltage_callback(axis, msg);
            break;
        case MSG_GET_CONTROLLER_ERROR:
            get_controller_error_callback(axis);
            break;
        case MSG_EXTENDED_COMMAND:
            extended_command_callback(axis, msg);
            break;
        case MSG_SET_MIT_CONTROL:
            set_mit_control_callback(axis, msg);
            break;
        default:
            break;
    }
}

void CANSimple::nmt_callback(const Axis& axis, const can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis& axis, const can_Message_t& msg) {
    axis.error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

bool CANSimple::get_motor_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_MOTOR_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.motor_.error_, 0, 64, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.encoder_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_controller_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_CONTROLLER_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.controller_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

void CANSimple::set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg) {
    axis.config_.can.node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimple::set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg) {
    axis.requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg) {
    // Not Implemented
}

bool CANSimple::get_encoder_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    auto pos_estimate = axis.controller_.pos_estimate_linear_src_.any();
    if (!pos_estimate.has_value()) {
        pos_estimate = axis.encoder_.pos_estimate_.any();
    }

    auto vel_estimate = axis.controller_.vel_estimate_src_.any();
    if (!vel_estimate.has_value()) {
        vel_estimate = axis.encoder_.vel_estimate_.any();
    }

    can_setSignal<float>(txmsg, pos_estimate.value_or(0.0f), 0, 32, true);
    can_setSignal<float>(txmsg, vel_estimate.value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_count_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_COUNT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<int32_t>(txmsg, axis.encoder_.shadow_count_, 0, 32, true);
    can_setSignal<int32_t>(txmsg, axis.encoder_.count_in_cpr_, 32, 32, true);
    return canbus_->send_message(txmsg);
}

void CANSimple::set_input_pos_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.set_input_pos_and_steps(can_getSignal<float>(msg, 0, 32, true));
    axis.controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis.controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis.controller_.input_pos_updated();
}

void CANSimple::set_input_vel_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_torque_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_controller_modes_callback(Axis& axis, const can_Message_t& msg) {
    Controller::ControlMode const mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
    axis.controller_.config_.control_mode = static_cast<Controller::ControlMode>(mode);
    axis.controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
    axis.controller_.control_mode_updated();
}

void CANSimple::set_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::start_anticogging_callback(const Axis& axis, const can_Message_t& msg) {
    axis.controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_linear_count_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.set_linear_count(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_pos_gain_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_vel_gains_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

// =====================================================================
// MIT-style packed control frame (CMD 0x01F)
//
// 8-byte layout, big-endian bit packing (AK / T-Motor compatible):
//   p_des : 16 bit   (buf[0..1])
//   v_des : 12 bit   (buf[2] : buf[3] hi-nibble)
//   kp    : 12 bit   (buf[3] lo-nibble : buf[4])
//   kd    : 12 bit   (buf[5] : buf[6] hi-nibble)
//   t_ff  : 12 bit   (buf[6] lo-nibble : buf[7])
//
// Units: p_des [rad], v_des [rad/s], kp [Nm/rad], kd [Nm/(rad/s)], t_ff [Nm].
// =====================================================================
static constexpr float MIT_P_MIN  = -12.5f;   // [rad]
static constexpr float MIT_P_MAX  =  12.5f;
static constexpr float MIT_V_MIN  = -45.0f;   // [rad/s]
static constexpr float MIT_V_MAX  =  45.0f;
static constexpr float MIT_KP_MIN = 0.0f;     // [Nm/rad]
static constexpr float MIT_KP_MAX = 500.0f;
static constexpr float MIT_KD_MIN = 0.0f;     // [Nm/(rad/s)]
static constexpr float MIT_KD_MAX = 5.0f;
static constexpr float MIT_T_MIN  = -18.0f;   // [Nm]
static constexpr float MIT_T_MAX  =  18.0f;

// Inverse of the standard AK/T-Motor quantiser: maps a raw unsigned integer
// in [0, 2^bits - 1] back to a float in [min, max].
static inline float mit_uint_to_float(uint32_t raw, float min, float max, uint32_t bits) {
    float span = max - min;
    float scale = span / static_cast<float>((1u << bits) - 1u);
    return min + scale * static_cast<float>(raw);
}

void CANSimple::set_mit_control_callback(Axis& axis, const can_Message_t& msg) {
    // MIT frames are exactly 8 bytes; ignore malformed frames silently.
    if (msg.len != 8) {
        return;
    }

    const uint8_t* b = msg.buf;
    uint16_t p_int  = static_cast<uint16_t>((b[0] << 8) | b[1]);
    uint16_t v_int  = static_cast<uint16_t>((b[2] << 4) | (b[3] >> 4));
    uint16_t kp_int = static_cast<uint16_t>(((b[3] & 0x0F) << 8) | b[4]);
    uint16_t kd_int = static_cast<uint16_t>((b[5] << 4) | (b[6] >> 4));
    uint16_t t_int  = static_cast<uint16_t>(((b[6] & 0x0F) << 8) | b[7]);

    float p_des = mit_uint_to_float(p_int,  MIT_P_MIN,  MIT_P_MAX,  16);
    float v_des = mit_uint_to_float(v_int,  MIT_V_MIN,  MIT_V_MAX,  12);
    float kp    = mit_uint_to_float(kp_int, MIT_KP_MIN, MIT_KP_MAX, 12);
    float kd    = mit_uint_to_float(kd_int, MIT_KD_MIN, MIT_KD_MAX, 12);
    float t_ff  = mit_uint_to_float(t_int,  MIT_T_MIN,  MIT_T_MAX,  12);

    // Hand the decoded values to the controller. The controller only acts on
    // them when input_mode == INPUT_MODE_MIT; otherwise this just updates the
    // stored input and does not drive the motor.
    axis.controller_.set_mit_input(p_des, v_des, kp, kd, t_ff);
}

bool CANSimple::get_iq_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_IQ;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    std::optional<float2D> Idq_setpoint = axis.motor_.current_control_.Idq_setpoint_;
    if (!Idq_setpoint.has_value()) {
        Idq_setpoint = {0.0f, 0.0f};
    }
    
    static_assert(sizeof(float) == sizeof(Idq_setpoint->second));
    static_assert(sizeof(float) == sizeof(axis.motor_.current_control_.Iq_measured_));
    can_setSignal<float>(txmsg, Idq_setpoint->second, 0, 32, true);
    can_setSignal<float>(txmsg, axis.motor_.current_control_.Iq_measured_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_bus_voltage_current_callback(const Axis& axis) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_BUS_VOLTAGE_CURRENT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    static_assert(sizeof(float) == sizeof(vbus_voltage));
    static_assert(sizeof(float) == sizeof(ibus_));
    can_setSignal<float>(txmsg, vbus_voltage, 0, 32, true);
    can_setSignal<float>(txmsg, ibus_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_adc_voltage_callback(const Axis& axis, const can_Message_t& msg) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ADC_VOLTAGE;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    auto gpio_num = can_getSignal<uint8_t>(msg, 0, 8, true);
    if (gpio_num < GPIO_COUNT) {
        auto voltage = get_adc_voltage(get_gpio(gpio_num));
        can_setSignal<float>(txmsg, voltage, 0, 32, true);
        return canbus_->send_message(txmsg);
    } else {
        return false;
    }
}

void CANSimple::clear_errors_callback(Axis& axis, const can_Message_t& msg) {
    odrv.clear_errors();  // TODO: might want to clear axis errors only
}

uint32_t CANSimple::service_stack() {
    uint32_t nextServiceTime = UINT32_MAX;
    uint32_t now = HAL_GetTick();

    // TODO: remove this polling loop and replace with protocol hook
    bool node_id_changed = (axes[0].config_.can.node_id != node_ids_[0])
        || (axes[0].config_.can.is_extended != extended_node_ids_[0]);
    if (node_id_changed) {
        renew_subscription(0);
    }

    struct periodic {
        const uint32_t& rate;
        uint32_t& last_time;
        bool (CANSimple::* callback)(const Axis& axis);
    };

    Axis& axis = axes[0];
    std::array<periodic, 8> periodics = {{
            {axis.config_.can.heartbeat_rate_ms, axis.can_.last_heartbeat, &CANSimple::send_heartbeat},
            {axis.config_.can.encoder_rate_ms, axis.can_.last_encoder, &CANSimple::get_encoder_estimates_callback},
            {axis.config_.can.motor_error_rate_ms, axis.can_.last_motor_error, &CANSimple::get_motor_error_callback},
            {axis.config_.can.encoder_error_rate_ms, axis.can_.last_encoder_error, &CANSimple::get_encoder_error_callback},
            {axis.config_.can.controller_error_rate_ms, axis.can_.last_controller_error, &CANSimple::get_controller_error_callback},
            {axis.config_.can.encoder_count_rate_ms, axis.can_.last_encoder_count, &CANSimple::get_encoder_count_callback},
            {axis.config_.can.iq_rate_ms, axis.can_.last_iq, &CANSimple::get_iq_callback},
            {axis.config_.can.bus_vi_rate_ms, axis.can_.last_bus_vi, &CANSimple::get_bus_voltage_current_callback},
    }};

    MEASURE_TIME(axis.task_times_.can_heartbeat) {
        for (auto& msg : periodics) {
            if (msg.rate > 0) {
                if ((now - msg.last_time) >= msg.rate) {
                    if (std::invoke(msg.callback, this, axis)) {
                        msg.last_time = now;
                    }
                }

                int nextAxisService = msg.last_time + msg.rate - now;
                nextServiceTime = std::min(nextServiceTime, static_cast<uint32_t>(std::max(0, nextAxisService)));
            }
        }
    }

    return nextServiceTime;
}

bool CANSimple::send_heartbeat(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.error_, 0, 32, true);
    can_setSignal(txmsg, uint8_t(axis.current_state_), 32, 8, true);

    // Motor flags
    uint8_t motorFlags = axis.motor_.error_ != 0;

    // Encoder flags
    uint8_t encoderFlags = axis.encoder_.error_ != 0;

    // Controller flags
    uint8_t controllerFlags =axis.controller_.error_ != 0;
    uint8_t trajDone = uint8_t(axis.controller_.trajectory_done_) << 7;
    controllerFlags |= trajDone;

    can_setSignal(txmsg, motorFlags, 40, 8, true);
    can_setSignal(txmsg, encoderFlags, 48, 8, true);
    can_setSignal(txmsg, controllerFlags, 56, 8, true);

    return canbus_->send_message(txmsg);
}

// =====================================================================
// Extended command (CMD 0x1E) - protocol version 1.0
//
// Request:  byte0=sub_cmd  byte1=param    byte2-3=reserved  byte4-7=value
// Response: byte0=sub_cmd  byte1=param    byte2=status      byte3=type/aux byte4-7=value
// Note: GET_AXIS_STATUS_EX uses byte1=status, byte2=current_state, byte3=flags.
// =====================================================================

// ---- file-scope constants -------------------------------------------
static constexpr uint8_t  EXT_TYPE_FLOAT32       = 1;
static constexpr uint8_t  EXT_TYPE_INT32         = 2;
static constexpr uint8_t  EXT_TYPE_UINT32        = 3;

static constexpr uint8_t  EXT_STATUS_OK          = 0;
static constexpr uint8_t  EXT_STATUS_UNKNOWN     = 1;
static constexpr uint8_t  EXT_STATUS_READONLY    = 2;
static constexpr uint8_t  EXT_STATUS_INVALID_TYPE = 3;
static constexpr uint8_t  EXT_STATUS_INVALID_VALUE = 4;
static constexpr uint8_t  EXT_STATUS_BUSY_ARMED  = 5;

static constexpr uint32_t EXT_PROTOCOL_VERSION   = 0x00000101;  // v1.1

// ---- utility --------------------------------------------------------
static bool any_axis_armed() {
    return std::any_of(axes.begin(), axes.end(),
        [](auto& a) { return a.motor_.is_armed_; });
}

static bool is_valid_motor_type(uint32_t value) {
    return value == Motor::MOTOR_TYPE_HIGH_CURRENT
        || value == Motor::MOTOR_TYPE_GIMBAL
        || value == Motor::MOTOR_TYPE_ACIM;
}

static bool is_valid_encoder_mode(uint32_t value) {
    switch (value) {
        case Encoder::MODE_SPI_ABS_MT6826S:
        case Encoder::MODE_SPI_ABS_MT6826S_VERNIER:
            return true;
        default:
            return false;
    }
}

static bool is_positive_finite(float value) {
    return std::isfinite(value) && value > 0.0f;
}

static bool is_nonnegative_finite(float value) {
    return std::isfinite(value) && value >= 0.0f;
}

static bool is_valid_spi_prescaler_divisor(uint32_t value) {
    return value == 2 || value == 4 || value == 8 || value == 16
        || value == 32 || value == 64 || value == 128 || value == 256;
}

// ---- dispatcher -----------------------------------------------------
bool CANSimple::extended_command_callback(Axis& axis, const can_Message_t& msg) {
    const uint8_t sub_cmd = msg.buf[0];
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_EXTENDED_COMMAND;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;
    memset(txmsg.buf, 0, 8);

    switch (sub_cmd) {
        case 0x01: return handle_get_axis_status_ex(axis, txmsg);
        case 0x02: return handle_set_precalibrated(axis, msg, txmsg);
        case 0x03: return handle_save_configuration(txmsg);
        case 0x04: return handle_get_calib_result(axis, msg, txmsg);
        case 0x05: return handle_get_device_info(msg, txmsg);
        case 0x06: return handle_get_basic_config(axis, msg, txmsg);
        case 0x07: return handle_set_basic_config(axis, msg, txmsg);
        case 0x08: return handle_get_anticogging_status(axis, msg, txmsg);
        case 0x09: return handle_set_anticogging_config(axis, msg, txmsg);
        case 0x0A: return handle_get_vernier_diagnostics(axis, msg, txmsg);
        default:   return false;
    }
}

// =====================================================================
// 0x01: GET_AXIS_STATUS_EX
// =====================================================================
bool CANSimple::handle_get_axis_status_ex(Axis& axis, can_Message_t& txmsg) {
    uint8_t flags = 0;
    if (axis.motor_.is_calibrated_)          flags |= (1 << 0);
    if (axis.encoder_.is_ready_)             flags |= (1 << 1);
    if (axis.motor_.error_ != 0)             flags |= (1 << 2);
    if (axis.encoder_.error_ != 0)           flags |= (1 << 3);
    if (axis.controller_.error_ != 0)        flags |= (1 << 4);
    if (axis.controller_.trajectory_done_)   flags |= (1 << 5);

    txmsg.buf[0] = 0x01;
    txmsg.buf[1] = 0x00;
    txmsg.buf[2] = static_cast<uint8_t>(axis.current_state_);
    txmsg.buf[3] = flags;
    can_setSignal<uint32_t>(txmsg, axis.error_, 32, 32, true);
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x02: SET_PRECALIBRATED
// =====================================================================
bool CANSimple::handle_set_precalibrated(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t flags = msg.buf[1];
    uint8_t status = EXT_STATUS_OK;

    if ((flags & (1 << 0)) && (flags & (1 << 4))) status = EXT_STATUS_UNKNOWN;
    if ((flags & (1 << 1)) && (flags & (1 << 5))) status = EXT_STATUS_UNKNOWN;
    if (!(flags & 0x33))                           status = EXT_STATUS_UNKNOWN;
    if (flags & ~0x33)                             status = EXT_STATUS_UNKNOWN;

    if (status == EXT_STATUS_OK) {
        if (flags & (1 << 0)) { axis.motor_.config_.pre_calibrated = true;  axis.motor_.is_calibrated_ = true; }
        if (flags & (1 << 1)) { axis.encoder_.config_.pre_calibrated = true; axis.encoder_.is_ready_ = true; }
        if (flags & (1 << 4)) { axis.motor_.config_.pre_calibrated = false; axis.motor_.is_calibrated_ = false; }
        if (flags & (1 << 5)) { axis.encoder_.config_.pre_calibrated = false; axis.encoder_.is_ready_ = false; }
    }

    txmsg.buf[0] = 0x02;
    txmsg.buf[1] = 0x00;
    txmsg.buf[2] = status;
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x03: SAVE_CONFIGURATION
// =====================================================================
bool CANSimple::handle_save_configuration(can_Message_t& txmsg) {
    txmsg.buf[0] = 0x03;
    txmsg.buf[1] = 0x00;

    if (any_axis_armed()) {
        txmsg.buf[2] = EXT_STATUS_BUSY_ARMED;
        return canbus_->send_message(txmsg);
    }

    txmsg.buf[2] = EXT_STATUS_OK;
    canbus_->send_message(txmsg);

    // save_configuration() calls NVIC_SystemReset() after writing.
    // The ACK above should have been transmitted by now.
    odrv.save_configuration();
    return true;
}

// =====================================================================
// 0x04: GET_CALIB_RESULT
// =====================================================================
bool CANSimple::handle_get_calib_result(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t item_id = msg.buf[1];
    uint8_t status = EXT_STATUS_OK;
    uint8_t type   = EXT_TYPE_UINT32;

    txmsg.buf[0] = 0x04;
    txmsg.buf[1] = item_id;

    switch (item_id) {
        case 0x01: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg, axis.motor_.config_.phase_resistance, 32, 32, true); break;
        case 0x02: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg, axis.motor_.config_.phase_inductance, 32, 32, true); break;
        case 0x03: type = EXT_TYPE_INT32;   can_setSignal<int32_t>(txmsg, axis.encoder_.config_.phase_offset, 32, 32, true); break;
        case 0x04: type = EXT_TYPE_INT32;   can_setSignal<int32_t>(txmsg, axis.encoder_.config_.direction, 32, 32, true); break;
        default:   status = EXT_STATUS_UNKNOWN; can_setSignal<uint32_t>(txmsg, 0, 32, 32, true); break;
    }

    txmsg.buf[2] = status;
    txmsg.buf[3] = type;
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x05: GET_DEVICE_INFO
// =====================================================================
bool CANSimple::handle_get_device_info(const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t item_id = msg.buf[1];
    uint8_t  status = EXT_STATUS_OK;
    uint8_t  type   = EXT_TYPE_UINT32;
    uint32_t value  = 0;

    switch (item_id) {
        case 0x01:  // protocol_version
            value = EXT_PROTOCOL_VERSION;
            break;
        case 0x02:  // fw_version  (major<<24 | minor<<16 | revision<<8 | unreleased)
            value = (static_cast<uint32_t>(odrv.fw_version_major_)  << 24)
                  | (static_cast<uint32_t>(odrv.fw_version_minor_)  << 16)
                  | (static_cast<uint32_t>(odrv.fw_version_revision_) << 8)
                  | (static_cast<uint32_t>(odrv.fw_version_unreleased_));
            break;
        case 0x03:  // hw_version  (major<<16 | minor<<8 | variant)
            value = (static_cast<uint32_t>(odrv.hw_version_major_)  << 16)
                  | (static_cast<uint32_t>(odrv.hw_version_minor_)  << 8)
                  | (static_cast<uint32_t>(odrv.hw_version_variant_));
            break;
        case 0x04:  // serial_number low 32 bits
            value = static_cast<uint32_t>(odrv.serial_number_ & 0xFFFFFFFFULL);
            break;
        case 0x05:  // serial_number high 32 bits
            value = static_cast<uint32_t>((odrv.serial_number_ >> 32) & 0xFFFFFFFFULL);
            break;
        default:
            status = EXT_STATUS_UNKNOWN;
            break;
    }

    txmsg.buf[0] = 0x05;
    txmsg.buf[1] = item_id;
    txmsg.buf[2] = status;
    txmsg.buf[3] = type;
    can_setSignal<uint32_t>(txmsg, value, 32, 32, true);
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x06: GET_BASIC_CONFIG
// =====================================================================
bool CANSimple::handle_get_basic_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t param_id = msg.buf[1];
    uint8_t status = EXT_STATUS_OK;
    uint8_t type   = EXT_TYPE_UINT32;

    txmsg.buf[0] = 0x06;
    txmsg.buf[1] = param_id;

    switch (param_id) {
        // --- motor ---
        case 0x10: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, static_cast<uint32_t>(axis.motor_.config_.motor_type), 32, 32, true); break;
        case 0x11: type = EXT_TYPE_INT32;   can_setSignal<int32_t>(txmsg,  axis.motor_.config_.pole_pairs, 32, 32, true); break;
        case 0x12: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.motor_.config_.calibration_current, 32, 32, true); break;
        case 0x13: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.motor_.config_.resistance_calib_max_voltage, 32, 32, true); break;
        case 0x14: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.motor_.config_.current_lim, 32, 32, true); break;
        case 0x15: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.motor_.config_.torque_constant, 32, 32, true); break;
        // --- encoder ---
        case 0x20: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, static_cast<uint32_t>(axis.encoder_.config_.mode), 32, 32, true); break;
        case 0x21: type = EXT_TYPE_INT32;   can_setSignal<int32_t>(txmsg,  axis.encoder_.config_.cpr, 32, 32, true); break;
        case 0x22: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.abs_spi_cs_gpio_pin, 32, 32, true); break;
        case 0x23: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.bandwidth, 32, 32, true); break;
        case 0x24: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.abs_spi_aux_cs_gpio_pin, 32, 32, true); break;
        case 0x25: type = EXT_TYPE_INT32;   can_setSignal<int32_t>(txmsg,  axis.encoder_.config_.vernier_virtual_cpr, 32, 32, true); break;
        case 0x26: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.vernier_main_ratio, 32, 32, true); break;
        case 0x27: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.vernier_aux_ratio, 32, 32, true); break;
        case 0x28: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.vernier_main_offset, 32, 32, true); break;
        case 0x29: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.vernier_aux_offset, 32, 32, true); break;
        case 0x2A: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.vernier_main_reversed ? 1u : 0u, 32, 32, true); break;
        case 0x2B: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.vernier_aux_reversed ? 1u : 0u, 32, 32, true); break;
        case 0x2C: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.mt6826s_spi_mode, 32, 32, true); break;
        case 0x2D: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.vernier_err_accept, 32, 32, true); break;
        case 0x2E: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.encoder_.config_.vernier_err_reject, 32, 32, true); break;
        case 0x2F: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.mt6826s_spi_prescaler, 32, 32, true); break;
        case 0x33: type = EXT_TYPE_UINT32;  can_setSignal<uint32_t>(txmsg, axis.encoder_.config_.vernier_output_reversed ? 1u : 0u, 32, 32, true); break;
        // --- controller ---
        case 0x30: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.controller_.config_.pos_gain, 32, 32, true); break;
        case 0x31: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.controller_.config_.vel_gain, 32, 32, true); break;
        case 0x32: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   axis.controller_.config_.vel_integrator_gain, 32, 32, true); break;
        // --- odrive power limits ---
        case 0x40: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   odrv.config_.dc_max_negative_current, 32, 32, true); break;
        case 0x41: type = EXT_TYPE_FLOAT32; can_setSignal<float>(txmsg,   odrv.config_.dc_max_positive_current, 32, 32, true); break;
        default:   status = EXT_STATUS_UNKNOWN; can_setSignal<uint32_t>(txmsg, 0, 32, 32, true); break;
    }

    txmsg.buf[2] = status;
    txmsg.buf[3] = type;
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x07: SET_BASIC_CONFIG
// =====================================================================
bool CANSimple::handle_set_basic_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t  param_id = msg.buf[1];
    const uint8_t  req_type = msg.buf[2];
    uint8_t status = EXT_STATUS_OK;

    txmsg.buf[0] = 0x07;
    txmsg.buf[1] = param_id;
    txmsg.buf[3] = req_type;

    // Refuse writes while any motor is armed
    if (any_axis_armed()) {
        txmsg.buf[2] = EXT_STATUS_BUSY_ARMED;
        return canbus_->send_message(txmsg);
    }

    switch (param_id) {
        // --- motor ---
        case 0x10: {  // motor_type (uint32)
            if (req_type != EXT_TYPE_UINT32 && req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            uint32_t value = can_getSignal<uint32_t>(msg, 32, 32, true);
            if (!is_valid_motor_type(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.motor_.config_.motor_type = static_cast<Motor::MotorType>(value);
            break;
        }
        case 0x11: {  // pole_pairs (int32)
            if (req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            int32_t value = can_getSignal<int32_t>(msg, 32, 32, true);
            if (value <= 0) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.motor_.config_.pole_pairs = value;
            break;
        }
        case 0x12: {  // calibration_current (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.motor_.config_.calibration_current = value;
            break;
        }
        case 0x13: {  // resistance_calib_max_voltage (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.motor_.config_.resistance_calib_max_voltage = value;
            break;
        }
        case 0x14: {  // current_lim (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.motor_.config_.current_lim = value;
            break;
        }
        case 0x15: {  // torque_constant (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.motor_.config_.torque_constant = value;
            break;
        }
        // --- encoder ---
        case 0x20: {  // encoder mode (uint32)
            if (req_type != EXT_TYPE_UINT32 && req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            uint32_t value = can_getSignal<uint32_t>(msg, 32, 32, true);
            if (!is_valid_encoder_mode(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.mode = static_cast<Encoder::Mode>(value);
            break;
        }
        case 0x21: {  // cpr (int32)
            if (req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            int32_t value = can_getSignal<int32_t>(msg, 32, 32, true);
            if (value <= 0) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.cpr = value;
            break;
        }
        case 0x22: {  // abs_spi_cs_gpio_pin (uint32, stored as uint16)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            uint32_t value = can_getSignal<uint32_t>(msg, 32, 32, true);
            if (value >= GPIO_COUNT) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_abs_spi_cs_gpio_pin(static_cast<uint16_t>(value));
            break;
        }
        case 0x23: {  // encoder bandwidth (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_bandwidth(value);
            break;
        }
        case 0x24: {  // abs_spi_aux_cs_gpio_pin (uint32, stored as uint16)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            uint32_t value = can_getSignal<uint32_t>(msg, 32, 32, true);
            if (value >= GPIO_COUNT) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_abs_spi_aux_cs_gpio_pin(static_cast<uint16_t>(value));
            break;
        }
        case 0x25: {  // vernier_virtual_cpr (int32)
            if (req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            int32_t value = can_getSignal<int32_t>(msg, 32, 32, true);
            if (value <= 0) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.vernier_virtual_cpr = value;
            break;
        }
        case 0x26: {  // vernier_main_ratio (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_vernier_main_ratio(value);
            break;
        }
        case 0x27: {  // vernier_aux_ratio (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_vernier_aux_ratio(value);
            break;
        }
        case 0x28: {  // vernier_main_offset (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!std::isfinite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_vernier_main_offset(value);
            break;
        }
        case 0x29: {  // vernier_aux_offset (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!std::isfinite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_vernier_aux_offset(value);
            break;
        }
        case 0x2A: {  // vernier_main_reversed (uint32 as bool)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            axis.encoder_.config_.set_vernier_main_reversed(can_getSignal<uint32_t>(msg, 32, 32, true) != 0);
            break;
        }
        case 0x2B: {  // vernier_aux_reversed (uint32 as bool)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            axis.encoder_.config_.set_vernier_aux_reversed(can_getSignal<uint32_t>(msg, 32, 32, true) != 0);
            break;
        }
        case 0x2C: {  // mt6826s_spi_mode (uint32, 0..3)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            uint32_t value = can_getSignal<uint32_t>(msg, 32, 32, true);
            if (value > 3u) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_mt6826s_spi_mode(static_cast<uint16_t>(value));
            break;
        }
        case 0x2D: {  // vernier_err_accept (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_vernier_err_accept(value);
            break;
        }
        case 0x2E: {  // vernier_err_reject (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_vernier_err_reject(value);
            break;
        }
        case 0x2F: {  // mt6826s_spi_prescaler (uint32 divisor)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            uint32_t value = can_getSignal<uint32_t>(msg, 32, 32, true);
            if (!is_valid_spi_prescaler_divisor(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.encoder_.config_.set_mt6826s_spi_prescaler(static_cast<uint16_t>(value));
            break;
        }
        case 0x33: {  // vernier_output_reversed (uint32 as bool)
            if (req_type != EXT_TYPE_UINT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            axis.encoder_.config_.set_vernier_output_reversed(can_getSignal<uint32_t>(msg, 32, 32, true) != 0);
            break;
        }
        // --- controller ---
        case 0x30: {  // pos_gain (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_nonnegative_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.controller_.config_.pos_gain = value;
            break;
        }
        case 0x31: {  // vel_gain (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_nonnegative_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.controller_.config_.vel_gain = value;
            break;
        }
        case 0x32: {  // vel_integrator_gain (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_nonnegative_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.controller_.config_.vel_integrator_gain = value;
            break;
        }
        case 0x40: {  // odrv.config.dc_max_negative_current (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!std::isfinite(value) || value > 0.0f) { status = EXT_STATUS_INVALID_VALUE; break; }
            odrv.config_.dc_max_negative_current = value;
            break;
        }
        case 0x41: {  // odrv.config.dc_max_positive_current (float32)
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            odrv.config_.dc_max_positive_current = value;
            break;
        }
        default:
            status = EXT_STATUS_UNKNOWN;
            break;
    }

    txmsg.buf[2] = status;
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x0A: GET_VERNIER_DIAGNOSTICS
//
// item 0x00: main_angle (uint32)
// item 0x01: aux_angle (uint32)
// item 0x02: main_valid (uint32/bool)
// item 0x03: aux_valid (uint32/bool)
// item 0x04: pair_sequence (uint32)
// item 0x05: pair_valid / resolver_locked placeholder (uint32/bool)
// item 0x06: resolver_residual placeholder (float32)
// item 0x07: vernier_virtual_count placeholder (int32)
// item 0x08: vernier_position_turns placeholder (float32)
// item 0x09: main_error_count (uint32)
// item 0x0A: aux_error_count (uint32)
// item 0x0B: resolver_state placeholder (uint32)
// item 0x0C: pair_error_count (uint32)
// item 0x10: main_raw_003_006, little-endian byte pack (uint32)
// item 0x11: aux_raw_003_006, little-endian byte pack (uint32)
// item 0x12: main_crc_recv_calc, received in bits[7:0], calculated in bits[15:8] (uint32)
// item 0x13: aux_crc_recv_calc, received in bits[7:0], calculated in bits[15:8] (uint32)
// item 0x14: main_spi_dma_error_count (uint32)
// item 0x15: main_crc_error_count (uint32)
// item 0x16: main_fixed_bit_error_count (uint32)
// item 0x17: main_status_warning_count (uint32)
// item 0x18: main_sample_count (uint32)
// item 0x19: aux_spi_dma_error_count (uint32)
// item 0x1A: aux_crc_error_count (uint32)
// item 0x1B: aux_fixed_bit_error_count (uint32)
// item 0x1C: aux_status_warning_count (uint32)
// item 0x1D: aux_sample_count (uint32)
// item 0x1E: main_sequence (uint32)
// item 0x1F: aux_sequence (uint32)
// item 0x20: encoder_pos_estimate (float32)
// item 0x21: encoder_vel_estimate (float32)
// item 0x22: encoder_pos_circular (float32)
// =====================================================================
static uint32_t pack_mt6826s_raw(const Mt6826sSpi::Sample& sample) {
    return static_cast<uint32_t>(sample.raw[0])
        | (static_cast<uint32_t>(sample.raw[1]) << 8)
        | (static_cast<uint32_t>(sample.raw[2]) << 16)
        | (static_cast<uint32_t>(sample.raw[3]) << 24);
}

static uint32_t pack_mt6826s_crc(const Mt6826sSpi::Sample& sample) {
    return static_cast<uint32_t>(sample.crc)
        | (static_cast<uint32_t>(sample.crc_calc) << 8);
}

bool CANSimple::handle_get_vernier_diagnostics(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t item_id = msg.buf[1];
    uint8_t status = EXT_STATUS_OK;
    uint8_t type = EXT_TYPE_UINT32;
    Encoder::VernierDiagnosticsSnapshot snapshot = {};
    axis.encoder_.get_vernier_diagnostics_snapshot(&snapshot);

    txmsg.buf[0] = 0x0A;
    txmsg.buf[1] = item_id;

    switch (item_id) {
        case 0x00:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_sample.angle, 32, 32, true);
            break;
        case 0x01:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_sample.angle, 32, 32, true);
            break;
        case 0x02:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_sample.valid ? 1u : 0u, 32, 32, true);
            break;
        case 0x03:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_sample.valid ? 1u : 0u, 32, 32, true);
            break;
        case 0x04:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.pair_count, 32, 32, true);
            break;
        case 0x05:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.pair_valid ? 1u : 0u, 32, 32, true);
            break;
        case 0x06:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, snapshot.residual, 32, 32, true);
            break;
        case 0x07:
            type = EXT_TYPE_INT32;
            can_setSignal<int32_t>(txmsg, snapshot.virtual_count, 32, 32, true);
            break;
        case 0x08:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, snapshot.position_turns, 32, 32, true);
            break;
        case 0x09:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_error_count, 32, 32, true);
            break;
        case 0x0A:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_error_count, 32, 32, true);
            break;
        case 0x0B:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.state, 32, 32, true);
            break;
        case 0x0C:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.pair_error_count, 32, 32, true);
            break;
        case 0x10:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, pack_mt6826s_raw(snapshot.main_sample), 32, 32, true);
            break;
        case 0x11:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, pack_mt6826s_raw(snapshot.aux_sample), 32, 32, true);
            break;
        case 0x12:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, pack_mt6826s_crc(snapshot.main_sample), 32, 32, true);
            break;
        case 0x13:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, pack_mt6826s_crc(snapshot.aux_sample), 32, 32, true);
            break;
        case 0x14:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_spi_dma_error_count, 32, 32, true);
            break;
        case 0x15:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_crc_error_count, 32, 32, true);
            break;
        case 0x16:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_fixed_bit_error_count, 32, 32, true);
            break;
        case 0x17:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_status_warning_count, 32, 32, true);
            break;
        case 0x18:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_sample_count, 32, 32, true);
            break;
        case 0x19:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_spi_dma_error_count, 32, 32, true);
            break;
        case 0x1A:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_crc_error_count, 32, 32, true);
            break;
        case 0x1B:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_fixed_bit_error_count, 32, 32, true);
            break;
        case 0x1C:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_status_warning_count, 32, 32, true);
            break;
        case 0x1D:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_sample_count, 32, 32, true);
            break;
        case 0x1E:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.main_sample.sequence, 32, 32, true);
            break;
        case 0x1F:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, snapshot.aux_sample.sequence, 32, 32, true);
            break;
        case 0x20:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, snapshot.encoder_pos_estimate, 32, 32, true);
            break;
        case 0x21:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, snapshot.encoder_vel_estimate, 32, 32, true);
            break;
        case 0x22:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, snapshot.encoder_pos_circular, 32, 32, true);
            break;
        // Debug counters (0x30-0x39)
        case 0x30: {
            type = EXT_TYPE_UINT32;
            uint32_t v = g_debug.foc_bad_timing_cnt;
            can_setSignal<uint32_t>(txmsg, v, 32, 32, true);
            break;
        }
        case 0x34: {
            type = EXT_TYPE_UINT32;
            uint32_t v = g_debug.cl_adc_fail_pre_cnt;
            can_setSignal<uint32_t>(txmsg, v, 32, 32, true);
            break;
        }
        case 0x35: {
            type = EXT_TYPE_UINT32;
            uint32_t v = g_debug.cl_adc_fail_post_cnt;
            can_setSignal<uint32_t>(txmsg, v, 32, 32, true);
            break;
        }
        case 0x36: {
            type = EXT_TYPE_UINT32;
            uint32_t v = g_debug.cl_deadline_miss_cnt;
            can_setSignal<uint32_t>(txmsg, v, 32, 32, true);
            break;
        }
        case 0x37: {
            type = EXT_TYPE_UINT32;
            uint32_t v = g_debug.enc_pair_busy_cnt;
            can_setSignal<uint32_t>(txmsg, v, 32, 32, true);
            break;
        }
        case 0x38: {
            type = EXT_TYPE_UINT32;
            uint32_t v = g_debug.enc_pair_ok_cnt;
            can_setSignal<uint32_t>(txmsg, v, 32, 32, true);
            break;
        }
        default:
            status = EXT_STATUS_UNKNOWN;
            can_setSignal<uint32_t>(txmsg, 0, 32, 32, true);
            break;
    }

    txmsg.buf[2] = status;
    txmsg.buf[3] = type;
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x08: GET_ANTICOGGING_STATUS
//
// item 0x01: flags
//   bit0 calib_anticogging, bit1 anticogging_valid, bit2 pre_calibrated,
//   bit3 anticogging_enabled
// item 0x02: calibration index [0..3600]
// item 0x03: calib_pos_threshold [encoder counts]
// item 0x04: calib_vel_threshold [encoder counts/s]
// item 0x05: cogging_ratio [turn/index]
// item 0x06: odrv.error
// item 0x10: cogging_map[index], where request value is the uint32 index
// =====================================================================
bool CANSimple::handle_get_anticogging_status(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t item_id = msg.buf[1];
    uint8_t status = EXT_STATUS_OK;
    uint8_t type = EXT_TYPE_UINT32;

    txmsg.buf[0] = 0x08;
    txmsg.buf[1] = item_id;

    switch (item_id) {
        case 0x01: {
            uint32_t flags = 0;
            if (axis.controller_.config_.anticogging.calib_anticogging) flags |= (1 << 0);
            if (axis.controller_.anticogging_valid_)                    flags |= (1 << 1);
            if (axis.controller_.config_.anticogging.pre_calibrated)     flags |= (1 << 2);
            if (axis.controller_.config_.anticogging.anticogging_enabled) flags |= (1 << 3);
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, flags, 32, 32, true);
            break;
        }
        case 0x02:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, axis.controller_.config_.anticogging.index, 32, 32, true);
            break;
        case 0x03:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, axis.controller_.config_.anticogging.calib_pos_threshold, 32, 32, true);
            break;
        case 0x04:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, axis.controller_.config_.anticogging.calib_vel_threshold, 32, 32, true);
            break;
        case 0x05:
            type = EXT_TYPE_FLOAT32;
            can_setSignal<float>(txmsg, axis.encoder_.getCoggingRatio(), 32, 32, true);
            break;
        case 0x06:
            type = EXT_TYPE_UINT32;
            can_setSignal<uint32_t>(txmsg, static_cast<uint32_t>(odrv.error_), 32, 32, true);
            break;
        case 0x10: {
            uint32_t index = can_getSignal<uint32_t>(msg, 32, 32, true);
            type = EXT_TYPE_FLOAT32;
            if (index >= 3600) {
                status = EXT_STATUS_INVALID_VALUE;
                can_setSignal<float>(txmsg, 0.0f, 32, 32, true);
            } else {
                can_setSignal<float>(txmsg, axis.controller_.config_.anticogging.cogging_map[index], 32, 32, true);
            }
            break;
        }
        default:
            status = EXT_STATUS_UNKNOWN;
            can_setSignal<uint32_t>(txmsg, 0, 32, 32, true);
            break;
    }

    txmsg.buf[2] = status;
    txmsg.buf[3] = type;
    return canbus_->send_message(txmsg);
}

// =====================================================================
// 0x09: SET_ANTICOGGING_CONFIG
//
// item 0x01: anticogging_enabled (uint32/bool)
// item 0x02: pre_calibrated (uint32/bool)
// item 0x03: calib_pos_threshold [encoder counts] (float32)
// item 0x04: calib_vel_threshold [encoder counts/s] (float32)
// item 0x05: reset calibration state/map index (uint32, nonzero resets)
// =====================================================================
bool CANSimple::handle_set_anticogging_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg) {
    const uint8_t item_id = msg.buf[1];
    const uint8_t req_type = msg.buf[2];
    uint8_t status = EXT_STATUS_OK;

    txmsg.buf[0] = 0x09;
    txmsg.buf[1] = item_id;
    txmsg.buf[3] = req_type;

    if (any_axis_armed()) {
        txmsg.buf[2] = EXT_STATUS_BUSY_ARMED;
        return canbus_->send_message(txmsg);
    }

    switch (item_id) {
        case 0x01: {
            if (req_type != EXT_TYPE_UINT32 && req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            axis.controller_.config_.anticogging.anticogging_enabled = can_getSignal<uint32_t>(msg, 32, 32, true) != 0;
            break;
        }
        case 0x02: {
            if (req_type != EXT_TYPE_UINT32 && req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            bool value = can_getSignal<uint32_t>(msg, 32, 32, true) != 0;
            axis.controller_.config_.anticogging.pre_calibrated = value;
            axis.controller_.anticogging_valid_ = value;
            break;
        }
        case 0x03: {
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.controller_.config_.anticogging.calib_pos_threshold = value;
            break;
        }
        case 0x04: {
            if (req_type != EXT_TYPE_FLOAT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            float value = can_getSignal<float>(msg, 32, 32, true);
            if (!is_positive_finite(value)) { status = EXT_STATUS_INVALID_VALUE; break; }
            axis.controller_.config_.anticogging.calib_vel_threshold = value;
            break;
        }
        case 0x05: {
            if (req_type != EXT_TYPE_UINT32 && req_type != EXT_TYPE_INT32) { status = EXT_STATUS_INVALID_TYPE; break; }
            if (can_getSignal<uint32_t>(msg, 32, 32, true) != 0) {
                axis.controller_.config_.anticogging.index = 0;
                axis.controller_.config_.anticogging.calib_anticogging = false;
                axis.controller_.anticogging_valid_ = false;
            }
            break;
        }
        default:
            status = EXT_STATUS_UNKNOWN;
            break;
    }

    txmsg.buf[2] = status;
    return canbus_->send_message(txmsg);
}
