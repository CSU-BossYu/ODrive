#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "canbus.hpp"
#include "axis.hpp"
#include "can_product_schema_generated.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = static_cast<uint8_t>(odrive::can::product::MessageId::NMT),
        MSG_ODRIVE_HEARTBEAT = static_cast<uint8_t>(odrive::can::product::MessageId::HEARTBEAT),
        MSG_ODRIVE_ESTOP = static_cast<uint8_t>(odrive::can::product::MessageId::ESTOP),
        MSG_GET_MOTOR_ERROR = static_cast<uint8_t>(odrive::can::product::MessageId::GET_MOTOR_ERROR),
        MSG_GET_ENCODER_ERROR = static_cast<uint8_t>(odrive::can::product::MessageId::GET_ENCODER_ERROR),
        MSG_COMMAND_ACK = static_cast<uint8_t>(odrive::can::product::MessageId::COMMAND_ACK),
        MSG_SET_AXIS_NODE_ID = static_cast<uint8_t>(odrive::can::product::MessageId::SET_AXIS_NODE_ID),
        MSG_SET_AXIS_REQUESTED_STATE = static_cast<uint8_t>(odrive::can::product::MessageId::SET_AXIS_STATE),
        MSG_MANAGEMENT_COMMAND = static_cast<uint8_t>(odrive::can::product::MessageId::MANAGEMENT_COMMAND),
        MSG_GET_ENCODER_ESTIMATES = static_cast<uint8_t>(odrive::can::product::MessageId::ENCODER_ESTIMATES),
        MSG_GET_ENCODER_COUNT = static_cast<uint8_t>(odrive::can::product::MessageId::GET_ENCODER_COUNT),
        MSG_SET_CONTROLLER_MODES = static_cast<uint8_t>(odrive::can::product::MessageId::SET_CONTROLLER_MODE),
        MSG_SET_INPUT_POS = static_cast<uint8_t>(odrive::can::product::MessageId::SET_INPUT_POS),
        MSG_SET_INPUT_VEL = static_cast<uint8_t>(odrive::can::product::MessageId::SET_INPUT_VEL),
        MSG_SET_INPUT_TORQUE = static_cast<uint8_t>(odrive::can::product::MessageId::SET_INPUT_TORQUE),
        MSG_SET_LIMITS = static_cast<uint8_t>(odrive::can::product::MessageId::SET_LIMITS),
        MSG_PRODUCT_STATUS = static_cast<uint8_t>(odrive::can::product::MessageId::PRODUCT_STATUS),
        MSG_SET_TRAJ_VEL_LIMIT = static_cast<uint8_t>(odrive::can::product::MessageId::SET_TRAJ_VEL_LIMIT),
        MSG_SET_TRAJ_ACCEL_LIMITS = static_cast<uint8_t>(odrive::can::product::MessageId::SET_TRAJ_ACCEL_LIMITS),
        MSG_SET_TRAJ_INERTIA = static_cast<uint8_t>(odrive::can::product::MessageId::SET_TRAJ_INERTIA),
        MSG_GET_IQ = static_cast<uint8_t>(odrive::can::product::MessageId::IQ),
        MSG_RESET_ODRIVE = static_cast<uint8_t>(odrive::can::product::MessageId::REBOOT),
        MSG_GET_BUS_VOLTAGE_CURRENT = static_cast<uint8_t>(odrive::can::product::MessageId::BUS_VOLTAGE_CURRENT),
        MSG_CLEAR_ERRORS = static_cast<uint8_t>(odrive::can::product::MessageId::CLEAR_ERRORS),
        MSG_SET_LINEAR_COUNT = static_cast<uint8_t>(odrive::can::product::MessageId::SET_LINEAR_COUNT),
        MSG_SET_POS_GAIN = static_cast<uint8_t>(odrive::can::product::MessageId::SET_POS_GAIN),
        MSG_SET_VEL_GAINS = static_cast<uint8_t>(odrive::can::product::MessageId::SET_VEL_GAINS),
        MSG_GET_ADC_VOLTAGE = static_cast<uint8_t>(odrive::can::product::MessageId::GET_ADC_VOLTAGE),
        MSG_GET_CONTROLLER_ERROR = static_cast<uint8_t>(odrive::can::product::MessageId::GET_CONTROLLER_ERROR),
        MSG_EXTENDED_COMMAND = static_cast<uint8_t>(odrive::can::product::MessageId::LEGACY_EXTENDED),
        MSG_SET_MIT_CONTROL = static_cast<uint8_t>(odrive::can::product::MessageId::SET_MIT_CONTROL),
    };

    CANSimple(CanBusBase* canbus) : canbus_(canbus) {}

    bool init();
    uint32_t service_stack();

   private:

    bool renew_subscription(size_t i);
    bool send_heartbeat(const Axis& axis);
    bool send_product_status(const Axis& axis);
    bool handle_management_command(Axis& axis, const can_Message_t& msg);
    bool send_command_ack(uint16_t request_id, uint8_t status,
                          uint16_t state_epoch, uint16_t reason);
    void publish_command_result(const odrive::safety::CommandResult& result);
    static void command_result_sink(
        void* context, const odrive::safety::CommandResult& result);

    void handle_can_message(const can_Message_t& msg);

    void do_command(Axis& axis, const can_Message_t& cmd);
    
    // Get functions (msg.rtr bit must be set)
    bool get_motor_error_callback(const Axis& axis);
    bool get_encoder_error_callback(const Axis& axis);
    bool get_controller_error_callback(const Axis& axis);
    bool get_encoder_estimates_callback(const Axis& axis);
    bool get_encoder_count_callback(const Axis& axis);
    bool get_iq_callback(const Axis& axis);
    bool get_bus_voltage_current_callback(const Axis& axis);
    // msg.rtr bit must NOT be set
    bool get_adc_voltage_callback(const Axis& axis, const can_Message_t& msg);

    // Set functions
    static void set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_pos_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_vel_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_torque_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_modes_callback(Axis& axis, const can_Message_t& msg);
    static void set_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_inertia_callback(Axis& axis, const can_Message_t& msg);
    static void set_linear_count_callback(Axis& axis, const can_Message_t& msg);
    static void set_pos_gain_callback(Axis& axis, const can_Message_t& msg);
    static void set_vel_gains_callback(Axis& axis, const can_Message_t& msg);
    static void set_mit_control_callback(Axis& axis, const can_Message_t& msg);

    // Extended command (CMD 0x1E) dispatcher + sub-handlers
    bool legacy_extended_command_callback(Axis& axis, const can_Message_t& msg);

    bool handle_get_axis_status_ex(Axis& axis, can_Message_t& txmsg);
    bool handle_set_precalibrated(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_save_configuration(can_Message_t& txmsg);
    bool handle_get_calib_result(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_get_device_info(const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_get_basic_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_set_basic_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_get_vernier_diagnostics(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_get_fault_snapshot(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_vernier_calibration(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_calibration_session(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_get_control_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);
    bool handle_set_control_config(Axis& axis, const can_Message_t& msg, can_Message_t& txmsg);

    // Other functions
    static void estop_callback(Axis& axis, const can_Message_t& msg);
    static void clear_errors_callback(Axis& axis, const can_Message_t& msg);

    static constexpr uint8_t NUM_NODE_ID_BITS = 6;
    static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

    // Utility functions
    static constexpr uint32_t get_node_id(uint32_t msgID) {
        return (msgID >> NUM_CMD_ID_BITS);  // Upper 6 or more bits
    };

    static constexpr uint8_t get_cmd_id(uint32_t msgID) {
        return (msgID & 0x01F);  // Bottom 5 bits
    }

    CanBusBase* canbus_;
    CanBusBase::CanSubscription* subscription_handles_[AXIS_COUNT];

    // TODO: we this is a hack but actually we should use protocol hooks to
    // renew our filter when the node ID changes
    uint32_t node_ids_[AXIS_COUNT];
    bool extended_node_ids_[AXIS_COUNT];
    odrive::safety::FixedSpscRing<odrive::safety::CommandResult, 17>
        command_result_ring_;
    uint32_t command_result_overflow_ = 0u;
    std::array<uint16_t, 8> pending_management_requests_{};
    odrive::safety::CommandResult pending_command_result_{};
    bool pending_command_result_valid_ = false;
    uint32_t last_product_status_ = 0u;
};

#endif
