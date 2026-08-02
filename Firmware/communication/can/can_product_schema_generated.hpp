#ifndef ODRIVE_CAN_PRODUCT_SCHEMA_GENERATED_HPP
#define ODRIVE_CAN_PRODUCT_SCHEMA_GENERATED_HPP

#include <cstdint>

namespace odrive::can::product {

inline constexpr uint32_t kProtocolVersion = 0x0000010Cu;
inline constexpr uint8_t kNodeIdBits = 6u;
inline constexpr uint8_t kCommandIdBits = 5u;

enum class MessageId : uint8_t {
    NMT = 0x00u,
    HEARTBEAT = 0x01u,
    ESTOP = 0x02u,
    GET_MOTOR_ERROR = 0x03u,
    GET_ENCODER_ERROR = 0x04u,
    COMMAND_ACK = 0x05u,
    SET_AXIS_NODE_ID = 0x06u,
    SET_AXIS_STATE = 0x07u,
    MANAGEMENT_COMMAND = 0x08u,
    ENCODER_ESTIMATES = 0x09u,
    GET_ENCODER_COUNT = 0x0Au,
    SET_CONTROLLER_MODE = 0x0Bu,
    SET_INPUT_POS = 0x0Cu,
    SET_INPUT_VEL = 0x0Du,
    SET_INPUT_TORQUE = 0x0Eu,
    SET_LIMITS = 0x0Fu,
    PRODUCT_STATUS = 0x10u,
    SET_TRAJ_VEL_LIMIT = 0x11u,
    SET_TRAJ_ACCEL_LIMITS = 0x12u,
    SET_TRAJ_INERTIA = 0x13u,
    IQ = 0x14u,
    REBOOT = 0x16u,
    BUS_VOLTAGE_CURRENT = 0x17u,
    CLEAR_ERRORS = 0x18u,
    SET_LINEAR_COUNT = 0x19u,
    SET_POS_GAIN = 0x1Au,
    SET_VEL_GAINS = 0x1Bu,
    GET_ADC_VOLTAGE = 0x1Cu,
    GET_CONTROLLER_ERROR = 0x1Du,
    LEGACY_EXTENDED = 0x1Eu,
    SET_MIT_CONTROL = 0x1Fu,
};

struct ManagementCommand {
    uint16_t request_id = 0u;
    uint8_t command_type = 0u;
    uint8_t operation = 0u;
    uint32_t arg0 = 0u;
};

struct CommandAck {
    uint16_t request_id = 0u;
    uint8_t status = 0u;
    uint16_t state_epoch_low = 0u;
    uint16_t reason = 0u;
};

inline ManagementCommand decode_management_command(const uint8_t* data) {
    return {
        static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8u)),
        data[2], data[3],
        static_cast<uint32_t>(data[4]) |
            (static_cast<uint32_t>(data[5]) << 8u) |
            (static_cast<uint32_t>(data[6]) << 16u) |
            (static_cast<uint32_t>(data[7]) << 24u)};
}

inline void encode_command_ack(const CommandAck& value, uint8_t* data) {
    data[0] = static_cast<uint8_t>(value.request_id);
    data[1] = static_cast<uint8_t>(value.request_id >> 8u);
    data[2] = value.status;
    data[3] = 0u;
    data[4] = static_cast<uint8_t>(value.state_epoch_low);
    data[5] = static_cast<uint8_t>(value.state_epoch_low >> 8u);
    data[6] = static_cast<uint8_t>(value.reason);
    data[7] = static_cast<uint8_t>(value.reason >> 8u);
}

}  // namespace odrive::can::product

#endif
