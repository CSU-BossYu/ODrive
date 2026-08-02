#include <doctest.h>

#include "communication/can/can_product_schema_generated.hpp"

using namespace odrive::can::product;

TEST_SUITE("CanProductProtocol") {
    TEST_CASE("management request has a fixed little-endian layout") {
        const uint8_t payload[8] = {
            0x34u, 0x12u, 0x04u, 0x02u, 0x78u, 0x56u, 0x34u, 0x12u};
        const ManagementCommand command = decode_management_command(payload);
        CHECK(command.request_id == 0x1234u);
        CHECK(command.command_type == 4u);
        CHECK(command.operation == 2u);
        CHECK(command.arg0 == 0x12345678u);
    }

    TEST_CASE("command ACK has a fixed little-endian layout") {
        uint8_t payload[8] = {};
        encode_command_ack({0x1234u, 2u, 0x5678u, 0x9ABCu}, payload);
        const uint8_t expected[8] = {
            0x34u, 0x12u, 0x02u, 0x00u, 0x78u, 0x56u, 0xBCu, 0x9Au};
        for (size_t index = 0u; index < 8u; ++index) {
            CHECK(payload[index] == expected[index]);
        }
    }

    TEST_CASE("product IDs preserve setpoint compatibility and reserve ACK") {
        CHECK(static_cast<uint8_t>(MessageId::COMMAND_ACK) == 0x05u);
        CHECK(static_cast<uint8_t>(MessageId::MANAGEMENT_COMMAND) == 0x08u);
        CHECK(static_cast<uint8_t>(MessageId::PRODUCT_STATUS) == 0x10u);
        CHECK(static_cast<uint8_t>(MessageId::SET_INPUT_POS) == 0x0Cu);
        CHECK(static_cast<uint8_t>(MessageId::SET_MIT_CONTROL) == 0x1Fu);
    }
}
