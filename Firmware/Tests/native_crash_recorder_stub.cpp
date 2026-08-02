#include "MotorControl/crash_recorder.hpp"

namespace odrive::crash {

size_t encode_pending(uint8_t*, size_t, uint32_t*) {
    return 0u;
}

bool pending() {
    return false;
}

bool pending_crc(uint32_t) {
    return false;
}

bool clear_if_crc(uint32_t) {
    return false;
}

}  // namespace odrive::crash
