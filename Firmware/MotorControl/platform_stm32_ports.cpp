#include "platform_ports.hpp"

#include "stm32f4xx.h"

namespace odrive::platform {

uint32_t cycle_count() {
    return DWT->CYCCNT;
}

}  // namespace odrive::platform
