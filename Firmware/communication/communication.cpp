/* Includes ------------------------------------------------------------------*/

#include "communication.h"
#include "interface_usb.h"
#include "interface_can.hpp"
#include "calibration_transport.hpp"
#include "odrive_main.h"

/* Global variables ----------------------------------------------------------*/

uint64_t serial_number;
char serial_number_str[13]; // 12 digits + null termination

void init_communication(void) {
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
