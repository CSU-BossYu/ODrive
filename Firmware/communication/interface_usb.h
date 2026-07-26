#ifndef __INTERFACE_USB_HPP
#define __INTERFACE_USB_HPP


#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern osThreadId usb_thread;
extern const uint32_t stack_size_usb_thread;

typedef struct {
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t tx_overrun_cnt;
} USBStats_t;

extern USBStats_t usb_stats_;

void start_usb_server(void);
size_t usb_stdout_write(const uint8_t* data, size_t length);
// Queues the complete frame atomically or queues nothing. Intended for binary
// calibration records that must never be interleaved with stdout text.
bool usb_stdout_write_frame(const uint8_t* data, size_t length);
bool usb_stdout_is_connected(void);

#ifdef __cplusplus
}
#endif


#endif // __INTERFACE_USB_HPP
