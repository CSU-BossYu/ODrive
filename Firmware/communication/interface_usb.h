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
bool usb_stdout_is_connected(void);

// Binary Debug/Test channel. Bytes enter here only from the USB task after the
// CDC interrupt has copied them into its bounded C ingress ring.
void usb_debug_receive_bytes(const uint8_t* data, size_t length);
void usb_debug_notify_connected(bool connected);
void usb_debug_service(void);
bool usb_debug_begin_tx_chunk(uint16_t maximum_length, const uint8_t** data,
                              uint16_t* length);
void usb_debug_complete_tx_chunk(uint16_t transmitted_length);
bool usb_debug_binary_session_active(void);
bool usb_debug_publish_calibration(uint16_t record_type, uint32_t sequence,
                                   const uint8_t* payload, size_t length);

#ifdef __cplusplus
}
#endif


#endif // __INTERFACE_USB_HPP
