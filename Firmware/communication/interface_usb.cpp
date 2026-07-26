#include "interface_usb.h"

#include <algorithm>
#include <cmsis_os.h>
#include <freertos_vars.h>
#include <FreeRTOS.h>
#include <task.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>

osThreadId usb_thread;
const uint32_t stack_size_usb_thread = 2048; // Bytes
USBStats_t usb_stats_;

namespace {

// Large enough to absorb several calibration frames while USB FS completes
// the currently active packet. Long-term calibration storage remains host-side.
constexpr size_t kUsbLogBufferSize = 4096;
constexpr size_t kUsbPacketSize = USB_TX_DATA_SIZE - 1;
uint8_t log_buffer[kUsbLogBufferSize];
size_t read_index = 0;
size_t write_index = 0;
size_t active_length = 0;
volatile bool connected = false;
bool tx_active = false;

void reset_transport() {
    read_index = 0;
    write_index = 0;
    active_length = 0;
    tx_active = false;
}

void start_next_packet() {
    if (!connected || tx_active || read_index == write_index) {
        return;
    }

    size_t contiguous = write_index > read_index
        ? write_index - read_index
        : kUsbLogBufferSize - read_index;
    active_length = std::min(contiguous, kUsbPacketSize);

    if (CDC_Transmit_FS(&log_buffer[read_index], active_length, CDC_IN_EP) == USBD_OK) {
        tx_active = true;
        ++usb_stats_.tx_cnt;
    } else {
        active_length = 0;
        ++usb_stats_.tx_overrun_cnt;
    }
}

void usb_server_thread(void*) {
    for (;;) {
        osEvent event = osMessageGet(usb_event_queue, osWaitForever);
        if (event.status != osEventMessage) {
            continue;
        }

        switch (event.value.v) {
            case 1: // USB configured
                connected = true;
                start_next_packet();
                break;
            case 2: // USB disconnected
                connected = false;
                reset_transport();
                break;
            case 3: // CDC TX complete
                if (tx_active) {
                    read_index = (read_index + active_length) % kUsbLogBufferSize;
                }
                active_length = 0;
                tx_active = false;
                start_next_packet();
                break;
            case 7: // stdout data queued
                start_next_packet();
                break;
            default:
                break;
        }
    }
}

} // namespace

size_t usb_stdout_write(const uint8_t* data, size_t length) {
    size_t written = 0;

    taskENTER_CRITICAL();
    while (written < length) {
        size_t next = (write_index + 1) % kUsbLogBufferSize;
        if (next == read_index) {
            break;
        }
        log_buffer[write_index] = data[written++];
        write_index = next;
    }
    taskEXIT_CRITICAL();

    if (written < length) {
        ++usb_stats_.tx_overrun_cnt;
    }
    if (written != 0) {
        osMessagePut(usb_event_queue, 7, 0);
    }
    return written;
}

bool usb_stdout_write_frame(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0 || length >= kUsbLogBufferSize) {
        return false;
    }

    bool queued = false;
    taskENTER_CRITICAL();
    const size_t used = write_index >= read_index
        ? write_index - read_index
        : kUsbLogBufferSize - (read_index - write_index);
    const size_t free = kUsbLogBufferSize - used - 1;
    if (free >= length) {
        for (size_t i = 0; i < length; ++i) {
            log_buffer[write_index] = data[i];
            write_index = (write_index + 1) % kUsbLogBufferSize;
        }
        queued = true;
    }
    taskEXIT_CRITICAL();

    if (queued) {
        osMessagePut(usb_event_queue, 7, 0);
    }
    return queued;
}

bool usb_stdout_is_connected() {
    return connected;
}

void start_usb_server() {
    reset_transport();
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0,
                stack_size_usb_thread / sizeof(StackType_t));
    usb_thread = osThreadCreate(osThread(usb_server_thread_def), nullptr);
}
