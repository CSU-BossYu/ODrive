#include "interface_usb.h"

#include <atomic>
#include <cmsis_os.h>
#include <freertos_vars.h>
#include <FreeRTOS.h>
#include <task.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include <usb_device.h>

osThreadId usb_thread;
// Covers the fixed 512-byte protocol payload, parser and scope encoder call
// paths without dynamic allocation.
const uint32_t stack_size_usb_thread = 4096; // Bytes
USBStats_t usb_stats_;

namespace {

// Never submit an exact full-speed max-packet transfer here. The customized
// STM32 CDC class closes such a transfer with a second, zero-length packet and
// keeps CDC_Tx.State busy until that second interrupt. A lost/coalesced ZLP
// completion would stall the only TX owner forever (CAPABILITIES is exactly
// 64 bytes). Splitting into at most 63-byte chunks avoids that driver state
// altogether while preserving arbitrary protocol-frame fragmentation.
static_assert(USB_TX_DATA_SIZE > 1u, "CDC TX buffer must hold a payload byte");
constexpr uint16_t kUsbTxChunkSize = USB_TX_DATA_SIZE - 1u;
size_t active_length = 0;
std::atomic<bool> connected{false};
bool tx_active = false;

void drain_rx_ingress() {
    uint8_t packet[USB_RX_DATA_SIZE];
    size_t length = 0;
    while ((length = usb_cdc_ingress_read(packet, sizeof(packet))) != 0u) {
        // This compatibility entry point now runs exclusively in USB task
        // context. The CDC interrupt only writes the bounded C ingress ring.
        usb_debug_receive_bytes(packet, length);
    }
}

void reset_transport() {
    active_length = 0;
    tx_active = false;
}

void complete_active_packet() {
    if (!tx_active) {
        return;
    }
    usb_debug_complete_tx_chunk(static_cast<uint16_t>(active_length));
    active_length = 0;
    tx_active = false;
}

void reconcile_tx_completion() {
    if (!tx_active || hUsbDeviceFS.pClassData == nullptr) {
        return;
    }
    const auto* hcdc = static_cast<const USBD_CDC_HandleTypeDef*>(
        hUsbDeviceFS.pClassData);
    if (hcdc->CDC_Tx.State == 0u) {
        // TX-complete queue events are only wake-up hints and may be coalesced
        // or dropped. The CDC state is authoritative for transfer completion.
        complete_active_packet();
    }
}

void start_next_packet() {
    if (!connected.load(std::memory_order_acquire)) {
        return;
    }

    reconcile_tx_completion();
    if (tx_active) {
        return;
    }

    // `connected` is the single physical-link authority. The configured event
    // can precede construction of this task, so continuously project the
    // authoritative state into the protocol transport before servicing RX/TX
    // instead of relying on one event to keep two state machines synchronized.
    usb_debug_notify_connected(true);

    // CDC is a binary protocol transport from the moment it is configured.
    // There is no legacy stdout stream and therefore no text/binary migration
    // boundary or active-transfer handoff during HELLO.
    drain_rx_ingress();
    usb_debug_service();
    const uint8_t* binary_data = nullptr;
    uint16_t binary_length = 0;
    if (usb_debug_begin_tx_chunk(kUsbTxChunkSize,
                                 &binary_data, &binary_length)) {
        if (CDC_Transmit_FS(const_cast<uint8_t*>(binary_data), binary_length,
                            CDC_IN_EP) == USBD_OK) {
            tx_active = true;
            active_length = binary_length;
            ++usb_stats_.tx_cnt;
        } else {
            ++usb_stats_.tx_overrun_cnt;
        }
        return;
    }
}

void usb_server_thread(void*) {
    for (;;) {
        osEvent event = osMessageGet(usb_event_queue, 1);

        if (event.status == osEventMessage) switch (event.value.v) {
            case 1: // USB configured
                connected.store(true, std::memory_order_release);
                usb_debug_notify_connected(true);
                break;
            case 2: // USB disconnected
                connected.store(false, std::memory_order_release);
                usb_debug_notify_connected(false);
                reset_transport();
                // Preserve strict SPSC ownership of the C ingress indices:
                // the IRQ only advances write and this task only advances
                // read. Move any final packet across, then service() discards
                // it because the physical link is already disconnected.
                drain_rx_ingress();
                usb_debug_service();
                break;
            case 3: // CDC TX complete
                // Wake-up hint only. A queued event may refer to an older
                // transfer, so it must never complete the current descriptor.
                // start_next_packet() reconciles against CDC_Tx.State instead.
                break;
            case 8: { // binary RX data queued
                break;
            }
            default:
                break;
        }
        start_next_packet();
    }
}

} // namespace

size_t usb_stdout_write(const uint8_t* data, size_t length) {
    // stdout is deliberately not multiplexed onto the binary CDC stream.
    // Structured LOG_RECORD/FAULT_EVENT/STATE_EVENT frames are the only USB
    // diagnostics. Report the bytes consumed so libc callers never block.
    if (data != nullptr && length != 0u) {
        ++usb_stats_.tx_overrun_cnt;
    }
    return data == nullptr ? 0u : length;
}

bool usb_stdout_is_connected() {
    return connected.load(std::memory_order_acquire);
}

void start_usb_server() {
    reset_transport();
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0,
                stack_size_usb_thread / sizeof(StackType_t));
    usb_thread = osThreadCreate(osThread(usb_server_thread_def), nullptr);
}
