#include "calibration_transport.hpp"

#include <cstddef>

#include <cmsis_os.h>

#include "interface_usb.h"
#include "MotorControl/odrive_main.h"

CalibrationTransportStats calibration_transport_stats;

namespace {

void calibration_transport_task(void*) {
    using Buffer = CalibrationRecordBuffer<32>;
    Buffer::Slot pending = {};
    bool has_pending = false;

    for (;;) {
        Axis& axis = odrv.get_axis(0);
        if (!usb_stdout_is_connected() ||
            !usb_debug_binary_session_active()) {
            ++calibration_transport_stats.disconnect_waits;
            if (has_pending) {
                has_pending = false;
                ++calibration_transport_stats.records_discarded_disconnected;
            }
            // Raw streaming is optional. Drain transport copies while no host
            // is listening so a CAN-only calibration cannot fail from queue
            // backpressure. The online fitter consumes samples synchronously.
            for (uint32_t i = 0; i < 16 && axis.calibration_record_buffer_.pop(&pending);
                 ++i) {
                ++calibration_transport_stats.records_discarded_disconnected;
            }
            osDelay(1);
            continue;
        }

        if (!has_pending) {
            if (!axis.calibration_record_buffer_.pop(&pending)) {
                osDelay(1);
                continue;
            }
            has_pending = true;
        }

        uint32_t sequence = 0u;
        if (pending.payload_size >= 12u) {
            sequence = static_cast<uint32_t>(pending.payload[8]) |
                static_cast<uint32_t>(pending.payload[9]) << 8u |
                static_cast<uint32_t>(pending.payload[10]) << 16u |
                static_cast<uint32_t>(pending.payload[11]) << 24u;
        }

        if (usb_debug_publish_calibration(
                pending.record_type, sequence, pending.payload.data(),
                pending.payload_size)) {
            ++calibration_transport_stats.frames_sent;
            has_pending = false;
        } else {
            ++calibration_transport_stats.queue_retries;
            osDelay(1);
        }
    }
}

} // namespace

void start_calibration_transport() {
    constexpr uint32_t kStackSize = 1536;
    osThreadDef(calibration_transport_thread, calibration_transport_task,
                osPriorityAboveNormal, 0,
                kStackSize / sizeof(StackType_t));
    osThreadCreate(osThread(calibration_transport_thread), nullptr);
}
