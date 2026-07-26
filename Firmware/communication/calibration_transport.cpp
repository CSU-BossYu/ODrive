#include "calibration_transport.hpp"

#include <array>
#include <cstddef>
#include <cstring>

#include <cmsis_os.h>

#include "interface_usb.h"
#include "MotorControl/odrive_main.h"

CalibrationTransportStats calibration_transport_stats;

namespace {

constexpr uint32_t kFrameMagic = 0x5243444Fu; // "ODCR" little-endian
constexpr uint16_t kFrameSchema = 1;

#pragma pack(push, 1)
struct CalibrationStreamHeaderV1 {
    uint32_t magic;
    uint16_t schema;
    uint16_t header_size;
    uint16_t axis;
    uint16_t record_type;
    uint16_t payload_size;
    uint16_t flags;
    uint32_t sequence;
    uint32_t payload_crc32;
    uint32_t header_crc32;
};
#pragma pack(pop)

static_assert(sizeof(CalibrationStreamHeaderV1) == 28,
              "calibration stream header layout changed");

uint32_t crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

void calibration_transport_task(void*) {
    using Buffer = CalibrationRecordBuffer<32>;
    constexpr size_t kMaxFrameSize =
        sizeof(CalibrationStreamHeaderV1) + Buffer::kMaxPayloadSize;
    std::array<uint8_t, kMaxFrameSize> frame = {};
    Buffer::Slot pending = {};
    bool has_pending = false;

    for (;;) {
        Axis& axis = odrv.get_axis(0);
        if (!usb_stdout_is_connected()) {
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

        uint32_t sequence = 0;
        if (pending.payload_size >= 12) {
            std::memcpy(&sequence, pending.payload.data() + 8,
                        sizeof(sequence));
        }

        CalibrationStreamHeaderV1 header = {};
        header.magic = kFrameMagic;
        header.schema = kFrameSchema;
        header.header_size = sizeof(header);
        header.axis = static_cast<uint16_t>(axis.axis_num_);
        header.record_type = pending.record_type;
        header.payload_size = pending.payload_size;
        header.sequence = sequence;
        header.payload_crc32 = crc32(pending.payload.data(),
                                     pending.payload_size);
        header.header_crc32 = crc32(
            reinterpret_cast<const uint8_t*>(&header),
            offsetof(CalibrationStreamHeaderV1, header_crc32));

        std::memcpy(frame.data(), &header, sizeof(header));
        std::memcpy(frame.data() + sizeof(header), pending.payload.data(),
                    pending.payload_size);
        const size_t frame_size = sizeof(header) + pending.payload_size;

        if (usb_stdout_write_frame(frame.data(), frame_size)) {
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
