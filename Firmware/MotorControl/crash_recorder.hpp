#ifndef ODRIVE_CRASH_RECORDER_HPP
#define ODRIVE_CRASH_RECORDER_HPP

#include <cstddef>
#include <cstdint>

#include "crash_recorder_c.h"

namespace odrive::crash {

constexpr uint32_t kRecordMagic = 0x43525348u;  // "CRSH"
constexpr uint32_t kRecordVersion = 1u;
constexpr size_t kCriticalEventCount = 2u;
constexpr size_t kEncodedPayloadSize = 204u;

enum class FaultKind : uint32_t {
    HARD_FAULT = 1u,
    MEM_MANAGE = 2u,
    BUS_FAULT = 3u,
    USAGE_FAULT = 4u,
};

enum RecordFlags : uint32_t {
    STACK_FRAME_VALID = 1u << 0,
    CONTEXT_VALID = 1u << 1,
    CRITICAL_EVENTS_VALID = 1u << 2,
    EXTENDED_FP_FRAME = 1u << 3,
};

// Encodes the pending retained record with explicit little-endian fields.
// Returns zero when there is no valid record or the output is too small.
size_t encode_pending(uint8_t* output, size_t capacity,
                      uint32_t* record_crc = nullptr);
bool pending();
bool pending_crc(uint32_t record_crc);
bool clear_if_crc(uint32_t record_crc);

}  // namespace odrive::crash

#endif
