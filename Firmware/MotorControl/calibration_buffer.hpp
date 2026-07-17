#ifndef __CALIBRATION_BUFFER_HPP
#define __CALIBRATION_BUFFER_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "calibration_record.hpp"

// Fixed-memory SPSC queue between the control-loop producer and a lower-rate
// calibration transport/fit consumer. Push never blocks and never overwrites
// unread data. An overflow is recorded and marked on the next accepted sample,
// so an offline fitter cannot silently treat a discontinuity as contiguous.
template <size_t Capacity>
class CalibrationRecordBuffer {
    static_assert(Capacity >= 2, "calibration buffer needs at least two slots");

public:
    static constexpr uint16_t kMaxPayloadSize = sizeof(CalibrationSampleV1);

    struct Slot {
        uint16_t record_type = 0;
        uint16_t payload_size = 0;
        std::array<uint8_t, kMaxPayloadSize> payload = {};
    };

    template <typename Record>
    bool push_from_isr(CalibrationRecordType record_type, const Record& source) {
        static_assert(std::is_trivially_copyable<Record>::value,
                      "calibration records must be trivially copyable");
        static_assert(sizeof(Record) <= kMaxPayloadSize,
                      "calibration record exceeds slot payload");

        const uint32_t write = write_count_.load(std::memory_order_relaxed);
        const uint32_t read = read_count_.load(std::memory_order_acquire);
        if ((write - read) >= Capacity) {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
            gap_pending_.store(true, std::memory_order_relaxed);
            return false;
        }

        Record record = source;
        if (gap_pending_.exchange(false, std::memory_order_relaxed)) {
            record.flags |= CAL_SAMPLE_DROPPED_BEFORE;
        }

        Slot& slot = slots_[write % Capacity];
        slot.record_type = static_cast<uint16_t>(record_type);
        slot.payload_size = sizeof(Record);
        std::memcpy(slot.payload.data(), &record, sizeof(Record));
        write_count_.store(write + 1, std::memory_order_release);
        update_high_watermark((write + 1) - read);
        return true;
    }

    bool pop(Slot* destination) {
        if (destination == nullptr) {
            return false;
        }
        const uint32_t read = read_count_.load(std::memory_order_relaxed);
        const uint32_t write = write_count_.load(std::memory_order_acquire);
        if (read == write) {
            return false;
        }
        *destination = slots_[read % Capacity];
        read_count_.store(read + 1, std::memory_order_release);
        return true;
    }

    // Only call while capture and consumption are stopped.
    void reset() {
        read_count_.store(0, std::memory_order_relaxed);
        write_count_.store(0, std::memory_order_relaxed);
        dropped_count_.store(0, std::memory_order_relaxed);
        high_watermark_.store(0, std::memory_order_relaxed);
        gap_pending_.store(false, std::memory_order_relaxed);
    }

    uint32_t size() const {
        const uint32_t write = write_count_.load(std::memory_order_acquire);
        const uint32_t read = read_count_.load(std::memory_order_acquire);
        return write - read;
    }
    uint32_t dropped_count() const {
        return dropped_count_.load(std::memory_order_relaxed);
    }
    uint32_t high_watermark() const {
        return high_watermark_.load(std::memory_order_relaxed);
    }
    static constexpr uint32_t capacity() { return Capacity; }

private:
    void update_high_watermark(uint32_t value) {
        uint32_t previous = high_watermark_.load(std::memory_order_relaxed);
        while (previous < value &&
               !high_watermark_.compare_exchange_weak(
                   previous, value,
                   std::memory_order_relaxed,
                   std::memory_order_relaxed)) {
        }
    }

    std::array<Slot, Capacity> slots_ = {};
    std::atomic<uint32_t> write_count_{0};
    std::atomic<uint32_t> read_count_{0};
    std::atomic<uint32_t> dropped_count_{0};
    std::atomic<uint32_t> high_watermark_{0};
    std::atomic<bool> gap_pending_{false};
};

#endif // __CALIBRATION_BUFFER_HPP
