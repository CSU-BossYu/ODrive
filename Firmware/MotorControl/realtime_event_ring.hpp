#ifndef ODRIVE_REALTIME_EVENT_RING_HPP
#define ODRIVE_REALTIME_EVENT_RING_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "fault_manager.hpp"

namespace odrive::safety {

// This is a single-producer/single-consumer queue. The producer is allowed to
// be an ISR and the consumer is a task-context Supervisor. It deliberately
// has no wait, allocation, formatting, USB, or CAN operation.
template <typename T, size_t Capacity>
class FixedSpscRing {
    static_assert(Capacity >= 2, "an SPSC ring needs at least one usable slot");

public:
    bool push(const T& value) {
        const uint32_t write = write_index_.load(std::memory_order_relaxed);
        const uint32_t read = read_index_.load(std::memory_order_acquire);
        const uint32_t next = (write + 1u) % static_cast<uint32_t>(Capacity);
        if (next == read) {
            overflow_count_.fetch_add(1u, std::memory_order_relaxed);
            return false;
        }

        slots_[write] = value;
        write_index_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T* value) {
        if (value == nullptr) {
            return false;
        }

        const uint32_t read = read_index_.load(std::memory_order_relaxed);
        const uint32_t write = write_index_.load(std::memory_order_acquire);
        if (read == write) {
            return false;
        }

        *value = slots_[read];
        read_index_.store(
            (read + 1u) % static_cast<uint32_t>(Capacity),
            std::memory_order_release);
        return true;
    }

    size_t size() const {
        const uint32_t write = write_index_.load(std::memory_order_acquire);
        const uint32_t read = read_index_.load(std::memory_order_acquire);
        return (write + Capacity - read) % Capacity;
    }

    uint32_t overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

    static constexpr size_t capacity() { return Capacity - 1; }

private:
    std::array<T, Capacity> slots_{};
    std::atomic<uint32_t> write_index_{0};
    std::atomic<uint32_t> read_index_{0};
    std::atomic<uint32_t> overflow_count_{0};
};

enum class RealtimeEventType : uint8_t {
    READY_SNAPSHOT = 0,
    ARM_CONFIRMED = 1,
    DISARM_CONFIRMED = 2,
    FAULT = 3,
    PREPARE_REJECTED = 4,
};

enum ReadinessFlags : uint8_t {
    READINESS_ENCODER = 1u << 0,
    READINESS_PHASE = 1u << 1,
    READINESS_CURRENT = 1u << 2,
    READINESS_CONTROLLER = 1u << 3,
    READINESS_POWER_STAGE = 1u << 4,
};

struct RealtimeEvent {
    uint32_t sequence = 0;
    uint32_t epoch = 0;
    uint32_t feedback_sequence = 0;
    uint32_t control_sequence = 0;
    uint32_t timestamp_cycles = 0;
    RealtimeEventType type = RealtimeEventType::FAULT;
    uint8_t readiness_flags = 0;
    uint8_t trace_recorded = 0;
    uint8_t reserved = 0;
    odrive::fault::FaultSource source = odrive::fault::FaultSource::ISR;
    odrive::fault::FaultCode code = odrive::fault::FaultCode::NONE;
    odrive::fault::FaultSite site = odrive::fault::FaultSite::UNKNOWN;
    odrive::fault::FaultSeverity severity = odrive::fault::FaultSeverity::INFO;
    uint32_t parent_fault_sequence = 0;
    uint32_t arg0 = 0;
    uint32_t arg1 = 0;
    uint32_t arg2 = 0;
    uint32_t legacy_projection = 0;
};

class RealtimeEventRing {
public:
    static constexpr size_t kCapacity = 32;

    bool push(const RealtimeEvent& event) { return ring_.push(event); }
    bool pop(RealtimeEvent* event) { return ring_.pop(event); }
    size_t size() const { return ring_.size(); }
    uint32_t overflow_count() const { return ring_.overflow_count(); }
    static constexpr size_t capacity() { return ring_type::capacity(); }

private:
    using ring_type = FixedSpscRing<RealtimeEvent, kCapacity>;
    ring_type ring_;
};

}  // namespace odrive::safety

#endif
