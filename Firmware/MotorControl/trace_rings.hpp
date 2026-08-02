#ifndef ODRIVE_TRACE_RINGS_HPP
#define ODRIVE_TRACE_RINGS_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "fault_manager.hpp"
#include "realtime_event_ring.hpp"

namespace odrive::trace {

enum class TraceEventKind : uint8_t {
    CRITICAL = 0,
    STATE = 1,
    LOG = 2,
    SCOPE = 3,
};

enum class CriticalEventType : uint8_t {
    FAULT = 0,
    SAFETY_STOP = 1,
    OVERFLOW = 2,
};

enum class StateEventType : uint8_t {
    TRANSITION = 0,
    COMMAND_RESULT = 1,
    READINESS = 2,
};

enum class LogLevel : uint8_t {
    INFO = 0,
    WARNING = 1,
    ERROR = 2,
};

struct CriticalEvent {
    uint32_t sequence = 0;
    uint32_t control_sequence = 0;
    uint32_t state_epoch = 0;
    uint32_t timestamp_cycles = 0;
    uint32_t fault_sequence = 0;
    CriticalEventType type = CriticalEventType::FAULT;
    uint8_t reserved0 = 0;
    uint16_t reserved1 = 0;
    odrive::fault::FaultSource source = odrive::fault::FaultSource::ISR;
    odrive::fault::FaultCode code = odrive::fault::FaultCode::NONE;
    odrive::fault::FaultSite site = odrive::fault::FaultSite::UNKNOWN;
    odrive::fault::FaultSeverity severity = odrive::fault::FaultSeverity::INFO;
    uint32_t parent_fault_sequence = 0;
    uint32_t arg0 = 0;
    uint32_t arg1 = 0;
    uint32_t arg2 = 0;
};

struct StateEvent {
    uint32_t sequence = 0;
    uint32_t state_epoch = 0;
    uint32_t timestamp_cycles = 0;
    uint32_t request_id = 0;
    uint8_t state = 0;
    uint8_t operation = 0;
    uint8_t axis_state = 0;
    StateEventType type = StateEventType::TRANSITION;
    odrive::fault::FaultCode reason = odrive::fault::FaultCode::NONE;
};

struct LogRecord {
    uint32_t sequence = 0;
    uint32_t timestamp_cycles = 0;
    uint32_t arg0 = 0;
    uint32_t arg1 = 0;
    uint16_t code = 0;
    LogLevel level = LogLevel::INFO;
    uint8_t category = 0;
};

struct ScopeRecord {
    uint32_t sequence = 0;
    uint32_t control_sequence = 0;
    uint32_t state_epoch = 0;
    uint32_t timestamp_cycles = 0;
    float phase = 0.0f;
    float phase_velocity = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float id_measured = 0.0f;
    float iq_measured = 0.0f;
    float id_setpoint = 0.0f;
    float iq_setpoint = 0.0f;
    float torque_setpoint = 0.0f;
    float controller_output = 0.0f;
};

static_assert(std::is_trivially_copyable<CriticalEvent>::value,
              "CriticalEvent must be a fixed-size record");
static_assert(std::is_trivially_copyable<StateEvent>::value,
              "StateEvent must be a fixed-size record");
static_assert(std::is_trivially_copyable<LogRecord>::value,
              "LogRecord must be a fixed-size record");
static_assert(std::is_trivially_copyable<ScopeRecord>::value,
              "ScopeRecord must be a fixed-size record");

// A bounded MPSC ring is used only for critical records because a fault can
// originate in either an ISR or a task callback. It never allocates and never
// waits. The other rings use the smaller SPSC primitive from Phase 2.
template <typename T, size_t Capacity>
class FixedMpscRing {
    static_assert(Capacity >= 2, "MPSC ring capacity must be at least two");

    struct Cell {
        std::atomic<uint32_t> sequence{0};
        T value{};
    };

public:
    FixedMpscRing() {
        for (uint32_t index = 0; index < Capacity; ++index) {
            cells_[index].sequence.store(index, std::memory_order_relaxed);
        }
    }

    bool push(const T& value) {
        uint32_t position = enqueue_position_.load(std::memory_order_relaxed);
        for (uint32_t attempt = 0; attempt < 3u; ++attempt) {
            Cell& cell = cells_[position % Capacity];
            const uint32_t sequence = cell.sequence.load(
                std::memory_order_acquire);
            const int32_t difference = static_cast<int32_t>(sequence - position);
            if (difference == 0) {
                if (enqueue_position_.compare_exchange_weak(
                        position, position + 1u,
                        std::memory_order_relaxed,
                        std::memory_order_relaxed)) {
                    cell.value = value;
                    cell.sequence.store(position + 1u,
                                        std::memory_order_release);
                    return true;
                }
            } else if (difference < 0) {
                overflow_count_.fetch_add(1u, std::memory_order_relaxed);
                return false;
            } else {
                position = enqueue_position_.load(std::memory_order_relaxed);
            }
        }
        overflow_count_.fetch_add(1u, std::memory_order_relaxed);
        return false;
    }

    bool pop(T* value) {
        if (value == nullptr) {
            return false;
        }
        uint32_t position = dequeue_position_.load(std::memory_order_relaxed);
        for (;;) {
            Cell& cell = cells_[position % Capacity];
            const uint32_t sequence = cell.sequence.load(
                std::memory_order_acquire);
            const int32_t difference =
                static_cast<int32_t>(sequence - (position + 1u));
            if (difference == 0) {
                if (dequeue_position_.compare_exchange_weak(
                        position, position + 1u,
                        std::memory_order_relaxed,
                        std::memory_order_relaxed)) {
                    *value = cell.value;
                    cell.sequence.store(position + Capacity,
                                        std::memory_order_release);
                    return true;
                }
            } else if (difference < 0) {
                return false;
            } else {
                position = dequeue_position_.load(std::memory_order_relaxed);
            }
        }
    }

    size_t size() const {
        const uint32_t enqueue = enqueue_position_.load(
            std::memory_order_acquire);
        const uint32_t dequeue = dequeue_position_.load(
            std::memory_order_acquire);
        return static_cast<size_t>(enqueue - dequeue);
    }

    uint32_t overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

    static constexpr size_t capacity() { return Capacity; }

private:
    std::array<Cell, Capacity> cells_{};
    std::atomic<uint32_t> enqueue_position_{0};
    std::atomic<uint32_t> dequeue_position_{0};
    std::atomic<uint32_t> overflow_count_{0};
};

template <typename T, size_t Capacity>
class TraceSpscRing {
public:
    bool push(const T& value) { return ring_.push(value); }
    bool pop(T* value) { return ring_.pop(value); }
    size_t size() const { return ring_.size(); }
    uint32_t overflow_count() const { return ring_.overflow_count(); }
    uint32_t dropped_count() const { return ring_.overflow_count(); }
    static constexpr size_t capacity() { return ring_type::capacity(); }

private:
    using ring_type = odrive::safety::FixedSpscRing<T, Capacity>;
    ring_type ring_;
};

class CriticalEventRing {
public:
    static constexpr size_t kStorageCapacity = 64;
    bool push_from_isr(const CriticalEvent& event) { return ring_.push(event); }
    bool push_from_task(const CriticalEvent& event) { return ring_.push(event); }
    bool pop(CriticalEvent* event) { return ring_.pop(event); }
    size_t size() const { return ring_.size(); }
    uint32_t overflow_count() const { return ring_.overflow_count(); }
    uint32_t dropped_count() const { return ring_.overflow_count(); }
    static constexpr size_t capacity() { return kStorageCapacity; }

private:
    FixedMpscRing<CriticalEvent, kStorageCapacity> ring_;
};

class StateEventRing : public TraceSpscRing<StateEvent, 65> {};
class LogRing : public TraceSpscRing<LogRecord, 65> {};
class ScopeRing : public TraceSpscRing<ScopeRecord, 257> {};

class CriticalBlackBox {
public:
    static constexpr size_t kCapacity = 32;
    static constexpr size_t kPostTriggerCapacity = 16;

    // The owning Axis serializes record/reset/snapshot against its control ISR.
    // Atomic counters make status observation safe; they do not make the
    // multiword payload independently MPSC-safe.
    bool record_from_isr(const CriticalEvent& event);
    bool record_from_context(const CriticalEvent& event) {
        return record_from_isr(event);
    }
    bool freeze_on_first_fault();
    void reset();
    bool frozen() const { return frozen_.load(std::memory_order_acquire); }
    uint32_t post_trigger_remaining() const {
        return post_trigger_remaining_.load(std::memory_order_acquire);
    }
    uint32_t dropped_count() const {
        return dropped_count_.load(std::memory_order_acquire);
    }
    bool snapshot(CriticalEvent* output, size_t capacity,
                  size_t* written) const;

private:
    std::array<CriticalEvent, kCapacity> events_{};
    std::atomic<uint32_t> write_count_{0};
    std::atomic<bool> frozen_{false};
    std::atomic<uint32_t> post_trigger_remaining_{0};
    std::atomic<uint32_t> dropped_count_{0};
};

struct TraceDispatchRecord {
    TraceEventKind kind = TraceEventKind::LOG;
    uint8_t reserved[3] = {0, 0, 0};
    uint32_t sequence = 0;
    union Payload {
        CriticalEvent critical;
        StateEvent state;
        LogRecord log;
        ScopeRecord scope;
        Payload() {}
    } payload;
};

struct TraceDispatchStats {
    uint32_t dispatched_critical = 0;
    uint32_t dispatched_state = 0;
    uint32_t dispatched_log = 0;
    uint32_t dispatched_scope = 0;
    uint32_t deferred_records = 0;
    uint32_t degraded_records = 0;
    uint32_t out_of_order_records = 0;
    uint32_t dropped_critical = 0;
    uint32_t dropped_state = 0;
    uint32_t dropped_log = 0;
    uint32_t dropped_scope = 0;
    uint8_t scope_degraded = 0;
};

using TraceDispatchFn = void (*)(void* context,
                                  const TraceDispatchRecord& record);

class TraceConsumer {
public:
    static constexpr size_t kBatchCapacity = 64;
    static constexpr size_t kCriticalHighWatermark = 32;

    TraceConsumer(CriticalEventRing& critical,
                  StateEventRing& state,
                  LogRing& log,
                  ScopeRing& scope,
                  void* callback_context = nullptr,
                  TraceDispatchFn callback = nullptr);

    void set_callback(void* callback_context, TraceDispatchFn callback) {
        callback_context_ = callback_context;
        callback_ = callback;
    }
    void consume();
    const TraceDispatchStats& stats() const { return stats_; }

private:
    bool append(const TraceDispatchRecord& record);
    void sort_batch();
    void dispatch_batch();
    static uint8_t priority(TraceEventKind kind);
    static bool precedes(const TraceDispatchRecord& lhs,
                         const TraceDispatchRecord& rhs);

    CriticalEventRing& critical_;
    StateEventRing& state_;
    LogRing& log_;
    ScopeRing& scope_;
    void* callback_context_ = nullptr;
    TraceDispatchFn callback_ = nullptr;
    std::array<TraceDispatchRecord, kBatchCapacity> batch_{};
    size_t batch_count_ = 0;
    TraceDispatchStats stats_{};
    uint32_t last_sequence_ = 0;
    bool have_last_sequence_ = false;
    TraceEventKind last_kind_ = TraceEventKind::LOG;
};

}  // namespace odrive::trace

#endif
