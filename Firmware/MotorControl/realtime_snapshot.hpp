#ifndef ODRIVE_REALTIME_SNAPSHOT_HPP
#define ODRIVE_REALTIME_SNAPSHOT_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace odrive::trace {

template <typename T>
class FixedTripleBuffer {
public:
    static constexpr uint32_t kSlotCount = 3u;
    static constexpr uint32_t kNoSlot = kSlotCount;
    static constexpr size_t kReadAttempts = 3u;

    void publish(const T& value) {
        const uint32_t active = active_index_.load(std::memory_order_seq_cst);
        const uint32_t reader = reader_index_.load(std::memory_order_seq_cst);
        uint32_t target = 0u;
        while (target == active || target == reader) {
            ++target;
        }
        slots_[target] = value;
        active_index_.store(target, std::memory_order_seq_cst);
    }

    bool read(T* value) const {
        if (value == nullptr) {
            return false;
        }
        for (size_t attempt = 0; attempt < kReadAttempts; ++attempt) {
            const uint32_t active = active_index_.load(std::memory_order_seq_cst);
            if (active == kNoSlot) {
                return false;
            }
            reader_index_.store(active, std::memory_order_seq_cst);
            if (active != active_index_.load(std::memory_order_seq_cst)) {
                reader_index_.store(kNoSlot, std::memory_order_seq_cst);
                continue;
            }
            *value = slots_[active];
            reader_index_.store(kNoSlot, std::memory_order_seq_cst);
            return true;
        }
        reader_index_.store(kNoSlot, std::memory_order_seq_cst);
        return false;
    }

private:
    std::array<T, kSlotCount> slots_{};
    std::atomic<uint32_t> active_index_{kNoSlot};
    mutable std::atomic<uint32_t> reader_index_{kNoSlot};
};

// All fields are copied as one fixed-size value. No pointer or string is
// allowed in this record because it is published from the control ISR.
struct RealtimeSnapshot {
    uint32_t sequence = 0;
    uint32_t control_sequence = 0;
    uint32_t state_epoch = 0;
    uint32_t encoder_sample_sequence = 0;
    uint32_t feedback_sequence = 0;
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

    uint8_t safety_state = 0;
    uint8_t operation = 0;
    uint8_t readiness_flags = 0;
    uint8_t flags = 0;

    // Complete duration of the preceding control callback, including trace
    // publication and the final error GPIO update.
    uint32_t isr_cycles = 0;
    uint32_t encoder_cycles = 0;
    uint32_t controller_cycles = 0;
    uint32_t motor_cycles = 0;
};

class RealtimeSnapshotBuffer {
public:
    void publish_from_isr(const RealtimeSnapshot& snapshot);
    bool read_consistent(RealtimeSnapshot* snapshot) const;

    uint32_t published_sequence() const {
        return published_sequence_.load(std::memory_order_acquire);
    }

private:
    FixedTripleBuffer<RealtimeSnapshot> buffer_;
    std::atomic<uint32_t> published_sequence_{0};
};

struct SupervisorTraceState {
    uint32_t state_epoch = 0;
    uint8_t safety_state = 0;
    uint8_t operation = 0;
    uint8_t readiness_flags = 0;
    uint8_t reserved = 0;
};

class SupervisorTraceMailbox {
public:
    void publish_from_task(const SupervisorTraceState& state) {
        buffer_.publish(state);
    }
    bool read_from_isr(SupervisorTraceState* state) const {
        return buffer_.read(state);
    }

private:
    FixedTripleBuffer<SupervisorTraceState> buffer_;
};

}  // namespace odrive::trace

#endif
