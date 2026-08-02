#include "trace_rings.hpp"

#include <algorithm>
#include <type_traits>

namespace odrive::trace {

static_assert(std::is_trivially_copyable<TraceDispatchRecord>::value,
              "trace dispatch records must remain fixed-size");

bool CriticalBlackBox::record_from_isr(const CriticalEvent& event) {
    const bool is_frozen = frozen_.load(std::memory_order_acquire);
    if (is_frozen) {
        uint32_t remaining = post_trigger_remaining_.load(
            std::memory_order_relaxed);
        while (remaining != 0u &&
               !post_trigger_remaining_.compare_exchange_weak(
                   remaining, remaining - 1u,
                   std::memory_order_acq_rel,
                   std::memory_order_relaxed)) {}
        if (remaining == 0u) {
            dropped_count_.fetch_add(1u, std::memory_order_relaxed);
            return false;
        }
    }

    const uint32_t position = write_count_.fetch_add(
        1u, std::memory_order_relaxed);
    events_[position % kCapacity] = event;
    return true;
}

bool CriticalBlackBox::freeze_on_first_fault() {
    bool expected = false;
    if (!frozen_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        return false;
    }
    post_trigger_remaining_.store(kPostTriggerCapacity,
                                  std::memory_order_release);
    return true;
}

void CriticalBlackBox::reset() {
    frozen_.store(false, std::memory_order_release);
    post_trigger_remaining_.store(0u, std::memory_order_release);
    dropped_count_.store(0u, std::memory_order_release);
    write_count_.store(0u, std::memory_order_release);
}

bool CriticalBlackBox::snapshot(CriticalEvent* output, size_t capacity,
                                size_t* written) const {
    if (output == nullptr || written == nullptr || capacity == 0u) {
        return false;
    }
    for (size_t attempt = 0; attempt < 3u; ++attempt) {
        const uint32_t end = write_count_.load(std::memory_order_acquire);
        const uint32_t count = end < kCapacity ? end : kCapacity;
        const uint32_t start = end - count;
        const uint32_t copy_count = static_cast<uint32_t>(
            std::min<size_t>(count, capacity));
        const uint32_t copy_start = end - copy_count;
        for (uint32_t index = 0; index < copy_count; ++index) {
            output[index] = events_[(copy_start + index) % kCapacity];
        }
        if (end == write_count_.load(std::memory_order_acquire)) {
            *written = copy_count;
            (void)start;
            return true;
        }
    }
    return false;
}

TraceConsumer::TraceConsumer(CriticalEventRing& critical,
                             StateEventRing& state,
                             LogRing& log,
                             ScopeRing& scope,
                             void* callback_context,
                             TraceDispatchFn callback)
    : critical_(critical),
      state_(state),
      log_(log),
      scope_(scope),
      callback_context_(callback_context),
      callback_(callback) {}

bool TraceConsumer::append(const TraceDispatchRecord& record) {
    if (batch_count_ >= kBatchCapacity) {
        ++stats_.deferred_records;
        return false;
    }
    batch_[batch_count_++] = record;
    return true;
}

uint8_t TraceConsumer::priority(TraceEventKind kind) {
    switch (kind) {
        case TraceEventKind::CRITICAL: return 0u;
        case TraceEventKind::STATE: return 1u;
        case TraceEventKind::LOG: return 2u;
        case TraceEventKind::SCOPE: return 3u;
    }
    return 3u;
}

bool TraceConsumer::precedes(const TraceDispatchRecord& lhs,
                             const TraceDispatchRecord& rhs) {
    const uint8_t lhs_priority = priority(lhs.kind);
    const uint8_t rhs_priority = priority(rhs.kind);
    if (lhs_priority != rhs_priority) {
        return lhs_priority < rhs_priority;
    }
    return static_cast<int32_t>(lhs.sequence - rhs.sequence) < 0;
}

void TraceConsumer::sort_batch() {
    for (size_t index = 1; index < batch_count_; ++index) {
        const TraceDispatchRecord value = batch_[index];
        size_t position = index;
        while (position != 0u && precedes(value, batch_[position - 1u])) {
            batch_[position] = batch_[position - 1u];
            --position;
        }
        batch_[position] = value;
    }
}

void TraceConsumer::dispatch_batch() {
    for (size_t index = 0; index < batch_count_; ++index) {
        const TraceDispatchRecord& record = batch_[index];
        if (have_last_sequence_ && record.kind == last_kind_ &&
            static_cast<int32_t>(record.sequence - last_sequence_) < 0) {
            ++stats_.out_of_order_records;
        }
        last_sequence_ = record.sequence;
        have_last_sequence_ = true;
        last_kind_ = record.kind;
        switch (record.kind) {
            case TraceEventKind::CRITICAL: ++stats_.dispatched_critical; break;
            case TraceEventKind::STATE: ++stats_.dispatched_state; break;
            case TraceEventKind::LOG: ++stats_.dispatched_log; break;
            case TraceEventKind::SCOPE: ++stats_.dispatched_scope; break;
        }
        if (callback_ != nullptr) {
            callback_(callback_context_, record);
        }
    }
}

void TraceConsumer::consume() {
    batch_count_ = 0u;
    stats_.dropped_critical = critical_.dropped_count();
    stats_.dropped_state = state_.dropped_count();
    stats_.dropped_log = log_.dropped_count();
    stats_.dropped_scope = scope_.dropped_count();
    const bool critical_pressure =
        critical_.size() >= kCriticalHighWatermark;
    CriticalEvent critical;
    while (batch_count_ < kBatchCapacity && critical_.pop(&critical)) {
        TraceDispatchRecord record;
        record.kind = TraceEventKind::CRITICAL;
        record.sequence = critical.sequence;
        record.payload.critical = critical;
        append(record);
    }

    if (critical_pressure) {
        stats_.scope_degraded = 1u;
        LogRecord dropped_log;
        ScopeRecord dropped_scope;
        while (log_.pop(&dropped_log)) {
            ++stats_.degraded_records;
        }
        while (scope_.pop(&dropped_scope)) {
            ++stats_.degraded_records;
        }
    }

    StateEvent state;
    while (batch_count_ < kBatchCapacity && state_.pop(&state)) {
        TraceDispatchRecord record;
        record.kind = TraceEventKind::STATE;
        record.sequence = state.sequence;
        record.payload.state = state;
        append(record);
    }
    LogRecord log;
    while (!critical_pressure && batch_count_ < kBatchCapacity &&
           log_.pop(&log)) {
        TraceDispatchRecord record;
        record.kind = TraceEventKind::LOG;
        record.sequence = log.sequence;
        record.payload.log = log;
        append(record);
    }
    ScopeRecord scope;
    while (!critical_pressure && batch_count_ < kBatchCapacity &&
           scope_.pop(&scope)) {
        TraceDispatchRecord record;
        record.kind = TraceEventKind::SCOPE;
        record.sequence = scope.sequence;
        record.payload.scope = scope;
        append(record);
    }

    sort_batch();
    dispatch_batch();
    batch_count_ = 0u;
}

}  // namespace odrive::trace
