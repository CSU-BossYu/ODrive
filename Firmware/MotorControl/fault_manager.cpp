#include "fault_manager.hpp"

namespace odrive::fault {

namespace {

uint32_t saturating_increment(uint32_t value) {
    return value == UINT32_MAX ? value : value + 1u;
}

}  // namespace

bool FaultManager::is_clearable(FaultSeverity severity) {
    return severity != FaultSeverity::IMMEDIATE_SHUTDOWN;
}

bool FaultManager::same_fault(const FaultRecord& lhs, const FaultRecord& rhs) {
    return lhs.source == rhs.source &&
           lhs.code == rhs.code &&
           lhs.site == rhs.site &&
           lhs.severity == rhs.severity &&
           lhs.state_epoch == rhs.state_epoch &&
           lhs.parent_fault_sequence == rhs.parent_fault_sequence &&
           lhs.arg0 == rhs.arg0 &&
           lhs.arg1 == rhs.arg1 &&
           lhs.arg2 == rhs.arg2;
}

bool FaultManager::raise(FaultRecord& record) {
    if (record.code == FaultCode::NONE) {
        return false;
    }

    for (size_t i = 0; i < record_count_; ++i) {
        if (!same_fault(records_[i], record)) {
            continue;
        }
        records_[i].occurrence_count =
            saturating_increment(records_[i].occurrence_count);
        records_[i].last_control_sequence = record.control_sequence;
        records_[i].last_timestamp_cycles = record.timestamp_cycles;
        record = records_[i];
        return true;
    }

    if (record_count_ >= kMaxRecords) {
        dropped_count_ = saturating_increment(dropped_count_);
        return false;
    }

    // Sequence ownership belongs to the manager. Caller-provided values are
    // deliberately ignored so records cannot collide or forge ancestry.
    record.fault_sequence = next_sequence_++;
    if (next_sequence_ == 0) {
        next_sequence_ = 1;
    }
    record.occurrence_count = 1;
    record.last_control_sequence = record.control_sequence;
    record.last_timestamp_cycles = record.timestamp_cycles;
    records_[record_count_] = record;
    if (first_fault_sequence_ == 0) {
        first_fault_sequence_ = record.fault_sequence;
    }
    ++record_count_;
    return true;
}

bool FaultManager::clear(const ClearRequest& request) {
    bool changed = false;
    size_t write_index = 0;
    for (size_t read_index = 0; read_index < record_count_; ++read_index) {
        const FaultRecord& record = records_[read_index];
        const bool selected = request.fault_sequence == 0 ||
                              request.fault_sequence == record.fault_sequence;
        if (selected && is_clearable(record.severity)) {
            changed = true;
            continue;
        }
        if (write_index != read_index) {
            records_[write_index] = record;
        }
        ++write_index;
    }
    for (size_t i = write_index; i < record_count_; ++i) {
        records_[i] = {};
    }
    record_count_ = write_index;
    recompute_first_fault();
    return changed;
}

void FaultManager::recompute_first_fault() {
    first_fault_sequence_ = 0;
    for (size_t i = 0; i < record_count_; ++i) {
        if (records_[i].parent_fault_sequence != 0) {
            continue;
        }
        if (first_fault_sequence_ == 0 ||
            records_[i].fault_sequence < first_fault_sequence_) {
            first_fault_sequence_ = records_[i].fault_sequence;
        }
    }
    if (first_fault_sequence_ == 0 && record_count_ != 0) {
        first_fault_sequence_ = records_[0].fault_sequence;
    }
}

bool FaultManager::first_fault(FaultRecord* out) const {
    if (out == nullptr || first_fault_sequence_ == 0) {
        return false;
    }
    for (size_t i = 0; i < record_count_; ++i) {
        if (records_[i].fault_sequence == first_fault_sequence_) {
            *out = records_[i];
            return true;
        }
    }
    return false;
}

bool FaultManager::record_at(size_t index, FaultRecord* out, bool* active) const {
    if (index >= record_count_ || out == nullptr || active == nullptr) {
        return false;
    }
    *out = records_[index];
    *active = true;
    return true;
}

}  // namespace odrive::fault
