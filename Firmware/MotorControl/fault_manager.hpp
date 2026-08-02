#ifndef ODRIVE_FAULT_MANAGER_HPP
#define ODRIVE_FAULT_MANAGER_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "autogen/fault_codes_generated.hpp"

namespace odrive::fault {

struct FaultRecord {
    uint32_t fault_sequence = 0;
    uint32_t control_sequence = 0;
    uint32_t timestamp_cycles = 0;
    uint32_t state_epoch = 0;
    FaultSource source = FaultSource::AXIS;
    FaultCode code = FaultCode::NONE;
    FaultSite site = FaultSite::UNKNOWN;
    FaultSeverity severity = FaultSeverity::INFO;
    uint32_t parent_fault_sequence = 0;
    uint32_t arg0 = 0;
    uint32_t arg1 = 0;
    uint32_t arg2 = 0;
    uint32_t occurrence_count = 1;
    uint32_t last_control_sequence = 0;
    uint32_t last_timestamp_cycles = 0;
};

struct ClearRequest {
    uint32_t request_id = 0;
    uint32_t fault_sequence = 0;  // zero means all clearable active faults
};

class FaultManager {
public:
    static constexpr size_t kMaxRecords = 32;

    bool raise(FaultRecord& record);
    bool clear(const ClearRequest& request);
    bool first_fault(FaultRecord* out) const;
    bool record_at(size_t index, FaultRecord* out, bool* active) const;
    size_t record_count() const { return record_count_; }
    size_t active_count() const { return record_count_; }
    uint32_t dropped_count() const { return dropped_count_; }

private:
    static bool is_clearable(FaultSeverity severity);
    static bool same_fault(const FaultRecord& lhs, const FaultRecord& rhs);
    void recompute_first_fault();

    std::array<FaultRecord, kMaxRecords> records_{};
    size_t record_count_ = 0;
    uint32_t next_sequence_ = 1;
    uint32_t first_fault_sequence_ = 0;
    uint32_t dropped_count_ = 0;
};

}  // namespace odrive::fault

#endif
