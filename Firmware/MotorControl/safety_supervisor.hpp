#ifndef ODRIVE_SAFETY_SUPERVISOR_HPP
#define ODRIVE_SAFETY_SUPERVISOR_HPP

#include <array>
#include <cstdint>

#include "fault_manager.hpp"
#include "realtime_event_ring.hpp"

namespace odrive::safety {

enum class SafetyState : uint8_t {
    BOOT = 0,
    SAFE_OFF = 1,
    PREPARING = 2,
    READY = 3,
    ARMED = 4,
    STOPPING = 5,
    FAULT_LATCHED = 6,
};

enum class Operation : uint8_t {
    NONE = 0,
    CLOSED_LOOP = 1,
    CALIBRATION = 2,
    SELF_TEST = 3,
};

enum class CommandSource : uint8_t {
    INTERNAL = 0,
    CAN = 1,
    USB = 2,
};

enum class CommandType : uint8_t {
    BOOT_COMPLETE = 0,
    SET_OPERATION = 1,
    ARM = 2,
    DISARM = 3,
    CLEAR_FAULTS = 4,
    RESET_ESTIMATOR = 5,
    RESTART_CALIBRATION = 6,
    LEGACY_AXIS_STATE = 7,
};

enum class CommandStatus : uint8_t {
    ACCEPTED = 0,
    REJECTED = 1,
    COMPLETED = 2,
    FAILED = 3,
};

enum class RealtimeRequestType : uint8_t {
    PREPARE_OPERATION = 0,
    ARM = 1,
    DISARM = 2,
};

struct Command {
    uint32_t request_id = 0;
    CommandSource source = CommandSource::INTERNAL;
    CommandType type = CommandType::BOOT_COMPLETE;
    Operation operation = Operation::NONE;
    uint32_t arg0 = 0;
};

struct CommandResult {
    uint32_t request_id = 0;
    CommandStatus status = CommandStatus::REJECTED;
    uint32_t state_epoch = 0;
    odrive::fault::FaultCode reason = odrive::fault::FaultCode::NONE;
    CommandSource source = CommandSource::INTERNAL;
};

struct ReadinessSnapshot {
    uint32_t epoch = 0;
    uint32_t sequence = 0;
    uint32_t feedback_sequence = 0;
    uint8_t flags = 0;
};

struct RealtimeRequest {
    uint32_t request_id = 0;
    uint32_t epoch = 0;
    RealtimeRequestType type = RealtimeRequestType::PREPARE_OPERATION;
    Operation operation = Operation::NONE;
};

constexpr uint8_t kRequiredReadinessFlags =
    READINESS_ENCODER |
    READINESS_PHASE |
    READINESS_CURRENT |
    READINESS_CONTROLLER |
    READINESS_POWER_STAGE;

using FaultRaiseFn = void (*)(void* context,
                              odrive::fault::FaultRecord& record,
                              uint32_t projection,
                              bool record_trace);
using FaultClearFn = bool (*)(void* context,
                              const odrive::fault::ClearRequest& request,
                              bool* empty_after_clear);
using FaultFirstFn = bool (*)(void* context,
                              odrive::fault::FaultRecord* record);

class SafetySupervisor {
public:
    static constexpr size_t kCommandCapacity = 8;
    static constexpr size_t kResultCapacity = 16;
    static constexpr size_t kRequestCapacity = 16;

    SafetySupervisor(odrive::fault::FaultManager& fault_manager,
                     RealtimeEventRing& realtime_events,
                     void* fault_context = nullptr,
                     FaultRaiseFn fault_raise = nullptr,
                     FaultClearFn fault_clear = nullptr,
                     FaultFirstFn fault_first = nullptr);

    bool submit_command(const Command& command);
    bool pop_result(CommandResult* result);
    bool pop_legacy_axis_state(uint32_t* axis_state);
    bool pop_realtime_request(RealtimeRequest* request);
    void handle_realtime_event(const RealtimeEvent& event);
    void handle_external_clear();

    // Called only by the Supervisor task. It performs bounded work: at most
    // one complete pass over each fixed-capacity input queue.
    void process();
    void tick(uint32_t control_sequence);

    SafetyState state() const { return state_; }
    Operation operation() const { return operation_; }
    uint32_t state_epoch() const { return state_epoch_; }
    ReadinessSnapshot readiness() const { return readiness_; }
    uint32_t dropped_commands() const { return command_ring_.overflow_count(); }
    uint32_t dropped_results() const { return result_ring_.overflow_count(); }

    bool can_run_controller() const;
    bool can_enable_pwm() const;
    bool can_modify_config() const;

private:
    enum class TransitionEvent : uint8_t {
        BOOT_COMPLETE = 0,
        SET_OPERATION = 1,
        READY_SNAPSHOT = 2,
        ARM_REQUEST = 3,
        ARM_CONFIRMED = 4,
        DISARM_REQUEST = 5,
        DISARM_CONFIRMED = 6,
        CLEAR_FAULTS = 7,
        FAULT = 8,
        PREPARE_REJECTED = 9,
    };

    enum class TransitionAction : uint8_t {
        NONE = 0,
        ENTER_PREPARING = 1,
        ENTER_READY = 2,
        ENTER_ARMED = 3,
        ENTER_STOPPING = 4,
        ENTER_SAFE_OFF = 5,
        ENTER_FAULT = 6,
    };

    struct TransitionRule {
        SafetyState from;
        TransitionEvent event;
        SafetyState to;
        TransitionAction exit_action;
        TransitionAction entry_action;
        uint32_t timeout_sequences;
        odrive::fault::FaultCode failure_reason;
    };

    static const std::array<TransitionRule, 23> kTransitions;

    bool apply_transition(TransitionEvent event);
    const TransitionRule* find_transition(TransitionEvent event) const;
    void process_command(const Command& command);
    void process_event(const RealtimeEvent& event);
    void process_fault_event(const RealtimeEvent& event);
    void process_ready_event(const RealtimeEvent& event);
    void process_arm_event(const RealtimeEvent& event);
    void process_disarm_event(const RealtimeEvent& event);
    void process_prepare_rejected_event(const RealtimeEvent& event);
    void emit_result(const Command& command, CommandStatus status,
                     odrive::fault::FaultCode reason =
                         odrive::fault::FaultCode::NONE);
    void emit_result(uint32_t request_id, CommandSource source,
                     CommandStatus status,
                     odrive::fault::FaultCode reason =
                         odrive::fault::FaultCode::NONE);
    void reject(const Command& command, odrive::fault::FaultCode reason);
    void fail_pending(odrive::fault::FaultCode reason);
    void raise_timeout_fault();
    void synchronize_fault_manager();
    bool valid_legacy_axis_state(uint32_t axis_state) const;
    void arm_transition_deadline(uint32_t timeout_sequences);
    static uint32_t next_epoch(uint32_t epoch);

    odrive::fault::FaultManager& fault_manager_;
    RealtimeEventRing& realtime_events_;
    void* fault_context_ = nullptr;
    FaultRaiseFn fault_raise_ = nullptr;
    FaultClearFn fault_clear_ = nullptr;
    FaultFirstFn fault_first_ = nullptr;
    FixedSpscRing<Command, kCommandCapacity> command_ring_;
    FixedSpscRing<CommandResult, kResultCapacity> result_ring_;
    FixedSpscRing<uint32_t, kCommandCapacity> legacy_axis_state_ring_;
    FixedSpscRing<RealtimeRequest, kRequestCapacity> realtime_request_ring_;

    SafetyState state_ = SafetyState::BOOT;
    Operation operation_ = Operation::NONE;
    uint32_t state_epoch_ = 0;
    uint32_t state_enter_sequence_ = 0;
    uint32_t transition_deadline_ = 0;
    uint32_t transition_timeout_sequences_ = 0;
    bool transition_deadline_active_ = false;
    ReadinessSnapshot readiness_{};
    uint32_t pending_operation_request_ = 0;
    CommandSource pending_operation_source_ = CommandSource::INTERNAL;
    uint32_t pending_arm_request_ = 0;
    CommandSource pending_arm_source_ = CommandSource::INTERNAL;
    uint32_t pending_disarm_request_ = 0;
    CommandSource pending_disarm_source_ = CommandSource::INTERNAL;
    uint32_t legacy_auto_arm_request_ = 0;
    uint32_t observed_realtime_overflow_ = 0;
};

}  // namespace odrive::safety

#endif
