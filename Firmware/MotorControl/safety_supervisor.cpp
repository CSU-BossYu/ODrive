#include "safety_supervisor.hpp"

namespace odrive::safety {

const std::array<SafetySupervisor::TransitionRule, 23>
    SafetySupervisor::kTransitions = {{
        {SafetyState::BOOT, TransitionEvent::BOOT_COMPLETE,
         SafetyState::SAFE_OFF, TransitionAction::NONE,
         TransitionAction::ENTER_SAFE_OFF, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::BOOT, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::SAFE_OFF, TransitionEvent::SET_OPERATION,
         SafetyState::PREPARING, TransitionAction::NONE,
         TransitionAction::ENTER_PREPARING, 20000u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::SAFE_OFF, TransitionEvent::CLEAR_FAULTS,
         SafetyState::SAFE_OFF, TransitionAction::NONE,
         TransitionAction::ENTER_SAFE_OFF, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::SAFE_OFF, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::PREPARING, TransitionEvent::READY_SNAPSHOT,
         SafetyState::READY, TransitionAction::ENTER_PREPARING,
         TransitionAction::ENTER_READY, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::PREPARING, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::PREPARING, TransitionEvent::DISARM_REQUEST,
         SafetyState::STOPPING, TransitionAction::NONE,
         TransitionAction::ENTER_STOPPING, 20000u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::PREPARING, TransitionEvent::PREPARE_REJECTED,
         SafetyState::SAFE_OFF, TransitionAction::ENTER_PREPARING,
         TransitionAction::ENTER_SAFE_OFF, 0u,
         odrive::fault::FaultCode::CONTROLLER_REJECTED},
        {SafetyState::READY, TransitionEvent::SET_OPERATION,
         SafetyState::PREPARING, TransitionAction::ENTER_READY,
         TransitionAction::ENTER_PREPARING, 20000u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::READY, TransitionEvent::ARM_REQUEST,
         SafetyState::READY, TransitionAction::NONE,
         TransitionAction::NONE, 20000u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::READY, TransitionEvent::ARM_CONFIRMED,
         SafetyState::ARMED, TransitionAction::ENTER_READY,
         TransitionAction::ENTER_ARMED, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::READY, TransitionEvent::DISARM_REQUEST,
         SafetyState::STOPPING, TransitionAction::ENTER_READY,
         TransitionAction::ENTER_STOPPING, 20000u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::READY, TransitionEvent::CLEAR_FAULTS,
         SafetyState::SAFE_OFF, TransitionAction::NONE,
         TransitionAction::ENTER_SAFE_OFF, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::READY, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::ARMED, TransitionEvent::DISARM_REQUEST,
         SafetyState::STOPPING, TransitionAction::NONE,
         TransitionAction::ENTER_STOPPING, 20000u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::ARMED, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::STOPPING, TransitionEvent::DISARM_CONFIRMED,
         SafetyState::SAFE_OFF, TransitionAction::ENTER_STOPPING,
         TransitionAction::ENTER_SAFE_OFF, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::STOPPING, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::FAULT_LATCHED, TransitionEvent::CLEAR_FAULTS,
         SafetyState::SAFE_OFF, TransitionAction::ENTER_FAULT,
         TransitionAction::ENTER_SAFE_OFF, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::FAULT_LATCHED, TransitionEvent::FAULT,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::ENTER_FAULT, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::SAFE_OFF, TransitionEvent::BOOT_COMPLETE,
         SafetyState::SAFE_OFF, TransitionAction::NONE,
         TransitionAction::NONE, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
        {SafetyState::FAULT_LATCHED, TransitionEvent::BOOT_COMPLETE,
         SafetyState::FAULT_LATCHED, TransitionAction::NONE,
         TransitionAction::NONE, 0u,
         odrive::fault::FaultCode::INVALID_STATE},
    }};

SafetySupervisor::SafetySupervisor(
    odrive::fault::FaultManager& fault_manager,
    RealtimeEventRing& realtime_events,
    void* fault_context,
    FaultRaiseFn fault_raise,
    FaultClearFn fault_clear,
    FaultFirstFn fault_first)
    : fault_manager_(fault_manager),
      realtime_events_(realtime_events),
      fault_context_(fault_context),
      fault_raise_(fault_raise),
      fault_clear_(fault_clear),
      fault_first_(fault_first) {}

bool SafetySupervisor::submit_command(const Command& command) {
    return command_ring_.push(command);
}

bool SafetySupervisor::pop_result(CommandResult* result) {
    return result_ring_.pop(result);
}

bool SafetySupervisor::pop_legacy_axis_state(uint32_t* axis_state) {
    return legacy_axis_state_ring_.pop(axis_state);
}

bool SafetySupervisor::pop_realtime_request(RealtimeRequest* request) {
    return realtime_request_ring_.pop(request);
}

void SafetySupervisor::handle_realtime_event(const RealtimeEvent& event) {
    process_event(event);
}

void SafetySupervisor::handle_external_clear() {
    odrive::fault::FaultRecord retained;
    const bool has_fault = fault_first_ != nullptr
        ? fault_first_(fault_context_, &retained)
        : fault_manager_.first_fault(&retained);
    if (has_fault) {
        return;
    }
    if (state_ == SafetyState::FAULT_LATCHED ||
        state_ == SafetyState::SAFE_OFF || state_ == SafetyState::READY) {
        apply_transition(TransitionEvent::CLEAR_FAULTS);
    }
}

void SafetySupervisor::process() {
    const uint32_t overflow = realtime_events_.overflow_count();
    if (overflow != observed_realtime_overflow_) {
        RealtimeEvent fault;
        fault.type = RealtimeEventType::FAULT;
        fault.epoch = state_epoch_;
        fault.source = odrive::fault::FaultSource::ISR;
        fault.code = odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN;
        fault.site = odrive::fault::FaultSite::STATE_TRANSITION;
        fault.severity = odrive::fault::FaultSeverity::LATCHED;
        fault.arg0 = overflow - observed_realtime_overflow_;
        observed_realtime_overflow_ = overflow;
        process_fault_event(fault);
    }

    Command command;
    for (size_t i = 0; i < kCommandCapacity && command_ring_.pop(&command); ++i) {
        process_command(command);
    }

    RealtimeEvent event;
    for (size_t i = 0; i < RealtimeEventRing::kCapacity &&
                        realtime_events_.pop(&event); ++i) {
        process_event(event);
    }
    synchronize_fault_manager();
}

void SafetySupervisor::tick(uint32_t control_sequence) {
    process();
    if (transition_timeout_sequences_ != 0u &&
        !transition_deadline_active_) {
        state_enter_sequence_ = control_sequence;
        transition_deadline_ =
            control_sequence + transition_timeout_sequences_;
        transition_deadline_active_ = true;
    } else if (transition_deadline_active_ &&
               static_cast<int32_t>(control_sequence - transition_deadline_) >= 0) {
        raise_timeout_fault();
    }
}

const SafetySupervisor::TransitionRule*
SafetySupervisor::find_transition(TransitionEvent event) const {
    for (const TransitionRule& rule : kTransitions) {
        if (rule.from == state_ && rule.event == event) {
            return &rule;
        }
    }
    return nullptr;
}

bool SafetySupervisor::apply_transition(TransitionEvent event) {
    const TransitionRule* rule = find_transition(event);
    if (rule == nullptr) {
        return false;
    }
    state_ = rule->to;
    transition_deadline_ = 0u;
    arm_transition_deadline(rule->timeout_sequences);
    transition_deadline_active_ = false;
    if (rule->entry_action == TransitionAction::ENTER_SAFE_OFF) {
        operation_ = Operation::NONE;
        readiness_ = {};
    } else if (rule->entry_action == TransitionAction::ENTER_FAULT) {
        operation_ = Operation::NONE;
        readiness_ = {};
    }
    return true;
}

void SafetySupervisor::arm_transition_deadline(uint32_t timeout_sequences) {
    transition_timeout_sequences_ = timeout_sequences;
}

void SafetySupervisor::process_command(const Command& command) {
    if (command.request_id == 0u) {
        reject(command, odrive::fault::FaultCode::INVALID_STATE);
        return;
    }

    switch (command.type) {
        case CommandType::BOOT_COMPLETE:
            if (apply_transition(TransitionEvent::BOOT_COMPLETE)) {
                emit_result(command, CommandStatus::ACCEPTED);
                emit_result(command, CommandStatus::COMPLETED);
            } else {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
            }
            break;

        case CommandType::SET_OPERATION:
            if (command.operation == Operation::NONE ||
                (state_ != SafetyState::SAFE_OFF &&
                 state_ != SafetyState::READY) ||
                pending_operation_request_ != 0u) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            if (!apply_transition(TransitionEvent::SET_OPERATION)) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            state_epoch_ = next_epoch(state_epoch_);
            operation_ = command.operation;
            readiness_ = {};
            pending_operation_request_ = command.request_id;
            pending_operation_source_ = command.source;
            if (!realtime_request_ring_.push({
                    command.request_id, state_epoch_,
                    RealtimeRequestType::PREPARE_OPERATION,
                    command.operation})) {
                pending_operation_request_ = 0u;
                RealtimeEvent fault;
                fault.type = RealtimeEventType::FAULT;
                fault.epoch = state_epoch_;
                fault.source = odrive::fault::FaultSource::SAFETY;
                fault.code = odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN;
                fault.site = odrive::fault::FaultSite::STATE_TRANSITION;
                fault.severity = odrive::fault::FaultSeverity::IMMEDIATE_SHUTDOWN;
                process_fault_event(fault);
                emit_result(command, CommandStatus::FAILED, fault.code);
                break;
            }
            emit_result(command, CommandStatus::ACCEPTED);
            break;

        case CommandType::ARM:
            if (state_ != SafetyState::READY || pending_arm_request_ != 0u ||
                !apply_transition(TransitionEvent::ARM_REQUEST)) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            pending_arm_request_ = command.request_id;
            pending_arm_source_ = command.source;
            if (!realtime_request_ring_.push({
                    command.request_id, state_epoch_, RealtimeRequestType::ARM,
                    operation_})) {
                pending_arm_request_ = 0u;
                RealtimeEvent fault;
                fault.type = RealtimeEventType::FAULT;
                fault.epoch = state_epoch_;
                fault.source = odrive::fault::FaultSource::SAFETY;
                fault.code = odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN;
                fault.site = odrive::fault::FaultSite::STATE_TRANSITION;
                fault.severity = odrive::fault::FaultSeverity::IMMEDIATE_SHUTDOWN;
                process_fault_event(fault);
                emit_result(command, CommandStatus::FAILED, fault.code);
                break;
            }
            emit_result(command, CommandStatus::ACCEPTED);
            break;

        case CommandType::DISARM:
            // A safety stop is idempotent. Callers must be able to request it
            // during cleanup without first mirroring Supervisor state.
            if (state_ == SafetyState::SAFE_OFF ||
                state_ == SafetyState::FAULT_LATCHED) {
                emit_result(command, CommandStatus::ACCEPTED);
                emit_result(command, CommandStatus::COMPLETED);
                break;
            }
            if ((state_ != SafetyState::ARMED &&
                 state_ != SafetyState::PREPARING &&
                 state_ != SafetyState::READY) ||
                pending_disarm_request_ != 0u ||
                !apply_transition(TransitionEvent::DISARM_REQUEST)) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            pending_disarm_request_ = command.request_id;
            pending_disarm_source_ = command.source;
            if (!realtime_request_ring_.push({
                    command.request_id, state_epoch_, RealtimeRequestType::DISARM,
                    Operation::NONE})) {
                pending_disarm_request_ = 0u;
                RealtimeEvent fault;
                fault.type = RealtimeEventType::FAULT;
                fault.epoch = state_epoch_;
                fault.source = odrive::fault::FaultSource::SAFETY;
                fault.code = odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN;
                fault.site = odrive::fault::FaultSite::STATE_TRANSITION;
                fault.severity = odrive::fault::FaultSeverity::IMMEDIATE_SHUTDOWN;
                process_fault_event(fault);
                emit_result(command, CommandStatus::FAILED, fault.code);
                break;
            }
            emit_result(command, CommandStatus::ACCEPTED);
            break;

        case CommandType::CLEAR_FAULTS: {
            if (state_ != SafetyState::SAFE_OFF &&
                state_ != SafetyState::FAULT_LATCHED) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            bool empty_after_clear = false;
            const odrive::fault::ClearRequest clear_request{
                command.request_id, 0u};
            if (fault_clear_ != nullptr) {
                fault_clear_(fault_context_, clear_request, &empty_after_clear);
            } else {
                fault_manager_.clear(clear_request);
                empty_after_clear = fault_manager_.record_count() == 0u;
            }
            if (!empty_after_clear) {
                emit_result(command, CommandStatus::ACCEPTED);
                emit_result(command, CommandStatus::FAILED,
                            odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN);
                break;
            }
            if (!apply_transition(TransitionEvent::CLEAR_FAULTS)) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            emit_result(command, CommandStatus::ACCEPTED);
            emit_result(command, CommandStatus::COMPLETED);
            break;
        }

        case CommandType::LEGACY_AXIS_STATE:
            if (command.arg0 == 8u) {
                legacy_auto_arm_request_ = command.request_id;
                Command mapped = command;
                mapped.type = CommandType::SET_OPERATION;
                mapped.operation = Operation::CLOSED_LOOP;
                process_command(mapped);
                if (pending_operation_request_ != command.request_id) {
                    legacy_auto_arm_request_ = 0u;
                }
                break;
            }
            if (command.arg0 == 1u &&
                (state_ == SafetyState::PREPARING ||
                 state_ == SafetyState::READY ||
                 state_ == SafetyState::ARMED)) {
                Command mapped = command;
                mapped.type = CommandType::DISARM;
                mapped.operation = Operation::NONE;
                process_command(mapped);
                break;
            }
            if (!valid_legacy_axis_state(command.arg0) ||
                !legacy_axis_state_ring_.push(command.arg0)) {
                reject(command, odrive::fault::FaultCode::INVALID_STATE);
                break;
            }
            emit_result(command, CommandStatus::ACCEPTED);
            emit_result(command, CommandStatus::COMPLETED);
            break;

        case CommandType::RESET_ESTIMATOR:
        case CommandType::RESTART_CALIBRATION:
            reject(command, odrive::fault::FaultCode::CONTROLLER_REJECTED);
            break;
    }
}

void SafetySupervisor::process_event(const RealtimeEvent& event) {
    switch (event.type) {
        case RealtimeEventType::READY_SNAPSHOT:
            process_ready_event(event);
            break;
        case RealtimeEventType::ARM_CONFIRMED:
            process_arm_event(event);
            break;
        case RealtimeEventType::DISARM_CONFIRMED:
            process_disarm_event(event);
            break;
        case RealtimeEventType::FAULT:
            process_fault_event(event);
            break;
        case RealtimeEventType::PREPARE_REJECTED:
            process_prepare_rejected_event(event);
            break;
    }
}

void SafetySupervisor::process_ready_event(const RealtimeEvent& event) {
    if (state_ != SafetyState::PREPARING ||
        pending_operation_request_ == 0u || event.epoch != state_epoch_ ||
        event.sequence == 0u || event.feedback_sequence == 0u ||
        (event.readiness_flags & kRequiredReadinessFlags) !=
            kRequiredReadinessFlags) {
        return;
    }

    readiness_.epoch = event.epoch;
    readiness_.sequence = event.sequence;
    readiness_.feedback_sequence = event.feedback_sequence;
    readiness_.flags = event.readiness_flags;
    if (apply_transition(TransitionEvent::READY_SNAPSHOT)) {
        const uint32_t request_id = pending_operation_request_;
        const CommandSource source = pending_operation_source_;
        pending_operation_request_ = 0u;
        if (legacy_auto_arm_request_ == request_id) {
            pending_arm_request_ = request_id;
            pending_arm_source_ = source;
            if (!apply_transition(TransitionEvent::ARM_REQUEST) ||
                !realtime_request_ring_.push({
                    request_id, state_epoch_, RealtimeRequestType::ARM,
                    operation_})) {
                legacy_auto_arm_request_ = 0u;
                RealtimeEvent fault;
                fault.type = RealtimeEventType::FAULT;
                fault.epoch = state_epoch_;
                fault.source = odrive::fault::FaultSource::SAFETY;
                fault.code = odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN;
                fault.site = odrive::fault::FaultSite::STATE_TRANSITION;
                fault.severity = odrive::fault::FaultSeverity::LATCHED;
                process_fault_event(fault);
            }
        } else {
            emit_result(request_id, source, CommandStatus::COMPLETED);
        }
    }
}

void SafetySupervisor::process_arm_event(const RealtimeEvent& event) {
    if (state_ != SafetyState::READY || pending_arm_request_ == 0u ||
        event.epoch != state_epoch_ || event.sequence == 0u ||
        !apply_transition(TransitionEvent::ARM_CONFIRMED)) {
        return;
    }
    const uint32_t request_id = pending_arm_request_;
    const CommandSource source = pending_arm_source_;
    pending_arm_request_ = 0u;
    legacy_auto_arm_request_ = 0u;
    emit_result(request_id, source, CommandStatus::COMPLETED);
}

void SafetySupervisor::process_disarm_event(const RealtimeEvent& event) {
    if (state_ != SafetyState::STOPPING || pending_disarm_request_ == 0u ||
        event.epoch != state_epoch_ || event.sequence == 0u ||
        !apply_transition(TransitionEvent::DISARM_CONFIRMED)) {
        return;
    }
    const uint32_t request_id = pending_disarm_request_;
    const CommandSource source = pending_disarm_source_;
    pending_disarm_request_ = 0u;
    emit_result(request_id, source, CommandStatus::COMPLETED);
}

void SafetySupervisor::process_prepare_rejected_event(
        const RealtimeEvent& event) {
    if (state_ != SafetyState::PREPARING ||
        pending_operation_request_ == 0u || event.epoch != state_epoch_ ||
        event.sequence == 0u ||
        !apply_transition(TransitionEvent::PREPARE_REJECTED)) {
        return;
    }
    const uint32_t request_id = pending_operation_request_;
    const CommandSource source = pending_operation_source_;
    pending_operation_request_ = 0u;
    legacy_auto_arm_request_ = 0u;
    emit_result(request_id, source, CommandStatus::FAILED,
                event.code == odrive::fault::FaultCode::NONE
                    ? odrive::fault::FaultCode::CONTROLLER_REJECTED
                    : event.code);
}

void SafetySupervisor::process_fault_event(const RealtimeEvent& event) {
    odrive::fault::FaultRecord record;
    record.control_sequence = event.control_sequence;
    record.timestamp_cycles = event.timestamp_cycles;
    record.state_epoch = event.epoch == 0u ? state_epoch_ : event.epoch;
    record.source = event.source;
    record.code = event.code == odrive::fault::FaultCode::NONE
        ? odrive::fault::FaultCode::CONTROLLER_ERROR : event.code;
    record.site = event.site;
    record.severity = event.severity;
    record.parent_fault_sequence = event.parent_fault_sequence;
    record.arg0 = event.arg0;
    record.arg1 = event.arg1;
    record.arg2 = event.arg2;
    if (fault_raise_ != nullptr) {
        fault_raise_(fault_context_, record, event.legacy_projection,
                     event.trace_recorded == 0u);
    } else {
        fault_manager_.raise(record);
    }
    apply_transition(TransitionEvent::FAULT);
    fail_pending(record.code);
}

void SafetySupervisor::emit_result(const Command& command,
                                   CommandStatus status,
                                   odrive::fault::FaultCode reason) {
    emit_result(command.request_id, command.source, status, reason);
}

void SafetySupervisor::emit_result(uint32_t request_id,
                                   CommandSource source,
                                   CommandStatus status,
                                   odrive::fault::FaultCode reason) {
    result_ring_.push({request_id, status, state_epoch_, reason, source});
}

void SafetySupervisor::reject(const Command& command,
                              odrive::fault::FaultCode reason) {
    emit_result(command, CommandStatus::REJECTED, reason);
}

void SafetySupervisor::fail_pending(odrive::fault::FaultCode reason) {
    if (pending_operation_request_ != 0u) {
        emit_result(pending_operation_request_, pending_operation_source_,
                    CommandStatus::FAILED, reason);
        pending_operation_request_ = 0u;
    }
    legacy_auto_arm_request_ = 0u;
    if (pending_arm_request_ != 0u) {
        emit_result(pending_arm_request_, pending_arm_source_,
                    CommandStatus::FAILED, reason);
        pending_arm_request_ = 0u;
    }
    if (pending_disarm_request_ != 0u) {
        emit_result(pending_disarm_request_, pending_disarm_source_,
                    CommandStatus::FAILED, reason);
        pending_disarm_request_ = 0u;
    }
}

void SafetySupervisor::raise_timeout_fault() {
    if (state_ != SafetyState::PREPARING &&
        !(state_ == SafetyState::READY && pending_arm_request_ != 0u) &&
        state_ != SafetyState::STOPPING) {
        transition_deadline_ = 0u;
        transition_timeout_sequences_ = 0u;
        transition_deadline_active_ = false;
        return;
    }

    RealtimeEvent event;
    event.type = RealtimeEventType::FAULT;
    event.epoch = state_epoch_;
    event.control_sequence = transition_deadline_;
    event.source = odrive::fault::FaultSource::SAFETY;
    event.code = odrive::fault::FaultCode::TIMEOUT;
    event.site = odrive::fault::FaultSite::COMMAND_TIMEOUT;
    event.severity = odrive::fault::FaultSeverity::LATCHED;
    event.arg0 = static_cast<uint32_t>(state_);
    process_fault_event(event);
    transition_deadline_ = 0u;
    transition_timeout_sequences_ = 0u;
    transition_deadline_active_ = false;
}

void SafetySupervisor::synchronize_fault_manager() {
    if (state_ == SafetyState::FAULT_LATCHED) {
        return;
    }
    odrive::fault::FaultRecord first;
    const bool found = fault_first_ != nullptr
        ? fault_first_(fault_context_, &first)
        : fault_manager_.first_fault(&first);
    if (!found) {
        return;
    }
    apply_transition(TransitionEvent::FAULT);
    fail_pending(first.code);
}

bool SafetySupervisor::valid_legacy_axis_state(uint32_t axis_state) const {
    switch (axis_state) {
        case 0u:  // UNDEFINED
        case 1u:  // IDLE
        case 2u:  // STARTUP_SEQUENCE
        case 3u:  // FULL_CALIBRATION_SEQUENCE
        case 4u:  // MOTOR_CALIBRATION
        case 7u:  // ENCODER_OFFSET_CALIBRATION
        case 8u:  // CLOSED_LOOP_CONTROL
        case 9u:  // LOCKIN_SPIN
            return true;
        default:
            return false;
    }
}

uint32_t SafetySupervisor::next_epoch(uint32_t epoch) {
    ++epoch;
    return epoch == 0u ? 1u : epoch;
}

bool SafetySupervisor::can_run_controller() const {
    return state_ == SafetyState::ARMED && operation_ == Operation::CLOSED_LOOP;
}

bool SafetySupervisor::can_enable_pwm() const {
    return state_ == SafetyState::ARMED;
}

bool SafetySupervisor::can_modify_config() const {
    switch (state_) {
        case SafetyState::SAFE_OFF:
        case SafetyState::FAULT_LATCHED:
            return true;
        case SafetyState::BOOT:
        case SafetyState::PREPARING:
        case SafetyState::READY:
        case SafetyState::ARMED:
        case SafetyState::STOPPING:
            return false;
    }
    return false;
}

}  // namespace odrive::safety
