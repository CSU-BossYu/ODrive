#include <doctest.h>

#include "MotorControl/safety_supervisor.hpp"

using namespace odrive::fault;
using namespace odrive::safety;

namespace {

void submit_and_process(SafetySupervisor& supervisor, const Command& command) {
    REQUIRE(supervisor.submit_command(command));
    supervisor.process();
}

CommandResult pop_result(SafetySupervisor& supervisor) {
    CommandResult result;
    REQUIRE(supervisor.pop_result(&result));
    return result;
}

RealtimeEvent ready_event(uint32_t epoch, uint32_t sequence = 10u) {
    RealtimeEvent event;
    event.type = RealtimeEventType::READY_SNAPSHOT;
    event.epoch = epoch;
    event.sequence = sequence;
    event.feedback_sequence = sequence + 1u;
    event.readiness_flags = kRequiredReadinessFlags;
    return event;
}

RealtimeEvent confirmation_event(RealtimeEventType type, uint32_t epoch) {
    RealtimeEvent event;
    event.type = type;
    event.epoch = epoch;
    event.sequence = 20u;
    return event;
}

}  // namespace

TEST_SUITE("SafetySupervisor") {
    TEST_CASE("command results close only after coherent epoch evidence") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {1u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        CHECK(supervisor.state() == SafetyState::SAFE_OFF);

        submit_and_process(supervisor,
                           {2u, CommandSource::CAN,
                            CommandType::SET_OPERATION,
                            Operation::CLOSED_LOOP});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(supervisor.state() == SafetyState::PREPARING);
        CHECK(supervisor.state_epoch() == 1u);
        RealtimeRequest prepare_request;
        REQUIRE(supervisor.pop_realtime_request(&prepare_request));
        CHECK(prepare_request.request_id == 2u);
        CHECK(prepare_request.epoch == supervisor.state_epoch());
        CHECK(prepare_request.type == RealtimeRequestType::PREPARE_OPERATION);

        auto stale = ready_event(99u);
        REQUIRE(events.push(stale));
        supervisor.process();
        CHECK(supervisor.state() == SafetyState::PREPARING);

        REQUIRE(events.push(ready_event(supervisor.state_epoch())));
        supervisor.process();
        CHECK(supervisor.state() == SafetyState::READY);
        auto operation_done = pop_result(supervisor);
        CHECK(operation_done.request_id == 2u);
        CHECK(operation_done.status == CommandStatus::COMPLETED);
        CHECK(operation_done.source == CommandSource::CAN);

        submit_and_process(supervisor,
                           {3u, CommandSource::CAN, CommandType::ARM});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        RealtimeRequest arm_request;
        REQUIRE(supervisor.pop_realtime_request(&arm_request));
        CHECK(arm_request.type == RealtimeRequestType::ARM);
        REQUIRE(events.push(confirmation_event(RealtimeEventType::ARM_CONFIRMED,
                                                supervisor.state_epoch())));
        supervisor.process();
        CHECK(supervisor.state() == SafetyState::ARMED);
        const auto arm_done = pop_result(supervisor);
        CHECK(arm_done.status == CommandStatus::COMPLETED);
        CHECK(arm_done.source == CommandSource::CAN);
        CHECK(supervisor.can_run_controller());
        CHECK(supervisor.can_enable_pwm());
        CHECK_FALSE(supervisor.can_modify_config());

        submit_and_process(supervisor,
                           {4u, CommandSource::CAN, CommandType::DISARM});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        RealtimeRequest disarm_request;
        REQUIRE(supervisor.pop_realtime_request(&disarm_request));
        CHECK(disarm_request.type == RealtimeRequestType::DISARM);
        REQUIRE(events.push(confirmation_event(
            RealtimeEventType::DISARM_CONFIRMED, supervisor.state_epoch())));
        supervisor.process();
        CHECK(supervisor.state() == SafetyState::SAFE_OFF);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        CHECK(supervisor.can_modify_config());
    }

    TEST_CASE("illegal commands and stale confirmations have final outcomes") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {7u, CommandSource::CAN, CommandType::ARM});
        const auto rejected = pop_result(supervisor);
        CHECK(rejected.request_id == 7u);
        CHECK(rejected.status == CommandStatus::REJECTED);
        CHECK(rejected.reason == FaultCode::INVALID_STATE);

        submit_and_process(supervisor,
                           {8u, CommandSource::CAN,
                            CommandType::LEGACY_AXIS_STATE,
                            Operation::NONE, 8u});
        CHECK(pop_result(supervisor).status == CommandStatus::REJECTED);

        RealtimeEvent stale = confirmation_event(
            RealtimeEventType::ARM_CONFIRMED, 123u);
        REQUIRE(events.push(stale));
        supervisor.process();
        CHECK(supervisor.state() == SafetyState::BOOT);
    }

    TEST_CASE("disarm is idempotent in already-safe states") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {70u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);

        submit_and_process(supervisor,
                           {71u, CommandSource::CAN, CommandType::DISARM});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        CHECK(supervisor.state() == SafetyState::SAFE_OFF);
        RealtimeRequest unused;
        CHECK_FALSE(supervisor.pop_realtime_request(&unused));

        RealtimeEvent fault;
        fault.type = RealtimeEventType::FAULT;
        fault.epoch = supervisor.state_epoch();
        fault.sequence = 72u;
        fault.code = FaultCode::CONTROLLER_ERROR;
        fault.severity = FaultSeverity::LATCHED;
        supervisor.handle_realtime_event(fault);
        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);

        submit_and_process(supervisor,
                           {73u, CommandSource::USB, CommandType::DISARM});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);
        REQUIRE(faults.record_count() == 1u);
    }

    TEST_CASE("prepare rejection returns safe-off without latching a fault") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {80u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        submit_and_process(supervisor,
                           {81u, CommandSource::CAN,
                            CommandType::SET_OPERATION,
                            Operation::CLOSED_LOOP});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        RealtimeRequest prepare;
        REQUIRE(supervisor.pop_realtime_request(&prepare));

        RealtimeEvent rejection;
        rejection.type = RealtimeEventType::PREPARE_REJECTED;
        rejection.epoch = prepare.epoch;
        rejection.sequence = 82u;
        rejection.source = FaultSource::CONTROLLER;
        rejection.code = FaultCode::CONTROLLER_REJECTED;
        rejection.site = FaultSite::CLOSED_LOOP_PREPARE;
        rejection.severity = FaultSeverity::INFO;
        supervisor.handle_realtime_event(rejection);

        const auto failed = pop_result(supervisor);
        CHECK(failed.request_id == 81u);
        CHECK(failed.status == CommandStatus::FAILED);
        CHECK(failed.reason == FaultCode::CONTROLLER_REJECTED);
        CHECK(supervisor.state() == SafetyState::SAFE_OFF);
        CHECK(supervisor.operation() == Operation::NONE);
        CHECK(faults.record_count() == 0u);
    }

    TEST_CASE("faults fail pending operations and latch safe state") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {10u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        submit_and_process(supervisor,
                           {11u, CommandSource::CAN,
                            CommandType::SET_OPERATION,
                            Operation::CALIBRATION});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);

        RealtimeEvent fault;
        fault.type = RealtimeEventType::FAULT;
        fault.epoch = supervisor.state_epoch();
        fault.control_sequence = 44u;
        fault.source = FaultSource::ENCODER;
        fault.code = FaultCode::FEEDBACK_MISSING;
        fault.site = FaultSite::ENCODER_UPDATE;
        fault.severity = FaultSeverity::LATCHED;
        REQUIRE(events.push(fault));
        supervisor.process();

        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);
        const auto failed = pop_result(supervisor);
        CHECK(failed.request_id == 11u);
        CHECK(failed.status == CommandStatus::FAILED);
        CHECK(failed.reason == FaultCode::FEEDBACK_MISSING);
        FaultRecord first;
        REQUIRE(faults.first_fault(&first));
        CHECK(first.code == FaultCode::FEEDBACK_MISSING);

        submit_and_process(supervisor,
                           {12u, CommandSource::USB,
                            CommandType::CLEAR_FAULTS});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        CHECK(supervisor.state() == SafetyState::SAFE_OFF);
    }

    TEST_CASE("controller rejection fails an accepted arm request concretely") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {30u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        submit_and_process(supervisor,
                           {31u, CommandSource::CAN,
                            CommandType::SET_OPERATION,
                            Operation::CLOSED_LOOP});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        RealtimeRequest prepare_request;
        REQUIRE(supervisor.pop_realtime_request(&prepare_request));
        CHECK(prepare_request.type == RealtimeRequestType::PREPARE_OPERATION);
        REQUIRE(events.push(ready_event(supervisor.state_epoch(), 40u)));
        supervisor.process();
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);

        submit_and_process(supervisor,
                           {32u, CommandSource::CAN, CommandType::ARM});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        RealtimeRequest arm_request;
        REQUIRE(supervisor.pop_realtime_request(&arm_request));
        CHECK(arm_request.type == RealtimeRequestType::ARM);

        RealtimeEvent rejection;
        rejection.type = RealtimeEventType::FAULT;
        rejection.epoch = supervisor.state_epoch();
        rejection.sequence = 41u;
        rejection.source = FaultSource::CONTROLLER;
        rejection.code = FaultCode::CONTROLLER_REJECTED;
        rejection.site = FaultSite::CONTROLLER_UPDATE;
        rejection.severity = FaultSeverity::LATCHED;
        REQUIRE(events.push(rejection));
        supervisor.process();

        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);
        const auto failed = pop_result(supervisor);
        CHECK(failed.request_id == 32u);
        CHECK(failed.status == CommandStatus::FAILED);
        CHECK(failed.reason == FaultCode::CONTROLLER_REJECTED);
    }

    TEST_CASE("prepare timeout is a concrete fault, not a delayed bool") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {20u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        submit_and_process(supervisor,
                           {21u, CommandSource::INTERNAL,
                            CommandType::SET_OPERATION,
                            Operation::CLOSED_LOOP});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        supervisor.tick(1u);
        supervisor.tick(20002u);
        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);
        FaultRecord first;
        REQUIRE(faults.first_fault(&first));
        CHECK(first.code == FaultCode::TIMEOUT);
        const auto failed = pop_result(supervisor);
        CHECK(failed.request_id == 21u);
        CHECK(failed.status == CommandStatus::FAILED);
        CHECK(failed.reason == FaultCode::TIMEOUT);
    }

    TEST_CASE("legacy state eight completes through prepare and arm evidence") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {40u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        submit_and_process(supervisor,
                           {41u, CommandSource::CAN,
                            CommandType::LEGACY_AXIS_STATE,
                            Operation::NONE, 8u});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);

        RealtimeRequest prepare;
        REQUIRE(supervisor.pop_realtime_request(&prepare));
        supervisor.handle_realtime_event(ready_event(prepare.epoch, 50u));
        CHECK(supervisor.state() == SafetyState::READY);

        RealtimeRequest arm;
        REQUIRE(supervisor.pop_realtime_request(&arm));
        CHECK(arm.type == RealtimeRequestType::ARM);
        supervisor.handle_realtime_event(
            confirmation_event(RealtimeEventType::ARM_CONFIRMED, arm.epoch));
        CHECK(supervisor.state() == SafetyState::ARMED);
        const auto completed = pop_result(supervisor);
        CHECK(completed.request_id == 41u);
        CHECK(completed.status == CommandStatus::COMPLETED);
    }

    TEST_CASE("transition timeout remains correct across uint32 wrap") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);

        submit_and_process(supervisor,
                           {50u, CommandSource::INTERNAL,
                            CommandType::BOOT_COMPLETE});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);
        CHECK(pop_result(supervisor).status == CommandStatus::COMPLETED);
        submit_and_process(supervisor,
                           {51u, CommandSource::INTERNAL,
                            CommandType::SET_OPERATION,
                            Operation::CLOSED_LOOP});
        CHECK(pop_result(supervisor).status == CommandStatus::ACCEPTED);

        supervisor.tick(0xfffffff0u);
        supervisor.tick(0xfffffff1u);
        CHECK(supervisor.state() == SafetyState::PREPARING);
        supervisor.tick(0x00004e10u);
        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);
    }

    TEST_CASE("realtime ring overflow is visible to the Supervisor") {
        FaultManager faults;
        RealtimeEventRing events;
        SafetySupervisor supervisor(faults, events);
        for (size_t index = 0; index < RealtimeEventRing::capacity(); ++index) {
            RealtimeEvent stale = ready_event(99u, static_cast<uint32_t>(index + 1u));
            REQUIRE(events.push(stale));
        }
        RealtimeEvent overflow = ready_event(99u, 100u);
        CHECK_FALSE(events.push(overflow));

        supervisor.process();
        CHECK(supervisor.state() == SafetyState::FAULT_LATCHED);
        FaultRecord first;
        REQUIRE(faults.first_fault(&first));
        CHECK(first.code == FaultCode::POWER_STAGE_SHUTDOWN);
    }
}

TEST_SUITE("RealtimeEventRing") {
    TEST_CASE("bounded overflow is explicit and preserves ordering") {
        RealtimeEventRing events;
        for (size_t index = 0; index < RealtimeEventRing::capacity(); ++index) {
            RealtimeEvent event;
            event.sequence = static_cast<uint32_t>(index + 1u);
            REQUIRE(events.push(event));
        }

        RealtimeEvent overflow;
        overflow.sequence = 999u;
        CHECK_FALSE(events.push(overflow));
        CHECK(events.overflow_count() == 1u);

        for (size_t index = 0; index < RealtimeEventRing::capacity(); ++index) {
            RealtimeEvent event;
            REQUIRE(events.pop(&event));
            CHECK(event.sequence == index + 1u);
        }
        CHECK_FALSE(events.pop(&overflow));
    }
}
