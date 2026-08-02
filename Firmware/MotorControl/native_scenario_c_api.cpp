#include "native_scenario_c_api.h"

#include <new>

#include "fault_manager.hpp"
#include "realtime_event_ring.hpp"
#include "safety_supervisor.hpp"

struct ODriveScenario {
    odrive::fault::FaultManager faults;
    odrive::safety::RealtimeEventRing events;
    odrive::safety::SafetySupervisor supervisor;
    uint32_t control_sequence = 0u;

    ODriveScenario() : supervisor(faults, events) {}
};

namespace {

static_assert(sizeof(ODriveScenarioCommand) == 12u, "scenario command ABI");
static_assert(sizeof(ODriveScenarioEvent) == 52u, "scenario event ABI");
static_assert(sizeof(ODriveScenarioResult) == 12u, "scenario result ABI");
static_assert(sizeof(ODriveScenarioRealtimeRequest) == 12u,
              "scenario realtime request ABI");
static_assert(sizeof(ODriveScenarioState) == 36u, "scenario state ABI");
static_assert(sizeof(ODriveScenarioFault) == 44u, "scenario fault ABI");

template <typename Enum>
Enum enum_value(uint8_t value) {
    return static_cast<Enum>(value);
}

template <typename Enum>
Enum enum_value(uint16_t value) {
    return static_cast<Enum>(value);
}

bool valid_command(const ODriveScenarioCommand& value) {
    return value.reserved == 0u && value.source <= 2u && value.type <= 7u &&
           value.operation <= 3u;
}

bool valid_event(const ODriveScenarioEvent& value) {
    return value.type <= 4u && value.readiness_flags <= 0x1fu &&
           value.source >= 1u && value.source <= 9u && value.code <= 10u &&
           value.site <= 10u && value.severity <= 4u &&
           value.trace_recorded <= 1u;
}

}  // namespace

extern "C" {

uint32_t odrive_scenario_abi_version(void) {
    return ODRIVE_SCENARIO_ABI_VERSION;
}

ODriveScenario* odrive_scenario_create(void) {
    return new (std::nothrow) ODriveScenario();
}

void odrive_scenario_destroy(ODriveScenario* scenario) {
    delete scenario;
}

int odrive_scenario_submit(ODriveScenario* scenario,
                           const ODriveScenarioCommand* command) {
    if (scenario == nullptr || command == nullptr || !valid_command(*command)) {
        return 0;
    }
    odrive::safety::Command value;
    value.request_id = command->request_id;
    value.arg0 = command->arg0;
    value.source = enum_value<odrive::safety::CommandSource>(command->source);
    value.type = enum_value<odrive::safety::CommandType>(command->type);
    value.operation = enum_value<odrive::safety::Operation>(command->operation);
    if (!scenario->supervisor.submit_command(value)) return 0;
    scenario->supervisor.process();
    return 1;
}

int odrive_scenario_inject_event(ODriveScenario* scenario,
                                 const ODriveScenarioEvent* event) {
    if (scenario == nullptr || event == nullptr || !valid_event(*event)) {
        return 0;
    }
    odrive::safety::RealtimeEvent value;
    value.sequence = event->sequence;
    value.epoch = event->epoch;
    value.feedback_sequence = event->feedback_sequence;
    value.control_sequence = event->control_sequence;
    value.timestamp_cycles = event->timestamp_cycles;
    value.parent_fault_sequence = event->parent_fault_sequence;
    value.arg0 = event->arg0;
    value.arg1 = event->arg1;
    value.arg2 = event->arg2;
    value.legacy_projection = event->legacy_projection;
    value.source = enum_value<odrive::fault::FaultSource>(event->source);
    value.code = enum_value<odrive::fault::FaultCode>(event->code);
    value.site = enum_value<odrive::fault::FaultSite>(event->site);
    value.type = enum_value<odrive::safety::RealtimeEventType>(event->type);
    value.readiness_flags = event->readiness_flags;
    value.severity = enum_value<odrive::fault::FaultSeverity>(event->severity);
    value.trace_recorded = event->trace_recorded;
    if (!scenario->events.push(value)) return 0;
    scenario->supervisor.process();
    return 1;
}

void odrive_scenario_advance(ODriveScenario* scenario,
                             uint32_t control_sequence) {
    if (scenario == nullptr) return;
    scenario->control_sequence = control_sequence;
    scenario->supervisor.tick(control_sequence);
    scenario->supervisor.process();
}

int odrive_scenario_pop_result(ODriveScenario* scenario,
                               ODriveScenarioResult* result) {
    if (scenario == nullptr || result == nullptr) return 0;
    odrive::safety::CommandResult value;
    if (!scenario->supervisor.pop_result(&value)) return 0;
    result->request_id = value.request_id;
    result->state_epoch = value.state_epoch;
    result->reason = static_cast<uint16_t>(value.reason);
    result->source = static_cast<uint8_t>(value.source);
    result->status = static_cast<uint8_t>(value.status);
    return 1;
}

int odrive_scenario_pop_realtime_request(
        ODriveScenario* scenario, ODriveScenarioRealtimeRequest* request) {
    if (scenario == nullptr || request == nullptr) return 0;
    odrive::safety::RealtimeRequest value;
    if (!scenario->supervisor.pop_realtime_request(&value)) return 0;
    request->request_id = value.request_id;
    request->epoch = value.epoch;
    request->type = static_cast<uint8_t>(value.type);
    request->operation = static_cast<uint8_t>(value.operation);
    request->reserved = 0u;
    return 1;
}

int odrive_scenario_read_state(const ODriveScenario* scenario,
                               ODriveScenarioState* state) {
    if (scenario == nullptr || state == nullptr) return 0;
    const auto readiness = scenario->supervisor.readiness();
    state->state_epoch = scenario->supervisor.state_epoch();
    state->control_sequence = scenario->control_sequence;
    state->readiness_sequence = readiness.sequence;
    state->feedback_sequence = readiness.feedback_sequence;
    state->dropped_commands = scenario->supervisor.dropped_commands();
    state->dropped_results = scenario->supervisor.dropped_results();
    state->realtime_event_overflows = scenario->events.overflow_count();
    state->fault_count = static_cast<uint32_t>(scenario->faults.record_count());
    state->state = static_cast<uint8_t>(scenario->supervisor.state());
    state->operation = static_cast<uint8_t>(scenario->supervisor.operation());
    state->readiness_flags = readiness.flags;
    state->reserved = 0u;
    return 1;
}

int odrive_scenario_read_first_fault(const ODriveScenario* scenario,
                                     ODriveScenarioFault* fault) {
    if (scenario == nullptr || fault == nullptr) return 0;
    odrive::fault::FaultRecord value;
    if (!scenario->faults.first_fault(&value)) return 0;
    fault->fault_sequence = value.fault_sequence;
    fault->control_sequence = value.control_sequence;
    fault->timestamp_cycles = value.timestamp_cycles;
    fault->state_epoch = value.state_epoch;
    fault->parent_fault_sequence = value.parent_fault_sequence;
    fault->arg0 = value.arg0;
    fault->arg1 = value.arg1;
    fault->arg2 = value.arg2;
    fault->occurrence_count = value.occurrence_count;
    fault->source = static_cast<uint16_t>(value.source);
    fault->code = static_cast<uint16_t>(value.code);
    fault->site = static_cast<uint16_t>(value.site);
    fault->severity = static_cast<uint8_t>(value.severity);
    fault->reserved = 0u;
    return 1;
}

}  // extern "C"
