#ifndef ODRIVE_NATIVE_SCENARIO_C_API_H
#define ODRIVE_NATIVE_SCENARIO_C_API_H

#include <stdint.h>

#if defined(_WIN32) && defined(ODRIVE_SCENARIO_BUILD)
#define ODRIVE_SCENARIO_API __declspec(dllexport)
#elif defined(_WIN32) && defined(ODRIVE_SCENARIO_USE_DLL)
#define ODRIVE_SCENARIO_API __declspec(dllimport)
#elif !defined(_WIN32)
#define ODRIVE_SCENARIO_API __attribute__((visibility("default")))
#else
#define ODRIVE_SCENARIO_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum { ODRIVE_SCENARIO_ABI_VERSION = 1u };

typedef struct ODriveScenario ODriveScenario;

typedef struct {
    uint32_t request_id;
    uint32_t arg0;
    uint8_t source;
    uint8_t type;
    uint8_t operation;
    uint8_t reserved;
} ODriveScenarioCommand;

typedef struct {
    uint32_t sequence;
    uint32_t epoch;
    uint32_t feedback_sequence;
    uint32_t control_sequence;
    uint32_t timestamp_cycles;
    uint32_t parent_fault_sequence;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
    uint32_t legacy_projection;
    uint16_t source;
    uint16_t code;
    uint16_t site;
    uint8_t type;
    uint8_t readiness_flags;
    uint8_t severity;
    uint8_t trace_recorded;
} ODriveScenarioEvent;

typedef struct {
    uint32_t request_id;
    uint32_t state_epoch;
    uint16_t reason;
    uint8_t source;
    uint8_t status;
} ODriveScenarioResult;

typedef struct {
    uint32_t request_id;
    uint32_t epoch;
    uint8_t type;
    uint8_t operation;
    uint16_t reserved;
} ODriveScenarioRealtimeRequest;

typedef struct {
    uint32_t state_epoch;
    uint32_t control_sequence;
    uint32_t readiness_sequence;
    uint32_t feedback_sequence;
    uint32_t dropped_commands;
    uint32_t dropped_results;
    uint32_t realtime_event_overflows;
    uint32_t fault_count;
    uint8_t state;
    uint8_t operation;
    uint8_t readiness_flags;
    uint8_t reserved;
} ODriveScenarioState;

typedef struct {
    uint32_t fault_sequence;
    uint32_t control_sequence;
    uint32_t timestamp_cycles;
    uint32_t state_epoch;
    uint32_t parent_fault_sequence;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
    uint32_t occurrence_count;
    uint16_t source;
    uint16_t code;
    uint16_t site;
    uint8_t severity;
    uint8_t reserved;
} ODriveScenarioFault;

ODRIVE_SCENARIO_API uint32_t odrive_scenario_abi_version(void);
ODRIVE_SCENARIO_API ODriveScenario* odrive_scenario_create(void);
ODRIVE_SCENARIO_API void odrive_scenario_destroy(ODriveScenario* scenario);
ODRIVE_SCENARIO_API int odrive_scenario_submit(
    ODriveScenario* scenario, const ODriveScenarioCommand* command);
ODRIVE_SCENARIO_API int odrive_scenario_inject_event(
    ODriveScenario* scenario, const ODriveScenarioEvent* event);
ODRIVE_SCENARIO_API void odrive_scenario_advance(
    ODriveScenario* scenario, uint32_t control_sequence);
ODRIVE_SCENARIO_API int odrive_scenario_pop_result(
    ODriveScenario* scenario, ODriveScenarioResult* result);
ODRIVE_SCENARIO_API int odrive_scenario_pop_realtime_request(
    ODriveScenario* scenario, ODriveScenarioRealtimeRequest* request);
ODRIVE_SCENARIO_API int odrive_scenario_read_state(
    const ODriveScenario* scenario, ODriveScenarioState* state);
ODRIVE_SCENARIO_API int odrive_scenario_read_first_fault(
    const ODriveScenario* scenario, ODriveScenarioFault* fault);

#ifdef __cplusplus
}
#endif

#endif
