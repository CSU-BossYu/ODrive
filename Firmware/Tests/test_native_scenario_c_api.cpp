#include <doctest.h>

#include "MotorControl/native_scenario_c_api.h"

namespace {

ODriveScenarioResult pop_result(ODriveScenario* scenario) {
    ODriveScenarioResult result{};
    REQUIRE(odrive_scenario_pop_result(scenario, &result));
    return result;
}

}  // namespace

TEST_SUITE("NativeScenarioCAbi") {
    TEST_CASE("closed loop lifecycle is deterministic across the C ABI") {
        CHECK(odrive_scenario_abi_version() == ODRIVE_SCENARIO_ABI_VERSION);
        ODriveScenario* scenario = odrive_scenario_create();
        REQUIRE(scenario != nullptr);

        ODriveScenarioCommand boot{1u, 0u, 0u, 0u, 0u, 0u};
        REQUIRE(odrive_scenario_submit(scenario, &boot));
        CHECK(pop_result(scenario).status == 0u);
        CHECK(pop_result(scenario).status == 2u);

        ODriveScenarioCommand operation{2u, 0u, 1u, 1u, 1u, 0u};
        REQUIRE(odrive_scenario_submit(scenario, &operation));
        CHECK(pop_result(scenario).status == 0u);
        ODriveScenarioRealtimeRequest prepare{};
        REQUIRE(odrive_scenario_pop_realtime_request(scenario, &prepare));
        CHECK(prepare.type == 0u);

        ODriveScenarioEvent ready{};
        ready.sequence = 10u;
        ready.epoch = prepare.epoch;
        ready.feedback_sequence = 10u;
        ready.control_sequence = 10u;
        ready.source = 7u;
        ready.type = 0u;
        ready.readiness_flags = 0x1fu;
        REQUIRE(odrive_scenario_inject_event(scenario, &ready));
        CHECK(pop_result(scenario).status == 2u);

        ODriveScenarioCommand arm{3u, 0u, 1u, 2u, 0u, 0u};
        REQUIRE(odrive_scenario_submit(scenario, &arm));
        CHECK(pop_result(scenario).status == 0u);
        ODriveScenarioRealtimeRequest arm_request{};
        REQUIRE(odrive_scenario_pop_realtime_request(scenario, &arm_request));
        CHECK(arm_request.type == 1u);

        ODriveScenarioEvent armed{};
        armed.sequence = 11u;
        armed.epoch = arm_request.epoch;
        armed.control_sequence = 11u;
        armed.source = 7u;
        armed.type = 1u;
        REQUIRE(odrive_scenario_inject_event(scenario, &armed));
        CHECK(pop_result(scenario).status == 2u);

        ODriveScenarioState state{};
        REQUIRE(odrive_scenario_read_state(scenario, &state));
        CHECK(state.state == 4u);
        CHECK(state.operation == 1u);
        CHECK(state.fault_count == 0u);
        odrive_scenario_destroy(scenario);
    }

    TEST_CASE("timeout and malformed ABI input fail closed") {
        ODriveScenario* scenario = odrive_scenario_create();
        REQUIRE(scenario != nullptr);
        ODriveScenarioCommand malformed{1u, 0u, 9u, 0u, 0u, 0u};
        CHECK_FALSE(odrive_scenario_submit(scenario, &malformed));

        ODriveScenarioCommand boot{1u, 0u, 0u, 0u, 0u, 0u};
        REQUIRE(odrive_scenario_submit(scenario, &boot));
        CHECK(pop_result(scenario).status == 0u);
        CHECK(pop_result(scenario).status == 2u);
        ODriveScenarioCommand operation{2u, 0u, 1u, 1u, 1u, 0u};
        REQUIRE(odrive_scenario_submit(scenario, &operation));
        CHECK(pop_result(scenario).status == 0u);
        odrive_scenario_advance(scenario, 1u);
        odrive_scenario_advance(scenario, 20001u);

        ODriveScenarioState state{};
        REQUIRE(odrive_scenario_read_state(scenario, &state));
        CHECK(state.state == 6u);
        ODriveScenarioFault fault{};
        REQUIRE(odrive_scenario_read_first_fault(scenario, &fault));
        CHECK(fault.code == 1u);
        CHECK(pop_result(scenario).status == 3u);
        odrive_scenario_destroy(scenario);
    }
}
