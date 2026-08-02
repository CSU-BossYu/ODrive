#include <doctest.h>

#include "MotorControl/fault_manager.hpp"

using namespace odrive::fault;

namespace {
FaultRecord make_record(FaultSource source, FaultCode code, FaultSite site,
                        FaultSeverity severity, uint32_t epoch,
                        uint32_t parent = 0) {
    FaultRecord value;
    value.control_sequence = 42;
    value.timestamp_cycles = 168000;
    value.state_epoch = epoch;
    value.source = source;
    value.code = code;
    value.site = site;
    value.severity = severity;
    value.parent_fault_sequence = parent;
    return value;
}
}

TEST_SUITE("FaultManager") {
    TEST_CASE("first fault is retained and consequence chain is explicit") {
        FaultManager manager;
        auto root = make_record(FaultSource::ENCODER, FaultCode::FEEDBACK_MISSING,
                                FaultSite::ENCODER_UPDATE, FaultSeverity::LATCHED, 7);
        CHECK(manager.raise(root));

        auto consequence = make_record(FaultSource::CONTROLLER,
                                       FaultCode::CONTROLLER_REJECTED,
                                       FaultSite::CONTROLLER_UPDATE,
                                       FaultSeverity::LATCHED, 7,
                                       root.fault_sequence);
        CHECK(manager.raise(consequence));

        FaultRecord first;
        REQUIRE(manager.first_fault(&first));
        CHECK(first.fault_sequence == root.fault_sequence);
        CHECK(first.source == FaultSource::ENCODER);
        CHECK(consequence.parent_fault_sequence == first.fault_sequence);
    }

    TEST_CASE("selective clear removes a fault and preserves active survivors") {
        FaultManager manager;
        auto first = make_record(FaultSource::MOTOR, FaultCode::MOTOR_ERROR,
                                 FaultSite::MOTOR_UPDATE, FaultSeverity::LATCHED, 11);
        auto second = make_record(FaultSource::POWER_STAGE,
                                  FaultCode::POWER_STAGE_SHUTDOWN,
                                  FaultSite::POWER_STAGE_ARM,
                                  FaultSeverity::IMMEDIATE_SHUTDOWN, 11);
        REQUIRE(manager.raise(first));
        REQUIRE(manager.raise(second));
        CHECK(manager.active_count() == 2);

        CHECK(manager.clear(ClearRequest{99, first.fault_sequence}));
        CHECK(manager.active_count() == 1);
        CHECK_FALSE(manager.clear(ClearRequest{100, second.fault_sequence}));

        FaultRecord retained;
        REQUIRE(manager.first_fault(&retained));
        CHECK(retained.state_epoch == 11);
        CHECK(retained.fault_sequence == second.fault_sequence);
        CHECK(manager.record_count() == 1);
    }

    TEST_CASE("duplicate active faults are coalesced") {
        FaultManager manager;
        auto first = make_record(FaultSource::ENCODER, FaultCode::FEEDBACK_MISSING,
                                 FaultSite::ENCODER_UPDATE,
                                 FaultSeverity::LATCHED, 2);
        REQUIRE(manager.raise(first));
        const uint32_t sequence = first.fault_sequence;

        auto repeated = first;
        repeated.fault_sequence = 12345;
        repeated.control_sequence = 43;
        repeated.timestamp_cycles = 172000;
        REQUIRE(manager.raise(repeated));
        CHECK(manager.record_count() == 1);
        CHECK(repeated.fault_sequence == sequence);
        CHECK(repeated.occurrence_count == 2);
        CHECK(repeated.last_control_sequence == 43);
        CHECK(repeated.last_timestamp_cycles == 172000);
    }

    TEST_CASE("clear reclaims capacity and starts a new first-fault epoch") {
        FaultManager manager;
        for (uint32_t cycle = 0; cycle < 96; ++cycle) {
            auto record = make_record(FaultSource::MOTOR, FaultCode::MOTOR_ERROR,
                                      FaultSite::MOTOR_UPDATE,
                                      FaultSeverity::LATCHED, cycle);
            record.arg0 = cycle + 1;
            REQUIRE(manager.raise(record));
            REQUIRE(manager.clear(ClearRequest{cycle, 0}));
            CHECK(manager.record_count() == 0);
            FaultRecord none;
            CHECK_FALSE(manager.first_fault(&none));
        }
        CHECK(manager.dropped_count() == 0);
    }

    TEST_CASE("overflow is explicit and does not evict the first active fault") {
        FaultManager manager;
        uint32_t first_sequence = 0;
        for (uint32_t index = 0; index < FaultManager::kMaxRecords; ++index) {
            auto record = make_record(FaultSource::MOTOR, FaultCode::MOTOR_ERROR,
                                      FaultSite::MOTOR_UPDATE,
                                      FaultSeverity::LATCHED, 4);
            record.arg0 = index + 1;
            REQUIRE(manager.raise(record));
            if (index == 0) {
                first_sequence = record.fault_sequence;
            }
        }

        auto overflow = make_record(FaultSource::ENCODER,
                                    FaultCode::ENCODER_ERROR,
                                    FaultSite::ENCODER_UPDATE,
                                    FaultSeverity::LATCHED, 4);
        CHECK_FALSE(manager.raise(overflow));
        CHECK(manager.dropped_count() == 1);
        CHECK(manager.record_count() == FaultManager::kMaxRecords);

        FaultRecord first;
        REQUIRE(manager.first_fault(&first));
        CHECK(first.fault_sequence == first_sequence);
    }
}
