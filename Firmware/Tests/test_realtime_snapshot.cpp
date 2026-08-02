#include <doctest.h>

#include "MotorControl/realtime_snapshot.hpp"

using namespace odrive::trace;

TEST_SUITE("RealtimeSnapshot") {
    TEST_CASE("triple buffer returns one complete published sequence") {
        RealtimeSnapshotBuffer buffer;
        RealtimeSnapshot read;
        CHECK_FALSE(buffer.read_consistent(&read));
        RealtimeSnapshot first;
        first.sequence = 10u;
        first.control_sequence = 10u;
        first.state_epoch = 3u;
        first.phase = 1.25f;
        first.iq_measured = 2.5f;
        buffer.publish_from_isr(first);

        REQUIRE(buffer.read_consistent(&read));
        CHECK(read.sequence == 10u);
        CHECK(read.state_epoch == 3u);
        CHECK(read.phase == doctest::Approx(1.25f));
        CHECK(read.iq_measured == doctest::Approx(2.5f));

        RealtimeSnapshot second = first;
        second.sequence = 11u;
        second.control_sequence = 11u;
        second.state_epoch = 4u;
        second.phase = -0.75f;
        second.iq_measured = -4.0f;
        buffer.publish_from_isr(second);
        REQUIRE(buffer.read_consistent(&read));
        CHECK(read.sequence == 11u);
        CHECK(read.control_sequence == 11u);
        CHECK(read.state_epoch == 4u);
        CHECK(read.phase == doctest::Approx(-0.75f));
        CHECK(read.iq_measured == doctest::Approx(-4.0f));
    }

    TEST_CASE("null snapshot read is rejected") {
        RealtimeSnapshotBuffer buffer;
        CHECK_FALSE(buffer.read_consistent(nullptr));
    }

    TEST_CASE("Supervisor trace state is published as one coherent value") {
        SupervisorTraceMailbox mailbox;
        SupervisorTraceState read;
        CHECK_FALSE(mailbox.read_from_isr(&read));
        mailbox.publish_from_task({7u, 4u, 1u, 0x1fu, 0u});
        REQUIRE(mailbox.read_from_isr(&read));
        CHECK(read.state_epoch == 7u);
        CHECK(read.safety_state == 4u);
        CHECK(read.operation == 1u);
        CHECK(read.readiness_flags == 0x1fu);
    }
}
