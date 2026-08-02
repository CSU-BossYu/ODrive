#include <array>
#include <doctest.h>

#include "MotorControl/trace_rings.hpp"

using namespace odrive::fault;
using namespace odrive::trace;

namespace {

CriticalEvent critical(uint32_t sequence) {
    CriticalEvent event;
    event.sequence = sequence;
    event.control_sequence = sequence;
    event.code = FaultCode::CONTROLLER_REJECTED;
    event.site = FaultSite::CONTROLLER_UPDATE;
    event.severity = FaultSeverity::LATCHED;
    return event;
}

StateEvent state(uint32_t sequence) {
    StateEvent event;
    event.sequence = sequence;
    event.state_epoch = sequence;
    return event;
}

ScopeRecord scope(uint32_t sequence) {
    ScopeRecord record;
    record.sequence = sequence;
    record.phase = static_cast<float>(sequence);
    return record;
}

struct DispatchCapture {
    std::array<TraceEventKind, TraceConsumer::kBatchCapacity> kinds{};
    size_t count = 0;
};

void capture(void* context, const TraceDispatchRecord& record) {
    auto* output = static_cast<DispatchCapture*>(context);
    if (output->count < output->kinds.size()) {
        output->kinds[output->count++] = record.kind;
    }
}

}  // namespace

TEST_SUITE("TraceRings") {
    TEST_CASE("scope overflow cannot consume critical capacity") {
        ScopeRing scope_ring;
        CriticalEventRing critical_ring;
        for (size_t index = 0; index < ScopeRing::capacity(); ++index) {
            REQUIRE(scope_ring.push(scope(static_cast<uint32_t>(index))));
        }
        CHECK_FALSE(scope_ring.push(scope(999u)));
        CHECK(scope_ring.overflow_count() == 1u);
        REQUIRE(critical_ring.push_from_isr(critical(1u)));
        CriticalEvent event;
        REQUIRE(critical_ring.pop(&event));
        CHECK(event.sequence == 1u);
        CHECK(critical_ring.overflow_count() == 0u);
    }

    TEST_CASE("critical black box keeps pretrigger and bounded posttrigger") {
        CriticalBlackBox black_box;
        for (uint32_t sequence = 1u;
             sequence <= CriticalBlackBox::kCapacity + 3u; ++sequence) {
            REQUIRE(black_box.record_from_isr(critical(sequence)));
        }
        CHECK(black_box.freeze_on_first_fault());
        CHECK(black_box.frozen());
        for (uint32_t sequence = 100u;
             sequence < 100u + CriticalBlackBox::kPostTriggerCapacity;
             ++sequence) {
            REQUIRE(black_box.record_from_isr(critical(sequence)));
        }
        CHECK(black_box.post_trigger_remaining() == 0u);
        CHECK_FALSE(black_box.record_from_isr(critical(999u)));

        std::array<CriticalEvent, CriticalBlackBox::kCapacity> output{};
        size_t written = 0;
        REQUIRE(black_box.snapshot(output.data(), output.size(), &written));
        REQUIRE(written == CriticalBlackBox::kCapacity);
        CHECK(output.back().sequence == 115u);
    }

    TEST_CASE("task consumer prioritizes and sorts fixed records") {
        CriticalEventRing critical_ring;
        StateEventRing state_ring;
        LogRing log_ring;
        ScopeRing scope_ring;
        DispatchCapture capture_output;
        TraceConsumer consumer(critical_ring, state_ring, log_ring, scope_ring,
                               &capture_output, capture);

        REQUIRE(scope_ring.push(scope(3u)));
        REQUIRE(log_ring.push(LogRecord{2u}));
        REQUIRE(state_ring.push(state(1u)));
        REQUIRE(critical_ring.push_from_task(critical(4u)));
        consumer.consume();

        REQUIRE(capture_output.count == 4u);
        CHECK(capture_output.kinds[0] == TraceEventKind::CRITICAL);
        CHECK(capture_output.kinds[1] == TraceEventKind::STATE);
        CHECK(capture_output.kinds[2] == TraceEventKind::LOG);
        CHECK(capture_output.kinds[3] == TraceEventKind::SCOPE);
        CHECK(consumer.stats().dispatched_critical == 1u);
        CHECK(consumer.stats().dispatched_scope == 1u);
    }

    TEST_CASE("critical pressure degrades lower priority records") {
        CriticalEventRing critical_ring;
        StateEventRing state_ring;
        LogRing log_ring;
        ScopeRing scope_ring;
        for (size_t index = 0; index < TraceConsumer::kCriticalHighWatermark;
             ++index) {
            REQUIRE(critical_ring.push_from_task(
                critical(static_cast<uint32_t>(index + 1u))));
        }
        REQUIRE(scope_ring.push(scope(100u)));
        TraceConsumer consumer(critical_ring, state_ring, log_ring, scope_ring);
        consumer.consume();
        CHECK(consumer.stats().scope_degraded == 1u);
        CHECK(consumer.stats().degraded_records == 1u);
        CHECK(consumer.stats().dispatched_critical ==
              TraceConsumer::kCriticalHighWatermark);
    }
}
