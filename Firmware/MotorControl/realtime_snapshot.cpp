#include "realtime_snapshot.hpp"

#include <type_traits>

namespace odrive::trace {

static_assert(std::is_trivially_copyable<RealtimeSnapshot>::value,
              "RealtimeSnapshot must remain a fixed-copy ISR record");
static_assert(sizeof(RealtimeSnapshot) <= 128u,
              "RealtimeSnapshot exceeds the Phase 3 fixed-size budget");

void RealtimeSnapshotBuffer::publish_from_isr(
        const RealtimeSnapshot& snapshot) {
    buffer_.publish(snapshot);
    published_sequence_.store(snapshot.sequence, std::memory_order_release);
}

bool RealtimeSnapshotBuffer::read_consistent(
        RealtimeSnapshot* snapshot) const {
    return buffer_.read(snapshot);
}

}  // namespace odrive::trace
