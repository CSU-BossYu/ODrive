#include "realtime_event_ring.hpp"

namespace odrive::safety {

static_assert(RealtimeEventRing::kCapacity == 32,
              "the ISR event ring capacity is part of the Phase 2 contract");

}  // namespace odrive::safety
