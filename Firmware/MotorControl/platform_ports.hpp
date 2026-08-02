#ifndef ODRIVE_PLATFORM_PORTS_HPP
#define ODRIVE_PLATFORM_PORTS_HPP

#include <cstddef>
#include <cstdint>

namespace odrive::platform {

// Non-owning fixed function tables keep ISR paths allocation-free while
// allowing deterministic native fakes at the hardware boundary.
uint32_t cycle_count();

struct AbsoluteSensorSample {
    uint16_t angle = 0u;
    uint8_t status = 0u;
    uint32_t sequence = 0u;
    bool valid = false;
};

struct AbsoluteSensorFrame {
    AbsoluteSensorSample main{};
    AbsoluteSensorSample auxiliary{};
    uint32_t sequence = 0u;
    bool coherent_pair = false;
    bool valid = false;
};

using SensorReadFn = bool (*)(void*, bool, AbsoluteSensorFrame*);
struct SensorSourcePort {
    void* context = nullptr;
    SensorReadFn read = nullptr;
    bool sample(bool coherent_pair, AbsoluteSensorFrame* frame) const {
        return read != nullptr && frame != nullptr &&
               read(context, coherent_pair, frame);
    }
};

using PwmWriteFn = bool (*)(void*, const uint16_t[3], bool);
using PowerStageDisarmFn = bool (*)(void*);
struct PowerStagePort {
    void* context = nullptr;
    PwmWriteFn write_pwm = nullptr;
    PowerStageDisarmFn disarm = nullptr;
    bool apply(const uint16_t timings[3], bool enable_on_update) const {
        return write_pwm != nullptr && timings != nullptr &&
               write_pwm(context, timings, enable_on_update);
    }
    bool force_disarm() const {
        return disarm != nullptr && disarm(context);
    }
};

using NvmReadFn = bool (*)(void*, size_t, uint8_t*, size_t);
using NvmWriteFn = bool (*)(void*, const uint8_t*, size_t);
struct NvmPort {
    void* context = nullptr;
    NvmReadFn read = nullptr;
    NvmWriteFn atomic_write = nullptr;
};

using TracePublishFn = bool (*)(void*, const void*, size_t);
struct TraceSinkPort {
    void* context = nullptr;
    TracePublishFn publish = nullptr;
};

}  // namespace odrive::platform

#endif
