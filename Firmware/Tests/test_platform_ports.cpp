#include <doctest.h>

#include "MotorControl/platform_ports.hpp"

namespace {

struct FakePlatform {
    uint32_t sensor_reads = 0u;
    uint32_t pwm_writes = 0u;
    uint32_t disarms = 0u;
    bool fail_power_stage = false;
};

bool read_sensor(void* context, bool coherent_pair,
                 odrive::platform::AbsoluteSensorFrame* frame) {
    auto& fake = *static_cast<FakePlatform*>(context);
    ++fake.sensor_reads;
    frame->main = {123u, 0u, fake.sensor_reads, true};
    frame->auxiliary = {456u, 0u, fake.sensor_reads, coherent_pair};
    frame->sequence = fake.sensor_reads;
    frame->coherent_pair = coherent_pair;
    frame->valid = true;
    return true;
}

bool write_pwm(void* context, const uint16_t timings[3], bool) {
    auto& fake = *static_cast<FakePlatform*>(context);
    ++fake.pwm_writes;
    return !fake.fail_power_stage && timings[0] <= timings[1] &&
           timings[1] <= timings[2];
}

bool disarm(void* context) {
    auto& fake = *static_cast<FakePlatform*>(context);
    ++fake.disarms;
    return !fake.fail_power_stage;
}

}  // namespace

TEST_SUITE("PlatformPorts") {
    TEST_CASE("sensor source preserves explicit coherent-pair evidence") {
        FakePlatform fake;
        const odrive::platform::SensorSourcePort sensor{&fake, read_sensor};
        odrive::platform::AbsoluteSensorFrame main{};
        REQUIRE(sensor.sample(false, &main));
        CHECK(main.main.angle == 123u);
        CHECK_FALSE(main.coherent_pair);
        odrive::platform::AbsoluteSensorFrame pair{};
        REQUIRE(sensor.sample(true, &pair));
        CHECK(pair.auxiliary.angle == 456u);
        CHECK(pair.coherent_pair);
        CHECK(fake.sensor_reads == 2u);
    }

    TEST_CASE("power stage failure is observable and disarm is explicit") {
        FakePlatform fake;
        const odrive::platform::PowerStagePort power{
            &fake, write_pwm, disarm};
        const uint16_t timings[3] = {10u, 20u, 30u};
        CHECK(power.apply(timings, true));
        CHECK(power.force_disarm());
        fake.fail_power_stage = true;
        CHECK_FALSE(power.apply(timings, false));
        CHECK_FALSE(power.force_disarm());
        CHECK(fake.pwm_writes == 2u);
        CHECK(fake.disarms == 2u);
    }

    TEST_CASE("unbound ports fail closed") {
        const odrive::platform::SensorSourcePort sensor{};
        const odrive::platform::PowerStagePort power{};
        odrive::platform::AbsoluteSensorFrame frame{};
        const uint16_t timings[3] = {};
        CHECK_FALSE(sensor.sample(false, &frame));
        CHECK_FALSE(power.apply(timings, false));
        CHECK_FALSE(power.force_disarm());
    }
}
