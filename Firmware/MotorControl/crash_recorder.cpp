#include "crash_recorder.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"
#include "stm32f4xx.h"
#include "usb_debug_identity.hpp"

namespace odrive::crash {
namespace {

struct CompactCriticalEvent {
    uint32_t sequence;
    uint32_t control_sequence;
    uint32_t timestamp_cycles;
    uint32_t source_code;
    uint32_t site_severity;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
};

struct CrashRecord {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t crc;
    uint32_t generation;
    uint32_t fault_kind;
    uint32_t flags;
    std::array<uint8_t, 8> build_id;
    std::array<uint8_t, 16> manifest_identity;
    std::array<uint32_t, 8> stacked;
    uint32_t exception_return;
    uint32_t msp;
    uint32_t psp;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t icsr;
    uint32_t shcsr;
    uint32_t control_sequence;
    uint32_t state_epoch;
    uint32_t safety_operation;
    uint32_t active_irq;
    uint32_t critical_count;
    std::array<CompactCriticalEvent, kCriticalEventCount> critical;
};

static_assert(sizeof(CrashRecord) == 212u,
              "retained crash record layout changed unexpectedly");

struct RuntimeContext {
    volatile uint32_t sequence = 0u;
    volatile uint32_t control_sequence = 0u;
    volatile uint32_t state_epoch = 0u;
    volatile uint32_t safety_operation = 0u;
};

struct CriticalMirror {
    volatile uint32_t sequence_lock = 0u;
    volatile uint32_t write_count = 0u;
    std::array<CompactCriticalEvent, kCriticalEventCount> events{};
};

// Startup deliberately does not initialize this section. It survives a
// Cortex system reset, but is not claimed to survive power loss.
__attribute__((section(".noinit.crash"), used, aligned(4)))
CrashRecord retained_record;

RuntimeContext runtime_context;
CriticalMirror critical_mirror;

constexpr size_t kCrcOffset = offsetof(CrashRecord, crc);

uint32_t crc32_step(uint32_t crc, uint8_t value) {
    crc ^= value;
    for (uint32_t bit = 0u; bit < 8u; ++bit) {
        crc = (crc >> 1u) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return crc;
}

uint32_t calculate_record_crc(const CrashRecord& record) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&record);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t index = 0u; index < sizeof(record); ++index) {
        uint8_t value = bytes[index];
        if (index < sizeof(uint32_t)) {
            value = static_cast<uint8_t>(kRecordMagic >> (8u * index));
        } else if (index >= kCrcOffset &&
                   index < kCrcOffset + sizeof(uint32_t)) {
            value = 0u;
        }
        crc = crc32_step(crc, value);
    }
    return crc ^ 0xFFFFFFFFu;
}

bool valid_record() {
    return retained_record.magic == kRecordMagic &&
           retained_record.version == kRecordVersion &&
           retained_record.size == sizeof(CrashRecord) &&
           retained_record.crc == calculate_record_crc(retained_record);
}

bool valid_stack_range(const uint32_t* stack) {
    const uintptr_t start = reinterpret_cast<uintptr_t>(stack);
    const uintptr_t end = start + 8u * sizeof(uint32_t);
    const bool system_sram = start >= 0x20000000u && end <= 0x20020000u;
    const bool ccm_sram = start >= 0x10000000u && end <= 0x10010000u;
    return (start % alignof(uint32_t)) == 0u && (system_sram || ccm_sram);
}

void put_u16(uint8_t* output, size_t* offset, uint16_t value) {
    output[(*offset)++] = static_cast<uint8_t>(value);
    output[(*offset)++] = static_cast<uint8_t>(value >> 8u);
}

void put_u32(uint8_t* output, size_t* offset, uint32_t value) {
    for (size_t byte = 0u; byte < 4u; ++byte) {
        output[(*offset)++] = static_cast<uint8_t>(value >> (8u * byte));
    }
}

}  // namespace

bool pending() {
    return valid_record();
}

bool pending_crc(uint32_t record_crc) {
    return valid_record() && retained_record.crc == record_crc;
}

bool clear_if_crc(uint32_t record_crc) {
    if (!valid_record() || retained_record.crc != record_crc) {
        return false;
    }
    retained_record.magic = 0u;
    __DMB();
    return true;
}

size_t encode_pending(uint8_t* output, size_t capacity,
                      uint32_t* record_crc) {
    if (output == nullptr || capacity < kEncodedPayloadSize ||
        !valid_record()) {
        return 0u;
    }
    size_t offset = 0u;
    put_u16(output, &offset, static_cast<uint16_t>(retained_record.version));
    put_u16(output, &offset, static_cast<uint16_t>(kEncodedPayloadSize));
    put_u32(output, &offset, retained_record.crc);
    put_u32(output, &offset, retained_record.generation);
    put_u32(output, &offset, retained_record.fault_kind);
    put_u32(output, &offset, retained_record.flags);
    for (uint8_t value : retained_record.build_id) output[offset++] = value;
    for (uint8_t value : retained_record.manifest_identity) output[offset++] = value;
    for (uint32_t value : retained_record.stacked) put_u32(output, &offset, value);
    put_u32(output, &offset, retained_record.exception_return);
    put_u32(output, &offset, retained_record.msp);
    put_u32(output, &offset, retained_record.psp);
    put_u32(output, &offset, retained_record.cfsr);
    put_u32(output, &offset, retained_record.hfsr);
    put_u32(output, &offset, retained_record.dfsr);
    put_u32(output, &offset, retained_record.afsr);
    put_u32(output, &offset, retained_record.mmfar);
    put_u32(output, &offset, retained_record.bfar);
    put_u32(output, &offset, retained_record.icsr);
    put_u32(output, &offset, retained_record.shcsr);
    put_u32(output, &offset, retained_record.control_sequence);
    put_u32(output, &offset, retained_record.state_epoch);
    put_u32(output, &offset, retained_record.safety_operation);
    put_u32(output, &offset, retained_record.active_irq);
    put_u32(output, &offset, retained_record.critical_count);
    for (const auto& event : retained_record.critical) {
        put_u32(output, &offset, event.sequence);
        put_u32(output, &offset, event.control_sequence);
        put_u32(output, &offset, event.timestamp_cycles);
        put_u32(output, &offset, event.source_code);
        put_u32(output, &offset, event.site_severity);
        put_u32(output, &offset, event.arg0);
        put_u32(output, &offset, event.arg1);
        put_u32(output, &offset, event.arg2);
    }
    if (record_crc != nullptr) *record_crc = retained_record.crc;
    return offset == kEncodedPayloadSize ? offset : 0u;
}

}  // namespace odrive::crash

extern "C" void crash_recorder_update_context(
        uint32_t control_sequence, uint32_t state_epoch,
        uint8_t safety_state, uint8_t operation) {
    using namespace odrive::crash;
    ++runtime_context.sequence;
    __DMB();
    runtime_context.control_sequence = control_sequence;
    runtime_context.state_epoch = state_epoch;
    runtime_context.safety_operation =
        static_cast<uint32_t>(safety_state) |
        (static_cast<uint32_t>(operation) << 8u);
    __DMB();
    ++runtime_context.sequence;
}

extern "C" void crash_recorder_note_critical(
        uint32_t sequence, uint32_t control_sequence,
        uint32_t timestamp_cycles, uint16_t source, uint16_t code,
        uint16_t site, uint8_t severity, uint32_t arg0,
        uint32_t arg1, uint32_t arg2) {
    using namespace odrive::crash;
    ++critical_mirror.sequence_lock;
    __DMB();
    const uint32_t write = critical_mirror.write_count++;
    CompactCriticalEvent& event =
        critical_mirror.events[write % kCriticalEventCount];
    event.sequence = sequence;
    event.control_sequence = control_sequence;
    event.timestamp_cycles = timestamp_cycles;
    event.source_code = static_cast<uint32_t>(source) |
                        (static_cast<uint32_t>(code) << 16u);
    event.site_severity = static_cast<uint32_t>(site) |
                         (static_cast<uint32_t>(severity) << 16u);
    event.arg0 = arg0;
    event.arg1 = arg1;
    event.arg2 = arg2;
    __DMB();
    ++critical_mirror.sequence_lock;
}

extern "C" __attribute__((noreturn)) void crash_recorder_capture(
        const uint32_t* exception_stack, uint32_t exception_return,
        uint32_t fault_kind) {
    using namespace odrive::crash;
    __disable_irq();

    // Hardware-safe shutdown that does not depend on C++ object integrity.
    TIM1->BDTR &= ~(TIM_BDTR_AOE_Msk | TIM_BDTR_MOE_Msk);
    TIM8->BDTR &= ~(TIM_BDTR_AOE_Msk | TIM_BDTR_MOE_Msk);
    TIM2->CCR3 = 0u;
    TIM2->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1u;
    __DSB();

    const uint32_t previous_generation = valid_record()
        ? retained_record.generation : 0u;
    retained_record.magic = 0u;
    retained_record.version = kRecordVersion;
    retained_record.size = sizeof(CrashRecord);
    retained_record.crc = 0u;
    retained_record.generation = previous_generation + 1u;
    retained_record.fault_kind = fault_kind;
    retained_record.flags = 0u;
    retained_record.build_id = odrive::usb::identity::kBuildId;
    retained_record.manifest_identity =
        odrive::usb::identity::kManifestIdentity;

    const bool extended_frame = (exception_return & (1u << 4u)) == 0u;
    if (extended_frame) {
        retained_record.flags |= EXTENDED_FP_FRAME;
        exception_stack += 18u;
    }
    if (valid_stack_range(exception_stack)) {
        for (size_t index = 0u; index < retained_record.stacked.size(); ++index) {
            retained_record.stacked[index] = exception_stack[index];
        }
        retained_record.flags |= STACK_FRAME_VALID;
    } else {
        retained_record.stacked = {};
    }

    retained_record.exception_return = exception_return;
    retained_record.msp = __get_MSP();
    retained_record.psp = __get_PSP();
    retained_record.cfsr = SCB->CFSR;
    retained_record.hfsr = SCB->HFSR;
    retained_record.dfsr = SCB->DFSR;
    retained_record.afsr = SCB->AFSR;
    retained_record.mmfar = SCB->MMFAR;
    retained_record.bfar = SCB->BFAR;
    retained_record.icsr = SCB->ICSR;
    retained_record.shcsr = SCB->SHCSR;
    retained_record.active_irq = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;

    const uint32_t context_before = runtime_context.sequence;
    __DMB();
    retained_record.control_sequence = runtime_context.control_sequence;
    retained_record.state_epoch = runtime_context.state_epoch;
    retained_record.safety_operation = runtime_context.safety_operation;
    __DMB();
    const uint32_t context_after = runtime_context.sequence;
    if (context_before == context_after && (context_before & 1u) == 0u) {
        retained_record.flags |= CONTEXT_VALID;
    }

    const uint32_t critical_before = critical_mirror.sequence_lock;
    __DMB();
    const uint32_t count = critical_mirror.write_count;
    retained_record.critical_count =
        count < kCriticalEventCount ? count : kCriticalEventCount;
    for (size_t index = 0u; index < kCriticalEventCount; ++index) {
        const uint32_t source_index = count >= kCriticalEventCount
            ? (count + index) % kCriticalEventCount : index;
        retained_record.critical[index] = critical_mirror.events[source_index];
    }
    __DMB();
    const uint32_t critical_after = critical_mirror.sequence_lock;
    if (critical_before == critical_after && (critical_before & 1u) == 0u) {
        retained_record.flags |= CRITICAL_EVENTS_VALID;
    } else {
        retained_record.critical_count = 0u;
    }

    retained_record.crc = calculate_record_crc(retained_record);
    __DMB();
    retained_record.magic = kRecordMagic;
    __DSB();

    NVIC_SystemReset();
    for (;;) __NOP();
}
