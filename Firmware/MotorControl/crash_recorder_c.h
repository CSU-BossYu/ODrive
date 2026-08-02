#ifndef ODRIVE_CRASH_RECORDER_C_H
#define ODRIVE_CRASH_RECORDER_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CRASH_FAULT_KIND_HARD = 1,
    CRASH_FAULT_KIND_MEM_MANAGE = 2,
    CRASH_FAULT_KIND_BUS = 3,
    CRASH_FAULT_KIND_USAGE = 4,
};

void crash_recorder_update_context(uint32_t control_sequence,
                                   uint32_t state_epoch,
                                   uint8_t safety_state,
                                   uint8_t operation);
void crash_recorder_note_critical(uint32_t sequence,
                                  uint32_t control_sequence,
                                  uint32_t timestamp_cycles,
                                  uint16_t source,
                                  uint16_t code,
                                  uint16_t site,
                                  uint8_t severity,
                                  uint32_t arg0,
                                  uint32_t arg1,
                                  uint32_t arg2);
__attribute__((noreturn))
void crash_recorder_capture(const uint32_t* exception_stack,
                            uint32_t exception_return,
                            uint32_t fault_kind);

#ifdef __cplusplus
}
#endif

#endif
