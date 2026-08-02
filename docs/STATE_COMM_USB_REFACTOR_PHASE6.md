# State/Communication USB Refactor - Phase 6 evidence

Date: 2026-08-02
Workspace: `F:\project_source_code\ODrive`
Scope: retained Cortex-M crash recording, exact-build symbolization, and UI
inspection. Calibration and motor-control behavior are intentionally excluded.

## Implementation

The STM32 exception handlers for HardFault, MemManage, BusFault, and UsageFault
are naked wrappers which select the faulting MSP/PSP stack and tail-call a
pure-C capture boundary. Capture performs a direct TIM1/TIM8 power-stage and
TIM2 brake-output shutdown, validates the stack address and exception-frame
shape, copies core/fault registers plus the latest state context and two
critical events into `.noinit.crash`, commits CRC/magic last, and requests a
Cortex system reset. It does not call USB, CAN, RTOS, formatting, allocation,
or flash-write APIs.

The retained record is a fixed 212-byte in-memory object with an explicit
204-byte little-endian USB payload. It contains an independent crashing build
ID and manifest identity, generation, fault kind, stacked R0-R3/R12/LR/PC/xPSR,
EXC_RETURN/MSP/PSP, SCB fault registers, active IRQ, control sequence, state
epoch, SafetyState/Operation/readiness, and the two most recent compact
critical events. Retention is guaranteed across Cortex system reset only, not
power loss.

After a valid HELLO/CAPABILITIES session, firmware publishes `CRASH_REPORT`.
The host acknowledges the record CRC with `CRASH_ACK`; firmware clears only a
matching retained record and only after the acknowledgement response has been
queued. Capability bit 8 advertises this feature. Protocol release is `4.4`
and schema version is `3`.

Every firmware build archives its ELF, map, manifest, and fault/protocol/scope
schemas under `Firmware/build/artifacts/<build_id>/`. The backend symbolizes PC
and LR only when both build ID and manifest identity exactly match an archived
ELF whose embedded identities also match. Otherwise it shows raw addresses and
an explicit unreliable-symbolization reason; it never silently uses the
current or nearest ELF. The frontend Crash Inspector displays the retained
register/state context and exact source location when available.

## Verification

The Phase 6 static gate verifies exception routing, direct shutdown, retained
linker placement, CRC/identity fields, forbidden fault-path calls, protocol
messages, artifact archiving, and exact-match symbolizer policy. The release
link map places the retained reboot cookie and crash record in the explicit
NOLOAD `.noinit` output section. A local exact-artifact symbolizer check
resolved a known firmware address to `crash_recorder_capture` and its source
line, demonstrating the archive/toolchain lookup path without GDB.

The final build/test counts and identity are authoritative in
`Firmware/build/build_manifest.json` and the matching artifact directory; they
are generated after this document is included in the dirty-worktree identity.

## Open hardware acceptance

No deliberate hardware exception has been injected. Phase 6 therefore still
needs one controlled, power-stage-safe acceptance run on the final flashed
image: inject a test UsageFault/HardFault from task context, observe reset,
reconnect USB, verify exact PC/LR source symbolization in the UI, acknowledge
the record, and verify it is not replayed after the next reconnect. This is a
destructive reset test and must be explicitly authorized before execution.
