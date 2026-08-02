# Phase 3 evidence: realtime snapshots, trace rings, and black box

Date: 2026-08-01
Workspace: `F:\project_source_code\ODrive`

## 1. Implemented scope

- `Firmware/MotorControl/realtime_snapshot.hpp/.cpp`
  - Fixed-size `RealtimeSnapshot` containing control/epoch/feedback sequence,
    encoder phase and position data, Id/Iq and setpoints, controller output,
    SafetySupervisor state/operation/readiness and timing counters.
  - Three-slot single-writer/single-reader publication. A reader claims the
    active slot and the writer selects a slot owned by neither side, so no
    non-atomic payload is read while it is being overwritten.
  - Supervisor state/operation/readiness is published from task context through
    a separate three-slot mailbox; the ISR never reads Supervisor-owned fields.
- `Firmware/MotorControl/trace_rings.hpp/.cpp`
  - Independent `CriticalEventRing`, `StateEventRing`, `LogRing`, and
    `ScopeRing` with fixed-size trivially-copyable records.
  - Usable capacities: Critical 64, State 64, Log 64, Scope 256.
  - Each ring exposes independent overflow and dropped-record counters.
  - Critical records use a bounded three-attempt MPSC enqueue because faults
    can be raised by the ISR or a task callback. No retry loop can block the
    ISR indefinitely.
  - `CriticalBlackBox` keeps a fixed pre-trigger window, freezes immediately at
    the first ISR fault, and permits 16 bounded post-trigger records. Axis
    critical sections serialize its multiword storage, reset, and snapshot.
  - `TraceConsumer` samples Critical pressure before draining, drains in task
    context, sorts by priority and sequence,
    records same-stream ordering violations, and degrades log/scope data when
    Critical pressure reaches its high-water mark.
- `Firmware/MotorControl/axis.hpp/.cpp` and `main.cpp`
  - Control callback publishes the snapshot and Scope record without USB/CAN,
    formatting, allocation, blocking, or RTOS waits.
  - Fault paths publish structured Critical records; an ISR-recorded fault is
    not duplicated when Supervisor later projects it into FaultManager. Axis
    task drains state and command-result records through the TraceConsumer.
  - Existing Phase 2 CAN -> Supervisor -> PREPARE -> READY -> ARM -> confirmation
    flow remains in place.
- `Firmware/CMakeLists.txt`, `Firmware/Tests/CMakeLists.txt`
  - Added Phase 3 sources, CTest test sources, and `phase3_static_gate`.
- `Firmware/Tests/test_realtime_snapshot.cpp` and
  `Firmware/Tests/test_trace_rings.cpp`
  - Cover sequence-consistent snapshots, null-reader rejection, independent
    ring overflow, black-box pre/post trigger, priority sorting and lower
    priority degradation under Critical pressure.
- `tools/check_phase3_static_gate.py`
  - Checks the fixed record boundary, forbidden ISR APIs, independent ring
    contracts, snapshot/consumer integration, pressure-before-drain ordering,
    immediate first-fault freeze, and Phase 2 transaction symbols.

## 2. Review corrections

- Replaced the two-slot seqlock payload copy with a race-free triple-buffer
  ownership protocol and rejected reads before first publication.
- Removed direct ISR reads of task-owned SafetySupervisor fields.
- Serialized CriticalBlackBox payload access and changed post-trigger
  reservation to a non-underflowing CAS operation.
- Moved Critical high-watermark sampling before the consumer drains the ring.
- Freeze now occurs at ISR fault ingress; the later Supervisor projection is
  marked as already traced and is not recorded twice.
- `isr_cycles` now reports the preceding complete callback, including Phase 3
  snapshot/scope publication and the final error-GPIO operation.

Existing uncommitted changes and `Firmware/Tests/hex_4342_mt6826s/dual_encoder_motion.csv`
were preserved. No reset, checkout, deletion, or overwrite of user changes was
performed.

## 3. Verification

| Check | Command | Result |
|---|---|---|
| ARM Release | `cmake --build --preset firmware-release -j4` | PASS; `text=151680 data=1416 bss=133912` |
| ARM Debug | `cmake --build --preset firmware-debug -j4` | PASS; `text=164440 data=1416 bss=133912` |
| Phase 2 gate | `python tools/check_phase2_static_gate.py` | PASS |
| Phase 3 gate | `python tools/check_phase3_static_gate.py` | PASS |
| Legacy error gate | `python tools/check_legacy_error_writes.py` | PASS; 5 audited projections |
| Python regression | `python -m unittest discover -s Firmware/Tests -p "test_*.py"` | PASS; 41 tests |
| C++ test source compile check | ARM `arm-none-eabi-g++ -std=c++17 -Wall -Wextra -Wpedantic -c` for both new test files | PASS; compile-only, not a native test result |
| Diff check | `git diff --check` | PASS apart from existing LF/CRLF notices |

The firmware build executes `generate_fault_schema`, `build_manifest`,
`fault_write_gate`, `phase2_static_gate`, and `phase3_static_gate`. The final
manifest JSON files are:

- `Firmware/build/firmware-release/build_manifest.json`
- `Firmware/build/firmware-debug/build_manifest.json`

They remain dirty-worktree manifests by design and contain the compiler
identity, configuration, schema hash, dirty hash, and build ID.

## 4. Native CTest status

Native CTest was attempted and remains blocked by the known host toolchain
failure. `cmake --preset host-debug` references the missing
`E:/Qt5.14.2/Tools/mingw730_32/bin/g++.exe`; `ctest --preset host-debug`
reports `No tests were found!!!`. No native CTest, ASan, or UBSan pass is
claimed. ARM compile-only checks are not being used as a substitute.

## 5. Constraint and risk status

- The four rings are separate, so Scope/Log overflow cannot overwrite Critical
  storage. Critical pressure intentionally drains lower-priority records in
  task context and increments degradation statistics.
- New trace and snapshot code contains no USB/CAN calls, formatting,
  allocation, blocking API, RTOS wait, or `volatile bool` protocol.
- No CAN debug item was added.
- The Phase 2 transaction remains intact; the two unverified
  `closed_loop_phase_feedback_ready_` and
  `closed_loop_controller_ready_` experiment variables were not promoted to
  the Phase 3 protocol.
- `RealtimeSnapshot.isr_cycles` records the preceding complete control callback,
  but no device run was available to establish or prove a worst-case budget.
- No device was connected. USB disconnect/blocked behavior, 10-minute control
  stability, state 8 behavior, PWM safety, and motor-motion quality remain
  unverified. This phase does not claim any hardware or motion problem is
  solved.
- The Phase 3 consumer currently has no USB transport callback; it consumes,
  orders, degrades, and counts records locally. USB protocol integration is a
  later phase and must not be added to the ISR path.
