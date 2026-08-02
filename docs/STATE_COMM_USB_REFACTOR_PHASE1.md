# Phase 1 evidence: unified fault model

Date: 2026-08-01

## Implemented

- Added a fixed-capacity `FaultManager` for active faults. It assigns fault
  sequences, preserves the first active root fault, coalesces identical faults,
  counts repeats, reports overflow, and reclaims slots when faults are cleared.
- Axis wrappers serialize manager access and legacy projection in a target
  critical section. This is the Phase 1 single-core synchronization boundary;
  ISR event ingestion is still deferred to the supervisor phase.
- Migrated watchdog, invalid state, command timeout, controller, encoder,
  motor, closed-loop controller rejection, and CAN emergency-stop reporting to
  the unified entry point. Legacy error fields remain compatibility projections.
- Made whole-axis clearing atomic with manager clearing. If a non-clearable
  immediate-shutdown record remains, legacy fields are conservatively retained.
  Clearing LZ5710 transport latches no longer resets the Vernier estimator.
- Added `docs/fault_schema.json` as the source for generated C++, Python, and
  TypeScript enums. All three outputs are dependencies of the CMake generation
  target.
- Replaced the count-based legacy-write check with a line-level gate: direct
  `error_ |=` writes in MotorControl and CAN must carry an audited compatibility
  projection marker.
- Build manifests now contain a non-empty configuration, compiler identity,
  fault-schema hash, dirty-worktree hash (including untracked file contents),
  and derived build ID.
- Native-test CMake now compiles `fault_manager.cpp`; host-only configuration
  requests only the C++ language instead of also requiring embedded C/ASM.

The pre-existing `closed_loop_phase_feedback_ready_` and
`closed_loop_controller_ready_` experiment remains unverified user work. It is
preserved, but is not considered a stable state-machine interface in this phase.

## Verification

- `cmake --build --preset firmware-release -j 4`: passed.
  `text=140176`, `data=1416`, `bss=104824` bytes.
- `cmake --build --preset firmware-debug -j 4`: passed.
  `text=154460`, `data=1416`, `bss=104824` bytes.
- `python tools/check_legacy_error_writes.py`: passed with five audited
  compatibility projections.
- `python -m unittest discover -s Firmware/Tests -p "test_*encoder*.py" -v`:
  41 tests passed.
- Build manifests report `firmware-release` and `firmware-debug` rather than an
  empty generator configuration.
- `git diff --check`: passed; Git reports only the repository's existing
  LF-to-CRLF conversion warnings.

## Native test status

`test_fault_manager.cpp` covers first-fault selection, parent linkage,
selective/non-clearable clearing, duplicate coalescing, repeated slot reuse, and
overflow without first-fault eviction. It is registered with CTest, but it was
not executable on this machine: the discovered MinGW `gcc.exe` exits with
Windows error `-1073741515`, and the configured `g++.exe` path does not exist.
No native CTest, ASan, or UBSan pass is claimed.

## Open evidence and later-phase work

1. No connected-device run was performed, so state-8 behavior, CAN captures,
   cycle cost, and build-ID-matched USB fault output remain unverified.
2. Production callers do not yet populate state epochs or a multi-record causal
   parent chain. Those require the supervisor/state-machine boundary rather than
   inventing ancestry in component setters.
3. `FaultManager` currently protects target access through short interrupt-off
   sections. A bounded ISR event ring and task-context supervisor remain the
   intended architecture before richer USB tracing is added.
4. USB diagnostics/waveform transport and CAN protocol reduction are not Phase
   1 deliverables and have not been implemented here.
5. A working host C++ toolchain is still required to close native sanitizer
   evidence.
