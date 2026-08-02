# Phase 2 evidence: SafetySupervisor and closed-loop transaction

Date: 2026-08-01

## Implemented boundary

- `SafetySupervisor` owns explicit BOOT, SAFE_OFF, PREPARING, READY, ARMED,
  STOPPING and FAULT_LATCHED transitions.
- Axis submits `BOOT_COMPLETE`, calls `tick()` from task context, consumes
  fixed-capacity `RealtimeRequest` objects, and returns matching epoch/sequence
  confirmations.
- Closed-loop startup is split into task-context PREPARE and ARM operations.
  PREPARE connects feedback and waits for coherent encoder/controller evidence;
  ARM enables the power stage only after READY. DISARM is likewise confirmed.
- Legacy CAN state 8 is translated into one supervised PREPARE -> READY -> ARM
  transaction. Legacy IDLE requests disarm through the Supervisor when an
  operation is preparing, ready, or armed. Other legacy states remain an
  explicit compatibility bridge to the old Axis state machine.
- CAN clear-errors is a Supervisor command. FaultManager and legacy
  Axis/Motor/Encoder/Controller projections are cleared atomically only when no
  non-clearable fault remains.
- The realtime event ring remains single-producer/single-consumer: the control
  ISR writes it and the Axis task drains it. Task-generated confirmations are
  delivered directly to the Supervisor and do not become a second producer.
- Ring overflow performs the existing immediate hardware-safe fallback and is
  additionally converted into a Supervisor-visible latched fault, preventing
  FaultManager and SafetyState divergence.
- Direct legacy/component faults are detected from FaultManager during
  Supervisor service and force FAULT_LATCHED plus motor disarm.
- Transition deadlines use wrap-safe modular sequence comparison.
- Command results are drained in Axis task context into the latest-result
  snapshot, so the bounded result ring does not fill permanently while the CAN
  compatibility protocol has no result frame.

## Correctness fixes after review

- Added the missing `READY + ARM_CONFIRMED -> ARMED` transition.
- Added PREPARING/READY disarm transitions.
- Added actual RealtimeRequest execution; the queue is no longer unconsumed
  scaffolding.
- Unified Supervisor clearing with legacy error clearing.
- Made event overflow visible to Supervisor state.
- Added timeout-wrap, legacy state-8 transaction, and overflow-latching tests.
- Extended the Phase 2 static gate to require `tick()`, request consumption,
  task-context confirmations, CAN clear routing, and the ARM confirmation rule.

## Verification

- `cmake --build --preset firmware-release -j 4`: passed;
  `text=146544`, `data=1416`, `bss=107232` bytes.
- `cmake --build --preset firmware-debug -j 4`: passed;
  `text=159592`, `data=1416`, `bss=107232` bytes.
- `python tools/check_phase2_static_gate.py`: passed.
- `python tools/check_legacy_error_writes.py`: passed with five audited legacy
  projections.
- `python -m unittest discover -s Firmware/Tests -p "test_*.py" -q`: 41 tests
  passed.
- `test_safety_supervisor.cpp` was compiled successfully as an ARM C++17 object
  with warnings enabled, including the new transaction and wrap tests.
- `git diff --check`: passed apart from Git's existing LF/CRLF notices.

## Evidence still open

1. Native CTest execution remains blocked by the missing/broken host MinGW C++
   compiler. ARM object compilation is not represented as a native test pass.
2. No hardware was connected. State 8, actual PWM enable, encoder readiness,
   CAN timing and safe-stop behavior still require HIL verification with the
   matching build manifest.
3. Calibration and self-test operations still use the explicit legacy bridge;
   only the closed-loop/IDLE product path is a complete Supervisor transaction.
4. Remaining component setters can still record faults directly. They now
   converge to FAULT_LATCHED during the 1 ms Axis service, but migration to an
   ISR critical-event ingress remains later work.
5. USB structured diagnostics and waveform transport are not part of Phase 2.
