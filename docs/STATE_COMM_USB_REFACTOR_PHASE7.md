# Phase 7: Product CAN Boundary

## Outcome

Phase 7 makes CAN a bounded product-control transport and keeps detailed
diagnosis on framed USB. It does not redesign calibration or motor-control
algorithms.

`docs/can_product_protocol_schema.json` is the single source of truth for CAN
message IDs, protocol version, ownership, lifecycle, and the representative
bus-load profile. C++, Python, and TypeScript contracts are generated from it.
The build fails when a generated contract is stale or the representative load
exceeds 30 percent at 1 Mbit/s.

## Product path

- `MANAGEMENT_COMMAND` (`0x08`) carries a request ID and a typed supervisor
  command.
- `COMMAND_ACK` (`0x05`) returns accepted, completed, rejected, or failed
  results with the supervisor epoch and reason.
- CAN management is submitted to `SafetySupervisor`; it does not mutate axis
  state directly.
- `PRODUCT_STATUS` (`0x10`) publishes the fault summary, safety state, active
  operation, readiness flags, and timeout flags.
- Setpoints and bounded operational telemetry remain on CAN.

The fixed Axis result fan-out permits USB and CAN to observe the same command
result without either transport owning the state machine callback.

## Diagnostic boundary

Detailed motor, encoder, controller, raw-count, Vernier, calibration, and fault
snapshot requests are not periodic product traffic. Normal backend polling no
longer requests them. Fault chronology, crash reports, exact source-location
symbolization, and waveform capture remain USB-only.

Legacy on-request CAN diagnostics and command `0x1E` are retained behind the
explicit `legacy_extended_command_callback` compatibility boundary so existing
bench scripts are not broken before calibration/control redesign. No new
feature may be added to that dispatcher. Its accepted subcommand set is frozen
by the Phase 7 gate.

## Compatibility correction

The former implicit C++ enum skipped no placeholder at `0x10`, shifting later
firmware IDs relative to Python and documentation. All IDs are now explicit
generated values. `PRODUCT_STATUS` occupies `0x10`; trajectory velocity starts
at `0x11`, and bus voltage/current is `0x17`.

## Verification and hardware boundary

The representative profile is 1230 eight-byte frames/s. Using a conservative
135 bits/frame estimate, utilization is 16.61 percent at 1 Mbit/s.

Host/unit/static/build verification can establish contract and implementation
correctness. A later, explicit hardware acceptance must verify ACK retry,
duplicate request rejection, status cadence, timeout behavior, and observed bus
load. Firmware must not be flashed and motor commands must not be issued as an
implicit part of this phase.

## First hardware acceptance findings

Build `2ae8760420f00705` verified USB identity, Product Status, ACK correlation,
diagnostic traffic removal, and Scope capture. It also exposed two state-machine
semantics that were corrected before final acceptance:

- DISARM is now idempotent in SAFE_OFF and FAULT_LATCHED. It completes without
  creating a realtime request or clearing a retained fault.
- A clean closed-loop preparation rejection now produces
  `PREPARE_REJECTED`, returns the accepted command as FAILED with
  `CONTROLLER_REJECTED`, undoes partial preparation, and returns to SAFE_OFF
  without inserting a fault record. Existing component errors, realtime queue
  failures, arm failures, and transition timeouts remain lockable faults.
- Control timeout checks are inactive outside an enabled control interval.
  Stopping clears runtime timeout state, and a successful arm establishes a
  fresh command-watchdog grace period. Product Status therefore cannot expose
  stale idle timeout flags.
