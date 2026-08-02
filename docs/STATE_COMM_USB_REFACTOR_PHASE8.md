# Phase 8: Test Architecture

## Scope and current status

Phase 8 is accepted. It establishes executable native seams, removes the
motor-specific Python compatibility module, and adds repeatable offline and
read-only hardware qualification. It does not change calibration, estimator,
or closed-loop control algorithms.

## Shared Python boundary

`Firmware/Tests/odrive_hil` now owns the shared CANSimple codec and HIL lifecycle.
Product CAN IDs and management codecs are generated into this package from the
same JSON contract used by firmware, backend, and frontend. All direct-run bench
scripts import this shared package through a path-only bootstrap.
`hex_4342_mt6826s/common.py` has been deleted, and the Phase 8 gate rejects its
file or imports if they return.

The Phase 8 static gate rejects every script that instantiates `python-can`
directly. The former position- and velocity-step exceptions now use
`HilSession`, require explicit hardware-estop confirmation, respect the shared
hard ceilings, enter closed loop through correlated product management, and
close through the shared safe-stop sequence.

## Fail-closed HIL lifecycle

`HilSession` requires an explicit independent emergency-stop confirmation before
opening a motion session. It validates fixed hard ceilings (2 rev/s, 1 A,
0.2 Nm, 120 s), applies the selected velocity/current limits, maintains a session
deadline, and performs best-effort zero torque, zero velocity, product DISARM,
legacy IDLE, and adapter shutdown on every exit path.

`repro_ui_velocity_stream.py` is the first migrated motion script. Its former
defaults of 5 rev/s, 3 A, and 10 rev/s limit have been replaced by conservative
0.3 rev/s, 0.5 A, and 0.5 rev/s defaults. It cannot run motion unless
`--hardware-estop-ready` is supplied. Closed-loop entry now uses correlated,
terminally acknowledged product `SET_OPERATION` and `ARM` commands instead of
the deprecated direct Axis-state write.

## USB Test Session (protocol 4.5/schema 4)

HELLO continues to establish a read-only binary diagnostics connection. Scope,
fault, crash and log inspection therefore do not acquire motor-control ownership.
State-changing USB `COMMAND` frames now require a separate Test Session:

1. BEGIN validates the independent-estop declaration, lease and hard ceilings.
2. Firmware applies the ceilings to controller velocity, trajectory velocity,
   motor current and motor torque before returning ACTIVE.
3. The backend renews the lease with KEEPALIVE while the owner is alive.
4. STOP submits DISARM and must receive a terminal COMPLETED result.
5. END releases ownership and restores the previous runtime limits.

Lease expiry and physical USB disconnect submit DISARM inside firmware. They do
not restore the previous, potentially higher limits; conservative limits remain
until a clean session or explicit later configuration/reset. ARM is rejected for
read-only sessions or sessions that did not confirm the independent estop.

The backend exposes `/api/usb/session/begin`, `/command`, `/keepalive`, `/stop`
and `/end`, with equivalent WebSocket messages. Test-session state is included
in the regular USB status response.

## Versioned capture replay

`/api/usb/export/replay` exports a deterministic `odrive-usb-replay` schema 1
bundle containing the exact bounded wire capture, decoded events, firmware build
and manifest identity, and protocol identity. The canonical Python replay module
is shared with `Firmware/Tests/odrive_hil` rather than copied into test scripts.
It provides deterministic mutations for missing control sequences, corrupt frame
CRCs, and stale feedback across a wrapping 32-bit firmware clock. Opaque byte
payloads are retained as tagged base64, so any decoder output remains valid JSON.

## Layer inventory

`Firmware/Tests/test_layers.json` is the machine-readable L0-L4 inventory.
Existing host-independent C++ sources already cover much of L0 and the
SafetySupervisor event scenarios in L1. USB parser/ring tests map to L2, guarded
motor tests map to L3, and the generated product-CAN contract/load gate maps to
L4.

`Firmware/Tests/pytest.ini` excludes the energized `hex_4342_mt6826s` programs
from ordinary pytest discovery. Those files are executable HIL tools, not unit
test modules: several intentionally check optional bench dependencies and exit
at import time. Offline CI can now run `pytest Firmware/Tests` without importing
or accidentally starting hardware programs.

## Native scenario ABI (first slice)

`native_scenario_c_api.h` defines version 1 of a fixed-width C ABI around the
hardware-independent FaultManager, RealtimeEventRing, and SafetySupervisor.
An opaque scenario owns all state. Callers can submit validated commands,
inject readiness/arm/disarm/fault events, explicitly advance the wrapping
control sequence, and read correlated results, realtime requests, state, and
the first fault. Compile-time structure-size assertions prevent silent ctypes
layout drift, and invalid enum/reserved fields fail at the ABI boundary.

The `odrive_scenario` shared-library target and
`odrive_hil.native_scenario.NativeScenario` ctypes client are wired into the
host test architecture. The Python replay iterator preserves sequence gaps and
stale-feedback annotations without synthesizing missing samples. A MinGW host
build now executes 74 C++ scenarios (about 1.43 million assertions), and CTest
also loads the generated DLL through the standard-library Python smoke test.
The full firmware Python suite exercises the same ctypes lifecycle and timeout
path. The selected Windows MinGW distribution has no ASan/UBSan runtime, so
sanitizer coverage remains an explicit CI/toolchain gap rather than a claimed
result.

Linux CI closes that gap with Clang ASan/UBSan, CTest, the ctypes smoke test,
and the offline Python suite. CTest passes the shared-library path explicitly,
so execution does not depend on a developer-specific build directory.

## Injectable platform ports

`platform_ports.hpp` defines allocation-free Clock, SensorSource, PowerStage,
NVM, and TraceSink function tables. Controller, encoder, and motor fault
timestamps no longer read DWT directly. Encoder sampling can be supplied by a
deterministic dual-sensor source; Motor PWM writes and forced disarm can be
supplied by a fake power stage. Unbound and failing ports fail closed, and
native tests cover pair evidence, PWM/disarm failure, and unbound ports.

NVM already has a narrow C transaction boundary in `stm32_nvm.h`, and Axis
trace dispatch already has an injected sink, so no duplicate wrapper was added
around those existing seams.

## Qualification

`tools/run_architecture_qualification.py` runs the static gate, Python suite,
native build/CTest, and release cross-build. With `--hardware-readonly` it also
checks firmware identity, parser counters, read-only Test Session ownership,
and fail-closed ARM rejection. It never requests calibration, an operation, a
motion-capable lease, or a motor setpoint.

## Deliberately deferred algorithm tools

Calibration/estimator experiments still use the isolated legacy CAN 0x1E
endpoint because their replacement belongs to the planned algorithm redesign.
They no longer define a second protocol implementation or participate in the
product control boundary. Adding a generic USB memory/config RPC merely to keep
those experiments unchanged would recreate the coupling this refactor removed,
so they remain quarantined until their algorithms are replaced. This is outside
Phase 8 and is not a prerequisite for the production state/CAN/USB architecture.
