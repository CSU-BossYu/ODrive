# Phase 4 evidence: USB Debug/Test binary protocol

Date: 2026-08-01
Workspace: `F:\project_source_code\ODrive`

## 1. Scope completed

- Added `Firmware/USB/usb_debug_protocol.hpp/.cpp` as a HAL-independent,
  fixed-memory codec and CDC byte-stream parser.
- Added `Firmware/USB/usb_debug_transport.hpp/.cpp` with bounded RX bytes,
  typed Command delivery, request tracking, independent TX priority rings,
  CommandResult correlation, parser statistics, and task-context TX staging.
- Added explicit `HELLO/CAPABILITIES`, `COMMAND/COMMAND_RESULT`,
  `FAULT_EVENT`, `STATE_EVENT`, `LOG_RECORD`, `SCOPE_RECORD`, `STATS`, and
  `CALIBRATION_DATA` message types. `SCOPE_RECORD` is a transport record only;
  no Phase 5 UI or capture configuration was added.
- Migrated the existing calibration stream away from the stdout queue. CDC is
  binary-only from configuration onward: the raw stdout-frame compatibility
  API is removed and the legacy `@log` producer is not started.
- CDC RX only copies bounded bytes and wakes the USB task. The parser,
  validation, request-window checks, and Command delivery run in task context.
  The Command sink is the existing Axis/SafetySupervisor boundary; USB does
  not write Axis, Motor, Controller, configuration, PWM, or SafetyState
  fields.
- Axis now exposes task-context result/trace sinks. Phase 3's
  `TraceConsumer` feeds the USB transport only after consuming the independent
  Critical/State/Log/Scope rings in task context.
- Added generated manifest identity. The final manifests contain the existing
  CAN protocol version, USB protocol version `4.1`, USB protocol schema hash,
  fault schema hash, dirty-worktree hash, compiler identity, and build ID.
- No Phase 5 waveform UI, Phase 6 Crash Recorder, or Phase 7 CAN deletion was
  implemented. No CAN debug item was added and existing CAN compatibility was
  retained.

## 2. Wire format

All fields are explicitly encoded little-endian; no C++ struct is sent over
CDC. The fixed header is 40 bytes, followed by a payload of at most 512 bytes
and a 4-byte CRC-32/IEEE over `header || payload`.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | magic `0x4F44` |
| 2 | 1 | protocol version `4` |
| 3 | 1 | schema version `1` |
| 4 | 1 | message type |
| 5 | 1 | flags |
| 6 | 4 | TX sequence |
| 10 | 4 | request ID |
| 14 | 2 | payload length |
| 16 | 8 | build ID bytes |
| 24 | 16 | manifest identity prefix |
| 40 | N | payload, `N <= 512` |
| 40 + N | 4 | CRC-32/IEEE |

The authoritative schema is
`docs/usb_debug_protocol_schema.json`. Command results use the existing
`ACCEPTED`, `REJECTED`, `COMPLETED`, and `FAILED` values. Duplicate and expired
request IDs receive explicit rejected results with protocol reasons.

## 3. Boundary and backpressure behavior

```text
CDC RX callback
  -> bounded RX byte ring
  -> USB task parser/resynchronizer/CRC
  -> bounded typed Command delivery
  -> CommandService/SafetySupervisor
  -> RealtimeRequest
  -> Axis task execution
  -> confirmation
  -> CommandResult
  -> bounded TX priority rings
  -> USB task frame submission
  -> CDC TX callback
```

TX priority is `Critical > CommandResult/State > Calibration > Scope > Log`.
Critical TX storage is independent from every lower-priority queue. USB
disconnect, TX busy, host not reading, RX garbage, parser errors, and queue
overflow only drop or count transport data; they do not block the control loop,
change SafetyState, or affect CAN. The binary session is established by
`HELLO`; no stdout text is ever multiplexed onto the CDC byte stream.

Each binary protocol frame is retained by the USB task and submitted to the
CDC driver in chunks no larger than `USB_TX_DATA_SIZE` (64 bytes). A frame is
removed from the priority transport only after every chunk has completed.
USB packet boundaries therefore have no protocol meaning and frames larger
than one endpoint packet cannot wedge the active TX slot.

`HELLO` is a strict migration gate: unsolicited Trace, Calibration, and
CommandResult records are neither queued nor transmitted before the binary
session is active. Command results retain `CommandSource`; only USB-originated
results are correlated onto the USB request/response stream.

The CDC callback performs only bounded byte enqueue and event notification.
It does not call the protocol codec, calculate CRC, format strings, allocate,
wait, or access control state. The USB task owns codec, command routing, and
CDC TX submission. No formal `volatile bool` handshake was added.

## 4. Verification

| Check | Command | Result |
|---|---|---|
| ARM Release | `cmake --build --preset firmware-release -j4` | PASS; `text=158732 data=1416 bss=168272` |
| ARM Debug | `cmake --build --preset firmware-debug -j4` | PASS; `text=172488 data=1416 bss=168272` |
| Phase 2 gate | `python tools/check_phase2_static_gate.py` | PASS |
| Phase 3 gate | `python tools/check_phase3_static_gate.py` | PASS |
| Phase 4 gate | `python tools/check_phase4_static_gate.py` | PASS |
| Legacy error gate | `python tools/check_legacy_error_writes.py` | PASS; 5 audited projections |
| Python regression | `python -m unittest discover -s Firmware/Tests -p "test_*.py" -q` | PASS; 47 tests |
| New C++ test source | `arm-none-eabi-g++ -std=c++17 -Wall -Wextra -Wpedantic ... -c Tests/test_usb_debug_protocol.cpp` | PASS; compile-only |
| Diff check | `git diff --check` | PASS apart from existing LF/CRLF notices |

The new C++ tests cover the golden vector, single-byte and arbitrary
fragmentation, sticky frames, noise resynchronization, CRC recovery, malformed
length, unknown version/type, deterministic bounded fuzz input, request
duplicate/expiry, command delivery, disconnect behavior, TX busy retention,
and independent RX/Critical-TX overflow counters. The Python test decodes the
same golden vector and applies the same fixed-seed fragmentation and fuzz
limits. The transport test also reconstructs a 100-byte Scope frame from
64-byte-or-smaller TX chunks, verifies HELLO gating, and verifies that a CAN
result cannot satisfy a USB request.

## 5. Native CTest status

Native configuration was attempted with `cmake --preset host-debug`, but the
known host toolchain remains unavailable:

```text
E:/Qt5.14.2/Tools/mingw730_32/bin/g++.exe
is not a full path to an existing compiler tool
```

`ctest --preset host-debug` consequently reports `No tests were found!!!`.
No Native CTest, ASan, or UBSan pass is claimed. The new C++ test source was
only checked with the ARM compiler; that is not a native test result.

## 6. Final manifest identity

| Configuration | USB protocol | Protocol schema SHA-256 | Manifest |
|---|---|---|---|
| firmware-release | `4.1` | `49101865dc39ece6f3e77835d0e1ae10b6ba2caa48e7c44ff6e8687108667255` | `Firmware/build/firmware-release/build_manifest.json` |
| firmware-debug | `4.1` | `49101865dc39ece6f3e77835d0e1ae10b6ba2caa48e7c44ff6e8687108667255` | `Firmware/build/firmware-debug/build_manifest.json` |

The `build_id` and dirty-worktree identity are intentionally read from the
manifest files rather than copied into this evidence document, so the
manifest remains a truthful identity of the final dirty workspace.

Manifest files:

- `Firmware/build/firmware-release/build_manifest.json`
- `Firmware/build/firmware-debug/build_manifest.json`

## 7. Evidence still open

- No physical USB device, continuous USB plug/unplug run, host-not-reading
  stress run, USB packet capture, or target loopback was available.
- No hardware evidence is provided for USB timing, CDC re-arm behavior,
  control-loop stability, state 8, PWM safety, CAN timing, or motor motion.
- `RealtimeSnapshot.isr_cycles` remains an observation field; no hardware
  worst-case cycle budget has been established.
- Native CTest remains blocked until a working host MinGW C++ compiler is
  supplied.

Existing uncommitted user changes and
`Firmware/Tests/hex_4342_mt6826s/dual_encoder_motion.csv` were preserved. No
reset, checkout, deletion, or overwrite was performed. Work stops here for
independent Phase 4 review.

## 8. Superseding closure note (2026-08-02)

This evidence predates the Phase 5/6 protocol additions. The authoritative
current contract is now protocol release `4.4`, schema `3`; the frame envelope
is unchanged. A release-4.3 firmware was physically flashed and the backend
completed the binary HELLO/CAPABILITIES handshake over COM4 with exact build
and manifest identity matching. Read-only status and Scope diagnostic actions
also completed without issuing a motor command.

The hardware stream run exposed a host parser defect under high-rate sticky
frames: `StreamDecoder` bounded a large serial read before decoding it. That
discarded valid frame prefixes and produced false noise/resynchronization
counts. The decoder now parses all complete frames before bounding only the
undecoded residue, with a regression test covering 64 complete frames in one
read. This fix and protocol 4.4 still require one final flashed-build hardware
verification; the earlier “no physical device” statement applies only to the
original 2026-08-01 evidence run.
