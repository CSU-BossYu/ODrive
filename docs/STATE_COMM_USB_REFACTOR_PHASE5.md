# State/Communication USB Refactor — Phase 5 evidence

Date: 2026-08-01
Workspace: `F:\project_source_code\ODrive`
Scope: waveform capture, USB diagnostics transport, and host UI only. Phase 6
Crash Recorder work was not started.

## Implementation summary

Phase 5 adds a generated Scope channel contract and a fixed-memory capture
engine. The USB task validates `SCOPE_CONFIG`, publishes an immutable
`CapturePlan` through the existing fixed triple-buffer boundary, and services
`SCOPE_ARM`, `SCOPE_STOP`, `SCOPE_STATUS`, and bounded `SCOPE_DATA_BATCH`
messages. The control ISR only receives a typed sample context, reads the
generated channel whitelist, stores fixed-size samples, and raises atomic
manual/fault/state/threshold trigger flags. It does not parse USB/CAN data,
allocate memory, format strings, or call blocking/RTOS APIs.

The capture ring is 192 samples (`64 pre + 128 post`). `ScopeSample` is 44
bytes, so the sample storage is 8,448 bytes; commit markers add 768 bytes.
The final firmware links with `bss=177,800` bytes in both configurations. The
capture engine has no heap allocation and its stream is placed behind the
existing Scope-priority USB queue, separate from CAN.

Supported capture behavior:

- continuous and triggered capture;
- decimation and per-channel maximum sample-rate validation;
- pre-trigger/post-trigger windows;
- manual, fault, state, and threshold-crossing triggers;
- raw fixed-point/f32 wire values with firmware sequence/timestamp fields;
- explicit sequence-gap, dropped-sample, completion, and timestamp-wrap
  accounting;
- bounded binary capture and CSV export.

The protocol delta and compatibility policy are recorded in
[`usb_debug_protocol_schema_diff.md`](usb_debug_protocol_schema_diff.md).

## Scope schema and generated outputs

Source schema: `docs/scope_channel_schema.json` (15 channels, IDs 1–15).
Generated outputs:

- `Firmware/MotorControl/scope_channels_generated.hpp`
- `foc_ui/backend/odrive_usb/scope_channels_generated.py`
- `foc_ui/frontend/src/scope_channels_generated.ts`

The protocol schema SHA-256 used by the final firmware manifest is:

`bd41fa1a96d0a61b14dbb899bb2ed7fe065b4c2a36d84611680670510c464efc`

The Scope channel schema SHA-256 used by the final firmware manifest is:

`d138c8131fd03edc15993bf8235371b1a06ced60264931487271132e37830cce`

The build manifest also records the generated build ID and dirty-worktree
identity. They are intentionally not duplicated here because the manifest
hash includes the dirty workspace, including this evidence file; use the
`build_id` field in the linked final manifests as the authoritative value:

- [`firmware-release/build_manifest.json`](../Firmware/build/firmware-release/build_manifest.json)
- [`firmware-debug/build_manifest.json`](../Firmware/build/firmware-debug/build_manifest.json)

## Host backend and UI

`foc_ui/backend/odrive_usb/` is an independent package. It does not import
`odrive_can`; CAN routes, `/docs`, and the SPA fallback remain available.
The backend provides strict HELLO/CAPABILITIES identity checks, incremental
framing, fake serial injection, bounded event/raw buffers, sequence-gap and
timestamp-wrap detection, USB status/scope routes, `/ws/usb`, and binary/CSV
export paths. Disconnect only closes the USB transport; it does not send
DISARM, CLEAR_FAULTS, or another motor-state command.

The UI adds a USB status/handshake indicator and a diagnostics panel with:

- channel selection and generated units/ranges;
- continuous/triggered configuration, decimation and pre/post controls;
- arm/manual-trigger/stop controls using the diagnostic Scope actions;
- firmware-timestamp-aligned waveform preview;
- State Timeline, Fault Inspector, bounded Test Session history;
- CSV and raw binary export.

Waveform preview data is bounded to 2,000 samples. USB WebSocket queues are
bounded to 64 events per browser; when a client is slow, old waveform events
are dropped before status/fault/command events, so the serial reader is never
backpressured by the browser.

## Verification evidence

Passed:

- `cmake --build --preset firmware-release -j4`
  - `text=164,656`, `data=1,416`, `bss=177,800`;
- `cmake --build --preset firmware-debug -j4`
  - `text=178,612`, `data=1,416`, `bss=177,800`;
- Phase 2, Phase 3, Phase 4, Phase 5 static gates and the legacy error-write
  gate;
- ARM compile-only checks for `test_scope_capture.cpp` and
  `test_usb_scope_protocol.cpp` with `arm-none-eabi-g++`;
- `python -m pytest -q` from `foc_ui/backend`: **162 passed**;
- `npm ci`: completed; `npm run build`: TypeScript check and Vite production
  build completed;
- `git diff --check`: no whitespace errors (Git emitted only existing
  LF/CRLF conversion warnings).

Native CTest is not claimed. `cmake --preset host-debug` remains blocked by
the unavailable configured compiler:

`E:/Qt5.14.2/Tools/mingw730_32/bin/g++.exe is not a full path to an existing compiler tool`

Therefore no native CTest, ASan, or UBSan pass is reported. The added C++ test
sources were compile-checked with the ARM compiler, not executed natively.

## Hardware evidence and open risks

No physical ODrive, USB packet capture, target loopback, motor, power stage, or
encoder replay was available in this workspace. Consequently there is no
hardware evidence for CDC re-arm, plug/unplug behavior, control-loop stability,
10 kHz worst-case ISR timing, PWM safety, motor motion, or real dropped-frame
behavior. `RealtimeSnapshot.isr_cycles` remains an observation field; a target
worst-case cycle budget still needs a hardware run.

The Phase 5 implementation is ready for independent review, but the workflow
stops here before Phase 6 as requested.

## Changed files

Primary new files:

- `docs/scope_channel_schema.json`
- `docs/usb_debug_protocol_schema_diff.md`
- `Firmware/MotorControl/scope_capture.hpp/.cpp`
- `foc_ui/backend/odrive_usb/{protocol,transport,app}.py`
- `foc_ui/frontend/src/composables/useUsbDiagnostics.ts`
- `foc_ui/frontend/src/components/UsbDiagnosticsPanel.vue`

Generated source, protocol/transport integration, Axis ISR/task hooks, CMake
targets, Python tests, frontend wiring, and build-manifest inputs are also
updated. Existing user changes and
`Firmware/Tests/hex_4342_mt6826s/dual_encoder_motion.csv` were preserved; no
reset, checkout, deletion, commit, or push was performed.

## Superseding closure note (2026-08-02)

Phase 5 was exercised on COM4 with a flashed release-4.3 build using a
diagnostic-only continuous Scope capture (`1 kHz`, decimation `10`, five
channels). No motor arm/control command was sent. Configuration, ARM, status,
STOP, and stable stopped-state observation succeeded. The run also found that
the Python stream decoder truncated a 4 KiB serial read to its approximately
2.2 KiB residue bound *before* parsing; this caused false dropped/noise counts
under waveform load. It was fixed by parsing complete frames first and
bounding only undecoded residue.

Coverage now also asserts that one oversized sticky read containing 64 valid
frames loses none, and that 100 control-loop calls at decimation 10 yield
exactly 10 samples which are drained once without batch replay. Phase 5 is
software-complete, but the parser fix and current protocol 4.4 firmware need a
single final hardware verification after the final image is flashed. A
10-minute unplug/host-not-reading soak and measured ISR worst-case budget
remain system qualification work rather than implementation blockers.
