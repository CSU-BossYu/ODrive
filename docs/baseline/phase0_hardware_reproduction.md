# Phase 0 hardware reproduction record

Status: **not executed in this workspace**.

No device, CAN adapter, power-supply record, wiring record, or raw state-8
failure log was available to the agent. Therefore the hardware portion of the
Phase 0 acceptance gate is explicitly open and is not marked as passed.

The required raw capture, when hardware is available, must include:

1. firmware build ID from the matching manifest;
2. CAN request for axis state 8;
3. heartbeat frames before, during, and after the request;
4. motor, encoder, controller, and axis error reads;
5. power, wiring, encoder, and safety-stop conditions; and
6. the final safe-stop result.

The existing `Firmware/Tests/hex_4342_mt6826s/baseline_8khz.csv` is a runtime
telemetry sample, not a state-8 failure log, and is retained as-is.
