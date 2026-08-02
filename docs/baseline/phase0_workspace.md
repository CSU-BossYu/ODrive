# Phase 0 baseline: workspace and constraints

Recorded before Phase 0 changes on 2026-08-01.

The workspace was intentionally not clean. The following user changes were
present and were not reset, overwritten, deleted, or absorbed:

- `Firmware/MotorControl/axis.cpp`
- `Firmware/MotorControl/axis.hpp`
- `Firmware/MotorControl/encoder.cpp`
- `Firmware/MotorControl/encoder.hpp`
- `Firmware/MotorControl/main.cpp`
- `Firmware/MotorControl/motor.cpp`
- `Firmware/Tests/hex_4342_mt6826s/run_dual_encoder_motion_quality.py`
- `Firmware/Tests/test_dual_encoder_motion_quality.py`
- `Firmware/Tests/test_lz5710_encoder_chain.py`
- `Firmware/Tests/hex_4342_mt6826s/dual_encoder_motion.csv`

The experiment changes include `closed_loop_phase_feedback_ready_` and
`closed_loop_controller_ready_`. They are treated as unverified temporary
handshake logic, not as a stable design. The CSV remains untouched.

Commands required by the plan were run before changes:

```text
git status --short
git diff --stat
git diff -- Firmware/MotorControl Firmware/communication Firmware/Tests foc_ui
```

The current branch was `codex/lz5710n-clean` at revision
`91a9d60e test(lz5710): add motion encoder quality diagnostic`.

The baseline firmware build directory existed as `Firmware/build`, but it was
not treated as reproducible evidence because it was a pre-existing build
directory and the current C++ tests were not registered with CTest.
