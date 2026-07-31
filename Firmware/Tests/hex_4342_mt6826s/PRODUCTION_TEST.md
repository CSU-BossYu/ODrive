# Single-axis production gate

This gate is for the trimmed v3.6 STM32F405 single-axis firmware profile.
The firmware is M0-only, uses CAN simple at 1 Mbps, and keeps the generic
single-SPI absolute and dual-SPI vernier encoder boundaries with the MT6826S
driver as the current reference implementation.

## Build artifacts

After each accepted build, archive:

- `Firmware/build/ODriveFirmware.hex`
- `Firmware/build/ODriveFirmware.elf`
- `Firmware/build/ODriveFirmware.map`
- `arm-none-eabi-size Firmware/build/ODriveFirmware.elf`
- `git rev-parse HEAD`

## Configuration recovery

This production profile intentionally bumps the NVM config version when default
safety values change. If the board boots with default/erased config, write the
known MT6826S vernier configuration, save, and reset before running the gate:

```powershell
python Firmware\Tests\hex_4342_mt6826s\configure_mt6826s_vernier.py --bitrate 1000000 --save
```

Then reset the board, for example with STM32CubeProgrammer:

```powershell
& 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' -c port=SWD -rst
```

Expected M0 defaults for this profile:

- CAN bitrate: 1 Mbps
- CAN node ID: 0
- motor calibration current: 2 A
- motor current limit: 3 A

## Hardware gate

Run with a current-limited 24 V bus supply.

1. Idle for at least 60 s and confirm:
   `ADCpre=0 ADCpost=0 DLmiss=0 FOC_BT=0`.
2. Run two motor calibrations at 2 A and confirm R/L differ by less than 5%:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\diagnose_motor_phases.py --bitrate 1000000 --test-current 2 --run-timeout 30
   ```

3. Run encoder offset calibration at 2 A calibration current and 3 A current limit:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\observe_mt6826s_offset_calibration.py --bitrate 1000000 --calibration-current 2 --current-lim 3 --clear-at-end
   ```

   Pass criterion: scan response error < 2%, bus remains about 24 V.

4. Check calibration values before any closed-loop gates:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\run_calibration_persistence_check.py --bitrate 1000000 --clear-at-end
   ```

   Pass criterion: phase resistance/inductance are positive and encoder
   direction is nonzero. If you intend to keep the calibration across reset,
   explicitly mark and save it:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\run_calibration_persistence_check.py --bitrate 1000000 --mark-precalibrated --save --clear-at-end
   ```

   Saving configuration triggers the firmware reset-after-ACK path.

5. Calibrate the dual-MT6826S vernier geometry offsets from several static
   output positions. This is separate from encoder electrical offset
   calibration and does not require a full output-shaft revolution:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\calibrate_mt6826s_vernier_offsets.py --bitrate 1000000 --points 5 --firmware-fit --apply
   ```

   The default fit keeps `vernier_main_offset` unchanged and updates
   `vernier_aux_offset`, preserving the main encoder as the output coordinate
   reference. With `--firmware-fit`, point capture and fitting run on the
   device through extended subcommand `0x0D`; the script only prompts for
   each static pose. It searches near the current offsets by default; use
   `--search-radius` only if the existing single-point value is known to be
   far away. Review the fit score before saving. If the fitted residuals are
   acceptable, repeat with `--save` to persist them.

6. Run velocity closed-loop at 5 turns/s for 10 s with 3 A current limit:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\run_mt6826s_velocity_loop.py --bitrate 1000000 --velocity 5 --duration 10 --current-limit 3.0 --clear-at-end
   ```

7. Return to IDLE and confirm all errors are zero:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\read_mt6826s_pair.py --bitrate 1000000 --period 1 --samples 1 --show-status
   ```

### Dual-encoder motion quality

With the `foc_ui` backend running and the shaft unloaded, run a conservative
forward/stop/reverse test at 3 rpm:

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_dual_encoder_motion_quality.py --rpm 3 --clear-at-start --csv Firmware\Tests\hex_4342_mt6826s\dual_encoder_motion.csv
```

The script uses the backend WebSocket, so the UI may remain open and must keep
exclusive ownership of the PCAN adapter. It does not alter gains, limits,
calibration values, or NVM. It always requests 0 rpm and IDLE on normal exit,
fault, or Ctrl+C.

Review the final main/aux CRC, fixed-bit, and DMA rates; Vernier residual and
margin; readiness loss; chain-fault mask; and maximum main-sample age. At the
10 kHz control rate, 20 sample-age cycles equal 2 ms and are the boundary at
which sustained main-encoder loss can invalidate motor electrical feedback.

Position mode is not part of this gate until the vernier false-wrap issue is
closed.

## Algorithm link gates

These gates are intentionally narrower than the production hardware gate above.
They prove that the retained control algorithms and CAN command surfaces are
still reachable after trimming. Keep the limits conservative and return to
IDLE after each test.

### Stage A: torque mode

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_torque_mode_test.py --bitrate 1000000 --torque 0.005 --duration 1.0 --current-limit 3.0 --clear-at-end
```

### Stage B: MIT packed-control mode

This gate streams conservative neutral `0x01F Set_MIT_Control` frames in
`TORQUE_CONTROL + INPUT_MODE_MIT`, then returns to IDLE.
It requires valid runtime motor calibration and encoder direction. If the board
was reset and calibration values read back as zero, rerun the low-current motor
and encoder offset calibration gates first.

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_mit_mode_link_test.py --bitrate 1000000 --set-precalibrated-if-needed --clear-at-start --clear-at-end
```

Optional tiny torque-feedforward link test:

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_mit_mode_link_test.py --bitrate 1000000 --set-precalibrated-if-needed --torque-ff 0.005 --clear-at-start --clear-at-end
```

### Stage C: position passthrough

Position mode is currently a link/entry test only. Do not use it as an
anticogging or production-position gate until the vernier position-following
issue is closed.

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_position_passthrough_test.py --bitrate 1000000 --step 0 --pos-gain 0 --duration 0.5 --settle-duration 0.5 --current-limit 3.0 --clear-at-end
```

### Stage D: anticogging CAN link

This gate checks the anticogging CAN status/configuration surface without
starting a full cogging-map calibration sweep. Full calibration depends on
qualified position following and should remain a separate hardware test.

Read-only link test:

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_anticogging_link_test.py --bitrate 1000000 --clear-at-end
```

Optional setter echo test while IDLE. This writes the current anticogging enable
and threshold values back unchanged, then verifies that they did not change:

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_anticogging_link_test.py --bitrate 1000000 --write-same-config --clear-at-end
```

Experimental unloaded full calibration, after flashing the vernier-aware
anticogging firmware:

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_mt6826s_anticogging_calibration.py --clear-at-end
```

The preflight must report a 41:1 sweep of approximately 8.780 degrees. Run
without `--save` for the first hardware validation. After reviewing the full
3600-point run and testing the resulting compensation, repeat with `--save`
to persist the map.

### Stage E: input modes and trajectory CAN link

This gate checks the retained input-mode and trajectory CAN command surfaces
while the axis remains IDLE. It does not qualify dynamic position following,
trajectory tracking, or circular setpoint behavior.

```powershell
python Firmware\Tests\hex_4342_mt6826s\run_input_modes_link_test.py --bitrate 1000000 --clear-at-end
```

Covered surfaces:

- `VELOCITY_CONTROL + VEL_RAMP`
- `TORQUE_CONTROL + TORQUE_RAMP`
- `POSITION_CONTROL + POS_FILTER`
- `POSITION_CONTROL + TRAP_TRAJ`
- `Set_Traj_Vel_Limit`, `Set_Traj_Accel_Limits`, `Set_Traj_Inertia`
- circular encoder diagnostic readout
