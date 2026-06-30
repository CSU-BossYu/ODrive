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

4. Run velocity closed-loop at 5 turns/s for 10 s with 3 A current limit:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\run_mt6826s_velocity_loop.py --bitrate 1000000 --velocity 5 --duration 10 --current-limit 3.0 --clear-at-end
   ```

5. Return to IDLE and confirm all errors are zero:

   ```powershell
   python Firmware\Tests\hex_4342_mt6826s\read_mt6826s_pair.py --bitrate 1000000 --period 1 --samples 1 --show-status
   ```

Position mode is not part of this gate until the vernier false-wrap issue is
closed.
