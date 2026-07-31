# LZ5710 dual-encoder position chain

## Confirmed hardware identity

The production configuration assigns `abs_spi_cs_gpio_pin = 2` and
`abs_spi_aux_cs_gpio_pin = 4`. `Encoder::setup()` constructs the main
`Mt6826sSpi` from the first pin and the auxiliary instance from the second.
The board definitions map IO2 to PA1 and IO4 to PA3.

The physical identity cannot be inferred from those signal names alone. The
LZ5710 wiring confirmation is:

- IO2: main MT6826S on the 22-tooth center gear;
- IO4: auxiliary MT6826S on the 21-tooth Vernier gear;
- both sensors are on the rotor-side gear train, not on the output shaft;
- the gears mesh externally, so their raw count directions are opposite.

The electrical calibration determines the motor-to-encoder direction. The
Vernier sensor offsets and configured output orientation determine which
physical pose is `output_position_rad = 0` and which output direction is
positive. These values must be valid in NVM before `encoder.is_ready` can
become true.

## Mechanical model and the 21 branches

Let `q` be output position in radians. After each sensor's raw direction and
zero offset are applied, the ideal observations are

```text
main_phase_rad = wrap_2pi(10 q)
aux_phase_rad  = wrap_2pi((10 * 22/21) q)
               = wrap_2pi((220/21) q)
```

The physical tooth ratio is **22:21**. The number **10/21** is not a tooth
ratio: it is the difference in corrected sensor revolutions per output
revolution:

```text
220/21 - 10 = 10/21
```

If `u_m = main_phase_rad / (2 pi)`, the main observation alone permits

```text
q_k = 2 pi (k + u_m) / 10
```

Within one pair repeat interval, `k` takes every value from 0 through 20.
Adjacent candidates predict auxiliary phases separated by

```text
(220/21) * (2 pi / 10) = 2 pi * 22/21
                               = 2 pi/21 modulo 2 pi.
```

The auxiliary residual therefore selects one of the 21 candidates. The pair
repeats when `(10/21) * output_turns` is an integer, so its smallest positive
repeat is `21/10 = 2.1` output revolutions:

```text
unique_range_rad = 2 pi * 21/10
main cycles in range = 10 * 21/10 = 21
main counts in range = 21 * 32768 = 688128
```

Thus `21 * 2^15` describes the main-encoder count span of the unique absolute
range. It does not mean 21 output revolutions. At power-up the pair recovers
absolute position modulo 2.1 output revolutions. Travel beyond that range is
not power-cycle absolute; it is maintained by the runtime integer main-count
accumulator.

## Runtime chain

```text
IO2/IO4 SPI burst
  -> CRC, fixed-bit, status and DMA validity
  -> count to corrected single-turn sensor phases [0, 2 pi)
  -> 21-candidate Vernier solve and residual/margin checks
  -> unique absolute position [0, 2 pi * 21/10)
  -> signed minimum-count-delta unwrap with int64 accumulator
  -> continuous output_position_rad
  -> optional periodic output-geometry LUT
  -> continuous-position PLL
  -> output_position_rad, wrapped_output_phase_rad, velocity_estimate_rpm
```

The existing 64-bin LUT is an **output geometry correction**, not a magnetic
linearity LUT for either MT6826S. Its common and direction-dependent tables
are indexed by wrapped raw output position, interpolate periodically between
bin 63 and bin 0, and are therefore applied after the Vernier solve. Neither
the clean baseline nor the reference branch contains separately generated
main/aux sensor-linearity LUTs, so the firmware does not pretend that this one
table belongs to both sensors. The table, enable flag, and effective scale are
part of the versioned encoder configuration and are loaded from NVM.

## Continuity, aliasing, and PLL

Main count deltas are normalized to the signed half-CPR interval before they
are accumulated. A first valid pair only initializes absolute and PLL state.
Invalid frames do not update the tracker or PLL. A sample interval over 2 ms,
a displacement above the configured 300 rpm output limit, a Vernier
consistency failure, or a PLL observation step over 0.25 rad invalidates the
chain rather than clamping position.

The normal main sample period is 100 us; the auxiliary branch check is
refreshed every 25 samples (2.5 ms). The auxiliary-rate main-phase unwrap is
unambiguous below 1200 rpm output speed:

```text
abs(10 * omega_output * 0.0025) < pi
```

The explicit 300 rpm tracker limit is four times lower. It also exceeds the
existing controller default limit of 120 rpm, leaving a verifiable margin.

The output PLL uses `bandwidth = 120 rad/s`, damping `sqrt(1/2)`, and the
actual elapsed observation interval. Its detector error is the linear
difference between two continuous positions and is never wrapped. Velocity is
kept at the external boundary in rpm:

```text
rpm = rad_per_second * 60 / (2 pi)
```

At the accepted maximum `dt = 2 ms`, `bandwidth * dt = 0.24`.

## Readiness and diagnostics

Readiness requires three consecutive valid main observations, three accepted
auxiliary observations, a locked non-ambiguous branch, residual within limit,
valid continuous tracking, an initialized PLL, valid direction, and a valid
LUT when enabled. The 400 Hz auxiliary stream also has an explicit 10 ms age
timeout, so a previously locked but no longer updating auxiliary sensor cannot
leave the encoder ready.

CAN extended subcommand `0x0A` exposes:

| Item | Value |
| ---: | --- |
| `0x80` | PLL linear error, rad |
| `0x81` | active LUT correction, rad |
| `0x82` | readiness bits: main, aux, resolver, tracker, PLL, direction, ready |
| `0x74` | independent chain fault bitmask |
| `0x75` | Vernier branch index, 0..20 |
| `0x76` | runtime 2.1-turn range index |
| `0x77`, `0x78` | corrected main/aux single-turn phases, rad |
| `0x79` | unique-range absolute position, rad |
| `0x7A` | wrapped output phase, rad |
| `0x7B` | continuous output position, rad |
| `0x7C` | output velocity, rpm |
| `0x7D`, `0x7E` | residual and second-best margin, rad |
| `0x7F` | LUT enabled/valid bits |

Chain-fault bits independently report main communication, auxiliary
communication, frame check, LUT validity, no solution, ambiguity, excessive
residual, position/branch jump, PLL timeout, and invalid direction.
