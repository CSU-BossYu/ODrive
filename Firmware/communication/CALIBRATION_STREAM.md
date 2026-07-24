# Calibration raw stream

Calibration is commanded through CAN subcommand `0x0F`; CAN does not carry the
high-rate samples. While a session captures data, the firmware drains its fixed
SPSC record queue to the stdout USB CDC endpoint as binary `ODCR` frames. Normal
text logs may appear between frames, so consumers must scan for magic and
validate both CRCs.

## ODCR frame schema 1

All integers and floats are little-endian. The packed 28-byte header contains:

| Field | Type | Meaning |
| --- | --- | --- |
| magic | 4 bytes | ASCII `ODCR` |
| schema | uint16 | frame schema, currently 1 |
| header_size | uint16 | 28 |
| axis | uint16 | source axis index |
| record_type | uint16 | 1 full, 2 electrical-fast |
| payload_size | uint16 | 112 or 64 bytes |
| flags | uint16 | transport flags, currently 0 |
| sequence | uint32 | must equal the sequence inside the payload |
| payload_crc32 | uint32 | IEEE CRC-32 of payload |
| header_crc32 | uint32 | IEEE CRC-32 of the preceding 24 header bytes |

The electrical-fast record is emitted at PWM rate during R/L excitation. The
full record is emitted at 1 kHz during geometry motion and adds paired main/aux
sample sequences plus their completion-time skew in CPU cycles. Invalid fields
are zero and must be gated by the sample validity flags.

Queue overflow never overwrites unread data. It increments the session dropped
counter, creates a sample-sequence gap, and marks the next accepted record with
`CAL_SAMPLE_DROPPED_BEFORE`. Such a gap invalidates the host-side raw artifact,
but does not invalidate synchronous on-device identification because the fitter
consumes the sample before the optional transport copy is queued.

USB capture is optional. When stdout CDC is disconnected, the transport thread
drains and discards its record copies so CAN-only calibration cannot fail merely
because no stream consumer is attached. This does not affect the synchronous
on-device geometry fitter. When a host is connected, queue overflow remains
explicitly visible through the record flag and CAN dropped-record counter.

The host implementation is `CalibrationStreamDecoder` in
`foc_ui/backend/odrive_can/calibration_record.py`. It handles arbitrary CDC
fragmentation, text-log interleaving, corrupt-frame rejection, and resynchronizes
without unbounded buffering.

## Relative-angle model

The geometry fitter consumes complete forward and reverse scans. Pair sample
skew is corrected while fitting and remains acquisition metadata, not a
persistent calibration parameter. The observable runtime model contains:

- an effective output-ratio scale;
- a 64-bin circular common-error correction table;
- a 64-bin circular direction-dependent correction table.

Firmware linearly interpolates both tables in the uncorrected output-turn phase,
then divides the corrected observed position and velocity by the ratio scale.
The direction table keeps the last nonzero motion direction at standstill to
avoid a position discontinuity. These corrections affect controller output
coordinates only; they do not alter the Vernier absolute-turn branch resolver.

The firmware identifies this model without retaining the complete scan. It runs
one forward/reverse pass for centered ratio regression and a second
forward/reverse pass for binned residual accumulation. The full 64-bin tables
remain in the transactional candidate result; CAN result items `0x26` through
`0x2A` return the ratio scale and quality summary rather than transferring the
tables to the host.

## Flux linkage and torque constant

The same bidirectional motion also identifies permanent-magnet flux from the
measured dq voltage equation
`psi = (vq - R*iq) / electrical_velocity - L*id`. Samples below 20 electrical
rad/s, saturated PWM samples, and invalid current/voltage samples are excluded.
Forward and reverse means are averaged to reject approximately constant inverter
voltage bias. The candidate motor torque constant is derived, rather than fitted
independently, as `Kt = 1.5 * pole_pairs * psi`.
To prevent PWM and current-loop ripple from dominating the quality gate, only
steady-speed samples are admitted and flux is averaged in 32 ms blocks. The
reported dispersion is the standard deviation of block means, while the
reported used-sample count remains the number of admitted raw samples.

## Electrical phase delay

After geometry, flux, and mechanical candidates exist, firmware temporarily
applies them and performs a closed-loop, bidirectional three-speed scan. It
reconstructs the back-EMF vector from measured dq voltage/current, fits its
phase error versus electrical speed, and stores the slope as an electrical
delay in seconds. The committed value advances runtime encoder phase by
`electrical_velocity * electrical_phase_delay`; the fitted intercept and RMS
remain result-quality diagnostics rather than additional compensation terms.
Before phase extraction, firmware multiplies both reconstructed back-EMF axes
by the sign of electrical speed. This maps reverse rotation onto the same local
q-axis reference and prevents its physically reversed EMF vector from appearing
as a spurious pi-radian phase error.
Accepted delay samples are averaged in 32 ms blocks within each commanded speed
segment before linear regression. The physical quality limit remains 0.10 rad,
but it is applied to block means so switching/current-loop ripple does not
masquerade as uncertainty in the low-frequency delay slope.

At runtime `flux_linkage` directly drives back-EMF feedforward while the derived
`torque_constant` drives torque/current conversion. This keeps the two consumers
consistent and avoids storing two independently adjustable motor constants.

## Output-shaft mechanical model

After geometry/flux identification, firmware runs unloaded forward and reverse
segments at 0.02, 0.05, and 0.10 output turn/s. Each segment travels 0.25 output
turn with 0.20 turn/s^2 acceleration. A centered five-parameter regression fits
the output-coordinate model
`torque = J*acceleration + direction*Coulomb + B_direction*velocity`.

The fit rejects missing directions, insufficient acceleration, and singular
excitation. If noise, gravity load, or estimator phase lag makes an unconstrained
coefficient negative, the five-parameter fit is repeated as a nonnegative least-
squares problem; an unobservable component is fixed at zero rather than failing
the otherwise healthy calibration. Static friction is not admitted as
an independent parameter from this moving experiment; commit maps each Coulomb
candidate to the corresponding static-friction default unless a later dedicated
breakaway experiment proves separately observable. `J` is consumed by controller
acceleration feedforward, while the four asymmetric friction terms are consumed
by the existing friction feedforward path.

## Validation and atomic commit

FULL calibration admits a candidate only when all required validity bits are
present for the same session and electrical, geometry, flux, and mechanical
quality bounds pass together. No individual field is committed early. Firmware
then swaps the complete motor, encoder, geometry-LUT, inertia, and friction model
into the disarmed runtime under a critical section.

Persistence reuses the board NVM2 ping-pong sectors. Each complete configuration
record has a monotonic sequence, payload length, CRC32, header CRC32, and a final
flash allocation-state commit marker. Startup selects the newest complete valid
record. If saving fails, firmware restores the pre-commit runtime snapshots and
reports calibration failure code 8; a validation rejection reports code 7.
