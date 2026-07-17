# Single-axis production CAN protocol

This document freezes the CAN protocol used by the trimmed single-axis ODrive
v3.6 production firmware profile on branch `feature/mt6826s-encoder`.

The generated DBC is `tools/odrive-cansimple.dbc`.

## Transport

- CAN bitrate: 1 Mbps
- Default node ID: 0
- Standard ID by default
- Axis count: one axis, Axis0 only
- CAN frame ID format: `(node_id << 5) | command_id`
- Node ID bits: 6
- Command ID bits: 5
- Multi-axis IDs and axis1 messages are intentionally not exposed.

## Data encoding

- Normal CAN Simple scalar fields are little-endian.
- `float32` uses IEEE-754 little-endian.
- `int32`/`uint32` use little-endian.
- MIT packed control uses the AK/T-Motor big-endian bit layout documented below.

## Command map

| Cmd | Name | Direction | Status |
| --- | --- | --- | --- |
| `0x000` | CANOpen NMT | Master -> Axis0 | implemented |
| `0x001` | Heartbeat | Axis0 -> Master | implemented |
| `0x002` | Estop | Master -> Axis0 | implemented |
| `0x003` | Get_Motor_Error | Axis0 -> Master | implemented |
| `0x004` | Get_Encoder_Error | Axis0 -> Master | implemented |
| `0x005` | Reserved_005 | reserved | reserved, former sensorless error |
| `0x006` | Set_Axis_Node_ID | Master -> Axis0 | implemented |
| `0x007` | Set_Axis_State | Master -> Axis0 | implemented |
| `0x008` | Set_Axis_Startup_Config | Master -> Axis0 | reserved/no-op in this profile |
| `0x009` | Get_Encoder_Estimates | Axis0 -> Master | implemented |
| `0x00A` | Get_Encoder_Count | Axis0 -> Master | implemented |
| `0x00B` | Set_Controller_Mode | Master -> Axis0 | implemented |
| `0x00C` | Set_Input_Pos | Master -> Axis0 | implemented |
| `0x00D` | Set_Input_Vel | Master -> Axis0 | implemented |
| `0x00E` | Set_Input_Torque | Master -> Axis0 | implemented |
| `0x00F` | Set_Limits | Master -> Axis0 | implemented |
| `0x011` | Set_Traj_Vel_Limit | Master -> Axis0 | implemented |
| `0x012` | Set_Traj_Accel_Limits | Master -> Axis0 | implemented |
| `0x013` | Set_Traj_Inertia | Master -> Axis0 | implemented |
| `0x014` | Get_Iq | Axis0 -> Master | implemented |
| `0x015` | Reserved_015 | reserved | reserved, former sensorless estimates |
| `0x016` | Reboot | Master -> Axis0 | implemented |
| `0x017` | Get_Bus_Voltage_Current | Axis0 -> Master | implemented |
| `0x018` | Clear_Errors | Master -> Axis0 | implemented |
| `0x019` | Set_Linear_Count | Master -> Axis0 | implemented |
| `0x01A` | Set_Pos_Gain | Master -> Axis0 | implemented |
| `0x01B` | Set_Vel_Gains | Master -> Axis0 | implemented |
| `0x01C` | Get_ADC_Voltage | Axis0 -> Master | implemented |
| `0x01D` | Get_Controller_Error | Axis0 -> Master | implemented |
| `0x01E` | Extended_Command | bidirectional | implemented |
| `0x01F` | Set_MIT_Control | Master -> Axis0 | implemented |

Do not reuse reserved command IDs without bumping the extended protocol version.

## Control modes and input modes

Control modes:

| Value | Name |
| --- | --- |
| `0` | VOLTAGE_CONTROL |
| `1` | TORQUE_CONTROL |
| `2` | VELOCITY_CONTROL |
| `3` | POSITION_CONTROL |

Input modes:

| Value | Name |
| --- | --- |
| `0` | INACTIVE |
| `1` | PASSTHROUGH |
| `2` | VEL_RAMP |
| `3` | POS_FILTER |
| `4` | MIX_CHANNELS |
| `5` | TRAP_TRAJ |
| `6` | TORQUE_RAMP |
| `8` | TUNING |
| `9` | MIT |

`MIX_CHANNELS` is retained in enum space for compatibility but should not be
used by new single-axis applications.

Timeout actions (item `0x54` of Get/Set_Control_Config, `0x0B`/`0x0C`):

| Value | Name | Default for |
| --- | --- | --- |
| `0` | HOLD_LAST_POSITION | position modes (TRAP_TRAJ) |
| `1` | QUICK_STOP | explicit override |
| `2` | QUICK_STOP_AND_HOLD | velocity modes (also the auto default value) |
| `3` | TORQUE_ZERO | torque and MIT modes |
| `4` | FAULT_DISABLE | explicit override |

When `timeout_action` is left at the default `QUICK_STOP_AND_HOLD` (value `2`),
the firmware picks a mode-aware action from the table above. Any other value is
treated as an explicit override.

Servo control modes (item `0x5B` of Get/Set_Control_Config, `0x0B`/`0x0C`)
map a business-layer mode to a canonical `(ControlMode, InputMode)` pair and a
default timeout action. The getter derives the current mode from the active
pair; a non-canonical pair reads back as `0xFF`:

| Value | Name | Maps to | Default timeout action |
| --- | --- | --- | --- |
| `0` | TORQUE | TORQUE_CONTROL + PASSTHROUGH | TORQUE_ZERO |
| `1` | VELOCITY | VELOCITY_CONTROL + VEL_RAMP | QUICK_STOP_AND_HOLD |
| `2` | PROFILE_POSITION | POSITION_CONTROL + TRAP_TRAJ | HOLD_LAST_POSITION |
| `3` | MIT_REALTIME | TORQUE_CONTROL + MIT | TORQUE_ZERO |

## MIT packed control, command `0x01F`

Frame length: 8 bytes. Big-endian AK/T-Motor-compatible bit packing.

| Field | Bits | Range | Unit |
| --- | --- | --- | --- |
| `p_des` | 16 | `-12.5 .. +12.5` | rad |
| `v_des` | 12 | `-45 .. +45` | rad/s |
| `kp` | 12 | `0 .. 500` | Nm/rad |
| `kd` | 12 | `0 .. 5` | Nm/(rad/s) |
| `t_ff` | 12 | `-18 .. +18` | Nm |

In MT6826S Vernier mode these position, velocity, gain, and torque units refer
to the output shaft. Firmware converts the resulting torque to motor-side
torque by the configured main ratio before FOC.

For the signed fields whose ranges straddle zero (`p_des`, `v_des`, `t_ff`),
the two adjacent center raw codes are both decoded as exactly `0.0`. This
removes the otherwise unavoidable one-half-LSB positive/negative bias in
neutral MIT frames.

Layout:

```text
byte0: p[15:8]
byte1: p[7:0]
byte2: v[11:4]
byte3: v[3:0] | kp[11:8]
byte4: kp[7:0]
byte5: kd[11:4]
byte6: kd[3:0] | t[11:8]
byte7: t[7:0]
```

The controller only acts on MIT frames when:

```text
control_mode = TORQUE_CONTROL
input_mode = MIT
```

Otherwise frames only update the stored MIT input values.

## Extended command `0x01E`

Request layout:

```text
byte0 = sub_cmd
byte1 = item / parameter
byte2 = requested type for setters, otherwise 0
byte3 = reserved
byte4..7 = value, little-endian
```

Response layout:

```text
byte0 = sub_cmd
byte1 = item / parameter
byte2 = status
byte3 = type / aux
byte4..7 = value, little-endian
```

Types:

| Value | Type |
| --- | --- |
| `1` | float32 |
| `2` | int32 |
| `3` | uint32 |

Statuses:

| Value | Name |
| --- | --- |
| `0` | OK |
| `1` | UNKNOWN |
| `2` | READONLY |
| `3` | INVALID_TYPE |
| `4` | INVALID_VALUE |
| `5` | BUSY_ARMED |
| `6` | STORAGE_ERROR |

Subcommands:

| Subcmd | Name | Status |
| --- | --- | --- |
| `0x01` | Get_Axis_Status_Ex | implemented |
| `0x02` | Set_Precalibrated | implemented |
| `0x03` | Save_Configuration | implemented, ACK reports the Flash commit result; no reboot |
| `0x04` | Get_Calib_Result | implemented |
| `0x05` | Get_Device_Info | implemented |
| `0x06` | Get_Basic_Config | implemented |
| `0x07` | Set_Basic_Config | implemented |
| `0x0A` | Get_Vernier_Diagnostics | implemented |
| `0x0B` | Get_Control_Config | implemented |
| `0x0C` | Set_Control_Config | implemented |
| `0x0D` | Vernier_Calibration | implemented |
| `0x0E` | Get_Fault_Snapshot | implemented (overspeed/fault snapshot, items 0x40-0x5F) |
| `0x0F` | Calibration_Session | implemented (runtime transaction envelope) |
| `0x10..0x1F` | Reserved | reserved for production protocol growth |

The current extended protocol version is returned by subcommand `0x05`, item
`0x01`, and is `0x0000010B` (v1.11: added the runtime Calibration_Session
transaction envelope on subcommand `0x0F`). v1.10: Save_Configuration no longer reboots and
returns `STORAGE_ERROR` when the verified Flash commit fails). v1.9 added
`Get_Fault_Snapshot` subcommand `0x0E`
as the clean home for the overspeed/fault snapshot — items `0x40`-`0x5F` moved
here from `0x0A` (which keeps them for backward compat, deprecated); added
controller velocity-limit config items `0x60`-`0x63` to Control Config). v1.8
added firmware-side `Vernier_Calibration` subcommand `0x0D` for multi-point
static fitting of `vernier_aux_offset`. v1.7 added ADRC trim params
`0x6D`-`0x6F` and friction-compensation params `0x70`-`0x7C` to Control Config.

Get_Device_Info items: `0x01` protocol_version, `0x02` fw_version,
`0x03` hw_version, `0x04`/`0x05` serial_number low/high 32 bits, `0x06`
user_config_loaded (uint32; NVM bytes loaded on boot, 0 = load failed and the
device is running factory defaults -- reconfigure and `save_configuration`),
`0x07` system_error (uint32; ODrive board-level error word, e.g.
DC_BUS_UNDER/OVER_VOLTAGE).

## Basic configuration extended items

Subcommands `0x06` and `0x07`, Get/Set_Basic_Config, use the same item IDs.
Motor/encoder model parameters are baked per motor model in
`production_config.h`. The identity params (motor_type, pole_pairs,
torque_constant, encoder mode/cpr, brake_resistance, dc_bus thresholds) are
exposed GET-only (readable for verification, return `UNKNOWN` on set). The
remaining model params (calibration_current, resistance_calib_max_voltage,
encoder CS, vernier geometry/SPI/thresholds) are not exposed at all.

Setters return `BUSY_ARMED` while the motor is armed, except for the
controller gains (`0x30`-`0x32`, `0x35`) which are live-tunable.

| Item | Type | Meaning |
| --- | --- | --- |
| `0x10` | uint32 | motor_type (GET-only, baked) |
| `0x11` | int32 | pole_pairs (GET-only, baked) |
| `0x15` | float32 | torque_constant [Nm/A] (GET-only, baked) |
| `0x20` | uint32 | encoder mode (GET-only, baked) |
| `0x21` | int32 | encoder cpr (GET-only, baked) |
| `0x14` | float32 | motor current_lim [A] (customer-tunable per load) |
| `0x23` | float32 | encoder bandwidth [rad/s] |
| `0x28` | float32 | vernier_main_offset [turn] (per-unit calibration) |
| `0x29` | float32 | vernier_aux_offset [turn] (per-unit calibration) |
| `0x30` | float32 | pos_gain [(turn/s)/turn] (live-tunable) |
| `0x31` | float32 | vel_gain [Nm/(turn/s)] (live-tunable) |
| `0x32` | float32 | vel_integrator_gain [Nm/(turn/s)/s] (live-tunable) |
| `0x35` | float32 | pos_integrator_gain [(turn/s)/(turn*s)] (live-tunable) |
| `0x40` | float32 | dc_max_negative_current [A] |
| `0x41` | float32 | dc_max_positive_current [A] |
| `0x42` | float32 | brake_resistance [ohm] (GET-only, baked) |
| `0x43` | float32 | dc_bus_undervoltage_trip_level [V] (GET-only, baked) |
| `0x44` | float32 | dc_bus_overvoltage_trip_level [V] (GET-only, baked) |

## Control configuration extended items

Subcommands `0x0B` and `0x0C`, Get/Set_Control_Config, use the same item IDs
(range `0x50`-`0x7C`). These cover the mode-aware command watchdog, heartbeat
watchdog, velocity/quick-stop ramp limits, position profile limits, and
bounded ADRC/friction-compensation params. The
control-timeout fields are persistent configuration:
`velocity_accel_limit`/`velocity_decel_limit`/`quick_stop_decel_limit`/`timeout_action`
live in `controller.config`; `can_watchdog_timeout_ms`/`heartbeat_timeout_ms`
live in `axis.config.can`. They are saved by `save_configuration` (command
`0x03`) and restored on boot. Like Set_Basic_Config, Set_Control_Config
refuses writes while any motor is armed (`BUSY_ARMED`); disarm, edit, save,
then re-enter closed loop. Items `0x58`-`0x5A` are readonly and return
`READONLY` on set.

The command watchdog is the motion-control-layer timeout, distinct from the
legacy axis safety watchdog (`config.enable_watchdog` /
`config.watchdog_timeout`, which remains a hard-disarm fault layer). It is fed
only by motion command frames: `0x00B` Set_Controller_Modes, `0x00C`
Set_Input_Pos, `0x00D` Set_Input_Vel, `0x00E` Set_Input_Torque, and `0x01F`
Set_MIT_Control. Read/status/config frames do not feed it. On expiry
(`can_watchdog_timeout_ms` with no motion command), the firmware executes the
mode-aware timeout action. A fresh motion command clears the timeout state.

A separate heartbeat watchdog (`heartbeat_timeout_ms`, item `0x5C`) is fed only
by master heartbeat/NMT frames (`0x000`, `0x001`, `0x700`). On expiry the same
timeout action path runs with `last_timeout_reason = 2`.

| Item | Type | Meaning |
| --- | --- | --- |
| `0x50` | float32 | velocity_accel_limit [turn/s²] for VEL_RAMP acceleration |
| `0x51` | float32 | velocity_decel_limit [turn/s²] for VEL_RAMP deceleration |
| `0x52` | float32 | quick_stop_decel_limit [turn/s²] for command-timeout quick stop |
| `0x53` | uint32 | can_watchdog_timeout_ms, 0 = disabled |
| `0x54` | uint32 | timeout_action enum (TimeoutAction) |
| `0x55` | float32 | profile_vel_limit [turn/s], mirrors `trap_traj.config.vel_limit` |
| `0x56` | float32 | profile_accel_limit [turn/s²], mirrors `trap_traj.config.accel_limit` |
| `0x57` | float32 | profile_decel_limit [turn/s²], mirrors `trap_traj.config.decel_limit` |
| `0x58` | uint32 | control_runtime_state flags (readonly): bit0 enabled, bit1 running, bit2 holding, bit3 quick_stop_active, bit4 comm_timeout, bit5 command_watchdog_expired, bit6 mit_frame_stale, bit7 trajectory_done, bit8 heartbeat_expired |
| `0x59` | uint32 | last_timeout_reason (readonly): 0=none, 1=command_watchdog, 2=heartbeat |
| `0x5A` | uint32 | trajectory_done (readonly), bool |
| `0x5B` | uint32 | servo_mode enum (ServoControlMode); setter maps to `(ControlMode, InputMode)` plus default timeout_action |
| `0x5C` | uint32 | heartbeat_timeout_ms, 0 = disabled; otherwise minimum 250 ms |
| `0x5D` | float32 | pos_integrator_gain [(turn/s)/(turn*s)] |
| `0x60` | float32 | vel_limit [turn/s], `controller.config.vel_limit`; +inf disables. Setter also clamps `trap_traj.config.vel_limit` (same side effect as `Set_Limits` 0x00F) |
| `0x61` | float32 | vel_limit_tolerance [ratio], `controller.config.vel_limit_tolerance`; +inf disables the overspeed check |
| `0x62` | uint32 | enable_vel_limit (bool), `controller.config.enable_vel_limit` |
| `0x63` | uint32 | enable_torque_mode_vel_limit (bool), `controller.config.enable_torque_mode_vel_limit`; set 0 to stop velocity-estimate clamping of torque in current/torque mode |
| `0x6D` | float32 | adrc_trim_slew_rate [Nm/s], controller/output-shaft in vernier mode |
| `0x6E` | uint32 | enable_adrc (bool), bounded disturbance trim after PI/MIT torque |
| `0x6F` | float32 | adrc_trim_torque_limit [Nm], controller/output-shaft in vernier mode; 0 = disabled |
| `0x70` | uint32 | enable_friction_compensation (bool), position/velocity servo modes |
| `0x71` | float32 | friction_pos_deadband [turn], output shaft |
| `0x72` | float32 | friction_vel_deadband [turn/s], output shaft |
| `0x73` | float32 | friction_stribeck_vel [turn/s], output shaft |
| `0x74` | float32 | friction_static_pos [Nm], output shaft |
| `0x75` | float32 | friction_static_neg [Nm], output shaft |
| `0x76` | float32 | friction_coulomb_pos [Nm], output shaft |
| `0x77` | float32 | friction_coulomb_neg [Nm], output shaft |
| `0x78` | float32 | friction_viscous_pos [Nm/(turn/s)] |
| `0x79` | float32 | friction_viscous_neg [Nm/(turn/s)] |
| `0x7A` | float32 | friction_max_torque [Nm], inf = disabled |
| `0x7B` | float32 | friction_torque_slew_rate [Nm/s], inf = disabled |
| `0x7C` | uint32 | enable_mit_friction_compensation (bool), MIT packed-control servo mode |

Normal host UI controls use finite values for `0x60`/`0x61`; disable the
controller velocity limit with `0x62 = 0`. The wire protocol still accepts
`+inf` for low-level tooling that wants the original ODrive-style disable
sentinel.

The heartbeat (command `0x001`) controller flags byte (bits 56-63) mirrors a
subset of `control_runtime_state`: bit0 controller error present, bit1
comm_timeout, bit2 quick_stop_active, bit3 holding, bit4
command_watchdog_expired, bit5 mit_frame_stale, bit6 running, bit7
trajectory_done. Bit0 and bit7 are backward-compatible with older hosts.

## Vernier diagnostics extended items

Subcommand `0x0A` is diagnostic and may grow, but existing item IDs are frozen.

> **Deprecated:** items `0x40`-`0x5F` (the controller OVERSPEED/fault snapshot)
> remain in `0x0A` for backward compatibility but are now served by
> `0x0E Get_Fault_Snapshot`. Hosts should read fault snapshots via `0x0E`.

Selected diagnostic items include:

- `0x2C`: output_pair_vel_estimate, load-side velocity estimated from accepted Vernier pair samples
- `0x2D`: output_last_aux_correction, last low-frequency auxiliary position correction in output turns
- `0x30`: FOC_BAD_TIMING
- `0x34`: ADC_PRE
- `0x35`: ADC_POST
- `0x36`: DEADLINE_MISS
- `0x37`: SPI_PAIR_BUSY
- `0x38`: SPI_PAIR_OK

## Vernier calibration extended items

Subcommand `0x0D` performs firmware-side multi-point static calibration of the
dual-MT6826S vernier geometry offset. It is separate from the normal encoder
electrical offset calibration. Items `0x00` (reset), `0x02` (fit), and `0x03`
(apply) return `BUSY_ARMED` while any motor is armed; `0x01` (capture) is
allowed while armed so the host auto-sampling flow can capture at each held
position without arm/disarm cycles. Applying a fit updates RAM
configuration only; persist with subcommand `0x03 Save_Configuration` after
validation.

| Item | Request | Response | Meaning |
| --- | --- | --- | --- |
| `0x00` | any | uint32 | reset captured calibration points |
| `0x01` | any | uint32 | capture latest valid static main/aux point; returns point count |
| `0x02` | float32 optional | float32 | fit aux offset around current value; request value is search radius in turns, default 0.05 |
| `0x03` | any | uint32 | apply pending fit to `encoder.config.vernier_aux_offset` |
| `0x04` | any | uint32 | captured point count |
| `0x05` | any | uint32 | pending fit valid |
| `0x06` | any | float32 | fitted main offset; currently mirrors current main offset |
| `0x07` | any | float32 | fitted aux offset |
| `0x08` | any | float32 | RMS residual of latest fit |
| `0x09` | any | float32 | worst absolute residual of latest fit |
| `0x0A` | any | uint32 | maximum captured point count |

## Calibration session extended items

Subcommand `0x0F` is the single public command family for calibration. The host
starts or aborts a calibration, but does not drive individual calibration
stages. After `START` is accepted, firmware owns the complete stage sequence,
data collection, fitting, validation, and transactional result commit.

A CAN request cannot remain pending for the duration of a multi-second
calibration. `START` therefore returns an immediate acknowledgement. The host
may poll the read-only status items below, and reads the final result from a
reserved result-item range in this same command family. A future asynchronous
completion notification may be added without exposing stage control.

The session is runtime-only. Committed result fields are persisted separately
and replace active configuration only after validation succeeds. `START` is
rejected with `BUSY_ARMED` while any axis is armed; `ABORT` remains available
during powered calibration motion.

The valid forward path is:

```text
EMPTY -> COLLECTING -> COLLECTED -> FITTING -> IDENTIFIED
      -> VALIDATING -> VALIDATED -> STAGED -> COMMITTED
```

These states describe firmware-internal progress; none is directly writable by
the host. An active session can transition to `FAILED` or `ABORTED`. Firmware
marks an identified or newer result `STALE` when an upstream dependency
changes.

| Item | Request | Response | Meaning |
| --- | --- | --- | --- |
| `0x00` | read | uint32 | session schema version, currently 1 |
| `0x01` | read | uint32 | firmware-generated session ID |
| `0x02` | read | uint32 | state enum: EMPTY=0 through STALE=11 |
| `0x03` | read | uint32 | firmware-owned calibration stage ID |
| `0x04` | read | uint32 | nonzero failure code for FAILED state |
| `0x05` | read | uint32 | flags: active, identified, validated, applicable, staged, committed, stale, terminal |
| `0x06` | read | uint32 | monotonic runtime transition counter |
| `0x07` | read | uint32 | accepted request options; profile is stored in bits 7:0 |
| `0x08` | read | uint32 | overall progress in permille, 0 through 1000 |
| `0x09` | read | uint32 | records currently buffered for transport/fitting |
| `0x0A` | read | uint32 | records dropped because the non-blocking buffer was full |
| `0x0B` | read | uint32 | session buffer high-water mark in records |
| `0x0C` | read | uint32 | record buffer capacity |
| `0x0D` | read | uint32 | CRC-protected USB calibration frames queued |
| `0x0E` | read | uint32 | USB frame enqueue retries caused by backpressure |
| `0x0F` | read | uint32 | transport polling intervals spent disconnected |
| `0x10` | uint32 | uint32 | START; profiles: 0=full, 1=electrical, 2=mechanical, 3=validate-only; remaining bits are option flags |
| `0x11` | any | uint32 | ABORT the active calibration |
| `0x20` | read | uint32 | pending-result validity bits: R, L, direction, electrical offset, geometry model |
| `0x21` | read | float32 | candidate phase resistance [ohm], not active until commit |
| `0x22` | read | float32 | candidate phase inductance [H], not active until commit |
| `0x23` | read | int32 | candidate encoder direction |
| `0x24` | read | int32 | candidate integer electrical phase offset [counts] |
| `0x25` | read | float32 | candidate fractional electrical phase offset [counts] |
| `0x26` | read | float32 | candidate effective output-ratio scale |
| `0x27` | read | float32 | geometry residual RMS before LUT compensation [output turn] |
| `0x28` | read | float32 | geometry residual RMS after LUT compensation [output turn] |
| `0x29` | read | float32 | peak-to-peak direction-dependent geometry term [output turn] |
| `0x2A` | read | uint32 | samples admitted to the two-pass geometry fitter |
| `0x2B` | read | float32 | candidate PM flux linkage [V/(electrical rad/s)] |
| `0x2C` | read | float32 | derived candidate motor torque constant [Nm/Aq] |
| `0x2D` | read | float32 | flux sample standard deviation |
| `0x2E` | read | uint32 | samples admitted to flux identification |
| `0x2F` | read | int32 | candidate integer motor pole-pair count |
| `0x30` | read | float32 | candidate output equivalent inertia [Nm/(turn/s^2)] |
| `0x31` | read | float32 | positive-direction Coulomb friction [output Nm] |
| `0x32` | read | float32 | negative-direction Coulomb friction magnitude [output Nm] |
| `0x33` | read | float32 | positive-direction viscous coefficient [Nm/(turn/s)] |
| `0x34` | read | float32 | negative-direction viscous coefficient [Nm/(turn/s)] |
| `0x35` | read | float32 | mechanical fit residual RMS [output Nm] |
| `0x36` | read | uint32 | samples admitted to mechanical fit |
| `0x37` | read | float32 | candidate electrical phase delay [s] |
| `0x38` | read | float32 | delay-fit residual phase intercept [electrical rad] |
| `0x39` | read | float32 | delay-fit residual RMS [electrical rad] |
| `0x3A` | read | uint32 | samples admitted to electrical-delay fit |
| `0x3B` | read | uint32 | mechanical samples attempted |
| `0x3C` | read | uint32 | mechanical samples rejected for invalid current/encoder |
| `0x3D` | read | uint32 | mechanical samples rejected for PWM saturation |
| `0x3E` | read | uint32 | mechanical samples rejected below velocity threshold |
| `0x3F` | read | float32 | maximum measured mechanical speed [output turn/s] |
| `0x40` | read | uint32 | mechanical samples rejected for invalid timestamp delta |
| `0x41` | read | uint32 | electrical-delay samples attempted |
| `0x42` | read | uint32 | delay samples rejected for invalid current/voltage |
| `0x43` | read | uint32 | delay samples rejected for PWM saturation |
| `0x44` | read | uint32 | delay samples rejected for low electrical speed |
| `0x45` | read | uint32 | delay samples rejected for insufficient back-EMF |
| `0x46` | read | uint32 | delay samples rejected for implausible phase error |
| `0x47` | read | float32 | maximum measured electrical speed [rad/s] |
| `0x48`-`0x7F` | read | typed | reserved for additional result fields and metadata |

Coarse stage IDs are firmware-owned and stable for monitoring: `0` none, `1`
precheck, `10` electrical capture, `20` encoder geometry, `30` mechanical
capture, `35` electrical delay, `40` fitting, `50` validation, and `60` commit. Experiment-specific
substeps are deliberately not part of the CAN contract.

For `FULL`, request-option bits `15:8` select the bidirectional geometry scan
length in output-shaft turns (`1` through `8`, `0` selects the default `2`). The
mechanism must be unloaded and free to rotate through that range. Firmware
scans the requested distance forward and backward twice at 0.05 output turn/s.
The first pair identifies effective ratio scale; the second pair accumulates
the common and directional 64-bin residual tables. The on-device fitter consumes
samples synchronously; optional USB-copy loss is reported by item `0x0A` but
does not corrupt or invalidate the fitted candidate.

All four segments also feed bidirectional flux identification. The result is
admitted only when each direction has at least 128 usable unsaturated samples
and within-direction dispersion is below 25% of the fitted flux. Torque constant
item `0x2C` is derived from flux item `0x2B`, not independently fitted.

The mechanical stage temporarily applies the validated electrical/encoder
candidate and runs six closed-loop, unloaded velocity ramp-and-return segments:
forward and reverse at 0.02, 0.05, and 0.10 output turn/s. Measured closed-loop
Iq, output velocity, and acceleration identify output inertia and asymmetric
Coulomb/viscous friction. Existing mechanical feed-forward is disabled during
this experiment and the prior runtime configuration is restored afterward.
The electrical-delay stage then runs closed-loop at three speeds in both
directions and regresses back-EMF phase error against electrical speed. The
accepted slope is persisted as encoder phase advance and is consumed directly
by the runtime electrical phase calculation.
With the default two-turn geometry option, the full motion portion is roughly
215 seconds plus alignment and speed-dependent settling.

For `FULL`, successful identification automatically advances through fitting,
validation, staging, and atomic NVM commit; the host does not send per-stage or
save commands. Failure code `7` means the complete candidate failed physical or
cross-parameter validation. Failure code `8` means atomic configuration storage
failed and the previous runtime configuration was restored. Failure code `9`
means the closed-loop electrical-delay experiment or fit failed. Failure code
`10`, `11`, and `12` respectively mean insufficient steady-speed flux blocks,
a nonphysical flux mean, or excessive block-mean dispersion. A valid geometry
candidate and invalid (grey) flux diagnostics may still be reported, but nothing
is committed.
Failure codes `13`, `14`, and `15` respectively identify insufficient closed-loop
mechanical excitation, a singular five-parameter regression, or nonphysical
inertia/friction estimates. Invalid signed fit diagnostics can remain readable
without setting the mechanical validity bit.
Failure codes `16` and `17` distinguish failure to arm the temporary candidate
closed loop from loss of controlled motion or exceeding the calibration speed
safety bound. Exact low-speed setpoint tracking is not a pass condition; the
fitter uses measured motion and separately enforces observability.
Failure codes `18`, `19`, and `20` identify all-invalid acquisition, all-PWM-
saturated acquisition, and no measurable output motion respectively.
Failure code `21` identifies an invalid CPU-cycle timebase. Firmware explicitly
enables DWT CYCCNT during board initialization so standalone behavior no longer
depends on whether a debug probe enabled the counter.
Failure codes `22` through `25` distinguish delay-stage closed-loop startup,
insufficient bidirectional samples, unobservable speed regression, and a fitted
delay/intercept/RMS outside physical quality limits.

## Compatibility rule

Additive changes may only use explicitly reserved command or extended item
ranges. Any semantic change to an existing field, unit, scale, bit position, or
error/status value requires a protocol version bump.
