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
| `0x010` | Start_Anticogging | Master -> Axis0 | implemented |
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

Subcommands:

| Subcmd | Name | Status |
| --- | --- | --- |
| `0x01` | Get_Axis_Status_Ex | implemented |
| `0x02` | Set_Precalibrated | implemented |
| `0x03` | Save_Configuration | implemented, resets after ACK |
| `0x04` | Get_Calib_Result | implemented |
| `0x05` | Get_Device_Info | implemented |
| `0x06` | Get_Basic_Config | implemented |
| `0x07` | Set_Basic_Config | implemented |
| `0x08` | Get_Anticogging_Status | implemented |
| `0x09` | Set_Anticogging_Config | implemented |
| `0x0A` | Get_Vernier_Diagnostics | implemented |
| `0x0B` | Get_Control_Config | implemented |
| `0x0C` | Set_Control_Config | implemented |
| `0x0D..0x1F` | Reserved | reserved for production protocol growth |

The current extended protocol version is returned by subcommand `0x05`, item
`0x01`, and is `0x00000103`.

Get_Device_Info items: `0x01` protocol_version, `0x02` fw_version,
`0x03` hw_version, `0x04`/`0x05` serial_number low/high 32 bits, `0x06`
user_config_loaded (uint32; NVM bytes loaded on boot, 0 = load failed and the
device is running factory defaults -- reconfigure and `save_configuration`).

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

## Anticogging extended items

Subcommand `0x08`, Get_Anticogging_Status:

| Item | Type | Meaning |
| --- | --- | --- |
| `0x01` | uint32 | flags: bit0 calib_anticogging, bit1 valid, bit2 pre_calibrated, bit3 enabled |
| `0x02` | uint32 | calibration index |
| `0x03` | float32 | calibration position threshold, encoder counts |
| `0x04` | float32 | calibration velocity threshold, encoder counts/s |
| `0x05` | float32 | cogging ratio, turns/index |
| `0x06` | uint32 | ODrive system error |
| `0x10` | float32 | cogging_map[index], request value is index in byte4..7 |

Subcommand `0x09`, Set_Anticogging_Config:

| Item | Type | Meaning |
| --- | --- | --- |
| `0x01` | uint32 | anticogging_enabled |
| `0x02` | uint32 | pre_calibrated, also sets runtime valid flag |
| `0x03` | float32 | calibration position threshold |
| `0x04` | float32 | calibration velocity threshold |
| `0x05` | uint32 | nonzero resets calibration state/index |

Setters return `BUSY_ARMED` while the motor is armed.

## Control configuration extended items

Subcommands `0x0B` and `0x0C`, Get/Set_Control_Config, use the same item IDs
(range `0x50`-`0x5D`). These cover the mode-aware command watchdog, heartbeat
watchdog, velocity/quick-stop ramp limits, and position profile limits. The
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
| `0x5C` | uint32 | heartbeat_timeout_ms, 0 = disabled |
| `0x5D` | float32 | pos_integrator_gain [(turn/s)/(turn*s)] |

The heartbeat (command `0x001`) controller flags byte (bits 56-63) mirrors a
subset of `control_runtime_state`: bit0 controller error present, bit1
comm_timeout, bit2 quick_stop_active, bit3 holding, bit4
command_watchdog_expired, bit5 mit_frame_stale, bit6 running, bit7
trajectory_done. Bit0 and bit7 are backward-compatible with older hosts.

## Vernier diagnostics extended items

Subcommand `0x0A` is diagnostic and may grow, but existing item IDs are frozen.
Selected diagnostic items include:

- `0x2C`: output_pair_vel_estimate, load-side velocity estimated from accepted Vernier pair samples
- `0x2D`: output_last_aux_correction, last low-frequency auxiliary position correction in output turns
- `0x30`: FOC_BAD_TIMING
- `0x34`: ADC_PRE
- `0x35`: ADC_POST
- `0x36`: DEADLINE_MISS
- `0x37`: SPI_PAIR_BUSY
- `0x38`: SPI_PAIR_OK

## Compatibility rule

Additive changes may only use explicitly reserved command or extended item
ranges. Any semantic change to an existing field, unit, scale, bit position, or
error/status value requires a protocol version bump.
