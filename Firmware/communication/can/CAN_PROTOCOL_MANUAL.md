# CAN Communication Protocol Manual

Document version: v1.0  
Firmware profile: single-axis ODrive v3.6 production firmware, MT6826S vernier output-shaft mode  
Default bitrate: 1 Mbps  
Default node ID: 0

This document describes the standard CAN interface exposed by the single-axis
servo firmware. It is written in a motor-driver vendor protocol style: bus
format first, then command table, frame definitions, MIT mode, extended
configuration, and common operation sequences.

## 1. Bus And Frame Format

### 1.1 CAN Physical Layer

| Item | Value |
| --- | --- |
| CAN type | Classic CAN 2.0 |
| Arbitration ID | Standard 11-bit ID |
| Default bitrate | 1 Mbps |
| Default node ID | 0 |
| Data length | 0 to 8 bytes |
| Byte order | Little-endian for normal scalar frames |
| MIT packed frame | Big-endian bit packing |

### 1.2 Arbitration ID

The 11-bit standard arbitration ID is:

```text
arb_id = (node_id << 5) | cmd_id
```

| Field | Bits | Range | Description |
| --- | ---: | --- | --- |
| node_id | 10:5 | 0..63 | Device node ID |
| cmd_id | 4:0 | 0x00..0x1F | Command ID |

Example for node `0`, command `0x00C Set_Input_Pos`:

```text
arb_id = (0 << 5) | 0x00C = 0x00C
```

Example for node `3`, command `0x001 Heartbeat`:

```text
arb_id = (3 << 5) | 0x001 = 0x061
```

### 1.3 Data Types

| Type | Size | Encoding |
| --- | ---: | --- |
| uint8 | 1 byte | Unsigned integer |
| uint16 | 2 bytes | Little-endian |
| int16 | 2 bytes | Little-endian |
| uint32 | 4 bytes | Little-endian |
| int32 | 4 bytes | Little-endian |
| uint64 | 8 bytes | Little-endian |
| float32 | 4 bytes | IEEE-754 little-endian |

Unless explicitly stated, position and velocity in normal CAN Simple frames are
in output-shaft turns and turns/s in the MT6826S vernier firmware profile.
Torque is output-shaft Nm in host-facing commands.

## 2. State And Mode Enums

### 2.1 Axis State

| Value | Name |
| ---: | --- |
| 0 | UNDEFINED |
| 1 | IDLE |
| 2 | STARTUP_SEQUENCE |
| 3 | FULL_CALIBRATION_SEQUENCE |
| 4 | MOTOR_CALIBRATION |
| 7 | ENCODER_OFFSET_CALIBRATION |
| 8 | CLOSED_LOOP_CONTROL |
| 9 | LOCKIN_SPIN |

### 2.2 Control Mode

| Value | Name |
| ---: | --- |
| 0 | VOLTAGE_CONTROL |
| 1 | TORQUE_CONTROL |
| 2 | VELOCITY_CONTROL |
| 3 | POSITION_CONTROL |

### 2.3 Input Mode

| Value | Name | Typical Use |
| ---: | --- | --- |
| 0 | INACTIVE | Disabled input |
| 1 | PASSTHROUGH | Direct command |
| 2 | VEL_RAMP | Velocity ramp mode |
| 3 | POS_FILTER | Reserved/compatibility |
| 4 | MIX_CHANNELS | Reserved/compatibility |
| 5 | TRAP_TRAJ | Profile position mode |
| 6 | TORQUE_RAMP | Torque ramp mode |
| 8 | TUNING | Test/tuning |
| 9 | MIT | MIT packed realtime control |

### 2.4 Servo Mode, Extended Item 0x5B

The servo mode is a higher-level mode selector exposed through
`Extended_Command` subcommand `0x0B/0x0C` item `0x5B`.

| Value | Name | Maps To | Default Timeout Action |
| ---: | --- | --- | --- |
| 0 | TORQUE | TORQUE_CONTROL + PASSTHROUGH | TORQUE_ZERO |
| 1 | VELOCITY | VELOCITY_CONTROL + VEL_RAMP | QUICK_STOP_AND_HOLD |
| 2 | PROFILE_POSITION | POSITION_CONTROL + TRAP_TRAJ | HOLD_LAST_POSITION |
| 3 | MIT_REALTIME | TORQUE_CONTROL + MIT | TORQUE_ZERO |
| 255 | UNKNOWN | Non-canonical pair | N/A |

## 3. Command Overview

| Cmd ID | Name | Direction | DLC | Description |
| ---: | --- | --- | ---: | --- |
| 0x000 | NMT | Host -> Drive | 0 | Master heartbeat feed |
| 0x001 | Heartbeat | Drive -> Host | 8 | Periodic state/error frame |
| 0x002 | Estop | Host -> Drive | 0 | Emergency stop |
| 0x003 | Get_Motor_Error | Drive -> Host | 8 | Motor error word, requestable |
| 0x004 | Get_Encoder_Error | Drive -> Host | 4 | Encoder error word, requestable |
| 0x006 | Set_Axis_Node_ID | Host -> Drive | 4 | Set node ID |
| 0x007 | Set_Axis_State | Host -> Drive | 4 | Request axis state |
| 0x009 | Get_Encoder_Estimates | Drive -> Host | 8 | Position and velocity estimate |
| 0x00A | Get_Encoder_Count | Drive -> Host | 8 | Raw count information |
| 0x00B | Set_Controller_Mode | Host -> Drive | 8 | Set control/input mode |
| 0x00C | Set_Input_Pos | Host -> Drive | 8 | Position command |
| 0x00D | Set_Input_Vel | Host -> Drive | 8 | Velocity command |
| 0x00E | Set_Input_Torque | Host -> Drive | 4 | Torque command |
| 0x00F | Set_Limits | Host -> Drive | 8 | Velocity/current limits |
| 0x011 | Set_Traj_Vel_Limit | Host -> Drive | 4 | Profile position velocity limit |
| 0x012 | Set_Traj_Accel_Limits | Host -> Drive | 8 | Profile acceleration/deceleration |
| 0x013 | Set_Traj_Inertia | Host -> Drive | 4 | Trajectory inertia feedforward |
| 0x014 | Get_Iq | Drive -> Host | 8 | Iq setpoint and measured current |
| 0x016 | Reboot | Host -> Drive | 0 | Reboot device |
| 0x017 | Get_Bus_Voltage_Current | Drive -> Host | 8 | DC bus voltage/current |
| 0x018 | Clear_Errors | Host -> Drive | 0 or 8 | Clear latched errors |
| 0x019 | Set_Linear_Count | Host -> Drive | 4 | Set encoder linear count |
| 0x01A | Set_Pos_Gain | Host -> Drive | 4 | Position proportional gain |
| 0x01B | Set_Vel_Gains | Host -> Drive | 8 | Velocity PI gains |
| 0x01C | Get_ADC_Voltage | Drive -> Host | 8 | ADC voltage, requestable |
| 0x01D | Get_Controller_Error | Drive -> Host | 4 | Controller error word |
| 0x01E | Extended_Command | Bidirectional | 8 | Vendor extension |
| 0x01F | Set_MIT_Control | Host -> Drive | 8 | MIT packed realtime command |

Requestable telemetry frames may be requested by sending a zero-length frame
with the same command ID. Some frames may also be emitted periodically by the
firmware depending on configuration.

## 4. Standard Frame Definitions

### 4.1 0x001 Heartbeat

Direction: Drive -> Host  
DLC: 8

| Byte | Type | Name | Description |
| ---: | --- | --- | --- |
| 0..3 | uint32 | axis_error | Axis error bitmask |
| 4 | uint8 | axis_state | AxisState enum |
| 5 | uint8 | motor_flags | bit0 = motor error present |
| 6 | uint8 | encoder_flags | bit0 = encoder error present |
| 7 | uint8 | controller_flags | Runtime flags |

Controller flags:

| Bit | Mask | Meaning |
| ---: | ---: | --- |
| 0 | 0x01 | controller error present |
| 1 | 0x02 | communication timeout |
| 2 | 0x04 | quick stop active |
| 3 | 0x08 | holding |
| 4 | 0x10 | command watchdog expired |
| 5 | 0x20 | MIT frame stale |
| 6 | 0x40 | controller running |
| 7 | 0x80 | trajectory done |

### 4.2 0x007 Set_Axis_State

Direction: Host -> Drive  
DLC: 4

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | uint32 | requested_state | AxisState |

Common values:

```text
1 = IDLE
8 = CLOSED_LOOP_CONTROL
```

### 4.3 0x009 Get_Encoder_Estimates

Direction: Drive -> Host  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | pos_estimate | turn |
| 4..7 | float32 | vel_estimate | turn/s |

### 4.4 0x00A Get_Encoder_Count

Direction: Drive -> Host  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | int32 | shadow_count | count |
| 4..7 | int32 | count_in_cpr | count |

### 4.5 0x00B Set_Controller_Mode

Direction: Host -> Drive  
DLC: 8

| Byte | Type | Name | Description |
| ---: | --- | --- | --- |
| 0..3 | uint32 | control_mode | ControlMode enum |
| 4..7 | uint32 | input_mode | InputMode enum |

Examples:

```text
Torque passthrough:    control_mode=1, input_mode=1
Velocity ramp:         control_mode=2, input_mode=2
Profile position:      control_mode=3, input_mode=5
MIT realtime:          control_mode=1, input_mode=9
```

### 4.6 0x00C Set_Input_Pos

Direction: Host -> Drive  
DLC: 8

| Byte | Type | Name | Unit / Scale |
| ---: | --- | --- | --- |
| 0..3 | float32 | input_pos | turn |
| 4..5 | int16 | vel_ff | raw * 0.001 turn/s |
| 6..7 | int16 | torque_ff | raw * 0.001 Nm |

This frame feeds the command watchdog.

### 4.7 0x00D Set_Input_Vel

Direction: Host -> Drive  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | input_vel | turn/s |
| 4..7 | float32 | torque_ff | Nm |

### 4.8 0x00E Set_Input_Torque

Direction: Host -> Drive  
DLC: 4

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | input_torque | Nm |

### 4.9 0x00F Set_Limits

Direction: Host -> Drive  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | velocity_limit | turn/s |
| 4..7 | float32 | current_limit | A |

`Set_Limits` updates the controller velocity limit and motor current limit. It
also clamps the profile velocity limit to the same velocity ceiling.

### 4.10 0x011 Set_Traj_Vel_Limit

Direction: Host -> Drive  
DLC: 4

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | traj_vel_limit | turn/s |

### 4.11 0x012 Set_Traj_Accel_Limits

Direction: Host -> Drive  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | accel_limit | turn/s^2 |
| 4..7 | float32 | decel_limit | turn/s^2 |

### 4.12 0x014 Get_Iq

Direction: Drive -> Host  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | iq_setpoint | A |
| 4..7 | float32 | iq_measured | A |

### 4.13 0x017 Get_Bus_Voltage_Current

Direction: Drive -> Host  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | vbus | V |
| 4..7 | float32 | ibus | A |

### 4.14 0x018 Clear_Errors

Direction: Host -> Drive  
DLC: 0 or 8

Clears latched axis, motor, encoder, controller, and stored fault snapshot
state where applicable.

### 4.15 0x019 Set_Linear_Count

Direction: Host -> Drive  
DLC: 4

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | int32 | linear_count | count |

Use carefully. It changes the encoder count reference used by the controller.

### 4.16 0x01A Set_Pos_Gain

Direction: Host -> Drive  
DLC: 4

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | pos_gain | (turn/s)/turn |

### 4.17 0x01B Set_Vel_Gains

Direction: Host -> Drive  
DLC: 8

| Byte | Type | Name | Unit |
| ---: | --- | --- | --- |
| 0..3 | float32 | vel_gain | Nm/(turn/s) |
| 4..7 | float32 | vel_integrator_gain | Nm/(turn/s)/s |

## 5. MIT Packed Realtime Control, Cmd 0x01F

Direction: Host -> Drive  
DLC: 8  
Active mode: `TORQUE_CONTROL + MIT`

The MIT frame follows the AK/T-Motor style packed 8-byte format.

### 5.1 Physical Ranges

| Field | Bits | Range | Unit |
| --- | ---: | --- | --- |
| p_des | 16 | -12.5 .. +12.5 | rad |
| v_des | 12 | -45 .. +45 | rad/s |
| kp | 12 | 0 .. 500 | Nm/rad |
| kd | 12 | 0 .. 5 | Nm/(rad/s) |
| t_ff | 12 | -18 .. +18 | Nm |

In MT6826S vernier mode, these units refer to the output shaft. The firmware
converts the commanded output-shaft torque to motor-side torque internally.

### 5.2 Byte Layout

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

### 5.3 Float To Unsigned Integer

For each field:

```text
raw = round((value - min) / (max - min) * (2^bits - 1))
raw = clamp(raw, 0, 2^bits - 1)
```

For signed fields (`p_des`, `v_des`, `t_ff`), firmware decodes both adjacent
center raw values as exactly zero to remove neutral-frame bias.

### 5.4 Realtime Requirement

MIT mode has a stale-frame safety path. The host should stream MIT frames
periodically. A practical command rate is 50 to 200 Hz; 100 Hz is recommended
for bring-up. If frames stop, firmware marks `mit_frame_stale` and sends the
configured timeout action path.

Recommended neutral frame before arming:

```text
p_des = 0
v_des = 0
kp = 0
kd = 0
t_ff = 0
```

## 6. Extended Command, Cmd 0x01E

`Extended_Command` provides versioned production configuration and diagnostics.
All extended command frames are 8 bytes.

### 6.1 Request Layout

```text
byte0    sub_cmd
byte1    item
byte2    type, 0 for read requests
byte3    reserved, set to 0
byte4..7 value, little-endian
```

### 6.2 Response Layout

```text
byte0    sub_cmd
byte1    item
byte2    status
byte3    type or auxiliary flags
byte4..7 value, little-endian
```

### 6.3 Type And Status Values

| Type | Name |
| ---: | --- |
| 1 | float32 |
| 2 | int32 |
| 3 | uint32 |

| Status | Name | Meaning |
| ---: | --- | --- |
| 0 | OK | Success |
| 1 | UNKNOWN | Unknown subcommand or item |
| 2 | READONLY | Item cannot be written |
| 3 | INVALID_TYPE | Setter type mismatch |
| 4 | INVALID_VALUE | Value out of range |
| 5 | BUSY_ARMED | Write refused while armed |

### 6.4 Subcommand Table

| Subcmd | Name | Description |
| ---: | --- | --- |
| 0x01 | Get_Axis_Status_Ex | Axis state, ready flags, axis error |
| 0x02 | Set_Precalibrated | Mark motor/encoder runtime ready flags |
| 0x03 | Save_Configuration | Save RAM config to NVM, then reset |
| 0x04 | Get_Calib_Result | Read calibration result |
| 0x05 | Get_Device_Info | Read protocol/firmware/device info |
| 0x06 | Get_Basic_Config | Read basic config item |
| 0x07 | Set_Basic_Config | Write basic config item |
| 0x0A | Get_Vernier_Diagnostics | Read vernier diagnostics |
| 0x0B | Get_Control_Config | Read control config item |
| 0x0C | Set_Control_Config | Write control config item |
| 0x0D | Vernier_Calibration | Firmware-side vernier multi-point calibration |
| 0x0E | Get_Fault_Snapshot | Read fault/overspeed snapshot |

### 6.5 Get_Axis_Status_Ex, Subcmd 0x01

Response:

| Byte | Name | Meaning |
| ---: | --- | --- |
| 1 | status | Axis state |
| 3 | flags | bit0 motor_calibrated, bit1 encoder_ready, bit5 trajectory_done |
| 4..7 | value | axis_error uint32 |

### 6.6 Device Info, Subcmd 0x05

| Item | Type | Name |
| ---: | --- | --- |
| 0x01 | uint32 | protocol_version |
| 0x02 | uint32 | fw_version |
| 0x03 | uint32 | hw_version |
| 0x04 | uint32 | serial_number_low |
| 0x05 | uint32 | serial_number_high |
| 0x06 | uint32 | user_config_loaded |
| 0x07 | uint32 | system_error |

The current protocol version is `0x0000010A`.

## 7. Basic Configuration Items

Subcommands:

```text
0x06 Get_Basic_Config
0x07 Set_Basic_Config
```

Setters generally return `BUSY_ARMED` while the motor is armed. Live-tunable
controller gains are exceptions.

| Item | Type | Access | Name | Unit / Description |
| ---: | --- | --- | --- | --- |
| 0x10 | uint32 | RO | motor_type | Baked motor model |
| 0x11 | int32 | RO | pole_pairs | Baked motor model |
| 0x14 | float32 | RW | current_lim | A |
| 0x15 | float32 | RO | torque_constant | Nm/A |
| 0x20 | uint32 | RO | encoder_mode | Baked encoder mode |
| 0x21 | int32 | RO | encoder_cpr | counts/rev |
| 0x23 | float32 | RW | encoder_bandwidth | rad/s |
| 0x28 | float32 | RW | vernier_main_offset | turn |
| 0x29 | float32 | RW | vernier_aux_offset | turn |
| 0x30 | float32 | RW live | pos_gain | (turn/s)/turn |
| 0x31 | float32 | RW live | vel_gain | Nm/(turn/s) |
| 0x32 | float32 | RW live | vel_integrator_gain | Nm/(turn/s)/s |
| 0x35 | float32 | RW live | pos_integrator_gain | (turn/s)/(turn*s) |
| 0x40 | float32 | RW | dc_max_negative_current | A |
| 0x41 | float32 | RW | dc_max_positive_current | A |
| 0x42 | float32 | RO | brake_resistance | ohm |
| 0x43 | float32 | RO | dc_bus_undervoltage_trip_level | V |
| 0x44 | float32 | RO | dc_bus_overvoltage_trip_level | V |

## 8. Control Configuration Items

Subcommands:

```text
0x0B Get_Control_Config
0x0C Set_Control_Config
```

Most writes require IDLE state. Save persistent changes with subcommand `0x03`
`Save_Configuration`.

| Item | Type | Access | Name | Unit / Meaning |
| ---: | --- | --- | --- | --- |
| 0x50 | float32 | RW | velocity_accel_limit | turn/s^2 |
| 0x51 | float32 | RW | velocity_decel_limit | turn/s^2 |
| 0x52 | float32 | RW | quick_stop_decel_limit | turn/s^2 |
| 0x53 | uint32 | RW | can_watchdog_timeout_ms | 0 disables |
| 0x54 | uint32 | RW | timeout_action | TimeoutAction enum |
| 0x55 | float32 | RW | profile_vel_limit | turn/s |
| 0x56 | float32 | RW | profile_accel_limit | turn/s^2 |
| 0x57 | float32 | RW | profile_decel_limit | turn/s^2 |
| 0x58 | uint32 | RO | control_runtime_state | runtime flags |
| 0x59 | uint32 | RO | last_timeout_reason | 0 none, 1 command, 2 heartbeat |
| 0x5A | uint32 | RO | trajectory_done | bool |
| 0x5B | uint32 | RW | servo_mode | ServoControlMode enum |
| 0x5C | uint32 | RW | heartbeat_timeout_ms | 0 disables |
| 0x5D | float32 | RW | pos_integrator_gain | (turn/s)/(turn*s) |
| 0x60 | float32 | RW | vel_limit | turn/s |
| 0x61 | float32 | RW | vel_limit_tolerance | ratio |
| 0x62 | uint32 | RW | enable_vel_limit | bool |
| 0x63 | uint32 | RW | enable_torque_mode_vel_limit | bool |
| 0x6D | float32 | RW | adrc_trim_slew_rate | Nm/s |
| 0x6E | uint32 | RW | enable_adrc | bool |
| 0x6F | float32 | RW | adrc_trim_torque_limit | Nm |
| 0x70 | uint32 | RW | enable_friction_compensation | bool, position/velocity modes |
| 0x71 | float32 | RW | friction_pos_deadband | turn |
| 0x72 | float32 | RW | friction_vel_deadband | turn/s |
| 0x73 | float32 | RW | friction_stribeck_vel | turn/s |
| 0x74 | float32 | RW | friction_static_pos | Nm |
| 0x75 | float32 | RW | friction_static_neg | Nm |
| 0x76 | float32 | RW | friction_coulomb_pos | Nm |
| 0x77 | float32 | RW | friction_coulomb_neg | Nm |
| 0x78 | float32 | RW | friction_viscous_pos | Nm/(turn/s) |
| 0x79 | float32 | RW | friction_viscous_neg | Nm/(turn/s) |
| 0x7A | float32 | RW | friction_max_torque | Nm, inf disables clamp |
| 0x7B | float32 | RW | friction_torque_slew_rate | Nm/s, inf disables slew |
| 0x7C | uint32 | RW | enable_mit_friction_compensation | bool, MIT mode |

### 8.1 Timeout Action Enum

| Value | Name | Meaning |
| ---: | --- | --- |
| 0 | HOLD_LAST_POSITION | Hold current/last position |
| 1 | QUICK_STOP | Decelerate using quick stop |
| 2 | QUICK_STOP_AND_HOLD | Quick stop then hold |
| 3 | TORQUE_ZERO | Command zero torque |
| 4 | FAULT_DISABLE | Fault/disarm |

### 8.2 Runtime State Flags, Item 0x58

| Bit | Mask | Meaning |
| ---: | ---: | --- |
| 0 | 0x001 | enabled |
| 1 | 0x002 | running |
| 2 | 0x004 | holding |
| 3 | 0x008 | quick_stop_active |
| 4 | 0x010 | comm_timeout |
| 5 | 0x020 | command_watchdog_expired |
| 6 | 0x040 | mit_frame_stale |
| 7 | 0x080 | trajectory_done |
| 8 | 0x100 | heartbeat_expired |

## 9. Vernier Diagnostics, Subcmd 0x0A

Selected items:

| Item | Type | Name | Description |
| ---: | --- | --- | --- |
| 0x27 | uint32 | output_estimate_valid | Output estimate valid flag |
| 0x28 | float32 | output_pos_estimate | Output-shaft position, turn |
| 0x29 | float32 | output_vel_estimate | Output-shaft velocity, turn/s |
| 0x2C | float32 | output_pair_vel_estimate | Pair-sample velocity estimate |
| 0x2D | float32 | output_last_aux_correction | Last aux correction, turn |
| 0x30 | uint32 | FOC_BAD_TIMING | Timing diagnostic |
| 0x34 | uint32 | ADC_PRE | ADC timing diagnostic |
| 0x35 | uint32 | ADC_POST | ADC timing diagnostic |
| 0x36 | uint32 | DEADLINE_MISS | Deadline miss count |
| 0x37 | uint32 | SPI_PAIR_BUSY | SPI busy count |
| 0x38 | uint32 | SPI_PAIR_OK | SPI pair success count |

## 10. Vernier Calibration, Subcmd 0x0D

This subcommand performs firmware-side multi-point static calibration of the
dual-MT6826S vernier geometry offset. It is separate from motor electrical
offset calibration.

| Item | Request | Response | Description |
| ---: | --- | --- | --- |
| 0x00 | any | uint32 | Reset captured points |
| 0x01 | any | uint32 | Capture latest valid static point |
| 0x02 | float32 optional | float32 | Fit aux offset; request is search radius in turns |
| 0x03 | any | uint32 | Apply pending fit to RAM config |
| 0x04 | any | uint32 | Captured point count |
| 0x05 | any | uint32 | Pending fit valid flag |
| 0x06 | any | float32 | Fitted main offset |
| 0x07 | any | float32 | Fitted aux offset |
| 0x08 | any | float32 | RMS residual |
| 0x09 | any | float32 | Worst absolute residual |
| 0x0A | any | uint32 | Maximum captured point count |

Applying a fit updates RAM configuration only. Persist it with
`Save_Configuration` after validation.

## 11. Error Words

### 11.1 Axis Error, Heartbeat byte0..3

| Mask | Name |
| ---: | --- |
| 0x00001 | INVALID_STATE |
| 0x00040 | MOTOR_FAILED |
| 0x00100 | ENCODER_FAILED |
| 0x00200 | CONTROLLER_FAILED |
| 0x00800 | WATCHDOG_TIMER_EXPIRED |
| 0x01000 | ESTOP_REQUESTED |
| 0x02000 | OVER_TEMP |
| 0x04000 | UNKNOWN_POSITION |

### 11.2 Common Motor Error Bits

| Mask | Name |
| ---: | --- |
| 0x00000001 | PHASE_RESISTANCE_OUT_OF_RANGE |
| 0x00000002 | PHASE_INDUCTANCE_OUT_OF_RANGE |
| 0x00000008 | DRV_FAULT |
| 0x00000010 | CONTROL_DEADLINE_MISSED |
| 0x00000080 | MODULATION_MAGNITUDE |
| 0x00000400 | CURRENT_SENSE_SATURATION |
| 0x00001000 | CURRENT_LIMIT_VIOLATION |
| 0x00010000 | MODULATION_IS_NAN |
| 0x00200000 | CONTROLLER_FAILED |
| 0x04000000 | UNKNOWN_PHASE_ESTIMATE |
| 0x08000000 | UNKNOWN_PHASE_VEL |
| 0x10000000 | UNKNOWN_TORQUE |
| 0x20000000 | UNKNOWN_CURRENT_COMMAND |

### 11.3 Encoder Error Bits

| Mask | Name |
| ---: | --- |
| 0x0001 | UNSTABLE_GAIN |
| 0x0002 | CPR_POLEPAIRS_MISMATCH |
| 0x0004 | NO_RESPONSE |
| 0x0008 | UNSUPPORTED_ENCODER_MODE |
| 0x0040 | ABS_SPI_TIMEOUT |
| 0x0080 | ABS_SPI_COM_FAIL |
| 0x0100 | ABS_SPI_NOT_READY |
| 0x0200 | VERNIER_RESOLVER_FAIL |

### 11.4 Controller Error Bits

| Mask | Name |
| ---: | --- |
| 0x0001 | OVERSPEED |
| 0x0002 | INVALID_INPUT_MODE |
| 0x0004 | UNSTABLE_GAIN |
| 0x0020 | INVALID_ESTIMATE |
| 0x0040 | INVALID_CIRCULAR_RANGE |
| 0x0080 | SPINOUT_DETECTED |

## 12. Common Operation Sequences

### 12.1 Enter Profile Position Mode

```text
1. Clear_Errors                         cmd 0x018
2. Set_Controller_Mode                  cmd 0x00B, control=3, input=5
3. Set_Axis_State CLOSED_LOOP_CONTROL   cmd 0x007, value=8
4. Set_Input_Pos                        cmd 0x00C
```

Profile limits are configured by:

```text
0x011 Set_Traj_Vel_Limit
0x012 Set_Traj_Accel_Limits
or Extended Control Config 0x55/0x56/0x57
```

### 12.2 Enter Velocity Mode

```text
1. Set_Controller_Mode                  control=2, input=2
2. Set_Axis_State CLOSED_LOOP_CONTROL
3. Stream Set_Input_Vel                 cmd 0x00D
```

### 12.3 Enter Torque Mode

```text
1. Set_Controller_Mode                  control=1, input=1
2. Set_Axis_State CLOSED_LOOP_CONTROL
3. Stream Set_Input_Torque              cmd 0x00E
```

### 12.4 Enter MIT Realtime Mode

Recommended safe sequence:

```text
1. Clear_Errors
2. Set_Limits                           choose safe velocity/current limits
3. Set_Controller_Mode                  control=1, input=9
4. Send neutral MIT frames              cmd 0x01F, 20 to 30 frames
5. Set_Axis_State CLOSED_LOOP_CONTROL
6. Continue streaming MIT frames        50 to 200 Hz
7. On stop: send neutral MIT frames, then Set_Axis_State IDLE
```

### 12.5 Save Persistent Configuration

```text
1. Set_Axis_State IDLE
2. Write Basic/Control config items
3. Extended_Command subcmd 0x03 Save_Configuration
4. Device acknowledges and resets
```

## 13. Implementation Notes

1. Normal command frames and MIT frames feed the command watchdog. Read-only
   status requests do not.
2. Most persistent config writes are rejected while the axis is armed and
   return `BUSY_ARMED`.
3. The UI may display velocities in rpm. CAN wire units are turn/s.
4. MIT position and velocity are rad/rad/s, not turn/turn/s.
5. In this firmware profile, host-facing normal servo units are output-shaft
   units under vernier mode.
6. Reserved command IDs and item IDs must not be reused without a protocol
   version bump.
