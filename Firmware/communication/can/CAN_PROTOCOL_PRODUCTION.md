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

## MIT packed control, command `0x01F`

Frame length: 8 bytes. Big-endian AK/T-Motor-compatible bit packing.

| Field | Bits | Range | Unit |
| --- | --- | --- | --- |
| `p_des` | 16 | `-12.5 .. +12.5` | rad |
| `v_des` | 12 | `-45 .. +45` | rad/s |
| `kp` | 12 | `0 .. 500` | Nm/rad |
| `kd` | 12 | `0 .. 5` | Nm/(rad/s) |
| `t_ff` | 12 | `-18 .. +18` | Nm |

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
| `0x0B..0x1F` | Reserved | reserved for production protocol growth |

The current extended protocol version is returned by subcommand `0x05`, item
`0x01`, and is `0x00000100`.

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

## Vernier diagnostics extended items

Subcommand `0x0A` is diagnostic and may grow, but existing item IDs are frozen.
Items `0x30`, `0x34`, `0x35`, `0x36`, `0x37`, and `0x38` expose the minimal
timing/SPI counters used by production gates:

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
