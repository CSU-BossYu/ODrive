# Phase 0 protocol inventory

This is an inventory of the existing interface; Phase 0 does not change CAN
frames or item behavior.

## Version discrepancy

- `docs/PROJECT_OVERVIEW.md` reports product CAN protocol v1.9 (`0x00000109`).
- `Firmware/communication/can/can_simple.cpp` reports extended protocol
  `0x0000010B` (v1.11).
- `Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md` also reports
  `0x0000010B` (v1.11).
- `tools/odrive-cansimple.dbc` has database document version 0.5.6; this is
  not the firmware extended protocol version.

This mismatch is a baseline risk and is deliberately not resolved in Phase 0.
The firmware source and production protocol document are the detailed source
for the current extended command map until a protocol-owner decision is made.

## Standard command map

The current Classic CAN 2.0 interface uses 1 Mbps, standard 11-bit IDs,
`(node_id << 5) | command_id`, node 0, and one exposed axis. Implemented
commands are `0x000` NMT, `0x001` Heartbeat, `0x002` Estop, `0x003`
Get_Motor_Error, `0x004` Get_Encoder_Error, `0x006` Set_Axis_Node_ID, `0x007`
Set_Axis_State, `0x009` Get_Encoder_Estimates, `0x00A` Get_Encoder_Count,
`0x00B` Set_Controller_Mode, `0x00C` Set_Input_Pos, `0x00D` Set_Input_Vel,
`0x00E` Set_Input_Torque, `0x00F` Set_Limits, `0x011` Set_Traj_Vel_Limit,
`0x012` Set_Traj_Accel_Limits, `0x013` Set_Traj_Inertia, `0x014` Get_Iq,
`0x016` Reboot, `0x017` Get_Bus_Voltage_Current, `0x018` Clear_Errors,
`0x019` Set_Linear_Count, `0x01A` Set_Pos_Gain, `0x01B` Set_Vel_Gains,
`0x01C` Get_ADC_Voltage, `0x01D` Get_Controller_Error, `0x01E`
Extended_Command, and `0x01F` Set_MIT_Control. `0x005`, `0x008`, and `0x015`
are reserved.

## Extended items

Implemented subcommands are `0x01`, `0x02`, `0x03`, `0x04`, `0x05`, `0x06`,
`0x07`, `0x0A`, `0x0B`, `0x0C`, `0x0D`, `0x0E`, and `0x0F`; `0x10..0x1F` are
reserved. The detailed item ranges remain in
`Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md` and
`CAN_PROTOCOL_MANUAL.md`, including basic config, control config, Vernier
diagnostics, fault snapshot, Vernier calibration, and calibration-session
items. No new debug item is added by Phase 0.

## Bus-load baseline

No bus capture or hardware CAN load measurement was available in this
workspace. The nominal physical configuration is 1 Mbps. A measured typical
and worst-case utilization must be supplied by a hardware capture before a
protocol change can be accepted; it is not inferred from UI polling or source
code.
