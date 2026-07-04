// Shared TypeScript types for the FOC upper-computer frontend.
//
// These mirror the backend message schema in
// F:\souce_code\foc_ui\backend\foc_backend\ws_hub.py and the channel table
// in telemetry.py. If either changes, update both sides.

export interface ChannelDef {
  index: number
  key: string
  label: string
  unit: string
  scale: number
  group: 'pos' | 'vel' | 'cur' | 'vol' | 'state'
}

export interface ErrorBitDef {
  bit: number
  name: string
  fatal: boolean
}

// --- Inbound WS messages (backend -> browser) ---

export interface TelemetryMsg {
  type: 'telemetry'
  t: number          // monotonic ms (backend clock)
  seq: number
  gap: number        // frames missing since previous
  ch: Record<string, number>   // 16 channel values keyed by ChannelDef.key
}

export interface LogMsg {
  type: 'log'
  t: number
  tag: string        // e.g. 'FOC', 'BOOT', 'CAN', '?'
  text: string
}

export interface CliMsg {
  type: 'cli'
  t: number
  line: string       // [CLI] tag already stripped; empty string when done=true
  done: boolean
}

export interface StatusMsg {
  type: 'status'
  t: number
  connected: boolean
  port: string
  frames: number
  bad_crc: number
  dropped: number
}

export interface CmdDoneMsg {
  type: 'cmd_done'
  is_error: boolean
  lines: string[]
  text: string
}

export interface RecStateMsg {
  type: 'rec_state'
  recording: boolean
  path: string | null
}

export interface ErrorMsg {
  type: 'error'
  msg: string
}

export type InboundMsg =
  | TelemetryMsg | LogMsg | CliMsg | StatusMsg
  | CmdDoneMsg | RecStateMsg | ErrorMsg

// --- Outbound WS messages (browser -> backend) ---

export interface CmdOut { type: 'cmd'; text: string }
export interface RawCliOut { type: 'rawcli'; text: string }
export interface SetOut { type: 'set'; key: string; value: number | [number, number] }
export interface RecStartOut { type: 'rec'; action: 'start'; path?: string }
export interface RecStopOut { type: 'rec'; action: 'stop' }

export type OutboundMsg = CmdOut | RawCliOut | SetOut | RecStartOut | RecStopOut

// --- Parsed state flags (channel 15), mirrors telemetry.py StateFlags ---

export interface StateFlags {
  running: boolean
  control_mode: number        // 0=current, 1=velocity, 2=position
  current_limited: boolean
  voltage_limited: boolean
  overspeed_error: boolean
  trajectory_done: boolean
}

export function decodeStateFlags(raw: number): StateFlags {
  // NOTE: firmware also writes overspeed_error to bit 8 (duplicate of bit 6);
  // we only read bit 6 to match the documented protocol.
  return {
    running: !!(raw & (1 << 0)),
    control_mode: (raw >> 1) & 0x07,
    current_limited: !!(raw & (1 << 4)),
    voltage_limited: !!(raw & (1 << 5)),
    overspeed_error: !!(raw & (1 << 6)),
    trajectory_done: !!(raw & (1 << 7)),
  }
}

export const MODE_NAMES: Record<number, string> = {
  0: 'current',
  1: 'velocity',
  2: 'position',
}

// ===================================================================
// ODrive CAN types  (added for ?mode=can workflow)
// ===================================================================

export type UIMode = 'serial' | 'can'

export interface ODriveChannelDef {
  index: number
  key: string
  label: string
  unit: string
  group: 'pos' | 'vel' | 'cur' | 'bus' | 'state'
}

export const AXIS_STATES: Record<number, string> = {
  0: 'UNDEFINED',
  1: 'IDLE',
  2: 'STARTUP_SEQUENCE',
  3: 'FULL_CALIBRATION_SEQUENCE',
  4: 'MOTOR_CALIBRATION',
  7: 'ENCODER_OFFSET_CALIBRATION',
  8: 'CLOSED_LOOP_CONTROL',
  9: 'LOCKIN_SPIN',
  11: 'HOMING',
}

export const CONTROL_MODES: Record<number, string> = {
  0: 'VOLTAGE_CONTROL',
  1: 'TORQUE_CONTROL',
  2: 'VELOCITY_CONTROL',
  3: 'POSITION_CONTROL',
}

export const INPUT_MODES: Record<number, string> = {
  0: 'INACTIVE',
  1: 'PASSTHROUGH',
  2: 'VEL_RAMP',
  3: 'POS_FILTER',
  4: 'MIX_CHANNELS',
  5: 'TRAP_TRAJ',
  6: 'TORQUE_RAMP',
  8: 'TUNING',
  9: 'MIT',
}

export interface ODriveErrorBitDef {
  mask: number
  name: string
  category: 'odrive' | 'axis' | 'motor' | 'encoder' | 'controller'
}

export const AXIS_ERROR_BITS: ODriveErrorBitDef[] = [
  { mask: 0x00001, name: 'INVALID_STATE',          category: 'axis' },
  { mask: 0x00040, name: 'MOTOR_FAILED',            category: 'axis' },
  { mask: 0x00100, name: 'ENCODER_FAILED',          category: 'axis' },
  { mask: 0x00200, name: 'CONTROLLER_FAILED',       category: 'axis' },
  { mask: 0x00800, name: 'WATCHDOG_TIMER_EXPIRED',  category: 'axis' },
  { mask: 0x01000, name: 'MIN_ENDSTOP_PRESSED',     category: 'axis' },
  { mask: 0x02000, name: 'MAX_ENDSTOP_PRESSED',     category: 'axis' },
  { mask: 0x04000, name: 'ESTOP_REQUESTED',         category: 'axis' },
  { mask: 0x20000, name: 'HOMING_WITHOUT_ENDSTOP',  category: 'axis' },
  { mask: 0x40000, name: 'OVER_TEMP',               category: 'axis' },
  { mask: 0x80000, name: 'UNKNOWN_POSITION',        category: 'axis' },
]

export const MOTOR_ERROR_BITS: ODriveErrorBitDef[] = [
  { mask: 0x00000001, name: 'PHASE_RESISTANCE_OUT_OF_RANGE', category: 'motor' },
  { mask: 0x00000002, name: 'PHASE_INDUCTANCE_OUT_OF_RANGE', category: 'motor' },
  { mask: 0x00000008, name: 'DRV_FAULT',                     category: 'motor' },
  { mask: 0x00000010, name: 'CONTROL_DEADLINE_MISSED',       category: 'motor' },
  { mask: 0x00000080, name: 'MODULATION_MAGNITUDE',          category: 'motor' },
  { mask: 0x00000400, name: 'CURRENT_SENSE_SATURATION',      category: 'motor' },
  { mask: 0x00001000, name: 'CURRENT_LIMIT_VIOLATION',       category: 'motor' },
  { mask: 0x00010000, name: 'MODULATION_IS_NAN',             category: 'motor' },
  { mask: 0x00020000, name: 'MOTOR_THERMISTOR_OVER_TEMP',    category: 'motor' },
  { mask: 0x00040000, name: 'FET_THERMISTOR_OVER_TEMP',      category: 'motor' },
  { mask: 0x00080000, name: 'TIMER_UPDATE_MISSED',           category: 'motor' },
  { mask: 0x00100000, name: 'CURRENT_MEASUREMENT_UNAVAILABLE', category: 'motor' },
  { mask: 0x00200000, name: 'CONTROLLER_FAILED',             category: 'motor' },
  { mask: 0x00400000, name: 'I_BUS_OUT_OF_RANGE',            category: 'motor' },
  { mask: 0x00800000, name: 'BRAKE_RESISTOR_DISARMED',       category: 'motor' },
  { mask: 0x01000000, name: 'SYSTEM_LEVEL',                  category: 'motor' },
  { mask: 0x02000000, name: 'BAD_TIMING',                    category: 'motor' },
  { mask: 0x04000000, name: 'UNKNOWN_PHASE_ESTIMATE',        category: 'motor' },
  { mask: 0x08000000, name: 'UNKNOWN_PHASE_VEL',             category: 'motor' },
  { mask: 0x10000000, name: 'UNKNOWN_TORQUE',                category: 'motor' },
  { mask: 0x20000000, name: 'UNKNOWN_CURRENT_COMMAND',       category: 'motor' },
  { mask: 0x40000000, name: 'UNKNOWN_CURRENT_MEASUREMENT',   category: 'motor' },
  { mask: 0x80000000, name: 'UNKNOWN_VBUS_VOLTAGE',          category: 'motor' },
  { mask: 0x100000000, name: 'UNKNOWN_VOLTAGE_COMMAND',      category: 'motor' },
  { mask: 0x200000000, name: 'UNKNOWN_GAINS',                category: 'motor' },
  { mask: 0x400000000, name: 'CONTROLLER_INITIALIZING',      category: 'motor' },
  { mask: 0x800000000, name: 'UNBALANCED_PHASES',            category: 'motor' },
]

export const ENCODER_ERROR_BITS: ODriveErrorBitDef[] = [
  { mask: 0x01, name: 'UNSTABLE_GAIN',             category: 'encoder' },
  { mask: 0x02, name: 'CPR_POLEPAIRS_MISMATCH',    category: 'encoder' },
  { mask: 0x04, name: 'NO_RESPONSE',               category: 'encoder' },
  { mask: 0x08, name: 'UNSUPPORTED_ENCODER_MODE',   category: 'encoder' },
  { mask: 0x40, name: 'ABS_SPI_TIMEOUT',           category: 'encoder' },
  { mask: 0x80, name: 'ABS_SPI_COM_FAIL',          category: 'encoder' },
  { mask: 0x100, name: 'ABS_SPI_NOT_READY',        category: 'encoder' },
  { mask: 0x200, name: 'VERNIER_RESOLVER_FAIL',    category: 'encoder' },
]

export const CONTROLLER_ERROR_BITS: ODriveErrorBitDef[] = [
  { mask: 0x01, name: 'OVERSPEED',               category: 'controller' },
  { mask: 0x02, name: 'INVALID_INPUT_MODE',      category: 'controller' },
  { mask: 0x04, name: 'UNSTABLE_GAIN',           category: 'controller' },
  { mask: 0x20, name: 'INVALID_ESTIMATE',        category: 'controller' },
  { mask: 0x40, name: 'INVALID_CIRCULAR_RANGE',  category: 'controller' },
  { mask: 0x80, name: 'SPINOUT_DETECTED',        category: 'controller' },
]

export const ODRIVE_ERROR_BITS: ODriveErrorBitDef[] = [
  { mask: 0x00000001, name: 'CONTROL_ITERATION_MISSED', category: 'odrive' },
  { mask: 0x00000002, name: 'DC_BUS_UNDER_VOLTAGE', category: 'odrive' },
  { mask: 0x00000004, name: 'DC_BUS_OVER_VOLTAGE', category: 'odrive' },
  { mask: 0x00000008, name: 'DC_BUS_OVER_REGEN_CURRENT', category: 'odrive' },
  { mask: 0x00000010, name: 'DC_BUS_OVER_CURRENT', category: 'odrive' },
  { mask: 0x00000020, name: 'BRAKE_DEADTIME_VIOLATION', category: 'odrive' },
  { mask: 0x00000040, name: 'BRAKE_DUTY_CYCLE_NAN', category: 'odrive' },
  { mask: 0x00000080, name: 'INVALID_BRAKE_RESISTANCE', category: 'odrive' },
]

// --- ODrive Inbound WS messages (backend -> browser) ---

export interface ODriveTelemetryMsg {
  type: 'telemetry'
  t: number
  ch: Record<string, number>
}

export interface ODriveHeartbeatMsg {
  type: 'heartbeat'
  t: number
  axis_error: number
  axis_state: number
  motor_err_flag: boolean
  encoder_err_flag: boolean
  controller_err_flag: boolean
  comm_timeout: boolean
  quick_stop_active: boolean
  holding: boolean
  cmd_watchdog_expired: boolean
  mit_frame_stale: boolean
  running: boolean
  traj_done: boolean
  controller_flags: number
}

export interface ODriveStatusMsg {
  type: 'status'
  t: number
  connected: boolean
  interface: string
  channel: string
  node_id: number
  frames_rx: number
  frames_tx: number
  bus_errors: number
  poll_hz: number
}

export interface ODriveExtRespMsg {
  type: 'ext_resp'
  t?: number
  sub_cmd: number
  item: number
  status: number
  value: number
}

export interface ODriveOverspeedSnapshotMsg {
  type: 'overspeed_snapshot'
  snapshot: Record<string, number | null>
}

export interface ODriveControlConfigMsg {
  type: 'control_config'
  config: Record<string, number | null>
}

export interface ODrivePongMsg { type: 'pong' }

export type ODriveInboundMsg =
  | ODriveTelemetryMsg | ODriveHeartbeatMsg | ODriveStatusMsg
  | ODriveExtRespMsg | LogMsg | ErrorMsg | RecStateMsg
  | ODriveOverspeedSnapshotMsg | ODriveControlConfigMsg | ODrivePongMsg

// --- ODrive Outbound WS messages (browser -> backend) ---

export interface SetStateOut { type: 'set_state'; state: number }
export interface SetModeOut {
  type: 'set_mode'; control_mode: number; input_mode: number
}
export interface SetPosOut {
  type: 'set_pos'; pos: number; vel_ff?: number; torque_ff?: number
}
export interface SetVelOut {
  type: 'set_vel'; vel: number; torque_ff?: number
}
export interface SetTorqueOut { type: 'set_torque'; torque: number }
export interface MitOut {
  type: 'mit'
  p_des: number; v_des: number
  kp: number; kd: number; t_ff: number
}
export interface SetGainOut {
  type: 'set_gain'; name: string; value: number; integrator?: number; gain?: number
}
export interface SetLimitsOut {
  type: 'set_limits'; vel_limit: number; current_limit: number
}
export interface ClearErrorsOut { type: 'clear_errors' }
export interface EstopOut { type: 'estop' }
export interface RebootOut { type: 'reboot' }
export interface ExtCmdOut {
  type: 'ext_cmd'; sub_cmd: number; item: number
  ext_type?: number; value?: number; timeout?: number
}
export interface AnticoggingStartOut { type: 'anticogging_start' }
export interface AnticoggingStatusOut { type: 'anticogging_status' }
export interface AnticoggingConfigOut {
  type: 'anticogging_config'
  enabled?: boolean; pre_calibrated?: boolean
  pos_threshold?: number; vel_threshold?: number; reset?: boolean
}
export interface SetPollHzOut { type: 'set_poll_hz'; hz: number }
export interface GetOverspeedSnapshotOut { type: 'get_overspeed_snapshot' }
export interface GetControlConfigOut { type: 'get_control_config' }
export interface SetControlConfigOut {
  type: 'set_control_config'
  item: number
  value: number
  is_float?: boolean
}
export interface SetServoModeOut { type: 'set_servo_mode'; mode: number }
export interface PingOut { type: 'ping' }

export type ODriveOutboundMsg =
  | SetStateOut | SetModeOut | SetPosOut | SetVelOut | SetTorqueOut
  | MitOut | SetGainOut | SetLimitsOut | ClearErrorsOut | EstopOut
  | RebootOut | ExtCmdOut | AnticoggingStartOut | AnticoggingStatusOut
  | AnticoggingConfigOut | SetPollHzOut | RecStartOut | RecStopOut
  | GetOverspeedSnapshotOut | GetControlConfigOut | SetControlConfigOut
  | SetServoModeOut | PingOut
