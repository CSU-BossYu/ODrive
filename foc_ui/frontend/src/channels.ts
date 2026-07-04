// Static channel and error tables.
//
// These are duplicated from the backend so the UI can render immediately on
// page load without waiting for /api/channels. The backend is still the source
// of truth.

import type { ChannelDef, ErrorBitDef, ODriveChannelDef, ODriveErrorBitDef } from './types'
import {
  AXIS_ERROR_BITS, MOTOR_ERROR_BITS, ENCODER_ERROR_BITS, CONTROLLER_ERROR_BITS,
  ODRIVE_ERROR_BITS,
} from './types'

// Order MUST match firmware motor_stream.c data[0]..data[15].
export const CHANNELS: ChannelDef[] = [
  { index: 0,  key: 'pos',         label: 'Position',       unit: 'rad',   scale: 1000, group: 'pos' },
  { index: 1,  key: 'vel_ref',     label: 'Velocity ref',   unit: 'rad/s', scale: 100,  group: 'vel' },
  { index: 2,  key: 'vel_fb',      label: 'Velocity fb',    unit: 'rad/s', scale: 100,  group: 'vel' },
  { index: 3,  key: 'iq_ref',      label: 'Iq ref',         unit: 'A',     scale: 1000, group: 'cur' },
  { index: 4,  key: 'iq_meas',     label: 'Iq meas',        unit: 'A',     scale: 1000, group: 'cur' },
  { index: 5,  key: 'id_meas',     label: 'Id meas',        unit: 'A',     scale: 1000, group: 'cur' },
  { index: 6,  key: 'vq',          label: 'Vq',             unit: 'V',     scale: 100,  group: 'vol' },
  { index: 7,  key: 'vd',          label: 'Vd',             unit: 'V',     scale: 100,  group: 'vol' },
  { index: 8,  key: 'vbus',        label: 'Vbus',           unit: 'V',     scale: 100,  group: 'vol' },
  { index: 9,  key: 'phase',       label: 'Phase',          unit: 'rad',   scale: 1000, group: 'pos' },
  { index: 10, key: 'vel_err',     label: 'Vel error',      unit: 'rad/s', scale: 100,  group: 'vel' },
  { index: 11, key: 'integrator',  label: 'Vel integrator', unit: 'A',     scale: 1000, group: 'cur' },
  { index: 12, key: 'vq_ff',       label: 'Vq feedforward', unit: 'V',     scale: 100,  group: 'vol' },
  { index: 13, key: 'vd_ff',       label: 'Vd feedforward', unit: 'V',     scale: 100,  group: 'vol' },
  { index: 14, key: 'errors',      label: 'Error flags',    unit: 'bits',  scale: 1,    group: 'state' },
  { index: 15, key: 'state',       label: 'State flags',    unit: 'bits',  scale: 1,    group: 'state' },
]

export const CHANNEL_BY_KEY: Record<string, ChannelDef> = Object.fromEntries(
  CHANNELS.map((c) => [c.key, c]),
)

export const DEFAULT_VISIBLE: string[] = ['vel_ref', 'vel_fb', 'iq_ref', 'iq_meas', 'id_meas', 'vbus']

const PRIMARY_COLORS: Record<string, string> = {
  vel_ref: '#2563eb',
  vel_fb: '#06b6d4',
  iq_ref: '#dc2626',
  iq_meas: '#f59e0b',
  id_meas: '#7c3aed',
  vbus: '#16a34a',
}
const SECONDARY_COLORS = [
  '#ec4899', '#84cc16', '#0ea5e9', '#f97316',
  '#a855f7', '#14b8a6', '#eab308', '#64748b',
  '#ef4444', '#3b82f6',
]

export function channelColor(key: string): string {
  if (key in PRIMARY_COLORS) return PRIMARY_COLORS[key]
  const others = CHANNELS.filter((c) => !(c.key in PRIMARY_COLORS))
  const idx = others.findIndex((c) => c.key === key)
  return SECONDARY_COLORS[(idx < 0 ? 0 : idx) % SECONDARY_COLORS.length]
}

export const ERROR_BITS: ErrorBitDef[] = [
  { bit: 0,  name: 'DRV_FAULT',              fatal: true },
  { bit: 1,  name: 'CURRENT_SENSE',          fatal: true },
  { bit: 2,  name: 'OVERCURRENT',            fatal: false },
  { bit: 3,  name: 'DC_BUS_UNDERVOLTAGE',    fatal: false },
  { bit: 4,  name: 'ENCODER',                fatal: false },
  { bit: 5,  name: 'ENCODER_SPI',            fatal: false },
  { bit: 6,  name: 'ENCODER_NO_RESPONSE',    fatal: false },
  { bit: 7,  name: 'PHASE_RESISTANCE',       fatal: true },
  { bit: 8,  name: 'PHASE_INDUCTANCE',       fatal: true },
  { bit: 9,  name: 'CONTROL_DEADLINE',       fatal: false },
  { bit: 10, name: 'SPINOUT',                fatal: false },
  { bit: 11, name: 'INVALID_STATE',          fatal: false },
  { bit: 12, name: 'PWM_START',              fatal: false },
  { bit: 13, name: 'ADC_START',              fatal: false },
  { bit: 14, name: 'TIM_FAULT',              fatal: false },
  { bit: 15, name: 'ENCODER_NOT_READY',      fatal: false },
  { bit: 16, name: 'ENCODER_OFFSET_INVALID', fatal: false },
  { bit: 17, name: 'ENCODER_DIRECTION_INVALID', fatal: false },
  { bit: 18, name: 'ENCODER_CPR_MISMATCH',   fatal: false },
  { bit: 19, name: 'ENCODER_SCAN_TIMEOUT',   fatal: false },
  { bit: 20, name: 'CONFIG_INVALID',         fatal: true },
  { bit: 23, name: 'CALIBRATION_MISMATCH',   fatal: true },
  { bit: 24, name: 'OVERSPEED',              fatal: false },
  { bit: 25, name: 'DC_BUS_OVERVOLTAGE',     fatal: false },
  { bit: 26, name: 'ENCODER_TIMEOUT',        fatal: false },
  { bit: 27, name: 'STALL',                  fatal: false },
]

export function decodeErrorBits(word: number): ErrorBitDef[] {
  return ERROR_BITS.filter((e) => (word >>> e.bit) & 1)
}

// ODrive CAN waveform channels. Keep this list limited to useful curves:
// status, faults, and bus voltage live in the status cards instead.
export const ODRIVE_CHANNELS: ODriveChannelDef[] = [
  { index: 0, key: 'pos',     label: '位置',    unit: 'deg', group: 'pos' },
  { index: 1, key: 'vel',     label: '速度',    unit: 'rpm', group: 'vel' },
  { index: 2, key: 'iq_sp',   label: 'Iq 目标', unit: 'A',   group: 'cur' },
  { index: 3, key: 'iq_meas', label: 'Iq 实测', unit: 'A',   group: 'cur' },
]

export const ODRIVE_CHANNEL_BY_KEY: Record<string, ODriveChannelDef> =
  Object.fromEntries(ODRIVE_CHANNELS.map((c) => [c.key, c]))

export const ODRIVE_DEFAULT_VISIBLE: string[] = ['pos', 'vel', 'iq_sp', 'iq_meas']

const ODRIVE_PRIMARY_COLORS: Record<string, string> = {
  pos: '#3b82f6',
  vel: '#06b6d4',
  iq_sp: '#ef4444',
  iq_meas: '#f59e0b',
}

export function odriveChannelColor(key: string): string {
  if (key in ODRIVE_PRIMARY_COLORS) return ODRIVE_PRIMARY_COLORS[key]
  const others = ODRIVE_CHANNELS.filter((c) => !(c.key in ODRIVE_PRIMARY_COLORS))
  const idx = others.findIndex((c) => c.key === key)
  return SECONDARY_COLORS[(idx < 0 ? 0 : idx) % SECONDARY_COLORS.length]
}

export function decodeOdriveErrors(
  word: number, table: ODriveErrorBitDef[],
): ODriveErrorBitDef[] {
  let big: bigint
  try { big = BigInt(word) } catch { return [] }
  return table.filter((e) => (big & BigInt(e.mask)) !== 0n)
}

export function decodeAxisErrors(word: number): ODriveErrorBitDef[] {
  return decodeOdriveErrors(word, AXIS_ERROR_BITS)
}

export function decodeMotorErrors(word: number): ODriveErrorBitDef[] {
  return decodeOdriveErrors(word, MOTOR_ERROR_BITS)
}

export function decodeEncoderErrors(word: number): ODriveErrorBitDef[] {
  return decodeOdriveErrors(word, ENCODER_ERROR_BITS)
}

export function decodeControllerErrors(word: number): ODriveErrorBitDef[] {
  return decodeOdriveErrors(word, CONTROLLER_ERROR_BITS)
}

export function decodeOdriveSystemErrors(word: number): ODriveErrorBitDef[] {
  return decodeOdriveErrors(word, ODRIVE_ERROR_BITS)
}
