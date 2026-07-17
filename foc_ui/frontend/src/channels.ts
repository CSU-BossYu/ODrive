// Static channel and error tables.
//
// These are duplicated from the backend so the UI can render immediately on
// page load without waiting for /api/channels. The backend is still the source
// of truth.

import type { ODriveChannelDef, ODriveErrorBitDef } from './types'
import {
  AXIS_ERROR_BITS, MOTOR_ERROR_BITS, ENCODER_ERROR_BITS, CONTROLLER_ERROR_BITS,
  ODRIVE_ERROR_BITS,
} from './types'

const SECONDARY_COLORS = [
  '#ec4899', '#84cc16', '#0ea5e9', '#f97316',
  '#a855f7', '#14b8a6', '#eab308', '#64748b',
  '#ef4444', '#3b82f6',
]

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
