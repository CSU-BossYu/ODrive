// Generated from docs/scope_channel_schema.json; do not edit by hand.
export type ScopeWireDataType = 'u8' | 'u32' | 'f32'
export interface ScopeChannel {
  id: number; name: string; wireType: ScopeWireDataType; unit: string;
  displayMin: number; displayMax: number; maxSampleRateHz: number;
  thresholdAllowed: boolean; source: string;
}
export const MAX_CAPTURE_CHANNELS = 8 as const
export const MAX_PRE_SAMPLES = 64 as const
export const MAX_POST_SAMPLES = 128 as const
export const MAX_BATCH_SAMPLES = 6 as const

export const SCOPE_CHANNELS: readonly ScopeChannel[] = [
  { id: 1, name: 'control_sequence', wireType: 'u32', unit: 'count', displayMin: 0, displayMax: 100000, maxSampleRateHz: 10000, thresholdAllowed: false, source: 'control_sequence' },
  { id: 2, name: 'timestamp_cycles', wireType: 'u32', unit: 'cycles', displayMin: 0, displayMax: 4294967295, maxSampleRateHz: 10000, thresholdAllowed: false, source: 'timestamp_cycles' },
  { id: 3, name: 'state_epoch', wireType: 'u32', unit: 'epoch', displayMin: 0, displayMax: 4294967295, maxSampleRateHz: 1000, thresholdAllowed: false, source: 'state_epoch' },
  { id: 4, name: 'SafetyState', wireType: 'u8', unit: 'enum', displayMin: 0, displayMax: 7, maxSampleRateHz: 1000, thresholdAllowed: true, source: 'safety_state' },
  { id: 5, name: 'Operation', wireType: 'u8', unit: 'enum', displayMin: 0, displayMax: 3, maxSampleRateHz: 1000, thresholdAllowed: true, source: 'operation' },
  { id: 6, name: 'phase', wireType: 'f32', unit: 'rad', displayMin: -3.1415927, displayMax: 3.1415927, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'phase' },
  { id: 7, name: 'phase_velocity', wireType: 'f32', unit: 'rad/s', displayMin: -10000, displayMax: 10000, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'phase_velocity' },
  { id: 8, name: 'position', wireType: 'f32', unit: 'rev', displayMin: -1000, displayMax: 1000, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'position' },
  { id: 9, name: 'velocity', wireType: 'f32', unit: 'rev/s', displayMin: -1000, displayMax: 1000, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'velocity' },
  { id: 10, name: 'Id measured', wireType: 'f32', unit: 'A', displayMin: -50, displayMax: 50, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'id_measured' },
  { id: 11, name: 'Iq measured', wireType: 'f32', unit: 'A', displayMin: -50, displayMax: 50, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'iq_measured' },
  { id: 12, name: 'Id setpoint', wireType: 'f32', unit: 'A', displayMin: -50, displayMax: 50, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'id_setpoint' },
  { id: 13, name: 'Iq setpoint', wireType: 'f32', unit: 'A', displayMin: -50, displayMax: 50, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'iq_setpoint' },
  { id: 14, name: 'torque setpoint', wireType: 'f32', unit: 'Nm', displayMin: -50, displayMax: 50, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'torque_setpoint' },
  { id: 15, name: 'controller output', wireType: 'f32', unit: 'command', displayMin: -1, displayMax: 1, maxSampleRateHz: 10000, thresholdAllowed: true, source: 'controller_output' },
]
export const SCOPE_CHANNEL_BY_ID = Object.fromEntries(SCOPE_CHANNELS.map((item) => [item.id, item])) as Record<number, ScopeChannel>
