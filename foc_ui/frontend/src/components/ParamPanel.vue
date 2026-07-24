<script setup lang="ts">
// Parameter tuning panel for ODrive CAN mode.
// Gains, limits, poll rate, and config management.

import { ref, watch } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const oSocket = useOdriveSocket()
const EXT_FLOAT32 = 1
const EXT_UINT32 = 3
const SUB_GET_BASIC = 0x06
const SUB_SET_BASIC = 0x07
const SUB_GET_CONTROL = 0x0B
const SUB_SET_CONTROL = 0x0C

interface ParamDef {
  key: string; label: string; unit: string
  min: number; max: number; step: number
  scale?: number
  item?: number
  readSubCmd?: number
  setSubCmd?: number
  isFloat?: boolean
}

interface ControlParamDef extends ParamDef {
  item: number
  isFloat: boolean
  readonly?: boolean
}

const PARAMS: ParamDef[] = [
  { key: 'pos_gain',            label: '位置增益',       unit: '(rev/s)/rev', min: 0, max: 100, step: 0.1 },
  { key: 'vel_gain',            label: '速度增益',       unit: 'Nm/(rev/s)',  min: 0, max: 100, step: 0.01 },
  { key: 'vel_integrator_gain', label: '速度积分增益',   unit: 'Nm/(rev/s)/s',min: 0, max: 100, step: 0.01 },
  { key: 'vel_limit',           label: '速度上限',       unit: 'rpm',         min: 0, max: 1800, step: 10, scale: 60 },
  { key: 'current_limit',       label: '电流上限',       unit: 'A',           min: 0, max: 50,  step: 0.1 },
  { key: 'poll_hz',             label: '轮询频率',       unit: 'Hz',          min: 5, max: 100, step: 1 },
]

Object.assign(PARAMS.find((p) => p.key === 'pos_gain')!, {
  item: 0x30, readSubCmd: SUB_GET_BASIC, setSubCmd: SUB_SET_BASIC, isFloat: true,
})
Object.assign(PARAMS.find((p) => p.key === 'vel_gain')!, {
  item: 0x31, readSubCmd: SUB_GET_BASIC, setSubCmd: SUB_SET_BASIC, isFloat: true,
})
Object.assign(PARAMS.find((p) => p.key === 'vel_integrator_gain')!, {
  item: 0x32, readSubCmd: SUB_GET_BASIC, setSubCmd: SUB_SET_BASIC, isFloat: true,
})
Object.assign(PARAMS.find((p) => p.key === 'vel_limit')!, {
  min: 0.1, item: 0x60, readSubCmd: SUB_GET_CONTROL, setSubCmd: SUB_SET_CONTROL, isFloat: true,
})
Object.assign(PARAMS.find((p) => p.key === 'current_limit')!, {
  item: 0x14, readSubCmd: SUB_GET_BASIC, setSubCmd: SUB_SET_BASIC, isFloat: true,
})

const CONTROL_PARAMS: ControlParamDef[] = [
  { key: 'profile_vel_limit', label: '位置规划最高速度', unit: 'rpm', min: 0, max: 1800, step: 10, item: 0x55, isFloat: true, scale: 60 },
  { key: 'profile_accel_limit', label: '位置规划加速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x56, isFloat: true, scale: 60 },
  { key: 'profile_decel_limit', label: '位置规划减速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x57, isFloat: true, scale: 60 },
  { key: 'velocity_accel_limit', label: '速度模式加速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x50, isFloat: true, scale: 60 },
  { key: 'velocity_decel_limit', label: '速度模式减速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x51, isFloat: true, scale: 60 },
  { key: 'quick_stop_decel_limit', label: '快速停止减速度', unit: 'rpm/s', min: 0, max: 120000, step: 10, item: 0x52, isFloat: true, scale: 60 },
  { key: 'can_watchdog_timeout_ms', label: '命令超时', unit: 'ms', min: 0, max: 60000, step: 10, item: 0x53, isFloat: false },
  { key: 'heartbeat_timeout_ms', label: '心跳超时（0或≥250）', unit: 'ms', min: 0, max: 60000, step: 50, item: 0x5C, isFloat: false },
  { key: 'timeout_action', label: '超时动作', unit: '', min: 0, max: 4, step: 1, item: 0x54, isFloat: false },
  { key: 'servo_mode', label: '伺服模式', unit: '', min: 0, max: 3, step: 1, item: 0x5B, isFloat: false },
  { key: 'control_runtime_state', label: '运行标志', unit: 'bits', min: 0, max: 0xffffffff, step: 1, item: 0x58, isFloat: false, readonly: true },
  { key: 'last_timeout_reason', label: '上次超时原因', unit: '', min: 0, max: 0xffffffff, step: 1, item: 0x59, isFloat: false, readonly: true },
  { key: 'trajectory_done', label: '轨迹完成', unit: '', min: 0, max: 1, step: 1, item: 0x5A, isFloat: false, readonly: true },
  // Sguan STA selection (armed-guarded — disarm to IDLE before setting).
  { key: 'enable_sta', label: 'Sguan STA 使能', unit: '', min: 0, max: 1, step: 1, item: 0x6E, isFloat: false },
]

CONTROL_PARAMS.splice(13, 0,
  { key: 'vel_limit_tolerance', label: '速度保护容差', unit: 'x', min: 1, max: 10, step: 0.05, item: 0x61, isFloat: true },
  { key: 'enable_vel_limit', label: '速度限制使能', unit: '', min: 0, max: 1, step: 1, item: 0x62, isFloat: false },
  { key: 'enable_torque_mode_vel_limit', label: '力矩模式限速', unit: '', min: 0, max: 1, step: 1, item: 0x63, isFloat: false },
)

PARAMS.splice(1, 0, {
  key: 'pos_integrator_gain',
  label: '位置积分增益',
  unit: '(rev/s)/rev/s',
  min: 0,
  max: 100,
  step: 0.001,
  item: 0x5D,
  readSubCmd: SUB_GET_CONTROL,
  setSubCmd: SUB_SET_CONTROL,
  isFloat: true,
})

// Motor-model identity (baked in production_config.h, GET-only). Read-only
// display so the customer can verify the build (pole_pairs, torque_constant,
// encoder type, brake/dc-bus thresholds). Not settable from the UI.
interface MotorModelParamDef {
  key: string; label: string; unit: string; item: number
  format?: (v: number) => string
}
function motorTypeName(v: number): string {
  return ({ 0: 'HIGH_CURRENT', 1: 'GIMBAL', 2: 'ACIM' } as Record<number, string>)[v] ?? `(${v})`
}
function encoderModeName(v: number): string {
  if (v === 0x106) return 'MT6826S_VERNIER'
  if (v === 0x105) return 'MT6826S'
  return `0x${v.toString(16)}`
}
const MOTOR_MODEL_PARAMS: MotorModelParamDef[] = [
  { key: 'motor_type', label: '电机类型', unit: '', item: 0x10, format: motorTypeName },
  { key: 'pole_pairs', label: '极对数', unit: '', item: 0x11 },
  { key: 'torque_constant', label: '力矩常数', unit: 'Nm/A', item: 0x15 },
  { key: 'encoder_mode', label: '编码器模式', unit: '', item: 0x20, format: encoderModeName },
  { key: 'encoder_cpr', label: '编码器 CPR', unit: '', item: 0x21 },
  { key: 'brake_resistance', label: '制动电阻', unit: 'Ω', item: 0x42 },
  { key: 'dc_bus_undervoltage', label: '欠压保护', unit: 'V', item: 0x43 },
  { key: 'dc_bus_overvoltage', label: '过压保护', unit: 'V', item: 0x44 },
]

const values = ref<Record<string, number>>({
  pos_gain: 7, vel_gain: 10, vel_integrator_gain: 1.5,
  vel_limit: 60, current_limit: 3, poll_hz: 20,
})
values.value.pos_integrator_gain = 0

const controlValues = ref<Record<string, number>>({
  profile_vel_limit: 60,
  profile_accel_limit: 60,
  profile_decel_limit: 60,
  velocity_accel_limit: 60,
  velocity_decel_limit: 60,
  quick_stop_decel_limit: 120,
  can_watchdog_timeout_ms: 300,
  heartbeat_timeout_ms: 0,
  timeout_action: 2,
  servo_mode: 2,
  control_runtime_state: 0,
  last_timeout_reason: 0,
  trajectory_done: 0,
  vel_limit_tolerance: 1.2,
  enable_vel_limit: 1,
  enable_torque_mode_vel_limit: 1,
  enable_sta: 0,
})

const motorModelValues = ref<Record<string, number>>({})

const lastResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(PARAMS.map((p) => [p.key, null]))
)
const controlResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(CONTROL_PARAMS.map((p) => [p.key, null]))
)

watch(() => oSocket.controlConfig.value, (cfg) => {
  if (!cfg) return
  for (const p of PARAMS) {
    const v = cfg[p.key]
    if (typeof v === 'number' && Number.isFinite(v)) {
      values.value[p.key] = p.scale ? v * p.scale : v
      lastResult.value[p.key] = { ok: true, text: 'read' }
    }
  }
  for (const p of CONTROL_PARAMS) {
    const v = cfg[p.key]
    if (typeof v === 'number' && Number.isFinite(v)) {
      controlValues.value[p.key] = p.scale ? v * p.scale : v
      controlResult.value[p.key] = { ok: true, text: 'read' }
    }
  }
}, { deep: true })

function extTypeFor(isFloat?: boolean) {
  return isFloat ? EXT_FLOAT32 : EXT_UINT32
}

function statusText(resp: { status: number; error?: string }) {
  if (resp.status === 0) return 'ok'
  if (resp.status === 5) return 'BUSY_ARMED'
  if (resp.status === 4) return 'INVALID_VALUE'
  if (resp.status === 3) return 'INVALID_TYPE'
  if (resp.status === 2) return 'READONLY'
  if (resp.status === 1) return 'UNKNOWN'
  return resp.error || `status ${resp.status}`
}

function applyReadValue(target: Record<string, number>, p: ParamDef, value: unknown) {
  if (typeof value !== 'number' || !Number.isFinite(value)) return false
  target[p.key] = p.scale ? value * p.scale : value
  return true
}

watch(() => oSocket.extSeq.value, () => {
  const list = oSocket.extResponses.value
  const resp = list[list.length - 1]
  if (!resp) return

  if (resp.sub_cmd === SUB_GET_BASIC) {
    const modelParam = MOTOR_MODEL_PARAMS.find((item) => item.item === resp.item)
    if (modelParam && resp.status === 0 && typeof resp.value === 'number' && Number.isFinite(resp.value)) {
      motorModelValues.value[modelParam.key] = resp.value
    }

    const p = PARAMS.find((item) => item.readSubCmd === SUB_GET_BASIC && item.item === resp.item)
    if (p) {
      const ok = resp.status === 0 && applyReadValue(values.value, p, resp.value)
      lastResult.value[p.key] = { ok, text: ok ? 'read' : statusText(resp) }
    }
    return
  }

  if (resp.sub_cmd === SUB_SET_BASIC) {
    const p = PARAMS.find((item) => item.setSubCmd === SUB_SET_BASIC && item.item === resp.item)
    if (p) lastResult.value[p.key] = { ok: resp.status === 0, text: statusText(resp) }
    return
  }

  if (resp.sub_cmd === SUB_GET_CONTROL) {
    const p = PARAMS.find((item) => item.readSubCmd === SUB_GET_CONTROL && item.item === resp.item)
    if (p) {
      const ok = resp.status === 0 && applyReadValue(values.value, p, resp.value)
      lastResult.value[p.key] = { ok, text: ok ? 'read' : statusText(resp) }
    }
    const cp = CONTROL_PARAMS.find((item) => item.item === resp.item)
    if (cp) {
      const ok = resp.status === 0 && applyReadValue(controlValues.value, cp, resp.value)
      controlResult.value[cp.key] = { ok, text: ok ? 'read' : statusText(resp) }
    }
    return
  }

  if (resp.sub_cmd === SUB_SET_CONTROL) {
    const p = PARAMS.find((item) => item.setSubCmd === SUB_SET_CONTROL && item.item === resp.item)
    if (p) lastResult.value[p.key] = { ok: resp.status === 0, text: statusText(resp) }
    const cp = CONTROL_PARAMS.find((item) => item.item === resp.item)
    if (cp) controlResult.value[cp.key] = { ok: resp.status === 0, text: statusText(resp) }
  }
})

function apply(p: ParamDef) {
  const v = values.value[p.key]
  // Guard NaN/empty-string before the range check: v-model.number on a
  // cleared input yields '' (looseToNumber), and '' < min coerces to 0 and
  // passes. Sending '' would crash the backend's float().
  if (typeof v !== 'number' || !Number.isFinite(v) || v < p.min || v > p.max) {
    lastResult.value[p.key] = { ok: false, text: `Out of range [${p.min}, ${p.max}]` }
    return
  }
  if (p.key === 'poll_hz') {
    oSocket.setPollHz(v)
    lastResult.value[p.key] = { ok: true, text: 'ok' }
    return
  } else if (p.item !== undefined && p.setSubCmd !== undefined) {
    oSocket.extCmd(p.setSubCmd, p.item, extTypeFor(p.isFloat), p.scale ? v / p.scale : v)
  } else if (p.key === 'vel_gain') {
    // Backend set_vel_gains sends (gain, integrator) together; pass the
    // companion value so editing vel_gain alone doesn't zero the integrator.
    // Guard against a cleared input ('') which the backend's float() can't
    // parse (would 500 the set_gain handler).
    const ig = (typeof values.value.vel_integrator_gain === 'number'
      && Number.isFinite(values.value.vel_integrator_gain))
      ? values.value.vel_integrator_gain : 0
    oSocket.setGain('vel_gain', v, { integrator: ig })
  } else if (p.key === 'vel_integrator_gain') {
    const vg = (typeof values.value.vel_gain === 'number'
      && Number.isFinite(values.value.vel_gain))
      ? values.value.vel_gain : 0
    oSocket.setGain('vel_integrator_gain', v, { gain: vg })
  } else {
    oSocket.setGain(p.key, v)
  }
  lastResult.value[p.key] = { ok: true, text: 'sent' }
}

function readConfig() {
  PARAMS.forEach((p) => {
    if (p.item !== undefined && p.readSubCmd !== undefined) {
      oSocket.extCmd(p.readSubCmd, p.item)
    }
  })
}

function readControlConfig() {
  oSocket.getControlConfig()
}

function readMotorModel() {
  MOTOR_MODEL_PARAMS.forEach((p) => oSocket.extCmd(0x06, p.item))
}

function applyControl(p: ControlParamDef) {
  if (p.readonly) {
    readControlConfig()
    return
  }
  const v = controlValues.value[p.key]
  if (typeof v !== 'number' || !Number.isFinite(v) || v < p.min || v > p.max) {
    controlResult.value[p.key] = { ok: false, text: `Out of range [${p.min}, ${p.max}]` }
    return
  }
  if (p.key === 'servo_mode') {
    oSocket.setServoMode(v)
  } else {
    oSocket.setControlConfig(p.item, p.scale ? v / p.scale : v, p.isFloat)
  }
  controlResult.value[p.key] = { ok: true, text: 'sent' }
}

function saveConfig() {
  if (confirm('Save configuration to Flash? The axis must be IDLE.')) {
    oSocket.extCmd(0x03, 0x00, 3, 0, 3.0)  // SAVE_CONFIGURATION
  }
}

function deviceInfo() {
  oSocket.extCmd(0x05, 0x01)  // GET_DEVICE_INFO
}
</script>

<template>
  <div class="panel param-panel">
    <div class="row">
      <h3 class="panel-title grow">参数</h3>
      <button @click="readConfig" title="Read config from device">Read</button>
      <button @click="saveConfig" title="Save config to flash" class="warn">Save</button>
      <button @click="deviceInfo" title="Get device info">Info</button>
    </div>
    <div class="param-list">
      <div class="section-heading">
        <span>电机型号 (只读)</span>
        <button @click="readMotorModel">读取</button>
      </div>
      <div v-for="p in MOTOR_MODEL_PARAMS" :key="p.key" class="param-row motor-model-row">
        <label>{{ p.label }}</label>
        <div class="row">
          <span class="motor-model-value">{{ motorModelValues[p.key] !== undefined ? (p.format ? p.format(motorModelValues[p.key]) : motorModelValues[p.key]) : '--' }}</span>
          <span class="unit">{{ p.unit }}</span>
        </div>
      </div>

      <div v-for="p in PARAMS" :key="p.key" class="param-row">
        <label :title="`${p.min}..${p.max}`">{{ p.label }}</label>
        <div class="row">
          <input
            type="number" :step="p.step" :min="p.min" :max="p.max"
            v-model.number="values[p.key]"
            @keyup.enter="apply(p)"
          />
          <span class="unit">{{ p.unit }}</span>
          <button @click="apply(p)">set</button>
        </div>
        <div v-if="lastResult[p.key]" class="result"
          :class="{ ok: lastResult[p.key]?.ok, err: !lastResult[p.key]?.ok }">
          {{ lastResult[p.key]?.text }}
        </div>
      </div>

      <div class="section-heading">
        <span>控制配置</span>
        <button @click="readControlConfig">读取</button>
      </div>
      <div v-for="p in CONTROL_PARAMS" :key="p.key" class="param-row">
        <label :title="`${p.min}..${p.max}`">{{ p.label }}</label>
        <div class="row">
          <input
            type="number" :step="p.step" :min="p.min" :max="p.max"
            v-model.number="controlValues[p.key]"
            :disabled="p.readonly"
            @keyup.enter="applyControl(p)"
          />
          <span class="unit">{{ p.unit }}</span>
          <button @click="applyControl(p)">{{ p.readonly ? '读取' : '设置' }}</button>
        </div>
        <div v-if="controlResult[p.key]" class="result"
          :class="{ ok: controlResult[p.key]?.ok, err: !controlResult[p.key]?.ok }">
          {{ controlResult[p.key]?.text }}
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.param-panel { display: flex; flex-direction: column; gap: 6px; overflow: hidden; }
.param-list { overflow-y: auto; display: flex; flex-direction: column; gap: 6px; }
.param-row {
  display: flex; flex-direction: column; gap: 2px;
  padding-bottom: 4px; border-bottom: 1px solid var(--border);
}
.motor-model-value {
  font-family: var(--mono);
  font-size: 12px;
  color: var(--fg);
  flex: 1;
  padding: 2px 4px;
  background: var(--bg-2);
  border-radius: 3px;
  min-height: 16px;
}
.unit { font-size: 10px; color: var(--fg-dim); min-width: 36px; text-align: right; }
.section-heading {
  display: flex; align-items: center; justify-content: space-between;
  padding-top: 4px;
  font-size: 10px; color: var(--fg-dim); text-transform: uppercase;
}
.result { font-size: 10px; font-family: var(--mono); }
.result.ok { color: var(--ok); }
.result.err { color: var(--err); }
</style>
