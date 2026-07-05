<script setup lang="ts">
// Parameter tuning panel for ODrive CAN mode.
// Gains, limits, poll rate, and config management.

import { ref, watch } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const oSocket = useOdriveSocket()

interface ParamDef {
  key: string; label: string; unit: string
  min: number; max: number; step: number
  scale?: number
}

interface ControlParamDef extends ParamDef {
  item: number
  isFloat: boolean
  readonly?: boolean
}

const PARAMS: ParamDef[] = [
  { key: 'pos_gain',            label: '位置增益',       unit: '(rev/s)/rev', min: 0, max: 100, step: 0.1 },
  { key: 'vel_gain',            label: '速度增益',       unit: 'Nm/(rev/s)',  min: 0, max: 10,  step: 0.001 },
  { key: 'vel_integrator_gain', label: '速度积分增益',   unit: 'Nm/(rev/s)/s',min: 0, max: 100, step: 0.01 },
  { key: 'vel_limit',           label: '速度上限',       unit: 'rpm',         min: 0, max: 1800, step: 10, scale: 60 },
  { key: 'current_limit',       label: '电流上限',       unit: 'A',           min: 0, max: 50,  step: 0.1 },
  { key: 'poll_hz',             label: '轮询频率',       unit: 'Hz',          min: 5, max: 100, step: 1 },
]

const CONTROL_PARAMS: ControlParamDef[] = [
  { key: 'profile_vel_limit', label: '位置规划最高速度', unit: 'rpm', min: 0, max: 1800, step: 10, item: 0x55, isFloat: true, scale: 60 },
  { key: 'profile_accel_limit', label: '位置规划加速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x56, isFloat: true, scale: 60 },
  { key: 'profile_decel_limit', label: '位置规划减速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x57, isFloat: true, scale: 60 },
  { key: 'velocity_accel_limit', label: '速度模式加速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x50, isFloat: true, scale: 60 },
  { key: 'velocity_decel_limit', label: '速度模式减速度', unit: 'rpm/s', min: 0, max: 60000, step: 10, item: 0x51, isFloat: true, scale: 60 },
  { key: 'quick_stop_decel_limit', label: '快速停止减速度', unit: 'rpm/s', min: 0, max: 120000, step: 10, item: 0x52, isFloat: true, scale: 60 },
  { key: 'can_watchdog_timeout_ms', label: '命令超时', unit: 'ms', min: 0, max: 60000, step: 10, item: 0x53, isFloat: false },
  { key: 'heartbeat_timeout_ms', label: '心跳超时', unit: 'ms', min: 0, max: 60000, step: 10, item: 0x5C, isFloat: false },
  { key: 'timeout_action', label: '超时动作', unit: '', min: 0, max: 4, step: 1, item: 0x54, isFloat: false },
  { key: 'servo_mode', label: '伺服模式', unit: '', min: 0, max: 3, step: 1, item: 0x5B, isFloat: false },
  { key: 'control_runtime_state', label: '运行标志', unit: 'bits', min: 0, max: 0xffffffff, step: 1, item: 0x58, isFloat: false, readonly: true },
  { key: 'last_timeout_reason', label: '上次超时原因', unit: '', min: 0, max: 0xffffffff, step: 1, item: 0x59, isFloat: false, readonly: true },
  { key: 'trajectory_done', label: '轨迹完成', unit: '', min: 0, max: 1, step: 1, item: 0x5A, isFloat: false, readonly: true },
]

const ADRC_PARAMS: ControlParamDef[] = [
  { key: 'adrc_enabled', label: 'ADRC启用', unit: '', min: 0, max: 1, step: 1, item: 0x60, isFloat: false },
  { key: 'adrc_b0', label: 'ADRC b0', unit: '(rev/s²)/Nm', min: 0.001, max: 100000, step: 0.001, item: 0x61, isFloat: true },
  { key: 'adrc_bandwidth', label: 'ADRC观测带宽', unit: '1/s', min: 1, max: 1000, step: 1, item: 0x62, isFloat: true },
  { key: 'adrc_pos_gain', label: 'ADRC位置增益', unit: '1/s²', min: 0, max: 100000, step: 1, item: 0x63, isFloat: true },
  { key: 'adrc_vel_gain', label: 'ADRC速度增益', unit: '1/s', min: 0, max: 10000, step: 0.1, item: 0x64, isFloat: true },
  { key: 'adrc_disturbance_limit', label: 'ADRC扰动限幅', unit: 'rev/s²', min: 0.001, max: 100000, step: 1, item: 0x65, isFloat: true },
  { key: 'adrc_z1', label: 'ADRC z1位置', unit: 'rev', min: -1e9, max: 1e9, step: 0.0001, item: 0x66, isFloat: true, readonly: true },
  { key: 'adrc_z2', label: 'ADRC z2速度', unit: 'rev/s', min: -1e9, max: 1e9, step: 0.0001, item: 0x67, isFloat: true, readonly: true },
  { key: 'adrc_z3', label: 'ADRC z3扰动', unit: 'rev/s²', min: -1e9, max: 1e9, step: 0.0001, item: 0x68, isFloat: true, readonly: true },
]

const VERNIER_PARAMS: ControlParamDef[] = [
  { key: 'vernier_aux_correction_bandwidth', label: 'Vernier aux pos BW', unit: '1/s', min: 0, max: 100, step: 0.1, item: 0x36, isFloat: true },
  { key: 'vernier_aux_velocity_bandwidth', label: 'Vernier aux vel BW', unit: '1/s', min: 0, max: 500, step: 0.1, item: 0x37, isFloat: true },
  { key: 'vernier_aux_max_correction', label: 'Vernier max corr', unit: 'rev/sample', min: 0, max: 0.1, step: 0.0001, item: 0x38, isFloat: true },
]

PARAMS.splice(1, 0, {
  key: 'pos_integrator_gain',
  label: '位置积分增益',
  unit: '(rev/s)/rev/s',
  min: 0,
  max: 100,
  step: 0.001,
})

const values = ref<Record<string, number>>({
  pos_gain: 20, vel_gain: 0.5, vel_integrator_gain: 10,
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
  can_watchdog_timeout_ms: 500,
  heartbeat_timeout_ms: 0,
  timeout_action: 2,
  servo_mode: 2,
  adrc_enabled: 1,
  adrc_b0: 1,
  adrc_bandwidth: 30,
  adrc_pos_gain: 100,
  adrc_vel_gain: 20,
  adrc_disturbance_limit: 1000,
  adrc_z1: 0,
  adrc_z2: 0,
  adrc_z3: 0,
  control_runtime_state: 0,
  last_timeout_reason: 0,
  trajectory_done: 0,
})

const vernierValues = ref<Record<string, number>>({
  vernier_aux_correction_bandwidth: 0,
  vernier_aux_velocity_bandwidth: 0,
  vernier_aux_max_correction: 0.002,
})

const lastResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(PARAMS.map((p) => [p.key, null]))
)
const controlResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries([...ADRC_PARAMS, ...CONTROL_PARAMS].map((p) => [p.key, null]))
)
const vernierResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(VERNIER_PARAMS.map((p) => [p.key, null]))
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
  for (const p of [...ADRC_PARAMS, ...CONTROL_PARAMS]) {
    const v = cfg[p.key]
    if (typeof v === 'number' && Number.isFinite(v)) {
      controlValues.value[p.key] = p.scale ? v * p.scale : v
      controlResult.value[p.key] = { ok: true, text: 'read' }
    }
  }
}, { deep: true })

watch(() => oSocket.extSeq.value, () => {
  const list = oSocket.extResponses.value
  const resp = list[list.length - 1]
  if (!resp || (resp.sub_cmd !== 0x06 && resp.sub_cmd !== 0x07)) return
  const p = VERNIER_PARAMS.find((item) => item.item === resp.item)
  if (!p) return
  const ok = resp.status === 0
  if (ok && resp.sub_cmd === 0x06 && typeof resp.value === 'number' && Number.isFinite(resp.value)) {
    vernierValues.value[p.key] = resp.value
  }
  vernierResult.value[p.key] = { ok, text: ok ? (resp.sub_cmd === 0x06 ? 'read' : 'sent') : `status ${resp.status}` }
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
  if (p.key === 'vel_limit' || p.key === 'current_limit') {
    const vl = values.value.vel_limit
    const cl = values.value.current_limit
    if (typeof vl !== 'number' || !Number.isFinite(vl) ||
        typeof cl !== 'number' || !Number.isFinite(cl)) {
      lastResult.value[p.key] = { ok: false, text: 'vel_limit and current_limit must both be valid' }
      return
    }
    const velParam = PARAMS.find((item) => item.key === 'vel_limit')
    oSocket.setLimits(velParam?.scale ? vl / velParam.scale : vl, cl)
  } else if (p.key === 'poll_hz') {
    oSocket.setPollHz(v)
  } else if (p.key === 'vel_gain') {
    // Backend set_vel_gains sends (gain, integrator) together; pass the
    // companion value so editing vel_gain alone doesn't zero the integrator.
    oSocket.setGain('vel_gain', v, { integrator: values.value.vel_integrator_gain })
  } else if (p.key === 'vel_integrator_gain') {
    oSocket.setGain('vel_integrator_gain', v, { gain: values.value.vel_gain })
  } else {
    oSocket.setGain(p.key, v)
  }
  lastResult.value[p.key] = { ok: true, text: 'sent' }
}

function readConfig() {
  oSocket.extCmd(0x06, 0x01)  // GET_BASIC_CONFIG
}

function readControlConfig() {
  oSocket.getControlConfig()
}

function readVernierConfig() {
  VERNIER_PARAMS.forEach((p) => oSocket.extCmd(0x06, p.item))
}

function applyVernier(p: ControlParamDef) {
  const v = vernierValues.value[p.key]
  if (typeof v !== 'number' || !Number.isFinite(v) || v < p.min || v > p.max) {
    vernierResult.value[p.key] = { ok: false, text: `Out of range [${p.min}, ${p.max}]` }
    return
  }
  oSocket.extCmd(0x07, p.item, p.isFloat ? 1 : 3, v)
  vernierResult.value[p.key] = { ok: true, text: 'sent' }
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
  if (confirm('Save configuration? ODrive will reset.')) {
    oSocket.extCmd(0x03, 0x00)  // SAVE_CONFIGURATION
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
        <span>Vernier Fusion</span>
        <button @click="readVernierConfig">Read</button>
      </div>
      <div v-for="p in VERNIER_PARAMS" :key="p.key" class="param-row">
        <label :title="`${p.min}..${p.max}`">{{ p.label }}</label>
        <div class="row">
          <input
            type="number" :step="p.step" :min="p.min" :max="p.max"
            v-model.number="vernierValues[p.key]"
            @keyup.enter="applyVernier(p)"
          />
          <span class="unit">{{ p.unit }}</span>
          <button @click="applyVernier(p)">set</button>
        </div>
        <div v-if="vernierResult[p.key]" class="result"
          :class="{ ok: vernierResult[p.key]?.ok, err: !vernierResult[p.key]?.ok }">
          {{ vernierResult[p.key]?.text }}
        </div>
      </div>

      <div class="section-heading">
        <span>ADRC</span>
        <button @click="readControlConfig">读取</button>
      </div>
      <div v-for="p in ADRC_PARAMS" :key="p.key" class="param-row adrc-row">
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
