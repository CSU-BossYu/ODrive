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

const values = ref<Record<string, number>>({
  pos_gain: 20, vel_gain: 0.5, vel_integrator_gain: 10,
  vel_limit: 60, current_limit: 3, poll_hz: 20,
})

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
  control_runtime_state: 0,
  last_timeout_reason: 0,
  trajectory_done: 0,
})

const lastResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(PARAMS.map((p) => [p.key, null]))
)
const controlResult = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(CONTROL_PARAMS.map((p) => [p.key, null]))
)

watch(() => oSocket.controlConfig.value, (cfg) => {
  if (!cfg) return
  for (const p of CONTROL_PARAMS) {
    const v = cfg[p.key]
    if (typeof v === 'number' && Number.isFinite(v)) {
      controlValues.value[p.key] = p.scale ? v * p.scale : v
      controlResult.value[p.key] = { ok: true, text: 'read' }
    }
  }
}, { deep: true })

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
