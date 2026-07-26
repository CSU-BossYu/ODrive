<script setup lang="ts">
// Friction-compensation panel: view/edit the 13 Stribeck params + run the
// one-click breakaway-torque calibration sweep.
//
// Fields ride on the existing get/set_control_config path (ext 0x0B/0x0C,
// items 0x70-0x7C). The Calibrate button launches a backend torque sweep
// (friction_calibrate) that measures breakaway and writes static/coulomb/
// max/slew; progress streams back as friction_progress, the final values
// as friction_result.

import { ref, watch, onMounted, computed } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const oSocket = useOdriveSocket()

interface FrictionParamDef {
  key: string; label: string; unit: string
  min: number; max: number; step: number
  item: number; isFloat: boolean
  infDisabled?: boolean  // +inf means "disabled" (max_torque / slew_rate)
}

const FRICTION_PARAMS: FrictionParamDef[] = [
  { key: 'friction_pos_deadband',   label: '位置死区',        unit: 'turn',         min: 0, max: 1,    step: 0.0001, item: 0x71, isFloat: true },
  { key: 'friction_vel_deadband',   label: '速度死区',        unit: 'turn/s',       min: 0, max: 1,    step: 0.0001, item: 0x72, isFloat: true },
  { key: 'friction_stribeck_vel',   label: 'Stribeck 速度',   unit: 'turn/s',       min: 0, max: 10,   step: 0.001,  item: 0x73, isFloat: true },
  { key: 'friction_static_pos',     label: '静摩擦 (+)',      unit: 'Nm',           min: 0, max: 50,   step: 0.01,   item: 0x74, isFloat: true },
  { key: 'friction_static_neg',     label: '静摩擦 (-)',      unit: 'Nm',           min: 0, max: 50,   step: 0.01,   item: 0x75, isFloat: true },
  { key: 'friction_coulomb_pos',    label: '库仑摩擦 (+)',    unit: 'Nm',           min: 0, max: 50,   step: 0.01,   item: 0x76, isFloat: true },
  { key: 'friction_coulomb_neg',    label: '库仑摩擦 (-)',    unit: 'Nm',           min: 0, max: 50,   step: 0.01,   item: 0x77, isFloat: true },
  { key: 'friction_viscous_pos',    label: '粘性摩擦 (+)',    unit: 'Nm/(turn/s)',  min: 0, max: 100,  step: 0.001,  item: 0x78, isFloat: true },
  { key: 'friction_viscous_neg',    label: '粘性摩擦 (-)',    unit: 'Nm/(turn/s)',  min: 0, max: 100,  step: 0.001,  item: 0x79, isFloat: true },
  { key: 'friction_max_torque',     label: '补偿上限',        unit: 'Nm',           min: 0, max: 100,  step: 0.01,   item: 0x7A, isFloat: true, infDisabled: true },
  { key: 'friction_torque_slew_rate', label: '斜率限制',      unit: 'Nm/s',         min: 0, max: 10000, step: 1,     item: 0x7B, isFloat: true, infDisabled: true },
]

const ITEM_ENABLE_POS = 0x70
const ITEM_ENABLE_MIT = 0x7C

const enablePos = ref(false)
const enableMit = ref(false)
const values = ref<Record<string, number>>(
  Object.fromEntries(FRICTION_PARAMS.map((p) => [p.key, 0]))
)
// Tracks fields whose read-back is +inf (disabled). JSON can't carry Infinity
// on the write path, so "set ∞" sends a huge finite number instead.
const isInf = ref<Record<string, boolean>>(
  Object.fromEntries(FRICTION_PARAMS.map((p) => [p.key, false]))
)
const result = ref<Record<string, { ok: boolean; text: string } | null>>(
  Object.fromEntries(FRICTION_PARAMS.map((p) => [p.key, null]))
)
const enableResult = ref<{
  pos: { ok: boolean; text: string } | null
  mit: { ok: boolean; text: string } | null
}>({ pos: null, mit: null })

// setControlConfig is one-way on the wire, but the backend replies with an
// ext_resp (sub_cmd 0x0C, item = the param id) carrying the firmware status.
// Track the most recent write so its reply updates the right badge — without
// this the UI shows "sent" even when the firmware rejected it (BUSY_ARMED
// while the axis is still armed).
const SUB_SET_CTRL = 0x0C
const pendingParam = ref<{ item: number; key: string } | null>(null)
const pendingEnable = ref<{ item: number; which: 'pos' | 'mit' } | null>(null)

function latestExtResponse(sub: number, item: number) {
  for (let i = oSocket.extResponses.value.length - 1; i >= 0; i--) {
    const r = oSocket.extResponses.value[i]
    if (r.sub_cmd === sub && r.item === item) return r
  }
  return null
}

function statusText(status: number): string {
  switch (status) {
    case 0: return 'ok'
    case 1: return 'UNKNOWN'
    case 2: return 'READONLY'
    case 3: return 'INVALID_TYPE'
    case 4: return 'INVALID_VALUE'
    case 5: return 'BUSY_ARMED (disarm first)'
    default: return `status ${status}`
  }
}

const maxTorque = ref(2.0)
const velThreshold = ref(0.01)

const running = computed(() => oSocket.frictionRunning.value)
const progress = computed(() => oSocket.frictionProgress.value)
const calibResult = computed(() => oSocket.frictionResult.value)

watch(() => oSocket.controlConfig.value, (cfg) => {
  if (!cfg) return
  enablePos.value = !!cfg.enable_friction_compensation
  enableMit.value = !!cfg.enable_mit_friction_compensation
  for (const p of FRICTION_PARAMS) {
    const v = cfg[p.key]
    if (typeof v === 'number' && Number.isFinite(v)) {
      values.value[p.key] = v
      isInf.value[p.key] = false
    } else if (typeof v === 'number' && !Number.isFinite(v)) {
      // +inf (disabled) — leave the input at its last finite value.
      isInf.value[p.key] = v > 0
    }
  }
}, { deep: true })

// After a successful sweep, the backend wrote new values — re-read to refresh.
watch(() => oSocket.frictionResult.value, (r) => {
  if (r && r.ok) oSocket.getControlConfig()
})

onMounted(() => { oSocket.getControlConfig() })

// Re-fetch when the socket becomes ready — onMounted may fire before the WS
// is open, in which case the getControlConfig send is silently dropped and
// the fields would stay at their 0 defaults until a manual "读取".
watch(() => oSocket.ready.value, (r) => {
  if (r) oSocket.getControlConfig()
})

// Correlate the ext_resp for the most recent param/enable write so the badge
// reflects the firmware's verdict (ok / BUSY_ARMED / INVALID_VALUE / …)
// instead of a blindly-optimistic "sent".
watch(() => oSocket.extSeq.value, () => {
  if (pendingParam.value) {
    const r = latestExtResponse(SUB_SET_CTRL, pendingParam.value.item)
    if (r) {
      result.value[pendingParam.value.key] = { ok: r.status === 0, text: statusText(r.status) }
      pendingParam.value = null
    }
  }
  if (pendingEnable.value) {
    const r = latestExtResponse(SUB_SET_CTRL, pendingEnable.value.item)
    if (r) {
      enableResult.value[pendingEnable.value.which] = { ok: r.status === 0, text: statusText(r.status) }
      pendingEnable.value = null
    }
  }
})

function applyEnable(which: 'pos' | 'mit') {
  const item = which === 'pos' ? ITEM_ENABLE_POS : ITEM_ENABLE_MIT
  const v = which === 'pos' ? enablePos.value : enableMit.value
  oSocket.setControlConfig(item, v ? 1 : 0, false)
  enableResult.value[which] = { ok: true, text: 'sent…' }
  pendingEnable.value = { item, which }
}

function applyParam(p: FrictionParamDef) {
  const v = values.value[p.key]
  if (typeof v !== 'number' || !Number.isFinite(v) || v < p.min || v > p.max) {
    result.value[p.key] = { ok: false, text: `Out of range [${p.min}, ${p.max}]` }
    return
  }
  oSocket.setControlConfig(p.item, v, p.isFloat)
  pendingParam.value = { item: p.item, key: p.key }
  result.value[p.key] = { ok: true, text: 'sent…' }
  isInf.value[p.key] = false
}

function setInf(p: FrictionParamDef) {
  // JSON can't carry Infinity; send a huge finite number that the firmware
  // clamps to (effectively no limit, well above any real Tlim).
  oSocket.setControlConfig(p.item, 1e30, p.isFloat)
  result.value[p.key] = { ok: true, text: 'set ∞' }
  isInf.value[p.key] = true
}

function startCalib() {
  oSocket.frictionCalibrate(maxTorque.value, velThreshold.value)
}
function cancelCalib() {
  oSocket.frictionCalibrateCancel()
}
function saveConfig() {
  if (confirm('保存配置到 Flash？请先确保轴已 disarm（IDLE）。保存后控制器不会复位。')) {
    oSocket.extCmd(0x03, 0x00, 3, 0, 3.0)  // SAVE_CONFIGURATION
  }
}
</script>

<template>
  <div class="panel friction-panel">
    <div class="row">
      <h3 class="panel-title grow">摩擦补偿</h3>
      <button @click="oSocket.getControlConfig()" :disabled="!oSocket.ready.value">读取</button>
      <button @click="saveConfig" :disabled="!oSocket.ready.value" class="warn" title="保存到 NVM（需先 disarm 到 IDLE）">保存</button>
    </div>

    <div class="section">
      <div class="section-heading"><span>使能</span></div>
      <div class="enable-row">
        <label>
          <input type="checkbox" v-model="enablePos" @change="applyEnable('pos')" :disabled="!oSocket.ready.value" />
          位置/速度伺服补偿
        </label>
        <span v-if="enableResult.pos" class="result"
          :class="{ ok: enableResult.pos.ok, err: !enableResult.pos.ok }">{{ enableResult.pos.text }}</span>
      </div>
      <div class="enable-row">
        <label>
          <input type="checkbox" v-model="enableMit" @change="applyEnable('mit')" :disabled="!oSocket.ready.value" />
          MIT 伺服补偿
        </label>
        <span v-if="enableResult.mit" class="result"
          :class="{ ok: enableResult.mit.ok, err: !enableResult.mit.ok }">{{ enableResult.mit.text }}</span>
      </div>
    </div>

    <div class="section">
      <div class="section-heading"><span>Stribeck 参数 (输出轴)</span></div>
      <div v-for="p in FRICTION_PARAMS" :key="p.key" class="param-row">
        <label :title="`${p.min}..${p.max}`">{{ p.label }}</label>
        <div class="row">
          <input
            type="number" :step="p.step" :min="p.min" :max="p.max"
            v-model.number="values[p.key]"
            :disabled="!oSocket.ready.value"
            @keyup.enter="applyParam(p)"
          />
          <span class="unit">{{ p.unit }}</span>
          <button @click="applyParam(p)" :disabled="!oSocket.ready.value">设置</button>
          <button v-if="p.infDisabled" @click="setInf(p)" :disabled="!oSocket.ready.value" title="设为无穷大（禁用限幅）">∞</button>
        </div>
        <div class="row badges">
          <span v-if="isInf[p.key]" class="badge inf" title="当前为无穷大（禁用）">∞ 已禁用</span>
          <span v-if="result[p.key]" class="result"
            :class="{ ok: result[p.key]?.ok, err: !result[p.key]?.ok }">
            {{ result[p.key]?.text }}
          </span>
        </div>
      </div>
    </div>

    <div class="section calibrate-card">
      <div class="section-heading"><span>标定 (破静摩擦扫掠)</span></div>
      <div class="calib-note">
        一键测正反向 breakaway 并写入 static/coulomb/max/slew。需电机已校准、轴可进入闭环。扫掠中会切到力矩模式，结束后恢复原模式。
      </div>
      <div class="param-row">
        <label>最大扫掠扭矩</label>
        <div class="row">
          <input type="number" step="0.05" min="0.1" max="10" v-model.number="maxTorque" :disabled="running" />
          <span class="unit">Nm</span>
        </div>
      </div>
      <div class="param-row">
        <label>刚动速度阈值</label>
        <div class="row">
          <input type="number" step="0.001" min="0.001" max="1" v-model.number="velThreshold" :disabled="running" />
          <span class="unit">turn/s</span>
        </div>
      </div>
      <div class="calib-actions">
        <button @click="startCalib" class="warn" :disabled="!oSocket.ready.value || running">标定</button>
        <button @click="cancelCalib" :disabled="!running">中止</button>
      </div>
      <div v-if="running || progress" class="progress-track">
        <div class="progress-fill" :style="{ width: `${progress?.progress ?? 0}%` }"></div>
      </div>
      <div v-if="progress" class="progress-stage">{{ progress.stage }} ({{ progress.progress.toFixed(0) }}%)</div>
      <div v-if="calibResult" class="calib-result" :class="{ ok: calibResult.ok, err: !calibResult.ok }">
        <div v-if="calibResult.error">{{ calibResult.error }}</div>
        <template v-if="calibResult.ok">
          <div>breakaway +: {{ calibResult.breakaway_pos.toFixed(3) }} Nm &nbsp; -: {{ calibResult.breakaway_neg.toFixed(3) }} Nm</div>
          <div>static +/−: {{ calibResult.static_pos.toFixed(3) }} / {{ calibResult.static_neg.toFixed(3) }} Nm</div>
          <div>coulomb +/−: {{ calibResult.coulomb_pos.toFixed(3) }} / {{ calibResult.coulomb_neg.toFixed(3) }} Nm</div>
          <div>max: {{ calibResult.max_torque.toFixed(3) }} Nm &nbsp; slew: {{ calibResult.slew_rate.toFixed(1) }} Nm/s</div>
          <div class="hint">已写入，未使能。勾选上方使能开关启用。</div>
        </template>
      </div>
    </div>
  </div>
</template>

<style scoped>
.friction-panel { display: flex; flex-direction: column; gap: 6px; min-height: 0; overflow-y: auto; }
.section { display: flex; flex-direction: column; gap: 4px; padding-bottom: 4px; border-bottom: 1px solid var(--border); }
.section-heading {
  font-size: 10px; color: var(--fg-dim); text-transform: uppercase;
  padding-top: 2px;
}
.enable-row { display: flex; align-items: center; gap: 8px; font-size: 12px; }
.enable-row label { display: flex; align-items: center; gap: 4px; cursor: pointer; }
.param-row { display: flex; flex-direction: column; gap: 2px; }
.param-row > label { font-size: 12px; color: var(--fg); }
.row { display: flex; align-items: center; gap: 4px; }
.row input[type="number"] { flex: 1; min-width: 0; }
.unit { font-size: 10px; color: var(--fg-dim); min-width: 36px; text-align: right; }
.badges { gap: 8px; min-height: 14px; }
.badge { font-size: 10px; padding: 1px 4px; border-radius: 3px; }
.badge.inf { background: var(--bg-2); color: var(--fg-dim); }
.result { font-size: 10px; font-family: var(--mono); }
.result.ok { color: var(--ok); }
.result.err { color: var(--err); }
.calibrate-card { border-bottom: none; }
.calib-note { font-size: 10px; color: var(--fg-dim); line-height: 1.4; }
.calib-actions { display: flex; gap: 6px; }
.progress-track {
  height: 7px; overflow: hidden;
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.24);
  border-radius: 999px;
}
.progress-fill {
  height: 100%; min-width: 0;
  background: var(--warn);
  transition: width 0.2s ease;
}
.progress-stage { font-size: 10px; color: var(--fg-dim); font-family: var(--mono); }
.calib-result { font-size: 11px; font-family: var(--mono); padding: 4px; border-radius: 3px; background: var(--bg-2); }
.calib-result.ok { color: var(--ok); }
.calib-result.err { color: var(--err); }
.calib-result .hint { color: var(--fg-dim); font-size: 10px; margin-top: 2px; }
</style>
