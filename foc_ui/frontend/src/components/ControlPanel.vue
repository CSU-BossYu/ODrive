<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'
import {
  decodeAxisErrors, decodeMotorErrors, decodeEncoderErrors, decodeControllerErrors,
  decodeOdriveSystemErrors,
} from '../channels'
import { AXIS_STATES, CONTROL_MODES, INPUT_MODES } from '../types'

type ModeKey = 'torque' | 'velocity' | 'position' | 'mit'

interface ModeDef {
  key: ModeKey
  title: string
  hint: string
  servoMode: number
  controlMode: number
  inputMode: number
}

const oSocket = useOdriveSocket()

const modes: ModeDef[] = [
  { key: 'torque', title: '力矩', hint: '持续发送力矩命令', servoMode: 0, controlMode: 1, inputMode: 1 },
  { key: 'velocity', title: '速度', hint: '按加减速限制逼近目标转速', servoMode: 1, controlMode: 2, inputMode: 2 },
  { key: 'position', title: '位置规划', hint: '发送单次绝对角度目标', servoMode: 2, controlMode: 3, inputMode: 5 },
  { key: 'mit', title: 'MIT 实时伺服', hint: '持续发送实时伺服帧', servoMode: 3, controlMode: 1, inputMode: 9 },
]

const selectedMode = ref<ModeKey>('velocity')
const requestedMode = ref<ModeKey | null>(null)
const lastPositionSentAt = ref(0)

const posTargetDeg = ref(0)
const posVelFFRpm = ref(0)
const velTargetRpm = ref(12)
const velTorqueFF = ref(0)
const torqueTarget = ref(0)
const mitPosDeg = ref(0)
const mitVelRpm = ref(0)
const mitKp = ref(0)
const mitKd = ref(0)
const mitTorque = ref(0)

const streamMode = ref<ModeKey | null>(null)
const streamHz = ref(20)
let streamTimer: number | null = null
let modeRequestTimer: number | null = null

const axisState = computed(() => oSocket.heartbeat.value?.axis_state ?? 0)
const axisStateName = computed(() => AXIS_STATES[axisState.value] ?? 'UNKNOWN')
const isClosedLoop = computed(() => axisState.value === 8)
const controlMode = computed(() => oSocket.latest.value?.ch.ctrl_mode ?? 0)
const inputMode = computed(() => oSocket.latest.value?.ch.input_mode ?? 0)
const controlModeName = computed(() => CONTROL_MODES[controlMode.value] ?? '?')
const inputModeName = computed(() => INPUT_MODES[inputMode.value] ?? '?')

const firmwareMode = computed<ModeKey | null>(() => {
  if (!isClosedLoop.value) return null
  if (controlMode.value === 1 && inputMode.value === 1) return 'torque'
  if (controlMode.value === 2 && inputMode.value === 2) return 'velocity'
  if (controlMode.value === 3 && inputMode.value === 5) return 'position'
  if (controlMode.value === 1 && inputMode.value === 9) return 'mit'
  return null
})

const selected = computed(() => modes.find((m) => m.key === selectedMode.value) ?? modes[0])
const isSelectedModeActive = computed(() => firmwareMode.value === selectedMode.value)
const isStreaming = computed(() => streamMode.value === selectedMode.value)

const posDeg = computed(() => (oSocket.latest.value?.ch.pos ?? 0) * 360)
const velRpmActual = computed(() => (oSocket.latest.value?.ch.vel ?? 0) * 60)
const iq = computed(() => oSocket.latest.value?.ch.iq_meas ?? 0)
const vbus = computed(() => oSocket.latest.value?.ch.vbus ?? 0)
const ibus = computed(() => oSocket.latest.value?.ch.ibus ?? 0)

const targetPosTurns = computed(() => safeNumber(posTargetDeg.value) / 360)
const targetVelTurnsPerSec = computed(() => safeNumber(velTargetRpm.value) / 60)
const velocityIsAggressive = computed(() => Math.abs(safeNumber(velTargetRpm.value)) > 60)

const cAxisErrs = computed(() => decodeAxisErrors(oSocket.heartbeat.value?.axis_error ?? 0))
const cOdriveErrs = computed(() => decodeOdriveSystemErrors(oSocket.latest.value?.ch.odrv_err ?? 0))
const cMotorErrs = computed(() => decodeMotorErrors(oSocket.latest.value?.ch.motor_err ?? 0))
const cEncoderErrs = computed(() => decodeEncoderErrors(oSocket.latest.value?.ch.enc_err ?? 0))
const cControllerErrs = computed(() => decodeControllerErrors(oSocket.latest.value?.ch.ctrl_err ?? 0))
const allErrors = computed(() => [
  ...cOdriveErrs.value,
  ...cAxisErrs.value,
  ...cMotorErrs.value,
  ...cEncoderErrs.value,
  ...cControllerErrs.value,
])

const faultText = computed(() => {
  if (!oSocket.ready.value) return 'CAN 未连接'
  if (allErrors.value.length) return `有 ${allErrors.value.length} 个错误`
  if (oSocket.heartbeat.value?.comm_timeout) return '通信超时'
  if (oSocket.heartbeat.value?.cmd_watchdog_expired) return '命令超时'
  if (oSocket.heartbeat.value?.quick_stop_active) return '快速停止中'
  return '无错误'
})

const modeStatusText = computed(() => {
  if (!oSocket.ready.value) return 'CAN 未连接'
  if (!isClosedLoop.value) return `轴状态：${axisStateName.value}`
  return `${controlModeName.value} / ${inputModeName.value}`
})

function safeNumber(v: unknown): number {
  return typeof v === 'number' && Number.isFinite(v) ? v : 0
}

function format(v: number, digits = 2): string {
  return Number.isFinite(v) ? v.toFixed(digits) : '--'
}

function selectMode(mode: ModeKey) {
  selectedMode.value = mode
}

function clearModeRequestLater() {
  if (modeRequestTimer != null) window.clearTimeout(modeRequestTimer)
  modeRequestTimer = window.setTimeout(() => {
    requestedMode.value = null
    modeRequestTimer = null
  }, 1800)
}

function activateSelectedMode() {
  stopStreaming(false)
  requestedMode.value = selectedMode.value
  clearModeRequestLater()
  oSocket.clearErrors()
  if (selectedMode.value === 'torque') oSocket.setTorque(0)
  if (selectedMode.value === 'velocity') oSocket.setVel(0)
  if (selectedMode.value === 'mit') oSocket.sendMit(0, 0, 0, 0, 0)
  oSocket.setServoMode(selected.value.servoMode)
  oSocket.setMode(selected.value.controlMode, selected.value.inputMode)
  setTimeout(() => oSocket.setState(8), selectedMode.value === 'mit' ? 200 : 150)
}

function disableAxis() {
  stopStreaming(true)
  requestedMode.value = null
  oSocket.setState(1)
}

function commandFor(mode: ModeKey) {
  if (mode === 'torque') return () => oSocket.setTorque(safeNumber(torqueTarget.value))
  if (mode === 'velocity') {
    return () => oSocket.setVel(safeNumber(velTargetRpm.value) / 60, safeNumber(velTorqueFF.value))
  }
  if (mode === 'mit') {
    return () => oSocket.sendMit(
      safeNumber(mitPosDeg.value) / 360,
      safeNumber(mitVelRpm.value) / 60,
      safeNumber(mitKp.value),
      safeNumber(mitKd.value),
      safeNumber(mitTorque.value),
    )
  }
  return null
}

function startStreaming(mode: ModeKey) {
  const fn = commandFor(mode)
  if (!fn) return
  stopStreaming(false)
  selectedMode.value = mode
  requestedMode.value = null
  fn()
  streamMode.value = mode
  const intervalMs = Math.max(20, Math.round(1000 / Math.max(1, safeNumber(streamHz.value))))
  streamTimer = window.setInterval(fn, intervalMs)
}

function stopStreaming(sendSafeCommand = true) {
  if (streamTimer != null) {
    clearInterval(streamTimer)
    streamTimer = null
  }
  const stopped = streamMode.value
  streamMode.value = null
  if (!sendSafeCommand) return
  if (stopped === 'velocity') oSocket.setVel(0)
  if (stopped === 'torque') oSocket.setTorque(0)
  if (stopped === 'mit') oSocket.sendMit(0, 0, 0, 0, 0)
}

function toggleStream() {
  if (isStreaming.value) stopStreaming(true)
  else startStreaming(selectedMode.value)
}

function applyPosition() {
  stopStreaming(false)
  requestedMode.value = null
  oSocket.setPos(safeNumber(posTargetDeg.value) / 360, safeNumber(posVelFFRpm.value) / 60, 0)
  lastPositionSentAt.value = Date.now()
}

function clearErrors() {
  oSocket.clearErrors()
}

watch(firmwareMode, (mode) => {
  if (mode && mode === requestedMode.value) {
    requestedMode.value = null
    if (modeRequestTimer != null) {
      window.clearTimeout(modeRequestTimer)
      modeRequestTimer = null
    }
  }
})

watch(streamHz, () => {
  if (streamMode.value) startStreaming(streamMode.value)
})

onBeforeUnmount(() => {
  stopStreaming(true)
  if (modeRequestTimer != null) window.clearTimeout(modeRequestTimer)
})
</script>

<template>
  <div class="panel control-panel">
    <div class="panel-head">
      <h3 class="panel-title">控制</h3>
      <span class="pill" :class="isClosedLoop ? 'ok' : 'dim'">{{ isClosedLoop ? '已使能' : '已失能' }}</span>
    </div>

    <div class="mode-cards">
      <button
        v-for="m in modes"
        :key="m.key"
        class="mode-card"
        :class="{
          selected: selectedMode === m.key,
          active: firmwareMode === m.key,
          pending: requestedMode === m.key,
        }"
        @click="selectMode(m.key)"
      >
        <span class="mode-title">{{ m.title }}</span>
        <span class="mode-hint">{{ m.hint }}</span>
        <span class="mode-state">
          {{ firmwareMode === m.key ? '当前模式' : requestedMode === m.key ? '切换中' : selectedMode === m.key ? '已选择' : '' }}
        </span>
      </button>
    </div>

    <div class="active-card">
      <div class="card-head">
        <div>
          <div class="card-title">{{ selected.title }}</div>
          <div class="card-sub">{{ modeStatusText }}</div>
        </div>
        <button @click="activateSelectedMode" class="primary" :class="{ active: isSelectedModeActive }">
          {{ isSelectedModeActive ? '已在该模式' : requestedMode === selectedMode ? '切换中...' : '进入模式' }}
        </button>
      </div>

      <div v-if="selectedMode === 'position'" class="mode-fields">
        <div class="explain">目标是输出轴绝对角度，不是增量移动。360 deg = 1 圈。</div>
        <div class="conversion">当前输入：{{ format(safeNumber(posTargetDeg), 2) }} deg = {{ format(targetPosTurns, 4) }} 圈</div>
        <label>输出轴绝对角度 (deg)
          <input type="number" step="1" v-model.number="posTargetDeg" @keyup.enter="applyPosition" />
        </label>
        <label>速度前馈 (rpm)
          <input type="number" step="1" v-model.number="posVelFFRpm" @keyup.enter="applyPosition" />
        </label>
        <button @click="applyPosition" class="primary">发送单次位置目标</button>
        <span v-if="lastPositionSentAt" class="pill ok">位置目标已发送</span>
      </div>

      <div v-else-if="selectedMode === 'velocity'" class="mode-fields">
        <div class="conversion" :class="{ warn: velocityIsAggressive }">
          当前输入：{{ format(safeNumber(velTargetRpm), 1) }} rpm = {{ format(targetVelTurnsPerSec, 4) }} 圈/秒
        </div>
        <div v-if="velocityIsAggressive" class="explain warn">
          当前台架上这个速度偏激进。之前 300 rpm 测试在到达目标前已经撞到 3A 电流限制。
        </div>
        <label>目标转速 (rpm)
          <input type="number" step="1" v-model.number="velTargetRpm" @keyup.enter="toggleStream" />
        </label>
        <label>力矩前馈 (Nm)
          <input type="number" step="0.001" v-model.number="velTorqueFF" @keyup.enter="toggleStream" />
        </label>
      </div>

      <div v-else-if="selectedMode === 'torque'" class="mode-fields">
        <label>目标力矩 (Nm)
          <input type="number" step="0.001" v-model.number="torqueTarget" @keyup.enter="toggleStream" />
        </label>
      </div>

      <div v-else class="mode-fields">
        <div class="mit-grid">
          <label>p_des (deg)<input type="number" step="1" v-model.number="mitPosDeg" /></label>
          <label>v_des (rpm)<input type="number" step="1" v-model.number="mitVelRpm" /></label>
          <label>kp<input type="number" step="1" v-model.number="mitKp" /></label>
          <label>kd<input type="number" step="0.01" v-model.number="mitKd" /></label>
          <label>t_ff (Nm)<input type="number" step="0.001" v-model.number="mitTorque" /></label>
        </div>
      </div>

      <div v-if="selectedMode !== 'position'" class="stream-row">
        <label>发送频率 (Hz)
          <input type="number" min="1" max="100" step="1" v-model.number="streamHz" />
        </label>
        <button @click="toggleStream" class="stream-button" :class="{ active: isStreaming }">
          {{ isStreaming ? '正在发送，点击停止' : '开始持续发送' }}
        </button>
      </div>
    </div>

    <div class="telemetry-card">
      <div class="card-title">当前读数</div>
      <div class="telemetry-grid">
        <div><span>位置</span><strong>{{ format(posDeg, 2) }}</strong><em>deg</em></div>
        <div><span>速度</span><strong>{{ format(velRpmActual, 1) }}</strong><em>rpm</em></div>
        <div><span>Iq</span><strong>{{ format(iq, 3) }}</strong><em>A</em></div>
        <div><span>母线</span><strong>{{ format(vbus, 1) }}</strong><em>V</em></div>
      </div>
    </div>

    <div class="fault-card" :class="{ fault: allErrors.length }">
      <div class="fault-head">
        <div>
          <div class="card-title">状态与错误</div>
          <div class="card-sub">{{ faultText }}</div>
        </div>
        <span class="pill" :class="allErrors.length ? 'err' : 'ok'">{{ allErrors.length ? '故障' : '正常' }}</span>
      </div>
      <div class="error-pills">
        <span class="pill" :class="cAxisErrs.length ? 'err' : 'ok'" title="Axis">A</span>
        <span class="pill" :class="cOdriveErrs.length ? 'err' : 'ok'" title="ODrive/System">O</span>
        <span class="pill" :class="cMotorErrs.length ? 'err' : 'ok'" title="Motor">M</span>
        <span class="pill" :class="cEncoderErrs.length ? 'err' : 'ok'" title="Encoder">E</span>
        <span class="pill" :class="cControllerErrs.length ? 'err' : 'ok'" title="Controller">C</span>
      </div>
      <div v-if="allErrors.length" class="error-list">
        {{ allErrors.map((e) => `${e.category}:${e.name}`).join(' / ') }}
      </div>
      <div class="fault-actions">
        <button @click="clearErrors" class="warn">清除错误</button>
        <button @click="disableAxis" class="danger">失能</button>
        <button @click="oSocket.estop()" class="danger">急停</button>
      </div>
    </div>
  </div>
</template>

<style scoped>
.control-panel {
  display: flex;
  flex-direction: column;
  gap: 10px;
  min-height: 0;
  overflow-y: auto;
}
.panel-head, .card-head, .stream-row, .fault-head {
  display: flex;
  align-items: center;
  gap: 8px;
}
.panel-head, .card-head, .fault-head { justify-content: space-between; }
.panel-title { margin: 0; }
.mode-cards {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
}
.mode-card {
  display: grid;
  gap: 3px;
  min-height: 72px;
  text-align: left;
  padding: 9px;
  background: rgba(15, 23, 42, 0.8);
  border-color: rgba(148, 163, 184, 0.28);
}
.mode-card.selected {
  border-color: var(--accent);
  background: rgba(59, 130, 246, 0.12);
}
.mode-card.active {
  border-color: var(--ok);
  background: rgba(34, 197, 94, 0.13);
  box-shadow: inset 3px 0 0 var(--ok);
}
.mode-card.pending {
  border-color: var(--warn);
  background: rgba(245, 158, 11, 0.12);
}
.mode-title {
  font-weight: 750;
  font-size: 14px;
}
.mode-hint, .mode-state, .card-sub {
  font-size: 10px;
  color: var(--fg-dim);
}
.mode-state {
  min-height: 12px;
  font-family: var(--mono);
  color: var(--accent-2);
}
.active-card, .telemetry-card, .fault-card {
  display: flex;
  flex-direction: column;
  gap: 10px;
  padding: 10px;
  background: rgba(2, 6, 23, 0.42);
  border: 1px solid rgba(148, 163, 184, 0.28);
  border-radius: 6px;
}
.card-title {
  font-size: 15px;
  font-weight: 750;
}
.mode-fields {
  display: grid;
  gap: 8px;
}
.mode-fields label, .stream-row label {
  display: grid;
  gap: 4px;
}
.stream-row {
  display: grid;
  grid-template-columns: 1fr 1.35fr;
  align-items: end;
}
.stream-button {
  min-height: 32px;
}
.stream-button.active, button.primary.active {
  background: var(--ok);
  border-color: var(--ok);
  color: white;
  font-weight: 700;
}
.explain {
  padding: 7px 8px;
  color: var(--fg-dim);
  background: rgba(6, 182, 212, 0.08);
  border: 1px solid rgba(6, 182, 212, 0.28);
  border-radius: 5px;
  font-size: 11px;
  line-height: 1.45;
}
.explain.warn, .conversion.warn {
  color: var(--warn);
  border-color: rgba(245, 158, 11, 0.35);
  background: rgba(245, 158, 11, 0.1);
}
.conversion {
  padding: 6px 8px;
  color: var(--fg-dim);
  background: rgba(148, 163, 184, 0.08);
  border: 1px solid rgba(148, 163, 184, 0.22);
  border-radius: 5px;
  font-family: var(--mono);
  font-size: 11px;
}
.mit-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 6px;
}
.mit-grid label { display: grid; gap: 4px; }
.telemetry-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
}
.telemetry-grid div {
  display: grid;
  grid-template-columns: 1fr auto auto;
  align-items: baseline;
  gap: 5px;
  padding: 7px 8px;
  background: rgba(15, 23, 42, 0.62);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 5px;
}
.telemetry-grid span {
  color: var(--fg-dim);
  font-size: 11px;
}
.telemetry-grid strong {
  font-family: var(--mono);
  font-size: 14px;
}
.telemetry-grid em {
  color: var(--fg-dim);
  font-style: normal;
  font-size: 10px;
}
.fault-card.fault {
  border-color: rgba(239, 68, 68, 0.45);
  background: rgba(239, 68, 68, 0.06);
}
.error-pills {
  display: flex;
  gap: 5px;
}
.error-pills .pill {
  min-width: 22px;
  text-align: center;
}
.error-list {
  color: var(--err);
  font-family: var(--mono);
  font-size: 10px;
  line-height: 1.45;
  word-break: break-word;
}
.fault-actions {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 6px;
}
@media (max-width: 1280px) {
  .mode-cards { grid-template-columns: 1fr; }
  .stream-row { grid-template-columns: 1fr; }
}
</style>
