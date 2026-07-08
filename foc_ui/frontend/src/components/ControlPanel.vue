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
const calibrationNotice = ref('')
const calibrationNoticeKind = ref<'dim' | 'ok' | 'warn' | 'err'>('dim')
const activeCalibration = ref<'full' | 'motor' | 'encoder' | 'anticog' | null>(null)
const calibrationStartedAt = ref(0)
const anticogCfgEnabled = ref(false)
const anticogCfgPreCalibrated = ref(false)
const anticogCfgPosThreshold = ref(1.0)
const anticogCfgVelThreshold = ref(1.0)

const streamMode = ref<ModeKey | null>(null)
const streamHz = ref(20)
let streamTimer: number | null = null
let modeRequestTimer: number | null = null
let calibrationPollTimer: number | null = null

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
const shadowCount = computed(() => oSocket.latest.value?.ch.shadow_count ?? 0)
const countInCpr = computed(() => oSocket.latest.value?.ch.count_in_cpr ?? 0)
const iq = computed(() => oSocket.latest.value?.ch.iq_meas ?? 0)
const vbus = computed(() => oSocket.latest.value?.ch.vbus ?? 0)
const ibus = computed(() => oSocket.latest.value?.ch.ibus ?? 0)

const targetPosTurns = computed(() => safeNumber(posTargetDeg.value) / 360)
const targetVelTurnsPerSec = computed(() => safeNumber(velTargetRpm.value) / 60)
const velocityIsAggressive = computed(() => Math.abs(safeNumber(velTargetRpm.value)) > 60)

const calibrationResultDefs = [
  { item: 0x01, label: 'phase_resistance', unit: 'ohm' },
  { item: 0x02, label: 'phase_inductance', unit: 'H' },
  { item: 0x03, label: 'encoder_phase_offset', unit: 'count' },
  { item: 0x04, label: 'encoder_direction', unit: '' },
]

function latestExtResponse(subCmd: number, item: number) {
  for (let i = oSocket.extResponses.value.length - 1; i >= 0; i--) {
    const r = oSocket.extResponses.value[i]
    if (r.sub_cmd === subCmd && r.item === item) return r
  }
  return null
}

const calibrationStatusFlags = computed(() => latestExtResponse(0x01, 0x00)?.ext_type ?? 0)
const isMotorCalibrated = computed(() => !!(calibrationStatusFlags.value & (1 << 0)))
const isEncoderReady = computed(() => !!(calibrationStatusFlags.value & (1 << 1)))
const encoderDirection = computed(() => latestExtResponse(0x04, 0x04)?.value ?? 0)
const closedLoopBlockers = computed(() => {
  const blockers: string[] = []
  if (!isMotorCalibrated.value) blockers.push('电机未校准')
  if (!isEncoderReady.value) blockers.push('编码器未 ready')
  if (Math.trunc(safeNumber(encoderDirection.value)) === 0) blockers.push('encoder direction 为 0')
  return blockers
})
const calibrationResults = computed(() => calibrationResultDefs.map((d) => {
  const r = latestExtResponse(0x04, d.item)
  return { ...d, status: r?.status, value: r?.value }
}))
const anticogStatusFlags = computed(() => latestExtResponse(0x08, 0x01)?.value ?? 0)
const anticogIndex = computed(() => latestExtResponse(0x08, 0x02)?.value ?? 0)
const anticogPosThreshold = computed(() => latestExtResponse(0x08, 0x03)?.value ?? 0)
const anticogVelThreshold = computed(() => latestExtResponse(0x08, 0x04)?.value ?? 0)
const anticogCoggingRatio = computed(() => latestExtResponse(0x08, 0x05)?.value ?? 0)
const anticogSystemError = computed(() => latestExtResponse(0x08, 0x06)?.value ?? 0)
const isAnticogCalibrating = computed(() => !!(anticogStatusFlags.value & (1 << 0)))
const isAnticogValid = computed(() => !!(anticogStatusFlags.value & (1 << 1)))
const isAnticogPreCalibrated = computed(() => !!(anticogStatusFlags.value & (1 << 2)))
const isAnticogEnabled = computed(() => !!(anticogStatusFlags.value & (1 << 3)))
const anticogProgress = computed(() => Math.max(0, Math.min(100, (safeNumber(anticogIndex.value) / 3600) * 100)))

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

function setCalibrationNotice(text: string, kind: 'dim' | 'ok' | 'warn' | 'err' = 'dim') {
  calibrationNotice.value = text
  calibrationNoticeKind.value = kind
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

function requestCalibrationSnapshot() {
  oSocket.extCmd(0x01, 0x00)
  calibrationResultDefs.forEach((d) => oSocket.extCmd(0x04, d.item))
}

function enterSelectedMode(targetMode: ModeKey) {
  if (targetMode !== selectedMode.value) return
  const blockers = closedLoopBlockers.value
  if (blockers.length) {
    requestedMode.value = null
    setCalibrationNotice(`无法进入闭环：${blockers.join(' / ')}。请先完整校准，或确认校准值有效后标记预校准并保存。`, 'err')
    return
  }

  stopStreaming(false)
  requestedMode.value = targetMode
  clearModeRequestLater()
  oSocket.clearErrors()
  if (targetMode === 'torque') oSocket.setTorque(0)
  if (targetMode === 'velocity') oSocket.setVel(0)
  if (targetMode === 'mit') oSocket.sendMit(0, 0, 0, 0, 0)
  oSocket.setServoMode(selected.value.servoMode)
  oSocket.setMode(selected.value.controlMode, selected.value.inputMode)
  setTimeout(() => oSocket.setState(8), targetMode === 'mit' ? 200 : 150)
}

function activateSelectedMode() {
  stopStreaming(false)
  requestedMode.value = selectedMode.value
  requestCalibrationSnapshot()
  setTimeout(() => enterSelectedMode(selectedMode.value), 220)
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

function readCalibrationResult(showNotice = true) {
  oSocket.extCmd(0x01, 0x00)
  calibrationResultDefs.forEach((d) => oSocket.extCmd(0x04, d.item))
  ;[0x01, 0x02, 0x03, 0x04, 0x05, 0x06].forEach((item) => oSocket.extCmd(0x08, item))
  if (showNotice) setCalibrationNotice('已请求校准状态与结果', 'dim')
}

function startCalibrationPolling() {
  readCalibrationResult(false)
  if (calibrationPollTimer != null) return
  calibrationPollTimer = window.setInterval(() => {
    readCalibrationResult(false)
    evaluateCalibrationStatus()
  }, 1000)
}

function stopCalibrationPolling() {
  if (calibrationPollTimer != null) {
    window.clearInterval(calibrationPollTimer)
    calibrationPollTimer = null
  }
}

function requestAxisState(state: number, label: string, action: 'full' | 'motor' | 'encoder') {
  stopStreaming(true)
  requestedMode.value = null
  activeCalibration.value = action
  calibrationStartedAt.value = Date.now()
  oSocket.clearErrors()
  oSocket.setState(1)
  window.setTimeout(() => oSocket.setState(state), 80)
  setCalibrationNotice(`${label} 已启动，等待状态回报`, 'warn')
  startCalibrationPolling()
}

function runFullCalibration() {
  requestAxisState(3, '完整校准', 'full')
}

function runMotorCalibration() {
  requestAxisState(4, '电机校准', 'motor')
}

function runEncoderCalibration() {
  requestAxisState(7, '编码器校准', 'encoder')
}

function markPrecalibrated() {
  oSocket.extCmd(0x02, 0x03, 3, 0)
  setCalibrationNotice('已发送：标记电机和编码器为预校准', 'warn')
  window.setTimeout(() => readCalibrationResult(false), 250)
}

function clearPrecalibrated() {
  oSocket.extCmd(0x02, 0x30, 3, 0)
  setCalibrationNotice('已发送：清除电机和编码器预校准标志', 'warn')
  window.setTimeout(() => readCalibrationResult(false), 250)
}

function saveConfiguration() {
  oSocket.extCmd(0x03, 0x00, 3, 0, 3.0)
  setCalibrationNotice('已请求保存配置，控制器可能会重启', 'warn')
}

function startAnticoggingCalibration() {
  stopStreaming(true)
  requestedMode.value = null
  activeCalibration.value = 'anticog'
  calibrationStartedAt.value = Date.now()
  oSocket.clearErrors()
  oSocket.setServoMode(2)
  oSocket.setMode(3, 5)
  oSocket.setState(8)
  setCalibrationNotice('齿槽转矩校准准备中：切入位置闭环', 'warn')
  window.setTimeout(() => {
    oSocket.anticoggingStart()
    setCalibrationNotice('齿槽转矩校准已启动，正在采集 map', 'warn')
    startCalibrationPolling()
  }, 250)
}

function applyAnticoggingConfig(field: 'enabled' | 'pre_calibrated' | 'pos_threshold' | 'vel_threshold' | 'reset') {
  const cfg: Record<string, unknown> = {}
  if (field === 'enabled') cfg.enabled = anticogCfgEnabled.value
  if (field === 'pre_calibrated') cfg.pre_calibrated = anticogCfgPreCalibrated.value
  if (field === 'pos_threshold') cfg.pos_threshold = safeNumber(anticogCfgPosThreshold.value)
  if (field === 'vel_threshold') cfg.vel_threshold = safeNumber(anticogCfgVelThreshold.value)
  if (field === 'reset') cfg.reset = true
  oSocket.anticoggingConfig(cfg as any)
  setCalibrationNotice(`已发送齿槽配置：${field}`, 'warn')
  window.setTimeout(() => readCalibrationResult(false), 250)
}

function evaluateCalibrationStatus() {
  const action = activeCalibration.value
  if (!action) return
  const elapsedMs = Date.now() - calibrationStartedAt.value
  if (allErrors.value.length || (action === 'anticog' && anticogSystemError.value)) {
    const detail = action === 'anticog' && anticogSystemError.value
      ? `，system_error=0x${Math.trunc(anticogSystemError.value).toString(16)}`
      : ''
    setCalibrationNotice(`校准失败：检测到错误${detail}`, 'err')
    activeCalibration.value = null
    stopCalibrationPolling()
    return
  }

  if (action === 'anticog') {
    if (isAnticogCalibrating.value) {
      setCalibrationNotice(`齿槽转矩校准中：${Math.round(anticogProgress.value)}% (${Math.trunc(anticogIndex.value)}/3600)`, 'warn')
      return
    }
    if (isAnticogValid.value && elapsedMs > 800) {
      setCalibrationNotice('齿槽转矩校准成功：map 有效', 'ok')
      activeCalibration.value = null
      stopCalibrationPolling()
    }
    return
  }

  if ([3, 4, 7].includes(axisState.value)) {
    setCalibrationNotice(`校准运行中：${axisStateName.value}`, 'warn')
    return
  }

  const done = action === 'motor'
    ? isMotorCalibrated.value
    : isMotorCalibrated.value && isEncoderReady.value
  if (done && elapsedMs > 800) {
    const label = action === 'motor' ? '电机校准' : action === 'encoder' ? '编码器校准' : '完整校准'
    setCalibrationNotice(`${label}成功：${isMotorCalibrated.value ? 'Motor OK' : 'Motor --'} / ${isEncoderReady.value ? 'Encoder OK' : 'Encoder --'}`, 'ok')
    activeCalibration.value = null
    stopCalibrationPolling()
  }
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

watch(() => oSocket.ready.value, (ready) => {
  if (ready) requestCalibrationSnapshot()
})

watch(streamHz, () => {
  if (streamMode.value) startStreaming(streamMode.value)
})

watch([
  axisState,
  isMotorCalibrated,
  isEncoderReady,
  isAnticogCalibrating,
  isAnticogValid,
  anticogSystemError,
  () => allErrors.value.length,
], () => {
  evaluateCalibrationStatus()
})

watch(isAnticogEnabled, (value) => {
  anticogCfgEnabled.value = value
})

watch(isAnticogPreCalibrated, (value) => {
  anticogCfgPreCalibrated.value = value
})

watch(anticogPosThreshold, (value) => {
  if (Number.isFinite(value) && value > 0) anticogCfgPosThreshold.value = value
})

watch(anticogVelThreshold, (value) => {
  if (Number.isFinite(value) && value > 0) anticogCfgVelThreshold.value = value
})

onBeforeUnmount(() => {
  stopStreaming(true)
  stopCalibrationPolling()
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

      <div v-if="closedLoopBlockers.length" class="explain warn">
        闭环前置条件未满足：{{ closedLoopBlockers.join(' / ') }}
      </div>

      <div class="encoder-readout">
        <div><span>累计 count</span><strong>{{ shadowCount }}</strong><em>count</em></div>
        <div><span>count (CPR)</span><strong>{{ countInCpr }}</strong><em>count</em></div>
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
        <div><span>位置</span><strong>{{ format(posDeg, 4) }}</strong><em>deg</em></div>
        <div><span>速度</span><strong>{{ format(velRpmActual, 3) }}</strong><em>rpm</em></div>
        <div><span>Iq</span><strong>{{ format(iq, 3) }}</strong><em>A</em></div>
        <div><span>母线</span><strong>{{ format(vbus, 1) }}</strong><em>V</em></div>
      </div>
    </div>

    <div class="calibration-card">
      <div class="card-head">
        <div>
          <div class="card-title">校准</div>
          <div class="card-sub">完整校准不包含齿槽转矩；齿槽 map 需要单独启动</div>
        </div>
        <button @click="readCalibrationResult()" :disabled="!oSocket.ready.value">读取状态</button>
      </div>

      <div class="calibration-section">
        <div class="section-title">基础校准</div>
        <div class="calibration-actions">
          <button @click="runFullCalibration" class="warn" :disabled="!oSocket.ready.value">完整校准</button>
          <button @click="runMotorCalibration" :disabled="!oSocket.ready.value">电机校准</button>
          <button @click="runEncoderCalibration" :disabled="!oSocket.ready.value">编码器校准</button>
        </div>
        <div class="calibration-status">
          <span class="pill" :class="isMotorCalibrated ? 'ok' : 'dim'">Motor {{ isMotorCalibrated ? 'OK' : '--' }}</span>
          <span class="pill" :class="isEncoderReady ? 'ok' : 'dim'">Encoder {{ isEncoderReady ? 'OK' : '--' }}</span>
        </div>
        <div class="calibration-results">
          <div v-for="r in calibrationResults" :key="r.item">
            <span>{{ r.label }}</span>
            <strong>{{ r.status === 0 && r.value != null ? format(r.value, r.item <= 0x02 ? 6 : 0) : '--' }}</strong>
            <em>{{ r.unit }}</em>
          </div>
        </div>
      </div>

      <div class="calibration-section">
        <div class="section-title">齿槽转矩</div>
        <div class="calibration-actions">
          <button @click="startAnticoggingCalibration" class="warn" :disabled="!oSocket.ready.value">开始齿槽校准</button>
          <button @click="applyAnticoggingConfig('reset')" :disabled="!oSocket.ready.value">重置 map</button>
          <button @click="applyAnticoggingConfig('enabled')" :disabled="!oSocket.ready.value">
            {{ anticogCfgEnabled ? '启用补偿' : '禁用补偿' }}
          </button>
        </div>
        <div class="calibration-status">
          <span class="pill" :class="isAnticogEnabled ? 'ok' : 'dim'">Comp {{ isAnticogEnabled ? 'ON' : 'OFF' }}</span>
          <span class="pill" :class="isAnticogValid ? 'ok' : 'dim'">Map {{ isAnticogValid ? 'Valid' : '--' }}</span>
          <span class="pill" :class="isAnticogPreCalibrated ? 'ok' : 'dim'">Precal {{ isAnticogPreCalibrated ? 'YES' : '--' }}</span>
          <span class="pill" :class="isAnticogCalibrating ? 'warn' : 'dim'">Index {{ Math.trunc(anticogIndex) }}/3600</span>
        </div>
        <div class="progress-track">
          <div class="progress-fill" :style="{ width: `${anticogProgress}%` }"></div>
        </div>
        <div class="anticog-grid">
          <label>位置阈值 (counts)
            <input type="number" step="0.1" v-model.number="anticogCfgPosThreshold" @keyup.enter="applyAnticoggingConfig('pos_threshold')" />
          </label>
          <label>速度阈值 (counts/s)
            <input type="number" step="0.1" v-model.number="anticogCfgVelThreshold" @keyup.enter="applyAnticoggingConfig('vel_threshold')" />
          </label>
          <button @click="applyAnticoggingConfig('pos_threshold')" :disabled="!oSocket.ready.value">set pos</button>
          <button @click="applyAnticoggingConfig('vel_threshold')" :disabled="!oSocket.ready.value">set vel</button>
        </div>
        <label class="check-row">
          <input type="checkbox" v-model="anticogCfgPreCalibrated" @change="applyAnticoggingConfig('pre_calibrated')" />
          齿槽 map 预校准有效
        </label>
        <div class="calibration-results compact">
          <div><span>pos_threshold</span><strong>{{ format(anticogPosThreshold, 4) }}</strong><em>counts</em></div>
          <div><span>vel_threshold</span><strong>{{ format(anticogVelThreshold, 4) }}</strong><em>counts/s</em></div>
          <div><span>cogging_ratio</span><strong>{{ format(anticogCoggingRatio, 6) }}</strong><em>turn</em></div>
          <div><span>system_error</span><strong>0x{{ Math.trunc(anticogSystemError).toString(16).padStart(8, '0') }}</strong><em></em></div>
        </div>
      </div>

      <div class="calibration-section">
        <div class="calibration-actions secondary">
          <button @click="markPrecalibrated" :disabled="!oSocket.ready.value">标记电机/编码器预校准</button>
          <button @click="clearPrecalibrated" :disabled="!oSocket.ready.value">清除电机/编码器预校准</button>
          <button @click="saveConfiguration" class="warn" :disabled="!oSocket.ready.value">保存配置</button>
        </div>
      </div>

      <div v-if="calibrationNotice" class="calibration-message" :class="calibrationNoticeKind">
        {{ calibrationNotice }}
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
.active-card, .telemetry-card, .calibration-card, .fault-card {
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
.encoder-readout {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
  margin-bottom: 8px;
}
.encoder-readout div {
  display: grid;
  grid-template-columns: 1fr auto auto;
  align-items: baseline;
  gap: 5px;
  padding: 7px 8px;
  background: rgba(15, 23, 42, 0.62);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 5px;
}
.encoder-readout span {
  color: var(--fg-dim);
  font-size: 11px;
}
.encoder-readout strong {
  font-family: var(--mono);
  font-size: 14px;
}
.encoder-readout em {
  color: var(--fg-dim);
  font-style: normal;
  font-size: 10px;
}
.calibration-card {
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.calibration-section {
  display: grid;
  gap: 8px;
  padding-top: 8px;
  border-top: 1px solid rgba(148, 163, 184, 0.18);
}
.calibration-section:first-of-type {
  padding-top: 0;
  border-top: none;
}
.section-title {
  font-size: 10px;
  color: var(--fg-dim);
  text-transform: uppercase;
  letter-spacing: 0.05em;
}
.calibration-actions {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 6px;
}
.calibration-actions.secondary button {
  min-height: 30px;
}
.calibration-status {
  display: flex;
  gap: 6px;
  flex-wrap: wrap;
}
.calibration-results {
  display: grid;
  gap: 6px;
}
.calibration-results.compact {
  gap: 4px;
}
.calibration-results div {
  display: grid;
  grid-template-columns: minmax(0, 1fr) auto auto;
  align-items: baseline;
  gap: 6px;
  padding: 6px 8px;
  background: rgba(15, 23, 42, 0.62);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 5px;
}
.calibration-results span {
  min-width: 0;
  color: var(--fg-dim);
  font-size: 11px;
}
.calibration-results strong {
  font-family: var(--mono);
  font-size: 12px;
}
.calibration-results em {
  color: var(--fg-dim);
  font-style: normal;
  font-size: 10px;
}
.progress-track {
  height: 7px;
  overflow: hidden;
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.24);
  border-radius: 999px;
}
.progress-fill {
  height: 100%;
  min-width: 0;
  background: var(--warn);
  transition: width 0.2s ease;
}
.anticog-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 6px;
  align-items: end;
}
.anticog-grid label {
  display: grid;
  gap: 4px;
}
.check-row {
  display: flex;
  align-items: center;
  gap: 6px;
}
.check-row input {
  width: auto;
}
.calibration-message {
  padding: 7px 8px;
  border: 1px solid var(--border);
  border-radius: 5px;
  font-size: 11px;
  line-height: 1.4;
}
.calibration-message.dim {
  color: var(--fg-dim);
  background: rgba(148, 163, 184, 0.08);
}
.calibration-message.ok {
  color: var(--ok);
  border-color: rgba(34, 197, 94, 0.45);
  background: rgba(34, 197, 94, 0.1);
}
.calibration-message.warn {
  color: var(--warn);
  border-color: rgba(245, 158, 11, 0.45);
  background: rgba(245, 158, 11, 0.1);
}
.calibration-message.err {
  color: var(--err);
  border-color: rgba(239, 68, 68, 0.5);
  background: rgba(239, 68, 68, 0.1);
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
  .calibration-actions, .anticog-grid { grid-template-columns: 1fr; }
}
</style>
