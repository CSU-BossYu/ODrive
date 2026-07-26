<script setup lang="ts">
import { computed, onMounted, ref } from 'vue'
import { useOdriveSocket, EXPECTED_PROTOCOL_VERSION } from '../composables/useOdriveSocket'
import {
  decodeAxisErrors, decodeMotorErrors, decodeEncoderErrors, decodeControllerErrors,
  decodeOdriveSystemErrors,
} from '../channels'
import { AXIS_STATES } from '../types'

interface CanIface { interface: string; desc: string }

const oSocket = useOdriveSocket()
const canInterfaces = ref<CanIface[]>([])
const selectedInterface = ref('pcan')
const selectedChannel = ref('PCAN_USBBUS1')
const nodeId = ref(0)
const canConnecting = ref(false)
const canErrorMsg = ref('')
const canFormValid = computed(() =>
  selectedInterface.value.trim().length > 0
  && selectedChannel.value.trim().length > 0
  && Number.isInteger(nodeId.value)
  && nodeId.value >= 0
  && nodeId.value <= 63
)

async function refreshInterfaces() {
  try {
    const r = await fetch('/api/can/interfaces')
    const j = await r.json()
    canInterfaces.value = j.interfaces ?? []
    if (canInterfaces.value.length && !canInterfaces.value.find(i => i.interface === selectedInterface.value)) {
      selectedInterface.value = canInterfaces.value[0].interface
    }
  } catch {
    canInterfaces.value = []
  }
}

async function canConnect() {
  if (!canFormValid.value || canConnecting.value) return
  canConnecting.value = true
  canErrorMsg.value = ''
  try {
    const r = await fetch('/api/can/connect', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        interface: selectedInterface.value,
        channel: selectedChannel.value,
        bitrate: 1000000,
        node_id: nodeId.value,
      }),
    })
    if (!r.ok) {
      const j = await r.json().catch(() => ({ error: r.statusText }))
      canErrorMsg.value = j.error || `HTTP ${r.status}`
    }
  } catch (e) {
    canErrorMsg.value = String(e)
  } finally {
    canConnecting.value = false
  }
}

async function canDisconnect() {
  canErrorMsg.value = ''
  try {
    const r = await fetch('/api/can/disconnect', { method: 'POST' })
    if (!r.ok) {
      const j = await r.json().catch(() => ({ error: r.statusText }))
      canErrorMsg.value = j.error || `HTTP ${r.status}`
    }
  } catch (e) {
    canErrorMsg.value = String(e)
  }
}

onMounted(refreshInterfaces)

const vbus = computed(() => oSocket.latest.value?.ch.vbus ?? 0)
const velocityRpm = computed(() => (oSocket.latest.value?.ch.vel ?? 0) * 60)
const iq = computed(() => oSocket.latest.value?.ch.iq_meas ?? 0)
const ibus = computed(() => oSocket.latest.value?.ch.ibus ?? 0)
const stats = computed(() => oSocket.status.value)
const transportConnected = computed(() => !!stats.value?.connected)
const axisStateValue = computed(() => oSocket.heartbeat.value?.axis_state ?? 0)
const axisStateName = computed(() => AXIS_STATES[axisStateValue.value] ?? 'UNKNOWN')

// Firmware reported 0 NVM bytes loaded on boot -> running factory defaults
// (e.g. after a config_version bump invalidated the saved config).
const configDefaulted = computed(() => oSocket.userConfigLoaded.value === 0)

// Firmware speaks an older CAN protocol than this UI expects.
const protocolMismatched = computed(() => {
  const v = oSocket.protocolVersion.value
  return v !== null && v < EXPECTED_PROTOCOL_VERSION
})

const axisErrs = computed(() => decodeAxisErrors(oSocket.heartbeat.value?.axis_error ?? 0))
const odriveErrs = computed(() => decodeOdriveSystemErrors(oSocket.latest.value?.ch.odrv_err ?? 0))
const motorErrs = computed(() => decodeMotorErrors(oSocket.latest.value?.ch.motor_err ?? 0))
const encoderErrs = computed(() => decodeEncoderErrors(oSocket.latest.value?.ch.enc_err ?? 0))
const controllerErrs = computed(() => decodeControllerErrors(oSocket.latest.value?.ch.ctrl_err ?? 0))
const errorCount = computed(() =>
  axisErrs.value.length + odriveErrs.value.length + motorErrs.value.length +
  encoderErrs.value.length + controllerErrs.value.length
)

const systemClass = computed(() => {
  const hb = oSocket.heartbeat.value
  if (!oSocket.ready.value) return 'dim'
  if (errorCount.value || hb?.comm_timeout || hb?.cmd_watchdog_expired) return 'err'
  if (hb?.quick_stop_active || hb?.mit_frame_stale) return 'warn'
  if (axisStateValue.value === 8) return 'ok'
  return 'dim'
})

const systemText = computed(() => {
  const hb = oSocket.heartbeat.value
  if (!oSocket.ready.value) return '未连接'
  if (errorCount.value) return `故障 ${errorCount.value}`
  if (hb?.comm_timeout) return '通信超时'
  if (hb?.cmd_watchdog_expired) return '命令超时'
  if (hb?.quick_stop_active) return '快速停止'
  if (hb?.holding) return '保持'
  if (axisStateValue.value === 8) return '就绪'
  return axisStateName.value
})

const runtimeFlags = computed(() => {
  const hb = oSocket.heartbeat.value
  return [
    { key: 'run', label: 'RUN', active: !!hb?.running, cls: 'ok', title: '控制器运行中' },
    { key: 'hold', label: 'HOLD', active: !!hb?.holding, cls: 'ok', title: '保持当前位置' },
    { key: 'qs', label: 'QS', active: !!hb?.quick_stop_active, cls: 'warn', title: '快速停止中' },
    { key: 'comm', label: 'COMM', active: !!hb?.comm_timeout, cls: 'err', title: '通信超时' },
    { key: 'wdog', label: 'WDOG', active: !!hb?.cmd_watchdog_expired, cls: 'err', title: '命令看门狗超时' },
    { key: 'mit', label: 'MIT', active: !!hb?.mit_frame_stale, cls: 'warn', title: 'MIT 帧过期' },
    { key: 'done', label: 'DONE', active: !!hb?.traj_done, cls: 'ok', title: '轨迹完成' },
  ].filter((f) => f.active)
})

function toggleRecord() {
  if (oSocket.recording.value) {
    oSocket.recStop()
  } else {
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)
    oSocket.recStart(`odrive_capture_${stamp}.csv`)
  }
}

function fmt(v: number, digits = 2): string {
  return Number.isFinite(v) ? v.toFixed(digits) : '--'
}
</script>

<template>
  <div class="status-bar">
    <div class="cell conn">
      <span class="section-kicker">CAN</span>
      <select v-model="selectedInterface" :disabled="transportConnected" class="iface-select">
        <option v-for="i in canInterfaces" :key="i.interface" :value="i.interface">{{ i.interface }}</option>
      </select>
      <input v-model="selectedChannel" :disabled="transportConnected" class="ch-input" placeholder="PCAN_USBBUS1" />
      <label class="node-label">N<input type="number" v-model.number="nodeId" :disabled="transportConnected" min="0" max="63" class="node-input" /></label>
      <button @click="refreshInterfaces" :disabled="transportConnected" title="刷新接口" class="icon-btn">刷新</button>
      <button v-if="!transportConnected" @click="canConnect" :disabled="canConnecting || !canFormValid" class="primary">
        {{ canConnecting ? '连接中...' : '连接' }}
      </button>
      <button v-else @click="canDisconnect" class="danger">断开</button>
      <span v-if="canErrorMsg" class="conn-error">{{ canErrorMsg }}</span>
    </div>

    <div class="divider"></div>
    <span class="pill system-pill" :class="systemClass">{{ systemText }}</span>
    <span class="pill dim">{{ axisStateName }}</span>
    <span v-if="configDefaulted" class="pill cfg-default-pill" title="固件未加载已保存的配置（config_version 变更或 NVM 校验失败），运行在出厂默认值。请重新标定并 save_configuration。">CFG 默认</span>
    <span v-if="protocolMismatched" class="pill cfg-default-pill" title="固件 CAN 协议版本低于上位机预期，部分功能可能不可用或语义不符。">PROTO</span>

    <div class="divider"></div>
    <div class="readings">
      <div class="reading"><span class="k">Vbus</span><span class="v">{{ fmt(vbus, 1) }}</span><span class="u">V</span></div>
      <div class="reading"><span class="k">速度</span><span class="v">{{ fmt(velocityRpm, 2) }}</span><span class="u">rpm</span></div>
      <div class="reading"><span class="k">Iq</span><span class="v">{{ fmt(iq, 3) }}</span><span class="u">A</span></div>
      <div class="reading"><span class="k">Ibus</span><span class="v">{{ fmt(ibus, 3) }}</span><span class="u">A</span></div>
    </div>

    <div class="runtime-pills" v-if="runtimeFlags.length">
      <span v-for="f in runtimeFlags" :key="f.key" class="pill" :class="f.cls" :title="f.title">{{ f.label }}</span>
    </div>

    <div class="divider"></div>
    <div class="reading"><span class="k">RX</span><span class="v">{{ stats?.frames_rx?.toLocaleString() ?? 0 }}</span></div>
    <div class="reading"><span class="k">TX</span><span class="v">{{ stats?.frames_tx?.toLocaleString() ?? 0 }}</span></div>
    <div class="reading" v-if="stats?.bus_errors"><span class="k">err</span><span class="v err-val">{{ stats.bus_errors }}</span></div>

    <div class="spacer"></div>
    <button @click="toggleRecord" :class="oSocket.recording.value ? 'rec-on' : 'rec-off'" class="rec-btn">
      <span class="rec-dot"></span>{{ oSocket.recording.value ? '停止' : '录制' }}
    </button>
  </div>
</template>

<style scoped>
.status-bar {
  display: flex;
  align-items: center;
  gap: 9px;
  padding: 7px 12px;
  background: var(--bg-2);
  flex-wrap: nowrap;
  overflow-x: auto;
  scrollbar-width: thin;
}
.cell { display: flex; align-items: center; gap: 4px; white-space: nowrap; }
.conn { flex: 0 0 auto; flex-wrap: wrap; }
.section-kicker {
  margin-right: 2px;
  color: var(--accent-2);
  font: 750 9px/1 var(--mono);
  letter-spacing: .12em;
}
.conn-error {
  font-size: 10px;
  color: var(--err);
  font-family: var(--mono);
  padding: 2px 4px;
  background: rgba(239, 68, 68, 0.1);
  border-radius: 3px;
  max-width: 200px;
  overflow: hidden;
  text-overflow: ellipsis;
}
.iface-select { width: 82px; }
.ch-input { width: 124px; font-size: 11px; font-family: var(--mono); padding: 4px 7px; }
.node-label { font-size: 10px; color: var(--fg-dim); display: flex; align-items: center; gap: 2px; }
.node-input { width: 36px; font-size: 11px; text-align: center; }
.icon-btn { padding: 6px 8px; }
.divider {
  width: 1px;
  align-self: stretch;
  background: var(--border-subtle);
  margin: 2px 2px;
  flex: 0 0 auto;
}
.system-pill { min-width: 78px; text-align: center; font-weight: 750; }
.cfg-default-pill {
  background: rgba(245, 158, 11, 0.16);
  color: var(--warn);
  border: 1px solid rgba(245, 158, 11, 0.35);
  font-size: 10px;
  font-family: var(--mono);
  padding: 1px 5px;
  white-space: nowrap;
}
.readings { display: flex; gap: 13px; }
.reading { display: flex; align-items: baseline; gap: 3px; white-space: nowrap; font-family: var(--mono); }
.reading .k { font-size: 9px; color: var(--fg-muted); text-transform: uppercase; }
.reading .v { font-size: 13px; color: var(--fg-strong); font-weight: 650; min-width: 3ch; }
.reading .u { font-size: 9px; color: var(--fg-dim); }
.reading .v.err-val { color: var(--err); }
.runtime-pills { display: flex; gap: 3px; }
.runtime-pills .pill { font-size: 10px; font-family: var(--mono); padding: 1px 5px; }
.runtime-pills .pill.ok { background: rgba(34, 197, 94, 0.15); color: var(--ok); border: 1px solid rgba(34, 197, 94, 0.3); }
.runtime-pills .pill.warn { background: rgba(245, 158, 11, 0.16); color: var(--warn); border: 1px solid rgba(245, 158, 11, 0.35); }
.runtime-pills .pill.err { background: var(--err); color: white; border: 1px solid var(--err); }
.spacer { flex: 1; }
.rec-btn {
  display: inline-flex;
  align-items: center;
  gap: 5px;
  font-size: 11px;
  min-height: 31px;
}
.rec-off { color: var(--fg-dim); }
.rec-on { background: var(--err); border-color: var(--err); color: white; }
.rec-dot {
  display: inline-block;
  width: 8px; height: 8px;
  border-radius: 50%;
  background: var(--err);
}
.rec-on .rec-dot { background: white; animation: pulse 1s infinite; }
@keyframes pulse { 50% { opacity: 0.4; } }

@media (max-width: 760px) {
  .status-bar { padding: 7px 0; }
  .readings { display: none; }
}
</style>
