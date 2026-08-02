<script setup lang="ts">
import { computed, ref } from 'vue'
import { FaultCode, FaultSeverity, FaultSite, FaultSource } from '../fault_codes'
import { SCOPE_CHANNELS } from '../scope_channels_generated'
import { useUsbDiagnostics, type UsbSample } from '../composables/useUsbDiagnostics'

const usb = useUsbDiagnostics()
const port = ref('')
const mode = ref(1)
const triggerType = ref(1)
const triggerChannel = ref(11)
const triggerState = ref(8)
const triggerEdge = ref(0)
const sampleRate = ref(10000)
const decimation = ref(1)
const preSamples = ref(64)
const postSamples = ref(128)
const threshold = ref(0)
const busy = ref(false)
const paused = ref(false)
const zoom = ref(1)
const frozenSamples = ref<UsbSample[]>([])
const testMotion = ref(false)
const hardwareEstopReady = ref(false)
const testVelocityLimit = ref(0.5)
const testCurrentLimit = ref(0.5)
const testTorqueLimit = ref(0.1)

const chartColors = ['#2dd4bf', '#60a5fa', '#f59e0b', '#f472b6', '#a78bfa', '#84cc16', '#fb7185', '#22d3ee']
const plottedSamples = computed(() => paused.value ? frozenSamples.value : usb.samples.value)
const visibleSamples = computed(() => {
  const count = Math.max(20, Math.floor(600 / zoom.value))
  return plottedSamples.value.slice(-count)
})

const selected = (id: number) => usb.selectedChannels.value.includes(id)
function toggleChannel(id: number) {
  const values = new Set(usb.selectedChannels.value)
  if (values.has(id)) values.delete(id); else if (values.size < 8) values.add(id)
  usb.selectedChannels.value = [...values]
}
async function run(action: () => Promise<unknown>) {
  busy.value = true
  try { await action() } catch (error) { usb.error.value = String(error) } finally { busy.value = false }
}
async function configure() {
  await run(() => usb.configureScope({
    mode: mode.value, trigger_type: triggerType.value,
    trigger_channel: triggerType.value === 4 ? triggerChannel.value : 0xffff,
    trigger_edge: triggerEdge.value, trigger_state: triggerState.value,
    sample_rate_hz: sampleRate.value, decimation: decimation.value,
    pre_samples: mode.value ? preSamples.value : 0,
    post_samples: mode.value ? postSamples.value : 0,
    channel_ids: usb.selectedChannels.value, threshold: threshold.value,
  }))
}
function togglePause() {
  if (!paused.value) frozenSamples.value = [...usb.samples.value]
  paused.value = !paused.value
}
function stateName(state: number | undefined) {
  return ['IDLE', 'CONFIGURED', 'ARMED', 'COMPLETE', 'STOPPED', 'ERROR'][state ?? 0] ?? 'UNKNOWN'
}
function triggerName(value: number) { return ['none', 'manual', 'fault', 'state', 'threshold'][value] ?? 'unknown' }
function cycleDelta(value: number, base: number) {
  return value >= base ? value - base : 0x100000000 - base + value
}
function chartPoints(channelId: number) {
  const samples = visibleSamples.value
  const channel = SCOPE_CHANNELS.find((item) => item.id === channelId)
  if (!samples.length || !channel) return ''
  const base = samples[0].timestamp_cycles >>> 0
  const span = Math.max(1, cycleDelta(samples[samples.length - 1].timestamp_cycles >>> 0, base))
  const range = Math.max(Number.EPSILON, channel.displayMax - channel.displayMin)
  return samples.map((sample) => {
    const x = cycleDelta(sample.timestamp_cycles >>> 0, base) / span * 600
    const raw = Number(sample.values[channelId] ?? 0)
    const normalized = Math.max(0, Math.min(1, (raw - channel.displayMin) / range))
    return `${x.toFixed(2)},${(110 - normalized * 100).toFixed(2)}`
  }).join(' ')
}
function enumName(table: Record<string, number>, value: number) {
  return Object.entries(table).find(([, candidate]) => candidate === value)?.[0] ?? `UNKNOWN(${value})`
}
function hex(value: number | undefined) {
  return `0x${Number(value ?? 0).toString(16).padStart(8, '0')}`
}
function faultKind(value: number) {
  return ['UNKNOWN', 'HardFault', 'MemManage', 'BusFault', 'UsageFault'][value] ?? `Fault(${value})`
}
function testSessionState(value: number | undefined) {
  return ['INACTIVE', 'ACTIVE', 'STOPPING', 'EXPIRED'][value ?? 0] ?? 'UNKNOWN'
}
async function beginTestSession() {
  await run(() => usb.beginTestSession({
    motion: testMotion.value,
    hardware_estop_confirmed: hardwareEstopReady.value,
    lease_ms: 2000,
    velocity_limit: testVelocityLimit.value,
    current_limit: testCurrentLimit.value,
    torque_limit: testTorqueLimit.value,
  }))
}
</script>

<template>
  <section class="panel usb-diagnostics">
    <div class="panel-head diag-head">
      <span class="title">USB Diagnostics</span>
      <span class="pill" :class="usb.connected.value ? 'ok' : 'dim'">USB {{ usb.connected.value ? 'connected' : 'offline' }}</span>
      <span class="pill" :class="usb.handshakeComplete.value ? 'ok' : usb.error.value ? 'err' : 'warn'">{{ usb.handshakeComplete.value ? 'HELLO OK' : usb.error.value ? 'HELLO failed' : 'HELLO pending' }}</span>
      <span class="spacer"></span>
      <input v-model="port" class="port-input" placeholder="COM port" aria-label="USB serial port" />
      <button v-if="!usb.connected.value" class="primary mini" @click="run(() => usb.connect(port))">连接 USB</button>
      <button v-else class="danger mini" @click="run(usb.disconnect)">断开 USB</button>
    </div>
    <div v-if="usb.error.value" class="diag-error">{{ usb.error.value }}</div>

    <div class="diag-grid">
      <div class="scope-config">
        <div class="subhead">Oscilloscope</div>
        <div class="config-grid">
          <label>模式<select v-model.number="mode"><option :value="0">continuous</option><option :value="1">triggered</option></select></label>
          <label>触发<select v-model.number="triggerType"><option :value="0">none</option><option :value="1">manual</option><option :value="2">fault</option><option :value="3">state</option><option :value="4">threshold</option></select></label>
          <label>采样 Hz<input v-model.number="sampleRate" type="number" min="1" max="10000" /></label>
          <label>decimation<input v-model.number="decimation" type="number" min="1" max="65535" /></label>
          <label>pre<input v-model.number="preSamples" type="number" min="0" max="64" /></label>
          <label>post<input v-model.number="postSamples" type="number" min="1" max="128" /></label>
          <label v-if="triggerType === 3">目标 state<input v-model.number="triggerState" type="number" min="0" max="255" /></label>
          <label v-if="triggerType === 4">触发通道<select v-model.number="triggerChannel"><option v-for="channel in SCOPE_CHANNELS.filter(item => item.thresholdAllowed)" :key="channel.id" :value="channel.id">{{ channel.name }}</option></select></label>
          <label v-if="triggerType === 4">threshold<input v-model.number="threshold" type="number" /></label>
        </div>
        <div class="channel-list">
          <button v-for="channel in SCOPE_CHANNELS" :key="channel.id" class="channel-chip" :class="{ on: selected(channel.id) }" @click="toggleChannel(channel.id)">
            <span>{{ channel.name }}</span><small>{{ channel.unit }}</small>
          </button>
        </div>
        <div class="row diag-actions">
          <button class="primary" :disabled="!usb.handshakeComplete.value || busy" @click="configure">应用配置</button>
          <button class="ok" :disabled="!usb.handshakeComplete.value || busy" @click="run(usb.armScope)">arm</button>
          <button class="warn" :disabled="!usb.handshakeComplete.value || busy" @click="run(usb.manualTrigger)">manual trigger</button>
          <button class="danger" :disabled="!usb.handshakeComplete.value || busy" @click="run(usb.stopScope)">stop</button>
          <button class="mini" @click="togglePause">{{ paused ? '继续' : '暂停' }}</button>
          <button class="mini" :disabled="zoom <= 1" @click="zoom = Math.max(1, zoom / 2)">−</button>
          <span class="zoom-label">{{ zoom }}×</span>
          <button class="mini" :disabled="zoom >= 16" @click="zoom = Math.min(16, zoom * 2)">+</button>
          <button class="mini" @click="usb.clearScope">clear</button>
          <button class="mini" @click="usb.downloadCsv">CSV</button>
          <button class="mini" @click="run(usb.downloadBinary)">binary</button>
          <button class="mini" @click="run(usb.downloadReplay)">replay</button>
        </div>
        <div class="scope-meta" v-if="usb.scopeStatus.value">
          capture #{{ usb.scopeStatus.value.capture_id }} · {{ stateName(usb.scopeStatus.value.state) }} · {{ triggerName(usb.scopeStatus.value.trigger_type) }} · firmware t={{ usb.scopeStatus.value.first_timestamp_cycles }}…{{ usb.scopeStatus.value.last_timestamp_cycles }} · gaps={{ usb.scopeStatus.value.sequence_gaps }} · dropped={{ usb.scopeStatus.value.dropped_samples }}
        </div>
        <div class="scope-legend">
          <span v-for="(channelId, index) in usb.selectedChannels.value" :key="channelId" :style="{ color: chartColors[index % chartColors.length] }">
            {{ usb.channelById.value.get(channelId)?.name }} [{{ usb.channelById.value.get(channelId)?.unit }}]
          </span>
        </div>
        <div class="scope-chart" aria-label="firmware timestamp aligned scope">
          <svg viewBox="0 0 600 120" preserveAspectRatio="none">
            <line v-for="y in [10, 35, 60, 85, 110]" :key="y" x1="0" :y1="y" x2="600" :y2="y" class="grid-line" />
            <polyline v-for="(channel, index) in usb.selectedChannels.value" :key="channel" :points="chartPoints(channel)" fill="none" :stroke="chartColors[index % chartColors.length]" stroke-width="1.4" vector-effect="non-scaling-stroke" />
          </svg>
          <span v-if="!visibleSamples.length">等待固件时间戳采样；不使用 WebSocket 到达时间。</span>
        </div>
      </div>

      <div class="diag-side">
        <div class="subhead">State Timeline</div>
        <div class="timeline scroll">
          <div v-for="event in usb.timeline.value.slice(-12).reverse()" :key="`${event.sequence}-${event.timestamp_cycles}`" class="timeline-row">
            <code>t={{ event.timestamp_cycles }}</code><span>S{{ event.state }} / O{{ event.operation }}</span><small>epoch {{ event.state_epoch }} · req {{ event.request_id }}</small>
          </div>
          <div v-if="!usb.timeline.value.length" class="empty-hint">等待 STATE_EVENT；时间轴使用固件 timestamp 和 state epoch。</div>
        </div>
        <div class="subhead fault-title">Crash Recorder</div>
        <div class="crash-list scroll">
          <div v-for="crash in usb.crashes.value.slice().reverse()" :key="`${crash.generation}-${crash.record_crc}`" class="crash-row">
            <strong>{{ faultKind(crash.fault_kind) }} · generation {{ crash.generation }}</strong>
            <small>PC {{ hex(crash.stacked.pc) }} · LR {{ hex(crash.stacked.lr) }} · IRQ {{ crash.active_irq }}</small>
            <small>CFSR {{ hex(crash.fault_status.cfsr) }} · HFSR {{ hex(crash.fault_status.hfsr) }}</small>
            <small>state {{ crash.safety_state }}/{{ crash.operation }} · epoch {{ crash.state_epoch }} · control {{ crash.control_sequence }}</small>
            <template v-if="crash.symbolization?.reliable">
              <small class="resolved">PC {{ crash.symbolization.locations?.pc?.function }}<br />{{ crash.symbolization.locations?.pc?.file }}:{{ crash.symbolization.locations?.pc?.line }}</small>
              <small>LR {{ crash.symbolization.locations?.lr?.function }}<br />{{ crash.symbolization.locations?.lr?.file }}:{{ crash.symbolization.locations?.lr?.line }}</small>
            </template>
            <small v-else class="unresolved">不可可靠符号化：{{ crash.symbolization?.error ?? '等待匹配 ELF' }}</small>
            <small>build {{ crash.record_build_id }} · CRC {{ hex(crash.record_crc) }}</small>
          </div>
          <div v-if="!usb.crashes.value.length" class="empty-hint">暂无复位后 Crash Record。</div>
        </div>
        <div class="subhead fault-title">Fault Inspector</div>
        <div class="fault-list scroll">
          <div v-for="fault in usb.faults.value.slice(-8).reverse()" :key="fault.fault_sequence" class="fault-row">
            <strong>fault #{{ fault.fault_sequence }} · {{ enumName(FaultCode, fault.code) }}</strong>
            <small>{{ enumName(FaultSource, fault.source) }} / {{ enumName(FaultSite, fault.site) }} / {{ enumName(FaultSeverity, fault.severity) }}</small>
            <small>epoch {{ fault.state_epoch }} · control seq {{ fault.control_sequence }} · t={{ fault.timestamp_cycles }}</small>
            <small>parent {{ fault.parent_fault_sequence }} · arg0/1/2 {{ fault.arg0 }}/{{ fault.arg1 }}/{{ fault.arg2 }}</small>
          </div>
          <div v-if="!usb.faults.value.length" class="empty-hint">暂无 USB fault event。</div>
        </div>
        <div class="subhead">Test Session</div>
        <div class="config-grid session-config">
          <label><input v-model="testMotion" type="checkbox" /> motion ownership</label>
          <label><input v-model="hardwareEstopReady" type="checkbox" /> hardware estop ready</label>
          <label>velocity rev/s<input v-model.number="testVelocityLimit" type="number" min="0.01" max="2" step="0.01" /></label>
          <label>current A<input v-model.number="testCurrentLimit" type="number" min="0.01" max="1" step="0.01" /></label>
          <label>torque Nm<input v-model.number="testTorqueLimit" type="number" min="0.01" max="0.2" step="0.01" /></label>
        </div>
        <div class="row diag-actions">
          <button class="primary mini" :disabled="!usb.handshakeComplete.value || busy || (testMotion && !hardwareEstopReady)" @click="beginTestSession">begin</button>
          <button class="danger mini" :disabled="usb.testSession.value?.state !== 1 || busy" @click="run(usb.stopTestSession)">safe stop</button>
          <button class="mini" :disabled="usb.testSession.value?.state !== 1 || !usb.testSession.value?.stop_confirmed || busy" @click="run(usb.endTestSession)">end</button>
        </div>
        <div class="session-meta">state {{ testSessionState(usb.testSession.value?.state) }} / id {{ usb.testSession.value?.session_id ?? 0 }} / lease {{ usb.testSession.value?.lease_remaining_ms ?? 0 }} ms<br />connected {{ usb.session.value.connectedAt || '--' }}<br />build {{ usb.session.value.buildId || '--' }}<br />schema {{ usb.session.value.schemaVersion || '--' }} · samples {{ usb.samples.value.length }}/2000<br />config {{ usb.session.value.captureConfig ? 'ACK applied' : 'not applied' }}</div>
      </div>
    </div>
  </section>
</template>

<style scoped>
.usb-diagnostics { min-height: 0; display: flex; flex-direction: column; gap: 10px; overflow: hidden; }
.diag-head { display: flex; align-items: center; gap: 9px; flex-wrap: wrap; }
.title { color: var(--fg); font-weight: 750; }
.spacer { flex: 1; }
.mini { padding: 4px 9px; font-size: 11px; }
.port-input { width: 154px; min-height: 30px; padding: 5px 8px; }
.diag-error { color: #fda4af; background: rgba(251,113,133,.1); border: 1px solid rgba(251,113,133,.25); padding: 5px 7px; border-radius: 6px; }
.diag-grid { display: grid; grid-template-columns: minmax(0, 1.6fr) minmax(250px, .85fr); gap: 14px; min-height: 0; overflow: hidden; }
.scope-config, .diag-side { min-width: 0; min-height: 0; overflow: auto; padding-right: 4px; }
.subhead { color: var(--fg-muted); text-transform: uppercase; letter-spacing: .1em; font-size: 11px; font-weight: 750; margin-bottom: 8px; }
.config-grid { display: grid; grid-template-columns: repeat(4, minmax(112px, 1fr)); gap: 8px; }
.config-grid label { display: grid; gap: 3px; }
.config-grid input, .config-grid select { min-height: 30px; padding: 4px 6px; font-size: 12px; }
.session-config { grid-template-columns: repeat(2, minmax(110px, 1fr)); margin-bottom: 8px; }
.session-config label:has(input[type="checkbox"]) { display: flex; align-items: center; gap: 6px; }
.channel-list { display: grid; grid-template-columns: repeat(auto-fit, minmax(132px, 1fr)); gap: 6px; margin: 9px 0; }
.channel-chip { padding: 6px 7px; font-size: 11px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.channel-chip small { color: var(--fg-muted); margin-left: 3px; }
.channel-chip.on { border-color: var(--accent-2); color: var(--fg); background: rgba(45,212,191,.13); }
.diag-actions { flex-wrap: wrap; }
.zoom-label { color: var(--fg-dim); font: 10px var(--mono); }
.scope-meta, .session-meta { color: var(--fg-dim); font: 11px/1.65 var(--mono); }
.scope-legend { display: flex; flex-wrap: wrap; gap: 9px; margin-top: 7px; font: 10px var(--mono); }
.scope-chart { position: relative; min-height: 145px; margin-top: 8px; background: #020617; border: 1px solid var(--border); border-radius: 7px; overflow: hidden; color: var(--fg-muted); display: grid; place-items: center; font-size: 11px; }
.scope-chart svg { position: absolute; inset: 0; width: 100%; height: 100%; }
.scope-chart span { position: relative; background: rgba(2,6,23,.7); padding: 4px; }
.grid-line { stroke: rgba(148, 163, 184, .12); stroke-width: 1; vector-effect: non-scaling-stroke; }
.diag-side { min-height: 0; display: flex; flex-direction: column; }
.timeline, .fault-list, .crash-list { min-height: 72px; max-height: 180px; overflow: auto; }
.timeline-row, .fault-row, .crash-row { display: grid; gap: 3px; padding: 6px 0; border-bottom: 1px solid var(--border-subtle); font-size: 11px; }
.timeline-row code, .fault-row small, .crash-row small { color: var(--fg-dim); font: 11px var(--mono); overflow-wrap: anywhere; }
.crash-row .resolved { color: #6ee7b7; }
.crash-row .unresolved { color: #fcd56b; }
.fault-title { margin-top: 10px; }
@media (max-width: 900px) { .diag-grid { grid-template-columns: 1fr; overflow: auto; } .scope-config, .diag-side { overflow: visible; } .channel-list { grid-template-columns: repeat(3, minmax(0, 1fr)); } }
</style>
