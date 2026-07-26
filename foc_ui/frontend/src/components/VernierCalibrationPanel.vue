<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const SUB_BASIC = 0x06
const SUB_SAVE = 0x03
const SUB_VERNIER_CAL = 0x0D
const EXT_UINT32 = 3

const ITEM_RESET = 0x00
const ITEM_CAPTURE = 0x01
const ITEM_FIT = 0x02
const ITEM_APPLY = 0x03
const ITEM_POINT_COUNT = 0x04
const ITEM_FIT_VALID = 0x05
const ITEM_FITTED_MAIN = 0x06
const ITEM_FITTED_AUX = 0x07
const ITEM_SCORE = 0x08
const ITEM_WORST = 0x09
const ITEM_MAX_POINTS = 0x0A

const BASIC_MAIN_OFFSET = 0x28
const BASIC_AUX_OFFSET = 0x29

const oSocket = useOdriveSocket()
const targetPoints = ref(5)
const searchRadius = ref(0.05)
const notice = ref('')
const noticeKind = ref<'dim' | 'ok' | 'warn' | 'err'>('dim')

const autoPointCount = ref(12)
const autoSweepTurns = ref(0.04)
const autoSearchRadius = ref(0.05)
const autoSettlePosTol = ref(0.002)
const autoRampVel = ref(0.005)
const autoSweepCycles = ref(2)

function latestExtResponse(subCmd: number, item: number) {
  for (let i = oSocket.extResponses.value.length - 1; i >= 0; i--) {
    const r = oSocket.extResponses.value[i]
    if (r.sub_cmd === subCmd && r.item === item) return r
  }
  return null
}

const pointCount = computed(() => latestExtResponse(SUB_VERNIER_CAL, ITEM_POINT_COUNT)?.value ?? 0)
const maxPoints = computed(() => latestExtResponse(SUB_VERNIER_CAL, ITEM_MAX_POINTS)?.value ?? 16)
const fitValid = computed(() => (latestExtResponse(SUB_VERNIER_CAL, ITEM_FIT_VALID)?.value ?? 0) !== 0)
const fittedMain = computed(() => latestExtResponse(SUB_VERNIER_CAL, ITEM_FITTED_MAIN)?.value)
const fittedAux = computed(() => latestExtResponse(SUB_VERNIER_CAL, ITEM_FITTED_AUX)?.value)
const fitScore = computed(() => latestExtResponse(SUB_VERNIER_CAL, ITEM_SCORE)?.value)
const fitWorst = computed(() => latestExtResponse(SUB_VERNIER_CAL, ITEM_WORST)?.value)
const currentMain = computed(() => latestExtResponse(SUB_BASIC, BASIC_MAIN_OFFSET)?.value)
const currentAux = computed(() => latestExtResponse(SUB_BASIC, BASIC_AUX_OFFSET)?.value)
const capturedEnough = computed(() => pointCount.value >= 2)
const targetReached = computed(() => pointCount.value >= targetPoints.value)

const vernierRunning = computed(() => oSocket.vernierRunning.value)
const vernierProgress = computed(() => oSocket.vernierProgress.value)
const vernierResult = computed(() => oSocket.vernierResult.value)

function fmt(value: number | undefined, digits = 6) {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--'
}

function setNotice(text: string, kind: 'dim' | 'ok' | 'warn' | 'err' = 'dim') {
  notice.value = text
  noticeKind.value = kind
}

function refresh() {
  oSocket.extCmd(SUB_BASIC, BASIC_MAIN_OFFSET)
  oSocket.extCmd(SUB_BASIC, BASIC_AUX_OFFSET)
  ;[
    ITEM_POINT_COUNT,
    ITEM_FIT_VALID,
    ITEM_FITTED_MAIN,
    ITEM_FITTED_AUX,
    ITEM_SCORE,
    ITEM_WORST,
    ITEM_MAX_POINTS,
  ].forEach((item) => oSocket.extCmd(SUB_VERNIER_CAL, item))
}

function refreshSoon() {
  window.setTimeout(refresh, 180)
}

function resetPoints() {
  oSocket.vernierCalib(ITEM_RESET, 0, false)
  setNotice('Reset request sent', 'warn')
  refreshSoon()
}

function capturePoint() {
  oSocket.vernierCalib(ITEM_CAPTURE, 0, false)
  setNotice('Capture request sent', 'warn')
  refreshSoon()
}

function fitOffsets() {
  const radius = Number.isFinite(searchRadius.value) ? Math.max(0.001, Math.min(0.5, searchRadius.value)) : 0.05
  oSocket.vernierCalib(ITEM_FIT, radius, true)
  setNotice('Fit request sent', 'warn')
  refreshSoon()
}

function applyFit() {
  oSocket.vernierCalib(ITEM_APPLY, 0, false)
  setNotice('Apply-to-RAM request sent', 'warn')
  refreshSoon()
}

function saveConfig() {
  oSocket.extCmd(SUB_SAVE, 0x00, EXT_UINT32, 0, 3.0)
  setNotice('Saving configuration; controller will remain online', 'warn')
}

function startAutoSweep() {
  oSocket.vernierAutoCalibrate(
    autoPointCount.value,
    autoSweepTurns.value,
    autoSearchRadius.value,
    autoSettlePosTol.value,
    autoRampVel.value,
    autoSweepCycles.value,
  )
}

function cancelAutoSweep() {
  oSocket.vernierAutoCancel()
}

watch(() => oSocket.ready.value, (ready) => {
  if (ready) refresh()
}, { immediate: true })

watch(() => oSocket.extSeq.value, () => {
  const applyResp = latestExtResponse(SUB_VERNIER_CAL, ITEM_APPLY)
  if (applyResp?.status === 0) {
    setNotice('Fit applied to RAM; verify readings before saving', 'ok')
  } else if (applyResp && applyResp.status !== 0) {
    setNotice(`Apply failed, firmware status ${applyResp.status}`, 'err')
  }
  const fitResp = latestExtResponse(SUB_VERNIER_CAL, ITEM_FIT)
  if (fitResp?.status === 0 && fitValid.value) {
    setNotice('Fit complete; check residual before applying', 'ok')
  } else if (fitResp && fitResp.status !== 0) {
    setNotice(`Fit failed, firmware status ${fitResp.status}`, 'err')
  }
})

watch(() => oSocket.vernierResult.value, (r) => {
  if (r && r.ok) refresh()
})
</script>

<template>
  <div class="panel vernier-panel">
    <div class="panel-head">
      <div>
        <h3 class="panel-title">Vernier Calibration</h3>
        <div class="panel-sub">Firmware 0x0D multi-point static offset fit</div>
      </div>
      <button @click="refresh" :disabled="!oSocket.ready.value">Read</button>
    </div>

    <div class="read-grid">
      <div><span>current main</span><strong>{{ fmt(currentMain) }}</strong><em>turn</em></div>
      <div><span>current aux</span><strong>{{ fmt(currentAux) }}</strong><em>turn</em></div>
      <div><span>samples</span><strong>{{ pointCount }}</strong><em>/ {{ maxPoints }}</em></div>
      <div><span>manual target</span><strong>{{ targetPoints }}</strong><em>pts</em></div>
    </div>

    <div class="control-row">
      <label>manual target points
        <input type="number" min="2" max="16" step="1" v-model.number="targetPoints" />
      </label>
      <label>fit search radius
        <input type="number" min="0.001" max="0.5" step="0.001" v-model.number="searchRadius" />
      </label>
    </div>

    <div class="action-grid">
      <button @click="resetPoints" :disabled="!oSocket.ready.value">Reset</button>
      <button @click="capturePoint" class="primary" :disabled="!oSocket.ready.value || pointCount >= maxPoints">Capture</button>
      <button @click="fitOffsets" :disabled="!oSocket.ready.value || !capturedEnough" :class="{ warn: targetReached }">Fit</button>
      <button @click="applyFit" :disabled="!oSocket.ready.value || !fitValid" class="warn">Apply</button>
      <button @click="saveConfig" :disabled="!oSocket.ready.value" class="danger">Save</button>
    </div>

    <div class="fit-grid">
      <div><span>fit main</span><strong>{{ fmt(fittedMain) }}</strong><em>turn</em></div>
      <div><span>fit aux</span><strong>{{ fmt(fittedAux) }}</strong><em>turn</em></div>
      <div><span>RMS residual</span><strong>{{ fmt(fitScore) }}</strong><em>turn</em></div>
      <div><span>worst residual</span><strong>{{ fmt(fitWorst) }}</strong><em>turn</em></div>
    </div>

    <div class="auto-card">
      <div class="card-head"><span>Auto scan: slow back-and-forth sweep around current position</span></div>
      <div class="control-row">
        <label>capture points
          <input type="number" min="2" max="16" step="1" v-model.number="autoPointCount" :disabled="vernierRunning" />
        </label>
        <label>total range (turn)
          <input type="number" min="0.002" max="0.25" step="0.002" v-model.number="autoSweepTurns" :disabled="vernierRunning" />
        </label>
        <label>sweep cycles
          <input type="number" min="1" max="8" step="1" v-model.number="autoSweepCycles" :disabled="vernierRunning" />
        </label>
        <label>scan speed (turn/s)
          <input type="number" min="0.0005" max="0.1" step="0.0005" v-model.number="autoRampVel" :disabled="vernierRunning" />
        </label>
        <label>fit search radius
          <input type="number" min="0.001" max="0.5" step="0.001" v-model.number="autoSearchRadius" :disabled="vernierRunning" />
        </label>
        <label>settle tolerance (turn)
          <input type="number" min="0.0001" max="0.02" step="0.0001" v-model.number="autoSettlePosTol" :disabled="vernierRunning" />
        </label>
      </div>
      <div class="action-row">
        <button @click="startAutoSweep" class="primary" :disabled="!oSocket.ready.value || vernierRunning">Auto Scan</button>
        <button @click="cancelAutoSweep" :disabled="!vernierRunning">Cancel</button>
      </div>
      <div v-if="vernierRunning || vernierProgress" class="progress-track">
        <div class="progress-fill" :style="{ width: `${vernierProgress?.progress ?? 0}%` }"></div>
      </div>
      <div v-if="vernierProgress" class="progress-stage">{{ vernierProgress.stage }} ({{ vernierProgress.progress.toFixed(0) }}%)</div>
      <div v-if="vernierResult" class="vernier-message" :class="{ ok: vernierResult.ok, err: !vernierResult.ok }">
        <template v-if="vernierResult.ok">
          captured {{ vernierResult.captured }} pts; aux={{ vernierResult.fitted_aux.toFixed(6) }}, RMS={{ vernierResult.score.toFixed(6) }}, worst={{ vernierResult.worst.toFixed(6) }}. Apply to RAM, verify, then save.
        </template>
        <template v-else>{{ vernierResult.error }}</template>
      </div>
    </div>

    <div v-if="notice" class="vernier-message" :class="noticeKind">{{ notice }}</div>
  </div>
</template>

<style scoped>
.vernier-panel {
  display: flex;
  flex-direction: column;
  gap: 8px;
  min-height: 0;
  overflow-y: auto;
}
.panel-head {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 8px;
}
.panel-title {
  margin: 0;
}
.panel-sub {
  color: var(--muted);
  font-size: 12px;
}
.read-grid, .fit-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 6px;
}
.read-grid div, .fit-grid div {
  display: grid;
  grid-template-columns: 1fr auto auto;
  align-items: baseline;
  gap: 5px;
  padding: 7px 8px;
  border: 1px solid var(--border);
  border-radius: 6px;
  background: rgba(15, 23, 42, 0.55);
  min-width: 0;
}
.read-grid span, .fit-grid span {
  color: var(--muted);
  font-size: 12px;
}
.read-grid strong, .fit-grid strong {
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 13px;
}
.read-grid em, .fit-grid em {
  color: var(--muted);
  font-size: 11px;
  font-style: normal;
}
.control-row {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
}
.control-row label {
  display: grid;
  gap: 3px;
  color: var(--muted);
  font-size: 12px;
}
.control-row input {
  width: 100%;
}
.action-grid {
  display: grid;
  grid-template-columns: repeat(5, minmax(0, 1fr));
  gap: 6px;
}
.action-grid button {
  min-width: 0;
  padding-inline: 7px;
}
.vernier-message {
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 7px 8px;
  font-size: 12px;
  color: var(--muted);
}
.vernier-message.ok {
  color: #86efac;
  border-color: rgba(34, 197, 94, 0.35);
}
.vernier-message.warn {
  color: #fde68a;
  border-color: rgba(245, 158, 11, 0.35);
}
.vernier-message.err {
  color: #fecaca;
  border-color: rgba(239, 68, 68, 0.35);
}
.auto-card {
  display: flex;
  flex-direction: column;
  gap: 6px;
  padding: 8px;
  border: 1px solid var(--border);
  border-radius: 6px;
  background: rgba(15, 23, 42, 0.4);
}
.auto-card .card-head {
  font-size: 12px;
  color: var(--muted);
}
.action-row {
  display: flex;
  gap: 6px;
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
.progress-stage {
  font-size: 10px;
  color: var(--muted);
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
}
@media (max-width: 1280px) {
  .action-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
}
</style>
