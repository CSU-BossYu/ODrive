<script setup lang="ts">
// Anticogging status and configuration panel.
//
// Fetches anticogging status via extended commands (sub_cmd 0x08)
// and allows configuration (sub_cmd 0x09).
//
// Three sections:
//   1. Status (read-only): flags, calib_index, thresholds, cogging_ratio
//   2. Config (writable): enabled, pre_calibrated, pos/vel thresholds, reset
//   3. Calibration: disabled with warning (position following not fully validated)

import { ref, watch, onMounted, computed } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'
import type { ODriveExtRespMsg } from '../types'

const oSocket = useOdriveSocket()

// Status fields (read from GET_ANTICOGGING_STATUS responses)
const statusFlags = ref(0)
const calibIndex = ref(0)
const posThreshold = ref(0)
const velThreshold = ref(0)
const coggingRatio = ref(0)
const systemError = ref(0)

// Derived flags from statusFlags
const isCalibrating = computed(() => !!(statusFlags.value & 0x01))
const isValid = computed(() => !!(statusFlags.value & 0x02))
const isPreCalibrated = computed(() => !!(statusFlags.value & 0x04))
const isEnabled = computed(() => !!(statusFlags.value & 0x08))

// Config fields (editable)
const cfgEnabled = ref(false)
const cfgPreCalibrated = ref(false)
const cfgPosThreshold = ref(1.0)
const cfgVelThreshold = ref(1.0)

// Feedback
const lastResult = ref<string>('')

function refreshStatus() {
  oSocket.anticoggingStatus()
  lastResult.value = 'Requesting status...'
}

function applyConfig(field: string) {
  const cfg: Record<string, unknown> = {}
  switch (field) {
    case 'enabled':
      cfg.enabled = cfgEnabled.value
      break
    case 'pre_calibrated':
      cfg.pre_calibrated = cfgPreCalibrated.value
      break
    case 'pos_threshold':
      cfg.pos_threshold = cfgPosThreshold.value
      break
    case 'vel_threshold':
      cfg.vel_threshold = cfgVelThreshold.value
      break
    case 'reset':
      cfg.reset = true
      break
  }
  oSocket.anticoggingConfig(cfg as any)
  lastResult.value = `Set ${field} sent`
}

// Watch extSeq (monotonic counter) instead of extResponses.length: once the
// ring buffer fills, push+shift keeps length constant so a length watch stops
// firing and the panel freezes. Process only the latest response on each fire.
watch(() => oSocket.extSeq.value, () => {
  const responses = oSocket.extResponses.value
  const r = responses[responses.length - 1]
  if (!r || r.sub_cmd !== 0x08) return  // 0x08 = GET_ANTICOGGING_STATUS
  switch (r.item) {
    case 0x01: statusFlags.value = r.value; break
    case 0x02: calibIndex.value = r.value; break
    case 0x03: posThreshold.value = r.value; break
    case 0x04: velThreshold.value = r.value; break
    case 0x05: coggingRatio.value = r.value; break
    case 0x06: systemError.value = r.value; break
  }
})

onMounted(() => {
  // Fetch status on mount
  refreshStatus()
})
</script>

<template>
  <div class="panel anticog-panel">
    <div class="row">
      <h3 class="panel-title grow">Anticogging</h3>
      <button @click="refreshStatus" title="Refresh status">⟳</button>
    </div>

    <!-- Status section -->
    <div class="section">
      <div class="section-label">Status</div>
      <div class="flag-grid">
        <span class="flag" :class="{ on: isEnabled }">
          {{ isEnabled ? '✓' : '✗' }} Enabled
        </span>
        <span class="flag" :class="{ on: isValid }">
          {{ isValid ? '✓' : '✗' }} Valid
        </span>
        <span class="flag" :class="{ on: isPreCalibrated }">
          {{ isPreCalibrated ? '✓' : '✗' }} Pre-calibrated
        </span>
        <span class="flag warn" :class="{ on: isCalibrating }">
          {{ isCalibrating ? '●' : '○' }} Calibrating
        </span>
      </div>
      <div class="stat-row">
        <span class="stat-label">Index</span>
        <span class="stat-value">{{ calibIndex }}</span>
        <span class="stat-label">Ratio</span>
        <span class="stat-value">{{ coggingRatio.toFixed(4) }}</span>
      </div>
      <div class="stat-row">
        <span class="stat-label">Pos thr</span>
        <span class="stat-value">{{ posThreshold.toFixed(4) }}</span>
        <span class="stat-label">Vel thr</span>
        <span class="stat-value">{{ velThreshold.toFixed(4) }}</span>
      </div>
      <div v-if="systemError" class="stat-row err-row">
        <span class="stat-label">Sys Error</span>
        <span class="stat-value err-val">0x{{ systemError.toString(16).padStart(8, '0') }}</span>
      </div>
    </div>

    <!-- Config section -->
    <div class="section">
      <div class="section-label">Configuration</div>
      <div class="cfg-row">
        <label class="cfg-check">
          <input type="checkbox" v-model="cfgEnabled" @change="applyConfig('enabled')" />
          Anticogging enabled
        </label>
      </div>
      <div class="cfg-row">
        <label class="cfg-check">
          <input type="checkbox" v-model="cfgPreCalibrated" @change="applyConfig('pre_calibrated')" />
          Pre-calibrated (map valid)
        </label>
      </div>
      <div class="cfg-field">
        <label>Pos threshold (counts)</label>
        <div class="row">
          <input type="number" step="0.1" v-model.number="cfgPosThreshold" />
          <button @click="applyConfig('pos_threshold')">set</button>
        </div>
      </div>
      <div class="cfg-field">
        <label>Vel threshold (counts/s)</label>
        <div class="row">
          <input type="number" step="0.1" v-model.number="cfgVelThreshold" />
          <button @click="applyConfig('vel_threshold')">set</button>
        </div>
      </div>
      <button @click="applyConfig('reset')" class="warn">Reset Calibration</button>
    </div>

    <!-- Calibration (disabled) -->
    <div class="section">
      <div class="section-label">Calibration</div>
      <button disabled class="cal-btn" title="Position following not fully validated">
        Start Anticogging Calibration
      </button>
      <div class="warning-text">
        ⚠ Calibration disabled: position following has not been fully validated.
        Use ODrive CLI tools for calibration.
      </div>
    </div>

    <div v-if="lastResult" class="result-msg">{{ lastResult }}</div>
  </div>
</template>

<style scoped>
.anticog-panel {
  display: flex; flex-direction: column; gap: 6px;
  overflow-y: auto; padding: 8px;
}
.section {
  border-top: 1px solid var(--border);
  padding-top: 6px;
  display: flex; flex-direction: column; gap: 4px;
}
.section-label {
  font-size: 10px; color: var(--fg-dim); text-transform: uppercase;
  letter-spacing: 0.05em;
}
.flag-grid {
  display: grid; grid-template-columns: 1fr 1fr; gap: 2px;
}
.flag {
  font-size: 10px; font-family: var(--mono); color: var(--fg-dim);
  padding: 1px 4px;
}
.flag.on { color: var(--ok); }
.flag.warn.on { color: var(--warn); }
.stat-row {
  display: flex; gap: 6px; align-items: baseline;
  font-family: var(--mono); font-size: 10px;
}
.stat-label { color: var(--fg-dim); }
.stat-value { color: var(--fg); font-weight: 600; }
.err-row .err-val { color: var(--err); }
.cfg-row { padding: 2px 0; }
.cfg-check {
  display: flex; align-items: center; gap: 4px;
  font-size: 11px; color: var(--fg); cursor: pointer;
}
.cfg-check input { width: auto; }
.cfg-field {
  display: flex; flex-direction: column; gap: 2px;
}
.cfg-field label { font-size: 10px; color: var(--fg-dim); }
.cfg-field .row { gap: 4px; }
.cfg-field input { flex: 1; }
.cal-btn {
  width: 100%; padding: 6px;
  opacity: 0.4; cursor: not-allowed;
}
.warning-text {
  font-size: 9px; color: var(--warn); line-height: 1.4;
  padding: 2px 4px; background: rgba(245, 158, 11, 0.08);
  border-radius: 3px;
}
.result-msg {
  font-size: 10px; font-family: var(--mono); color: var(--accent-2);
  padding: 2px 0;
}
</style>
