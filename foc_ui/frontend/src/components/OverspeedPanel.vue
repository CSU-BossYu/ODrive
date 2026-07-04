<script setup lang="ts">
// OverspeedSnapshot viewer.
//
// Firmware captures the full controller/encoder/resolver state at the instant
// of ERROR_OVERSPEED and holds it until clear_errors. This panel fetches that
// snapshot via the `get_overspeed_snapshot` WS command (ext 0x0A items
// 0x40-0x5F) and displays the key fields so you can see exactly which value
// tripped and what the resolver/setpoints looked like at the fault.

import { computed } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const oSocket = useOdriveSocket()

const snap = computed(() => oSocket.overspeedSnapshot.value)
const valid = computed(() => !!snap.value && !!snap.value.valid)
const fetching = computed(() => snap.value === null)

const CONTROL_MODES: Record<number, string> = {
  0: 'VOLTAGE', 1: 'TORQUE', 2: 'VELOCITY', 3: 'POSITION',
}
const INPUT_MODES: Record<number, string> = {
  0: 'INACTIVE', 1: 'PASSTHROUGH', 2: 'VEL_RAMP', 3: 'POS_FILTER',
  4: 'MIX', 5: 'TRAP_TRAJ', 6: 'TORQUE_RAMP', 8: 'TUNING', 9: 'MIT',
}
const RESOLVER_STATES: Record<number, string> = {
  0: 'UNINIT', 1: 'ACQUIRING', 2: 'LOCKED', 3: 'SUSPECT', 4: 'ERROR',
}

function f(v: number | null | undefined, d = 4): string {
  return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '–'
}
function modeName(table: Record<number, string>, v: number | null | undefined): string {
  if (typeof v !== 'number') return '–'
  return `${v} (${table[v] ?? '?'})`
}
function bool(v: number | null | undefined): string {
  return v ? '1' : '0'
}

// vel_estimate vs the overspeed threshold (vel_limit * vel_limit_tolerance)
const overThreshold = computed(() => {
  const s = snap.value
  if (!s) return null
  const ve = s.vel_estimate, vl = s.vel_limit, vt = s.vel_limit_tolerance
  if (typeof ve !== 'number' || typeof vl !== 'number' || typeof vt !== 'number') return null
  return vl * vt
})
</script>

<template>
  <div class="panel overspeed-panel">
    <div class="row">
      <h3 class="panel-title grow">Overspeed Snapshot</h3>
      <button @click="oSocket.getOverspeedSnapshot()" :disabled="!oSocket.connected.value">
        {{ fetching ? 'Read' : 'Refresh' }}
      </button>
    </div>

    <div v-if="!snap" class="hint">Click Read to fetch the last overspeed-fault snapshot.</div>
    <div v-else-if="!valid" class="hint">No overspeed fault captured (snapshot invalid). Trigger an overspeed fault first; it is held until Clear Errors.</div>
    <div v-else class="snap">
      <div class="section">
        <div class="section-label">Fault instant</div>
        <div class="grid">
          <span class="k">loop#</span><span class="v">{{ snap.control_loop_count }}</span>
          <span class="k">t (s)</span><span class="v">{{ f(snap.timestamp, 3) }}</span>
        </div>
      </div>

      <div class="section">
        <div class="section-label">Overspeed (the trip)</div>
        <div class="grid">
          <span class="k">vel_estimate</span><span class="v hl" :class="{ bad: overThreshold !== null && typeof snap.vel_estimate === 'number' && Math.abs(snap.vel_estimate) > (overThreshold ?? 0) }">{{ f(snap.vel_estimate) }}</span>
          <span class="k">vel_limit</span><span class="v">{{ f(snap.vel_limit) }}</span>
          <span class="k">×tolerance</span><span class="v">{{ f(snap.vel_limit_tolerance, 2) }}</span>
          <span class="k">threshold</span><span class="v">{{ overThreshold !== null ? f(overThreshold) : '–' }}</span>
        </div>
      </div>

      <div class="section">
        <div class="section-label">Setpoints / inputs</div>
        <div class="grid">
          <span class="k">pos_setpoint</span><span class="v">{{ f(snap.pos_setpoint) }}</span>
          <span class="k">vel_setpoint</span><span class="v">{{ f(snap.vel_setpoint) }}</span>
          <span class="k">torque_setpoint</span><span class="v">{{ f(snap.torque_setpoint, 5) }}</span>
          <span class="k">input_pos</span><span class="v">{{ f(snap.input_pos) }}</span>
          <span class="k">input_vel</span><span class="v">{{ f(snap.input_vel) }}</span>
          <span class="k">input_torque</span><span class="v">{{ f(snap.input_torque, 5) }}</span>
          <span class="k">ctrl_mode</span><span class="v">{{ modeName(CONTROL_MODES, snap.control_mode) }}</span>
          <span class="k">input_mode</span><span class="v">{{ modeName(INPUT_MODES, snap.input_mode) }}</span>
        </div>
      </div>

      <div class="section">
        <div class="section-label">Position / encoder</div>
        <div class="grid">
          <span class="k">pos_est_linear</span><span class="v">{{ f(snap.pos_estimate_linear) }}</span>
          <span class="k">pos_est_circular</span><span class="v">{{ f(snap.pos_estimate_circular) }}</span>
          <span class="k">enc_pos_est</span><span class="v">{{ f(snap.encoder_pos_estimate) }}</span>
          <span class="k">enc_vel_est</span><span class="v">{{ f(snap.encoder_vel_estimate) }}</span>
        </div>
      </div>

      <div class="section">
        <div class="section-label">Vernier resolver</div>
        <div class="grid">
          <span class="k">state</span><span class="v">{{ modeName(RESOLVER_STATES, snap.resolver_state) }}</span>
          <span class="k">valid/locked</span><span class="v">{{ bool(snap.resolver_valid) }}/{{ bool(snap.resolver_locked) }}</span>
          <span class="k">aux/degraded</span><span class="v">{{ bool(snap.resolver_accepted_aux) }}/{{ bool(snap.resolver_degraded) }}</span>
          <span class="k">pos_turns</span><span class="v">{{ f(snap.resolver_position_turns) }}</span>
          <span class="k">residual</span><span class="v">{{ f(snap.resolver_residual, 5) }}</span>
        </div>
      </div>

      <div class="section">
        <div class="section-label">Output-shaft estimate</div>
        <div class="grid">
          <span class="k">valid</span><span class="v">{{ bool(snap.output_estimate_valid) }}</span>
          <span class="k">out_pos</span><span class="v">{{ f(snap.output_pos_estimate) }}</span>
          <span class="k">out_vel</span><span class="v">{{ f(snap.output_vel_estimate) }}</span>
          <span class="k">sample_dt</span><span class="v">{{ f(snap.output_sample_dt, 6) }}</span>
          <span class="k">pair_seq</span><span class="v">{{ snap.output_pair_sequence ?? '–' }}</span>
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.overspeed-panel { display: flex; flex-direction: column; gap: 6px; overflow-y: auto; padding: 8px; }
.row { display: flex; align-items: center; gap: 6px; }
.grow { flex: 1; }
.hint { font-size: 10px; color: var(--fg-dim); line-height: 1.4; padding: 2px 4px; }
.section { border-top: 1px solid var(--border); padding-top: 4px; }
.section-label { font-size: 10px; color: var(--fg-dim); text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 2px; }
.grid {
  display: grid; grid-template-columns: 110px 1fr; gap: 1px 6px;
  font-family: var(--mono); font-size: 10px;
}
.grid .k { color: var(--fg-dim); }
.grid .v { color: var(--fg); font-weight: 600; word-break: break-all; }
.grid .v.hl { color: var(--accent-2); }
.grid .v.bad { color: var(--err); }
</style>
