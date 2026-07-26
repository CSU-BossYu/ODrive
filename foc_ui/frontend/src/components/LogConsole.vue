<script setup lang="ts">
// Log + raw CAN frame console (one card). Log tab shows text log lines; CAN
// tab shows actually sent/received frames as two-column rows
// (raw frame | decoded meaning). A "hide periodic" switch suppresses the
// high-rate telemetry/heartbeat frames that flood the view.

import { computed, nextTick, ref, watch } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'
import type { CanFrameDef } from '../types'

const oSocket = useOdriveSocket()

const containerEl = ref<HTMLDivElement>()
const pinnedToBottom = ref(true)

type View = 'log' | 'can'
const view = ref<View>('can')

// Periodic cmds the firmware broadcasts / the host polls on a schedule — the
// ones that flood the monitor. Toggle hides them so only command/exception
// traffic remains.
const PERIODIC_CMDS = new Set<number>([
  0x000, 0x001, 0x009, 0x00A, 0x014, 0x017, 0x003, 0x004, 0x01D,
])
const hidePeriodic = ref(true)
type DirFilter = 'all' | 'rx' | 'tx'
const dirFilter = ref<DirFilter>('all')

const MAX_ROWS = 300

const CMD_NAMES: Record<number, string> = {
  0x000: 'NMT', 0x001: 'HEARTBEAT', 0x002: 'ESTOP', 0x003: 'GET_MOTOR_ERR',
  0x004: 'GET_ENC_ERR', 0x006: 'SET_NODE_ID', 0x007: 'SET_AXIS_STATE',
  0x009: 'GET_ENC_EST', 0x00A: 'GET_ENC_CNT', 0x00B: 'SET_CTRL_MODE',
  0x00C: 'SET_INPUT_POS', 0x00D: 'SET_INPUT_VEL', 0x00E: 'SET_INPUT_TORQUE',
  0x00F: 'SET_LIMITS', 0x011: 'SET_TRAJ_VEL', 0x012: 'SET_TRAJ_ACCEL',
  0x013: 'SET_TRAJ_INERTIA', 0x014: 'GET_IQ', 0x016: 'REBOOT',
  0x017: 'GET_BUS_VI', 0x018: 'CLEAR_ERRORS', 0x019: 'SET_LINEAR_CNT',
  0x01A: 'SET_POS_GAIN', 0x01B: 'SET_VEL_GAINS', 0x01C: 'GET_ADC',
  0x01D: 'GET_CTRL_ERR', 0x01E: 'EXT', 0x01F: 'SET_MIT',
}

function tagClass(tag: string): string {
  switch (tag) {
    case 'FAULT':
    case 'ERR':     return 'log-err'
    case 'REC':     return 'log-rec'
    case 'CAN':     return 'log-can'
    case 'SAFETY':  return 'log-safety'
    case 'EXT':     return 'log-ext'
    case 'CMD':     return 'log-cmd'
    default:        return 'log-dim'
  }
}

const visibleFrames = computed<CanFrameDef[]>(() => {
  const arr = oSocket.canFrames.value
  const f = dirFilter.value
  let out = f === 'all' ? arr.slice() : arr.filter((x) => x.dir === f)
  if (hidePeriodic.value) out = out.filter((x) => !PERIODIC_CMDS.has(x.cmd))
  return out.slice(-MAX_ROWS)
})

const baseT = computed(() => {
  const arr = oSocket.canFrames.value
  return arr.length ? arr[0].t : 0
})

function cmdLabel(f: CanFrameDef): string {
  const base = CMD_NAMES[f.cmd] ?? `0x${f.cmd.toString(16)}`
  if (f.cmd === 0x1e && f.data.length >= 2) return `${base}_${f.data.slice(0, 2)}`
  return base
}
function arbId(f: CanFrameDef): string {
  return `0x${((f.node << 5) | (f.cmd & 0x1f)).toString(16).padStart(3, '0')}`
}
function dataHex(f: CanFrameDef): string {
  return f.data.match(/.{2}/g)?.join(' ') ?? ''
}
function relTime(f: CanFrameDef): string {
  return ((f.t - baseT.value) / 1000).toFixed(3)
}

const feedLen = computed(() =>
  view.value === 'log' ? oSocket.logs.value.length : visibleFrames.value.length,
)

watch(feedLen, async () => {
  if (!pinnedToBottom.value) return
  await nextTick()
  if (containerEl.value) containerEl.value.scrollTop = containerEl.value.scrollHeight
})

function onScroll() {
  if (!containerEl.value) return
  const el = containerEl.value
  pinnedToBottom.value = (el.scrollHeight - el.scrollTop - el.clientHeight) < 20
}

function clearAll() { oSocket.clearLogs() }
function setView(v: View) { view.value = v }
function setDir(d: DirFilter) { dirFilter.value = d }

// Quick command chips
const QUICK: { label: string; action: () => void }[] = [
  { label: 'Clear Err', action: () => oSocket.clearErrors() },
  { label: 'Estop',     action: () => oSocket.estop() },
  { label: 'Axis Status', action: () => oSocket.extCmd(0x01, 0x00) },
  { label: 'Save Cfg',  action: () => oSocket.extCmd(0x03, 0x00) },
  { label: 'Dev Info',  action: () => oSocket.extCmd(0x05, 0x01) },
  { label: 'Reboot',    action: () => oSocket.reboot() },
]

function runQuick(label: string, action: () => void) {
  if (!oSocket.ready.value) return
  if (label === 'Save Cfg' && !confirm('确认将当前配置保存到 Flash？')) return
  if (label === 'Reboot' && !confirm('确认重启控制器？当前运动将立即停止。')) return
  action()
}
</script>

<template>
  <div class="panel log-console">
    <div class="row toolbar">
      <div class="seg">
        <button :class="{ on: view === 'log' }" @click="setView('log')">Log</button>
        <button :class="{ on: view === 'can' }" @click="setView('can')">CAN</button>
      </div>
      <span class="spacer"></span>
      <template v-if="view === 'can'">
        <label class="chk" title="隐藏周期性遥测/心跳帧（HEARTBEAT / 编码器 / Iq / 母线 / 错误寄存器）">
          <input type="checkbox" v-model="hidePeriodic" /> 屏蔽周期帧
        </label>
        <div class="seg">
          <button :class="{ on: dirFilter === 'all' }" @click="setDir('all')">全部</button>
          <button :class="{ on: dirFilter === 'rx' }" @click="setDir('rx')">收</button>
          <button :class="{ on: dirFilter === 'tx' }" @click="setDir('tx')">发</button>
        </div>
      </template>
      <span class="pin-state" :class="{ off: !pinnedToBottom }">
        {{ pinnedToBottom ? '↓ follow' : '⏸ paused' }}
      </span>
      <button @click="clearAll" title="Clear">Clear</button>
    </div>

    <div ref="containerEl" class="console-area scroll" @scroll="onScroll">
      <!-- Log view -->
      <template v-if="view === 'log'">
        <div
          v-for="(l, i) in oSocket.logs.value"
          :key="i"
          class="log-line"
          :class="tagClass(l.tag)"
        >
          <span class="log-tag">{{ l.tag }}</span>
          <span class="log-text">{{ l.text }}</span>
        </div>
        <div v-if="!oSocket.logs.value.length" class="empty">无日志</div>
      </template>

      <!-- CAN view: left = raw frame, right = decoded meaning -->
      <template v-else>
        <div
          v-for="(f, i) in visibleFrames"
          :key="i"
          class="can-row"
          :class="f.dir"
        >
          <span class="can-t">{{ relTime(f) }}</span>
          <span class="can-dir" :class="f.dir">{{ f.dir === 'rx' ? '←' : '→' }}</span>
          <span class="can-raw">{{ cmdLabel(f) }} {{ arbId(f) }} {{ dataHex(f) }}</span>
          <span class="can-mean">{{ f.meaning }}</span>
        </div>
        <div v-if="!visibleFrames.length" class="empty">无 CAN 帧（被屏蔽或未连接）</div>
      </template>
    </div>

    <div class="quick-row">
      <button
        v-for="q in QUICK"
        :key="q.label"
        class="quick"
        :disabled="!oSocket.ready.value"
        @click="runQuick(q.label, q.action)"
      >{{ q.label }}</button>
    </div>
  </div>
</template>

<style scoped>
.log-console { display: flex; flex-direction: column; gap: 4px; height: 100%; overflow: hidden; }
.toolbar { display: flex; align-items: center; gap: 6px; padding: 2px 0; flex-wrap: wrap; }
.spacer { flex: 1; }
.seg { display: inline-flex; border: 1px solid var(--border); border-radius: 4px; overflow: hidden; }
.seg button { font-size: 10px; padding: 1px 8px; background: transparent; color: var(--fg-dim); border: none; border-right: 1px solid var(--border); }
.seg button:last-child { border-right: none; }
.seg button.on { background: var(--accent-2); color: #020617; }
.chk { display: inline-flex; align-items: center; gap: 3px; font-size: 10px; color: var(--fg-dim); cursor: pointer; }
.pin-state { font-size: 10px; color: var(--ok); font-family: var(--mono); }
.pin-state.off { color: var(--warn); }
.console-area {
  flex: 1; min-height: 0; overflow-y: auto;
  font-family: var(--mono); font-size: 11px; padding: 4px;
  background: #020617; border: 1px solid var(--border); border-radius: 4px;
}
.log-line { white-space: pre-wrap; word-break: break-all; line-height: 1.5; }
.log-tag { display: inline-block; min-width: 64px; color: var(--fg-dim); margin-right: 6px; }
.log-text { color: var(--fg); }
.log-err .log-text { color: var(--err); }
.log-err .log-tag { color: var(--err); }
.log-can .log-text { color: var(--warn); }
.log-safety .log-text { color: #f87171; font-weight: 600; }
.log-ext .log-text { color: #67e8f9; }
.log-cmd .log-text { color: #93c5fd; }
.log-rec .log-text { color: var(--ok); }
.log-dim .log-text { color: var(--fg-dim); }

.can-row { display: flex; align-items: baseline; gap: 6px; line-height: 1.5; white-space: nowrap; }
.can-t { color: var(--fg-dim); width: 52px; flex: 0 0 52px; }
.can-dir { width: 12px; flex: 0 0 12px; text-align: center; font-weight: 700; }
.can-row.rx .can-dir { color: #4ade80; }
.can-row.tx .can-dir { color: #22d3ee; }
.can-raw { color: var(--fg); flex: 0 0 auto; max-width: 55%; overflow: hidden; text-overflow: ellipsis; }
.can-row.rx .can-raw { color: #86efac; }
.can-row.tx .can-raw { color: #67e8f9; }
.can-mean { color: var(--fg-dim); flex: 1 1 auto; min-width: 0; overflow: hidden; text-overflow: ellipsis; }

.empty { color: var(--fg-dim); padding: 12px; text-align: center; }
.quick-row { display: flex; flex-wrap: wrap; gap: 3px; }
.quick {
  font-size: 10px; padding: 1px 6px;
  background: transparent; border: 1px solid var(--border); color: var(--fg-dim);
}
.quick:hover { color: var(--fg); border-color: var(--accent); }
</style>
