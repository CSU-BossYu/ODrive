<script setup lang="ts">
// Log viewer + quick command chips for ODrive CAN mode.

import { ref, watch, nextTick } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const oSocket = useOdriveSocket()

const containerEl = ref<HTMLDivElement>()
const pinnedToBottom = ref(true)

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

watch(() => oSocket.logs.value.length, async () => {
  if (!pinnedToBottom.value) return
  await nextTick()
  if (containerEl.value) {
    containerEl.value.scrollTop = containerEl.value.scrollHeight
  }
})

function onScroll() {
  if (!containerEl.value) return
  const el = containerEl.value
  pinnedToBottom.value = (el.scrollHeight - el.scrollTop - el.clientHeight) < 20
}

function clearLogs() {
  oSocket.clearLogs()
}

// Quick command chips
const QUICK: { label: string; action: () => void }[] = [
  { label: 'Clear Err', action: () => oSocket.clearErrors() },
  { label: 'Estop',     action: () => oSocket.estop() },
  { label: 'Axis Status', action: () => oSocket.extCmd(0x01, 0x00) },
  { label: 'Save Cfg',  action: () => oSocket.extCmd(0x03, 0x00) },
  { label: 'Dev Info',  action: () => oSocket.extCmd(0x05, 0x01) },
  { label: 'Reboot',    action: () => oSocket.reboot() },
]
</script>

<template>
  <div class="panel log-console">
    <div class="row toolbar">
      <span class="panel-title">Log / CAN</span>
      <span class="spacer"></span>
      <span class="pin-state" :class="{ off: !pinnedToBottom }">
        {{ pinnedToBottom ? '↓ follow' : '⏸ paused' }}
      </span>
      <button @click="clearLogs" title="Clear log">Clear</button>
    </div>
    <div ref="containerEl" class="log-area scroll" @scroll="onScroll">
      <div
        v-for="(l, i) in oSocket.logs.value"
        :key="i"
        class="log-line"
        :class="tagClass(l.tag)"
      >
        <span class="log-tag">{{ l.tag }}</span>
        <span class="log-text">{{ l.text }}</span>
      </div>
    </div>
    <div class="quick-row">
      <button v-for="q in QUICK" :key="q.label" class="quick" @click="q.action()">{{ q.label }}</button>
    </div>
  </div>
</template>

<style scoped>
.log-console { display: flex; flex-direction: column; gap: 4px; height: 100%; overflow: hidden; }
.toolbar { padding: 2px 0; }
.pin-state { font-size: 10px; color: var(--ok); font-family: var(--mono); }
.pin-state.off { color: var(--warn); }
.log-area {
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
.quick-row { display: flex; flex-wrap: wrap; gap: 3px; }
.quick {
  font-size: 10px; padding: 1px 6px;
  background: transparent; border: 1px solid var(--border); color: var(--fg-dim);
}
.quick:hover { color: var(--fg); border-color: var(--accent); }
</style>
