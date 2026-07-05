<script setup lang="ts">
// Root layout: top status bar + three-column workspace.
// CAN-only UI for ODrive single-axis controller.

import { ref } from 'vue'
import StatusBar from './components/StatusBar.vue'
import WaveformPanel from './components/WaveformPanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import ParamPanel from './components/ParamPanel.vue'
import LogConsole from './components/LogConsole.vue'
import OverspeedPanel from './components/OverspeedPanel.vue'
import { ODRIVE_DEFAULT_VISIBLE } from './channels'

const visibleChannels = ref<string[]>([...ODRIVE_DEFAULT_VISIBLE])
</script>

<template>
  <div class="app-root">
    <header class="app-header">
      <span class="app-name">ODrive CAN</span>
      <StatusBar />
    </header>
    <div class="workspace">
      <div class="left-col">
        <ControlPanel />
      </div>
      <div class="center-col">
        <div class="waveform-wrap">
          <WaveformPanel v-model:visible="visibleChannels" />
        </div>
        <div class="log-wrap">
          <LogConsole />
        </div>
      </div>
      <div class="right-col">
        <ParamPanel />
        <OverspeedPanel />
      </div>
    </div>
  </div>
</template>

<style scoped>
.app-root {
  display: flex;
  flex-direction: column;
  height: 100vh;
  overflow: hidden;
}
.app-header {
  display: flex;
  align-items: stretch;
  gap: 12px;
  background: #020617;
  border-bottom: 1px solid var(--border);
  flex: 0 0 auto;
}
.app-name {
  display: flex;
  align-items: center;
  padding: 0 14px;
  font-size: 13px;
  font-weight: 700;
  color: var(--accent-2);
  border-right: 1px solid var(--border);
  white-space: nowrap;
  letter-spacing: 0.05em;
}
.app-header :deep(.status-bar) {
  border-bottom: none;
  flex: 1;
}
.workspace {
  flex: 1;
  display: grid;
  grid-template-columns: minmax(320px, 360px) minmax(520px, 1fr) minmax(260px, 300px);
  gap: 10px;
  padding: 10px;
  min-height: 0;
}
.left-col, .right-col {
  display: flex;
  flex-direction: column;
  min-height: 0;
  overflow: hidden;
  gap: 8px;
}
.center-col {
  display: grid;
  grid-template-rows: minmax(300px, 1.45fr) minmax(150px, 0.85fr);
  gap: 10px;
  min-height: 0;
}
.waveform-wrap, .log-wrap { min-height: 0; min-width: 0; }

@media (max-width: 1280px) {
  .workspace { grid-template-columns: 300px 1fr 260px; }
}
@media (max-width: 1024px) {
  .workspace { grid-template-columns: 220px 1fr; }
  .right-col { grid-column: 1 / 3; max-height: 180px; }
}
</style>
