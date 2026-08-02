<script setup lang="ts">
// Root layout: top status bar + three-column workspace.
// CAN Control and USB Diagnostics are intentionally separate connections.

import { ref } from 'vue'
import StatusBar from './components/StatusBar.vue'
import WaveformPanel from './components/WaveformPanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import CalibrationPanel from './components/CalibrationPanel.vue'
import ParamPanel from './components/ParamPanel.vue'
import LogConsole from './components/LogConsole.vue'
import OverspeedPanel from './components/OverspeedPanel.vue'
import UsbDiagnosticsPanel from './components/UsbDiagnosticsPanel.vue'
import { ODRIVE_DEFAULT_VISIBLE } from './channels'

const visibleChannels = ref<string[]>([...ODRIVE_DEFAULT_VISIBLE])
</script>

<template>
  <div class="app-root">
    <header class="app-header">
      <div class="brand">
        <span class="brand-mark">FOC</span>
        <span class="brand-copy">
          <strong>Motion Console</strong>
          <small>ODrive · CAN 调试工作台</small>
        </span>
      </div>
      <div class="status-wrap">
        <StatusBar />
      </div>
    </header>
    <main class="workspace">
      <section class="left-col workspace-column" aria-label="运动控制">
        <div class="column-label"><span>01</span> 运动控制</div>
        <ControlPanel />
        <CalibrationPanel />
      </section>
      <section class="center-col workspace-column" aria-label="实时监控">
        <div class="column-label"><span>02</span> 实时监控</div>
        <div class="waveform-wrap">
          <WaveformPanel v-model:visible="visibleChannels" />
        </div>
        <UsbDiagnosticsPanel />
        <div class="log-wrap">
          <LogConsole />
        </div>
      </section>
      <aside class="right-col workspace-column" aria-label="参数与诊断">
        <div class="column-label"><span>03</span> 参数与诊断</div>
        <ParamPanel />
        <OverspeedPanel />
      </aside>
    </main>
  </div>
</template>

<style scoped>
.app-root {
  display: flex;
  flex-direction: column;
  height: 100vh;
  overflow: hidden;
  background:
    radial-gradient(circle at 48% -20%, rgba(45, 212, 191, 0.08), transparent 38%),
    var(--bg);
}
.app-header {
  display: flex;
  align-items: center;
  min-height: 76px;
  padding: 0 16px;
  gap: 20px;
  background: rgba(8, 13, 22, 0.94);
  border-bottom: 1px solid var(--border-subtle);
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.18);
  flex: 0 0 auto;
  position: relative;
  z-index: 5;
}
.brand {
  display: flex;
  align-items: center;
  gap: 10px;
  padding-right: 16px;
  border-right: 1px solid var(--border-subtle);
  white-space: nowrap;
}
.brand-mark {
  display: grid;
  place-items: center;
  width: 38px;
  height: 38px;
  border: 1px solid rgba(45, 212, 191, 0.45);
  border-radius: 11px;
  background: linear-gradient(145deg, rgba(45, 212, 191, 0.18), rgba(20, 184, 166, 0.04));
  color: var(--accent-2);
  font: 800 11px/1 var(--mono);
  letter-spacing: 0.08em;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.08), 0 0 22px rgba(45, 212, 191, 0.08);
}
.brand-copy {
  display: grid;
  gap: 2px;
}
.brand-copy strong {
  color: var(--fg-strong);
  font-size: 14px;
  letter-spacing: 0.01em;
}
.brand-copy small {
  color: var(--fg-muted);
  font-size: 10px;
  letter-spacing: 0.04em;
}
.status-wrap {
  min-width: 0;
  flex: 1;
}
.app-header :deep(.status-bar) {
  background: transparent;
  padding: 8px 0;
}
.workspace {
  flex: 1;
  display: grid;
  grid-template-columns: minmax(330px, 370px) minmax(680px, 1fr) minmax(320px, 360px);
  gap: 16px;
  padding: 16px;
  min-height: 0;
}
.workspace-column {
  position: relative;
  padding-top: 27px;
  min-width: 0;
}
.column-label {
  position: absolute;
  inset: 0 2px auto 2px;
  display: flex;
  align-items: center;
  gap: 7px;
  height: 24px;
  color: var(--fg-muted);
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 0.12em;
  text-transform: uppercase;
}
.column-label span {
  color: var(--accent-2);
  font-family: var(--mono);
  letter-spacing: 0;
}
.left-col, .right-col {
  display: flex;
  flex-direction: column;
  min-height: 0;
  overflow: hidden;
  gap: 10px;
}
.left-col :deep(.control-panel) { flex: 1 1 auto; }
.left-col :deep(.calibration-panel) { max-height: 44%; }
.right-col :deep(.param-panel),
.right-col :deep(.overspeed-panel) {
  flex: 1 1 0;
  min-height: 0;
}
.center-col {
  display: grid;
  grid-template-rows: 27px minmax(230px, 1fr) minmax(270px, 1.15fr) minmax(125px, .55fr);
  gap: 0;
  min-height: 0;
}
.center-col .column-label { position: static; }
.center-col .waveform-wrap { margin-bottom: 14px; }
.waveform-wrap, .log-wrap { min-height: 0; min-width: 0; }

@media (max-width: 1360px) {
  .workspace { grid-template-columns: 310px minmax(500px, 1fr) 300px; gap: 14px; padding: 14px; }
  .brand-copy { display: none; }
}
@media (max-width: 1080px) {
  .app-root {
    height: auto;
    min-height: 100vh;
    overflow: visible;
  }
  .app-header {
    align-items: flex-start;
    position: sticky;
    top: 0;
  }
  .brand { padding-top: 9px; }
  .workspace {
    grid-template-columns: minmax(300px, 0.78fr) minmax(460px, 1.22fr);
    align-items: start;
  }
  .left-col, .right-col { overflow: visible; }
  .left-col :deep(.calibration-panel) { max-height: none; }
  .right-col :deep(.param-panel),
  .right-col :deep(.overspeed-panel) { min-height: 520px; }
  .center-col { min-height: 720px; }
  .right-col {
    grid-column: 1 / -1;
    display: grid;
    grid-template-columns: 1fr 1fr;
    padding-top: 30px;
  }
}
@media (max-width: 760px) {
  .app-header { padding: 0 10px; gap: 8px; }
  .brand { display: none; }
  .workspace {
    display: flex;
    flex-direction: column;
    padding: 10px;
  }
  .workspace-column {
    width: 100%;
    overflow: visible;
  }
  .center-col {
    display: grid;
    grid-template-rows: 27px minmax(420px, 60vh) minmax(450px, 62vh) minmax(300px, 42vh);
    min-height: 780px;
  }
  .right-col { display: flex; }
  .right-col :deep(.param-panel),
  .right-col :deep(.overspeed-panel) { min-height: 480px; }
}

/* A short desktop viewport should scroll the workspace instead of squeezing
   the diagnostic and log cards into unreadable strips. */
@media (min-width: 1081px) and (max-height: 820px) {
  .app-root { height: auto; min-height: 100vh; overflow: auto; }
  .workspace { min-height: 760px; }
}
</style>
