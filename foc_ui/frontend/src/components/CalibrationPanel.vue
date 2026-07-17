<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from 'vue'
import { useOdriveSocket } from '../composables/useOdriveSocket'

const socket = useOdriveSocket()
const geometryTurns = ref(2)
let pollTimer: number | null = null

const stateNames: Record<number, string> = {
  0: '未开始', 1: '采集中', 2: '采集完成', 3: '拟合中', 4: '已识别',
  5: '验证中', 6: '验证通过', 7: '写入中', 8: '已提交', 9: '失败',
  10: '已中止', 11: '结果过期',
}
const stageNames: Record<number, string> = {
  0: '无', 1: '安全预检', 10: '电气参数', 20: '编码器与减速器几何',
  30: '磁链与机械模型', 35: '电角度动态延迟', 40: '参数拟合',
  50: '交叉验证', 60: '原子保存',
}
const failureNames: Record<number, string> = {
  0: '无', 1: '不支持的标定配置', 2: '电机电气标定失败',
  3: '编码器对齐失败', 4: '几何扫描失败', 5: '同步采样丢失',
  6: '机械扫描失败', 7: '候选参数验证失败', 8: '配置原子保存失败',
  9: '电角度动态延迟标定失败',
  10: '磁链标定失败：恒速分块样本不足',
  11: '磁链标定失败：磁链均值非物理',
  12: '磁链标定失败：分块均值离散度超限',
  13: '机械标定失败：闭环样本或加速度激励不足',
  14: '机械标定失败：参数回归矩阵奇异',
  15: '机械标定失败：惯量或摩擦参数非物理',
  16: '机械标定失败：候选参数闭环启动失败',
  17: '机械标定失败：运动失控或速度超出安全范围',
  18: '机械标定失败：全部样本的电流或编码器无效',
  19: '机械标定失败：全部样本处于 PWM 饱和状态',
  20: '机械标定失败：未检测到足够的输出轴运动',
  21: '机械标定失败：全部样本时间戳无效（DWT 周期计数器）',
  22: '电角度延迟标定失败：候选闭环启动失败',
  23: '电角度延迟标定失败：正反向有效样本不足',
  24: '电角度延迟标定失败：电角速度激励不可观测',
  25: '电角度延迟标定失败：延迟、截距或残差超出物理门限',
}

const resultDefs = [
  ['phase_resistance', '相电阻', 'Ω', 6, 1],
  ['phase_inductance', '相电感', 'H', 8, 2],
  ['pole_pairs', '极对数', '', 0, 64],
  ['encoder_direction', '编码器方向', '', 0, 4],
  ['phase_offset', '电角度整数偏移', 'count', 0, 8],
  ['phase_offset_float', '电角度小数偏移', 'count', 4, 8],
  ['effective_ratio_scale', '减速比修正系数', '', 7, 16],
  ['flux_linkage', '磁链', 'V·s/rad', 7, 32],
  ['torque_constant', '转矩常数 Kt', 'Nm/Aq', 6, 32],
  ['output_inertia', '输出端等效惯量', 'Nm/(turn/s²)', 7, 128],
  ['friction_coulomb_pos', '正向库仑摩擦', 'Nm', 5, 128],
  ['friction_coulomb_neg', '反向库仑摩擦', 'Nm', 5, 128],
  ['friction_viscous_pos', '正向粘滞系数', 'Nm/(turn/s)', 5, 128],
  ['friction_viscous_neg', '反向粘滞系数', 'Nm/(turn/s)', 5, 128],
  ['electrical_delay', '电角度动态延迟', 'µs', 2, 256, 1e6],
] as const

const qualityDefs = [
  ['geometry_raw_rms', '几何原始 RMS', 'turn', 7],
  ['geometry_corrected_rms', '几何补偿后 RMS', 'turn', 7],
  ['geometry_direction_peak_to_peak', '方向相关峰峰值', 'turn', 7],
  ['geometry_used_samples', '几何有效样本', '', 0],
  ['flux_sample_stddev', '磁链标准差', 'V·s/rad', 7],
  ['flux_used_samples', '磁链有效样本', '', 0],
  ['mechanical_residual_rms_torque', '机械拟合 RMS', 'Nm', 6],
  ['mechanical_used_samples', '机械有效样本', '', 0],
  ['mechanical_attempted_samples', '机械尝试样本', '', 0],
  ['mechanical_rejected_invalid', '机械无效电流/编码器', '', 0],
  ['mechanical_rejected_saturated', '机械 PWM 饱和样本', '', 0],
  ['mechanical_rejected_low_velocity', '机械低速拒绝样本', '', 0],
  ['mechanical_max_abs_velocity', '机械最大实测速度', 'turn/s', 6],
  ['mechanical_rejected_timing', '机械时间戳拒绝样本', '', 0],
  ['delay_residual_phase_offset', '延迟拟合相位截距', 'rad', 6],
  ['delay_residual_rms', '延迟拟合 RMS', 'rad', 6],
  ['delay_used_samples', '延迟有效样本', '', 0],
  ['delay_attempted_samples', '延迟尝试样本', '', 0],
  ['delay_rejected_invalid', '延迟无效电流/电压', '', 0],
  ['delay_rejected_saturated', '延迟 PWM 饱和样本', '', 0],
  ['delay_rejected_speed', '延迟低电角速度样本', '', 0],
  ['delay_rejected_emf', '延迟反电势不足样本', '', 0],
  ['delay_rejected_phase', '延迟相位异常样本', '', 0],
  ['delay_max_abs_electrical_speed', '最大实测电角速度', 'rad/s', 2],
] as const

const snapshot = computed(() => socket.calibrationSnapshot.value)
const session = computed(() => snapshot.value?.session ?? null)
const candidate = computed(() => snapshot.value?.candidate ?? null)
const state = computed(() => session.value?.state ?? 0)
const running = computed(() => state.value >= 1 && state.value <= 7)
const terminal = computed(() => [8, 9, 10, 11].includes(state.value))
const progress = computed(() => Math.max(0, Math.min(100, (session.value?.progress_permille ?? 0) / 10)))
const stateLabel = computed(() => stateNames[state.value] ?? `未知状态 ${state.value}`)
const stageLabel = computed(() => stageNames[session.value?.stage ?? 0] ?? `阶段 ${session.value?.stage}`)
const failureLabel = computed(() => failureNames[session.value?.failure_code ?? 0] ?? `错误 ${session.value?.failure_code}`)
const validity = computed(() => Number(candidate.value?.validity ?? 0))

function formatValue(key: string, digits: number, scale = 1): string {
  const raw = candidate.value?.[key]
  return typeof raw === 'number' && Number.isFinite(raw) ? (raw * scale).toFixed(digits) : '--'
}

function startPolling() {
  if (pollTimer != null) return
  pollTimer = window.setInterval(() => socket.getCalibrationStatus(false), 1000)
}

function stopPolling() {
  if (pollTimer != null) window.clearInterval(pollTimer)
  pollTimer = null
}

function startCalibration() {
  const turns = Math.max(1, Math.min(8, Math.trunc(geometryTurns.value || 2)))
  geometryTurns.value = turns
  socket.startCalibration(turns)
  startPolling()
}

function abortCalibration() {
  socket.abortCalibration()
}

watch(() => socket.ready.value, (ready) => {
  if (ready) socket.getCalibrationStatus(false)
  else stopPolling()
}, { immediate: true })

watch(snapshot, (value) => {
  if (value && !value.ok) stopPolling()
  else if (running.value) startPolling()
  else if (terminal.value || state.value === 0) stopPolling()
})

onBeforeUnmount(stopPolling)
</script>

<template>
  <div class="panel calibration-panel">
    <div class="panel-head">
      <div>
        <h3 class="panel-title">完整参数校准</h3>
        <div class="subtitle">一次指令，由固件完成采样、拟合、验证和原子保存</div>
      </div>
      <span class="pill" :class="state === 8 ? 'ok' : state === 9 ? 'err' : running ? 'warn' : 'dim'">
        {{ stateLabel }}
      </span>
    </div>

    <div class="warning">
      轴会正反多圈转动约数分钟。请卸载、清空运动范围并确保急停可用。
    </div>

    <div class="actions">
      <label>几何扫描圈数
        <input v-model.number="geometryTurns" type="number" min="1" max="8" step="1" :disabled="running" />
      </label>
      <button class="warn" @click="startCalibration" :disabled="!socket.ready.value || running">开始完整校准</button>
      <button @click="abortCalibration" :disabled="!socket.ready.value || !running">中止</button>
      <button @click="socket.getCalibrationStatus(true)" :disabled="!socket.ready.value">刷新结果</button>
    </div>

    <div class="progress-wrap">
      <div class="progress-meta"><span>{{ stageLabel }}</span><strong>{{ progress.toFixed(1) }}%</strong></div>
      <div class="progress-track"><div class="progress-fill" :style="{ width: `${progress}%` }"></div></div>
    </div>

    <div v-if="snapshot && !snapshot.ok" class="message err">{{ snapshot.error || '读取校准状态失败' }}</div>
    <div v-if="state === 9" class="message err">失败原因：{{ failureLabel }}</div>
    <div v-else-if="state === 8" class="message ok">全部候选参数验证通过，已写入配置并投入运行补偿。</div>
    <div v-else-if="state === 10" class="message dim">校准已中止，原运行参数保持不变。</div>

    <div class="session-meta" v-if="session">
      <span>会话 #{{ session.session_id }}</span>
      <span>原始流 {{ session.transport_frames_sent }} 帧</span>
      <span :class="session.dropped_records ? 'bad' : ''">丢样 {{ session.dropped_records }}</span>
    </div>

    <details v-if="candidate" open>
      <summary>补偿参数 <span class="validity">valid 0x{{ validity.toString(16).padStart(3, '0') }}</span></summary>
      <div class="result-grid">
        <div v-for="d in resultDefs" :key="d[0]" :class="{ pending: !(validity & d[4]) }">
          <span>{{ d[1] }}</span>
          <strong>{{ formatValue(d[0], d[3], d[5] ?? 1) }}</strong>
          <em>{{ d[2] }}</em>
        </div>
      </div>
    </details>

    <details v-if="candidate">
      <summary>拟合质量与样本统计</summary>
      <div class="result-grid quality">
        <div v-for="d in qualityDefs" :key="d[0]">
          <span>{{ d[1] }}</span><strong>{{ formatValue(d[0], d[3]) }}</strong><em>{{ d[2] }}</em>
        </div>
      </div>
    </details>
  </div>
</template>

<style scoped>
.calibration-panel { flex: 0 0 auto; padding: 12px; overflow: visible; }
.panel-head, .progress-meta, .session-meta { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
.panel-title { margin: 0; }
.subtitle { margin-top: 3px; color: var(--fg-dim); font-size: 11px; }
.warning { margin: 10px 0; padding: 8px; border: 1px solid #854d0e; border-radius: 5px; background: #42200655; color: #fde68a; font-size: 11px; line-height: 1.5; }
.actions { display: grid; grid-template-columns: 1fr 1.2fr .7fr .8fr; gap: 6px; align-items: end; }
.actions label { color: var(--fg-dim); font-size: 10px; }
.actions input { width: 100%; margin-top: 3px; box-sizing: border-box; }
.progress-wrap { margin-top: 10px; }
.progress-meta { font-size: 11px; margin-bottom: 4px; }
.progress-track { height: 7px; overflow: hidden; border-radius: 4px; background: #1e293b; }
.progress-fill { height: 100%; background: var(--accent); transition: width .25s ease; }
.message { margin-top: 8px; padding: 7px; border-radius: 4px; font-size: 11px; }
.message.err { background: #7f1d1d55; color: #fecaca; }
.message.ok { background: #14532d55; color: #bbf7d0; }
.message.dim { background: #1e293b; color: var(--fg-dim); }
.session-meta { margin-top: 8px; color: var(--fg-dim); font-size: 10px; }
.bad { color: #fca5a5; }
details { margin-top: 9px; border-top: 1px solid var(--border); padding-top: 7px; }
summary { cursor: pointer; color: var(--fg); font-size: 11px; font-weight: 600; }
.validity { float: right; color: var(--fg-dim); font-family: monospace; font-weight: 400; }
.result-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 4px 10px; margin-top: 7px; }
.result-grid > div { display: grid; grid-template-columns: 1fr auto auto; gap: 4px; align-items: baseline; font-size: 10px; min-width: 0; }
.result-grid span { color: var(--fg-dim); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.result-grid strong { font-family: monospace; font-size: 10px; }
.result-grid em { min-width: 24px; color: var(--fg-dim); font-size: 9px; font-style: normal; }
.result-grid > div.pending { opacity: .38; }
@media (max-width: 1200px) { .actions { grid-template-columns: 1fr 1fr; } .result-grid { grid-template-columns: 1fr; } }
</style>
