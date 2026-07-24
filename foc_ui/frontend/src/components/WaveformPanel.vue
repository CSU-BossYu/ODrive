<script setup lang="ts">
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { useOdriveSocket } from '../composables/useOdriveSocket'
import { ODRIVE_CHANNELS, ODRIVE_CHANNEL_BY_KEY, odriveChannelColor } from '../channels'

const props = defineProps<{ visible: string[] }>()
const emit = defineEmits<{ (e: 'update:visible', v: string[]): void }>()

const socket = useOdriveSocket()

const MAX_POINTS = 6000
const timeBuf = new Float64Array(MAX_POINTS)
const valueBufs = new Map<string, Float64Array>()
let writeIdx = 0
let filled = 0
let lastTmono = 0
let rafPending = false

const chartEl = ref<HTMLDivElement>()
const windowSeconds = ref(10)
const frameRate = ref(0)
const filledCount = ref(0)
const currentStats = ref<Record<string, { min: number; max: number; latest: number }>>({})

for (const c of ODRIVE_CHANNELS) valueBufs.set(c.key, new Float64Array(MAX_POINTS))

let uplot: uPlot | null = null

const hasData = computed(() => filledCount.value > 0)
const visibleDefs = computed(() =>
  props.visible.map((k) => ODRIVE_CHANNEL_BY_KEY[k]).filter(Boolean),
)

function displayValue(key: string, value: number): number {
  if (key === 'pos') return value * 2 * Math.PI
  if (key === 'vel') return value * 60
  return value
}

function formatValue(v: number | undefined, digits = 2): string {
  return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '--'
}

function buildOptions(): uPlot.Options {
  const series: uPlot.Series[] = [{
    label: '时间',
    value: (_u, v) => (v == null ? '--' : `${v.toFixed(2)}s`),
  }]

  for (const c of visibleDefs.value) {
    series.push({
      label: `${c.label} (${c.unit})`,
      scale: 'y',
      stroke: odriveChannelColor(c.key),
      width: 2,
      points: { show: false },
      value: (_u, v) => (v == null ? '--' : `${v.toFixed(c.group === 'cur' ? 3 : 2)} ${c.unit}`),
    })
  }

  const scales: Record<string, uPlot.Scale> = {
    x: { time: false },
    y: { auto: false, range: () => [-0.08, 1.08] },
  }

  const axes: uPlot.Axis[] = [{
    stroke: '#94a3b8',
    grid: { stroke: 'rgba(71, 85, 105, 0.55)', width: 1 },
    ticks: { stroke: '#475569', width: 1 },
    values: (_self, vals) => vals.map((v) => `${v.toFixed(1)}s`),
  }]

  axes.push({
    scale: 'y',
    side: 3,
    size: 52,
    stroke: '#94a3b8',
    grid: { stroke: 'rgba(30, 41, 59, 0.9)', width: 1 },
    ticks: { stroke: '#475569', width: 1 },
    label: '窗口归一化',
    labelSize: 18,
    labelFont: '11px system-ui',
    values: (_self, vals) => vals.map((v) => `${Math.round(Number(v) * 100)}%`),
  })

  return {
    width: chartEl.value?.clientWidth ?? 800,
    height: chartEl.value?.clientHeight ?? 300,
    series,
    scales,
    axes,
    legend: { show: false },
    cursor: { drag: { x: true, y: true, uni: 50 }, points: { show: false } },
    padding: [10, 10, 0, 4],
  }
}

function rebuildChart() {
  if (!chartEl.value) return
  uplot?.destroy()
  uplot = null
  const width = chartEl.value.clientWidth
  const height = chartEl.value.clientHeight
  if (width < 20 || height < 20) {
    nextTick(rebuildChart)
    return
  }
  uplot = new uPlot({ ...buildOptions(), width, height }, [], chartEl.value)
  pushBuffersToChart()
}

function collectWindowData() {
  const visible = visibleDefs.value
  if (filled === 0) {
    return {
      data: [[], ...visible.map(() => [])] as number[][],
      stats: {} as Record<string, { min: number; max: number; latest: number }>,
    }
  }

  const newestIdx = (writeIdx - 1 + MAX_POINTS) % MAX_POINTS
  const newestT = timeBuf[newestIdx]
  const startT = newestT - windowSeconds.value
  let n = 0
  while (n < filled) {
    const ringIdx = (writeIdx - 1 - n + MAX_POINTS) % MAX_POINTS
    if (timeBuf[ringIdx] < startT) break
    n++
  }

  const rawSeries: number[][] = visible.map(() => new Array(n))
  const data: (number | null)[][] = [new Array(n)]
  for (let i = 0; i < visible.length; i++) data.push(new Array(n))
  const stats: Record<string, { min: number; max: number; latest: number }> = {}

  for (let i = 0; i < n; i++) {
    const ringIdx = (writeIdx - n + i + MAX_POINTS) % MAX_POINTS
    data[0][i] = timeBuf[ringIdx] - newestT + windowSeconds.value
    visible.forEach((c, j) => {
      const raw = valueBufs.get(c.key)![ringIdx]
      const v = Number.isFinite(raw) ? displayValue(c.key, raw) : NaN
      rawSeries[j][i] = v
      if (!Number.isFinite(v)) return
      const s = stats[c.key]
      if (!s) stats[c.key] = { min: v, max: v, latest: v }
      else {
        s.min = Math.min(s.min, v)
        s.max = Math.max(s.max, v)
        s.latest = v
      }
    })
  }

  visible.forEach((c, j) => {
    const s = stats[c.key]
    const span = s ? s.max - s.min : 0
    for (let i = 0; i < n; i++) {
      const v = rawSeries[j][i]
      if (!Number.isFinite(v)) {
        data[j + 1][i] = null
      } else if (!s || Math.abs(span) < 1e-9) {
        data[j + 1][i] = 0.5
      } else {
        data[j + 1][i] = (v - s.min) / span
      }
    }
  })

  return { data, stats }
}

function pushBuffersToChart() {
  if (!uplot) return
  const { data, stats } = collectWindowData()
  currentStats.value = stats
  uplot.setData(data as any)
  uplot.setScale('x', { min: 0, max: windowSeconds.value })
}

function scheduleChartUpdate() {
  if (rafPending) return
  rafPending = true
  requestAnimationFrame(() => {
    rafPending = false
    pushBuffersToChart()
  })
}

function onTelemetry() {
  const msg = socket.latest.value
  if (!msg) return
  const t = msg.t / 1000
  if (lastTmono > 0) {
    const dt = t - lastTmono
    if (dt > 0) frameRate.value = frameRate.value ? frameRate.value * 0.9 + (1 / dt) * 0.1 : 1 / dt
  }
  lastTmono = t

  for (const c of ODRIVE_CHANNELS) {
    const value = msg.ch[c.key]
    valueBufs.get(c.key)![writeIdx] = typeof value === 'number' ? value : NaN
  }
  timeBuf[writeIdx] = t
  writeIdx = (writeIdx + 1) % MAX_POINTS
  filled = Math.min(filled + 1, MAX_POINTS)
  filledCount.value = filled
  scheduleChartUpdate()
}

function toggle(key: string) {
  const next = [...props.visible]
  const i = next.indexOf(key)
  if (i >= 0) next.splice(i, 1)
  else next.push(key)
  emit('update:visible', next)
}

function showOnly(key: string) {
  emit('update:visible', [key])
}

function clearWaveform() {
  writeIdx = 0
  filled = 0
  filledCount.value = 0
  currentStats.value = {}
  lastTmono = 0
  frameRate.value = 0
  pushBuffersToChart()
}

watch(() => socket.latest.value, onTelemetry)
watch(() => props.visible, () => nextTick(rebuildChart), { deep: true })
watch(windowSeconds, () => pushBuffersToChart())

let resizeObs: ResizeObserver | null = null
let resizeTimer: number | null = null
function onResize() {
  if (resizeTimer != null) clearTimeout(resizeTimer)
  resizeTimer = window.setTimeout(() => {
    if (!uplot || !chartEl.value) return
    uplot.setSize({ width: chartEl.value.clientWidth, height: chartEl.value.clientHeight })
  }, 80)
}

onMounted(async () => {
  await nextTick()
  requestAnimationFrame(rebuildChart)
  resizeObs = new ResizeObserver(onResize)
  if (chartEl.value) resizeObs.observe(chartEl.value)
})

onBeforeUnmount(() => {
  resizeObs?.disconnect()
  if (resizeTimer != null) clearTimeout(resizeTimer)
  uplot?.destroy()
  uplot = null
})
</script>

<template>
  <div class="panel waveform-panel">
    <div class="panel-head">
      <span class="title">波形</span>
      <span class="spacer"></span>
      <label class="control">窗口
        <select v-model.number="windowSeconds">
          <option :value="5">5s</option>
          <option :value="10">10s</option>
          <option :value="30">30s</option>
          <option :value="60">60s</option>
        </select>
      </label>
      <span class="rate-info dim">独立缩放</span>
      <span class="rate-info" :class="{ dim: frameRate <= 0 }">
        {{ frameRate > 0 ? `${frameRate.toFixed(0)} Hz` : '无数据' }}
      </span>
      <span class="rate-info">{{ visibleDefs.length }}/{{ ODRIVE_CHANNELS.length }} 通道</span>
      <button class="mini" @click="clearWaveform">清空</button>
    </div>

    <div class="chart-area">
      <div ref="chartEl" class="chart"></div>
      <div v-if="!hasData" class="empty-overlay">
        <div class="empty-text">
          等待 CAN 遥测
          <div class="empty-hint">连接 CAN 后，位置/速度/Iq 会显示在这里。</div>
        </div>
      </div>
      <div v-else-if="visibleDefs.length === 0" class="empty-overlay">
        <div class="empty-text">
          未选择曲线
          <div class="empty-hint">在下方选择一个通道。</div>
        </div>
      </div>
    </div>

    <div class="channel-strip">
      <button
        v-for="c in ODRIVE_CHANNELS"
        :key="c.key"
        class="channel-chip"
        :class="{ on: props.visible.includes(c.key) }"
        :style="{ '--cc': odriveChannelColor(c.key) }"
        :title="`双击只看 ${c.label}`"
        @click="toggle(c.key)"
        @dblclick.prevent="showOnly(c.key)"
      >
        <span class="dot"></span>
        <span class="label">{{ c.label }}</span>
        <span class="value">
          {{ formatValue(currentStats[c.key]?.latest, c.group === 'cur' ? 3 : 1) }} {{ c.unit }}
        </span>
      </button>
    </div>
  </div>
</template>

<style scoped>
.waveform-panel {
  display: flex;
  flex-direction: column;
  height: 100%;
  min-height: 0;
  gap: 8px;
  padding: 10px;
}
.panel-head {
  display: flex;
  align-items: center;
  gap: 8px;
  min-height: 28px;
  flex: 0 0 auto;
}
.title {
  font-size: 13px;
  color: var(--fg);
  font-weight: 750;
}
.spacer { flex: 1; }
.control {
  display: inline-flex;
  align-items: center;
  gap: 5px;
  font-size: 11px;
  color: var(--fg-dim);
}
.control select {
  width: 78px;
  padding: 3px 6px;
  font-size: 11px;
}
.rate-info {
  font-size: 11px;
  font-family: var(--mono);
  color: var(--accent-2);
  background: rgba(6, 182, 212, 0.1);
  padding: 3px 7px;
  border-radius: 4px;
  white-space: nowrap;
}
.rate-info.dim {
  color: var(--fg-dim);
  background: rgba(148, 163, 184, 0.1);
}
.mini {
  padding: 4px 10px;
  font-size: 11px;
}
.chart-area {
  flex: 1 1 auto;
  min-height: 180px;
  position: relative;
  background: #020617;
  border: 1px solid var(--border);
  border-radius: 6px;
  overflow: hidden;
}
.chart {
  position: absolute;
  inset: 0;
}
.empty-overlay {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  pointer-events: none;
  background: rgba(2, 6, 23, 0.45);
}
.empty-text {
  text-align: center;
  color: var(--fg);
  font-size: 14px;
  font-weight: 650;
}
.empty-hint {
  margin-top: 8px;
  font-size: 11px;
  line-height: 1.6;
  color: var(--fg-dim);
  font-weight: 400;
}
.channel-strip {
  flex: 0 0 auto;
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 6px;
  padding-top: 2px;
}
.channel-chip {
  display: grid;
  grid-template-columns: auto 1fr auto;
  align-items: center;
  gap: 6px;
  min-width: 0;
  padding: 6px 8px;
  font-size: 11px;
  background: rgba(15, 23, 42, 0.62);
  border: 1px solid rgba(148, 163, 184, 0.25);
  color: var(--fg-dim);
  opacity: 0.7;
  transition: opacity 0.12s, border-color 0.12s, background 0.12s;
}
.channel-chip:hover {
  opacity: 0.92;
}
.channel-chip.on {
  opacity: 1;
  border-color: var(--cc);
  color: var(--fg);
  background: color-mix(in srgb, var(--cc) 12%, rgba(15, 23, 42, 0.75));
}
.dot {
  width: 9px;
  height: 9px;
  border-radius: 50%;
  background: var(--cc);
  box-shadow: 0 0 0 2px color-mix(in srgb, var(--cc) 20%, transparent);
}
.label {
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  font-weight: 650;
}
.value {
  font-family: var(--mono);
  color: var(--fg-dim);
  white-space: nowrap;
}
@media (max-width: 1100px) {
  .panel-head {
    flex-wrap: wrap;
  }
  .channel-strip {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
}
</style>
