import { computed, onBeforeUnmount, ref } from 'vue'
import { SCOPE_CHANNELS, type ScopeChannel } from '../scope_channels_generated'

export interface UsbSample {
  control_sequence: number
  timestamp_cycles: number
  flags: number
  values: Record<number, number>
}

export interface UsbScopeStatus {
  capture_id: number
  state: number
  final_reason: number
  mode: number
  trigger_type: number
  sample_count: number
  dropped_samples: number
  sequence_gaps: number
  channel_count: number
  decimation: number
  first_timestamp_cycles: number
  last_timestamp_cycles: number
  ok?: boolean
  applied_config?: Record<string, unknown>
  rejected_config?: Record<string, unknown>
}

export interface UsbFaultEvent {
  fault_sequence: number
  control_sequence: number
  timestamp_cycles: number
  state_epoch: number
  source: number
  code: number
  site: number
  severity: number
  parent_fault_sequence: number
  arg0: number
  arg1: number
  arg2: number
}

export interface UsbTimelineEvent {
  sequence: number
  state_epoch: number
  timestamp_cycles: number
  request_id: number
  state: number
  operation: number
  axis_state: number
  reason: number
}

export interface UsbCrashReport {
  generation: number
  fault_kind: number
  flags: number
  record_crc: number
  record_build_id: string
  record_manifest_identity: string
  stack_frame_valid: boolean
  context_valid: boolean
  control_sequence: number
  state_epoch: number
  safety_state: number
  operation: number
  active_irq: number
  stacked: Record<string, number>
  fault_status: Record<string, number>
  critical_events: UsbFaultEvent[]
  symbolization?: {
    reliable: boolean
    elf?: string
    error?: string
    locations?: Record<string, { function: string, file?: string, line?: number, resolved: boolean }>
  }
}

export interface UsbTestSession {
  state: number
  flags: number
  reason: number
  session_id: number
  lease_remaining_ms: number
  velocity_limit: number
  current_limit: number
  torque_limit: number
  motion: boolean
  hardware_estop_confirmed: boolean
  stop_confirmed?: boolean
  ok?: boolean
}

const MAX_SAMPLES = 2000
const MAX_FAULTS = 128
const MAX_TIMELINE = 256
const MAX_CRASHES = 8

let singleton: ReturnType<typeof createUsbDiagnostics> | null = null

function createUsbDiagnostics() {
  const connected = ref(false)
  const handshakeComplete = ref(false)
  const status = ref<Record<string, unknown>>({})
  const scopeStatus = ref<UsbScopeStatus | null>(null)
  const samples = ref<UsbSample[]>([])
  const faults = ref<UsbFaultEvent[]>([])
  const timeline = ref<UsbTimelineEvent[]>([])
  const crashes = ref<UsbCrashReport[]>([])
  const testSession = ref<UsbTestSession | null>(null)
  const selectedChannels = ref<number[]>([6, 7, 11, 13, 4])
  const session = ref({
    connectedAt: '', buildId: '', schemaVersion: 0, captureConfig: null as Record<string, unknown> | null,
  })
  const error = ref('')
  let ws: WebSocket | null = null
  let intentionalClose = false

  const channelById = computed(() => new Map(SCOPE_CHANNELS.map((channel) => [channel.id, channel])))

  function pushBounded<T>(items: T[], item: T, limit: number) {
    items.push(item)
    if (items.length > limit) items.splice(0, items.length - limit)
  }

  function handleEvent(event: any) {
    if (event.type === 'usb_status') {
      connected.value = Boolean(event.connected)
      handshakeComplete.value = Boolean(event.handshake_complete)
      status.value = { ...status.value, ...event }
      if (event.handshake_error) error.value = String(event.handshake_error)
    } else if (event.type === 'capabilities') {
      handshakeComplete.value = true
      status.value = { ...status.value, ...event }
      session.value = {
        connectedAt: session.value.connectedAt || new Date().toISOString(),
        buildId: String(event.build_id ?? ''),
        schemaVersion: Number(event.schema_version ?? 0),
        captureConfig: session.value.captureConfig,
      }
    } else if (event.type === 'scope_status') {
      scopeStatus.value = event as UsbScopeStatus
      if (event.applied_config) {
        session.value.captureConfig = { ...event.applied_config }
        error.value = ''
      } else if (event.rejected_config) {
        error.value = 'Scope configuration rejected by firmware'
      }
    } else if (event.type === 'test_session_status') {
      testSession.value = event as UsbTestSession
      if (!event.ok) error.value = `Test Session rejected: 0x${Number(event.reason ?? 0).toString(16)}`
      else error.value = ''
    } else if (event.type === 'command_result') {
      if (event.test_session) testSession.value = event.test_session as UsbTestSession
      if (event.status === 1 || event.status === 3) {
        error.value = `USB command failed: status=${event.status}, reason=0x${Number(event.reason ?? 0).toString(16)}`
      }
    } else if (event.type === 'scope_data') {
      for (const sample of event.samples ?? []) pushBounded(samples.value, sample, MAX_SAMPLES)
    } else if (event.type === 'fault_event' && event.event) {
      pushBounded(faults.value, event.event as UsbFaultEvent, MAX_FAULTS)
    } else if (event.type === 'state_event' && event.event) {
      pushBounded(timeline.value, event.event as UsbTimelineEvent, MAX_TIMELINE)
    } else if (event.type === 'crash_report' && event.crash) {
      pushBounded(crashes.value, event.crash as UsbCrashReport, MAX_CRASHES)
    } else if (event.type === 'error') {
      error.value = String(event.error ?? 'USB error')
    }
  }

  function openWebSocket() {
    if (ws) return
    const protocol = location.protocol === 'https:' ? 'wss' : 'ws'
    ws = new WebSocket(`${protocol}://${location.host}/ws/usb`)
    ws.onopen = () => { error.value = '' }
    ws.onclose = () => {
      ws = null
      if (!intentionalClose) error.value = 'USB WebSocket disconnected'
    }
    ws.onerror = () => { error.value = 'USB WebSocket error' }
    ws.onmessage = (message) => {
      try { handleEvent(JSON.parse(message.data)) } catch { error.value = 'Invalid USB event' }
    }
  }

  async function connect(port = '') {
    intentionalClose = false
    const response = await fetch('/api/usb/connect', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ port: port || null }),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error ?? 'USB connect failed')
    connected.value = true
    handshakeComplete.value = false
    error.value = ''
    session.value.connectedAt = new Date().toISOString()
    openWebSocket()
  }

  async function disconnect() {
    intentionalClose = true
    ws?.close(); ws = null
    await fetch('/api/usb/disconnect', { method: 'POST' }).catch(() => undefined)
    connected.value = false
    handshakeComplete.value = false
  }

  async function postScope(path: string, body?: Record<string, unknown>) {
    const response = await fetch(`/api/usb/scope/${path}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      ...(body ? { body: JSON.stringify(body) } : {}),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error ?? `scope ${path} failed`)
    return payload
  }

  async function configureScope(config: Record<string, unknown>) {
    await postScope('config', config)
  }
  async function armScope() { await postScope('arm', { arm: true }) }
  async function manualTrigger() {
    // SCOPE_ARM action 2 is a diagnostic trigger and never a motor command.
    await postScope('arm', { arm: 2 })
  }
  async function stopScope() { await postScope('stop') }
  async function requestScopeStatus() { await fetch('/api/usb/scope/status') }

  async function postSession(path: string, body?: Record<string, unknown>) {
    const response = await fetch(`/api/usb/session/${path}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      ...(body ? { body: JSON.stringify(body) } : {}),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error ?? `session ${path} failed`)
    return payload
  }
  async function beginTestSession(config: Record<string, unknown>) { await postSession('begin', config) }
  async function stopTestSession() { await postSession('stop') }
  async function endTestSession() { await postSession('end') }

  function clearScope() { samples.value = []; scopeStatus.value = null }

  function csvExport(): string {
    const channels = selectedChannels.value
    const lines = [['control_sequence', 'timestamp_cycles', ...channels.map(String)].join(',')]
    for (const sample of samples.value) {
      lines.push([sample.control_sequence, sample.timestamp_cycles,
        ...channels.map((id) => sample.values[id] ?? '')].join(','))
    }
    return lines.join('\n') + '\n'
  }

  function downloadCsv() {
    const blob = new Blob([csvExport()], { type: 'text/csv;charset=utf-8' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a'); link.href = url; link.download = 'odrive_scope.csv'; link.click()
    URL.revokeObjectURL(url)
  }

  async function downloadBinary() {
    const response = await fetch('/api/usb/export/raw')
    if (!response.ok) throw new Error('USB binary export failed')
    const blob = await response.blob()
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a'); link.href = url; link.download = 'odrive_usb_capture.bin'; link.click()
    URL.revokeObjectURL(url)
  }

  async function downloadReplay() {
    const response = await fetch('/api/usb/export/replay')
    if (!response.ok) throw new Error('USB replay export failed')
    const blob = await response.blob()
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a'); link.href = url; link.download = 'odrive_usb_replay.json'; link.click()
    URL.revokeObjectURL(url)
  }

  function close() { intentionalClose = true; ws?.close(); ws = null }
  onBeforeUnmount(close)

  return {
    connected, handshakeComplete, status, scopeStatus, samples, faults, timeline, crashes,
    testSession,
    selectedChannels, session, error, channelById,
    connect, disconnect, configureScope, armScope, manualTrigger, stopScope,
    requestScopeStatus, clearScope, downloadCsv, downloadBinary, downloadReplay, close,
    beginTestSession, stopTestSession, endTestSession,
  }
}

export function useUsbDiagnostics() {
  if (!singleton) singleton = createUsbDiagnostics()
  return singleton
}

export { SCOPE_CHANNELS }
export type { ScopeChannel }
