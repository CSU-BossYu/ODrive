// Single WebSocket connection manager + reactive state, exposed as a
// composable. There is exactly one socket per page; multiple components
// subscribe to the reactive refs it maintains.
//
// The socket auto-reconnects with exponential backoff (capped at 3 s). All
// inbound messages are validated loosely and dispatched into typed reactive
// containers:
//   - latest telemetry frame -> `latest.value` (TelemetryMsg | null)
//   - status -> `status.value`
//   - logs -> circular buffer `logs.value` (max 1000 lines)
//   - cli lines -> circular buffer `cliLines.value` (max 500 lines)
//
// Outbound helpers: sendCmd, sendSet, sendRawCli, recStart, recStop.

import { ref, type Ref } from 'vue'
import type {
  InboundMsg, OutboundMsg, StatusMsg, TelemetryMsg, LogMsg, CliMsg,
} from '../types'

const MAX_LOGS = 1000
const MAX_CLI = 500

interface FocSocket {
  // connection
  connected: Ref<boolean>
  ready: Ref<boolean>           // WS open AND backend serial link connected
  status: Ref<StatusMsg | null>
  // data
  latest: Ref<TelemetryMsg | null>
  logs: Ref<LogMsg[]>
  cliLines: Ref<CliMsg[]>
  cmdResult: Ref<{ is_error: boolean; text: string } | null>
  // controls
  connect: () => void
  disconnect: () => void
  send: (m: OutboundMsg) => void
  sendCmd: (text: string) => void
  sendSet: (key: string, value: number | [number, number]) => void
  sendRawCli: (text: string) => void
  recStart: (path?: string) => void
  recStop: () => void
  clearLogs: () => void
}

let _singleton: FocSocket | null = null

export function useFocSocket(): FocSocket {
  if (_singleton) return _singleton

  const connected = ref(false)
  const ready = ref(false)
  const status = ref<StatusMsg | null>(null)
  const latest = ref<TelemetryMsg | null>(null)
  const logs = ref<LogMsg[]>([])
  const cliLines = ref<CliMsg[]>([])
  const cmdResult = ref<{ is_error: boolean; text: string } | null>(null)

  let ws: WebSocket | null = null
  let reconnectTimer: number | null = null
  let backoffMs = 200
  let manualClose = false

  function pushLog(arr: LogMsg[], item: LogMsg) {
    arr.push(item)
    if (arr.length > MAX_LOGS) arr.splice(0, arr.length - MAX_LOGS)
  }
  function pushCli(item: CliMsg) {
    cliLines.value.push(item)
    if (cliLines.value.length > MAX_CLI) cliLines.value.splice(0, cliLines.value.length - MAX_CLI)
  }

  function openSocket() {
    // Same-origin in prod (backend serves the page); dev uses Vite proxy at
    // /ws. Either way, a relative URL works.
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    const url = `${proto}://${location.host}/ws`
    try {
      ws = new WebSocket(url)
    } catch (e) {
      scheduleReconnect()
      return
    }

    ws.onopen = () => {
      connected.value = true
      backoffMs = 200
    }
    ws.onclose = () => {
      connected.value = false
      ready.value = false
      ws = null
      if (!manualClose) scheduleReconnect()
    }
    ws.onerror = () => {
      // onclose will follow; just suppress console noise.
    }
    ws.onmessage = (ev) => {
      let msg: InboundMsg
      try {
        msg = JSON.parse(ev.data)
      } catch {
        return
      }
      dispatch(msg)
    }
  }

  function scheduleReconnect() {
    if (reconnectTimer != null) return
    reconnectTimer = window.setTimeout(() => {
      reconnectTimer = null
      openSocket()
    }, backoffMs)
    backoffMs = Math.min(backoffMs * 2, 3000)
  }

  function dispatch(msg: InboundMsg) {
    switch (msg.type) {
      case 'telemetry':
        latest.value = msg
        break
      case 'status':
        status.value = msg
        ready.value = msg.connected
        break
      case 'log':
        pushLog(logs.value, msg)
        break
      case 'cli':
        pushCli(msg)
        break
      case 'cmd_done':
        cmdResult.value = { is_error: msg.is_error, text: msg.text }
        break
      case 'rec_state':
        // Surface as a synthetic log line so the user sees recording state.
        pushLog(logs.value, {
          type: 'log',
          t: Date.now(),
          tag: 'REC',
          text: msg.recording ? `recording -> ${msg.path}` : `stopped (wrote ${msg.path})`,
        })
        break
      case 'error':
        pushLog(logs.value, {
          type: 'log',
          t: Date.now(),
          tag: 'ERR',
          text: msg.msg,
        })
        break
    }
  }

  function connect() {
    manualClose = false
    if (ws == null && reconnectTimer == null) openSocket()
  }
  function disconnect() {
    manualClose = true
    if (reconnectTimer != null) {
      clearTimeout(reconnectTimer)
      reconnectTimer = null
    }
    if (ws) {
      ws.close()
      ws = null
    }
  }

  function send(m: OutboundMsg) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(m))
    }
  }
  function sendCmd(text: string) { send({ type: 'cmd', text }) }
  function sendSet(key: string, value: number | [number, number]) { send({ type: 'set', key, value }) }
  function sendRawCli(text: string) { send({ type: 'rawcli', text }) }
  function recStart(path?: string) { send({ type: 'rec', action: 'start', path }) }
  function recStop() { send({ type: 'rec', action: 'stop' }) }
  function clearLogs() { logs.value = []; cliLines.value = [] }

  _singleton = {
    connected, ready, status, latest, logs, cliLines, cmdResult,
    connect, disconnect, send, sendCmd, sendSet, sendRawCli, recStart, recStop, clearLogs,
  }
  // Auto-connect on first use.
  connect()
  return _singleton
}
