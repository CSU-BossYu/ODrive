// ODrive CAN WebSocket connection manager + reactive state.
//
// Singleton composable connecting to /ws/can. Handles ODrive-specific
// message types: telemetry (14-ch), heartbeat, status, ext_resp, log, error.

import { ref, type Ref } from 'vue'
import type {
  ODriveInboundMsg, ODriveOutboundMsg,
  ODriveStatusMsg, ODriveTelemetryMsg, ODriveHeartbeatMsg,
  ODriveExtRespMsg, LogMsg,
  AnticoggingConfigOut,
} from '../types'

const MAX_LOGS = 1000
const MAX_EXT = 100
// Firmware CAN extended-protocol version this UI expects (device info 0x05/0x01).
export const EXPECTED_PROTOCOL_VERSION = 0x00000103

interface ODriveSocket {
  connected: Ref<boolean>
  ready: Ref<boolean>
  status: Ref<ODriveStatusMsg | null>
  latest: Ref<ODriveTelemetryMsg | null>
  heartbeat: Ref<ODriveHeartbeatMsg | null>
  logs: Ref<LogMsg[]>
  extResponses: Ref<ODriveExtRespMsg[]>
  extSeq: Ref<number>
  recording: Ref<boolean>
  overspeedSnapshot: Ref<Record<string, number | null> | null>
  controlConfig: Ref<Record<string, number | null> | null>
  userConfigLoaded: Ref<number | null>
  protocolVersion: Ref<number | null>
  connect: () => void
  disconnect: () => void
  send: (m: ODriveOutboundMsg) => void
  setState: (state: number) => void
  setMode: (ctrl: number, inp: number) => void
  setPos: (pos: number, vel_ff?: number, torque_ff?: number) => void
  setVel: (vel: number, torque_ff?: number) => void
  setTorque: (torque: number) => void
  sendMit: (p: number, v: number, kp: number, kd: number, t: number) => void
  setGain: (name: string, value: number, extra?: Record<string, number>) => void
  setLimits: (vel: number, cur: number) => void
  clearErrors: () => void
  estop: () => void
  reboot: () => void
  extCmd: (sub: number, item: number, type?: number, value?: number, timeout?: number) => void
  getControlConfig: () => void
  setControlConfig: (item: number, value: number, isFloat?: boolean) => void
  setServoMode: (mode: number) => void
  anticoggingStart: () => void
  anticoggingStatus: () => void
  anticoggingConfig: (cfg: Partial<AnticoggingConfigOut>) => void
  setPollHz: (hz: number) => void
  recStart: (path?: string) => void
  recStop: () => void
  getOverspeedSnapshot: () => void
  fetchUserConfigLoaded: () => void
  fetchProtocolVersion: () => void
  clearLogs: () => void
}

let _singleton: ODriveSocket | null = null

export function useOdriveSocket(): ODriveSocket {
  if (_singleton) return _singleton

  const connected = ref(false)
  const ready = ref(false)
  const status = ref<ODriveStatusMsg | null>(null)
  const latest = ref<ODriveTelemetryMsg | null>(null)
  const heartbeat = ref<ODriveHeartbeatMsg | null>(null)
  const logs = ref<LogMsg[]>([])
  const extResponses = ref<ODriveExtRespMsg[]>([])
  // Monotonic counter incremented on every ext_resp. Watching .length stops
  // firing once the ring buffer is full (push+shift keeps length constant),
  // so AnticoggingPanel watches this instead.
  const extSeq = ref(0)
  // Recording state mirrored from backend rec_state messages so the UI stays
  // in sync with what the backend is actually doing (not just what we asked).
  const recording = ref(false)
  // OverspeedSnapshot captured by firmware at the last OVERSPEED fault.
  // null until a get_overspeed_snapshot request returns.
  const overspeedSnapshot = ref<Record<string, number | null> | null>(null)
  const controlConfig = ref<Record<string, number | null> | null>(null)
  // NVM bytes loaded on boot (device info item 0x06). 0 = load failed, the
  // firmware is running factory defaults. null = not yet queried.
  const userConfigLoaded = ref<number | null>(null)
  // Firmware CAN extended-protocol version (device info item 0x01).
  // null = not yet queried.
  const protocolVersion = ref<number | null>(null)

  let ws: WebSocket | null = null
  let reconnectTimer: number | null = null
  let backoffMs = 200
  let manualClose = false
  let keepaliveTimer: number | null = null

  function pushLog(arr: LogMsg[], item: LogMsg) {
    arr.push(item)
    if (arr.length > MAX_LOGS) arr.splice(0, arr.length - MAX_LOGS)
  }

  function openSocket() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    const url = `${proto}://${location.host}/ws/can`
    try {
      ws = new WebSocket(url)
    } catch {
      scheduleReconnect()
      return
    }

    ws.onopen = () => {
      connected.value = true
      backoffMs = 200
      // Keepalive: ping every 2s feeds the backend WS watchdog so the motor
      // isn't disarmed to IDLE while the user is just watching it run (the
      // watchdog fires after WS_WATCHDOG_S with no inbound WS message). A
      // frozen/crashed tab stops pinging -> backend safety-stops after the
      // timeout; a clean tab close is caught by onclose immediately.
      if (keepaliveTimer == null) {
        keepaliveTimer = window.setInterval(() => {
          if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'ping' }))
          }
        }, 2000)
      }
    }
    ws.onclose = () => {
      connected.value = false
      ready.value = false
      // Clear stale readings so the UI doesn't keep showing a "live" motor
      // (vbus/vel/Iq/axis state) after the backend has gone away.
      latest.value = null
      status.value = null
      heartbeat.value = null
      recording.value = false
      overspeedSnapshot.value = null
      controlConfig.value = null
      userConfigLoaded.value = null
      protocolVersion.value = null
      if (keepaliveTimer != null) {
        window.clearInterval(keepaliveTimer)
        keepaliveTimer = null
      }
      ws = null
      if (!manualClose) scheduleReconnect()
    }
    ws.onerror = () => {}
    ws.onmessage = (ev) => {
      let msg: ODriveInboundMsg
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

  function dispatch(msg: ODriveInboundMsg) {
    switch (msg.type) {
      case 'telemetry':
        latest.value = msg
        break
      case 'heartbeat':
        heartbeat.value = msg
        break
      case 'status': {
        const wasReady = ready.value
        status.value = msg
        ready.value = msg.connected
        // On (re)connect, fetch whether the firmware loaded a saved config
        // and the protocol version it speaks.
        if (msg.connected && !wasReady) {
          fetchUserConfigLoaded()
          fetchProtocolVersion()
        }
        break
      }
      case 'log':
        pushLog(logs.value, msg)
        break
      case 'ext_resp':
        extResponses.value.push(msg)
        if (extResponses.value.length > MAX_EXT)
          extResponses.value.shift()
        extSeq.value++
        // Device info: item 0x01 = protocol_version, item 0x06 = user_config_loaded.
        if (msg.sub_cmd === 0x05 && msg.item === 0x01) {
          protocolVersion.value = msg.status === 0 ? msg.value : null
        } else if (msg.sub_cmd === 0x05 && msg.item === 0x06) {
          userConfigLoaded.value = msg.status === 0 ? msg.value : 0
        }
        break
      case 'rec_state':
        recording.value = msg.recording
        break
      case 'overspeed_snapshot':
        overspeedSnapshot.value = msg.snapshot
        break
      case 'control_config':
        controlConfig.value = msg.config
        break
      case 'pong':
        // Backend keepalive reply; liveness is implied by telemetry flow.
        break
      case 'error':
        pushLog(logs.value, {
          type: 'log', t: Date.now(), tag: 'ERR', text: msg.msg,
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

  function send(m: ODriveOutboundMsg): boolean {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(m))
      return true
    }
    return false
  }
  function setState(state: number) { send({ type: 'set_state', state }) }
  function setMode(ctrl: number, inp: number) {
    send({ type: 'set_mode', control_mode: ctrl, input_mode: inp })
  }
  function setPos(pos: number, vel_ff?: number, torque_ff?: number) {
    send({ type: 'set_pos', pos, vel_ff, torque_ff })
  }
  function setVel(vel: number, torque_ff?: number) {
    send({ type: 'set_vel', vel, torque_ff })
  }
  function setTorque(torque: number) {
    send({ type: 'set_torque', torque })
  }
  function sendMit(p: number, v: number, kp: number, kd: number, t: number) {
    send({ type: 'mit', p_des: p, v_des: v, kp, kd, t_ff: t })
  }
  function setGain(name: string, value: number, extra?: Record<string, number>) {
    send({ type: 'set_gain', name, value, ...extra })
  }
  function setLimits(vel: number, cur: number) {
    send({ type: 'set_limits', vel_limit: vel, current_limit: cur })
  }
  function clearErrors() { send({ type: 'clear_errors' }) }
  function estop() { send({ type: 'estop' }) }
  function reboot() { send({ type: 'reboot' }) }
  function extCmd(sub: number, item: number, type?: number, value?: number, timeout?: number) {
    send({ type: 'ext_cmd', sub_cmd: sub, item, ext_type: type, value, timeout })
  }
  function getControlConfig() { send({ type: 'get_control_config' }) }
  function setControlConfig(item: number, value: number, isFloat = false) {
    send({ type: 'set_control_config', item, value, is_float: isFloat })
  }
  function setServoMode(mode: number) { send({ type: 'set_servo_mode', mode }) }
  function anticoggingStart() { send({ type: 'anticogging_start' }) }
  function anticoggingStatus() { send({ type: 'anticogging_status' }) }
  function anticoggingConfig(cfg: Partial<AnticoggingConfigOut>) {
    send({ type: 'anticogging_config', ...cfg })
  }
  function setPollHz(hz: number) { send({ type: 'set_poll_hz', hz }) }
  function recStart(path?: string) { send({ type: 'rec', action: 'start', path }) }
  function recStop() { send({ type: 'rec', action: 'stop' }) }
  function getOverspeedSnapshot() { send({ type: 'get_overspeed_snapshot' }) }
  function fetchUserConfigLoaded() { extCmd(0x05, 0x06) }
  function fetchProtocolVersion() { extCmd(0x05, 0x01) }
  function clearLogs() { logs.value = []; extResponses.value = [] }

  _singleton = {
    connected, ready, status, latest, heartbeat, logs, extResponses,
    extSeq, recording, overspeedSnapshot, controlConfig, userConfigLoaded, protocolVersion,
    connect, disconnect, send,
    setState, setMode, setPos, setVel, setTorque, sendMit,
    setGain, setLimits, clearErrors, estop, reboot,
    extCmd, getControlConfig, setControlConfig, setServoMode,
    anticoggingStart, anticoggingStatus, anticoggingConfig,
    setPollHz, recStart, recStop, getOverspeedSnapshot, fetchUserConfigLoaded, fetchProtocolVersion, clearLogs,
  }
  connect()
  return _singleton
}
