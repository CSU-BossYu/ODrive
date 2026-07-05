"""ODrive service layer: orchestration, polling, commands, and safety.

Central service tying transport, state cache, polling, command methods,
and safety monitoring together. This is the CAN equivalent of the serial
backend's serial_link.py + cli_responder.py combined.

Responsibilities:
  - Route incoming CAN frames to the state cache
  - Periodically poll for encoder/Iq/bus estimates
  - Provide high-level command methods with safety clamping
  - Correlate extended command request-response pairs
  - Monitor WebSocket liveness and MIT watchdog for safety shutdown
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any, Awaitable, Callable, Optional

from .protocol import (
    CmdId, AxisState, ControlMode, InputMode,
    ExtSubCmd, ExtStatus, ExtType,
    decode_heartbeat, decode_encoder_estimates, decode_iq, decode_bus_vi,
    decode_motor_error, decode_encoder_error, decode_controller_error,
    decode_extended_response,
    encode_set_axis_state, encode_set_controller_mode,
    encode_set_input_pos, encode_set_input_vel, encode_set_input_torque,
    encode_set_limits, encode_set_pos_gain, encode_set_vel_gains,
    encode_mit_control, encode_mit_neutral,
    encode_extended_request,
    OVERSPEED_SNAPSHOT_ITEMS,
    CONTROL_CONFIG_ITEMS,
    ServoControlMode,
)
from .state import AxisCache
from .transport import CanTransport

logger = logging.getLogger('odrive_can.service')

# --------------------------------------------------------------------------- #
# Safety Constants
# --------------------------------------------------------------------------- #

TORQUE_LIMIT: float = 0.005         # ±0.005 Nm initial test range
# MIT_TORQUE_LIMIT: The MIT quantization step is 36.0/4095 ≈ 0.00879 Nm.
# A limit of 0.005 would always quantize to zero. Set to one step (0.009)
# to allow minimum nonzero MIT t_ff while keeping the range safe for bench test.
MIT_TORQUE_LIMIT: float = 0.009     # ±0.009 Nm MIT t_ff (one quantization step)
CURRENT_LIMIT: float = 3.0          # 3 A max initial
# Bench-safe ceilings for the firmware's own safety limits and for setpoints.
# These exist so a malformed/attacked WS message cannot disable the firmware
# overspeed/current protection or command absurd motion.
VEL_LIMIT_MAX: float = 30.0         # turns/s ceiling for vel_limit and vel setpoints
POS_GAIN_MAX: float = 1000.0
POS_INTEGRATOR_MAX: float = 1000.0
VEL_GAIN_MAX: float = 100.0
VEL_INTEGRATOR_MAX: float = 100.0
MIT_KP_MAX: float = 500.0           # MIT protocol quantization max
MIT_KD_MAX: float = 5.0
WS_WATCHDOG_S: float = 10.0         # 10s no WS keepalive -> safety stop
MIT_WATCHDOG_S: float = 0.3         # 300ms no MIT frame -> neutral
POLL_HZ_DEFAULT: float = 20.0       # Default polling rate
POLL_HZ_MIN: float = 5.0
POLL_HZ_MAX: float = 100.0
ERROR_DETAIL_INTERVAL_S: float = 1.0  # Min interval between error detail requests


def _fclamp(value: float, lo: float, hi: float) -> float:
    """Clamp to [lo, hi]; NaN/Inf map to lo so non-finite WS payloads can't
    reach the firmware as a raw float32 (which the controller may interpret
    unpredictably)."""
    if not math.isfinite(value):
        return lo
    return max(lo, min(value, hi))


# Callback types
TelemetryCallback = Callable[[dict, float], Awaitable[None]]
HeartbeatCallback = Callable[[dict, float], Awaitable[None]]
LogCallback = Callable[[str], None]


class ODriveService:
    """Orchestrates CAN communication with a single ODrive axis."""

    def __init__(self, transport: CanTransport, node_id: int = 0):
        self.transport = transport
        self.node_id = node_id
        self.cache = AxisCache()

        # Polling
        self._poll_task: Optional[asyncio.Task] = None
        self._poll_hz: float = POLL_HZ_DEFAULT

        # Extended command correlation
        self._ext_pending: dict[tuple[int, int], asyncio.Future] = {}
        self._ext_lock = asyncio.Lock()

        # Safety monitoring
        self._safety_task: Optional[asyncio.Task] = None
        self._ws_connected: bool = False
        self._last_ws_msg_ts: float = 0.0

        # MIT state
        self._mit_active: bool = False
        self._mit_last_ts: float = 0.0

        # Master heartbeat sender (feeds firmware inbound heartbeat watchdog)
        self._last_heartbeat_ts: float = 0.0

        # Last-set gains (so setting one doesn't zero the other)
        self._last_vel_gain: float = 0.0
        self._last_vel_integrator_gain: float = 0.0
        self._last_vel_limit: float = 0.0

        # Error detail request throttling (prevent CAN bus flood on persistent faults)
        self._last_error_req_ts: dict[int, float] = {}  # cmd_id -> monotonic ts
        self._last_system_error_req_ts: float = 0.0

        # Callbacks (set by main.py)
        self.on_telemetry_update: Optional[TelemetryCallback] = None
        self.on_heartbeat_update: Optional[HeartbeatCallback] = None
        self.on_log: Optional[LogCallback] = None

    # ------------------------------------------------------------------ #
    # Frame Dispatch  (called by transport.pump via on_frame callback)
    # ------------------------------------------------------------------ #

    async def on_frame(self, cmd_id: int, data: bytes, ts: float) -> None:
        """Route a received CAN frame to the appropriate cache updater."""
        if cmd_id == -1:
            # Error from transport reader (data is error text)
            if self.on_log:
                self.on_log(f'[CAN-ERR] {data.decode("ascii", errors="replace")}')
            return

        if cmd_id == CmdId.HEARTBEAT:
            hb = decode_heartbeat(data)
            if not hb:
                return
            self.cache.heartbeat.axis_error = hb['axis_error']
            self.cache.heartbeat.axis_state = hb['axis_state']
            self.cache.heartbeat.motor_err_flag = hb['motor_err_flag']
            self.cache.heartbeat.encoder_err_flag = hb['encoder_err_flag']
            self.cache.heartbeat.controller_err_flag = hb['controller_err_flag']
            self.cache.heartbeat.comm_timeout = hb.get('comm_timeout', False)
            self.cache.heartbeat.quick_stop_active = hb.get('quick_stop_active', False)
            self.cache.heartbeat.holding = hb.get('holding', False)
            self.cache.heartbeat.cmd_watchdog_expired = hb.get('cmd_watchdog_expired', False)
            self.cache.heartbeat.mit_frame_stale = hb.get('mit_frame_stale', False)
            self.cache.heartbeat.running = hb.get('running', False)
            self.cache.heartbeat.trajectory_done = hb['traj_done']
            self.cache.heartbeat.controller_flags = hb.get('controller_flags', 0)
            self.cache.heartbeat.last_ts = ts

            if self.on_heartbeat_update:
                await self.on_heartbeat_update(hb, ts)

            # If error flags are set, request detailed error registers
            # (throttled to prevent CAN bus flood on persistent faults)
            if hb['motor_err_flag']:
                await self._request_error_detail(CmdId.GET_MOTOR_ERROR)
            if hb['encoder_err_flag']:
                await self._request_error_detail(CmdId.GET_ENCODER_ERROR)
            if hb['controller_err_flag']:
                await self._request_error_detail(CmdId.GET_CONTROLLER_ERROR)

        elif cmd_id == CmdId.GET_ENCODER_ESTIMATES:
            d = decode_encoder_estimates(data)
            if d:
                self.cache.encoder.pos_estimate = d['pos_estimate']
                self.cache.encoder.vel_estimate = d['vel_estimate']
                self.cache.encoder.last_ts = ts

        elif cmd_id == CmdId.GET_IQ:
            d = decode_iq(data)
            if d:
                self.cache.iq.iq_setpoint = d['iq_setpoint']
                self.cache.iq.iq_measured = d['iq_measured']
                self.cache.iq.last_ts = ts

        elif cmd_id == CmdId.GET_BUS_VOLTAGE_CURRENT:
            d = decode_bus_vi(data)
            if d:
                self.cache.bus.vbus = d['vbus']
                self.cache.bus.ibus = d['ibus']
                self.cache.bus.last_ts = ts

        elif cmd_id == CmdId.GET_MOTOR_ERROR:
            self.cache.errors.motor_error = decode_motor_error(data)
            self.cache.errors.last_ts = ts

        elif cmd_id == CmdId.GET_ENCODER_ERROR:
            self.cache.errors.encoder_error = decode_encoder_error(data)
            self.cache.errors.last_ts = ts

        elif cmd_id == CmdId.GET_CONTROLLER_ERROR:
            self.cache.errors.controller_error = decode_controller_error(data)
            self.cache.errors.last_ts = ts

        elif cmd_id == CmdId.EXTENDED_COMMAND:
            await self._handle_ext_response(data, ts)

    async def _request_error_detail(self, cmd_id: CmdId) -> None:
        """Send a request for a specific error register (throttled, fire-and-forget).

        Throttled to ERROR_DETAIL_INTERVAL_S (1s) per cmd_id to prevent CAN
        bus flooding when error flags stay set for long durations (e.g., a
        persistent motor fault produces heartbeats at 10-100 Hz, each with
        the error flag set).
        """
        now = time.monotonic()
        last_ts = self._last_error_req_ts.get(int(cmd_id), 0.0)
        if (now - last_ts) < ERROR_DETAIL_INTERVAL_S:
            return
        self._last_error_req_ts[int(cmd_id)] = now
        try:
            await asyncio.get_running_loop().run_in_executor(
                None, self.transport.send_request, cmd_id)
        except Exception as exc:
            logger.debug('error detail request failed: %s', exc)

    # ------------------------------------------------------------------ #
    # Polling
    # ------------------------------------------------------------------ #

    async def start_polling(self, hz: float = POLL_HZ_DEFAULT) -> None:
        """Start the periodic polling loop."""
        self._poll_hz = _fclamp(hz, POLL_HZ_MIN, POLL_HZ_MAX)
        if self._poll_task is not None:
            self._poll_task.cancel()
        self._poll_task = asyncio.create_task(self._poll_loop())

    async def stop_polling(self) -> None:
        """Stop the periodic polling loop."""
        if self._poll_task is not None:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except asyncio.CancelledError:
                pass
            self._poll_task = None

    async def _poll_loop(self) -> None:
        """Periodically request encoder/Iq/bus data and emit telemetry."""
        interval = 1.0 / self._poll_hz
        third = interval / 3.0

        while True:
            try:
                # Stagger requests to spread bus load
                await self._send_request(CmdId.GET_ENCODER_ESTIMATES)
                await asyncio.sleep(third)

                await self._send_request(CmdId.GET_IQ)
                await asyncio.sleep(third)

                await self._send_request(CmdId.GET_BUS_VOLTAGE_CURRENT)
                await asyncio.sleep(third)

                # Emit synthesized telemetry at poll rate
                if self.cache.heartbeat.motor_err_flag:
                    await self._request_system_error_detail()

                if self.on_telemetry_update:
                    await self.on_telemetry_update(
                        self.cache.synthesize_telemetry(), time.monotonic())

            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.debug('poll loop error: %s', exc)
                await asyncio.sleep(interval)

    async def _send_request(self, cmd_id: CmdId) -> None:
        """Send a zero-length request (asks firmware to respond with data)."""
        try:
            await asyncio.get_running_loop().run_in_executor(
                None, self.transport.send_request, cmd_id)
        except Exception as exc:
            logger.debug('send_request %s failed: %s', cmd_id, exc)

    async def _request_system_error_detail(self) -> None:
        now = time.monotonic()
        if (now - self._last_system_error_req_ts) < ERROR_DETAIL_INTERVAL_S:
            return
        self._last_system_error_req_ts = now
        resp = await self.ext_command(ExtSubCmd.GET_ANTICOGGING_STATUS, 0x06,
                                      ExtType.UINT32, 0, timeout=0.25)
        if resp.get('status') == ExtStatus.OK:
            self.cache.errors.odrive_error = int(resp.get('value', 0))

    # ------------------------------------------------------------------ #
    # Command Methods  (all clamp dangerous values for safety)
    # ------------------------------------------------------------------ #

    async def set_axis_state(self, state: AxisState | int) -> None:
        data = encode_set_axis_state(state)
        await self._send_cmd(CmdId.SET_AXIS_STATE, data)
        if self.on_log:
            try:
                name = AxisState(int(state)).name
            except ValueError:
                name = str(state)
            self.on_log(f'[CMD] set_axis_state -> {name}')

    async def set_controller_mode(self, ctrl: ControlMode | int,
                                  inp: InputMode | int) -> None:
        data = encode_set_controller_mode(ctrl, inp)
        await self._send_cmd(CmdId.SET_CONTROLLER_MODE, data)
        self.cache.control.control_mode = int(ctrl)
        self.cache.control.input_mode = int(inp)
        self.cache.control.last_ts = time.monotonic()
        if self.on_log:
            try:
                cn = ControlMode(int(ctrl)).name
            except ValueError:
                cn = str(ctrl)
            try:
                inn = InputMode(int(inp)).name
            except ValueError:
                inn = str(inp)
            self.on_log(f'[CMD] set_controller_mode -> {cn} / {inn}')

    async def set_input_pos(self, pos: float, vel_ff: float = 0.0,
                            torque_ff: float = 0.0) -> None:
        """Set position input. torque_ff clamped to ±TORQUE_LIMIT, vel_ff to
        ±VEL_LIMIT_MAX. pos is not range-limited (multi-turn) but NaN/Inf are
        rejected to avoid a garbage float32 reaching the firmware."""
        if not math.isfinite(pos):
            return
        vel_ff = _fclamp(vel_ff, -VEL_LIMIT_MAX, VEL_LIMIT_MAX)
        torque_ff = _fclamp(torque_ff, -TORQUE_LIMIT, TORQUE_LIMIT)
        data = encode_set_input_pos(pos, vel_ff, torque_ff)
        await self._send_cmd(CmdId.SET_INPUT_POS, data)

    async def set_input_vel(self, vel: float, torque_ff: float = 0.0) -> None:
        """Set velocity input. vel clamped to ±VEL_LIMIT_MAX, torque_ff to
        ±TORQUE_LIMIT."""
        vel = _fclamp(vel, -VEL_LIMIT_MAX, VEL_LIMIT_MAX)
        torque_ff = _fclamp(torque_ff, -TORQUE_LIMIT, TORQUE_LIMIT)
        data = encode_set_input_vel(vel, torque_ff)
        await self._send_cmd(CmdId.SET_INPUT_VEL, data)

    async def set_input_torque(self, torque: float) -> None:
        """Set torque input. Clamped to ±TORQUE_LIMIT."""
        torque = _fclamp(torque, -TORQUE_LIMIT, TORQUE_LIMIT)
        data = encode_set_input_torque(torque)
        await self._send_cmd(CmdId.SET_INPUT_TORQUE, data)

    async def set_limits(self, vel_limit: float, current_limit: float) -> None:
        """Set velocity and current limits. Both clamped to a safe bench range
        so a malformed WS message cannot disable the firmware's own overspeed /
        current protection."""
        vel_limit = _fclamp(vel_limit, 0.0, VEL_LIMIT_MAX)
        current_limit = _fclamp(current_limit, 0.0, CURRENT_LIMIT)
        self._last_vel_limit = vel_limit
        data = encode_set_limits(vel_limit, current_limit)
        await self._send_cmd(CmdId.SET_LIMITS, data)
        await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, 0x55,
                               ExtType.FLOAT32, vel_limit)
        if self.on_log:
            self.on_log(f'[CMD] set_limits -> vel={vel_limit:.2f} rev/s, '
                        f'cur={current_limit:.2f} A, profile_vel<=vel')

    async def set_pos_gain(self, gain: float) -> None:
        gain = _fclamp(gain, 0.0, POS_GAIN_MAX)
        data = encode_set_pos_gain(gain)
        await self._send_cmd(CmdId.SET_POS_GAIN, data)

    async def set_pos_integrator_gain(self, gain: float) -> None:
        gain = _fclamp(gain, 0.0, POS_INTEGRATOR_MAX)
        await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, 0x5D,
                               ExtType.FLOAT32, gain)

    async def set_vel_gains(self, gain: float, integrator_gain: float) -> None:
        gain = _fclamp(gain, 0.0, VEL_GAIN_MAX)
        integrator_gain = _fclamp(integrator_gain, 0.0, VEL_INTEGRATOR_MAX)
        data = encode_set_vel_gains(gain, integrator_gain)
        await self._send_cmd(CmdId.SET_VEL_GAINS, data)
        # Track last-set values so setting one alone doesn't zero the other
        self._last_vel_gain = gain
        self._last_vel_integrator_gain = integrator_gain

    async def set_mit_control(self, p_des: float, v_des: float,
                              kp: float, kd: float, t_ff: float) -> None:
        """Send MIT control frame. t_ff clamped to ±MIT_TORQUE_LIMIT; kp/kd
        clamped to bench-safe ceilings. Note the MIT torque limit only clamps
        the feedforward t_ff, not the PD term kp*error, so kp must be bounded
        here."""
        p_des = _fclamp(p_des, -1e6, 1e6)
        v_des = _fclamp(v_des, -VEL_LIMIT_MAX, VEL_LIMIT_MAX)
        kp = _fclamp(kp, 0.0, MIT_KP_MAX)
        kd = _fclamp(kd, 0.0, MIT_KD_MAX)
        t_ff = _fclamp(t_ff, -MIT_TORQUE_LIMIT, MIT_TORQUE_LIMIT)
        data = encode_mit_control(p_des, v_des, kp, kd, t_ff)
        await self._send_cmd(CmdId.SET_MIT_CONTROL, data)
        self._mit_active = True
        self._mit_last_ts = time.monotonic()

    async def clear_errors(self) -> None:
        await self._send_cmd(CmdId.CLEAR_ERRORS, b'')
        # Clear cached error codes immediately. Otherwise the UI keeps showing
        # a stale motor/encoder/controller error until the next heartbeat
        # re-flags it (which never happens once the fault is actually cleared),
        # making it look like the motor is still faulted after Clear Errors.
        self.cache.errors.motor_error = 0
        self.cache.errors.odrive_error = 0
        self.cache.errors.encoder_error = 0
        self.cache.errors.controller_error = 0
        self.cache.heartbeat.axis_error = 0
        if self.on_log:
            self.on_log('[CMD] clear_errors')

    async def estop(self) -> None:
        await self._send_cmd(CmdId.ESTOP, b'')
        if self.on_log:
            self.on_log('[CMD] estop')

    async def reboot(self) -> None:
        await self._send_cmd(CmdId.REBOOT, b'')
        if self.on_log:
            self.on_log('[CMD] reboot')

    async def start_anticogging(self) -> None:
        await self._send_cmd(CmdId.START_ANTICOGGING, b'')
        if self.on_log:
            self.on_log('[CMD] start_anticogging')

    async def _send_cmd(self, cmd_id: CmdId | int, data: bytes) -> None:
        """Send a CAN command via executor to avoid blocking the event loop."""
        await asyncio.get_running_loop().run_in_executor(
            None, self.transport.send, cmd_id, data)

    # ------------------------------------------------------------------ #
    # Extended Command Correlation
    # ------------------------------------------------------------------ #

    async def ext_command(self, sub_cmd: int, item: int,
                          ext_type: int = 0, value: Any = 0,
                          timeout: float = 1.0) -> dict:
        """Send an extended command and await the correlated response.

        Returns the decoded response dict, or a timeout/error dict.
        """
        key = (sub_cmd, item)
        async with self._ext_lock:
            if key in self._ext_pending:
                return {'status': -1, 'error': 'already_pending',
                        'sub_cmd': sub_cmd, 'item': item}
            fut = asyncio.get_running_loop().create_future()
            self._ext_pending[key] = fut

        data = encode_extended_request(sub_cmd, item, ext_type, value)
        try:
            await self._send_cmd(CmdId.EXTENDED_COMMAND, data)
        except Exception as exc:
            self._ext_pending.pop(key, None)
            return {'status': -1, 'error': str(exc),
                    'sub_cmd': sub_cmd, 'item': item}

        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            return {'status': -1, 'error': 'timeout',
                    'sub_cmd': sub_cmd, 'item': item}
        finally:
            self._ext_pending.pop(key, None)

    async def read_overspeed_snapshot(self) -> dict:
        """Read the OverspeedSnapshot captured at the last overspeed fault.

        Firmware captures the full controller/encoder/resolver state at the
        instant of ERROR_OVERSPEED and holds it until clear_errors. Exposed
        via ext sub_cmd 0x0A items 0x40-0x5F. Returns a flat dict; fields are
        None if the firmware didn't respond for that item. 'valid' is 0 when
        no overspeed fault has been captured.
        """
        snap: dict = {}
        for item, name, _is_float in OVERSPEED_SNAPSHOT_ITEMS:
            resp = await self.ext_command(ExtSubCmd.GET_VERNIER_DIAGNOSTICS, item)
            if resp.get('status') == ExtStatus.OK:
                snap[name] = resp.get('value')
            else:
                snap[name] = None
        return snap

    async def read_control_config(self) -> dict:
        """Read all control-config registers (ext 0x0B items 0x50-0x5D).

        Returns a flat dict of field -> value (None on per-item failure).
        Includes the runtime-state readonly items (control_runtime_state,
        last_timeout_reason, trajectory_done, servo_mode).
        """
        out: dict = {}
        for item, name, is_float in CONTROL_CONFIG_ITEMS:
            resp = await self.ext_command(ExtSubCmd.GET_CONTROL_CONFIG, item)
            if resp.get('status') == ExtStatus.OK:
                out[name] = resp.get('value')
            else:
                out[name] = None
        return out

    async def set_control_config(self, item: int, value: Any,
                                 is_float: bool) -> dict:
        """Set one control-config register (ext 0x0C). Returns the response."""
        if item == 0x55:
            profile_ceiling = self._last_vel_limit if self._last_vel_limit > 0.0 else VEL_LIMIT_MAX
            value = _fclamp(float(value), 0.0, min(profile_ceiling, VEL_LIMIT_MAX))
        ext_type = ExtType.FLOAT32 if is_float else ExtType.UINT32
        return await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, item,
                                      ext_type, value)

    async def set_servo_mode(self, mode: ServoControlMode) -> dict:
        """Set the business-layer servo mode (ext 0x0C item 0x5B).

        Firmware maps it to (ControlMode, InputMode) + a default timeout_action
        and calls control_mode_updated(). Feeds the command watchdog.
        """
        return await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, 0x5B,
                                      ExtType.UINT32, int(mode))

    async def send_heartbeat(self) -> None:
        """Send a master heartbeat (NMT, cmd 0x000) to feed the firmware's
        inbound heartbeat watchdog. Call periodically (faster than
        heartbeat_timeout_ms) when the firmware heartbeat watchdog is enabled.
        """
        await self._send_cmd(CmdId.NMT, b'')

    async def _handle_ext_response(self, data: bytes, ts: float) -> None:
        resp = decode_extended_response(data)
        key = (resp.get('sub_cmd', -1), resp.get('item', -1))
        fut = self._ext_pending.get(key)
        if fut and not fut.done():
            fut.set_result(resp)

    # ------------------------------------------------------------------ #
    # Safety Monitor
    # ------------------------------------------------------------------ #

    async def start_safety_monitor(self) -> None:
        """Start the background safety monitoring loop."""
        if self._safety_task is not None:
            self._safety_task.cancel()
        self._safety_task = asyncio.create_task(self._safety_loop())

    async def stop_safety_monitor(self) -> None:
        """Stop the safety monitoring loop."""
        if self._safety_task is not None:
            self._safety_task.cancel()
            try:
                await self._safety_task
            except asyncio.CancelledError:
                pass
            self._safety_task = None

    async def _safety_loop(self) -> None:
        """Monitor WS liveness and MIT watchdog; send safe commands on timeout."""
        while True:
            try:
                await asyncio.sleep(0.5)
                now = time.monotonic()

                # WS disconnect watchdog
                if (self._ws_connected and
                        self._last_ws_msg_ts > 0 and
                        (now - self._last_ws_msg_ts) > WS_WATCHDOG_S):
                    await self._safety_stop()
                    self._ws_connected = False
                    if self.on_log:
                        self.on_log(
                            '[SAFETY] WS watchdog expired -> zero torque + IDLE')

                # MIT watchdog: neutral frame if no MIT sent for MIT_WATCHDOG_S
                if (self._mit_active and
                        (now - self._mit_last_ts) > MIT_WATCHDOG_S):
                    await self._send_mit_neutral()
                    self._mit_active = False
                    if self.on_log:
                        self.on_log(
                            '[SAFETY] MIT watchdog expired -> neutral frame')

                # Master heartbeat: send an NMT frame every 1s while CAN is
                # open. Feeds the firmware's inbound heartbeat watchdog
                # (heartbeat_timeout_ms); harmless if that watchdog is disabled.
                if self.transport.is_open and (now - self._last_heartbeat_ts) >= 1.0:
                    self._last_heartbeat_ts = now
                    try:
                        await self.send_heartbeat()
                    except Exception as exc:
                        logger.debug('heartbeat send error: %s', exc)

            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.debug('safety loop error: %s', exc)

    async def _safety_stop(self) -> None:
        """Send zero commands and transition to IDLE."""
        try:
            await self.set_input_torque(0.0)
        except Exception:
            pass
        try:
            await self.set_axis_state(AxisState.IDLE)
        except Exception:
            pass
        self._mit_active = False

    async def _send_mit_neutral(self) -> None:
        """Send a neutral MIT frame (all zeros)."""
        try:
            data = encode_mit_neutral()
            await self._send_cmd(CmdId.SET_MIT_CONTROL, data)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # WS Connection Tracking
    # ------------------------------------------------------------------ #

    def notify_ws_connected(self) -> None:
        """Call when a WebSocket client connects."""
        self._ws_connected = True
        self._last_ws_msg_ts = time.monotonic()

    def notify_ws_disconnected(self) -> None:
        """Call when the last WebSocket client disconnects."""
        self._ws_connected = False
        # Immediately trigger safety stop
        asyncio.ensure_future(self._safety_stop())

    def notify_ws_activity(self) -> None:
        """Call on every WebSocket message to reset the watchdog timer."""
        self._last_ws_msg_ts = time.monotonic()

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #

    async def shutdown(self) -> None:
        """Clean shutdown: stop polling, safety monitor, send safety stop."""
        await self.stop_polling()
        await self.stop_safety_monitor()
        if self.transport.is_open:
            try:
                await self._safety_stop()
            except Exception:
                pass
