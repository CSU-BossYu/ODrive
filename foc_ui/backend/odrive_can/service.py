"""ODrive service layer: orchestration, polling, commands, and safety.

Central service tying transport, state cache, polling, command methods,
and safety monitoring together.

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
    decode_heartbeat, decode_encoder_estimates, decode_encoder_count, decode_iq, decode_bus_vi,
    decode_motor_error, decode_encoder_error, decode_controller_error,
    decode_extended_response,
    encode_set_axis_state, encode_set_controller_mode,
    encode_set_input_pos, encode_set_input_vel, encode_set_input_torque,
    encode_set_pos_gain, encode_set_vel_gains,
    encode_mit_control, encode_mit_neutral,
    encode_extended_request,
    MIT_P_MIN, MIT_P_MAX, MIT_V_MIN, MIT_V_MAX,
    OVERSPEED_SNAPSHOT_ITEMS,
    CONTROL_CONFIG_ITEMS,
    ServoControlMode,
    CalibrationProfile, CalibrationSessionItem, CalibrationSessionState,
)
from .state import AxisCache
from .transport import CanTransport

logger = logging.getLogger('odrive_can.service')

# --------------------------------------------------------------------------- #
# Safety Constants
# --------------------------------------------------------------------------- #

# Torque is user-decided: the host passes commanded torque through unscaled and
# the firmware's current_lim / Tlim is the safety net (a malformed WS payload
# can only reach the motor as a raw float32; the controller clamps the actual
# current). NaN/Inf are still rejected at the command methods — they are not
# valid user commands and would corrupt the MIT quantizer / float32 encoding.
MIT_POS_MIN_RAD: float = MIT_P_MIN
MIT_POS_MAX_RAD: float = MIT_P_MAX
MIT_VEL_MIN_RAD_S: float = MIT_V_MIN
MIT_VEL_MAX_RAD_S: float = MIT_V_MAX
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
MASTER_HEARTBEAT_PERIOD_S: float = 0.1
EXT_LATE_RESPONSE_GUARD_S: float = 0.05


def _fclamp(value: float, lo: float, hi: float) -> float:
    """Clamp to [lo, hi]; NaN/Inf map to lo so non-finite WS payloads can't
    reach the firmware as a raw float32 (which the controller may interpret
    unpredictably)."""
    if not math.isfinite(value):
        return lo
    return max(lo, min(value, hi))


# Hard ceiling on the friction-calibration sweep torque (output-shaft Nm).
# Defense-in-depth alongside the firmware's own Tlim: a malformed WS message
# cannot command a sweep above this.
FRICTION_SWEEP_TORQUE_MAX: float = 10.0

# Friction-compensation item IDs (ext 0x0C Set_Control_Config).
FRICTION_SERVO_MODE = 0x5B
FRICTION_STATIC_POS = 0x74
FRICTION_STATIC_NEG = 0x75
FRICTION_COULOMB_POS = 0x76
FRICTION_COULOMB_NEG = 0x77
FRICTION_MAX_TORQUE = 0x7A
FRICTION_SLEW_RATE = 0x7B

# Vernier-calibration ext 0x0D items.
VERNIER_ITEM_RESET = 0x00
VERNIER_ITEM_CAPTURE = 0x01
VERNIER_ITEM_FIT = 0x02
VERNIER_ITEM_POINT_COUNT = 0x04
VERNIER_ITEM_FIT_VALID = 0x05
VERNIER_ITEM_FITTED_MAIN = 0x06
VERNIER_ITEM_FITTED_AUX = 0x07
VERNIER_ITEM_SCORE = 0x08
VERNIER_ITEM_WORST = 0x09

# Automatic Vernier sampling uses a small, symmetric output-shaft motion
# window. "sweep_turns" is total peak-to-peak output turns, not one revolution.
VERNIER_AUTO_SWEEP_DEFAULT: float = 0.04
VERNIER_AUTO_SWEEP_MIN: float = 0.002
VERNIER_AUTO_SWEEP_MAX: float = 0.25
VERNIER_AUTO_SETTLE_POS_TOL_DEFAULT: float = 0.002
VERNIER_AUTO_SETTLE_POS_TOL_MIN: float = 0.0001
VERNIER_AUTO_SETTLE_POS_TOL_MAX: float = 0.02
VERNIER_AUTO_MIN_ACTUAL_SPAN: float = 0.002
VERNIER_AUTO_RAMP_VEL_DEFAULT: float = 0.005
VERNIER_AUTO_RAMP_VEL_MIN: float = 0.0005
VERNIER_AUTO_RAMP_VEL_MAX: float = 0.1
VERNIER_AUTO_SWEEP_CYCLES_DEFAULT: int = 2
VERNIER_AUTO_SWEEP_CYCLES_MAX: int = 8


def compute_friction_params(breakaway_pos: float, breakaway_neg: float) -> dict:
    """Derive static/coulomb/max/slew from measured breakaway torque.

    Input and output are output-shaft Nm. static = 0.8 * breakaway (margin
    below the true breakaway so the comp itself doesn't induce motion at
    rest); coulomb = 0.6 * static; max_torque = 1.2 * static; slew_rate =
    30 * static Nm/s. Directional params are independent.
    """
    sp = 0.8 * abs(breakaway_pos)
    sn = 0.8 * abs(breakaway_neg)
    smax = max(sp, sn)
    return {
        'friction_static_pos': sp,
        'friction_static_neg': sn,
        'friction_coulomb_pos': 0.6 * sp,
        'friction_coulomb_neg': 0.6 * sn,
        'friction_max_torque': 1.2 * smax,
        'friction_torque_slew_rate': 30.0 * smax,
    }


def build_vernier_auto_offsets(point_count: int, sweep_turns: float,
                               sweep_cycles: int = VERNIER_AUTO_SWEEP_CYCLES_DEFAULT) -> list[float]:
    """Return a continuous back-and-forth scan path around zero."""
    point_count = max(2, min(int(point_count), 16))
    sweep_turns = _fclamp(sweep_turns, VERNIER_AUTO_SWEEP_MIN,
                          VERNIER_AUTO_SWEEP_MAX)
    sweep_cycles = max(1, min(int(sweep_cycles), VERNIER_AUTO_SWEEP_CYCLES_MAX))
    half_range = 0.5 * sweep_turns
    if point_count == 2:
        return [-half_range, half_range]

    offsets: list[float] = []
    for i in range(point_count):
        phase = (i / (point_count - 1)) * sweep_cycles
        cycle = int(math.floor(phase))
        frac = phase - cycle
        if cycle >= sweep_cycles:
            cycle = sweep_cycles - 1
            frac = 1.0
        if cycle % 2 == 0:
            offsets.append(-half_range + 2.0 * half_range * frac)
        else:
            offsets.append(half_range - 2.0 * half_range * frac)
    return offsets


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

        # Extended commands have no transaction ID on the wire. Serialize all
        # requests so a response cannot be consumed by another in-flight call.
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

        # Friction-calibration sweep (background task)
        self._friction_task: Optional[asyncio.Task] = None
        self.on_friction_progress: Optional[Callable[[float, str], Awaitable[None]]] = None
        self.on_friction_result: Optional[Callable[[dict], Awaitable[None]]] = None

        # Vernier auto-sampling sweep (background task)
        self._vernier_task: Optional[asyncio.Task] = None
        self.on_vernier_progress: Optional[Callable[[float, str], Awaitable[None]]] = None
        self.on_vernier_result: Optional[Callable[[dict], Awaitable[None]]] = None

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

        elif cmd_id == CmdId.GET_ENCODER_COUNT:
            d = decode_encoder_count(data)
            if d:
                self.cache.encoder.shadow_count = d['shadow_count']
                self.cache.encoder.count_in_cpr = d['count_in_cpr']
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
                await self._send_request(CmdId.GET_ENCODER_COUNT)
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
            # Warning (not debug): a silent send failure here is the usual
            # cause of "Iq/vbus not updating"; the GET_IQ/GET_BUS requests
            # never go out, so the firmware never responds.
            logger.warning('send_request %s failed: %s', cmd_id, exc)

    async def _request_system_error_detail(self) -> None:
        now = time.monotonic()
        if (now - self._last_system_error_req_ts) < ERROR_DETAIL_INTERVAL_S:
            return
        self._last_system_error_req_ts = now
        resp = await self.ext_command(ExtSubCmd.GET_DEVICE_INFO, 0x07,
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
        # Leaving MIT input mode: stop the MIT watchdog so it doesn't keep
        # sending neutral MIT frames into a non-MIT mode.
        if not (int(ctrl) == int(ControlMode.TORQUE_CONTROL)
                and int(inp) == int(InputMode.MIT)):
            self._mit_active = False
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
        """Set position input. vel_ff clamped to ±VEL_LIMIT_MAX. pos and
        torque_ff are passed through (torque is user-decided; the firmware's
        current_lim/Tlim is the safety net). NaN/Inf are rejected to avoid a
        garbage float32 reaching the firmware."""
        if not math.isfinite(pos):
            return
        vel_ff = _fclamp(vel_ff, -VEL_LIMIT_MAX, VEL_LIMIT_MAX)
        if not math.isfinite(torque_ff):
            return
        data = encode_set_input_pos(pos, vel_ff, torque_ff)
        await self._send_cmd(CmdId.SET_INPUT_POS, data)

    async def set_input_vel(self, vel: float, torque_ff: float = 0.0) -> None:
        """Set velocity input. vel clamped to ±VEL_LIMIT_MAX; torque_ff passed
        through (firmware current_lim is the safety net)."""
        vel = _fclamp(vel, -VEL_LIMIT_MAX, VEL_LIMIT_MAX)
        if not math.isfinite(torque_ff):
            return
        data = encode_set_input_vel(vel, torque_ff)
        await self._send_cmd(CmdId.SET_INPUT_VEL, data)

    async def set_input_torque(self, torque: float) -> None:
        """Set torque input. Passed through unscaled — the user decides the
        torque and the firmware's current_lim / Tlim clamps the actual current.
        NaN/Inf are rejected (not a valid user command)."""
        if not math.isfinite(torque):
            return
        data = encode_set_input_torque(torque)
        await self._send_cmd(CmdId.SET_INPUT_TORQUE, data)

    async def set_limits(self, vel_limit: float, current_limit: float) -> list[dict]:
        """Set velocity and current limits via the ext config path (authoritative
        and ACKed). vel_limit -> 0x0C item 0x60 (the setter also clamps
        trap_traj_.vel_limit, the same side effect as SET_LIMITS), current_limit
        -> 0x07 item 0x14. The old SET_LIMITS(0x00F) CAN command stays in the
        firmware for script compat but is no longer the authoritative entry —
        having two unsynced setters let the UI misjudge success.

        Both items are armed-guarded in the firmware, so disarm to IDLE first
        or the writes return BUSY_ARMED.
        """
        vel_limit = _fclamp(vel_limit, 0.0, VEL_LIMIT_MAX)
        current_limit = _fclamp(current_limit, 0.0, CURRENT_LIMIT)
        vel_resp = await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, 0x60,
                                          ExtType.FLOAT32, vel_limit)
        if vel_resp.get('status') == ExtStatus.OK:
            self._last_vel_limit = vel_limit
        cur_resp = await self.ext_command(ExtSubCmd.SET_BASIC_CONFIG, 0x14,
                                          ExtType.FLOAT32, current_limit)
        if self.on_log:
            self.on_log(f'[CMD] set_limits -> vel={vel_limit:.2f} rev/s, '
                        f'cur={current_limit:.2f} A')
        return [vel_resp, cur_resp]

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
        """Send MIT control frame. p_des/v_des/kp/kd are bounded by the MIT
        quantization range (the encoding cannot represent values outside it).
        t_ff is passed through unscaled — the user decides the torque and the
        firmware's current_lim is the safety net; the MIT encoding saturates
        t_ff at ±MIT_T_MAX (18 Nm). NaN/Inf t_ff is rejected."""
        p_des = _fclamp(p_des, MIT_POS_MIN_RAD, MIT_POS_MAX_RAD)
        v_des = _fclamp(v_des, MIT_VEL_MIN_RAD_S, MIT_VEL_MAX_RAD_S)
        kp = _fclamp(kp, 0.0, MIT_KP_MAX)
        kd = _fclamp(kd, 0.0, MIT_KD_MAX)
        if not math.isfinite(t_ff):
            return
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
        await self.cancel_friction()
        await self.cancel_vernier()
        await self._send_cmd(CmdId.ESTOP, b'')
        if self.on_log:
            self.on_log('[CMD] estop')

    async def reboot(self) -> None:
        await self._send_cmd(CmdId.REBOOT, b'')
        if self.on_log:
            self.on_log('[CMD] reboot')

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
            fut = asyncio.get_running_loop().create_future()
            self._ext_pending[key] = fut
            data = encode_extended_request(sub_cmd, item, ext_type, value)
            try:
                await self._send_cmd(CmdId.EXTENDED_COMMAND, data)
                return await asyncio.wait_for(fut, timeout=timeout)
            except asyncio.TimeoutError:
                self._ext_pending.pop(key, None)
                # Drain the normal late-response window before allowing the
                # same untagged wire slot to be reused.
                await asyncio.sleep(EXT_LATE_RESPONSE_GUARD_S)
                return {'status': -1, 'error': 'timeout',
                        'sub_cmd': sub_cmd, 'item': item}
            except Exception as exc:
                return {'status': -1, 'error': str(exc),
                        'sub_cmd': sub_cmd, 'item': item}
            finally:
                self._ext_pending.pop(key, None)

    async def read_calibration_session(self) -> dict:
        """Read the firmware's runtime calibration transaction envelope."""
        result: dict[str, Any] = {}
        fields = (
            ('schema_version', CalibrationSessionItem.SCHEMA_VERSION),
            ('session_id', CalibrationSessionItem.SESSION_ID),
            ('state', CalibrationSessionItem.STATE),
            ('stage', CalibrationSessionItem.STAGE),
            ('failure_code', CalibrationSessionItem.FAILURE_CODE),
            ('flags', CalibrationSessionItem.FLAGS),
            ('transition_count', CalibrationSessionItem.TRANSITION_COUNT),
            ('request_options', CalibrationSessionItem.REQUEST_OPTIONS),
            ('progress_permille', CalibrationSessionItem.PROGRESS_PERMILLE),
            ('buffered_records', CalibrationSessionItem.BUFFERED_RECORDS),
            ('dropped_records', CalibrationSessionItem.DROPPED_RECORDS),
            ('buffer_high_watermark',
             CalibrationSessionItem.BUFFER_HIGH_WATERMARK),
            ('buffer_capacity', CalibrationSessionItem.BUFFER_CAPACITY),
            ('transport_frames_sent',
             CalibrationSessionItem.TRANSPORT_FRAMES_SENT),
            ('transport_queue_retries',
             CalibrationSessionItem.TRANSPORT_QUEUE_RETRIES),
            ('transport_disconnect_waits',
             CalibrationSessionItem.TRANSPORT_DISCONNECT_WAITS),
        )
        for name, item in fields:
            response = await self.ext_command(ExtSubCmd.CALIBRATION_SESSION, item)
            if response.get('status') != ExtStatus.OK:
                return {
                    'ok': False,
                    'error': f'calibration session read failed at {name}',
                    'response': response,
                }
            result[name] = int(response.get('value', 0))
        try:
            result['state_name'] = CalibrationSessionState(result['state']).name
        except ValueError:
            result['state_name'] = 'UNKNOWN'
        result['ok'] = True
        return result

    async def start_calibration(
            self, profile: CalibrationProfile = CalibrationProfile.FULL,
            option_flags: int = 0,
            geometry_turns: int | None = None) -> dict:
        if geometry_turns is not None:
            if not 1 <= int(geometry_turns) <= 8:
                raise ValueError('geometry_turns must be in [1, 8]')
            option_flags = (int(option_flags) & ~0x0000FF00) | \
                (int(geometry_turns) << 8)
        request_options = (int(option_flags) & 0xFFFFFF00) | (int(profile) & 0xFF)
        return await self.ext_command(
            ExtSubCmd.CALIBRATION_SESSION, CalibrationSessionItem.START,
            ExtType.UINT32, request_options)

    async def abort_calibration(self) -> dict:
        return await self.ext_command(
            ExtSubCmd.CALIBRATION_SESSION, CalibrationSessionItem.ABORT)

    async def read_calibration_candidate(self) -> dict:
        """Read runtime candidate values; these are not active or committed."""
        fields = (
            ('validity', CalibrationSessionItem.RESULT_VALIDITY),
            ('phase_resistance', CalibrationSessionItem.PHASE_RESISTANCE),
            ('phase_inductance', CalibrationSessionItem.PHASE_INDUCTANCE),
            ('encoder_direction', CalibrationSessionItem.ENCODER_DIRECTION),
            ('phase_offset', CalibrationSessionItem.PHASE_OFFSET),
            ('phase_offset_float', CalibrationSessionItem.PHASE_OFFSET_FLOAT),
            ('effective_ratio_scale',
             CalibrationSessionItem.EFFECTIVE_RATIO_SCALE),
            ('geometry_raw_rms', CalibrationSessionItem.GEOMETRY_RAW_RMS),
            ('geometry_corrected_rms',
             CalibrationSessionItem.GEOMETRY_CORRECTED_RMS),
            ('geometry_direction_peak_to_peak',
             CalibrationSessionItem.GEOMETRY_DIRECTION_PEAK_TO_PEAK),
            ('geometry_used_samples',
             CalibrationSessionItem.GEOMETRY_USED_SAMPLES),
            ('flux_linkage', CalibrationSessionItem.FLUX_LINKAGE),
            ('torque_constant', CalibrationSessionItem.TORQUE_CONSTANT),
            ('flux_sample_stddev',
             CalibrationSessionItem.FLUX_SAMPLE_STDDEV),
            ('flux_used_samples', CalibrationSessionItem.FLUX_USED_SAMPLES),
            ('pole_pairs', CalibrationSessionItem.POLE_PAIRS),
            ('output_inertia', CalibrationSessionItem.OUTPUT_INERTIA),
            ('friction_coulomb_pos',
             CalibrationSessionItem.FRICTION_COULOMB_POS),
            ('friction_coulomb_neg',
             CalibrationSessionItem.FRICTION_COULOMB_NEG),
            ('friction_viscous_pos',
             CalibrationSessionItem.FRICTION_VISCOUS_POS),
            ('friction_viscous_neg',
             CalibrationSessionItem.FRICTION_VISCOUS_NEG),
            ('mechanical_residual_rms_torque',
             CalibrationSessionItem.MECHANICAL_RESIDUAL_RMS_TORQUE),
            ('mechanical_used_samples',
             CalibrationSessionItem.MECHANICAL_USED_SAMPLES),
            ('electrical_delay', CalibrationSessionItem.ELECTRICAL_DELAY),
            ('delay_residual_phase_offset',
             CalibrationSessionItem.DELAY_RESIDUAL_PHASE_OFFSET),
            ('delay_residual_rms', CalibrationSessionItem.DELAY_RESIDUAL_RMS),
            ('delay_used_samples', CalibrationSessionItem.DELAY_USED_SAMPLES),
            ('mechanical_attempted_samples',
             CalibrationSessionItem.MECHANICAL_ATTEMPTED_SAMPLES),
            ('mechanical_rejected_invalid',
             CalibrationSessionItem.MECHANICAL_REJECTED_INVALID),
            ('mechanical_rejected_saturated',
             CalibrationSessionItem.MECHANICAL_REJECTED_SATURATED),
            ('mechanical_rejected_low_velocity',
             CalibrationSessionItem.MECHANICAL_REJECTED_LOW_VELOCITY),
            ('mechanical_max_abs_velocity',
             CalibrationSessionItem.MECHANICAL_MAX_ABS_VELOCITY),
            ('mechanical_rejected_timing',
             CalibrationSessionItem.MECHANICAL_REJECTED_TIMING),
            ('delay_attempted_samples',
             CalibrationSessionItem.DELAY_ATTEMPTED_SAMPLES),
            ('delay_rejected_invalid',
             CalibrationSessionItem.DELAY_REJECTED_INVALID),
            ('delay_rejected_saturated',
             CalibrationSessionItem.DELAY_REJECTED_SATURATED),
            ('delay_rejected_speed',
             CalibrationSessionItem.DELAY_REJECTED_SPEED),
            ('delay_rejected_emf', CalibrationSessionItem.DELAY_REJECTED_EMF),
            ('delay_rejected_phase', CalibrationSessionItem.DELAY_REJECTED_PHASE),
            ('delay_max_abs_electrical_speed',
             CalibrationSessionItem.DELAY_MAX_ABS_ELECTRICAL_SPEED),
            ('vernier_main_offset_rad',
             CalibrationSessionItem.VERNIER_MAIN_OFFSET_RAD),
            ('vernier_aux_offset_rad',
             CalibrationSessionItem.VERNIER_AUX_OFFSET_RAD),
            ('vernier_fit_rms_rad',
             CalibrationSessionItem.VERNIER_FIT_RMS_RAD),
            ('vernier_worst_residual_rad',
             CalibrationSessionItem.VERNIER_WORST_RESIDUAL_RAD),
            ('vernier_minimum_margin_rad',
             CalibrationSessionItem.VERNIER_MINIMUM_MARGIN_RAD),
            ('vernier_used_samples',
             CalibrationSessionItem.VERNIER_USED_SAMPLES),
        )
        result: dict[str, Any] = {}
        for name, item in fields:
            response = await self.ext_command(
                ExtSubCmd.CALIBRATION_SESSION, item)
            if response.get('status') != ExtStatus.OK:
                return {'ok': False, 'error': f'candidate read failed at {name}'}
            result[name] = response.get('value', 0)
        result['ok'] = True
        return result

    async def read_calibration_snapshot(self, include_candidate: bool = False) -> dict:
        """Read one UI-facing transaction snapshot with optional fit results."""
        session = await self.read_calibration_session()
        result: dict[str, Any] = {'ok': bool(session.get('ok')), 'session': session}
        if not result['ok']:
            result['error'] = session.get('error', 'calibration session read failed')
            return result
        state = int(session.get('state', 0))
        terminal = state in (
            int(CalibrationSessionState.COMMITTED),
            int(CalibrationSessionState.FAILED),
            int(CalibrationSessionState.ABORTED),
            int(CalibrationSessionState.STALE),
        )
        if include_candidate or state >= int(CalibrationSessionState.IDENTIFIED) or terminal:
            candidate = await self.read_calibration_candidate()
            result['candidate'] = candidate
            if not candidate.get('ok'):
                result['ok'] = False
                result['error'] = candidate.get('error', 'candidate read failed')
        else:
            result['candidate'] = None
        return result

    async def read_fault_snapshot(self) -> dict:
        """Read the OverspeedSnapshot captured at the last overspeed fault.

        Firmware captures the full controller/encoder/resolver state at the
        instant of ERROR_OVERSPEED and holds it until clear_errors. Exposed via
        ext sub_cmd 0x0E GET_FAULT_SNAPSHOT items 0x40-0x5F (the clean home;
        0x0A GET_VERNIER_DIAGNOSTICS still serves them for backward compat).
        Returns a flat dict; fields are None if the firmware didn't respond
        for that item. 'valid' is 0 when no overspeed fault has been captured.
        """
        snap: dict = {}
        for item, name, _is_float in OVERSPEED_SNAPSHOT_ITEMS:
            resp = await self.ext_command(ExtSubCmd.GET_FAULT_SNAPSHOT, item)
            if resp.get('status') == ExtStatus.OK:
                snap[name] = resp.get('value')
            else:
                snap[name] = None
        return snap

    async def read_control_config(self) -> dict:
        """Read all control-config registers exposed by CONTROL_CONFIG_ITEMS.

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
        elif item == 0x60:
            try:
                fv = float(value)
            except (TypeError, ValueError):
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'non-numeric',
                        'sub_cmd': int(ExtSubCmd.SET_CONTROL_CONFIG), 'item': item}
            if math.isinf(fv) and fv > 0:
                value = fv
            elif not math.isfinite(fv) or fv <= 0.0:
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'invalid vel_limit',
                        'sub_cmd': int(ExtSubCmd.SET_CONTROL_CONFIG), 'item': item}
            else:
                value = _fclamp(fv, 0.0, VEL_LIMIT_MAX)
        elif item == 0x61:
            try:
                fv = float(value)
            except (TypeError, ValueError):
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'non-numeric',
                        'sub_cmd': int(ExtSubCmd.SET_CONTROL_CONFIG), 'item': item}
            if math.isinf(fv) and fv > 0:
                value = fv
            elif not math.isfinite(fv) or fv < 1.0:
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'invalid vel_limit_tolerance',
                        'sub_cmd': int(ExtSubCmd.SET_CONTROL_CONFIG), 'item': item}
            else:
                value = fv
        elif is_float:
            # Reject non-finite floats so a malformed/attacked WS payload can't
            # reach the firmware as a raw NaN/Inf float32. +Inf is allowed only
            # for the "disable" items 0x7A/0x7B (friction_max_torque/slew_rate).
            try:
                fv = float(value)
            except (TypeError, ValueError):
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'non-numeric',
                        'sub_cmd': int(ExtSubCmd.SET_CONTROL_CONFIG), 'item': item}
            if not math.isfinite(fv):
                if not (math.isinf(fv) and fv > 0 and item in (0x7A, 0x7B)):
                    return {'status': ExtStatus.INVALID_VALUE, 'error': 'non-finite',
                            'sub_cmd': int(ExtSubCmd.SET_CONTROL_CONFIG), 'item': item}
            value = fv
        ext_type = ExtType.FLOAT32 if is_float else ExtType.UINT32
        result = await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, item,
                                        ext_type, value)
        if result.get('status') == ExtStatus.OK and item == 0x60:
            self._last_vel_limit = float(value)
        return result

    async def set_servo_mode(self, mode: ServoControlMode) -> dict:
        """Set the business-layer servo mode (ext 0x0C item 0x5B).

        Firmware maps it to (ControlMode, InputMode) + a default timeout_action
        and calls control_mode_updated(). Feeds the command watchdog.
        """
        result = await self.ext_command(ExtSubCmd.SET_CONTROL_CONFIG, 0x5B,
                                        ExtType.UINT32, int(mode))
        # Leaving MIT realtime mode: stop the MIT watchdog. Only act if the
        # write succeeded (BUSY_ARMED means the mode didn't change).
        if (result.get('status') == ExtStatus.OK
                and int(mode) != int(ServoControlMode.MIT_REALTIME)):
            self._mit_active = False
        return result

    async def vernier_calibration(self, item: int, value: Any = 0,
                                  is_float: bool = False,
                                  timeout: float = 1.0) -> dict:
        """Send firmware-side vernier calibration command (ext 0x0D).

        Item 0x01 (CAPTURE) can run while armed so auto-sampling can capture at
        each held target. Items 0x00 (RESET), 0x02 (FIT), and 0x03 (APPLY)
        mutate config and are guarded; disarm first. Item 0x02 accepts a
        float32 search radius in turns; other mutating items use uint32.
        """
        ext_type = ExtType.FLOAT32 if is_float else ExtType.UINT32
        if is_float:
            try:
                fv = float(value)
            except (TypeError, ValueError):
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'non-numeric',
                        'sub_cmd': int(ExtSubCmd.VERNIER_CALIBRATION), 'item': item}
            if not math.isfinite(fv):
                return {'status': ExtStatus.INVALID_VALUE, 'error': 'non-finite',
                        'sub_cmd': int(ExtSubCmd.VERNIER_CALIBRATION), 'item': item}
            value = _fclamp(fv, 0.001, 0.5)
        return await self.ext_command(ExtSubCmd.VERNIER_CALIBRATION, item,
                                      ext_type, value, timeout=timeout)

    # ------------------------------------------------------------------ #
    # Vernier-Offset Auto-Sampling Sweep
    # ------------------------------------------------------------------ #

    async def calibrate_vernier_auto(
            self, point_count: int = 8,
            sweep_turns: float = VERNIER_AUTO_SWEEP_DEFAULT,
            search_radius: float = 0.05,
            settle_vel: float = 0.01,
            settle_timeout_s: float = 3.0,
            settle_pos_tol: float = VERNIER_AUTO_SETTLE_POS_TOL_DEFAULT,
            ramp_vel: float = VERNIER_AUTO_RAMP_VEL_DEFAULT,
            sweep_cycles: int = VERNIER_AUTO_SWEEP_CYCLES_DEFAULT) -> None:
        """Auto-sample N output-shaft positions inside a small safe range.

        Enters POSITION_CONTROL + PASSTHROUGH, moves to each target, waits for
        the shaft to reach the target and settle, captures (CAPTURE is allowed
        while armed in the firmware), then disarms and fits (FIT is guarded).
        Restores the original mode/state in finally. Background task; cancel
        via cancel_vernier(). Broadcasts progress/result via callbacks.
        """
        point_count = max(2, min(int(point_count), 16))
        sweep_turns = _fclamp(sweep_turns, VERNIER_AUTO_SWEEP_MIN,
                              VERNIER_AUTO_SWEEP_MAX)
        search_radius = _fclamp(search_radius, 0.001, 0.5)
        settle_vel = max(1e-6, settle_vel)
        settle_pos_tol = _fclamp(settle_pos_tol,
                                 VERNIER_AUTO_SETTLE_POS_TOL_MIN,
                                 VERNIER_AUTO_SETTLE_POS_TOL_MAX)
        ramp_vel = _fclamp(ramp_vel, VERNIER_AUTO_RAMP_VEL_MIN,
                           VERNIER_AUTO_RAMP_VEL_MAX)
        sweep_cycles = max(1, min(int(sweep_cycles), VERNIER_AUTO_SWEEP_CYCLES_MAX))
        offsets = build_vernier_auto_offsets(point_count, sweep_turns,
                                             sweep_cycles)

        saved_servo: Optional[int] = None
        saved_state = self.cache.heartbeat.axis_state
        captured = 0
        captured_positions: list[float] = []

        async def _progress(pct: float, stage: str) -> None:
            if self.on_vernier_progress:
                try:
                    await self.on_vernier_progress(pct, stage)
                except Exception:
                    pass

        async def _read(item: int) -> Optional[float]:
            r = await self.ext_command(ExtSubCmd.VERNIER_CALIBRATION, item)
            return r.get('value') if r.get('status') == ExtStatus.OK else None

        def _require_closed_loop(stage: str) -> None:
            hb = self.cache.heartbeat
            if (hb.axis_state != int(AxisState.CLOSED_LOOP_CONTROL)
                    or hb.axis_error
                    or hb.motor_err_flag
                    or hb.encoder_err_flag
                    or hb.controller_err_flag):
                err = self.cache.errors
                raise RuntimeError(
                    f'vernier calibration aborted during {stage}: '
                    f'axis_state={hb.axis_state}, axis_error=0x{hb.axis_error:08X}, '
                    f'motor_error=0x{err.motor_error:016X}, '
                    f'encoder_error=0x{err.encoder_error:08X}, '
                    f'controller_error=0x{err.controller_error:08X}')

        async def _ramp_to(target: float, progress_base: float,
                           progress_span: float, label: str) -> None:
            start_pos = float(self.cache.encoder.pos_estimate)
            distance = target - start_pos
            duration = abs(distance) / ramp_vel
            if duration <= 0.05:
                await self.set_input_pos(target)
                await _progress(progress_base + progress_span, label)
                return

            steps = max(1, int(math.ceil(duration / 0.05)))
            for step_i in range(1, steps + 1):
                _require_closed_loop(label)
                frac = step_i / steps
                pos = start_pos + distance * frac
                await self.set_input_pos(pos)
                if step_i == steps or step_i % 5 == 0:
                    await _progress(progress_base + progress_span * frac,
                                    label)
                await asyncio.sleep(duration / steps)

        try:
            # Read current servo mode for restore.
            smode = await self.ext_command(ExtSubCmd.GET_CONTROL_CONFIG,
                                           FRICTION_SERVO_MODE)
            if smode.get('status') == ExtStatus.OK:
                v = int(smode.get('value', 0xFF))
                if v in (int(ServoControlMode.TORQUE),
                         int(ServoControlMode.VELOCITY),
                         int(ServoControlMode.PROFILE_POSITION),
                         int(ServoControlMode.MIT_REALTIME)):
                    saved_servo = v

            await _progress(2.0, 'reset_points')
            await self.set_axis_state(AxisState.IDLE)
            await asyncio.sleep(0.25)
            await self.vernier_calibration(VERNIER_ITEM_RESET)

            await _progress(4.0, 'enter_position_mode')
            await self.set_controller_mode(ControlMode.POSITION_CONTROL,
                                            InputMode.PASSTHROUGH)
            await asyncio.sleep(0.1)
            await _progress(5.0, 'enter_closed_loop')
            await self.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
            await asyncio.sleep(0.4)
            if self.cache.heartbeat.axis_state != int(AxisState.CLOSED_LOOP_CONTROL):
                raise RuntimeError(
                    f'axis did not enter CLOSED_LOOP (state='
                    f'{self.cache.heartbeat.axis_state}); clear errors and '
                    f'ensure motor+encoder are calibrated')

            start = float(self.cache.encoder.pos_estimate)
            for i, offset in enumerate(offsets):
                target = start + offset
                progress_start = 6.0 + 88.0 * (i / point_count)
                await _ramp_to(target, progress_start, 44.0 / point_count,
                               f'scan {i + 1}/{point_count} target={target:.5f}')
                # Wait for the shaft to be both still and actually at target.
                deadline = time.monotonic() + settle_timeout_s
                consec = 0
                last_pos = float(self.cache.encoder.pos_estimate)
                last_vel = float(self.cache.encoder.vel_estimate)
                while time.monotonic() < deadline:
                    await asyncio.sleep(0.05)
                    _require_closed_loop(f'settle target {i + 1}/{point_count}')
                    last_pos = float(self.cache.encoder.pos_estimate)
                    last_vel = float(self.cache.encoder.vel_estimate)
                    pos_err = abs(last_pos - target)
                    vel = abs(last_vel)
                    if pos_err <= settle_pos_tol and vel < settle_vel:
                        consec += 1
                        if consec >= 3:
                            break
                    else:
                        consec = 0
                else:
                    raise RuntimeError(
                        f'vernier target {i + 1}/{point_count} not reached: '
                        f'target={target:.6f}, pos={last_pos:.6f}, '
                        f'err={abs(last_pos - target):.6f}, vel={last_vel:.6f}')

                cap_resp = await self.vernier_calibration(VERNIER_ITEM_CAPTURE)
                if cap_resp.get('status') != ExtStatus.OK:
                    raise RuntimeError(
                        f'capture {i + 1}/{point_count} failed '
                        f'(status={cap_resp.get("status")})')
                captured += 1
                captured_positions.append(last_pos)
                await _progress(6.0 + 88.0 * ((i + 1) / point_count),
                                f'capture {i + 1}/{point_count}')

            actual_span = max(captured_positions) - min(captured_positions)
            min_span = min(0.5 * sweep_turns, VERNIER_AUTO_MIN_ACTUAL_SPAN)
            if actual_span < min_span:
                raise RuntimeError(
                    f'vernier samples have too little motion: '
                    f'span={actual_span:.6f} turn, required>={min_span:.6f}')

            # Disarm so FIT (guarded) is accepted.
            await _progress(95.0, 'disarm')
            await self.set_axis_state(AxisState.IDLE)
            await asyncio.sleep(0.25)
            await _progress(97.0, 'fit')
            fit_resp = await self.vernier_calibration(
                VERNIER_ITEM_FIT, search_radius, is_float=True)
            fit_ok = fit_resp.get('status') == ExtStatus.OK

            result: dict = {
                'ok': fit_ok,
                'captured': captured,
                'fit_valid': False,
                'fitted_main': 0.0,
                'fitted_aux': 0.0,
                'score': 0.0,
                'worst': 0.0,
                'error': None if fit_ok else f'fit failed (status={fit_resp.get("status")})',
            }
            if fit_ok:
                result['fit_valid'] = bool(await _read(VERNIER_ITEM_FIT_VALID))
                result['fitted_main'] = float((await _read(VERNIER_ITEM_FITTED_MAIN)) or 0.0)
                result['fitted_aux'] = float((await _read(VERNIER_ITEM_FITTED_AUX)) or 0.0)
                result['score'] = float((await _read(VERNIER_ITEM_SCORE)) or 0.0)
                result['worst'] = float((await _read(VERNIER_ITEM_WORST)) or 0.0)

            await _progress(100.0, 'done')
            await self._safe_vernier_result(result)

        except asyncio.CancelledError:
            await self._safe_vernier_result({
                'ok': False, 'error': 'cancelled', 'captured': captured,
                'fit_valid': False, 'fitted_main': 0.0, 'fitted_aux': 0.0,
                'score': 0.0, 'worst': 0.0})
            raise
        except Exception as exc:
            await self._safe_vernier_result({
                'ok': False, 'error': str(exc), 'captured': captured,
                'fit_valid': False, 'fitted_main': 0.0, 'fitted_aux': 0.0,
                'score': 0.0, 'worst': 0.0})
        finally:
            try:
                await self.set_axis_state(AxisState.IDLE)
            except Exception:
                pass
            await asyncio.sleep(0.25)
            if saved_servo is not None:
                try:
                    await self.set_servo_mode(ServoControlMode(saved_servo))
                except Exception:
                    pass
            try:
                await self.set_axis_state(AxisState(saved_state))
            except Exception:
                pass
            if self._vernier_task is asyncio.current_task():
                self._vernier_task = None

    async def cancel_vernier(self) -> None:
        """Cancel a running vernier auto-sampling sweep and await cleanup."""
        task = self._vernier_task
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self._vernier_task is task:
            self._vernier_task = None

    async def _safe_vernier_result(self, result: dict) -> None:
        if self.on_vernier_result:
            try:
                await self.on_vernier_result(result)
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    # Friction-Compensation Calibration Sweep
    # ------------------------------------------------------------------ #

    async def calibrate_friction(self, max_torque: float = 2.0,
                                 vel_threshold: float = 0.01,
                                 step: float = 0.05,
                                 step_ms: int = 100) -> None:
        """Bidirectional torque sweep → measure breakaway → write derived params.

        Enters closed-loop + torque mode, ramps torque in each direction until
        the output shaft moves (detected from the polled encoder velocity),
        records the breakaway torque, then writes static/coulomb/max/slew via
        set_control_config. Restores the original servo mode + axis state in
        finally. Runs as a background task; cancel via cancel_friction().
        Broadcasts progress/result via on_friction_progress/on_friction_result.

        Torque is sent via _send_cmd directly (NOT set_input_torque) so the
        sweep's own max_torque ceiling (clamped to FRICTION_SWEEP_TORQUE_MAX)
        is the only host-side limit; the firmware's current_lim / Tlim clamps
        the actual current. If max_torque exceeds Tlim the shaft may never
        break away — see the no-motion error hint.
        """
        max_torque = _fclamp(max_torque, 0.0, FRICTION_SWEEP_TORQUE_MAX)
        if max_torque <= 0.0:
            await self._safe_friction_result({
                'ok': False, 'error': 'max_torque must be > 0',
                'breakaway_pos': 0.0, 'breakaway_neg': 0.0,
                'static_pos': 0.0, 'static_neg': 0.0,
                'coulomb_pos': 0.0, 'coulomb_neg': 0.0,
                'max_torque': 0.0, 'slew_rate': 0.0})
            return
        vel_threshold = max(1e-6, vel_threshold)
        step = max(1e-3, min(step, max_torque))
        step_s = max(0.02, step_ms / 1000.0)

        breakaway = {+1: None, -1: None}
        saved_servo: Optional[int] = None
        saved_state = self.cache.heartbeat.axis_state

        async def _torque(t: float) -> None:
            t = max(-max_torque, min(max_torque, t))
            await self._send_cmd(CmdId.SET_INPUT_TORQUE, encode_set_input_torque(t))

        async def _progress(pct: float, stage: str) -> None:
            if self.on_friction_progress:
                try:
                    await self.on_friction_progress(pct, stage)
                except Exception:
                    pass

        try:
            # Read current servo mode (ext 0x5B) for restore. Only the four
            # canonical modes are restorable via set_servo_mode; a non-canonical
            # combination (derive_servo_mode -> UNKNOWN) is left in torque mode.
            resp = await self.ext_command(ExtSubCmd.GET_CONTROL_CONFIG,
                                          FRICTION_SERVO_MODE)
            if resp.get('status') == ExtStatus.OK:
                v = int(resp.get('value', 0xFF))
                if v in (int(ServoControlMode.TORQUE),
                         int(ServoControlMode.VELOCITY),
                         int(ServoControlMode.PROFILE_POSITION),
                         int(ServoControlMode.MIT_REALTIME)):
                    saved_servo = v

            await _progress(2.0, 'enter_torque_mode')
            # Set torque mode BEFORE arming. set_controller_mode (0x00B) is not
            # guarded by the armed guard; set_servo_mode (0x5B via 0x0C) returns
            # BUSY_ARMED once armed and would silently fail to switch mode,
            # leaving the sweep ramping 0x00E torque into a non-torque mode.
            await self.set_controller_mode(ControlMode.TORQUE_CONTROL,
                                           InputMode.PASSTHROUGH)
            await asyncio.sleep(0.1)
            await _progress(5.0, 'enter_closed_loop')
            await self.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
            await asyncio.sleep(0.4)
            # If the motor/encoder isn't calibrated, the axis errors back to
            # IDLE instead of entering closed loop; abort with a clear cause
            # rather than ramping torque into a disarmed axis.
            if self.cache.heartbeat.axis_state != int(AxisState.CLOSED_LOOP_CONTROL):
                raise RuntimeError(
                    f'axis did not enter CLOSED_LOOP (state='
                    f'{self.cache.heartbeat.axis_state}); clear errors and '
                    f'ensure motor+encoder are calibrated')
            await _torque(0.0)
            await asyncio.sleep(0.1)

            for idx, sign in enumerate((+1, -1)):
                stage = 'ramp_pos' if sign > 0 else 'ramp_neg'
                base = 10.0 + idx * 40.0  # pos: 10-50%, neg: 50-90%
                n = 0
                consec = 0
                n_max = int(max_torque / step) + 2
                while n < n_max:
                    t = sign * min((n + 1) * step, max_torque)
                    await _torque(t)
                    await asyncio.sleep(step_s)
                    vel = self.cache.encoder.vel_estimate or 0.0
                    if abs(vel) > vel_threshold:
                        consec += 1
                        if consec >= 2:
                            breakaway[sign] = t
                            break
                    else:
                        consec = 0
                    n += 1
                    await _progress(base + 40.0 * (n / n_max), stage)
                await _torque(0.0)
                await asyncio.sleep(0.15)

            await _progress(92.0, 'compute')
            bp, bn = breakaway[+1], breakaway[-1]
            ok = bp is not None and bn is not None
            result: dict = {
                'ok': ok,
                'breakaway_pos': bp if bp is not None else 0.0,
                'breakaway_neg': bn if bn is not None else 0.0,
                'static_pos': 0.0, 'static_neg': 0.0,
                'coulomb_pos': 0.0, 'coulomb_neg': 0.0,
                'max_torque': 0.0, 'slew_rate': 0.0,
                'error': None if ok else (
                    'no motion within max_torque. Raise max_torque, or raise '
                    'the firmware current_lim — the controller clamps torque '
                    'to its own Tlim (= current_lim * torque_constant, / vernier '
                    'ratio in vernier mode) which may be below max_torque.'),
            }
            if ok:
                p = compute_friction_params(bp, bn)
                result.update({
                    'static_pos': p['friction_static_pos'],
                    'static_neg': p['friction_static_neg'],
                    'coulomb_pos': p['friction_coulomb_pos'],
                    'coulomb_neg': p['friction_coulomb_neg'],
                    'max_torque': p['friction_max_torque'],
                    'slew_rate': p['friction_torque_slew_rate'],
                })
                await _progress(96.0, 'write')
                # Disarm so Set_Control_Config accepts the writes; it returns
                # BUSY_ARMED while any motor is armed (the sweep is in
                # closed-loop torque mode at this point).
                await self.set_axis_state(AxisState.IDLE)
                await asyncio.sleep(0.25)
                write_failed = []
                for item, val in (
                    (FRICTION_STATIC_POS, p['friction_static_pos']),
                    (FRICTION_STATIC_NEG, p['friction_static_neg']),
                    (FRICTION_COULOMB_POS, p['friction_coulomb_pos']),
                    (FRICTION_COULOMB_NEG, p['friction_coulomb_neg']),
                    (FRICTION_MAX_TORQUE, p['friction_max_torque']),
                    (FRICTION_SLEW_RATE, p['friction_torque_slew_rate']),
                ):
                    wresp = await self.set_control_config(item, val, True)
                    if wresp.get('status') != ExtStatus.OK:
                        write_failed.append(item)
                if write_failed:
                    result['ok'] = False
                    result['error'] = (
                        f'write rejected (BUSY_ARMED?) for items {write_failed}; '
                        'disarm and set manually')

            await _progress(100.0, 'done')
            await self._safe_friction_result(result)

        except asyncio.CancelledError:
            await self._safe_friction_result({
                'ok': False, 'error': 'cancelled',
                'breakaway_pos': breakaway[+1] or 0.0,
                'breakaway_neg': breakaway[-1] or 0.0,
                'static_pos': 0.0, 'static_neg': 0.0,
                'coulomb_pos': 0.0, 'coulomb_neg': 0.0,
                'max_torque': 0.0, 'slew_rate': 0.0})
            raise
        except Exception as exc:
            await self._safe_friction_result({
                'ok': False, 'error': str(exc),
                'breakaway_pos': breakaway[+1] or 0.0,
                'breakaway_neg': breakaway[-1] or 0.0,
                'static_pos': 0.0, 'static_neg': 0.0,
                'coulomb_pos': 0.0, 'coulomb_neg': 0.0,
                'max_torque': 0.0, 'slew_rate': 0.0})
        finally:
            # Never leave torque on; always restore mode/state (best-effort).
            try:
                await _torque(0.0)
            except Exception:
                pass
            # Disarm first so set_servo_mode (guarded by the armed guard,
            # returns BUSY_ARMED while armed) is accepted on the cancel/error
            # path where the sweep left the axis in CLOSED_LOOP.
            try:
                await self.set_axis_state(AxisState.IDLE)
            except Exception:
                pass
            # Let the firmware complete CLOSED_LOOP -> IDLE before restoring
            # the servo mode (set_servo_mode is guarded by the armed guard and
            # returns BUSY_ARMED while still armed).
            await asyncio.sleep(0.25)
            if saved_servo is not None:
                try:
                    await self.set_servo_mode(ServoControlMode(saved_servo))
                except Exception:
                    pass
            try:
                await self.set_axis_state(AxisState(saved_state))
            except Exception:
                pass
        # Only clear if we're still the active task; a new sweep may have
            # been started during our cleanup (cancel_friction awaits here).
            if self._friction_task is asyncio.current_task():
                self._friction_task = None

    async def cancel_friction(self) -> None:
        """Cancel a running friction-calibration sweep and await its cleanup."""
        task = self._friction_task
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        # Only clear if it's still us; a new sweep may have started while we
        # awaited the cancelled task's cleanup.
        if self._friction_task is task:
            self._friction_task = None

    async def _safe_friction_result(self, result: dict) -> None:
        if self.on_friction_result:
            try:
                await self.on_friction_result(result)
            except Exception:
                pass

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
                await asyncio.sleep(0.05)
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

                # Master heartbeat: send an NMT frame at 10 Hz while CAN is
                # open. Feeds the firmware's inbound heartbeat watchdog
                # (heartbeat_timeout_ms); harmless if that watchdog is disabled.
                if (self.transport.is_open and
                        (now - self._last_heartbeat_ts) >= MASTER_HEARTBEAT_PERIOD_S):
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
        await self.cancel_friction()
        await self.cancel_vernier()
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
        # Re-arm _ws_connected: the WS watchdog sets it False on expiry, but
        # any inbound message proves the client is alive, so the watchdog must
        # be re-enabled (otherwise one timeout disables it for the session).
        self._ws_connected = True
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
