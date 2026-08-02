#!/usr/bin/env python3
"""Measure LZ5710 dual-encoder quality while the output shaft is moving.

The script talks to the already-running foc_ui backend over WebSocket, so it
does not compete with the UI for ownership of the PCAN adapter.  It runs this
profile:

    IDLE -> 0 rpm -> +rpm -> 0 rpm -> -rpm -> 0 rpm -> IDLE

No calibration values, gains, limits, or NVM settings are changed.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import websockets


SUBCMD_VERNIER_DIAG = 0x0A
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

COUNTER_ITEMS = {
    "main_error": 0x09,
    "aux_error": 0x0A,
    "pair_error": 0x0C,
    "main_dma_error": 0x14,
    "main_crc_error": 0x15,
    "main_fixed_bit_error": 0x16,
    "main_status_warning": 0x17,
    "main_samples": 0x18,
    "aux_dma_error": 0x19,
    "aux_crc_error": 0x1A,
    "aux_fixed_bit_error": 0x1B,
    "aux_status_warning": 0x1C,
    "aux_samples": 0x1D,
    "pair_busy": 0x37,
    "pair_ok": 0x38,
}

LIVE_ITEMS = {
    "readiness_flags": (0x82, False),
    "chain_fault": (0x74, False),
    "branch": (0x75, True),
    "raw_main_phase_rad": (0x77, True),
    "raw_aux_phase_rad": (0x78, True),
    "output_position_rad": (0x7B, True),
    "output_velocity_rpm": (0x7C, True),
    "resolver_residual_rad": (0x7D, True),
    "resolver_margin_rad": (0x7E, True),
    "main_sample_age_cycles": (0x3D, False),
    "main_sample_age_max_cycles": (0x3E, False),
}

CHAIN_FAULT_NAMES = {
    1 << 0: "MAIN_COMMUNICATION",
    1 << 1: "AUX_COMMUNICATION",
    1 << 2: "FRAME_CHECK",
    1 << 3: "LUT_INVALID",
    1 << 4: "VERNIER_NO_SOLUTION",
    1 << 5: "VERNIER_AMBIGUOUS",
    1 << 6: "VERNIER_RESIDUAL",
    1 << 7: "POSITION_JUMP",
    1 << 8: "PLL_TIMEOUT",
    1 << 9: "DIRECTION_INVALID",
}


def counter_delta(new: int, old: int) -> int:
    """Return a uint32 lifetime-counter delta, including one wrap."""
    return (int(new) - int(old)) & 0xFFFFFFFF


def decode_mask(value: int, names: dict[int, str]) -> str:
    decoded = [name for bit, name in names.items() if value & bit]
    return ",".join(decoded) if decoded else "NONE"


def finite(values):
    return [value for value in values if isinstance(value, (int, float)) and math.isfinite(value)]


@dataclass
class Sample:
    elapsed_s: float
    stage: str
    command_rpm: float
    axis_state: int = -1
    axis_error: int = 0
    motor_error: int = 0
    encoder_error: int = 0
    controller_error: int = 0
    position_rad: float = math.nan
    velocity_rpm: float = math.nan
    readiness_flags: int = 0
    chain_fault: int = 0
    branch: int = -1
    raw_main_phase_rad: float = math.nan
    raw_aux_phase_rad: float = math.nan
    output_position_rad: float = math.nan
    output_velocity_rpm: float = math.nan
    resolver_residual_rad: float = math.nan
    resolver_margin_rad: float = math.nan
    main_sample_age_cycles: int = 0
    main_sample_age_max_cycles: int = 0


@dataclass
class QualityStats:
    samples: list[Sample] = field(default_factory=list)
    websocket_timeouts: int = 0
    observed_chain_fault_mask: int = 0

    def add(self, sample: Sample) -> None:
        self.samples.append(sample)
        self.observed_chain_fault_mask |= sample.chain_fault


class BackendClient:
    def __init__(self, url: str, timeout: float):
        self.url = url
        self.timeout = timeout
        self.ws = None
        self.receiver_task = None
        self.latest_telemetry = None
        self.telemetry_event = asyncio.Event()
        self.pending: dict[tuple[int, int], asyncio.Future] = {}

    async def __aenter__(self):
        self.ws = await websockets.connect(self.url)
        self.receiver_task = asyncio.create_task(self._receiver())
        return self

    async def __aexit__(self, exc_type, exc, traceback):
        if self.receiver_task:
            self.receiver_task.cancel()
            try:
                await self.receiver_task
            except asyncio.CancelledError:
                pass
        if self.ws:
            await self.ws.close()

    async def _receiver(self):
        async for raw in self.ws:
            import json
            message = json.loads(raw)
            kind = message.get("type")
            if kind == "telemetry":
                self.latest_telemetry = message
                self.telemetry_event.set()
            elif kind == "ext_resp":
                key = (int(message.get("sub_cmd", -1)),
                       int(message.get("item", -1)))
                future = self.pending.pop(key, None)
                if future and not future.done():
                    future.set_result(message)

    async def send(self, message: dict) -> None:
        import json
        await self.ws.send(json.dumps(message))

    async def ext(self, item: int):
        key = (SUBCMD_VERNIER_DIAG, item)
        if key in self.pending:
            raise RuntimeError(f"duplicate pending diagnostic request 0x{item:02X}")
        future = asyncio.get_running_loop().create_future()
        self.pending[key] = future
        await self.send({
            "type": "ext_cmd",
            "sub_cmd": SUBCMD_VERNIER_DIAG,
            "item": item,
        })
        try:
            response = await asyncio.wait_for(future, self.timeout)
        except BaseException:
            self.pending.pop(key, None)
            raise
        if int(response.get("status", -1)) != 0:
            raise RuntimeError(
                f"diagnostic 0x{item:02X} failed: status={response.get('status')}")
        return response.get("value")

    async def wait_telemetry(self, timeout: float | None = None):
        self.telemetry_event.clear()
        await asyncio.wait_for(
            self.telemetry_event.wait(),
            self.timeout if timeout is None else timeout)
        return self.latest_telemetry


def telemetry_errors(message: dict | None) -> tuple[int, int, int, int]:
    ch = (message or {}).get("ch", {})
    return (
        int(ch.get("axis_error", 0)),
        int(ch.get("motor_err", 0)),
        int(ch.get("enc_err", 0)),
        int(ch.get("ctrl_err", 0)),
    )


async def read_counters(client: BackendClient) -> dict[str, int]:
    result = {}
    for name, item in COUNTER_ITEMS.items():
        result[name] = int(await client.ext(item))
    return result


async def read_live_sample(
        client: BackendClient, elapsed_s: float, stage: str,
        command_rpm: float) -> Sample:
    values = {}
    for name, (item, _is_float) in LIVE_ITEMS.items():
        values[name] = await client.ext(item)

    ch = (client.latest_telemetry or {}).get("ch", {})
    return Sample(
        elapsed_s=elapsed_s,
        stage=stage,
        command_rpm=command_rpm,
        axis_state=int(ch.get("axis_state", -1)),
        axis_error=int(ch.get("axis_error", 0)),
        motor_error=int(ch.get("motor_err", 0)),
        encoder_error=int(ch.get("enc_err", 0)),
        controller_error=int(ch.get("ctrl_err", 0)),
        position_rad=float(ch.get("pos", math.nan)),
        velocity_rpm=float(ch.get("vel", math.nan)),
        readiness_flags=int(values["readiness_flags"]),
        chain_fault=int(values["chain_fault"]),
        branch=int(values["branch"]),
        raw_main_phase_rad=float(values["raw_main_phase_rad"]),
        raw_aux_phase_rad=float(values["raw_aux_phase_rad"]),
        output_position_rad=float(values["output_position_rad"]),
        output_velocity_rpm=float(values["output_velocity_rpm"]),
        resolver_residual_rad=float(values["resolver_residual_rad"]),
        resolver_margin_rad=float(values["resolver_margin_rad"]),
        main_sample_age_cycles=int(values["main_sample_age_cycles"]),
        main_sample_age_max_cycles=int(values["main_sample_age_max_cycles"]),
    )


async def require_clean_ready(client: BackendClient) -> None:
    telemetry = await client.wait_telemetry(2.0)
    errors = telemetry_errors(telemetry)
    if any(errors):
        raise RuntimeError(
            "preflight errors: axis=0x%08X motor=0x%016X "
            "encoder=0x%08X controller=0x%08X" % errors)
    deadline = time.monotonic() + 2.0
    consecutive_ready = 0
    flags = 0
    while time.monotonic() < deadline:
        flags = int(await client.ext(0x82))
        consecutive_ready = consecutive_ready + 1 \
            if (flags & 0xC0) == 0xC0 else 0
        if consecutive_ready >= 5:
            return
        await asyncio.sleep(0.03)
    raise RuntimeError(
        f"encoder did not remain ready: readiness_flags=0x{flags:02X}; "
        "bit6 full-chain and bit7 motor-phase must both be stable")


async def wait_for_state(
        client: BackendClient, state: int, timeout: float,
        consecutive: int = 1) -> None:
    deadline = time.monotonic() + timeout
    consecutive_matches = 0
    observed_states = []
    last_telemetry = None
    while time.monotonic() < deadline:
        telemetry = await client.wait_telemetry(min(0.5, timeout))
        last_telemetry = telemetry
        errors = telemetry_errors(telemetry)
        if any(errors):
            raise RuntimeError(
                "state transition fault: axis=0x%08X motor=0x%016X "
                "encoder=0x%08X controller=0x%08X" % errors)
        current = int(telemetry.get("ch", {}).get("axis_state", -1))
        if not observed_states or observed_states[-1] != current:
            observed_states.append(current)
        if current == state:
            consecutive_matches += 1
            if consecutive_matches >= consecutive:
                return
        else:
            consecutive_matches = 0
    ch = (last_telemetry or {}).get("ch", {})
    raise RuntimeError(
        "timed out waiting for axis state %d; transitions=%s last_state=%s "
        "ctrl_mode=%s input_mode=%s ctrl_flags=0x%02X" % (
            state, observed_states, ch.get("axis_state", -1),
            ch.get("ctrl_mode", -1), ch.get("input_mode", -1),
            int(ch.get("ctrl_flags", 0))))


async def command_pump(
        client: BackendClient, target_rpm: list[float],
        stop_event: asyncio.Event) -> None:
    """Keep control commands independent of slower diagnostic round-trips."""
    while not stop_event.is_set():
        await client.send({
            "type": "set_vel",
            "vel": target_rpm[0] / 60.0,
            "torque_ff": 0.0,
        })
        try:
            await asyncio.wait_for(stop_event.wait(), 0.05)
        except asyncio.TimeoutError:
            pass


async def safe_stop(client: BackendClient) -> None:
    # Repeated zero commands cover a temporarily busy CAN transmit queue.
    for _ in range(5):
        try:
            await client.send({"type": "set_vel", "vel": 0.0, "torque_ff": 0.0})
        except Exception:
            pass
        await asyncio.sleep(0.03)
    try:
        await client.send({"type": "set_state", "state": AXIS_STATE_IDLE})
    except Exception:
        pass


async def run_stage(
        client: BackendClient, stats: QualityStats, test_start: float,
        name: str, rpm: float, duration: float, sample_period: float,
        target_rpm: list[float]) -> None:
    print(f"\n{name}: command {rpm:+.3f} rpm for {duration:.2f} s")
    target_rpm[0] = rpm
    stage_start = time.monotonic()
    next_sample = stage_start
    while time.monotonic() - stage_start < duration:
        now = time.monotonic()
        if now >= next_sample:
            try:
                sample = await read_live_sample(
                    client, now - test_start, name, rpm)
            except asyncio.TimeoutError:
                stats.websocket_timeouts += 1
            else:
                stats.add(sample)
                print(
                    f"  t={sample.elapsed_s:6.2f}s "
                    f"v={sample.output_velocity_rpm:+8.3f} rpm "
                    f"ready=0x{sample.readiness_flags:02X} "
                    f"fault=0x{sample.chain_fault:03X} "
                    f"age={sample.main_sample_age_cycles}/"
                    f"{sample.main_sample_age_max_cycles} cycles "
                    f"res={sample.resolver_residual_rad:+.6f} rad "
                    f"margin={sample.resolver_margin_rad:.6f} rad")
                if (sample.axis_error or sample.motor_error or
                        sample.encoder_error or sample.controller_error):
                    raise RuntimeError(
                        "runtime fault: axis=0x%08X motor=0x%016X "
                        "encoder=0x%08X controller=0x%08X" % (
                            sample.axis_error, sample.motor_error,
                            sample.encoder_error, sample.controller_error))
                if sample.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    raise RuntimeError(
                        "closed loop dropped during %s: axis_state=%d "
                        "(expected %d)" % (
                            name, sample.axis_state,
                            AXIS_STATE_CLOSED_LOOP_CONTROL))
            next_sample += sample_period
        await asyncio.sleep(0.01)


def write_csv(path: Path, samples: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(Sample.__dataclass_fields__)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(vars(sample))


def print_counter_report(start: dict[str, int], end: dict[str, int]) -> bool:
    delta = {name: counter_delta(end[name], start[name]) for name in start}
    main_samples = max(delta["main_samples"], 1)
    aux_samples = max(delta["aux_samples"], 1)
    print("\nFrame counter deltas")
    for name in COUNTER_ITEMS:
        print(f"  {name:24s} {delta[name]:10d}")
    print(
        "  main CRC/fixed/DMA rate: "
        f"{1e6 * delta['main_crc_error'] / main_samples:.2f} / "
        f"{1e6 * delta['main_fixed_bit_error'] / main_samples:.2f} / "
        f"{1e6 * delta['main_dma_error'] / main_samples:.2f} ppm")
    print(
        "  aux  CRC/fixed/DMA rate: "
        f"{1e6 * delta['aux_crc_error'] / aux_samples:.2f} / "
        f"{1e6 * delta['aux_fixed_bit_error'] / aux_samples:.2f} / "
        f"{1e6 * delta['aux_dma_error'] / aux_samples:.2f} ppm")
    return not any(delta[name] for name in (
        "main_dma_error", "main_crc_error", "main_fixed_bit_error",
        "aux_dma_error", "aux_crc_error", "aux_fixed_bit_error"))


def print_quality_report(stats: QualityStats, frame_clean: bool) -> bool:
    if not stats.samples:
        print("\nFAIL: no live samples collected")
        return False
    full_ready = sum(bool(s.readiness_flags & 0x40) for s in stats.samples)
    phase_ready = sum(bool(s.readiness_flags & 0x80) for s in stats.samples)
    residuals = finite([abs(s.resolver_residual_rad) for s in stats.samples])
    margins = finite([s.resolver_margin_rad for s in stats.samples])
    velocities = finite([s.output_velocity_rpm for s in stats.samples])
    branches = sorted(set(s.branch for s in stats.samples))
    max_main_age = max(s.main_sample_age_max_cycles for s in stats.samples)
    print("\nMotion quality")
    print(f"  diagnostic samples:       {len(stats.samples)}")
    print(f"  WebSocket timeouts:       {stats.websocket_timeouts}")
    print(f"  full-chain ready:         {full_ready}/{len(stats.samples)}")
    print(f"  motor-phase ready:        {phase_ready}/{len(stats.samples)}")
    print(
        f"  max main sample age:      {max_main_age} control cycles "
        f"({max_main_age * 0.1:.1f} ms at 10 kHz)")
    print(
        f"  observed chain faults:    0x{stats.observed_chain_fault_mask:03X} "
        f"{decode_mask(stats.observed_chain_fault_mask, CHAIN_FAULT_NAMES)}")
    print(f"  observed branches:        {branches}")
    if residuals:
        print(
            f"  |resolver residual|:      mean={statistics.fmean(residuals):.6f} "
            f"max={max(residuals):.6f} rad")
    if margins:
        print(f"  minimum branch margin:    {min(margins):.6f} rad")
    if velocities:
        print(
            f"  measured velocity range:  {min(velocities):+.3f} .. "
            f"{max(velocities):+.3f} rpm")

    passed = (
        frame_clean and
        stats.websocket_timeouts == 0 and
        full_ready == len(stats.samples) and
        phase_ready == len(stats.samples) and
        max_main_age <= 20 and
        stats.observed_chain_fault_mask == 0)
    print(
        "\n%s" % (
            "PASS: no frame, readiness, or Vernier faults observed"
            if passed else
            "WARN/FAIL: one or more quality problems were observed; "
            "inspect the counters and CSV"))
    return passed


async def async_main(args) -> int:
    if args.rpm <= 0.0 or args.rpm > args.max_rpm:
        raise ValueError("--rpm must be positive and no greater than --max-rpm")
    if args.segment_duration <= 0.0 or args.sample_period < 0.05:
        raise ValueError("segment duration must be positive and sample period >= 0.05 s")

    stats = QualityStats()
    start_counters = None
    end_counters = None
    entered_closed_loop = False
    command_stop = asyncio.Event()
    target_rpm = [0.0]
    command_task = None
    async with BackendClient(args.url, args.timeout) as client:
        try:
            await client.wait_telemetry(2.0)
            if args.clear_at_start:
                await client.send({"type": "clear_errors"})
                await asyncio.sleep(0.5)
            await require_clean_ready(client)

            start_counters = await read_counters(client)
            await client.send({"type": "set_vel", "vel": 0.0, "torque_ff": 0.0})
            await client.send({"type": "set_servo_mode", "mode": 1})
            await client.send({
                "type": "set_mode",
                "control_mode": 2,
                "input_mode": 2,
            })
            await asyncio.sleep(0.15)
            command_task = asyncio.create_task(
                command_pump(client, target_rpm, command_stop))
            await client.send({
                "type": "set_state",
                "state": AXIS_STATE_CLOSED_LOOP_CONTROL,
            })
            await wait_for_state(client, AXIS_STATE_CLOSED_LOOP_CONTROL,
                                 args.enter_timeout, consecutive=5)
            entered_closed_loop = True

            test_start = time.monotonic()
            await run_stage(client, stats, test_start, "settle_zero", 0.0,
                            args.settle_duration, args.sample_period,
                            target_rpm)
            for cycle in range(args.cycles):
                await run_stage(
                    client, stats, test_start, f"forward_{cycle + 1}",
                    args.rpm, args.segment_duration, args.sample_period,
                    target_rpm)
                await run_stage(
                    client, stats, test_start, f"zero_after_forward_{cycle + 1}",
                    0.0, args.settle_duration, args.sample_period, target_rpm)
                await run_stage(
                    client, stats, test_start, f"reverse_{cycle + 1}",
                    -args.rpm, args.segment_duration, args.sample_period,
                    target_rpm)
                await run_stage(
                    client, stats, test_start, f"zero_after_reverse_{cycle + 1}",
                    0.0, args.settle_duration, args.sample_period, target_rpm)
            end_counters = await read_counters(client)
        finally:
            target_rpm[0] = 0.0
            command_stop.set()
            if command_task is not None:
                await command_task
            await safe_stop(client)
            if entered_closed_loop:
                try:
                    await wait_for_state(client, AXIS_STATE_IDLE, 2.0)
                except Exception as error:
                    print(f"WARNING: IDLE confirmation failed: {error}",
                          file=sys.stderr)

    if args.csv:
        write_csv(Path(args.csv), stats.samples)
        print(f"\nCSV: {Path(args.csv).resolve()}")
    if start_counters is None or end_counters is None:
        return 2
    frame_clean = print_counter_report(start_counters, end_counters)
    return 0 if print_quality_report(stats, frame_clean) else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test LZ5710 main/aux encoder quality during motion.")
    parser.add_argument(
        "--url", default="ws://127.0.0.1:8000/ws/can",
        help="foc_ui backend WebSocket URL")
    parser.add_argument(
        "--rpm", type=float, default=3.0,
        help="absolute output-shaft test speed (default: 3 rpm)")
    parser.add_argument(
        "--max-rpm", type=float, default=30.0,
        help="hard CLI safety ceiling for --rpm (default: 30 rpm)")
    parser.add_argument(
        "--segment-duration", type=float, default=5.0,
        help="duration of each forward/reverse segment")
    parser.add_argument(
        "--settle-duration", type=float, default=1.5,
        help="duration of each zero-speed segment")
    parser.add_argument(
        "--cycles", type=int, default=1,
        help="number of forward/zero/reverse/zero cycles")
    parser.add_argument(
        "--sample-period", type=float, default=0.10,
        help="live diagnostic sample period; minimum 0.05 s")
    parser.add_argument(
        "--enter-timeout", type=float, default=3.0)
    parser.add_argument(
        "--timeout", type=float, default=0.5,
        help="individual WebSocket diagnostic timeout")
    parser.add_argument(
        "--clear-at-start", action="store_true",
        help="clear existing faults before preflight (default: refuse)")
    parser.add_argument(
        "--csv", help="optional output CSV path")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.cycles < 1:
        raise ValueError("--cycles must be at least 1")
    try:
        return asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("\nInterrupted; safe-stop cleanup requested.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
