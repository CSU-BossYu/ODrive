#!/usr/bin/env python3
"""Velocity-loop current statistics over USB @log + CAN control.

Runs a velocity loop on the ODrive (controlled over CANSimple/PCAN, reusing
common.py) while a firmware log task streams Id/Iq/Inorm/Itrip/... over USB CDC
as `@log` CSV lines (see Firmware/MotorControl/log_task.hpp). The script reads
those lines, prints per-sample telemetry, and at the end prints per-column
min/max/mean/std plus max(Inorm)/Itrip (does the measured current vector
magnitude approach the trip threshold?).

Usage example (50 rpm on a 1:1 output):
    python run_log_stats_velocity_loop.py --serial COM5 --velocity 0.833

Requires pyserial (`pip install pyserial`) and python-can for the PCAN side.
"""
import argparse
import csv
import math
import statistics
import sys
import threading
import time
from collections import deque

import serial

from common import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CMD_GET_MOTOR_ERROR,
    CONTROL_MODE_VELOCITY_CONTROL,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_INT32,
    EXT_TYPE_UINT32,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    get_basic_config,
    get_calib_result,
    get_status_ex,
    open_bus,
    request_two_floats,
    request_u64,
    set_controller_modes,
    set_basic_config,
    set_input_vel,
    set_limits,
    set_requested_state,
    set_vel_gains,
    wait_heartbeat,
)

AXIS_STATE_UNDEFINED = 0
AXIS_STATE_STARTUP_SEQUENCE = 2


# --- Error bit tables (mirrors run_mt6826s_velocity_loop.py) -----------------

MOTOR_ERROR_BITS = [
    (0x0000000000000001, "PHASE_RESISTANCE_OUT_OF_RANGE"),
    (0x0000000000000002, "PHASE_INDUCTANCE_OUT_OF_RANGE"),
    (0x0000000000000008, "DRV_FAULT"),
    (0x0000000000000010, "CONTROL_DEADLINE_MISSED"),
    (0x0000000000000080, "MODULATION_MAGNITUDE"),
    (0x0000000000000400, "CURRENT_SENSE_SATURATION"),
    (0x0000000000001000, "CURRENT_LIMIT_VIOLATION"),
    (0x0000000000010000, "MODULATION_IS_NAN"),
    (0x0000000000200000, "CONTROLLER_FAILED"),
    (0x0000000000400000, "I_BUS_OUT_OF_RANGE"),
    (0x0000000000800000, "BRAKE_RESISTOR_DISARMED"),
    (0x0000000001000000, "SYSTEM_LEVEL"),
    (0x0000000002000000, "BAD_TIMING"),
    (0x0000000004000000, "UNKNOWN_PHASE_ESTIMATE"),
    (0x0000000008000000, "UNKNOWN_PHASE_VEL"),
    (0x0000000010000000, "UNKNOWN_TORQUE"),
    (0x0000000020000000, "UNKNOWN_CURRENT_COMMAND"),
    (0x0000000040000000, "UNKNOWN_CURRENT_MEASUREMENT"),
    (0x0000000080000000, "UNKNOWN_VBUS_VOLTAGE"),
    (0x0000000200000000, "UNKNOWN_GAINS"),
    (0x0000000800000000, "UNBALANCED_PHASES"),
]

AXIS_ERROR_BITS = [
    (0x00000001, "INVALID_STATE"),
    (0x00000040, "MOTOR_FAILED"),
    (0x00000100, "ENCODER_FAILED"),
    (0x00000200, "CONTROLLER_FAILED"),
    (0x00000800, "WATCHDOG_TIMER_EXPIRED"),
]


def decode_bits(value, table):
    names = [name for bit, name in table if value and value & bit]
    return ",".join(names) if names else "NONE"


def fmt_axis_error(value):
    return f"0x{value:08X} {decode_bits(value, AXIS_ERROR_BITS)}"


def ext_value(resp):
    if resp is None or resp["status"] != 0:
        return None
    if resp["type"] == EXT_TYPE_FLOAT32:
        return resp["value_f"]
    if resp["type"] == EXT_TYPE_UINT32:
        return resp["value_u"]
    if resp["type"] == EXT_TYPE_INT32:
        return resp["value_i"]
    return resp["value_i"]


def read_closed_loop_preflight(bus, node_id, extended_id=False):
    status = get_status_ex(bus, node_id, extended_id, timeout=0.5)
    values = {}
    for item, name in [
        (0x01, "phase_resistance"),
        (0x02, "phase_inductance"),
        (0x03, "encoder_phase_offset"),
        (0x04, "encoder_direction"),
    ]:
        _, value = get_calib_result(bus, node_id, item, extended_id)
        values[name] = value
    for item, name in [
        (0x11, "pole_pairs"),
        (0x20, "encoder_mode"),
        (0x21, "encoder_cpr"),
        (0x25, "vernier_virtual_cpr"),
        (0x26, "vernier_main_ratio"),
        (0x27, "vernier_aux_ratio"),
        (0x33, "vernier_output_reversed"),
        (0x34, "vernier_use_phase_difference"),
    ]:
        values[name] = ext_value(get_basic_config(bus, node_id, item, extended_id))
    return status, values


def format_closed_loop_preflight(status, values):
    if status is None:
        status_text = "status_ex=(no response)"
    else:
        status_text = (
            f"status_ex(state={status['axis_state']} "
            f"motor_calibrated={status['motor_calibrated']} "
            f"encoder_ready={status['encoder_ready']} "
            f"flags=0x{status['flags']:02x})"
        )
    value_text = " ".join(
        f"{name}={value if value is not None else 'n/a'}"
        for name, value in values.items()
    )
    return f"{status_text} {value_text}"


def wait_ready_for_closed_loop(bus, node_id, extended_id=False, timeout=5.0):
    """Wait until the axis state machine has completed startup and is idle."""
    deadline = time.monotonic() + timeout
    last_hb = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, node_id, extended_id, timeout=0.3)
        if hb is None:
            continue
        last_hb = hb
        if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
            return hb
        if hb["axis_state"] == AXIS_STATE_IDLE:
            return hb
        time.sleep(0.05)
    return last_hb


# --- USB @log reader --------------------------------------------------------

class UsbLogReader:
    """Background thread that parses firmware `@log_hdr`/`@log` lines."""

    def __init__(self, port, baudrate, verbose=False):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)
        self.verbose = verbose
        self.columns = []          # current column names from @log_hdr
        self.samples = deque()     # list of dict {col: float}
        self.last = None           # most recent sample dict
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                continue
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue
            if line.startswith("@log_hdr,"):
                cols = line[len("@log_hdr,"):].split(",")
                with self._lock:
                    self.columns = [c.strip() for c in cols]
                if self.verbose:
                    print(f"[hdr] {self.columns}")
            elif line.startswith("@log,"):
                parts = line[len("@log,"):].split(",")
                with self._lock:
                    cols = self.columns
                if not cols or len(parts) != len(cols):
                    continue  # header not seen yet or malformed
                try:
                    vals = [float(p) for p in parts]
                except ValueError:
                    continue
                sample = dict(zip(cols, vals))
                with self._lock:
                    self.samples.append(sample)
                    self.last = sample
            else:
                if self.verbose and line:
                    print(f"[usb] {line}")

    def send_ascii(self, line):
        """Send an ASCII protocol line (e.g. `l 63 100`). Tolerates failure."""
        try:
            self.ser.write((line + "\n").encode("ascii"))
            self.ser.flush()
        except Exception as ex:
            print(f"WARNING: USB write failed ({line!r}): {ex}")

    def configure(self, mask, rate):
        """Ask firmware to set the @log mask/rate via ASCII `l` command.

        `l <mask> <rate>` sets log_channel_mask (LogChannel bitmask) and
        log_rate_hz. Runtime only -- not persisted to flash. Best-effort: if
        the firmware predates the `l` command it just keeps the defaults.
        """
        if mask is not None and rate is not None:
            self.send_ascii(f"l {int(mask)} {float(rate)}")
        elif mask is not None:
            self.send_ascii(f"l {int(mask)}")

    def snapshot(self):
        with self._lock:
            return list(self.samples), list(self.columns), self.last


# --- Stats ------------------------------------------------------------------

def numeric_stats(values):
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return None
    return {
        "min": min(vals),
        "max": max(vals),
        "mean": statistics.fmean(vals),
        "std": statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        "n": len(vals),
    }


# --- Main -------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # CAN / control
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node-id", type=int, default=0)
    ap.add_argument("--extended-id", action="store_true")
    ap.add_argument("--velocity", type=float, default=0.833,
                    help="Output-shaft velocity in turns/s (0.833 ~= 50 rpm).")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--ramp-time", type=float, default=1.5)
    ap.add_argument("--sample-period", type=float, default=0.1)
    ap.add_argument("--vel-limit", type=float, default=2.0)
    ap.add_argument("--current-limit", type=float, default=3.0)
    ap.add_argument("--vel-gain", type=float, default=42.0)
    ap.add_argument("--vel-integrator-gain", type=float, default=2.1)
    ap.add_argument("--enter-timeout", type=float, default=3.0)
    ap.add_argument("--startup-timeout", type=float, default=5.0,
                    help="Seconds to wait for the axis to leave UNDEFINED/STARTUP and reach IDLE.")
    # USB log
    ap.add_argument("--serial", required=True, help="ODrive CDC COM port (e.g. COM5).")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--channels", default="0x3f",
                    help="LogChannel bitmask (hex/dec). Default 0x3f = Id/Iq/Inorm/Itrip/vel/ibus.")
    ap.add_argument("--rate", type=float, default=100.0, help="USB @log rate in Hz.")
    ap.add_argument("--trace", type=int, default=20, help="Frames to dump around an error.")
    ap.add_argument("--csv", help="Optional path to dump all samples as CSV.")
    ap.add_argument("--verbose-usb", action="store_true")
    args = ap.parse_args()

    mask = int(args.channels, 0)

    bus = open_bus(args.channel, args.bitrate)
    reader = UsbLogReader(args.serial, args.baud, verbose=args.verbose_usb)
    reader.start()
    motion_started = False

    try:
        print(f"Opened CAN {bus.channel_info} and USB {args.serial}")
        # Configure log stream (best-effort; firmware defaults match if this fails)
        reader.configure(mask, args.rate)
        time.sleep(0.15)  # let @log_hdr arrive

        clear_errors(bus, args.node_id, args.extended_id)
        time.sleep(0.3)

        # Wait for startup to finish. Right after reboot the heartbeat can report
        # UNDEFINED while the axis thread is still building its startup task chain.
        hb = wait_ready_for_closed_loop(
            bus, args.node_id, args.extended_id, timeout=args.startup_timeout)
        if hb is None:
            raise RuntimeError("No CAN heartbeat received")
        print(f"Heartbeat: state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'],'?')}) "
              f"axis_error={fmt_axis_error(hb['axis_error'])}")
        if hb["axis_state"] != AXIS_STATE_IDLE:
            _, motor = request_u64(bus, args.node_id, CMD_GET_MOTOR_ERROR,
                                    args.extended_id, timeout=0.5)
            raise RuntimeError(
                f"Axis not ready for closed loop after {args.startup_timeout:.1f}s: "
                f"state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'],'?')}) "
                f"axis={fmt_axis_error(hb['axis_error'])} "
                f"motor=0x{motor or 0:016x} {decode_bits(motor, MOTOR_ERROR_BITS)}")

        _, cols, _ = reader.snapshot()
        if not cols:
            print("WARNING: no @log_hdr seen yet (firmware not streaming?). "
                  "Check that log_channel_mask != 0 and USB CDC logging is active.")
        else:
            print(f"Log columns: {cols}")

        status_ex, preflight_values = read_closed_loop_preflight(
            bus, args.node_id, args.extended_id)
        print("Closed-loop preflight: " +
              format_closed_loop_preflight(status_ex, preflight_values))

        # Set up velocity mode
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        set_controller_modes(bus, args.node_id,
                             CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH,
                             args.extended_id)
        set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
        set_vel_gains(bus, args.node_id, args.vel_gain, args.vel_integrator_gain,
                      args.extended_id)
        time.sleep(0.1)

        # Enter closed loop
        set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
        start = time.monotonic()
        entered = False
        while time.monotonic() - start < args.enter_timeout:
            set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.3)
            if hb is None:
                continue
            if (hb["axis_state"] == AXIS_STATE_CLOSED_LOOP_CONTROL and hb["axis_error"] == 0
                    and not hb["motor_error_flag"]):
                entered = True
                break
            if hb["axis_error"] or hb["motor_error_flag"]:
                _, motor = request_u64(bus, args.node_id, CMD_GET_MOTOR_ERROR,
                                        args.extended_id, timeout=0.5)
                status_ex, preflight_values = read_closed_loop_preflight(
                    bus, args.node_id, args.extended_id)
                raise RuntimeError(
                    f"Error entering closed loop: axis={fmt_axis_error(hb['axis_error'])} "
                    f"motor=0x{motor or 0:016x} {decode_bits(motor, MOTOR_ERROR_BITS)} "
                    f"{format_closed_loop_preflight(status_ex, preflight_values)}")
            time.sleep(0.05)
        if not entered:
            raise RuntimeError("Timed out waiting for CLOSED_LOOP_CONTROL")
        motion_started = True
        print(f"Closed loop. Commanding {args.velocity:.4f} turns/s for {args.duration:.2f}s "
              f"(current_limit={args.current_limit:.3f}A, log mask=0x{mask:x}, rate={args.rate}Hz)")

        loop_start = time.monotonic()
        while time.monotonic() - loop_start < args.duration:
            t = time.monotonic() - loop_start
            ramp = min(1.0, t / args.ramp_time) if args.ramp_time > 0 else 1.0
            cmd = args.velocity * ramp
            set_input_vel(bus, args.node_id, cmd, 0.0, args.extended_id)

            _, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES,
                                        args.extended_id, timeout=0.1)
            _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ,
                                        args.extended_id, timeout=0.1)
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.1)

            vel = enc[1] if enc else float("nan")
            iq_meas_can = iq[1] if iq else float("nan")
            samples, cols, last = reader.snapshot()
            last_txt = ""
            if last:
                last_txt = " ".join(f"{c}={last[c]:+.4f}" for c in cols)
            hb_txt = ""
            if hb:
                hb_txt = (f" state={hb['axis_state']} axis=0x{hb['axis_error']:08x} "
                          f"mflag={hb['motor_flags']:02x}")
            print(f"t={t:5.2f} cmd={cmd:+.4f} vel={vel:+.4f} Iq_can={iq_meas_can:+.3f}"
                  f"{hb_txt} | {last_txt}")

            if hb and (hb["axis_error"] or hb["motor_error_flag"]):
                _, motor = request_u64(bus, args.node_id, CMD_GET_MOTOR_ERROR,
                                        args.extended_id, timeout=0.5)
                print(f"\n!!! ERROR at t={t:.3f}s !!!")
                print(f"  axis_error: {fmt_axis_error(hb['axis_error'])}")
                print(f"  motor_error: 0x{motor or 0:016x} "
                      f"{decode_bits(motor, MOTOR_ERROR_BITS)}")
                # Dump last N log frames around the trip
                tail = samples[-args.trace:] if samples else []
                print(f"  last {len(tail)} log frames:")
                for s in tail:
                    print("    " + " ".join(f"{c}={s[c]:+.4f}" for c in s))
                raise RuntimeError("Error observed during velocity loop")
            time.sleep(args.sample_period)

        print("Stopping, returning to IDLE...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        time.sleep(0.5)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        time.sleep(0.3)

    except BaseException:
        print("Stopping motor due to failure...")
        try:
            for _ in range(20):
                set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
                time.sleep(0.02)
            set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        except Exception as ex:
            print(f"WARNING: stop failed: {ex}")
        raise
    finally:
        if motion_started:
            try:
                set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
                set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            except Exception:
                pass
        reader.stop()
        bus.shutdown()

    # --- Statistics ---
    samples, cols, _ = reader.snapshot()
    if not samples:
        print("No log samples collected.")
        return 0

    print(f"\n===== Statistics over {len(samples)} samples, columns {cols} =====")
    for col in cols:
        st = numeric_stats([s[col] for s in samples if col in s])
        if st is None:
            print(f"  {col:12s}: (no finite data)")
            continue
        print(f"  {col:12s}: min={st['min']:+.4f} max={st['max']:+.4f} "
              f"mean={st['mean']:+.4f} std={st['std']:.4f} n={st['n']}")

    if "Inorm" in cols and "Itrip" in cols:
        inorm_vals = [s["Inorm"] for s in samples if math.isfinite(s.get("Inorm", float("nan")))]
        itrip_vals = [s["Itrip"] for s in samples if math.isfinite(s.get("Itrip", float("nan")))]
        if inorm_vals and itrip_vals:
            max_inorm = max(inorm_vals)
            itrip = statistics.fmean(itrip_vals)
            ratio = max_inorm / itrip if itrip > 0 else float("inf")
            print(f"\n  max(Inorm)={max_inorm:.4f}A  Itrip~{itrip:.4f}A  "
                  f"ratio={ratio:.3f}  {'<<< APPROACHED TRIP' if ratio > 0.8 else ''}")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for s in samples:
                w.writerow(f"{s[c]:.6f}" if c in s and math.isfinite(s[c]) else "" for c in cols)
        print(f"\nCSV written to {args.csv}")

    print("PASS: velocity loop completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
