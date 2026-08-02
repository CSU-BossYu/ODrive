#!/usr/bin/env python3
"""Observe stationary dual-encoder counts and the complete PLL velocity chain.

This tool is read-only. It refuses to sample an armed/non-IDLE axis unless
--allow-non-idle is explicitly supplied. Firmware must expose Vernier
diagnostic items 0x60..0x65.
"""

import argparse
import csv
import math
import statistics
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_IDLE,
    EXT_TYPE_UINT32,
    ext_request,
    open_bus,
    wait_heartbeat,
)


SUB_CMD_VERNIER_DIAG = 0x0A
DEFAULT_CPR = 32768
DEFAULT_MAIN_RATIO = 42.0

ITEMS = (
    (0x00, "main_angle", "u32"),
    (0x01, "aux_angle", "u32"),
    (0x02, "main_valid", "u32"),
    (0x03, "aux_valid", "u32"),
    (0x04, "pair_sequence", "u32"),
    (0x05, "pair_valid", "u32"),
    (0x06, "resolver_residual_turns", "f32"),
    (0x20, "encoder_pos_turns", "f32"),
    (0x21, "encoder_vel_turns_per_s", "f32"),
    (0x23, "resolver_valid", "u32"),
    (0x24, "resolver_locked", "u32"),
    (0x28, "output_pos_turns", "f32"),
    (0x29, "output_vel_turns_per_s", "f32"),
    (0x2C, "pair_vel_turns_per_s", "f32"),
    (0x3C, "cycle_counter_hz", "u32"),
    (0x3D, "main_sample_age_cycles", "u32"),
    (0x60, "pll_phase_error_counts", "f32"),
    (0x61, "pll_velocity_counts_per_s", "f32"),
    (0x62, "pll_position_counts", "f32"),
    (0x63, "filtered_motor_vel_turns_per_s", "f32"),
    (0x64, "shadow_count", "i32"),
    (0x65, "count_in_cpr", "i32"),
)


def read_item(bus, args, item_id, kind):
    response = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item_id, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=args.timeout,
    )
    if response is None:
        return None
    if response["status"] != 0:
        if item_id >= 0x60:
            raise RuntimeError(
                f"diagnostic item 0x{item_id:02X} unavailable "
                "(flash the newly built firmware)")
        return None
    if kind == "f32":
        return response["value_f"]
    if kind == "i32":
        return response["value_i"]
    return response["value_u"]


def circular_delta_counts(value, reference, cpr):
    delta = int(value) - int(reference)
    return (delta + cpr // 2) % cpr - cpr // 2


def capture(bus, args, index, start):
    capture_start = time.monotonic()
    row = {
        "index": index,
        "time_s": capture_start - start,
    }
    for item_id, name, kind in ITEMS:
        row[name] = read_item(bus, args, item_id, kind)
    capture_end = time.monotonic()
    row["capture_span_ms"] = (capture_end - capture_start) * 1000.0

    raw_pll = row["pll_velocity_counts_per_s"]
    if raw_pll is not None:
        row["raw_pll_output_rpm"] = (
            raw_pll / args.cpr / args.main_ratio * 60.0)
    else:
        row["raw_pll_output_rpm"] = None
    filtered_motor = row["filtered_motor_vel_turns_per_s"]
    row["filtered_output_rpm"] = (
        None if filtered_motor is None else
        filtered_motor / args.main_ratio * 60.0)
    output_velocity = row["output_vel_turns_per_s"]
    row["published_output_rpm"] = (
        None if output_velocity is None else output_velocity * 60.0)
    pair_velocity = row["pair_vel_turns_per_s"]
    row["pair_output_rpm"] = (
        None if pair_velocity is None else pair_velocity * 60.0)
    cycle_hz = row["cycle_counter_hz"]
    age_cycles = row["main_sample_age_cycles"]
    row["main_sample_age_us"] = (
        None if not cycle_hz or age_cycles is None else
        age_cycles * 1.0e6 / cycle_hz)
    return row


def finite_values(rows, key):
    return [
        float(row[key]) for row in rows
        if row.get(key) is not None and math.isfinite(float(row[key]))
    ]


def stats(rows, key):
    values = finite_values(rows, key)
    if not values:
        return None
    mean = statistics.fmean(values)
    return {
        "n": len(values),
        "min": min(values),
        "max": max(values),
        "p2p": max(values) - min(values),
        "mean": mean,
        "std": statistics.pstdev(values),
        "rms": math.sqrt(statistics.fmean(value * value for value in values)),
    }


def correlation(rows, left, right):
    pairs = [
        (float(row[left]), float(row[right])) for row in rows
        if row.get(left) is not None and row.get(right) is not None
        and math.isfinite(float(row[left]))
        and math.isfinite(float(row[right]))
    ]
    if len(pairs) < 3:
        return float("nan")
    xs, ys = zip(*pairs)
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    numerator = sum((x - mx) * (y - my) for x, y in pairs)
    denominator = math.sqrt(
        sum((x - mx) ** 2 for x in xs) *
        sum((y - my) ** 2 for y in ys))
    return numerator / denominator if denominator > 0.0 else float("nan")


def float32_ulp(value):
    """Return the spacing of binary32 numbers near value."""
    value = abs(float(value))
    if not math.isfinite(value):
        return math.nan
    if value == 0.0:
        return 2.0 ** -149
    return 2.0 ** (math.floor(math.log2(value)) - 23)


def add_relative_columns(rows, args):
    if not rows:
        return
    base_main = rows[0].get("main_angle")
    base_aux = rows[0].get("aux_angle")
    base_output = rows[0].get("output_pos_turns")
    base_shadow = rows[0].get("shadow_count")
    for row in rows:
        row["main_delta_counts"] = (
            None if base_main is None or row.get("main_angle") is None else
            circular_delta_counts(row["main_angle"], base_main, args.cpr))
        row["aux_delta_counts"] = (
            None if base_aux is None or row.get("aux_angle") is None else
            circular_delta_counts(row["aux_angle"], base_aux, args.cpr))
        row["shadow_delta_counts"] = (
            None if base_shadow is None or row.get("shadow_count") is None else
            row["shadow_count"] - base_shadow)
        row["output_delta_rad"] = (
            None if base_output is None or row.get("output_pos_turns") is None
            else (row["output_pos_turns"] - base_output) * 2.0 * math.pi)


def print_live(row):
    def f(key, digits=3):
        value = row.get(key)
        return "NA" if value is None else f"{value:+.{digits}f}"

    print(
        f"t={row['time_s']:6.2f}s "
        f"M/A={row.get('main_angle')}/{row.get('aux_angle')} "
        f"cnt={row.get('count_in_cpr')} shadow={row.get('shadow_count')} "
        f"ePLL={f('pll_phase_error_counts', 2)}cnt "
        f"vPLL={f('raw_pll_output_rpm', 3)}rpm "
        f"vLPF={f('filtered_output_rpm', 3)}rpm "
        f"vOUT={f('published_output_rpm', 3)}rpm "
        f"pair={f('pair_output_rpm', 3)}rpm "
        f"age={f('main_sample_age_us', 1)}us "
        f"valid={row.get('main_valid')}/{row.get('aux_valid')}/"
        f"{row.get('pair_valid')}/{row.get('resolver_locked')} "
        f"span={row['capture_span_ms']:.1f}ms"
    )


def print_stat_line(label, result, unit):
    if result is None:
        print(f"  {label:<25s} unavailable")
        return
    print(
        f"  {label:<25s} p2p={result['p2p']:.6g} {unit}, "
        f"std={result['std']:.6g}, rms={result['rms']:.6g}, "
        f"range=[{result['min']:.6g}, {result['max']:.6g}]")


def analyze(rows, args):
    add_relative_columns(rows, args)
    print("\nStationary summary")
    selected = (
        ("main raw displacement", "main_delta_counts", "count"),
        ("aux raw displacement", "aux_delta_counts", "count"),
        ("shadow displacement", "shadow_delta_counts", "count"),
        ("PLL phase error", "pll_phase_error_counts", "count"),
        ("PLL position", "pll_position_counts", "count"),
        ("raw PLL output speed", "raw_pll_output_rpm", "rpm"),
        ("filtered output speed", "filtered_output_rpm", "rpm"),
        ("published output speed", "published_output_rpm", "rpm"),
        ("pair output speed", "pair_output_rpm", "rpm"),
        ("published output position", "output_delta_rad", "rad"),
        ("main sample age", "main_sample_age_us", "us"),
    )
    results = {}
    for label, key, unit in selected:
        results[key] = stats(rows, key)
        print_stat_line(label, results[key], unit)

    phase_velocity_corr = correlation(
        rows, "pll_phase_error_counts", "pll_velocity_counts_per_s")
    filter_publish_corr = correlation(
        rows, "filtered_output_rpm", "published_output_rpm")
    print(f"  corr(PLL error, raw speed) = {phase_velocity_corr:+.4f}")
    print(f"  corr(filtered, published)  = {filter_publish_corr:+.4f}")
    if math.isfinite(filter_publish_corr) and filter_publish_corr < -0.9:
        print(
            "    (strong negative correlation is the configured output "
            "direction inversion, not a filter failure)")

    main_p2p = (results["main_delta_counts"] or {}).get("p2p", math.nan)
    aux_p2p = (results["aux_delta_counts"] or {}).get("p2p", math.nan)
    phase_p2p = (results["pll_phase_error_counts"] or {}).get("p2p", math.nan)
    pll_position_p2p = (
        results["pll_position_counts"] or {}).get("p2p", math.nan)
    speed_p2p = (results["published_output_rpm"] or {}).get("p2p", math.nan)

    print("\nDiagnosis")
    if not math.isfinite(speed_p2p):
        print("  FAIL: no published output velocity samples.")
    elif speed_p2p < 0.2:
        print("  Output-speed estimate is quiet (<0.2 rpm peak-to-peak).")
    else:
        print(f"  Output-speed estimate is noisy ({speed_p2p:.3f} rpm peak-to-peak).")

    shadow_values = finite_values(rows, "shadow_count")
    pll_position_values = finite_values(rows, "pll_position_counts")
    shadow_constant = bool(shadow_values) and max(shadow_values) == min(shadow_values)
    main_constant = math.isfinite(main_p2p) and main_p2p == 0.0
    pll_reference = (
        statistics.fmean(pll_position_values) if pll_position_values else 0.0)
    pll_ulp = float32_ulp(pll_reference)
    precision_limit_cycle = (
        main_constant and shadow_constant and
        math.isfinite(pll_position_p2p) and
        pll_position_p2p >= 0.9 * pll_ulp and
        abs(pll_reference) >= 2.0 ** 24)

    if precision_limit_cycle:
        print(
            f"  ROOT CAUSE: raw and shadow counts are constant, but the float32 "
            f"PLL position is at {pll_reference:.0f} counts where its ULP is "
            f"{pll_ulp:.0f} counts. The true integer count cannot be represented, "
            "forcing a +/-count PLL limit cycle.")
    elif math.isfinite(main_p2p) and main_p2p >= 20.0:
        print(
            f"  Main raw angle moved/jittered by {main_p2p:.0f} counts; "
            "raw sensing or real motor-side motion is a primary contributor.")
    elif math.isfinite(phase_p2p) and phase_p2p >= 20.0:
        print(
            "  Main raw samples are comparatively quiet but PLL phase error is "
            "large; inspect PLL discretization/sample timing.")
    else:
        print("  Main raw and PLL phase-error excursions are both small.")

    if math.isfinite(main_p2p) and math.isfinite(aux_p2p):
        main_output_span = main_p2p / args.main_ratio
        aux_output_span = aux_p2p / args.aux_ratio
        if main_output_span == 0.0 and aux_output_span == 0.0:
            print(
                "  Main and aux raw angles were both perfectly stationary at "
                "the diagnostic sample instants.")
            return
        ratio = main_output_span / max(aux_output_span, 1.0e-9)
        print(
            f"  Equivalent output spans: main={main_output_span:.3f} count, "
            f"aux={aux_output_span:.3f} count, ratio={ratio:.2f}.")
        if ratio > 3.0:
            print(
                "  Main moves far more than aux: suspect main-sensor noise or "
                "motor-side motion inside gearbox compliance/backlash.")
        elif ratio < 1.0 / 3.0:
            print("  Aux moves far more than main: inspect the auxiliary sensor.")
        else:
            print(
                "  Main and aux spans are comparable after gear scaling; "
                "external/output motion or common vibration is plausible.")


def write_csv(path, rows):
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(
        description="Read-only stationary MT6826S/PLL noise observer.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=0.3)
    parser.add_argument("--cpr", type=int, default=DEFAULT_CPR)
    parser.add_argument("--main-ratio", type=float, default=DEFAULT_MAIN_RATIO)
    parser.add_argument("--aux-ratio", type=float, default=41.0)
    parser.add_argument("--allow-non-idle", action="store_true")
    parser.add_argument(
        "--csv",
        default=f"stationary_encoder_pll_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    parser.add_argument("--no-csv", action="store_true")
    args = parser.parse_args()

    if args.duration <= 0.0 or args.period <= 0.0:
        parser.error("duration and period must be positive")
    if args.cpr <= 0 or args.main_ratio == 0.0 or args.aux_ratio == 0.0:
        parser.error("CPR and encoder ratios must be nonzero")

    bus = open_bus(args.channel, args.bitrate)
    rows = []
    try:
        print(f"Opened {bus.channel_info}")
        heartbeat = wait_heartbeat(
            bus, args.node_id, args.extended_id, timeout=5.0)
        if heartbeat is None:
            raise RuntimeError("no heartbeat received")
        print(
            f"Heartbeat: state={heartbeat['axis_state']} "
            f"axis_error=0x{heartbeat['axis_error']:08X}")
        if heartbeat["axis_state"] != AXIS_STATE_IDLE and not args.allow_non_idle:
            raise RuntimeError(
                "axis is not IDLE; this stationary test will not change state. "
                "Disable the axis or pass --allow-non-idle intentionally.")

        start = time.monotonic()
        index = 0
        while time.monotonic() - start < args.duration:
            scheduled = start + index * args.period
            delay = scheduled - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            row = capture(bus, args, index, start)
            rows.append(row)
            print_live(row)
            index += 1
    except KeyboardInterrupt:
        print("\nInterrupted; analyzing captured samples.")
    finally:
        bus.shutdown()

    analyze(rows, args)
    if rows and not args.no_csv:
        add_relative_columns(rows, args)
        write_csv(args.csv, rows)
        print(f"\nCSV saved: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
