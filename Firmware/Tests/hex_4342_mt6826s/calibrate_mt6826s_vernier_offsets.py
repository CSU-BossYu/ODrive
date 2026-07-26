#!/usr/bin/env python3
"""Multi-point static calibration for MT6826S vernier offsets.

The normal ODrive ENCODER_OFFSET_CALIBRATION finds the electrical phase offset.
This tool calibrates the separate dual-encoder geometry offsets used by the
vernier resolver.  It does not need a full output-shaft revolution: place the
joint at several reachable static positions and sample each position.
"""
import argparse
import csv
import math
import time
from dataclasses import dataclass
from types import SimpleNamespace

from common import (
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    get_basic_config,
    ext_request,
    open_bus,
    save_configuration,
    set_basic_config,
    set_requested_state,
    wait_heartbeat,
)
from read_mt6826s_pair import MT6826S_CPR, read_full_pair


PARAM_MAIN_OFFSET = 0x28
PARAM_AUX_OFFSET = 0x29
PARAM_ERR_ACCEPT = 0x2D
PARAM_ERR_REJECT = 0x2E
AXIS_STATE_IDLE = 1
SUB_CMD_VERNIER_CALIBRATION = 0x0D


@dataclass
class RawPoint:
    label: str
    main_angle: float
    aux_angle: float
    main_spread: float
    aux_spread: float


@dataclass
class FitConfig:
    cpr: int
    main_ratio: float
    aux_ratio: float
    main_reversed: bool
    aux_reversed: bool
    output_reversed: bool


def wrap01(value):
    value = value - math.floor(value)
    if value >= 1.0:
        value -= 1.0
    if value < 0.0:
        value += 1.0
    return value


def wrap_pm_half(value):
    value = wrap01(value + 0.5) - 0.5
    if value >= 0.5:
        value -= 1.0
    return value


def offset_for_print(value):
    value = wrap01(value)
    return value - 1.0 if value >= 0.5 else value


def circular_distance(a, b):
    return abs(wrap_pm_half(a - b))


def circular_mean(phases):
    if not phases:
        raise ValueError("no phases to average")
    x = sum(math.cos(2.0 * math.pi * phase) for phase in phases)
    y = sum(math.sin(2.0 * math.pi * phase) for phase in phases)
    return wrap01(math.atan2(y, x) / (2.0 * math.pi))


def angle_phase(angle, cpr):
    return wrap01(float(angle) / float(cpr))


def normalize_raw_angle(angle, reversed_, cpr):
    phase = angle_phase(angle, cpr)
    return wrap01(-phase) if reversed_ else phase


def residual_phase_difference(point, main_offset, aux_offset, config):
    main_phase = normalize_raw_angle(point.main_angle, config.main_reversed, config.cpr)
    aux_phase = normalize_raw_angle(point.aux_angle, config.aux_reversed, config.cpr)
    main_corr = wrap01(main_phase - main_offset)
    aux_corr = wrap01(aux_phase - aux_offset)
    ratio_delta = config.aux_ratio - config.main_ratio
    coarse_output_phase = wrap01(wrap_pm_half(aux_corr - main_corr) / ratio_delta)
    predicted_main_phase = wrap01(config.main_ratio * coarse_output_phase)
    return wrap_pm_half(main_corr - predicted_main_phase)


def output_error_deg(residual, config):
    if abs(config.main_ratio) < 1.0e-9:
        return float("nan")
    return residual / config.main_ratio * 360.0


def objective_value(points, main_offset, aux_offset, config, objective):
    residuals = [
        residual_phase_difference(point, main_offset, aux_offset, config)
        for point in points
    ]
    if objective == "max":
        score = max(abs(residual) for residual in residuals)
    else:
        score = math.sqrt(sum(residual * residual for residual in residuals) / len(residuals))
    return score, residuals


def score_is_better(score, distance, best_score, best_distance):
    if best_score is None:
        return True
    if score < best_score - 1.0e-12:
        return True
    return abs(score - best_score) <= 1.0e-12 and distance < best_distance


def search_values(center, radius, count):
    radius = max(0.0, min(0.5, radius))
    if radius >= 0.5:
        return [index / count for index in range(count)]
    if count <= 1:
        return [wrap01(center)]
    return [
        wrap01(center - radius + (2.0 * radius * index) / (count - 1))
        for index in range(count)
    ]


def within_radius(value, center, radius):
    return radius >= 0.5 or circular_distance(value, center) <= radius + 1.0e-12


def refine_1d(points, main_offset, aux_offset, config, objective, step, search_radius):
    best_aux = wrap01(aux_offset)
    best_score, best_residuals = objective_value(
        points, main_offset, best_aux, config, objective
    )

    for _ in range(12):
        improved = False
        for direction in (-1.0, 1.0):
            candidate = wrap01(best_aux + direction * step)
            if not within_radius(candidate, aux_offset, search_radius):
                continue
            score, residuals = objective_value(
                points, main_offset, candidate, config, objective
            )
            distance = circular_distance(candidate, aux_offset)
            best_distance = circular_distance(best_aux, aux_offset)
            if score_is_better(score, distance, best_score, best_distance):
                best_aux = candidate
                best_score = score
                best_residuals = residuals
                improved = True
        if not improved:
            step *= 0.25
    return best_aux, best_score, best_residuals


def refine_2d(points, main_offset, aux_offset, config, objective, step, search_radius):
    best_main = wrap01(main_offset)
    best_aux = wrap01(aux_offset)
    best_score, best_residuals = objective_value(
        points, best_main, best_aux, config, objective
    )

    for _ in range(12):
        improved = False
        for dm, da in [(-step, 0.0), (step, 0.0), (0.0, -step), (0.0, step)]:
            candidate_main = wrap01(best_main + dm)
            candidate_aux = wrap01(best_aux + da)
            if not within_radius(candidate_main, main_offset, search_radius):
                continue
            if not within_radius(candidate_aux, aux_offset, search_radius):
                continue
            score, residuals = objective_value(
                points, candidate_main, candidate_aux, config, objective
            )
            distance = circular_distance(candidate_main, main_offset) + circular_distance(candidate_aux, aux_offset)
            best_distance = circular_distance(best_main, main_offset) + circular_distance(best_aux, aux_offset)
            if score_is_better(score, distance, best_score, best_distance):
                best_main = candidate_main
                best_aux = candidate_aux
                best_score = score
                best_residuals = residuals
                improved = True
        if not improved:
            step *= 0.25
    return best_main, best_aux, best_score, best_residuals


def fit_offsets(points, config, current_main_offset, current_aux_offset, args):
    if args.fit_main_offset:
        grid_steps = max(24, min(args.grid_steps, 512))
        best = None
        for main_offset in search_values(current_main_offset, args.search_radius, grid_steps):
            for aux_offset in search_values(current_aux_offset, args.search_radius, grid_steps):
                score, residuals = objective_value(
                    points, main_offset, aux_offset, config, args.objective
                )
                distance = (
                    circular_distance(main_offset, current_main_offset)
                    + circular_distance(aux_offset, current_aux_offset)
                )
                if best is None or score_is_better(score, distance, best[0], best[4]):
                    best = (score, main_offset, aux_offset, residuals, distance)

        score, main_offset, aux_offset, _, _ = best
        main_offset, aux_offset, score, residuals = refine_2d(
            points, main_offset, aux_offset, config, args.objective,
            1.0 / grid_steps, args.search_radius,
        )
        return main_offset, aux_offset, score, residuals

    grid_steps = max(256, args.grid_steps)
    main_offset = current_main_offset
    best = None
    for aux_offset in search_values(current_aux_offset, args.search_radius, grid_steps):
        score, residuals = objective_value(
            points, main_offset, aux_offset, config, args.objective
        )
        distance = circular_distance(aux_offset, current_aux_offset)
        if best is None or score_is_better(score, distance, best[0], best[3]):
            best = (score, aux_offset, residuals, distance)

    score, aux_offset, _, _ = best
    aux_offset, score, residuals = refine_1d(
        points, main_offset, aux_offset, config, args.objective,
        1.0 / grid_steps, args.search_radius,
    )
    return main_offset, aux_offset, score, residuals


def read_basic_float(bus, args, param_id, default_value, label):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"{label}: TIMEOUT, using {default_value}")
        return default_value
    status = EXT_STATUS.get(resp["status"], resp["status"])
    if resp["status"] != 0 or resp["type"] != EXT_TYPE_FLOAT32:
        print(f"{label}: status={status}, using {default_value}")
        return default_value
    print(f"{label}: {resp['value_f']:.9g} status={status}")
    return resp["value_f"]


def set_basic_float(bus, args, param_id, value, label):
    resp = set_basic_config(
        bus, args.node_id, param_id, EXT_TYPE_FLOAT32,
        value_float=float(value), extended_id=args.extended_id,
    )
    if resp is None:
        print(f"{label} <- {value:.9g}: TIMEOUT")
        return False
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"{label} <- {value:.9g}: {status} raw={resp['raw']}")
    return resp["status"] == 0


def vernier_calib_request(bus, args, item_id, req_type=0, value=0, value_float=None):
    return ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_CALIBRATION,
        item=item_id, req_type=req_type, value=value, value_float=value_float,
        extended_id=args.extended_id, timeout=args.timeout,
    )


def print_calib_response(resp, label):
    if resp is None:
        print(f"{label}: TIMEOUT")
        return False
    status = EXT_STATUS.get(resp["status"], resp["status"])
    if resp["type"] == EXT_TYPE_FLOAT32:
        value = resp["value_f"]
    else:
        value = resp["value_u"]
    print(f"{label}: {status} value={value} raw={resp['raw']}")
    return resp["status"] == 0


def firmware_value(resp):
    if resp is None:
        return None
    return resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_u"]


def stable_point_from_rows(rows, label, cpr, max_spread_counts, allow_unstable):
    valid_rows = []
    for row in rows:
        if not row:
            continue
        if int(float(row.get("pair_valid", 1))) == 0:
            continue
        if int(float(row.get("main_valid", 1))) == 0:
            continue
        if int(float(row.get("aux_valid", 1))) == 0:
            continue
        valid_rows.append(row)

    if not valid_rows:
        raise RuntimeError(f"{label}: no valid pair samples")

    main_phases = [angle_phase(float(row["main_angle"]), cpr) for row in valid_rows]
    aux_phases = [angle_phase(float(row["aux_angle"]), cpr) for row in valid_rows]
    main_mean = circular_mean(main_phases)
    aux_mean = circular_mean(aux_phases)
    main_spread = max(circular_distance(phase, main_mean) for phase in main_phases) * cpr
    aux_spread = max(circular_distance(phase, aux_mean) for phase in aux_phases) * cpr

    if (main_spread > max_spread_counts or aux_spread > max_spread_counts) and not allow_unstable:
        raise RuntimeError(
            f"{label}: unstable sample spread main={main_spread:.2f} aux={aux_spread:.2f} "
            f"counts; increase --max-spread-counts or pass --allow-unstable"
        )

    return RawPoint(
        label=label,
        main_angle=main_mean * cpr,
        aux_angle=aux_mean * cpr,
        main_spread=main_spread,
        aux_spread=aux_spread,
    )


def collect_point(bus, args, point_index):
    diag_args = SimpleNamespace(
        node_id=args.node_id,
        extended_id=args.extended_id,
        timeout=args.timeout,
    )
    rows = []
    for _ in range(args.samples_per_point):
        data = read_full_pair(bus, diag_args)
        if data.get("main_angle") is not None and data.get("aux_angle") is not None:
            rows.append({
                "pair_valid": data.get("pair_valid"),
                "main_valid": data.get("main_valid"),
                "aux_valid": data.get("aux_valid"),
                "main_angle": data["main_angle"],
                "aux_angle": data["aux_angle"],
            })
        time.sleep(args.period)
    return stable_point_from_rows(
        rows, f"P{point_index}", args.cpr, args.max_spread_counts, args.allow_unstable
    )


def load_points_from_csv(path, args):
    with open(path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    if not rows:
        raise RuntimeError(f"{path}: no CSV rows")

    group_size = args.csv_group_size or len(rows)
    points = []
    for start in range(0, len(rows), group_size):
        group = rows[start:start + group_size]
        if not group:
            continue
        points.append(stable_point_from_rows(
            group,
            f"CSV{len(points) + 1}",
            args.cpr,
            args.max_spread_counts,
            args.allow_unstable,
        ))
    return points


def print_points(points):
    print("Calibration points:")
    for point in points:
        print(
            f"  {point.label}: main={point.main_angle:.3f} "
            f"aux={point.aux_angle:.3f} "
            f"spread_counts M={point.main_spread:.2f} A={point.aux_spread:.2f}"
        )


def print_residual_table(points, old_residuals, new_residuals, config):
    print("Residual comparison:")
    print("  point      old_res    old_deg     new_res    new_deg")
    for point, old_res, new_res in zip(points, old_residuals, new_residuals):
        print(
            f"  {point.label:8s} "
            f"{old_res:+.6f} {output_error_deg(old_res, config):+9.4f} "
            f"{new_res:+.6f} {output_error_deg(new_res, config):+9.4f}"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Fit MT6826S vernier_main_offset/vernier_aux_offset from multiple static points."
    )
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--cpr", type=int, default=MT6826S_CPR)
    parser.add_argument("--main-ratio", type=float, default=42.0)
    parser.add_argument("--aux-ratio", type=float, default=41.0)
    parser.add_argument("--main-reversed", type=int, choices=[0, 1], default=1)
    parser.add_argument("--aux-reversed", type=int, choices=[0, 1], default=0)
    parser.add_argument("--output-reversed", type=int, choices=[0, 1], default=0)
    parser.add_argument("--points", type=int, default=5)
    parser.add_argument("--samples-per-point", type=int, default=20)
    parser.add_argument("--period", type=float, default=0.03)
    parser.add_argument("--max-spread-counts", type=float, default=8.0)
    parser.add_argument("--allow-unstable", action="store_true")
    parser.add_argument("--from-csv")
    parser.add_argument(
        "--csv-group-size", type=int,
        help="Rows per static point when --from-csv is used. Default: all rows are one point.",
    )
    parser.add_argument("--fit-main-offset", action="store_true")
    parser.add_argument("--objective", choices=["rms", "max"], default="rms")
    parser.add_argument("--grid-steps", type=int, default=1024)
    parser.add_argument(
        "--search-radius", type=float, default=0.05,
        help="Circular offset search radius around current offsets. Use 0.5 for full-range search.",
    )
    parser.add_argument("--current-main-offset", type=float)
    parser.add_argument("--current-aux-offset", type=float)
    parser.add_argument("--apply", action="store_true", help="Write fitted offsets to RAM config.")
    parser.add_argument("--save", action="store_true", help="Save config to NVM after --apply.")
    parser.add_argument("--request-idle", action="store_true")
    parser.add_argument(
        "--firmware-fit", action="store_true",
        help="Use firmware subcommand 0x0D to capture points and fit/apply aux offset on the device.",
    )
    args = parser.parse_args()

    if args.save and not args.apply:
        raise RuntimeError("--save requires --apply")
    if args.from_csv and args.apply:
        raise RuntimeError("--apply is only supported during live CAN sampling, not --from-csv")
    if args.from_csv and args.firmware_fit:
        raise RuntimeError("--firmware-fit requires live CAN sampling, not --from-csv")
    if args.points < 2 and not args.from_csv:
        raise RuntimeError("--points must be at least 2")
    if abs(args.aux_ratio - args.main_ratio) < 1.0e-9:
        raise RuntimeError("main_ratio and aux_ratio must differ")

    config = FitConfig(
        cpr=args.cpr,
        main_ratio=args.main_ratio,
        aux_ratio=args.aux_ratio,
        main_reversed=bool(args.main_reversed),
        aux_reversed=bool(args.aux_reversed),
        output_reversed=bool(args.output_reversed),
    )

    if args.from_csv:
        points = load_points_from_csv(args.from_csv, args)
        current_main_offset = args.current_main_offset if args.current_main_offset is not None else 0.0
        current_aux_offset = args.current_aux_offset if args.current_aux_offset is not None else 0.0
    else:
        bus = open_bus(args.channel, args.bitrate)
        try:
            print(f"Opened {bus.channel_info}")
            hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
            if hb is None:
                raise RuntimeError("No heartbeat received")
            print(
                f"Heartbeat: state={hb['axis_state']} "
                f"axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}"
            )
            if args.request_idle:
                print("Requesting IDLE before calibration sampling")
                set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
                time.sleep(0.2)

            current_main_offset = read_basic_float(
                bus, args, PARAM_MAIN_OFFSET, 0.0, "current vernier_main_offset"
            )
            current_aux_offset = read_basic_float(
                bus, args, PARAM_AUX_OFFSET, 0.0, "current vernier_aux_offset"
            )
            if args.current_main_offset is not None:
                current_main_offset = args.current_main_offset
                print(f"override current vernier_main_offset: {current_main_offset:.9g}")
            if args.current_aux_offset is not None:
                current_aux_offset = args.current_aux_offset
                print(f"override current vernier_aux_offset: {current_aux_offset:.9g}")
            err_accept = read_basic_float(
                bus, args, PARAM_ERR_ACCEPT, 0.020, "current vernier_err_accept"
            )
            err_reject = read_basic_float(
                bus, args, PARAM_ERR_REJECT, 0.080, "current vernier_err_reject"
            )

            if args.firmware_fit:
                print()
                print("Using firmware vernier calibration subcommand 0x0D")
                ok = print_calib_response(
                    vernier_calib_request(bus, args, 0x00),
                    "firmware reset points",
                )
                if not ok:
                    return 1

                for index in range(1, args.points + 1):
                    if index == 1:
                        prompt = (
                            f"Place the joint at calibration point {index}/{args.points}, "
                            "then press Enter"
                        )
                    else:
                        prompt = (
                            f"Move to another reachable static point {index}/{args.points}, "
                            "then press Enter"
                        )
                    text = input(prompt + " (q to finish early): ").strip().lower()
                    if text in {"q", "quit", "done"}:
                        break
                    time.sleep(max(0.0, args.period))
                    ok = print_calib_response(
                        vernier_calib_request(bus, args, 0x01),
                        f"firmware capture P{index}",
                    )
                    if not ok:
                        return 1

                count_resp = vernier_calib_request(bus, args, 0x04)
                point_count = firmware_value(count_resp)
                if point_count is None or int(point_count) < 2:
                    raise RuntimeError("firmware calibration needs at least two captured points")

                fit_resp = vernier_calib_request(
                    bus, args, 0x02,
                    req_type=EXT_TYPE_FLOAT32,
                    value_float=args.search_radius,
                )
                if not print_calib_response(fit_resp, "firmware fit aux offset"):
                    return 1

                fitted_main = firmware_value(vernier_calib_request(bus, args, 0x06))
                fitted_aux = firmware_value(vernier_calib_request(bus, args, 0x07))
                fit_score = firmware_value(vernier_calib_request(bus, args, 0x08))
                worst = firmware_value(vernier_calib_request(bus, args, 0x09))

                print()
                print(
                    f"Old offsets: main={current_main_offset:.9g} "
                    f"aux={current_aux_offset:.9g}"
                )
                print(
                    f"Firmware fit: points={int(point_count)} "
                    f"main={fitted_main:.9g} aux={fitted_aux:.9g} "
                    f"score={fit_score:.6f} worst={worst:.6f}"
                )
                if worst is not None and worst > err_reject:
                    print(
                        f"WARNING: worst residual {worst:.6f} exceeds "
                        f"vernier_err_reject {err_reject:.6f}; offsets alone do not "
                        "explain these points well."
                    )
                elif worst is not None and worst > err_accept:
                    print(
                        f"NOTE: worst residual {worst:.6f} is above "
                        f"vernier_err_accept {err_accept:.6f}."
                    )

                if args.apply:
                    if not print_calib_response(
                        vernier_calib_request(bus, args, 0x03),
                        "firmware apply fit",
                    ):
                        return 1
                    if args.save:
                        resp = save_configuration(bus, args.node_id, args.extended_id, timeout=2.0)
                        if resp is None:
                            print("save_configuration: TIMEOUT; reset may already be in progress")
                            return 1
                        status = EXT_STATUS.get(resp["status"], resp["status"])
                        print(f"save_configuration: {status} raw={resp['raw']}")
                        if resp["status"] != 0:
                            return 1
                return 0

            points = []
            for index in range(1, args.points + 1):
                if index == 1:
                    prompt = (
                        f"Place the joint at calibration point {index}/{args.points}, "
                        "then press Enter"
                    )
                else:
                    prompt = (
                        f"Move to another reachable static point {index}/{args.points}, "
                        "then press Enter"
                    )
                text = input(prompt + " (q to finish early): ").strip().lower()
                if text in {"q", "quit", "done"}:
                    break
                point = collect_point(bus, args, index)
                print(
                    f"  captured {point.label}: main={point.main_angle:.3f} "
                    f"aux={point.aux_angle:.3f} "
                    f"spread M={point.main_spread:.2f} A={point.aux_spread:.2f} counts"
                )
                points.append(point)

            if len(points) < 2:
                raise RuntimeError("need at least two calibration points")

            print_points(points)
            old_score, old_residuals = objective_value(
                points, current_main_offset, current_aux_offset, config, args.objective
            )
            main_offset, aux_offset, new_score, new_residuals = fit_offsets(
                points, config, current_main_offset, current_aux_offset, args
            )

            print()
            print(f"Objective: {args.objective}")
            print(
                f"Old offsets: main={current_main_offset:.9g} "
                f"aux={current_aux_offset:.9g} score={old_score:.6f}"
            )
            print(
                f"New offsets: main={main_offset:.9g} "
                f"aux={aux_offset:.9g} score={new_score:.6f}"
            )
            print(
                f"Centered view: main={offset_for_print(main_offset):+.9g} "
                f"aux={offset_for_print(aux_offset):+.9g}"
            )
            print_residual_table(points, old_residuals, new_residuals, config)

            worst_new = max(abs(value) for value in new_residuals)
            if worst_new > err_reject:
                print(
                    f"WARNING: worst residual {worst_new:.6f} exceeds "
                    f"vernier_err_reject {err_reject:.6f}; offsets alone do not "
                    "explain these points well."
                )
            elif worst_new > err_accept:
                print(
                    f"NOTE: worst residual {worst_new:.6f} is above "
                    f"vernier_err_accept {err_accept:.6f}; this is usable for "
                    "coarse branch selection but not high-precision absolute feedback."
                )

            if args.apply:
                print()
                ok = True
                main_write = offset_for_print(main_offset)
                aux_write = offset_for_print(aux_offset)
                if args.fit_main_offset:
                    ok &= set_basic_float(
                        bus, args, PARAM_MAIN_OFFSET, main_write,
                        "encoder.vernier_main_offset",
                    )
                else:
                    print("Keeping encoder.vernier_main_offset unchanged")
                ok &= set_basic_float(
                    bus, args, PARAM_AUX_OFFSET, aux_write,
                    "encoder.vernier_aux_offset",
                )
                if args.save:
                    resp = save_configuration(bus, args.node_id, args.extended_id, timeout=2.0)
                    if resp is None:
                        print("save_configuration: TIMEOUT; reset may already be in progress")
                        ok = False
                    else:
                        status = EXT_STATUS.get(resp["status"], resp["status"])
                        print(f"save_configuration: {status} raw={resp['raw']}")
                        ok &= resp["status"] == 0
                return 0 if ok else 1
            return 0
        finally:
            bus.shutdown()

    if len(points) < 2:
        raise RuntimeError("need at least two calibration points")

    print_points(points)
    old_score, old_residuals = objective_value(
        points, current_main_offset, current_aux_offset, config, args.objective
    )
    main_offset, aux_offset, new_score, new_residuals = fit_offsets(
        points, config, current_main_offset, current_aux_offset, args
    )
    print(f"Objective: {args.objective}")
    print(f"Old offsets: main={current_main_offset:.9g} aux={current_aux_offset:.9g} score={old_score:.6f}")
    print(f"New offsets: main={main_offset:.9g} aux={aux_offset:.9g} score={new_score:.6f}")
    print(
        f"Centered view: main={offset_for_print(main_offset):+.9g} "
        f"aux={offset_for_print(aux_offset):+.9g}"
    )
    print_residual_table(points, old_residuals, new_residuals, config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
