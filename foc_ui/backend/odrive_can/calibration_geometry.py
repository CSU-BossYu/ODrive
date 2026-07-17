"""Fit an observable relative-angle model from bidirectional ODCR samples."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .calibration_record import CalibrationSampleV1


FLAG_MAIN_VALID = 1 << 1
FLAG_AUX_VALID = 1 << 2


def wrap01(value: float) -> float:
    return value - math.floor(value)


def wrap_pm_half(value: float) -> float:
    return value - math.floor(value + 0.5)


def wrap_pm_pi(value: float) -> float:
    return value - 2.0 * math.pi * math.floor(
        value / (2.0 * math.pi) + 0.5)


@dataclass(frozen=True)
class RelativeAngleFit:
    bins: int
    effective_ratio_scale: float
    common_correction: tuple[float, ...]
    direction_correction: tuple[float, ...]
    raw_rms_turns: float
    corrected_rms_turns: float
    direction_peak_to_peak_turns: float
    used_samples: int
    mean_pair_skew_seconds: float

    def compensate(self, position_turns: float, velocity_sign: int) -> float:
        phase = wrap01(position_turns)
        bin_position = phase * self.bins
        index0 = int(bin_position) % self.bins
        index1 = (index0 + 1) % self.bins
        fraction = bin_position - math.floor(bin_position)
        common = (self.common_correction[index0] + fraction *
                  (self.common_correction[index1] -
                   self.common_correction[index0]))
        directional = (self.direction_correction[index0] + fraction *
                       (self.direction_correction[index1] -
                        self.direction_correction[index0]))
        sign = 1.0 if velocity_sign > 0 else -1.0 if velocity_sign < 0 else 0.0
        corrected_observed = position_turns + common + sign * directional
        return corrected_observed / self.effective_ratio_scale


@dataclass
class _Point:
    observed: float
    reference: float
    direction: int
    phase: float


def _normalize_raw(raw: int, reversed_axis: bool) -> float:
    phase = float(raw) / 32768.0
    return wrap01(-phase if reversed_axis else phase)


def _resolve_output_phase(main: float, aux: float, main_ratio: float,
                          aux_ratio: float, main_offset: float,
                          aux_offset: float, output_reversed: bool) -> float:
    main_corr = wrap01(main - main_offset)
    aux_corr = wrap01(aux - aux_offset)
    ratio_delta = aux_ratio - main_ratio
    coarse = wrap01(wrap_pm_half(aux_corr - main_corr) / ratio_delta)
    predicted_main = wrap01(main_ratio * coarse)
    main_residual = wrap_pm_half(main_corr - predicted_main)
    refined = wrap01(coarse + main_residual / main_ratio)
    return wrap01(-refined if output_reversed else refined)


def _fill_circular(values: list[float | None]) -> list[float]:
    known = [i for i, value in enumerate(values) if value is not None]
    if not known:
        raise ValueError('geometry scan has no populated LUT bins')
    result = [0.0] * len(values)
    for index, value in enumerate(values):
        if value is not None:
            result[index] = value
            continue
        nearest = min(
            known,
            key=lambda candidate: min(
                (candidate - index) % len(values),
                (index - candidate) % len(values)))
        result[index] = float(values[nearest])
    return result


def fit_relative_angle_model(
        samples: Sequence[CalibrationSampleV1], *, pole_pairs: int,
        main_ratio: float, aux_ratio: float, cpu_hz: float,
        main_offset: float = 0.0, aux_offset: float = 0.0,
        main_reversed: bool = False, aux_reversed: bool = False,
        output_reversed: bool = False, bins: int = 64,
        minimum_speed: float = 1.0) -> RelativeAngleFit:
    if pole_pairs <= 0 or abs(main_ratio) < 1e-6:
        raise ValueError('pole_pairs and main_ratio must be valid')
    if abs(aux_ratio - main_ratio) < 1e-6:
        raise ValueError('vernier ratios must be different')
    if cpu_hz <= 0 or bins < 8:
        raise ValueError('cpu_hz and bins must be valid')

    valid = [sample for sample in samples
             if (sample.flags & (FLAG_MAIN_VALID | FLAG_AUX_VALID)) ==
             (FLAG_MAIN_VALID | FLAG_AUX_VALID)
             and abs(sample.electrical_velocity) >= minimum_speed]
    if len(valid) < bins * 2:
        raise ValueError('not enough valid moving geometry samples')

    segments: list[list[CalibrationSampleV1]] = []
    for sample in valid:
        direction = 1 if sample.electrical_velocity > 0 else -1
        if not segments or (
                (segments[-1][-1].electrical_velocity > 0) != (direction > 0)):
            segments.append([])
        segments[-1].append(sample)
    segments = [segment for segment in segments if len(segment) >= bins // 2]
    if {1 if segment[0].electrical_velocity > 0 else -1
            for segment in segments} != {-1, 1}:
        raise ValueError('both forward and reverse scans are required')

    points: list[_Point] = []
    skew_sum = 0.0
    skew_count = 0
    for segment in segments:
        direction = 1 if segment[0].electrical_velocity > 0 else -1
        previous_electrical = segment[0].electrical_phase
        electrical_unwrapped = previous_electrical
        previous_output: float | None = None
        output_unwrapped = 0.0
        segment_points: list[_Point] = []

        for sample in segment:
            electrical_unwrapped += wrap_pm_pi(
                sample.electrical_phase - previous_electrical)
            previous_electrical = sample.electrical_phase

            main = _normalize_raw(sample.main_raw, main_reversed)
            aux = _normalize_raw(sample.aux_raw, aux_reversed)
            skew_seconds = sample.pair_sample_skew_cycles / cpu_hz
            aux_velocity = (sample.electrical_velocity /
                            (2.0 * math.pi * pole_pairs * main_ratio) *
                            aux_ratio)
            aux = wrap01(aux - aux_velocity * skew_seconds)
            skew_sum += skew_seconds
            skew_count += 1

            output = _resolve_output_phase(
                main, aux, main_ratio, aux_ratio, main_offset, aux_offset,
                output_reversed)
            if previous_output is None:
                output_unwrapped = output
            else:
                output_unwrapped += wrap_pm_half(output - previous_output)
            previous_output = output
            motor_turns = electrical_unwrapped / (2.0 * math.pi * pole_pairs)
            reference = motor_turns / main_ratio
            segment_points.append(_Point(
                output_unwrapped, reference, direction, output_unwrapped))

        ref_mean = sum(point.reference for point in segment_points) / len(segment_points)
        obs_mean = sum(point.observed for point in segment_points) / len(segment_points)
        for point in segment_points:
            points.append(_Point(
                point.observed - obs_mean,
                point.reference - ref_mean,
                point.direction,
                point.phase))

    denominator = sum(point.reference * point.reference for point in points)
    if denominator <= 1e-12:
        raise ValueError('geometry reference motion is degenerate')
    scale = sum(point.reference * point.observed for point in points) / denominator

    bin_sums = {1: [0.0] * bins, -1: [0.0] * bins}
    bin_counts = {1: [0] * bins, -1: [0] * bins}
    residuals: list[tuple[float, int, float]] = []
    for point in points:
        residual = point.observed - scale * point.reference
        index = min(int(wrap01(point.phase) * bins), bins - 1)
        bin_sums[point.direction][index] += residual
        bin_counts[point.direction][index] += 1
        residuals.append((residual, point.direction, wrap01(point.phase)))

    direction_curves: dict[int, list[float]] = {}
    for direction in (1, -1):
        means: list[float | None] = [
            bin_sums[direction][i] / bin_counts[direction][i]
            if bin_counts[direction][i] else None
            for i in range(bins)]
        direction_curves[direction] = _fill_circular(means)

    common = [(direction_curves[1][i] + direction_curves[-1][i]) * 0.5
              for i in range(bins)]
    hysteresis = [(direction_curves[1][i] - direction_curves[-1][i]) * 0.5
                  for i in range(bins)]
    common_correction = tuple(-value for value in common)
    direction_correction = tuple(-value for value in hysteresis)
    raw_rms = math.sqrt(sum(value * value for value, _, _ in residuals) /
                        len(residuals))
    corrected_errors: list[float] = []
    for value, direction, phase in residuals:
        bin_position = phase * bins
        index0 = int(bin_position) % bins
        index1 = (index0 + 1) % bins
        fraction = bin_position - math.floor(bin_position)
        common_value = (common_correction[index0] + fraction *
                        (common_correction[index1] -
                         common_correction[index0]))
        direction_value = (direction_correction[index0] + fraction *
                           (direction_correction[index1] -
                            direction_correction[index0]))
        corrected_errors.append(
            value + common_value + direction * direction_value)
    corrected_rms = math.sqrt(
        sum(value * value for value in corrected_errors) /
        len(corrected_errors))

    return RelativeAngleFit(
        bins=bins,
        effective_ratio_scale=scale,
        common_correction=common_correction,
        direction_correction=direction_correction,
        raw_rms_turns=raw_rms,
        corrected_rms_turns=corrected_rms,
        direction_peak_to_peak_turns=2.0 * max(abs(v) for v in hysteresis),
        used_samples=len(points),
        mean_pair_skew_seconds=skew_sum / max(skew_count, 1),
    )
