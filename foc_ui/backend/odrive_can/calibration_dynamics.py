"""Reference fitters for motor flux and output-shaft dynamics calibration."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .calibration_record import CalibrationSampleV1


FLAG_CURRENT_VALID = 1 << 0
FLAG_APPLIED_VOLTAGE_VALID = 1 << 6
FLAG_PWM_SATURATED = 1 << 8


@dataclass(frozen=True)
class FluxLinkageFit:
    flux_linkage: float
    torque_constant: float
    sample_stddev: float
    used_samples: int


@dataclass(frozen=True)
class MechanicalFit:
    output_inertia: float
    coulomb_pos: float
    coulomb_neg: float
    viscous_pos: float
    viscous_neg: float
    residual_rms_torque: float
    used_samples: int


@dataclass(frozen=True)
class ElectricalDelayFit:
    electrical_delay: float
    residual_phase_offset: float
    residual_rms: float
    used_samples: int


def _solve_linear(matrix: list[list[float]]) -> list[float]:
    size = len(matrix)
    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(matrix[row][pivot]))
        if abs(matrix[best][pivot]) < 1e-10:
            raise ValueError('mechanical excitation is not observable')
        matrix[pivot], matrix[best] = matrix[best], matrix[pivot]
        divisor = matrix[pivot][pivot]
        matrix[pivot] = [value / divisor for value in matrix[pivot]]
        for row in range(size):
            if row == pivot:
                continue
            factor = matrix[row][pivot]
            matrix[row] = [value - factor * reference for value, reference in
                           zip(matrix[row], matrix[pivot])]
    return [matrix[row][-1] for row in range(size)]


def fit_mechanical_dynamics(
        observations: Sequence[tuple[float, float, float]], *,
        torque_constant: float, motor_turns_per_output_turn: float) -> MechanicalFit:
    """Fit J, asymmetric Coulomb and viscous friction from (v, a, iq)."""
    rows: list[tuple[list[float], float]] = []
    for velocity, acceleration, iq in observations:
        if abs(velocity) < 0.005:
            continue
        x = [acceleration, 0.0, 0.0, 0.0, 0.0]
        if velocity > 0:
            x[1], x[3] = 1.0, velocity
        else:
            x[2], x[4] = -1.0, velocity
        rows.append((x, iq))
    if len(rows) < 10:
        raise ValueError('not enough mechanical observations')
    size = 5
    xtx = [[sum(x[i] * x[j] for x, _ in rows)
            for j in range(size)] for i in range(size)]
    xty = [sum(x[i] * y for x, y in rows) for i in range(size)]
    beta = _solve_linear([xtx[i] + [xty[i]] for i in range(size)])
    if beta[1] + beta[2] < 0:
        beta = [-value for value in beta]
    if any(value < 0 for value in beta):
        raise ValueError('mechanical fit produced nonphysical parameters')
    torque_scale = torque_constant * abs(motor_turns_per_output_turn)
    errors = [y - sum(value * feature for value, feature in zip(beta, x))
              for x, y in rows]
    return MechanicalFit(
        output_inertia=beta[0] * torque_scale,
        coulomb_pos=beta[1] * torque_scale,
        coulomb_neg=beta[2] * torque_scale,
        viscous_pos=beta[3] * torque_scale,
        viscous_neg=beta[4] * torque_scale,
        residual_rms_torque=(sum(error * error for error in errors) /
                             len(errors)) ** 0.5 * torque_scale,
        used_samples=len(rows),
    )


def identify_integer_pole_pairs(*, electrical_scan_radians: float,
                                encoder_delta_counts: int,
                                encoder_cpr: int,
                                relative_tolerance: float = 0.02) -> int:
    """Identify integer pole pairs from open-loop electrical travel."""
    if electrical_scan_radians <= 0 or abs(encoder_delta_counts) < 8 or \
            encoder_cpr <= 0:
        raise ValueError('pole-pair scan did not produce usable motion')
    estimate = (electrical_scan_radians * encoder_cpr /
                (2.0 * math.pi * abs(encoder_delta_counts)))
    candidate = round(estimate)
    if not 1 <= candidate <= 128 or \
            abs(estimate - candidate) / candidate > relative_tolerance:
        raise ValueError('pole-pair estimate is not close to an integer')
    return candidate


def fit_electrical_delay(samples: Sequence[CalibrationSampleV1], *,
                         phase_resistance: float, phase_inductance: float,
                         minimum_electrical_speed: float = 30.0
                         ) -> ElectricalDelayFit:
    """Fit frame delay from back-EMF phase error versus electrical speed."""
    if phase_resistance <= 0 or phase_inductance <= 0:
        raise ValueError('motor electrical parameters must be positive')
    required = FLAG_CURRENT_VALID | FLAG_APPLIED_VOLTAGE_VALID
    raw_points: list[tuple[float, float]] = []
    directions = {1: 0, -1: 0}
    for sample in samples:
        if ((sample.flags & required) != required or
                sample.flags & FLAG_PWM_SATURATED or
                abs(sample.electrical_velocity) < minimum_electrical_speed):
            continue
        omega = sample.electrical_velocity
        ed = (sample.vd_applied - phase_resistance * sample.id +
              omega * phase_inductance * sample.iq)
        eq = (sample.vq_applied - phase_resistance * sample.iq -
              omega * phase_inductance * sample.id)
        if math.hypot(ed, eq) < 0.05:
            continue
        direction = 1.0 if omega > 0 else -1.0
        phase_error = math.atan2(-direction * ed, direction * eq)
        if not math.isfinite(phase_error) or abs(phase_error) > 0.5:
            continue
        raw_points.append((omega, phase_error))
        directions[1 if omega > 0 else -1] += 1
    if min(directions.values()) < 500:
        raise ValueError('both delay scan directions need at least 500 samples')
    block_size = 32
    points: list[tuple[float, float]] = []
    block: list[tuple[float, float]] = []
    previous_omega: float | None = None
    for point in raw_points:
        omega = point[0]
        segment_changed = (previous_omega is not None and
                           (omega * previous_omega <= 0 or
                            abs(omega - previous_omega) >
                            0.25 * max(abs(previous_omega), 1.0)))
        if segment_changed:
            block = []
        block.append(point)
        if len(block) == block_size:
            points.append((sum(x for x, _ in block) / block_size,
                           sum(y for _, y in block) / block_size))
            block = []
        previous_omega = omega
    if len(points) < 16:
        raise ValueError('not enough delay regression blocks')
    n = len(points)
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    sxy = sum(x * y for x, y in points)
    denominator = n * sxx - sx * sx
    if denominator <= 1e-6:
        raise ValueError('electrical speed excitation is not observable')
    delay = (n * sxy - sx * sy) / denominator
    intercept = (sy - delay * sx) / n
    rms = math.sqrt(sum((y - intercept - delay * x) ** 2
                        for x, y in points) / n)
    if not 0 <= delay <= 500e-6 or abs(intercept) > 0.20 or rms > 0.10:
        raise ValueError('electrical delay fit failed physical limits')
    return ElectricalDelayFit(delay, intercept, rms, len(raw_points))


def fit_flux_linkage(samples: Sequence[CalibrationSampleV1], *,
                     phase_resistance: float, phase_inductance: float,
                     pole_pairs: int,
                     minimum_electrical_speed: float = 20.0) -> FluxLinkageFit:
    if phase_resistance <= 0 or phase_inductance <= 0 or pole_pairs <= 0:
        raise ValueError('motor electrical parameters must be positive')
    directions: dict[int, list[float]] = {1: [], -1: []}
    required = FLAG_CURRENT_VALID | FLAG_APPLIED_VOLTAGE_VALID
    for sample in samples:
        if ((sample.flags & required) != required or
                sample.flags & FLAG_PWM_SATURATED or
                abs(sample.electrical_velocity) < minimum_electrical_speed):
            continue
        flux = ((sample.vq_applied - phase_resistance * sample.iq) /
                sample.electrical_velocity - phase_inductance * sample.id)
        if math.isfinite(flux) and 1e-6 < flux <= 1.0:
            directions[1 if sample.electrical_velocity > 0 else -1].append(flux)
    if min(map(len, directions.values())) < 128:
        raise ValueError('both flux scan directions need at least 128 samples')
    block_size = 32
    blocks = {
        direction: [sum(values[i:i + block_size]) / block_size
                    for i in range(0, len(values) - block_size + 1, block_size)]
        for direction, values in directions.items()
    }
    if min(map(len, blocks.values())) < 4:
        raise ValueError('both flux scan directions need at least four blocks')
    means = [sum(values) / len(values) for values in blocks.values()]
    flux_linkage = 0.5 * sum(means)
    squared_error = sum(
        sum((value - sum(values) / len(values)) ** 2 for value in values)
        for values in blocks.values())
    block_count = sum(map(len, blocks.values()))
    used = sum(map(len, directions.values()))
    stddev = math.sqrt(squared_error / max(block_count - 2, 1))
    if stddev > flux_linkage * 0.25:
        raise ValueError('flux estimate dispersion is too high')
    return FluxLinkageFit(
        flux_linkage=flux_linkage,
        torque_constant=1.5 * pole_pairs * flux_linkage,
        sample_stddev=stddev,
        used_samples=used,
    )
