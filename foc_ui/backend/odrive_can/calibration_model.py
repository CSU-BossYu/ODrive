"""Calibration parameter ownership, runtime consumers, and stale propagation.

This is deliberately a small explicit graph. A quantity is admitted only when
it has a runtime consumer; report-only diagnostics belong in session artifacts,
not in the committed calibration parameter set.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable


class ParameterStatus(IntEnum):
    MISSING = 0
    IDENTIFIED = 1
    VALIDATED = 2
    APPLICABLE = 3
    STALE = 4


@dataclass(frozen=True)
class CalibrationParameter:
    key: str
    dependencies: tuple[str, ...]
    consumers: tuple[str, ...]


PARAMETERS: dict[str, CalibrationParameter] = {
    'current_offset': CalibrationParameter(
        'current_offset', (), ('adc_current_reconstruction',)),
    'phase_current_balance': CalibrationParameter(
        'phase_current_balance', (), ('adc_current_reconstruction',)),
    'absolute_current_scale': CalibrationParameter(
        'absolute_current_scale', (),
        ('adc_current_reconstruction', 'current_limits', 'torque_estimate')),
    'vbus_scale': CalibrationParameter(
        'vbus_scale', (), ('pwm_voltage_reconstruction', 'bus_protection')),
    'inverter_voltage_model': CalibrationParameter(
        'inverter_voltage_model', ('absolute_current_scale', 'vbus_scale'),
        ('pwm_voltage_compensation', 'pwm_voltage_reconstruction')),
    'phase_resistance': CalibrationParameter(
        'phase_resistance',
        ('absolute_current_scale', 'vbus_scale', 'inverter_voltage_model'),
        ('current_controller_gains', 'motor_voltage_model')),
    'phase_inductance': CalibrationParameter(
        'phase_inductance', ('absolute_current_scale', 'vbus_scale'),
        ('current_controller_gains', 'dq_decoupling')),
    'phase_sequence': CalibrationParameter(
        'phase_sequence', (), ('pwm_phase_mapping',)),
    'encoder_direction': CalibrationParameter(
        'encoder_direction', (), ('electrical_angle', 'output_coordinates')),
    'pole_pairs': CalibrationParameter(
        'pole_pairs', (), ('electrical_angle', 'torque_constant_derivation')),
    'vernier_offsets': CalibrationParameter(
        'vernier_offsets', ('encoder_direction',), ('vernier_resolver',)),
    'vernier_sample_skew': CalibrationParameter(
        'vernier_sample_skew', ('encoder_direction',), ('vernier_resolver',)),
    'gear_ratio_scale': CalibrationParameter(
        'gear_ratio_scale', ('vernier_offsets',),
        ('vernier_resolver', 'output_coordinates')),
    'relative_angle_model': CalibrationParameter(
        'relative_angle_model',
        ('vernier_offsets', 'vernier_sample_skew', 'gear_ratio_scale'),
        ('vernier_resolver',)),
    'electrical_offset': CalibrationParameter(
        'electrical_offset',
        ('phase_sequence', 'encoder_direction', 'pole_pairs',
         'relative_angle_model'),
        ('electrical_angle',)),
    'electrical_delay': CalibrationParameter(
        'electrical_delay', ('electrical_offset',), ('electrical_angle',)),
    'flux_linkage': CalibrationParameter(
        'flux_linkage',
        ('absolute_current_scale', 'inverter_voltage_model',
         'phase_resistance', 'phase_inductance', 'pole_pairs',
         'electrical_offset', 'electrical_delay'),
        ('bemf_feedforward', 'torque_constant_derivation')),
    'friction_current_model': CalibrationParameter(
        'friction_current_model',
        ('current_offset', 'phase_current_balance', 'relative_angle_model'),
        ('friction_current_feedforward',)),
    'equivalent_output_inertia': CalibrationParameter(
        'equivalent_output_inertia',
        ('absolute_current_scale', 'flux_linkage', 'friction_current_model',
         'gear_ratio_scale', 'relative_angle_model'),
        ('acceleration_feedforward',)),
}


def validate_parameter_graph() -> None:
    """Raise ValueError for missing dependencies, cycles, or dead parameters."""
    for parameter in PARAMETERS.values():
        if not parameter.consumers:
            raise ValueError(f'{parameter.key} has no runtime consumer')
        for dependency in parameter.dependencies:
            if dependency not in PARAMETERS:
                raise ValueError(
                    f'{parameter.key} depends on unknown {dependency}')

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(key: str) -> None:
        if key in visiting:
            raise ValueError(f'calibration dependency cycle at {key}')
        if key in visited:
            return
        visiting.add(key)
        for dependency in PARAMETERS[key].dependencies:
            visit(dependency)
        visiting.remove(key)
        visited.add(key)

    for key in PARAMETERS:
        visit(key)


def stale_closure(changed: Iterable[str]) -> set[str]:
    """Return changed parameters and every transitive downstream dependent."""
    stale = set(changed)
    unknown = stale.difference(PARAMETERS)
    if unknown:
        raise KeyError(f'unknown calibration parameters: {sorted(unknown)}')

    changed_this_pass = True
    while changed_this_pass:
        changed_this_pass = False
        for key, parameter in PARAMETERS.items():
            if key not in stale and stale.intersection(parameter.dependencies):
                stale.add(key)
                changed_this_pass = True
    return stale


validate_parameter_graph()
