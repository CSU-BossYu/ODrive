import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.calibration_geometry import fit_relative_angle_model, wrap01
from odrive_can.calibration_record import CalibrationSampleV1


CPU_HZ = 168_000_000.0
POLE_PAIRS = 7
MAIN_RATIO = 42.0
AUX_RATIO = 41.0


def make_sample(sequence: int, x: float, direction: int) -> CalibrationSampleV1:
    common = 0.0015 * math.sin(2.0 * math.pi * x)
    directional = direction * 0.0004 * math.cos(4.0 * math.pi * x)
    observed = x + common + directional
    electrical_velocity = direction * 0.05 * MAIN_RATIO * POLE_PAIRS * 2.0 * math.pi
    skew_cycles = 240
    skew_seconds = skew_cycles / CPU_HZ
    aux_velocity = direction * 0.05 * AUX_RATIO
    main_phase = wrap01(MAIN_RATIO * observed)
    aux_phase = wrap01(AUX_RATIO * observed + aux_velocity * skew_seconds)
    electrical_phase = ((x * MAIN_RATIO * POLE_PAIRS * 2.0 * math.pi + math.pi) %
                        (2.0 * math.pi)) - math.pi

    return CalibrationSampleV1(
        sample_ticks=sequence * 168_000,
        control_ticks=sequence * 21_000,
        sequence=sequence,
        session_id=9,
        main_sequence=sequence,
        aux_sequence=sequence,
        pair_sequence=sequence,
        pair_sample_skew_cycles=skew_cycles,
        main_raw=round(main_phase * 32768.0) % 32768,
        aux_raw=round(aux_phase * 32768.0) % 32768,
        axis_state=4,
        flags=(1 << 1) | (1 << 2),
        ia=0.0, ib=0.0, ic=0.0, id=0.0, iq=0.0,
        vd_applied=0.0, vq_applied=0.0, vbus=24.0,
        duty_a=0.5, duty_b=0.5, duty_c=0.5,
        position_turns=observed,
        velocity_turns_per_s=direction * 0.05,
        electrical_phase=electrical_phase,
        electrical_velocity=electrical_velocity,
        output_position_turns=observed,
        board_temperature=30.0,
        motor_temperature=31.0,
    )


class CalibrationGeometryTests(unittest.TestCase):
    def test_bidirectional_lut_reduces_relative_angle_error(self):
        count = 4096
        samples = [make_sample(i, 2.0 * i / (count - 1), 1)
                   for i in range(count)]
        samples += [make_sample(count + i, 2.0 * (1.0 - i / (count - 1)), -1)
                    for i in range(count)]

        fit = fit_relative_angle_model(
            samples, pole_pairs=POLE_PAIRS, main_ratio=MAIN_RATIO,
            aux_ratio=AUX_RATIO, cpu_hz=CPU_HZ, bins=64)

        self.assertAlmostEqual(fit.effective_ratio_scale, 1.0, delta=0.002)
        self.assertGreater(fit.raw_rms_turns, 5e-4)
        self.assertLess(fit.corrected_rms_turns, fit.raw_rms_turns * 0.40)
        self.assertGreater(fit.direction_peak_to_peak_turns, 3e-4)
        self.assertEqual(fit.used_samples, count * 2)

    def test_requires_both_scan_directions(self):
        samples = [make_sample(i, 2.0 * i / 511.0, 1)
                   for i in range(512)]
        with self.assertRaisesRegex(ValueError, 'forward and reverse'):
            fit_relative_angle_model(
                samples, pole_pairs=POLE_PAIRS, main_ratio=MAIN_RATIO,
                aux_ratio=AUX_RATIO, cpu_hz=CPU_HZ, bins=32)


if __name__ == '__main__':
    unittest.main()
