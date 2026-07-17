import dataclasses
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.calibration_dynamics import (
    fit_electrical_delay, fit_flux_linkage, fit_mechanical_dynamics,
    identify_integer_pole_pairs)
from odrive_can.calibration_record import CalibrationSampleV1


def sample(sequence: int, direction: int, flux: float,
           voltage_bias: float) -> CalibrationSampleV1:
    resistance = 0.12
    inductance = 0.00018
    omega = direction * 120.0
    id_current = 2.0
    iq_current = direction * 0.08
    vq = (resistance * iq_current + omega *
          (inductance * id_current + flux) + voltage_bias)
    return CalibrationSampleV1(
        sample_ticks=sequence, control_ticks=sequence, sequence=sequence,
        session_id=1, main_sequence=sequence, aux_sequence=sequence,
        pair_sequence=sequence, pair_sample_skew_cycles=0,
        main_raw=0, aux_raw=0, axis_state=4,
        flags=(1 << 0) | (1 << 6),
        ia=0.0, ib=0.0, ic=0.0, id=id_current, iq=iq_current,
        vd_applied=0.0, vq_applied=vq, vbus=24.0,
        duty_a=0.5, duty_b=0.5, duty_c=0.5,
        position_turns=0.0, velocity_turns_per_s=0.0,
        electrical_phase=0.0, electrical_velocity=omega,
        output_position_turns=0.0, board_temperature=30.0,
        motor_temperature=31.0)


class CalibrationDynamicsTests(unittest.TestCase):
    def test_bidirectional_electrical_delay_fit(self):
        delay = 42e-6
        offset = 0.007
        resistance = 0.12
        inductance = 0.00018
        samples = []
        sequence = 0
        for omega in (60.0, 120.0, 180.0, -60.0, -120.0, -180.0):
            phase_error = offset + delay * omega
            emf = 1.2
            direction = 1.0 if omega > 0 else -1.0
            ed = -direction * emf * math.sin(phase_error)
            eq = direction * emf * math.cos(phase_error)
            for _ in range(500):
                s = sample(sequence, 1 if omega > 0 else -1, 0.0032, 0.0)
                s = dataclasses.replace(
                    s, electrical_velocity=omega, id=0.7, iq=0.2,
                    vd_applied=ed + resistance * 0.7 - omega * inductance * 0.2,
                    vq_applied=eq + resistance * 0.2 + omega * inductance * 0.7)
                samples.append(s)
                sequence += 1
        fit = fit_electrical_delay(
            samples, phase_resistance=resistance, phase_inductance=inductance)
        self.assertAlmostEqual(fit.electrical_delay, delay, places=9)
        self.assertAlmostEqual(fit.residual_phase_offset, offset, places=7)
        self.assertEqual(fit.used_samples, 3000)

    def test_electrical_delay_block_average_rejects_phase_ripple(self):
        delay = 120e-6
        resistance = 0.12
        inductance = 0.00018
        samples = []
        for omega in (80.0, 160.0, -80.0, -160.0):
            direction = 1.0 if omega > 0 else -1.0
            for i in range(512):
                phase_error = delay * omega + (0.15 if i % 2 else -0.15)
                emf = 1.2
                ed = -direction * emf * math.sin(phase_error)
                eq = direction * emf * math.cos(phase_error)
                s = sample(len(samples), int(direction), 0.0032, 0.0)
                samples.append(dataclasses.replace(
                    s, electrical_velocity=omega, id=0.7, iq=0.2,
                    vd_applied=ed + resistance * 0.7 - omega * inductance * 0.2,
                    vq_applied=eq + resistance * 0.2 + omega * inductance * 0.7))
        fit = fit_electrical_delay(
            samples, phase_resistance=resistance, phase_inductance=inductance)
        self.assertAlmostEqual(fit.electrical_delay, delay, places=8)
        self.assertLess(fit.residual_rms, 0.01)

    def test_asymmetric_friction_and_inertia_fit(self):
        kt = 0.035
        ratio = 42.0
        torque_scale = kt * ratio
        expected = (0.0022, 0.14, 0.17, 0.08, 0.11)
        observations = []
        for direction in (1, -1):
            for velocity_abs in (0.02, 0.05, 0.10):
                for acceleration in (-0.2, 0.0, 0.2):
                    velocity = direction * velocity_abs
                    inertia, cp, cn, bp, bn = expected
                    torque = inertia * acceleration
                    torque += (cp + bp * velocity if direction > 0 else
                               -cn + bn * velocity)
                    observations.append((velocity, acceleration,
                                         torque / torque_scale))
        fit = fit_mechanical_dynamics(
            observations, torque_constant=kt,
            motor_turns_per_output_turn=ratio)
        self.assertAlmostEqual(fit.output_inertia, expected[0], places=7)
        self.assertAlmostEqual(fit.coulomb_pos, expected[1], places=7)
        self.assertAlmostEqual(fit.coulomb_neg, expected[2], places=7)
        self.assertAlmostEqual(fit.viscous_pos, expected[3], places=7)
        self.assertAlmostEqual(fit.viscous_neg, expected[4], places=7)

    def test_integer_pole_pairs_from_electrical_scan(self):
        scan = 8.0 * 3.141592653589793
        counts = round(scan * 32768 / (2.0 * 3.141592653589793 * 14))
        self.assertEqual(identify_integer_pole_pairs(
            electrical_scan_radians=scan, encoder_delta_counts=counts,
            encoder_cpr=32768), 14)

    def test_noninteger_pole_pair_fit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'not close to an integer'):
            identify_integer_pole_pairs(
                electrical_scan_radians=8.0 * 3.141592653589793,
                encoder_delta_counts=9709, encoder_cpr=32768)

    def test_bidirectional_flux_fit_cancels_voltage_bias(self):
        expected_flux = 0.0032
        samples = [sample(i, 1, expected_flux, 0.03) for i in range(256)]
        samples += [sample(256 + i, -1, expected_flux, 0.03)
                    for i in range(256)]
        fit = fit_flux_linkage(
            samples, phase_resistance=0.12, phase_inductance=0.00018,
            pole_pairs=7)
        self.assertAlmostEqual(fit.flux_linkage, expected_flux, places=7)
        self.assertAlmostEqual(fit.torque_constant,
                               1.5 * 7 * expected_flux, places=7)
        self.assertEqual(fit.used_samples, 512)

    def test_flux_fit_requires_both_directions(self):
        samples = [sample(i, 1, 0.0032, 0.0) for i in range(256)]
        with self.assertRaisesRegex(ValueError, 'both flux scan directions'):
            fit_flux_linkage(
                samples, phase_resistance=0.12, phase_inductance=0.00018,
                pole_pairs=7)

    def test_flux_fit_block_average_rejects_switching_ripple(self):
        expected_flux = 0.0032
        samples = []
        for direction in (1, -1):
            for i in range(256):
                # Instantaneous ripple is deliberately larger than the old
                # 25% per-sample dispersion gate, but cancels in each block.
                ripple = (0.0015 if i % 2 == 0 else -0.0015)
                samples.append(sample(len(samples), direction,
                                      expected_flux + ripple, 0.0))
        fit = fit_flux_linkage(
            samples, phase_resistance=0.12, phase_inductance=0.00018,
            pole_pairs=7)
        self.assertAlmostEqual(fit.flux_linkage, expected_flux, places=7)
        self.assertEqual(fit.used_samples, 512)


if __name__ == '__main__':
    unittest.main()
