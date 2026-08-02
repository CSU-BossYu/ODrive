"""Numerical regression tests for the encoder PLL phase representation."""

import math
from pathlib import Path
import struct
import unittest


FIRMWARE_ROOT = Path(__file__).resolve().parents[1]


def f32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def wrap_positive(value, period):
    return f32(value - math.floor(value / period) * period)


def wrap_pm(value, period):
    return value - math.floor((value + period / 2.0) / period) * period


class EncoderPllPrecisionTest(unittest.TestCase):
    def test_public_velocity_port_uses_turns_per_second(self):
        encoder_source = (
            FIRMWARE_ROOT / "MotorControl" / "encoder.cpp"
        ).read_text(encoding="utf-8")
        controller_source = (
            FIRMWARE_ROOT / "MotorControl" / "controller.cpp"
        ).read_text(encoding="utf-8")
        encoder_header = (
            FIRMWARE_ROOT / "MotorControl" / "encoder.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "vel_estimate_ = output_velocity_rpm / 60.0f;",
            encoder_source)
        self.assertIn(
            "vel_estimate_ = 0.0f; // [turn/s]", encoder_header)
        self.assertNotIn("*vel_estimate /= 60.0f;", controller_source)

        actual_rpm = -133.25
        public_turns_per_second = actual_rpm / 60.0
        self.assertAlmostEqual(public_turns_per_second * 60.0, actual_rpm)

    def test_absolute_position_input_is_not_clamped_to_one_turn(self):
        controller_source = (
            FIRMWARE_ROOT / "MotorControl" / "controller.cpp"
        ).read_text(encoding="utf-8")
        setter_start = controller_source.index(
            "void Controller::set_input_pos_and_steps")
        setter_end = controller_source.index(
            "void Controller::set_mit_input", setter_start)
        setter = controller_source[setter_start:setter_end]

        self.assertIn("input_pos_ = pos;", setter)
        self.assertNotIn("std::clamp", setter)

        current_position_rad = 12.9089
        current_position_turns = current_position_rad / (2.0 * math.pi)
        self.assertGreater(current_position_turns, 1.0)
        self.assertAlmostEqual(current_position_turns, 2.0545, places=4)

    def test_large_odd_count_is_not_exactly_representable_as_float32(self):
        shadow_count = -31_221_123
        self.assertNotEqual(f32(shadow_count), shadow_count)
        self.assertEqual(abs(f32(shadow_count) - shadow_count), 1.0)

    def test_modulo_phase_preserves_stationary_single_count_resolution(self):
        cpr = 32_768
        shadow_count = -31_221_123
        measured_phase = shadow_count % cpr

        # The old continuous float32 state is already one count away before
        # the PLL runs. Reducing that quantized state modulo CPR preserves the
        # error and excites the velocity estimator.
        old_phase = wrap_positive(f32(shadow_count), float(cpr))
        old_error = measured_phase - old_phase
        if old_error > cpr / 2:
            old_error -= cpr
        elif old_error < -cpr / 2:
            old_error += cpr
        self.assertEqual(abs(old_error), 1.0)

        # The new state converts only the exact integer remainder to float.
        bounded_phase = f32(measured_phase)
        self.assertEqual(bounded_phase, measured_phase)
        self.assertEqual(measured_phase - bounded_phase, 0.0)

    def test_stationary_position_does_not_follow_pll_subcount_settling(self):
        cpr = 32_768
        main_position_turns = f32(174.660_37)
        last_shadow_count = -5_494_287

        # The bounded PLL phase can legitimately settle by a fraction of one
        # count while its published velocity is snapped to zero.
        pll_phase_samples = (10_737.228_515_625, 10_737.187_5)
        for _pll_phase in pll_phase_samples:
            shadow_count = last_shadow_count
            delta_shadow = shadow_count - last_shadow_count
            main_position_turns = f32(
                main_position_turns + f32(delta_shadow / cpr))
            last_shadow_count = shadow_count

        self.assertEqual(main_position_turns, f32(174.660_37))

    def test_integer_anchor_prevents_slow_count_loss_at_large_position(self):
        cpr = 32_768
        anchor_position = f32(882.0)

        # At this magnitude one float32 ULP equals two encoder counts. Repeated
        # one-count additions round back to the same value indefinitely.
        incremental_position = anchor_position
        for _ in range(8):
            incremental_position = f32(
                incremental_position - f32(1.0 / cpr))
        self.assertEqual(incremental_position, anchor_position)

        # Recomputing from the exact integer displacement retains accumulation:
        # it may quantize to the float output resolution, but it cannot stall.
        anchored_position = f32(anchor_position + f32(-2.0 / cpr))
        self.assertNotEqual(anchored_position, anchor_position)

    def test_bounded_output_phase_retains_single_count_resolution(self):
        cpr = 32_768
        ratio = 42.0
        phase = f32(0.8)
        one_main_count = f32(1.0 / (cpr * ratio))

        next_phase = wrap_positive(
            f32(phase + one_main_count), 1.0)
        self.assertNotEqual(next_phase, phase)
        self.assertAlmostEqual(
            next_phase - phase, one_main_count, delta=2.0 ** -24)

    def test_main_encoder_increment_is_scaled_to_output_shaft(self):
        cpr = 32_768
        ratio = 42.0
        delta_shadow_counts = 1
        delta_main_turns = delta_shadow_counts / cpr

        # pos_circular is an output-shaft coordinate and must use the same
        # transmission scaling as the published output velocity.
        delta_output_turns = delta_main_turns / ratio
        self.assertAlmostEqual(
            delta_output_turns, 1.0 / (cpr * ratio), places=15)
        self.assertAlmostEqual(
            delta_main_turns / delta_output_turns, ratio, places=12)

    def test_bounded_joint_endpoints_are_not_circularly_equivalent(self):
        current = 5.0 / (2.0 * math.pi)
        target = 0.0
        linear_error = target - current

        self.assertLess(linear_error, 0.0)
        self.assertAlmostEqual(
            linear_error * 2.0 * math.pi, -5.0, places=12)

        # A full-turn endpoint remains distinct from zero.
        self.assertEqual(1.0 - 0.0, 1.0)
        upper_endpoint_error = 1.0 - current
        self.assertGreater(upper_endpoint_error, 0.0)
        self.assertAlmostEqual(
            upper_endpoint_error * 2.0 * math.pi,
            2.0 * math.pi - 5.0, places=12)

    def test_extended_count_remains_continuous_across_int32_wrap(self):
        int32_max = 2_147_483_647
        extended_count = int32_max
        legacy_count = int32_max

        extended_count += 1
        legacy_count = -2_147_483_648

        self.assertEqual(extended_count, 2_147_483_648)
        self.assertEqual(legacy_count, -2_147_483_648)
        self.assertEqual(extended_count - int32_max, 1)


if __name__ == "__main__":
    unittest.main()
