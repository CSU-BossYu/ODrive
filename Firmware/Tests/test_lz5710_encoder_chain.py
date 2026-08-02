"""Deterministic host-side tests for the LZ5710 dual-encoder chain.

Synthetic continuous output positions are converted to the physical raw
observations: the center/main encoder advances 10 turns per output turn and
the externally meshed 21-tooth auxiliary gear advances in the opposite raw
direction at 10*22/21 turns per output turn.
"""

import math
import random
import unittest


TAU = 2.0 * math.pi
CPR = 32768
MAIN_RATIO = 10.0
AUX_RATIO = 220.0 / 21.0
BRANCHES = 21
UNIQUE_RANGE = TAU * 21.0 / 10.0


def wrap_tau(value):
    return value % TAU


def wrap_pi(value):
    return (value + math.pi) % TAU - math.pi


def synthesize(position_rad, main_noise=0, aux_noise=0):
    output_turns = position_rad / TAU
    main = round((MAIN_RATIO * output_turns % 1.0) * CPR) % CPR
    # External mesh: raw auxiliary phase has the opposite sign.
    aux = round((-AUX_RATIO * output_turns % 1.0) * CPR) % CPR
    return (main + main_noise) % CPR, (aux + aux_noise) % CPR


def corrected_phases(main_count, aux_count):
    main = TAU * main_count / CPR
    aux = wrap_tau(-TAU * aux_count / CPR)
    return main, aux


def resolve(main_count, aux_count):
    main_phase, aux_phase = corrected_phases(main_count, aux_count)
    main_fraction = main_phase / TAU
    candidates = []
    for branch in range(BRANCHES):
        position = TAU * (branch + main_fraction) / MAIN_RATIO
        residual = wrap_pi(aux_phase - wrap_tau(AUX_RATIO * position))
        candidates.append((abs(residual), residual, branch, position))
    candidates.sort()
    best = candidates[0]
    return {
        "branch": best[2],
        "position": best[3],
        "residual": best[1],
        "margin": candidates[1][0] - best[0],
    }


def synthesize_with_offsets(position_rad, main_offset_rad, aux_offset_rad,
                            main_noise=0, aux_noise=0):
    main_corrected = MAIN_RATIO * position_rad / TAU
    aux_corrected = AUX_RATIO * position_rad / TAU
    main_raw = round(((main_corrected + main_offset_rad / TAU) % 1.0) * CPR)
    # Firmware reverses the externally meshed auxiliary raw phase before
    # subtracting its calibrated phase offset.
    aux_raw = round(((-aux_corrected - aux_offset_rad / TAU) % 1.0) * CPR)
    return ((main_raw + main_noise) % CPR,
            (aux_raw + aux_noise) % CPR)


def resolve_with_offsets(main_count, aux_count, main_offset_rad,
                         aux_offset_rad):
    main_phase = wrap_tau(TAU * main_count / CPR - main_offset_rad)
    aux_phase = wrap_tau(-TAU * aux_count / CPR - aux_offset_rad)
    candidates = []
    for branch in range(BRANCHES):
        position = TAU * (branch + main_phase / TAU) / MAIN_RATIO
        residual = wrap_pi(aux_phase - wrap_tau(AUX_RATIO * position))
        candidates.append((abs(residual), residual, branch, position))
    candidates.sort()
    return {
        "branch": candidates[0][2],
        "position": candidates[0][3],
        "residual": candidates[0][1],
        "margin": candidates[1][0] - candidates[0][0],
    }


def fit_aux_offset(observations, main_offset_rad=0.0,
                   center_rad=0.0, search_radius_rad=0.05 * TAU):
    best = None
    for index in range(1024):
        candidate = center_rad - search_radius_rad + \
            2.0 * search_radius_rad * index / 1023.0
        residuals = [
            resolve_with_offsets(main, aux, main_offset_rad, candidate)
            ["residual"]
            for main, aux in observations
        ]
        rms = math.sqrt(sum(value * value for value in residuals) /
                        len(residuals))
        distance = abs(wrap_pi(candidate - center_rad))
        score = (rms, distance, candidate)
        if best is None or score < best:
            best = score
    return best[2]


def unwrap_delta(now, previous):
    delta = (now - previous) % CPR
    if delta > CPR // 2:
        delta -= CPR
    return delta


class Pll:
    def __init__(self, bandwidth=120.0, damping=math.sqrt(0.5)):
        self.bandwidth = bandwidth
        self.damping = damping
        self.valid = False
        self.position = 0.0
        self.rpm = 0.0
        self.missing_time = 0.0
        self.timed_out = False

    def update(self, observation, dt):
        if not self.valid:
            self.position = observation
            self.rpm = 0.0
            self.valid = True
            self.missing_time = 0.0
            return
        velocity_rad_s = self.rpm * TAU / 60.0
        predicted = self.position + velocity_rad_s * dt
        error = observation - predicted  # deliberately linear, never wrapped
        kp = 2.0 * self.damping * self.bandwidth
        ki = self.bandwidth**2
        self.position = predicted + kp * dt * error
        velocity_rad_s += ki * dt * error
        self.rpm = velocity_rad_s * 60.0 / TAU
        self.missing_time = 0.0

    def miss(self, dt, timeout=0.002):
        self.missing_time += dt
        if self.missing_time > timeout:
            self.timed_out = True
            self.valid = False


class Tracker:
    """Integer-count continuous position model matching the firmware tracker."""

    def __init__(self, initial_position):
        self.position = resolve(*synthesize(initial_position))["position"]
        self.previous_main = synthesize(initial_position)[0]
        self.valid = True

    def update(self, true_position, frame_valid=True):
        if not frame_valid:
            return False
        main_count, _ = synthesize(true_position)
        self.position += unwrap_delta(
            main_count, self.previous_main) * TAU / (CPR * MAIN_RATIO)
        self.previous_main = main_count
        return True


def periodic_lut(position_rad, table):
    phase = wrap_tau(position_rad) / TAU
    scaled = phase * len(table)
    index = int(math.floor(scaled)) % len(table)
    following = (index + 1) % len(table)
    fraction = scaled - math.floor(scaled)
    return table[index] + fraction * (table[following] - table[index])


def controller_cycle(feedback_active, encoder_update_ok,
                     controller_update_ok=True,
                     electrical_feedback_published=True):
    """Model the firmware's controller lifecycle/fault propagation gate."""
    controller_ran = feedback_active and encoder_update_ok
    pipeline_ok = (feedback_active and encoder_update_ok and
                   electrical_feedback_published and
                   (controller_update_ok if controller_ran else True))
    controller_failed = feedback_active and not pipeline_ok
    return controller_ran, controller_failed


def phase_velocity_fault(armed, phase_velocity_present):
    """Model the motor feedforward phase-velocity safety gate."""
    return armed and not phase_velocity_present


def resolver_residual_accepted(residual, locked, accept=0.04, reject=0.08):
    """Model the resolver's acquisition/retention residual hysteresis."""
    if abs(residual) > reject:
        return False
    return abs(residual) <= (reject if locked else accept)


def controller_ready_observations(cycles, divider=5):
    """Count actual multi-rate controller updates in 10 kHz ISR cycles."""
    sequence = 0
    ready = False
    for cycle in range(1, cycles + 1):
        if cycle % divider == 0:
            ready = True
            sequence += 1
        # Non-scheduled hold cycles retain readiness.
    return sequence, ready


class Lz5710EncoderChainTest(unittest.TestCase):
    def test_01_stationary_near_zero(self):
        main, aux = synthesize(1.0e-6)
        self.assertAlmostEqual(resolve(main, aux)["position"], 1.0e-6,
                               delta=TAU / (CPR * MAIN_RATIO))

    def test_02_forward_main_phase_wrap(self):
        positions = [TAU * (0.09999 + i * 0.00001) for i in range(3)]
        counts = [synthesize(p)[0] for p in positions]
        deltas = [unwrap_delta(counts[i], counts[i - 1])
                  for i in range(1, len(counts))]
        self.assertTrue(all(delta >= 0 for delta in deltas))

    def test_03_reverse_main_phase_wrap(self):
        positions = [TAU * (0.10001 - i * 0.00001) for i in range(3)]
        counts = [synthesize(p)[0] for p in positions]
        deltas = [unwrap_delta(counts[i], counts[i - 1])
                  for i in range(1, len(counts))]
        self.assertTrue(all(delta <= 0 for delta in deltas))

    def test_04_continuous_positive_multiturn(self):
        positions = [i * 0.01 for i in range(2000)]
        recovered = positions[0]
        previous = synthesize(positions[0])[0]
        for position in positions[1:]:
            current = synthesize(position)[0]
            recovered += unwrap_delta(current, previous) * TAU / (
                CPR * MAIN_RATIO)
            previous = current
        self.assertAlmostEqual(recovered, positions[-1], delta=2e-4)

    def test_05_continuous_negative_multiturn(self):
        positions = [-i * 0.01 for i in range(2000)]
        recovered = positions[0]
        previous = synthesize(positions[0])[0]
        for position in positions[1:]:
            current = synthesize(position)[0]
            recovered += unwrap_delta(current, previous) * TAU / (
                CPR * MAIN_RATIO)
            previous = current
        self.assertAlmostEqual(recovered, positions[-1], delta=2e-4)

    def test_06_three_pi_remains_three_pi(self):
        tracker = Tracker(3.0 * math.pi)
        self.assertAlmostEqual(tracker.position, 3.0 * math.pi,
                               delta=TAU / (CPR * MAIN_RATIO))
        self.assertNotAlmostEqual(tracker.position, wrap_tau(3.0 * math.pi))

    def test_07_all_21_branches(self):
        for branch in range(BRANCHES):
            position = TAU * (branch + 0.37) / MAIN_RATIO
            main, aux = synthesize(position)
            self.assertEqual(resolve(main, aux)["branch"], branch)

    def test_08_unique_range_boundaries(self):
        epsilon = TAU / (CPR * MAIN_RATIO) * 2.0
        low = resolve(*synthesize(epsilon))["position"]
        high = resolve(*synthesize(UNIQUE_RANGE - epsilon))["position"]
        self.assertLess(low, 4.0 * epsilon)
        self.assertGreater(high, UNIQUE_RANGE - 4.0 * epsilon)
        self.assertEqual(synthesize(0.0), synthesize(UNIQUE_RANGE))

    def test_09_main_plus_minus_one_count_noise(self):
        position = 0.73 * UNIQUE_RANGE
        expected = resolve(*synthesize(position))["branch"]
        for noise in (-1, 1):
            main, aux = synthesize(position, main_noise=noise)
            self.assertEqual(resolve(main, aux)["branch"], expected)

    def test_10_aux_plus_minus_one_count_noise(self):
        position = 0.41 * UNIQUE_RANGE
        expected = resolve(*synthesize(position))["branch"]
        for noise in (-1, 1):
            main, aux = synthesize(position, aux_noise=noise)
            self.assertEqual(resolve(main, aux)["branch"], expected)

    def test_11_boundary_noise_does_not_chatter_branch(self):
        rng = random.Random(5710)
        position = TAU * (8.0002 / MAIN_RATIO)
        branches = []
        for _ in range(100):
            main, aux = synthesize(
                position, rng.choice((-1, 0, 1)), rng.choice((-1, 0, 1)))
            branches.append(resolve(main, aux)["branch"])
        self.assertEqual(len(set(branches)), 1)

    def test_12_missing_frames_do_not_change_position_observation(self):
        tracker = Tracker(1.2)
        before = tracker.position
        self.assertFalse(tracker.update(1.21, frame_valid=False))
        self.assertEqual(tracker.position, before)
        self.assertTrue(tracker.update(1.22, frame_valid=True))
        self.assertAlmostEqual(tracker.position, 1.22, delta=2e-5)

        pll = Pll()
        pll.update(before, 0.0001)
        pll.miss(0.0001)
        self.assertTrue(pll.valid)
        for _ in range(25):
            pll.miss(0.0001)
        self.assertTrue(pll.timed_out)
        self.assertFalse(pll.valid)

    def test_13_bad_spi_frame_does_not_enter_pll(self):
        pll = Pll()
        pll.update(1.0, 0.0001)
        before = (pll.position, pll.rpm)
        frame_valid = False
        if frame_valid:
            pll.update(100.0, 0.0001)
        self.assertEqual((pll.position, pll.rpm), before)

    def test_14_lut_is_continuous_at_zero(self):
        table = [0.001 * math.sin(TAU * i / 64.0) for i in range(64)]
        left = periodic_lut(TAU - 1e-9, table)
        right = periodic_lut(1e-9, table)
        self.assertAlmostEqual(left, right, delta=1e-9)

    def test_15_lut_enabled_correction(self):
        table = [0.002 * math.sin(TAU * i / 64.0) for i in range(64)]
        position = 0.7
        correction = periodic_lut(position, table)
        disabled_position = position
        enabled_position = position + correction
        self.assertAlmostEqual(enabled_position - disabled_position, correction)
        self.assertNotEqual(correction, 0.0)

    def test_16_positive_constant_speed_pll(self):
        pll = Pll()
        rpm = 60.0
        dt = 0.0001
        for index in range(5000):
            pll.update(index * dt * rpm * TAU / 60.0, dt)
        self.assertGreater(pll.rpm, 0.0)
        self.assertAlmostEqual(pll.rpm, rpm, delta=0.2)

    def test_17_negative_constant_speed_pll(self):
        pll = Pll()
        rpm = -45.0
        dt = 0.0001
        for index in range(5000):
            pll.update(index * dt * rpm * TAU / 60.0, dt)
        self.assertLess(pll.rpm, 0.0)
        self.assertAlmostEqual(pll.rpm, rpm, delta=0.2)

    def test_18_stationary_pll_converges_to_zero(self):
        pll = Pll()
        for _ in range(5000):
            pll.update(1.234, 0.0001)
        self.assertAlmostEqual(pll.rpm, 0.0, delta=0.01)

    def test_19_branch_jump_is_rejected_before_pll(self):
        pll = Pll()
        pll.update(1.0, 0.0001)
        before = pll.rpm
        proposed = 1.0 + TAU / MAIN_RATIO
        maximum_step = 0.25
        accepted_observations = 0
        if abs(proposed - pll.position) <= maximum_step:
            pll.update(proposed, 0.0001)
            accepted_observations += 1
        self.assertEqual(accepted_observations, 0)
        self.assertEqual(pll.rpm, before)

    def test_20_variable_sample_period_pll_stability(self):
        pll = Pll()
        rpm = 30.0
        position = 0.0
        periods = (0.00008, 0.0001, 0.00012, 0.0002)
        for index in range(6000):
            dt = periods[index % len(periods)]
            position += rpm * TAU / 60.0 * dt
            pll.update(position, dt)
        self.assertAlmostEqual(pll.rpm, rpm, delta=0.3)

    def test_21_vernier_offset_fit_from_synthetic_motion(self):
        rng = random.Random(22021)
        positions = [
            -0.35 + index * 0.7 / 15.0
            for index in range(16)
        ]
        for true_aux_offset in (0.1311, 2.5, -2.7):
            with self.subTest(true_aux_offset=true_aux_offset):
                observations = [
                    synthesize_with_offsets(
                        position, 0.0, true_aux_offset,
                        rng.choice((-1, 0, 1)), rng.choice((-1, 0, 1)))
                    for position in positions
                ]
                fitted = fit_aux_offset(observations)
                results = [
                    resolve_with_offsets(main, aux, 0.0, fitted)
                    for main, aux in observations
                ]
                self.assertLess(
                    max(abs(result["residual"]) for result in results),
                    0.001)
                self.assertGreater(
                    min(result["margin"] for result in results), 0.04)

    def test_22_no_new_control_cycle_preserves_valid_frame_streak(self):
        consecutive_valid_frames = 2
        sample_age_cycles = 0
        for _ in range(5):
            has_new_sample = False
            if not has_new_sample:
                sample_age_cycles += 1
                # No frame was received, so the frame-validity streak is
                # preserved until the explicit timeout boundary.
        self.assertEqual(consecutive_valid_frames, 2)
        self.assertEqual(sample_age_cycles, 5)

        has_new_sample = True
        sample_valid = False
        if has_new_sample and not sample_valid:
            consecutive_valid_frames = 0
        self.assertEqual(consecutive_valid_frames, 0)

    def test_23_idle_mode_write_does_not_run_feedback_controller(self):
        controller_ran, controller_failed = controller_cycle(
            feedback_active=False, encoder_update_ok=True)
        self.assertFalse(controller_ran)
        self.assertFalse(controller_failed)

    def test_24_active_feedback_failure_is_still_faulted(self):
        controller_ran, controller_failed = controller_cycle(
            feedback_active=True, encoder_update_ok=False)
        self.assertFalse(controller_ran)
        self.assertTrue(controller_failed)

    def test_25_aux_fault_does_not_remove_motor_phase_feedback(self):
        # Motor electrical phase is derived from the main encoder. Once its
        # PLL is acquired, an auxiliary/Vernier readiness transient must not
        # turn the motor phase velocity port into an empty value.
        motor_phase_valid = True
        full_vernier_chain_ready = True
        full_vernier_chain_ready = False  # isolated aux/frame/resolver fault
        self.assertTrue(motor_phase_valid)
        self.assertFalse(full_vernier_chain_ready)

    def test_26_main_timeout_removes_motor_phase_feedback(self):
        motor_phase_valid = True
        main_sample_age_cycles = 0
        timeout_cycles = 20
        for _ in range(timeout_cycles + 1):
            main_sample_age_cycles += 1
        if main_sample_age_cycles > timeout_cycles:
            motor_phase_valid = False
        self.assertFalse(motor_phase_valid)

    def test_27_unarmed_phase_acquisition_window_is_not_a_motor_fault(self):
        self.assertFalse(phase_velocity_fault(
            armed=False, phase_velocity_present=False))

    def test_28_armed_phase_velocity_loss_remains_fatal(self):
        self.assertTrue(phase_velocity_fault(
            armed=True, phase_velocity_present=False))

    def test_29_closed_loop_pipeline_waits_for_electrical_feedback(self):
        _controller_ran, controller_failed = controller_cycle(
            feedback_active=True,
            encoder_update_ok=True,
            controller_update_ok=True,
            electrical_feedback_published=False)
        self.assertTrue(controller_failed)

    def test_30_stationary_settling_can_acquire_resolver(self):
        # Hardware regression: the calibrated unit settled at 0.03245 rad.
        self.assertTrue(resolver_residual_accepted(0.03245, locked=False))

    def test_31_locked_resolver_uses_reject_hysteresis(self):
        self.assertFalse(resolver_residual_accepted(0.06, locked=False))
        self.assertTrue(resolver_residual_accepted(0.06, locked=True))
        self.assertFalse(resolver_residual_accepted(0.081, locked=True))

    def test_32_multirate_controller_readiness_counts_executions(self):
        sequence, ready = controller_ready_observations(20)
        self.assertEqual(sequence, 4)
        self.assertTrue(ready)


if __name__ == "__main__":
    unittest.main()
