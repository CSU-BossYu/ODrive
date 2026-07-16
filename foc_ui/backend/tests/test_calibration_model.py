import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.calibration_model import PARAMETERS, stale_closure


class CalibrationModelTests(unittest.TestCase):
    def test_every_committed_parameter_has_runtime_consumer(self):
        self.assertTrue(PARAMETERS)
        for parameter in PARAMETERS.values():
            self.assertTrue(parameter.consumers, parameter.key)

    def test_resistance_change_invalidates_flux_and_inertia(self):
        stale = stale_closure({'phase_resistance'})
        self.assertIn('flux_linkage', stale)
        self.assertIn('equivalent_output_inertia', stale)
        self.assertNotIn('friction_current_model', stale)

    def test_relative_angle_change_invalidates_mechanical_models(self):
        stale = stale_closure({'relative_angle_model'})
        self.assertIn('friction_current_model', stale)
        self.assertIn('equivalent_output_inertia', stale)
        self.assertIn('electrical_offset', stale)

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(KeyError):
            stale_closure({'gear_efficiency'})


if __name__ == '__main__':
    unittest.main()
