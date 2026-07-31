import importlib.util
import io
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path


SCRIPT = (
    Path(__file__).parent /
    "hex_4342_mt6826s" /
    "run_dual_encoder_motion_quality.py"
)
SPEC = importlib.util.spec_from_file_location("dual_encoder_motion_quality", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class DualEncoderMotionQualityTest(unittest.TestCase):
    def test_counter_delta_handles_uint32_wrap(self):
        self.assertEqual(MODULE.counter_delta(3, 0xFFFFFFFE), 5)

    def test_fault_mask_names_remain_independent(self):
        decoded = MODULE.decode_mask(
            (1 << 0) | (1 << 6), MODULE.CHAIN_FAULT_NAMES)
        self.assertEqual(decoded, "MAIN_COMMUNICATION,VERNIER_RESIDUAL")

    def test_quality_rejects_main_sample_timeout_window(self):
        stats = MODULE.QualityStats()
        stats.add(MODULE.Sample(
            elapsed_s=0.1,
            stage="forward",
            command_rpm=3.0,
            readiness_flags=0xFF,
            main_sample_age_max_cycles=21,
        ))
        with redirect_stdout(io.StringIO()):
            passed = MODULE.print_quality_report(stats, frame_clean=True)
        self.assertFalse(passed)


if __name__ == "__main__":
    unittest.main()
