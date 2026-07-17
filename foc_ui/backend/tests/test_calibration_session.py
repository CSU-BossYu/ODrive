import asyncio
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.protocol import (
    CalibrationSessionItem,
    CalibrationProfile,
    CalibrationSessionState,
    ExtStatus,
    ExtSubCmd,
    ExtType,
    encode_extended_request,
)
from odrive_can.service import ODriveService


class ReadHarness:
    read_calibration_session = ODriveService.read_calibration_session
    read_calibration_candidate = ODriveService.read_calibration_candidate
    read_calibration_snapshot = ODriveService.read_calibration_snapshot
    start_calibration = ODriveService.start_calibration

    async def ext_command(self, sub_cmd, item, ext_type=0, value=0,
                          timeout=1.0):
        values = {
            CalibrationSessionItem.SCHEMA_VERSION: 1,
            CalibrationSessionItem.SESSION_ID: 42,
            CalibrationSessionItem.STATE:
                int(CalibrationSessionState.VALIDATED),
            CalibrationSessionItem.STAGE: 3,
            CalibrationSessionItem.FAILURE_CODE: 0,
            CalibrationSessionItem.FLAGS: 0x0E,
            CalibrationSessionItem.TRANSITION_COUNT: 6,
            CalibrationSessionItem.REQUEST_OPTIONS: 0,
            CalibrationSessionItem.PROGRESS_PERMILLE: 750,
            CalibrationSessionItem.BUFFERED_RECORDS: 7,
            CalibrationSessionItem.DROPPED_RECORDS: 2,
            CalibrationSessionItem.BUFFER_HIGH_WATERMARK: 31,
            CalibrationSessionItem.BUFFER_CAPACITY: 32,
            CalibrationSessionItem.TRANSPORT_FRAMES_SENT: 1024,
            CalibrationSessionItem.TRANSPORT_QUEUE_RETRIES: 3,
            CalibrationSessionItem.TRANSPORT_DISCONNECT_WAITS: 0,
            CalibrationSessionItem.RESULT_VALIDITY: 0x03,
            CalibrationSessionItem.PHASE_RESISTANCE: 0.125,
            CalibrationSessionItem.PHASE_INDUCTANCE: 0.0002,
            CalibrationSessionItem.ENCODER_DIRECTION: -1,
            CalibrationSessionItem.PHASE_OFFSET: 1234,
            CalibrationSessionItem.PHASE_OFFSET_FLOAT: 0.25,
            CalibrationSessionItem.EFFECTIVE_RATIO_SCALE: 0.9994,
            CalibrationSessionItem.GEOMETRY_RAW_RMS: 0.0012,
            CalibrationSessionItem.GEOMETRY_CORRECTED_RMS: 0.00018,
            CalibrationSessionItem.GEOMETRY_DIRECTION_PEAK_TO_PEAK: 0.0007,
            CalibrationSessionItem.GEOMETRY_USED_SAMPLES: 32000,
            CalibrationSessionItem.FLUX_LINKAGE: 0.0031,
            CalibrationSessionItem.TORQUE_CONSTANT: 0.03255,
            CalibrationSessionItem.FLUX_SAMPLE_STDDEV: 0.00008,
            CalibrationSessionItem.FLUX_USED_SAMPLES: 24000,
            CalibrationSessionItem.POLE_PAIRS: 14,
            CalibrationSessionItem.OUTPUT_INERTIA: 0.0021,
            CalibrationSessionItem.FRICTION_COULOMB_POS: 0.14,
            CalibrationSessionItem.FRICTION_COULOMB_NEG: 0.16,
            CalibrationSessionItem.FRICTION_VISCOUS_POS: 0.08,
            CalibrationSessionItem.FRICTION_VISCOUS_NEG: 0.09,
            CalibrationSessionItem.MECHANICAL_RESIDUAL_RMS_TORQUE: 0.012,
            CalibrationSessionItem.MECHANICAL_USED_SAMPLES: 18000,
            CalibrationSessionItem.ELECTRICAL_DELAY: 38e-6,
            CalibrationSessionItem.DELAY_RESIDUAL_PHASE_OFFSET: 0.006,
            CalibrationSessionItem.DELAY_RESIDUAL_RMS: 0.012,
            CalibrationSessionItem.DELAY_USED_SAMPLES: 5800,
            CalibrationSessionItem.MECHANICAL_ATTEMPTED_SAMPLES: 24000,
            CalibrationSessionItem.MECHANICAL_REJECTED_INVALID: 0,
            CalibrationSessionItem.MECHANICAL_REJECTED_SATURATED: 12,
            CalibrationSessionItem.MECHANICAL_REJECTED_LOW_VELOCITY: 5988,
            CalibrationSessionItem.MECHANICAL_MAX_ABS_VELOCITY: 0.101,
            CalibrationSessionItem.MECHANICAL_REJECTED_TIMING: 0,
            CalibrationSessionItem.DELAY_ATTEMPTED_SAMPLES: 9000,
            CalibrationSessionItem.DELAY_REJECTED_INVALID: 0,
            CalibrationSessionItem.DELAY_REJECTED_SATURATED: 0,
            CalibrationSessionItem.DELAY_REJECTED_SPEED: 0,
            CalibrationSessionItem.DELAY_REJECTED_EMF: 10,
            CalibrationSessionItem.DELAY_REJECTED_PHASE: 5,
            CalibrationSessionItem.DELAY_MAX_ABS_ELECTRICAL_SPEED: 443.0,
        }
        return {'status': ExtStatus.OK, 'value': values[item]}


class CollectingHarness:
    read_calibration_snapshot = ODriveService.read_calibration_snapshot

    async def read_calibration_session(self):
        return {'ok': True, 'state': int(CalibrationSessionState.COLLECTING)}

    async def read_calibration_candidate(self):
        raise AssertionError('candidate must not be read while collecting')


class CalibrationSessionTests(unittest.TestCase):
    def test_start_wire_encoding(self):
        data = encode_extended_request(
            ExtSubCmd.CALIBRATION_SESSION,
            CalibrationSessionItem.START,
            ExtType.UINT32,
            int(CalibrationProfile.ELECTRICAL),
        )
        self.assertEqual(data[:3], bytes((0x0F, 0x10, 0x03)))
        self.assertEqual(struct.unpack_from('<I', data, 4)[0], 1)

    def test_read_session(self):
        result = asyncio.run(ReadHarness().read_calibration_session())
        self.assertTrue(result['ok'])
        self.assertEqual(result['session_id'], 42)
        self.assertEqual(result['state_name'], 'VALIDATED')
        self.assertEqual(result['stage'], 3)
        self.assertEqual(result['progress_permille'], 750)
        self.assertEqual(result['buffered_records'], 7)
        self.assertEqual(result['dropped_records'], 2)
        self.assertEqual(result['buffer_capacity'], 32)
        self.assertEqual(result['transport_frames_sent'], 1024)

    def test_read_candidate_is_explicitly_pending(self):
        result = asyncio.run(ReadHarness().read_calibration_candidate())
        self.assertTrue(result['ok'])
        self.assertEqual(result['validity'], 0x03)
        self.assertAlmostEqual(result['phase_resistance'], 0.125)
        self.assertEqual(result['encoder_direction'], -1)
        self.assertAlmostEqual(result['effective_ratio_scale'], 0.9994)
        self.assertLess(result['geometry_corrected_rms'],
                        result['geometry_raw_rms'])
        self.assertEqual(result['geometry_used_samples'], 32000)
        self.assertAlmostEqual(result['flux_linkage'], 0.0031)
        self.assertAlmostEqual(result['torque_constant'], 0.03255)
        self.assertEqual(result['flux_used_samples'], 24000)
        self.assertEqual(result['pole_pairs'], 14)
        self.assertAlmostEqual(result['output_inertia'], 0.0021)
        self.assertAlmostEqual(result['friction_coulomb_neg'], 0.16)
        self.assertEqual(result['mechanical_used_samples'], 18000)
        self.assertAlmostEqual(result['electrical_delay'], 38e-6)
        self.assertAlmostEqual(result['delay_residual_rms'], 0.012)
        self.assertEqual(result['delay_used_samples'], 5800)
        self.assertEqual(result['mechanical_attempted_samples'], 24000)
        self.assertAlmostEqual(result['mechanical_max_abs_velocity'], 0.101)

    def test_snapshot_includes_candidate_after_identification(self):
        result = asyncio.run(ReadHarness().read_calibration_snapshot())
        self.assertTrue(result['ok'])
        self.assertEqual(result['session']['state_name'], 'VALIDATED')
        self.assertAlmostEqual(result['candidate']['electrical_delay'], 38e-6)

    def test_snapshot_skips_candidate_during_collection(self):
        result = asyncio.run(CollectingHarness().read_calibration_snapshot())
        self.assertTrue(result['ok'])
        self.assertIsNone(result['candidate'])

    def test_geometry_turn_limit_is_checked_before_can(self):
        with self.assertRaisesRegex(ValueError, 'geometry_turns'):
            asyncio.run(ReadHarness().start_calibration(geometry_turns=9))


if __name__ == '__main__':
    unittest.main()
