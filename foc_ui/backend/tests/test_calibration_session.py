import asyncio
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from odrive_can.protocol import (
    CalibrationSessionItem,
    CalibrationSessionState,
    ExtStatus,
    ExtSubCmd,
    ExtType,
    encode_extended_request,
)
from odrive_can.service import ODriveService


class ReadHarness:
    read_calibration_session = ODriveService.read_calibration_session

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
        }
        return {'status': ExtStatus.OK, 'value': values[item]}


class CalibrationSessionTests(unittest.TestCase):
    def test_begin_wire_encoding(self):
        data = encode_extended_request(
            ExtSubCmd.CALIBRATION_SESSION,
            CalibrationSessionItem.BEGIN,
            ExtType.UINT32,
            0x12345678,
        )
        self.assertEqual(data[:3], bytes((0x0F, 0x10, 0x03)))
        self.assertEqual(struct.unpack_from('<I', data, 4)[0], 0x12345678)

    def test_read_session(self):
        result = asyncio.run(ReadHarness().read_calibration_session())
        self.assertTrue(result['ok'])
        self.assertEqual(result['session_id'], 42)
        self.assertEqual(result['state_name'], 'VALIDATED')
        self.assertEqual(result['stage'], 3)


if __name__ == '__main__':
    unittest.main()
