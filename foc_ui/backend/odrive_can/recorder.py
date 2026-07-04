"""CAN CSV recorder for ODrive telemetry data.

Records the 14-channel synthesized telemetry dict to a CSV file.
Same start/stop/record pattern as ``foc_backend.recorder.Recorder``.

Columns:
  t_ms, pos, vel, iq_sp, iq_meas, vbus, ibus,
  axis_error, axis_state, ctrl_mode, input_mode,
  motor_err, enc_err, ctrl_err, traj_done
"""

from __future__ import annotations

import csv
import os
import time
from typing import Optional

CAN_COLUMNS = [
    't_ms', 'pos', 'vel', 'iq_sp', 'iq_meas', 'vbus', 'ibus',
    'axis_error', 'axis_state', 'ctrl_mode', 'input_mode',
    'motor_err', 'enc_err', 'ctrl_err', 'traj_done',
]

# Keys in the telemetry dict that are integers (no rounding)
_INT_KEYS = frozenset({
    'axis_error', 'axis_state', 'ctrl_mode', 'input_mode',
    'motor_err', 'enc_err', 'ctrl_err',
})


class CanRecorder:
    """Records CAN telemetry to a CSV file."""

    def __init__(self):
        self._file = None
        self._writer = None
        self._path: Optional[str] = None
        self._rows: int = 0

    @property
    def is_recording(self) -> bool:
        return self._file is not None

    @property
    def path(self) -> Optional[str]:
        return self._path

    @property
    def rows_written(self) -> int:
        return self._rows

    def start(self, path: str) -> None:
        """Open a new CSV file and write the header row."""
        if self._file is not None:
            raise RuntimeError('already recording')
        self._file = open(path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(CAN_COLUMNS)
        self._file.flush()
        self._path = os.path.abspath(path)
        self._rows = 0

    def stop(self) -> Optional[str]:
        """Close the CSV file and return the path."""
        f = self._file
        p = self._path
        self._file = None
        self._writer = None
        self._path = None
        if f is not None:
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        return p

    def record(self, telemetry: dict, ts_mono: float) -> None:
        """Write one telemetry dict as a CSV row."""
        if self._writer is None:
            return
        row = [int(ts_mono * 1000)]
        for key in CAN_COLUMNS[1:]:  # skip t_ms
            v = telemetry.get(key, 0)
            if key in _INT_KEYS:
                row.append(int(v))
            elif isinstance(v, float):
                row.append(round(v, 6))
            else:
                row.append(v)
        self._writer.writerow(row)
        self._rows += 1
        # Flush every 32 rows
        if self._rows % 32 == 0 and self._file is not None:
            try:
                self._file.flush()
            except Exception:
                pass
