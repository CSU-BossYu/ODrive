"""CSV recorder for the telemetry stream.

Writes one row per received (CRC-valid) frame. The CSV header is fixed:

    t_ms, seq, pos, vel_ref, vel_fb, iq_ref, iq_meas, id_meas,
    vq, vd, vbus, phase, vel_err, integrator, vq_ff, vd_ff,
    errors_lo16, state_flags

Both the engineering float values and the raw int16 fields are useful for
post-analysis (e.g. comparing actual currents against the quantized
representation), so the recorder stores the engineering form for the analog
channels and the raw integer for the two bitfield channels.

Recording is controlled by start()/stop(). The file is opened in append mode
and flushed on each frame so a crash leaves a usable file.
"""

from __future__ import annotations

import csv
import os
import time
from typing import Optional

from .telemetry import CHANNELS, TelemetryFrame


# Column order: timestamp + seq + 16 channels (matching CHANNELS order).
COLUMNS = ['t_ms', 'seq'] + [c.key for c in CHANNELS]


class Recorder:
    def __init__(self):
        self._fh = None
        self._writer: Optional[csv.writer] = None
        self._path: Optional[str] = None
        self._start_mono: float = 0.0
        self._rows = 0

    @property
    def is_recording(self) -> bool:
        return self._writer is not None

    @property
    def path(self) -> Optional[str]:
        return self._path

    @property
    def rows_written(self) -> int:
        return self._rows

    def start(self, path: str) -> None:
        if self._writer is not None:
            raise RuntimeError('recorder already running')
        # Append .csv if missing.
        if not path.lower().endswith('.csv'):
            path = path + '.csv'
        # Make sure the directory exists.
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        write_header = not os.path.exists(path) or os.path.getsize(path) == 0
        self._fh = open(path, 'a', newline='', encoding='ascii')
        self._writer = csv.writer(self._fh)
        if write_header:
            self._writer.writerow(COLUMNS)
        self._path = path
        self._start_mono = time.monotonic()
        self._rows = 0

    def stop(self) -> Optional[str]:
        if self._writer is None:
            return None
        try:
            self._fh.flush()
            self._fh.close()
        finally:
            self._fh = None
            self._writer = None
        p = self._path
        self._path = None
        return p

    def record(self, frame: TelemetryFrame, ts_mono: float) -> None:
        """Append one frame. Called on the event-loop thread."""
        if self._writer is None:
            return
        t_ms = int((ts_mono - self._start_mono) * 1000)
        row = [t_ms, frame.seq]
        for ch in CHANNELS:
            v = frame.values.get(ch.key, 0)
            # Floats: round to 6 sig digits to keep the CSV compact.
            if isinstance(v, float):
                row.append(round(v, 6))
            else:
                row.append(v)
        self._writer.writerow(row)
        self._rows += 1
        # Flush every 32 rows to bound data loss on crash without killing I/O.
        if (self._rows & 0x1F) == 0:
            self._fh.flush()
