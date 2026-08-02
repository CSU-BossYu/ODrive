"""Compatibility import for the canonical production replay implementation."""

from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from foc_ui.backend.odrive_usb.replay import *  # noqa: F401,F403,E402
