"""ctypes boundary for deterministic, hardware-free SafetySupervisor scenarios."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Iterator


ABI_VERSION = 1
_DLL_DIRECTORIES: list[Any] = []
_DLL_DIRECTORY_PATHS: set[str] = set()


class NativeScenarioUnavailable(RuntimeError):
    pass


class Command(ctypes.Structure):
    _fields_ = [
        ("request_id", ctypes.c_uint32),
        ("arg0", ctypes.c_uint32),
        ("source", ctypes.c_uint8),
        ("type", ctypes.c_uint8),
        ("operation", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]


class Event(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint32),
        ("epoch", ctypes.c_uint32),
        ("feedback_sequence", ctypes.c_uint32),
        ("control_sequence", ctypes.c_uint32),
        ("timestamp_cycles", ctypes.c_uint32),
        ("parent_fault_sequence", ctypes.c_uint32),
        ("arg0", ctypes.c_uint32),
        ("arg1", ctypes.c_uint32),
        ("arg2", ctypes.c_uint32),
        ("legacy_projection", ctypes.c_uint32),
        ("source", ctypes.c_uint16),
        ("code", ctypes.c_uint16),
        ("site", ctypes.c_uint16),
        ("type", ctypes.c_uint8),
        ("readiness_flags", ctypes.c_uint8),
        ("severity", ctypes.c_uint8),
        ("trace_recorded", ctypes.c_uint8),
    ]


class Result(ctypes.Structure):
    _fields_ = [
        ("request_id", ctypes.c_uint32),
        ("state_epoch", ctypes.c_uint32),
        ("reason", ctypes.c_uint16),
        ("source", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
    ]


class RealtimeRequest(ctypes.Structure):
    _fields_ = [
        ("request_id", ctypes.c_uint32),
        ("epoch", ctypes.c_uint32),
        ("type", ctypes.c_uint8),
        ("operation", ctypes.c_uint8),
        ("reserved", ctypes.c_uint16),
    ]


class State(ctypes.Structure):
    _fields_ = [
        ("state_epoch", ctypes.c_uint32),
        ("control_sequence", ctypes.c_uint32),
        ("readiness_sequence", ctypes.c_uint32),
        ("feedback_sequence", ctypes.c_uint32),
        ("dropped_commands", ctypes.c_uint32),
        ("dropped_results", ctypes.c_uint32),
        ("realtime_event_overflows", ctypes.c_uint32),
        ("fault_count", ctypes.c_uint32),
        ("state", ctypes.c_uint8),
        ("operation", ctypes.c_uint8),
        ("readiness_flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]


class Fault(ctypes.Structure):
    _fields_ = [
        ("fault_sequence", ctypes.c_uint32),
        ("control_sequence", ctypes.c_uint32),
        ("timestamp_cycles", ctypes.c_uint32),
        ("state_epoch", ctypes.c_uint32),
        ("parent_fault_sequence", ctypes.c_uint32),
        ("arg0", ctypes.c_uint32),
        ("arg1", ctypes.c_uint32),
        ("arg2", ctypes.c_uint32),
        ("occurrence_count", ctypes.c_uint32),
        ("source", ctypes.c_uint16),
        ("code", ctypes.c_uint16),
        ("site", ctypes.c_uint16),
        ("severity", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]


@dataclass(frozen=True)
class ReplaySample:
    control_sequence: int
    timestamp_cycles: int
    feedback_stale: bool
    values: dict[str, Any]


def iter_replay_samples(bundle: dict[str, Any]) -> Iterator[ReplaySample]:
    """Yield firmware-ordered scope samples without inventing missing data."""
    previous: int | None = None
    for event in bundle.get("capture", {}).get("events", []):
        if event.get("type") != "scope_data":
            continue
        for sample in event.get("samples", []):
            sequence = int(sample["control_sequence"]) & 0xFFFFFFFF
            if previous is not None:
                delta = (sequence - previous) & 0xFFFFFFFF
                if delta == 0 or delta >= 0x80000000:
                    raise ValueError("replay control sequences are not ordered")
            previous = sequence
            yield ReplaySample(
                control_sequence=sequence,
                timestamp_cycles=int(sample["timestamp_cycles"]) & 0xFFFFFFFF,
                feedback_stale=bool(sample.get("feedback_stale", False)),
                values=dict(sample.get("values", {})),
            )


class NativeScenario:
    def __init__(self, library_path: str | os.PathLike[str] | None = None):
        self._library = _load_library(library_path)
        _bind(self._library)
        if self._library.odrive_scenario_abi_version() != ABI_VERSION:
            raise NativeScenarioUnavailable("native scenario ABI version mismatch")
        self._handle = self._library.odrive_scenario_create()
        if not self._handle:
            raise NativeScenarioUnavailable("native scenario allocation failed")

    def close(self) -> None:
        if self._handle:
            self._library.odrive_scenario_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> "NativeScenario":
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close()
        return False

    def submit(self, *, request_id: int, source: int, command_type: int,
               operation: int = 0, arg0: int = 0) -> None:
        command = Command(request_id, arg0, source, command_type, operation, 0)
        if not self._library.odrive_scenario_submit(
                self._handle, ctypes.byref(command)):
            raise ValueError("native scenario rejected command ABI input or queue")

    def inject(self, event: Event) -> None:
        if not self._library.odrive_scenario_inject_event(
                self._handle, ctypes.byref(event)):
            raise ValueError("native scenario rejected event ABI input or queue")

    def advance(self, control_sequence: int) -> None:
        self._library.odrive_scenario_advance(
            self._handle, control_sequence & 0xFFFFFFFF)

    def pop_results(self) -> list[Result]:
        values = []
        while True:
            value = Result()
            if not self._library.odrive_scenario_pop_result(
                    self._handle, ctypes.byref(value)):
                return values
            values.append(value)

    def pop_realtime_requests(self) -> list[RealtimeRequest]:
        values = []
        while True:
            value = RealtimeRequest()
            if not self._library.odrive_scenario_pop_realtime_request(
                    self._handle, ctypes.byref(value)):
                return values
            values.append(value)

    def state(self) -> State:
        value = State()
        if not self._library.odrive_scenario_read_state(
                self._handle, ctypes.byref(value)):
            raise RuntimeError("native scenario state read failed")
        return value

    def first_fault(self) -> Fault | None:
        value = Fault()
        if not self._library.odrive_scenario_read_first_fault(
                self._handle, ctypes.byref(value)):
            return None
        return value


def _library_candidates() -> list[Path]:
    root = Path(__file__).resolve().parents[3]
    build = root / "Firmware" / "build" / "host-mingw-debug" / "Tests"
    names = ("odrive_scenario.dll", "libodrive_scenario.dll",
             "libodrive_scenario.so",
             "libodrive_scenario.dylib")
    configured = os.environ.get("ODRIVE_SCENARIO_LIBRARY")
    candidates = [Path(configured)] if configured else []
    return candidates + [build / name for name in names]


def _load_library(path: str | os.PathLike[str] | None) -> ctypes.CDLL:
    candidates = [Path(path)] if path is not None else _library_candidates()
    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            _add_windows_runtime_directory()
            return ctypes.CDLL(str(candidate))
        except OSError as exc:
            raise NativeScenarioUnavailable(
                f"cannot load native scenario library {candidate}: {exc}") from exc
    raise NativeScenarioUnavailable(
        "native scenario library not built; run cmake --build --preset host-debug")


def _add_windows_runtime_directory() -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return
    local = Path(os.environ.get("LOCALAPPDATA", ""))
    package_root = local / "Microsoft" / "WinGet" / "Packages"
    matches = sorted(package_root.glob(
        "BrechtSanders.WinLibs.MCF.UCRT_*/mingw64/bin"))
    if not matches:
        return
    path = str(matches[-1])
    if path in _DLL_DIRECTORY_PATHS:
        return
    _DLL_DIRECTORIES.append(os.add_dll_directory(path))
    _DLL_DIRECTORY_PATHS.add(path)


def _bind(library: ctypes.CDLL) -> None:
    library.odrive_scenario_abi_version.argtypes = []
    library.odrive_scenario_abi_version.restype = ctypes.c_uint32
    library.odrive_scenario_create.argtypes = []
    library.odrive_scenario_create.restype = ctypes.c_void_p
    library.odrive_scenario_destroy.argtypes = [ctypes.c_void_p]
    library.odrive_scenario_destroy.restype = None
    library.odrive_scenario_submit.argtypes = [ctypes.c_void_p,
                                               ctypes.POINTER(Command)]
    library.odrive_scenario_submit.restype = ctypes.c_int
    library.odrive_scenario_inject_event.argtypes = [ctypes.c_void_p,
                                                     ctypes.POINTER(Event)]
    library.odrive_scenario_inject_event.restype = ctypes.c_int
    library.odrive_scenario_advance.argtypes = [ctypes.c_void_p,
                                                ctypes.c_uint32]
    library.odrive_scenario_advance.restype = None
    library.odrive_scenario_pop_result.argtypes = [ctypes.c_void_p,
                                                   ctypes.POINTER(Result)]
    library.odrive_scenario_pop_result.restype = ctypes.c_int
    library.odrive_scenario_pop_realtime_request.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(RealtimeRequest)]
    library.odrive_scenario_pop_realtime_request.restype = ctypes.c_int
    library.odrive_scenario_read_state.argtypes = [ctypes.c_void_p,
                                                   ctypes.POINTER(State)]
    library.odrive_scenario_read_state.restype = ctypes.c_int
    library.odrive_scenario_read_first_fault.argtypes = [ctypes.c_void_p,
                                                         ctypes.POINTER(Fault)]
    library.odrive_scenario_read_first_fault.restype = ctypes.c_int
