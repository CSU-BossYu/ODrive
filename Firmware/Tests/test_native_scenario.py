import ctypes
from pathlib import Path

import pytest

from odrive_hil.native_scenario import (
    Command, Event, Fault, NativeScenario, NativeScenarioUnavailable,
    RealtimeRequest, Result, State, iter_replay_samples,
)


def test_ctypes_layout_matches_versioned_c_abi():
    assert ctypes.sizeof(Command) == 12
    assert ctypes.sizeof(Event) == 52
    assert ctypes.sizeof(Result) == 12
    assert ctypes.sizeof(RealtimeRequest) == 12
    assert ctypes.sizeof(State) == 36
    assert ctypes.sizeof(Fault) == 44


def test_replay_samples_preserve_gaps_stale_markers_and_string_channels():
    bundle = {"capture": {"events": [{
        "type": "scope_data",
        "samples": [
            {"control_sequence": 10, "timestamp_cycles": 100,
             "feedback_stale": False, "values": {"6": 1.0}},
            {"control_sequence": 30, "timestamp_cycles": 300,
             "feedback_stale": True, "values": {"6": 2.0}},
        ],
    }]}}

    samples = list(iter_replay_samples(bundle))

    assert [sample.control_sequence for sample in samples] == [10, 30]
    assert samples[1].feedback_stale
    assert samples[0].values == {"6": 1.0}


def test_replay_rejects_duplicate_or_backward_sequences():
    bundle = {"capture": {"events": [{
        "type": "scope_data",
        "samples": [
            {"control_sequence": 10, "timestamp_cycles": 100},
            {"control_sequence": 10, "timestamp_cycles": 101},
        ],
    }]}}
    with pytest.raises(ValueError, match="not ordered"):
        list(iter_replay_samples(bundle))


def test_missing_library_failure_is_explicit():
    with pytest.raises(NativeScenarioUnavailable, match="not built"):
        NativeScenario(Path(__file__).with_name("missing-scenario-library.dll"))


def test_ctypes_executes_closed_loop_and_timeout_scenarios():
    try:
        scenario_context = NativeScenario()
    except NativeScenarioUnavailable as exc:
        pytest.skip(str(exc))

    with scenario_context as scenario:
        scenario.submit(request_id=1, source=0, command_type=0)
        assert [result.status for result in scenario.pop_results()] == [0, 2]

        scenario.submit(request_id=2, source=1, command_type=1, operation=1)
        assert [result.status for result in scenario.pop_results()] == [0]
        prepare = scenario.pop_realtime_requests()
        assert len(prepare) == 1
        assert prepare[0].type == 0

        ready = Event()
        ready.sequence = 10
        ready.epoch = prepare[0].epoch
        ready.feedback_sequence = 10
        ready.control_sequence = 10
        ready.source = 7
        ready.type = 0
        ready.readiness_flags = 0x1F
        scenario.inject(ready)
        assert [result.status for result in scenario.pop_results()] == [2]

        scenario.submit(request_id=3, source=1, command_type=2)
        assert [result.status for result in scenario.pop_results()] == [0]
        arm = scenario.pop_realtime_requests()
        assert len(arm) == 1
        assert arm[0].type == 1

        armed = Event()
        armed.sequence = 11
        armed.epoch = arm[0].epoch
        armed.control_sequence = 11
        armed.source = 7
        armed.type = 1
        scenario.inject(armed)
        assert [result.status for result in scenario.pop_results()] == [2]
        assert scenario.state().state == 4

    with NativeScenario() as timeout:
        timeout.submit(request_id=10, source=0, command_type=0)
        timeout.pop_results()
        timeout.submit(request_id=11, source=1, command_type=1, operation=1)
        timeout.pop_results()
        timeout.advance(0xFFFFFFF0)
        timeout.advance(0x00004E10)
        assert timeout.state().state == 6
        fault = timeout.first_fault()
        assert fault is not None
        assert fault.code == 1
        assert [result.status for result in timeout.pop_results()] == [3]
