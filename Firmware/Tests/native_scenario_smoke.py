"""Standard-library smoke test for the built native scenario shared library."""

from odrive_hil.native_scenario import NativeScenario


def main() -> None:
    with NativeScenario() as scenario:
        scenario.submit(request_id=1, source=0, command_type=0)
        assert [result.status for result in scenario.pop_results()] == [0, 2]

        scenario.submit(request_id=2, source=1, command_type=1, operation=1)
        assert [result.status for result in scenario.pop_results()] == [0]
        assert len(scenario.pop_realtime_requests()) == 1

        scenario.advance(0xFFFFFFF0)
        scenario.advance(0x00004E10)
        assert scenario.state().state == 6
        fault = scenario.first_fault()
        assert fault is not None and fault.code == 1
        assert [result.status for result in scenario.pop_results()] == [3]


if __name__ == "__main__":
    main()
