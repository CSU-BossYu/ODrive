from types import SimpleNamespace
import struct

import pytest

from odrive_hil import HilSafetyLimits, HilSession, HilSessionError
from odrive_hil import session as session_module


class FakeBus:
    channel_info = "fake"

    def __init__(self):
        self.sent = []
        self.closed = False
        self.heartbeat_pending = True

    def recv(self, timeout):
        if not self.heartbeat_pending:
            return None
        self.heartbeat_pending = False
        return SimpleNamespace(arbitration_id=1, is_extended_id=False,
                               is_remote_frame=False,
                               data=b"\x00\x00\x00\x00\x01\x00\x00\x00")

    def send(self, message):
        self.sent.append(message)

    def shutdown(self):
        self.closed = True


@pytest.fixture(autouse=True)
def fake_python_can(monkeypatch):
    monkeypatch.setitem(__import__("sys").modules, "can", SimpleNamespace(
        Message=lambda **kwargs: SimpleNamespace(**kwargs)))


def test_motion_requires_independent_estop_before_opening_bus():
    opened = []
    session = HilSession(motion=True, bus_factory=lambda *_: opened.append(True))
    with pytest.raises(HilSessionError, match="hardware-estop"):
        session.__enter__()
    assert opened == []


def test_limits_reject_values_above_hard_ceiling():
    with pytest.raises(HilSessionError, match="hard ceiling"):
        HilSafetyLimits(velocity_turns_per_s=2.1).validate()


def test_context_exit_sends_zero_commands_disarm_and_idle():
    bus = FakeBus()
    with HilSession(motion=True, hardware_estop_confirmed=True,
                    bus_factory=lambda *_: bus) as session:
        session.set_velocity(0.2)
    command_ids = [message.arbitration_id & 0x1f for message in bus.sent]
    assert 0x0e in command_ids
    assert 0x0d in command_ids
    assert 0x08 in command_ids
    assert 0x07 in command_ids
    disarm = next(message for message in bus.sent
                  if (message.arbitration_id & 0x1f) == 0x08)
    assert struct.unpack("<HBBI", disarm.data)[1] == 3
    assert bus.closed


def test_process_exit_cleanup_is_registered_only_while_bus_is_owned(monkeypatch):
    registered = []
    unregistered = []
    monkeypatch.setattr(session_module.atexit, "register",
                        lambda callback: registered.append(callback))
    monkeypatch.setattr(session_module.atexit, "unregister",
                        lambda callback: unregistered.append(callback))
    bus = FakeBus()
    session = HilSession(motion=False, bus_factory=lambda *_: bus)

    session.__enter__()
    assert registered == [session.close]
    session.close()

    assert unregistered == [session.close]
    assert bus.closed


def test_deadline_expires_deterministically():
    bus = FakeBus()
    clock_values = iter((10.0, 12.0))
    with HilSession(motion=False, limits=HilSafetyLimits(duration_s=1.0),
                    bus_factory=lambda *_: bus,
                    clock=lambda: next(clock_values)) as session:
        with pytest.raises(HilSessionError, match="deadline"):
            session.assert_active()


def test_closed_loop_arm_uses_acked_product_commands(monkeypatch):
    bus = FakeBus()
    statuses = iter((0, 2, 0, 2))

    def ack_for_last_request(*args, **kwargs):
        request_id = struct.unpack("<HBBI", bus.sent[-1].data)[0]
        return struct.pack("<HBBHH", request_id, next(statuses), 0, 7, 0)

    with HilSession(motion=True, hardware_estop_confirmed=True,
                    bus_factory=lambda *_: bus) as session:
        monkeypatch.setattr(session_module.can_simple, "recv_matching",
                            ack_for_last_request)
        session.arm_closed_loop()
        management = [message for message in bus.sent
                      if (message.arbitration_id & 0x1f) == 0x08]
        command_types = [struct.unpack("<HBBI", message.data)[1]
                         for message in management]
        assert command_types == [1, 2]
