"""Fail-closed lifecycle for tests that can energize a real motor."""

from __future__ import annotations

import atexit
from dataclasses import dataclass
import time

from . import can_simple
from .can_product_schema_generated import (
    ProductCanMessageId, decode_command_ack, encode_management_command,
)


class HilSessionError(RuntimeError):
    pass


PRODUCT_COMMAND_DISARM = 3
PRODUCT_COMMAND_SET_OPERATION = 1
PRODUCT_COMMAND_ARM = 2
PRODUCT_OPERATION_CLOSED_LOOP = 1


@dataclass(frozen=True)
class HilSafetyLimits:
    velocity_turns_per_s: float = 0.5
    current_amps: float = 0.5
    torque_nm: float = 0.1
    duration_s: float = 30.0

    def validate(self) -> None:
        values = (self.velocity_turns_per_s, self.current_amps,
                  self.torque_nm, self.duration_s)
        if any(value <= 0 for value in values):
            raise HilSessionError("all HIL safety limits must be positive")
        if self.velocity_turns_per_s > 2.0 or self.current_amps > 1.0:
            raise HilSessionError("HIL hard ceiling is 2 rev/s and 1 A")
        if self.torque_nm > 0.2 or self.duration_s > 120.0:
            raise HilSessionError("HIL hard ceiling is 0.2 Nm and 120 s")


class HilSession:
    """Own one CAN adapter and guarantee a best-effort safe shutdown."""

    _request_id = 1

    def __init__(self, *, channel="PCAN_USBBUS1", bitrate=1_000_000,
                 node_id=0, extended_id=False, motion=False,
                 hardware_estop_confirmed=False, limits=None, bus_factory=None,
                 clock=time.monotonic):
        self.channel = channel
        self.bitrate = bitrate
        self.node_id = node_id
        self.extended_id = extended_id
        self.motion = motion
        self.hardware_estop_confirmed = hardware_estop_confirmed
        self.limits = limits or HilSafetyLimits()
        self._bus_factory = bus_factory or can_simple.open_bus
        self._clock = clock
        self._deadline = None
        self.bus = None
        self._exit_registered = False

    def __enter__(self):
        self.limits.validate()
        if self.motion and not self.hardware_estop_confirmed:
            raise HilSessionError(
                "motion HIL requires explicit hardware-estop confirmation")
        self.bus = self._bus_factory(self.channel, self.bitrate)
        atexit.register(self.close)
        self._exit_registered = True
        heartbeat = can_simple.wait_heartbeat(
            self.bus, self.node_id, self.extended_id, timeout=2.0)
        if heartbeat is None:
            self.close()
            raise HilSessionError("no heartbeat received before HIL session")
        self._deadline = self._clock() + self.limits.duration_s
        if self.motion:
            can_simple.set_limits(self.bus, self.node_id,
                                  self.limits.velocity_turns_per_s,
                                  self.limits.current_amps,
                                  self.extended_id)
        return self

    def assert_active(self):
        if self.bus is None:
            raise HilSessionError("HIL session is not active")
        if self._deadline is not None and self._clock() > self._deadline:
            self.safe_stop()
            raise HilSessionError("HIL session deadline exceeded")

    def set_velocity(self, velocity, torque_ff=0.0):
        self.assert_active()
        if abs(velocity) > self.limits.velocity_turns_per_s:
            raise HilSessionError("velocity command exceeds HIL session limit")
        can_simple.set_input_vel(self.bus, self.node_id, velocity, torque_ff,
                                 self.extended_id)

    def set_torque(self, torque):
        self.assert_active()
        if abs(torque) > self.limits.torque_nm:
            raise HilSessionError("torque command exceeds HIL session limit")
        can_simple.set_input_torque(self.bus, self.node_id, torque,
                                    self.extended_id)

    def command(self, command_type, operation=0, arg0=0, timeout=3.0):
        """Send one product management command and await its terminal ACK."""
        self.assert_active()
        request_id = self._allocate_request_id()
        payload = encode_management_command(request_id, command_type,
                                            operation, arg0)
        can_simple.send(self.bus, self.node_id,
                        int(ProductCanMessageId.MANAGEMENT_COMMAND), payload,
                        self.extended_id)
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            data = can_simple.recv_matching(
                self.bus, self.node_id, int(ProductCanMessageId.COMMAND_ACK),
                self.extended_id, max(0.0, deadline - self._clock()))
            if data is None:
                break
            ack = decode_command_ack(data)
            if ack["request_id"] != request_id:
                continue
            if ack["status"] == 2:
                return ack
            if ack["status"] in (1, 3):
                raise HilSessionError(
                    f"management command {command_type} failed: "
                    f"status={ack['status']} reason={ack['reason']}")
            if ack["status"] != 0:
                raise HilSessionError(
                    f"management command returned unknown status {ack['status']}")
        raise HilSessionError(
            f"management command {command_type} timed out before completion")

    def arm_closed_loop(self, timeout=3.0):
        self.command(PRODUCT_COMMAND_SET_OPERATION,
                     PRODUCT_OPERATION_CLOSED_LOOP, timeout=timeout)
        return self.command(PRODUCT_COMMAND_ARM,
                            PRODUCT_OPERATION_CLOSED_LOOP, timeout=timeout)

    def safe_stop(self):
        if self.bus is None:
            return
        for action in (
                lambda: can_simple.set_input_torque(
                    self.bus, self.node_id, 0.0, self.extended_id),
                lambda: can_simple.set_input_vel(
                    self.bus, self.node_id, 0.0, 0.0, self.extended_id),
                self._send_product_disarm,
                lambda: can_simple.set_requested_state(
                    self.bus, self.node_id, can_simple.AXIS_STATE_IDLE,
                    self.extended_id)):
            try:
                action()
            except Exception:
                pass

    def _send_product_disarm(self):
        request_id = self._allocate_request_id()
        payload = encode_management_command(request_id, PRODUCT_COMMAND_DISARM,
                                            0, 0)
        can_simple.send(self.bus, self.node_id,
                        int(ProductCanMessageId.MANAGEMENT_COMMAND), payload,
                        self.extended_id)

    @classmethod
    def _allocate_request_id(cls):
        request_id = cls._request_id
        cls._request_id = 1 if request_id == 0xFFFF else request_id + 1
        return request_id

    def close(self):
        if self.bus is None:
            if self._exit_registered:
                atexit.unregister(self.close)
                self._exit_registered = False
            return
        self.safe_stop()
        bus, self.bus = self.bus, None
        try:
            bus.shutdown()
        except Exception:
            pass
        if self._exit_registered:
            atexit.unregister(self.close)
            self._exit_registered = False

    def __exit__(self, exc_type, exc, traceback):
        self.close()
        return False
