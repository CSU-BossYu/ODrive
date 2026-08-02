#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import time

_TESTS_ROOT = Path(__file__).resolve().parents[1]
if str(_TESTS_ROOT) not in sys.path:
    sys.path.insert(0, str(_TESTS_ROOT))

from odrive_hil import HilSafetyLimits, HilSession
from odrive_hil.can_simple import (
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_CONTROLLER_ERROR,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CMD_GET_MOTOR_ERROR,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    clear_errors,
    ext_request,
    request_two_floats,
    request_u32,
    request_u64,
    wait_heartbeat,
)

GET_CTRL = 0x0B
SET_CTRL = 0x0C
SERVO_MODE = 0x5B
VEL_ACCEL = 0x50
VEL_DECEL = 0x51
CAN_WD_MS = 0x53

MOTOR_BITS = [
    (0x0000000000001000, "CURRENT_LIMIT_VIOLATION"),
    (0x0000000001000000, "SYSTEM_LEVEL"),
]
ODRV_BITS = [
    (0x00000002, "DC_BUS_UNDER_VOLTAGE"),
    (0x00000004, "DC_BUS_OVER_VOLTAGE"),
    (0x00000008, "DC_BUS_OVER_REGEN_CURRENT"),
    (0x00000010, "DC_BUS_OVER_CURRENT"),
]


def names(value, table):
    return ",".join(name for bit, name in table if value and value & bit) or "NONE"


def ctrl_set(bus, nid, item, value, is_float=False):
    resp = ext_request(
        bus, nid, SET_CTRL, item=item,
        req_type=EXT_TYPE_FLOAT32 if is_float else EXT_TYPE_UINT32,
        value_float=value if is_float else None,
        value=0 if is_float else int(value),
    )
    return resp


def ctrl_get(bus, nid, item, is_float=False):
    resp = ext_request(bus, nid, GET_CTRL, item=item)
    if resp is None:
        return None
    return resp["value_f"] if is_float else resp["value_u"]


def read_errors(bus, nid):
    _, motor = request_u64(bus, nid, CMD_GET_MOTOR_ERROR, timeout=0.3)
    _, enc = request_u32(bus, nid, CMD_GET_ENCODER_ERROR, timeout=0.3)
    _, ctrl = request_u32(bus, nid, CMD_GET_CONTROLLER_ERROR, timeout=0.3)
    odrv = ext_request(bus, nid, 0x08, item=0x06, req_type=EXT_TYPE_UINT32, value=0, timeout=0.3)
    return motor, enc, ctrl, (odrv["value_u"] if odrv else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node-id", type=int, default=0)
    ap.add_argument("--velocity", type=float, default=0.3)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--current-limit", type=float, default=0.5)
    ap.add_argument("--vel-limit", type=float, default=0.5)
    ap.add_argument("--accel", type=float, default=20.0)
    ap.add_argument("--watchdog-ms", type=int, default=800)
    ap.add_argument("--hardware-estop-ready", action="store_true",
                    help="confirm an independent hardware emergency stop is ready")
    args = ap.parse_args()

    session = HilSession(
        channel=args.channel, bitrate=args.bitrate, node_id=args.node_id,
        motion=True, hardware_estop_confirmed=args.hardware_estop_ready,
        limits=HilSafetyLimits(velocity_turns_per_s=args.vel_limit,
                               current_amps=args.current_limit,
                               duration_s=args.duration + 5.0))
    session.__enter__()
    bus = session.bus
    period = 1.0 / args.hz
    try:
        print(f"Opened {bus.channel_info}")
        clear_errors(bus, args.node_id)
        ctrl_set(bus, args.node_id, VEL_ACCEL, args.accel, True)
        ctrl_set(bus, args.node_id, VEL_DECEL, args.accel, True)
        ctrl_set(bus, args.node_id, CAN_WD_MS, args.watchdog_ms, False)
        ctrl_set(bus, args.node_id, SERVO_MODE, 1, False)
        session.set_velocity(0.0)
        session.arm_closed_loop()

        print(
            f"servo_mode={ctrl_get(bus, args.node_id, SERVO_MODE)} "
            f"accel={ctrl_get(bus, args.node_id, VEL_ACCEL, True)} "
            f"wd={ctrl_get(bus, args.node_id, CAN_WD_MS)}"
        )
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            t = time.monotonic() - start
            session.set_velocity(args.velocity)
            _, enc = request_two_floats(bus, args.node_id, CMD_GET_ENCODER_ESTIMATES, timeout=0.05)
            _, iq = request_two_floats(bus, args.node_id, CMD_GET_IQ, timeout=0.05)
            _, bus_vi = request_two_floats(bus, args.node_id, CMD_GET_BUS_VOLTAGE_CURRENT, timeout=0.05)
            hb = wait_heartbeat(bus, args.node_id, timeout=0.05)
            vel = enc[1] if enc else float("nan")
            iq_sp = iq[0] if iq else float("nan")
            iq_meas = iq[1] if iq else float("nan")
            vbus = bus_vi[0] if bus_vi else float("nan")
            ibus = bus_vi[1] if bus_vi else float("nan")
            hb_txt = ""
            if hb:
                hb_txt = (
                    f" state={hb['axis_state']} axis=0x{hb['axis_error']:08x} "
                    f"flags={hb['motor_flags']:02x}/{hb['encoder_flags']:02x}/{hb['controller_flags']:02x}"
                )
            print(
                f"t={t:5.2f} cmd={args.velocity: .3f} vel={vel: .3f} "
                f"Iq={iq_sp: .3f}/{iq_meas: .3f} bus={vbus: .2f}V/{ibus: .3f}A{hb_txt}"
            )
            if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
                motor, enc_err, ctrl_err, odrv = read_errors(bus, args.node_id)
                print(f"errors motor=0x{motor or 0:016x} {names(motor, MOTOR_BITS)}")
                print(f"errors encoder=0x{enc_err or 0:08x} controller=0x{ctrl_err or 0:08x}")
                print(f"errors odrv=0x{odrv or 0:08x} {names(odrv, ODRV_BITS)}")
                return 2
            time.sleep(period)
        return 0
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
