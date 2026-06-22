#!/usr/bin/env python3
import argparse
import struct
import time

import can


CMD_HEARTBEAT = 0x001
CMD_EXTENDED = 0x01E


def arb_id(node_id, cmd_id):
    return (node_id << 5) | cmd_id


def decode_heartbeat(msg):
    axis_error, axis_state, motor_err, enc_err, ctrl_flags = struct.unpack_from("<IBBBB", msg.data, 0)
    trajectory_done = bool(ctrl_flags & 0x80)
    return {
        "axis_error": axis_error,
        "axis_state": axis_state,
        "motor_error_flag": bool(motor_err),
        "encoder_error_flag": bool(enc_err),
        "controller_error_flag": bool(ctrl_flags & 0x01),
        "trajectory_done": trajectory_done,
    }


def recv_matching(bus, expected_id, extended_id=False, timeout=1.0, sub_cmd=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(deadline - time.monotonic())
        if msg is None:
            continue
        if msg.arbitration_id != expected_id or msg.is_extended_id != extended_id or msg.is_remote_frame:
            continue
        if sub_cmd is not None and (len(msg.data) < 1 or msg.data[0] != sub_cmd):
            continue
        return msg
    return None


def ext_request(bus, node_id, sub_cmd, item=0, req_type=0, value=0, extended_id=False, timeout=1.0):
    data = struct.pack("<BBBBi", sub_cmd, item, req_type, 0, value)
    msg = can.Message(
        arbitration_id=arb_id(node_id, CMD_EXTENDED),
        is_extended_id=extended_id,
        data=data,
        dlc=8,
    )
    bus.send(msg)
    resp = recv_matching(bus, arb_id(node_id, CMD_EXTENDED), extended_id, timeout, sub_cmd)
    if resp is None:
        return None
    sub_cmd_r, b1, b2, b3, value_i = struct.unpack("<BBBBi", bytes(resp.data[:8]))
    value_u = struct.unpack("<BBBBI", bytes(resp.data[:8]))[4]
    value_f = struct.unpack("<f", bytes(resp.data[4:8]))[0]
    return {
        "sub_cmd": sub_cmd_r,
        "byte1": b1,
        "byte2": b2,
        "byte3": b3,
        "value_i": value_i,
        "value_u": value_u,
        "value_f": value_f,
        "raw": bytes(resp.data).hex(" "),
    }


def main():
    parser = argparse.ArgumentParser(description="Verify ODrive CANSimple over PCAN-USB.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=250000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--listen-seconds", type=float, default=3.0)
    args = parser.parse_args()

    bus = can.Bus(interface="pcan", channel=args.channel, bitrate=args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        print(f"Listening for heartbeat: node_id={args.node_id}, id=0x{arb_id(args.node_id, CMD_HEARTBEAT):03X}")

        heartbeat = recv_matching(
            bus,
            arb_id(args.node_id, CMD_HEARTBEAT),
            args.extended_id,
            timeout=args.listen_seconds,
        )
        if heartbeat is None:
            print("FAIL: no heartbeat received.")
            print("Check: CANH/CANL/GND wiring, 120 ohm termination, ODrive CAN enabled, bitrate, node_id.")
            return 1

        hb = decode_heartbeat(heartbeat)
        print("Heartbeat OK:")
        print(f"  raw={bytes(heartbeat.data).hex(' ')}")
        print(f"  axis_state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X}")
        print(f"  motor_error_flag={hb['motor_error_flag']} encoder_error_flag={hb['encoder_error_flag']} controller_error_flag={hb['controller_error_flag']}")

        status = ext_request(bus, args.node_id, 0x01, extended_id=args.extended_id, timeout=1.0)
        if status is None:
            print("FAIL: heartbeat works, but Extended Command 0x01 did not respond.")
            return 2

        flags = status["byte3"]
        print("Extended GET_AXIS_STATUS_EX OK:")
        print(f"  raw={status['raw']}")
        print(f"  status={status['byte1']} current_state={status['byte2']} flags=0x{flags:02X} axis_error=0x{status['value_u']:08X}")
        print(f"  motor_calibrated={bool(flags & 0x01)} encoder_ready={bool(flags & 0x02)}")

        proto = ext_request(bus, args.node_id, 0x05, item=0x01, extended_id=args.extended_id, timeout=1.0)
        fw = ext_request(bus, args.node_id, 0x05, item=0x02, extended_id=args.extended_id, timeout=1.0)
        if proto is None or fw is None:
            print("FAIL: GET_DEVICE_INFO did not respond.")
            return 3

        fw_value = fw["value_u"]
        print("Extended GET_DEVICE_INFO OK:")
        print(f"  protocol=0x{proto['value_u']:08X}")
        print(f"  fw={(fw_value >> 24) & 0xff}.{(fw_value >> 16) & 0xff}.{(fw_value >> 8) & 0xff}.{fw_value & 0xff}")

        print("PASS: PCAN-USB <-> ODrive CAN path and firmware extension are responding.")
        return 0
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
