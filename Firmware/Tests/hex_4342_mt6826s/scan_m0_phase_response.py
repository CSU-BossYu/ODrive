import argparse
import time

from common import (
    CMD_GET_MOTOR_ERROR,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    clear_errors,
    ext_request,
    open_bus,
    request_u64,
    wait_heartbeat,
)


SUB_CMD_VERNIER_DIAG = 0x0A
SUB_CMD_PHASE_SCAN = 0x0B

VECTORS = [
    (1, "+alpha"),
    (2, "-alpha"),
    (3, "+beta"),
    (4, "-beta"),
    (5, "+60deg"),
    (6, "-60deg"),
]


def read_diag_item(bus, args, item_id, want_float=False):
    resp = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item_id, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=args.timeout,
    )
    if resp is None or resp["status"] != 0:
        return None
    return resp["value_f"] if want_float else resp["value_u"]


def read_system_error(bus, args):
    resp = ext_request(
        bus, args.node_id, 0x08,
        item=0x06, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=args.timeout,
    )
    if resp is None or resp["status"] != 0:
        return None
    return resp["value_u"]


def send_phase_scan(bus, args, item, voltage=0.0):
    value_float = voltage if item != 0 else 0.0
    resp = ext_request(
        bus, args.node_id, SUB_CMD_PHASE_SCAN,
        item=item, req_type=EXT_TYPE_FLOAT32, value_float=value_float,
        extended_id=args.extended_id, timeout=args.timeout,
    )
    if resp is None:
        raise RuntimeError(f"phase scan item {item}: TIMEOUT")
    status = EXT_STATUS.get(resp["status"], resp["status"])
    print(f"  phase_scan item={item} voltage={value_float:.3f}: {status} raw={resp['raw']}")
    if resp["status"] != 0:
        raise RuntimeError(f"phase scan item {item}: {status}")
    return resp


def read_sample(bus, args):
    sample = {
        "active": read_diag_item(bus, args, 0x5A),
        "vector": read_diag_item(bus, args, 0x5B),
        "v_alpha": read_diag_item(bus, args, 0x5C, True),
        "v_beta": read_diag_item(bus, args, 0x5D, True),
        "i_alpha": read_diag_item(bus, args, 0x5E, True),
        "i_beta": read_diag_item(bus, args, 0x5F, True),
        "missing": read_diag_item(bus, args, 0x60),
        "avg_n": read_diag_item(bus, args, 0x61),
        "avg_ia": read_diag_item(bus, args, 0x62, True),
        "avg_ib": read_diag_item(bus, args, 0x63, True),
        "avg_ic": read_diag_item(bus, args, 0x64, True),
        "avg_i_alpha": read_diag_item(bus, args, 0x65, True),
        "avg_i_beta": read_diag_item(bus, args, 0x66, True),
        "ia": read_diag_item(bus, args, 0x57, True),
        "ib": read_diag_item(bus, args, 0x58, True),
        "ic": read_diag_item(bus, args, 0x59, True),
        "present": read_diag_item(bus, args, 0x4F),
        "dcok": read_diag_item(bus, args, 0x50),
        "valid": read_diag_item(bus, args, 0x51),
    }
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)
    _, motor_error = request_u64(
        bus, args.node_id, CMD_GET_MOTOR_ERROR, args.extended_id, timeout=0.1
    )
    sample["axis_error"] = None if hb is None else hb["axis_error"]
    sample["axis_state"] = None if hb is None else hb["axis_state"]
    sample["motor_error"] = motor_error
    sample["system_error"] = read_system_error(bus, args)
    return sample


def wait_dc_calib_ready(bus, args, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        dcok = read_diag_item(bus, args, 0x50)
        present = read_diag_item(bus, args, 0x4F)
        valid = read_diag_item(bus, args, 0x51)
        if dcok == 1 and present == 1 and valid == 1:
            return True
        time.sleep(0.05)
    return False


def fmt(value, digits=3):
    return "nan" if value is None else f"{value:.{digits}f}"


def main():
    parser = argparse.ArgumentParser(description="Scan M0 phase response with short fixed voltage vectors.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--voltage", type=float, default=0.5)
    parser.add_argument("--settle", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--no-clear-start", action="store_true")
    args = parser.parse_args()

    if args.voltage <= 0.0 or args.voltage > 1.0:
        raise ValueError("--voltage must be in (0, 1.0] V")

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}")

        send_phase_scan(bus, args, 0)
        if not args.no_clear_start:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        print("Waiting for M0 current-sense DC calibration...")
        if not wait_dc_calib_ready(bus, args):
            raise RuntimeError("M0 current-sense DC calibration did not become ready")

        print("vector    Vab[V]          avgIab[A]       avgIabc[A]          lastIab/Iabc[A]        flags")
        for item, name in VECTORS:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.1)
            if not wait_dc_calib_ready(bus, args, timeout=1.0):
                print(f"{name:7s} skipped: dc calibration not ready")
                continue
            send_phase_scan(bus, args, item, args.voltage)
            time.sleep(args.settle)
            sample = read_sample(bus, args)
            axis_state = "?" if sample["axis_state"] is None else str(sample["axis_state"])
            axis_error = sample["axis_error"] or 0
            motor_error = sample["motor_error"] or 0
            system_error = sample["system_error"] or 0
            print(
                f"{name:7s} "
                f"{fmt(sample['v_alpha'])}/{fmt(sample['v_beta'])}   "
                f"{fmt(sample['avg_i_alpha'])}/{fmt(sample['avg_i_beta'])}   "
                f"{fmt(sample['avg_ia'])}/{fmt(sample['avg_ib'])}/{fmt(sample['avg_ic'])}   "
                f"{fmt(sample['i_alpha'])}/{fmt(sample['i_beta'])} "
                f"{fmt(sample['ia'])}/{fmt(sample['ib'])}/{fmt(sample['ic'])}   "
                f"active={sample['active']} vec={sample['vector']} "
                f"present={sample['present']} dcok={sample['dcok']} valid={sample['valid']} "
                f"n={sample['avg_n']} miss={sample['missing']} "
                f"axis={axis_state}/0x{axis_error:08X} "
                f"motor=0x{motor_error:016X} "
                f"sys=0x{system_error:08X}"
            )
            send_phase_scan(bus, args, 0)
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.08)

    finally:
        try:
            send_phase_scan(bus, args, 0)
        except Exception as ex:
            print(f"WARNING: failed to stop phase scan: {ex}")
        bus.shutdown()


if __name__ == "__main__":
    main()
