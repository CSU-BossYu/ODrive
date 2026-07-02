#!/usr/bin/env python3
import argparse
import math
import time

from common import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    ext_request,
    get_calib_result,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_controller_modes,
    set_input_pos,
    set_input_vel,
    set_limits,
    set_pos_gain,
    set_requested_state,
    wait_heartbeat,
)


SUB_CMD_VERNIER_DIAG = 0x0A
EXT_TYPE_UINT32 = 3


def sign(value, deadband=1e-6):
    if value > deadband:
        return "+"
    if value < -deadband:
        return "-"
    return "0"


def wrap_pm_half(value):
    wrapped = (value + 0.5) % 1.0 - 0.5
    if wrapped >= 0.5:
        wrapped -= 1.0
    return wrapped


def require_clean_hb(hb, context):
    if hb is None:
        raise RuntimeError(f"{context}: heartbeat timeout")
    if hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]:
        raise RuntimeError(
            f"{context}: heartbeat error state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} "
            f"flags=0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )


def wait_for_state(bus, args, target, timeout=5.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.25)
        if hb is None:
            continue
        last = hb
        print(
            f"  hb state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error=0x{hb['axis_error']:08X} flags="
            f"0x{hb['motor_flags']:02X}/0x{hb['encoder_flags']:02X}/0x{hb['controller_flags']:02X}"
        )
        require_clean_hb(hb, f"waiting for {target}")
        if hb["axis_state"] == target:
            return hb
    raise RuntimeError(f"Timed out waiting for {target}; last={last}")


def read_pair(bus, args, cmd, timeout=0.2):
    _, value = request_two_floats(bus, args.node_id, cmd, args.extended_id, timeout=timeout)
    return value


def read_vernier_float(bus, args, item):
    resp = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=0.3,
    )
    if resp is None or resp["status"] != 0:
        return float("nan")
    return resp["value_f"]


def read_vernier_u32(bus, args, item):
    resp = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=0.3,
    )
    if resp is None or resp["status"] != 0:
        return None
    return resp["value_u"]


def snapshot(bus, args):
    enc = read_pair(bus, args, CMD_GET_ENCODER_ESTIMATES)
    iq = read_pair(bus, args, CMD_GET_IQ)
    bus_vi = read_pair(bus, args, CMD_GET_BUS_VOLTAGE_CURRENT)
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)
    pos = enc[0] if enc else float("nan")
    vel = enc[1] if enc else float("nan")
    iq_set = iq[0] if iq else float("nan")
    iq_meas = iq[1] if iq else float("nan")
    vbus = bus_vi[0] if bus_vi else float("nan")
    ibus = bus_vi[1] if bus_vi else float("nan")
    main_angle = read_vernier_u32(bus, args, 0x00)
    aux_angle = read_vernier_u32(bus, args, 0x01)
    main_phase = ((main_angle % args.cpr) / float(args.cpr)) if main_angle is not None else float("nan")
    aux_phase = ((aux_angle % args.cpr) / float(args.cpr)) if aux_angle is not None else float("nan")
    aux_minus_main = wrap_pm_half(aux_phase - main_phase) if main_angle is not None and aux_angle is not None else float("nan")
    main_minus_aux = wrap_pm_half(main_phase - aux_phase) if main_angle is not None and aux_angle is not None else float("nan")
    vernier_pos = read_vernier_float(bus, args, 0x08)
    enc_pos_diag = read_vernier_float(bus, args, 0x20)
    enc_vel_diag = read_vernier_float(bus, args, 0x21)
    circ = read_vernier_float(bus, args, 0x22)
    if hb:
        require_clean_hb(hb, "snapshot")
    return {
        "pos": pos,
        "vel": vel,
        "iq_set": iq_set,
        "iq_meas": iq_meas,
        "vbus": vbus,
        "ibus": ibus,
        "main_angle": main_angle if main_angle is not None else -1,
        "aux_angle": aux_angle if aux_angle is not None else -1,
        "main_phase": main_phase,
        "aux_phase": aux_phase,
        "aux_minus_main": aux_minus_main,
        "main_minus_aux": main_minus_aux,
        "vernier_pos": vernier_pos,
        "enc_pos_diag": enc_pos_diag,
        "enc_vel_diag": enc_vel_diag,
        "circular": circ,
    }


def print_sample(label, t, s, base=None, target=None):
    dpos = s["pos"] - base["pos"] if base else 0.0
    dvernier = s["vernier_pos"] - base["vernier_pos"] if base else 0.0
    dmain = wrap_pm_half(s["main_phase"] - base["main_phase"]) if base else 0.0
    daux = wrap_pm_half(s["aux_phase"] - base["aux_phase"]) if base else 0.0
    d_aux_main = wrap_pm_half(s["aux_minus_main"] - base["aux_minus_main"]) if base else 0.0
    d_main_aux = wrap_pm_half(s["main_minus_aux"] - base["main_minus_aux"]) if base else 0.0
    err = target - s["pos"] if target is not None else float("nan")
    err_txt = "" if target is None else f" err={err:+.5f}({sign(err)})"
    print(
        f"  {label} t={t:4.2f}s pos={s['pos']:+.5f} dpos={dpos:+.5f}({sign(dpos)}) "
        f"vel={s['vel']:+.5f}({sign(s['vel'], 1e-3)}) "
        f"vern={s['vernier_pos']:+.5f} dvern={dvernier:+.5f}({sign(dvernier)}) "
        f"main={s['main_angle']:5d} dmain={dmain:+.5f}({sign(dmain)}) "
        f"aux={s['aux_angle']:5d} daux={daux:+.5f}({sign(daux)}) "
        f"dA-M={d_aux_main:+.5f}({sign(d_aux_main)}) dM-A={d_main_aux:+.5f}({sign(d_main_aux)}) "
        f"diag_pos={s['enc_pos_diag']:+.5f} diag_vel={s['enc_vel_diag']:+.5f} "
        f"circ={s['circular']:+.5f} Iq={s['iq_set']:+.3f}/{s['iq_meas']:+.3f}{err_txt}"
    )


def run_velocity_probe(bus, args, velocity):
    print(f"Velocity probe cmd={velocity:+.4f} turns/s for {args.velocity_duration:.2f}s")
    clear_errors(bus, args.node_id, args.extended_id)
    time.sleep(0.1)
    set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
    set_controller_modes(bus, args.node_id, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
    set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
    set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
    wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL)
    base = snapshot(bus, args)
    print_sample("vel-base", 0.0, base)
    set_input_vel(bus, args.node_id, velocity, 0.0, args.extended_id)
    start = time.monotonic()
    last = base
    while time.monotonic() - start < args.velocity_duration:
        t = time.monotonic() - start
        s = snapshot(bus, args)
        print_sample("vel", t, s, base)
        last = s
        time.sleep(args.sample_period)
    set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
    set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
    wait_for_state(bus, args, AXIS_STATE_IDLE, timeout=3.0)
    print(
        f"Velocity summary cmd={velocity:+.4f}: dpos={last['pos'] - base['pos']:+.5f} "
        f"sign(cmd)={sign(velocity)} sign(dpos)={sign(last['pos'] - base['pos'])} "
        f"last vel sign={sign(last['vel'], 1e-3)}"
    )


def run_position_probe(bus, args, step):
    print(f"Position probe step={step:+.4f} turn, pos_gain={args.pos_gain:.3f}, duration={args.position_duration:.2f}s")
    clear_errors(bus, args.node_id, args.extended_id)
    time.sleep(0.1)
    set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
    set_pos_gain(bus, args.node_id, args.pos_gain, args.extended_id)
    set_controller_modes(bus, args.node_id, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH, args.extended_id)
    base = snapshot(bus, args)
    target = base["pos"]
    set_input_pos(bus, args.node_id, target, 0.0, 0.0, args.extended_id)
    set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
    wait_for_state(bus, args, AXIS_STATE_CLOSED_LOOP_CONTROL)
    time.sleep(0.2)
    base = snapshot(bus, args)
    target = base["pos"] + step
    print_sample("pos-base", 0.0, base, target=target)
    set_input_pos(bus, args.node_id, target, 0.0, 0.0, args.extended_id)
    start = time.monotonic()
    last = base
    while time.monotonic() - start < args.position_duration:
        t = time.monotonic() - start
        s = snapshot(bus, args)
        print_sample("pos", t, s, base, target=target)
        last = s
        if abs(target - s["pos"]) > args.abort_position_error:
            print(f"  aborting position probe: |error|>{args.abort_position_error}")
            break
        time.sleep(args.sample_period)
    set_input_pos(bus, args.node_id, base["pos"], 0.0, 0.0, args.extended_id)
    time.sleep(0.1)
    set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
    wait_for_state(bus, args, AXIS_STATE_IDLE, timeout=3.0)
    print(
        f"Position summary step={step:+.4f}: dpos={last['pos'] - base['pos']:+.5f} "
        f"sign(step)={sign(step)} sign(dpos)={sign(last['pos'] - base['pos'])} "
        f"final error={target - last['pos']:+.5f}"
    )


def main():
    parser = argparse.ArgumentParser(description="Diagnose position/velocity sign chain for MT6826S vernier feedback.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--velocity", type=float, default=0.5)
    parser.add_argument("--velocity-duration", type=float, default=0.8)
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--position-duration", type=float, default=0.8)
    parser.add_argument("--sample-period", type=float, default=0.2)
    parser.add_argument("--pos-gain", type=float, default=0.5)
    parser.add_argument("--vel-limit", type=float, default=2.0)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--cpr", type=int, default=32768)
    parser.add_argument("--abort-position-error", type=float, default=0.12)
    parser.add_argument("--skip-velocity", action="store_true")
    parser.add_argument("--skip-position", action="store_true")
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        require_clean_hb(hb, "initial")
        status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
        print(f"Initial status: {status}")
        for item, name in [(0x01, "phase_resistance"), (0x02, "phase_inductance"), (0x03, "phase_offset"), (0x04, "direction")]:
            result = get_calib_result(bus, args.node_id, item, args.extended_id)
            print(f"  {name}: {result[1] if result else 'TIMEOUT'} raw={result[0]['raw'] if result else 'TIMEOUT'}")
        if status is None or status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
            raise RuntimeError("Axis is not calibrated/ready")

        if not args.skip_velocity:
            run_velocity_probe(bus, args, +abs(args.velocity))
            run_velocity_probe(bus, args, -abs(args.velocity))
        if not args.skip_position:
            run_position_probe(bus, args, +abs(args.step))
            run_position_probe(bus, args, -abs(args.step))

        if args.clear_at_end:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)
        print("PASS: direction diagnostic completed without final fault")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        set_input_vel(bus, args.node_id, 0.0, 0.0, args.extended_id)
        set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
