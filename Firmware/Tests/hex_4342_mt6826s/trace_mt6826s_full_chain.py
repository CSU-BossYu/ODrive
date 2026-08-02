#!/usr/bin/env python3
import argparse
import math
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATES,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CONTROL_MODE_POSITION_CONTROL,
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    INPUT_MODE_PASSTHROUGH,
    clear_errors,
    ext_request,
    get_basic_config,
    get_status_ex,
    open_bus,
    request_two_floats,
    set_controller_modes,
    set_input_pos,
    set_limits,
    set_pos_gain,
    set_requested_state,
    set_vel_gains,
    wait_heartbeat,
)

from run_mt6826s_velocity_loop import format_axis_error, print_error_summary, read_error_summary


SUB_CMD_VERNIER_DIAG = 0x0A
EXT_TYPE_UINT32 = 3
CPR = 32768

CONFIG_ITEMS = [
    (0x20, "encoder.mode"),
    (0x21, "encoder.cpr"),
    (0x22, "encoder.abs_spi_cs_gpio_pin"),
    (0x24, "encoder.abs_spi_aux_cs_gpio_pin"),
    (0x25, "encoder.vernier_virtual_cpr"),
    (0x26, "encoder.vernier_main_ratio"),
    (0x27, "encoder.vernier_aux_ratio"),
    (0x28, "encoder.vernier_main_offset"),
    (0x29, "encoder.vernier_aux_offset"),
    (0x2A, "encoder.vernier_main_reversed"),
    (0x2B, "encoder.vernier_aux_reversed"),
    (0x33, "encoder.vernier_output_reversed"),
    (0x2C, "encoder.mt6826s_spi_mode"),
    (0x2F, "encoder.mt6826s_spi_prescaler"),
]


def wrap_pm_half(value):
    wrapped = (value + 0.5) % 1.0 - 0.5
    if wrapped >= 0.5:
        wrapped -= 1.0
    return wrapped


def turns_to_deg(turns):
    return turns * 360.0


def read_pair(bus, args, cmd, timeout=0.2):
    _, value = request_two_floats(bus, args.node_id, cmd, args.extended_id, timeout=timeout)
    return value


def read_diag_item(bus, args, item):
    resp = ext_request(
        bus, args.node_id, SUB_CMD_VERNIER_DIAG,
        item=item, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=args.extended_id, timeout=0.3,
    )
    if resp is None or resp["status"] != 0:
        return None
    return resp


def print_config(bus, args):
    print("Current vernier-related config:")
    for item, name in CONFIG_ITEMS:
        resp = get_basic_config(bus, args.node_id, item, args.extended_id)
        if resp is None:
            print(f"  {name}: TIMEOUT")
            continue
        value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_i"]
        status = EXT_STATUS.get(resp["status"], resp["status"])
        print(f"  {name}: {value} status={status} raw={resp['raw']}")


def read_diag_u32(bus, args, item):
    resp = read_diag_item(bus, args, item)
    return None if resp is None else resp["value_u"]


def read_diag_f32(bus, args, item):
    resp = read_diag_item(bus, args, item)
    return float("nan") if resp is None else resp["value_f"]


def phase(angle):
    if angle is None:
        return float("nan")
    return (angle % CPR) / float(CPR)


def snapshot(bus, args):
    enc = read_pair(bus, args, CMD_GET_ENCODER_ESTIMATES)
    iq = read_pair(bus, args, CMD_GET_IQ)
    bus_vi = read_pair(bus, args, CMD_GET_BUS_VOLTAGE_CURRENT)
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=0.05)
    main = read_diag_u32(bus, args, 0x00)
    aux = read_diag_u32(bus, args, 0x01)
    main_phase = phase(main)
    aux_phase = phase(aux)
    aux_minus_main = wrap_pm_half(aux_phase - main_phase)
    main_minus_aux = wrap_pm_half(main_phase - aux_phase)
    return {
        "pos": enc[0] if enc else float("nan"),
        "vel": enc[1] if enc else float("nan"),
        "iq_set": iq[0] if iq else float("nan"),
        "iq_meas": iq[1] if iq else float("nan"),
        "vbus": bus_vi[0] if bus_vi else float("nan"),
        "ibus": bus_vi[1] if bus_vi else float("nan"),
        "hb": hb,
        "main": main,
        "aux": aux,
        "main_phase": main_phase,
        "aux_phase": aux_phase,
        "aux_minus_main": aux_minus_main,
        "main_minus_aux": main_minus_aux,
        "pair_seq": read_diag_u32(bus, args, 0x04),
        "pair_valid": read_diag_u32(bus, args, 0x05),
        "resolver_state": read_diag_u32(bus, args, 0x0B),
        "vernier_pos": read_diag_f32(bus, args, 0x08),
        "enc_pos": read_diag_f32(bus, args, 0x20),
        "enc_vel": read_diag_f32(bus, args, 0x21),
    }


def delta_phase(now, base, key):
    return wrap_pm_half(now[key] - base[key])


def print_sample(label, t, sample, base=None, target=None, total_base=None):
    if base is None:
        dpos = dmain = daux = ddiff = 0.0
    else:
        dpos = sample["enc_pos"] - base["enc_pos"]
        dmain = delta_phase(sample, base, "main_phase")
        daux = delta_phase(sample, base, "aux_phase")
        ddiff = delta_phase(sample, base, "aux_minus_main")
    if total_base is None:
        total_text = ""
    else:
        total_diff = delta_phase(sample, total_base, "aux_minus_main")
        total_main = delta_phase(sample, total_base, "main_phase")
        total_aux = delta_phase(sample, total_base, "aux_phase")
        total_text = (
            f" total:main={turns_to_deg(total_main):+.1f}deg"
            f" aux={turns_to_deg(total_aux):+.1f}deg"
            f" diff={turns_to_deg(total_diff):+.1f}deg"
        )

    err = "" if target is None else f" err={target - sample['pos']:+.6f}"
    hb = sample["hb"]
    hb_text = ""
    if hb is not None:
        hb_text = (
            f" state={hb['axis_state']}({AXIS_STATES.get(hb['axis_state'], '?')}) "
            f"axis_error={format_axis_error(hb['axis_error'])}"
        )

    print(
        f"{label:>10s} t={t:5.2f}s pos={sample['pos']:+.6f} "
        f"enc={sample['enc_pos']:+.6f} denc={dpos:+.6f}({turns_to_deg(dpos):+.1f}deg) "
        f"vel={sample['vel']:+.5f} Iq={sample['iq_set']:+.3f}/{sample['iq_meas']:+.3f} "
        f"main={sample['main']} dmain={dmain:+.6f}({turns_to_deg(dmain):+.1f}deg) "
        f"aux={sample['aux']} daux={daux:+.6f}({turns_to_deg(daux):+.1f}deg) "
        f"d(aux-main)={ddiff:+.6f}({turns_to_deg(ddiff):+.1f}deg) "
        f"vern={sample['vernier_pos']:+.6f} A-M={sample['aux_minus_main']:+.6f} "
        f"valid/state={sample['pair_valid']}/{sample['resolver_state']}{err}{total_text}{hb_text}"
    )


def require_ready(bus, args):
    hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=3.0)
    if hb is None:
        raise RuntimeError("No heartbeat")
    print(
        f"Heartbeat: state={hb['axis_state']} "
        f"axis_error={format_axis_error(hb['axis_error'])} raw={hb['raw']}"
    )
    status = get_status_ex(bus, args.node_id, args.extended_id, timeout=1.0)
    print(f"Status: {status}")
    if status is None or status["axis_error"] or not status["motor_calibrated"] or not status["encoder_ready"]:
        print_error_summary("Errors before abort:", read_error_summary(bus, args))
        raise RuntimeError("Axis is not ready")


def run_trace(bus, args):
    clear_errors(bus, args.node_id, args.extended_id)
    time.sleep(0.2)
    require_ready(bus, args)
    print_config(bus, args)

    print("Idle sensor trace. Physically mark/watch the output shaft during this section.")
    idle_base = snapshot(bus, args)
    print_sample("idle-base", 0.0, idle_base)
    start = time.monotonic()
    while time.monotonic() - start < args.idle_duration:
        time.sleep(args.sample_period)
        now = snapshot(bus, args)
        print_sample("idle", time.monotonic() - start, now, idle_base)

    home = snapshot(bus, args)["pos"]
    target = home + args.angle_deg / 360.0
    print(
        f"Closed-loop command: home={home:+.6f}, target={target:+.6f}, "
        f"delta={args.angle_deg:+.1f} deg output coordinate"
    )
    set_limits(bus, args.node_id, args.vel_limit, args.current_limit, args.extended_id)
    set_pos_gain(bus, args.node_id, args.pos_gain, args.extended_id)
    set_vel_gains(bus, args.node_id, args.vel_gain, args.vel_integrator_gain, args.extended_id)
    set_controller_modes(
        bus, args.node_id,
        CONTROL_MODE_POSITION_CONTROL,
        INPUT_MODE_PASSTHROUGH,
        args.extended_id,
    )
    set_input_pos(bus, args.node_id, home, 0.0, 0.0, args.extended_id)
    set_requested_state(bus, args.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, args.extended_id)
    time.sleep(0.2)

    base = snapshot(bus, args)
    print_sample("move-base", 0.0, base, target=target, total_base=idle_base)
    set_input_pos(bus, args.node_id, target, 0.0, 0.0, args.extended_id)
    start = time.monotonic()
    while time.monotonic() - start < args.move_duration:
        time.sleep(args.sample_period)
        now = snapshot(bus, args)
        print_sample("move", time.monotonic() - start, now, base, target=target, total_base=idle_base)
        hb = now["hb"]
        if hb and (hb["axis_error"] or hb["motor_error_flag"] or hb["encoder_error_flag"] or hb["controller_error_flag"]):
            print_error_summary("Errors:", read_error_summary(bus, args))
            raise RuntimeError("Error during trace")

    set_input_pos(bus, args.node_id, home, 0.0, 0.0, args.extended_id)
    time.sleep(args.return_duration)
    set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)


def main():
    parser = argparse.ArgumentParser(description="Trace the MT6826S vernier sensor-to-torque data chain.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--angle-deg", type=float, default=30.0)
    parser.add_argument("--idle-duration", type=float, default=2.0)
    parser.add_argument("--move-duration", type=float, default=2.0)
    parser.add_argument("--return-duration", type=float, default=0.5)
    parser.add_argument("--sample-period", type=float, default=0.2)
    parser.add_argument("--pos-gain", type=float, default=1.0)
    parser.add_argument("--vel-gain", type=float, default=1.0)
    parser.add_argument("--vel-integrator-gain", type=float, default=0.05)
    parser.add_argument("--vel-limit", type=float, default=0.2)
    parser.add_argument("--current-limit", type=float, default=3.0)
    parser.add_argument("--clear-at-end", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        run_trace(bus, args)
        if args.clear_at_end:
            time.sleep(0.2)
            clear_errors(bus, args.node_id, args.extended_id)
        print("PASS: full-chain trace completed")
        return 0
    except BaseException:
        print("Stopping motor due to failure...")
        try:
            set_input_pos(bus, args.node_id, 0.0, 0.0, 0.0, args.extended_id)
            set_requested_state(bus, args.node_id, AXIS_STATE_IDLE, args.extended_id)
            if args.clear_at_end:
                time.sleep(0.2)
                clear_errors(bus, args.node_id, args.extended_id)
        finally:
            raise
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
