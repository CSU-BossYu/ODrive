#!/usr/bin/env python3
"""Read raw dual MT6826S angles through CANSimple vernier diagnostics.

Requires firmware with encoder.mode = MODE_SPI_ABS_MT6826S_VERNIER (0x106).
This first-stage tool reads raw pair diagnostics only; resolver fields are
placeholders until the vernier resolver is connected to the control loop.
"""
import argparse
import csv
import time

from hil_bootstrap import ensure_test_support
ensure_test_support()

from odrive_hil.can_simple import (
    EXT_STATUS,
    EXT_TYPE_FLOAT32,
    EXT_TYPE_UINT32,
    clear_errors,
    ext_request,
    get_basic_config,
    get_status_ex,
    open_bus,
    set_requested_state,
    wait_heartbeat,
)


ENCODER_MODE_SPI_ABS_MT6826S_VERNIER = 0x106
MT6826S_CPR = 32768
SUB_CMD_VERNIER_DIAG = 0x0A

ITEM_MAIN_ANGLE = 0x00
ITEM_AUX_ANGLE = 0x01
ITEM_MAIN_VALID = 0x02
ITEM_AUX_VALID = 0x03
ITEM_PAIR_SEQUENCE = 0x04
ITEM_PAIR_VALID = 0x05
ITEM_RESOLVER_RESIDUAL = 0x06
ITEM_VERNIER_VIRTUAL_COUNT = 0x07
ITEM_VERNIER_POSITION_RAD = 0x08
ITEM_MAIN_ERROR_COUNT = 0x09
ITEM_AUX_ERROR_COUNT = 0x0A
ITEM_RESOLVER_STATE = 0x0B
ITEM_PAIR_ERROR_COUNT = 0x0C
ITEM_CONTROL_ISR_CYCLES = 0x0D
ITEM_CONTROL_ISR_MAX_CYCLES = 0x0E
ITEM_CONTROL_ISR_COUNT = 0x0F
ITEM_MAIN_RAW = 0x10
ITEM_AUX_RAW = 0x11
ITEM_MAIN_CRC = 0x12
ITEM_AUX_CRC = 0x13
ITEM_MAIN_DMA_ERROR_COUNT = 0x14
ITEM_MAIN_CRC_ERROR_COUNT = 0x15
ITEM_MAIN_FIXED_BIT_ERROR_COUNT = 0x16
ITEM_MAIN_STATUS_WARNING_COUNT = 0x17
ITEM_MAIN_SAMPLE_COUNT = 0x18
ITEM_AUX_DMA_ERROR_COUNT = 0x19
ITEM_AUX_CRC_ERROR_COUNT = 0x1A
ITEM_AUX_FIXED_BIT_ERROR_COUNT = 0x1B
ITEM_AUX_STATUS_WARNING_COUNT = 0x1C
ITEM_AUX_SAMPLE_COUNT = 0x1D
ITEM_MAIN_SEQUENCE = 0x1E
ITEM_AUX_SEQUENCE = 0x1F
ITEM_ENCODER_POS_ESTIMATE = 0x20
ITEM_ENCODER_VEL_ESTIMATE = 0x21
ITEM_ENCODER_POS_CIRCULAR = 0x22
ITEM_PAIR_TRANSACTION_CYCLES = 0x2E
ITEM_PAIR_TRANSACTION_MAX_CYCLES = 0x2F
ITEM_CONTROL_LOOP_CYCLES = 0x31
ITEM_CONTROL_LOOP_MAX_CYCLES = 0x32
ITEM_CONTROL_LOOP_COUNT = 0x33
ITEM_CONTROL_LOOP_OVER_50 = 0x39
ITEM_CONTROL_LOOP_OVER_70 = 0x3A
ITEM_CONTROL_LOOP_OVER_85 = 0x3B
ITEM_CYCLE_COUNTER_HZ = 0x3C
ITEM_MAIN_SAMPLE_AGE_CYCLES = 0x3D
ITEM_MAIN_SAMPLE_AGE_MAX_CYCLES = 0x3E
ITEM_PLL_ERROR_RAD = 0x80
ITEM_LUT_CORRECTION_RAD = 0x81
ITEM_READINESS_FLAGS = 0x82
ITEM_CHAIN_FAULT = 0x74
ITEM_VERNIER_BRANCH = 0x75
ITEM_RUNTIME_UNIQUE_RANGE = 0x76
ITEM_RAW_MAIN_PHASE_RAD = 0x77
ITEM_RAW_AUX_PHASE_RAD = 0x78
ITEM_UNIQUE_POSITION_RAD = 0x79
ITEM_WRAPPED_OUTPUT_PHASE_RAD = 0x7A
ITEM_OUTPUT_POSITION_RAD = 0x7B
ITEM_OUTPUT_VELOCITY_RPM = 0x7C
ITEM_RESOLVER_RESIDUAL_RAD = 0x7D
ITEM_RESOLVER_MARGIN_RAD = 0x7E
ITEM_LUT_FLAGS = 0x7F

DEBUG_FIELD_NAMES = {
    "main_raw", "aux_raw", "main_crc", "aux_crc",
    "main_dma_error_count", "main_crc_error_count",
    "main_fixed_bit_error_count", "main_status_warning_count",
    "main_sample_count", "aux_dma_error_count",
    "aux_crc_error_count", "aux_fixed_bit_error_count",
    "aux_status_warning_count", "aux_sample_count",
    "main_sequence", "aux_sequence",
    "encoder_pos_estimate", "encoder_vel_estimate", "encoder_pos_circular",
    "pair_transaction_cycles", "pair_transaction_max_cycles",
    "control_loop_cycles", "control_loop_max_cycles", "control_loop_count",
    "control_loop_over_50", "control_loop_over_70", "control_loop_over_85",
    "cycle_counter_hz", "main_sample_age_cycles", "main_sample_age_max_cycles",
    "control_isr_cycles", "control_isr_max_cycles", "control_isr_count",
}


def read_vernier_item(bus, node_id, item_id, extended_id=False, timeout=1.0):
    return ext_request(
        bus, node_id, SUB_CMD_VERNIER_DIAG,
        item=item_id, req_type=EXT_TYPE_UINT32, value=0,
        extended_id=extended_id, timeout=timeout,
    )


def read_item_value(bus, args, item_id, want_float=False):
    resp = read_vernier_item(bus, args.node_id, item_id, args.extended_id, args.timeout)
    if resp is None:
        return None, None
    status = EXT_STATUS.get(resp["status"], resp["status"])
    if resp["status"] != 0:
        return None, status
    return (resp["value_f"] if want_float else resp["value_u"]), status


def read_full_pair(bus, args):
    data = {}
    for item_id, name in [
        (ITEM_MAIN_ANGLE, "main_angle"),
        (ITEM_AUX_ANGLE, "aux_angle"),
        (ITEM_MAIN_VALID, "main_valid"),
        (ITEM_AUX_VALID, "aux_valid"),
        (ITEM_PAIR_SEQUENCE, "pair_sequence"),
        (ITEM_PAIR_VALID, "pair_valid"),
        (ITEM_VERNIER_VIRTUAL_COUNT, "vernier_virtual_count"),
        (ITEM_MAIN_ERROR_COUNT, "main_error_count"),
        (ITEM_AUX_ERROR_COUNT, "aux_error_count"),
        (ITEM_RESOLVER_STATE, "resolver_state"),
        (ITEM_PAIR_ERROR_COUNT, "pair_error_count"),
        (ITEM_CONTROL_ISR_CYCLES, "control_isr_cycles"),
        (ITEM_CONTROL_ISR_MAX_CYCLES, "control_isr_max_cycles"),
        (ITEM_CONTROL_ISR_COUNT, "control_isr_count"),
        (ITEM_MAIN_RAW, "main_raw"),
        (ITEM_AUX_RAW, "aux_raw"),
        (ITEM_MAIN_CRC, "main_crc"),
        (ITEM_AUX_CRC, "aux_crc"),
        (ITEM_MAIN_DMA_ERROR_COUNT, "main_dma_error_count"),
        (ITEM_MAIN_CRC_ERROR_COUNT, "main_crc_error_count"),
        (ITEM_MAIN_FIXED_BIT_ERROR_COUNT, "main_fixed_bit_error_count"),
        (ITEM_MAIN_STATUS_WARNING_COUNT, "main_status_warning_count"),
        (ITEM_MAIN_SAMPLE_COUNT, "main_sample_count"),
        (ITEM_AUX_DMA_ERROR_COUNT, "aux_dma_error_count"),
        (ITEM_AUX_CRC_ERROR_COUNT, "aux_crc_error_count"),
        (ITEM_AUX_FIXED_BIT_ERROR_COUNT, "aux_fixed_bit_error_count"),
        (ITEM_AUX_STATUS_WARNING_COUNT, "aux_status_warning_count"),
        (ITEM_AUX_SAMPLE_COUNT, "aux_sample_count"),
        (ITEM_MAIN_SEQUENCE, "main_sequence"),
        (ITEM_AUX_SEQUENCE, "aux_sequence"),
        (0x30, "dbg_foc_bad_timing_cnt"),
        (0x34, "dbg_cl_adc_fail_pre_cnt"),
        (0x35, "dbg_cl_adc_fail_post_cnt"),
        (0x36, "dbg_cl_deadline_miss_cnt"),
        (0x37, "dbg_enc_pair_busy_cnt"),
        (0x38, "dbg_enc_pair_ok_cnt"),
        (ITEM_PAIR_TRANSACTION_CYCLES, "pair_transaction_cycles"),
        (ITEM_PAIR_TRANSACTION_MAX_CYCLES, "pair_transaction_max_cycles"),
        (ITEM_CONTROL_LOOP_CYCLES, "control_loop_cycles"),
        (ITEM_CONTROL_LOOP_MAX_CYCLES, "control_loop_max_cycles"),
        (ITEM_CONTROL_LOOP_COUNT, "control_loop_count"),
        (ITEM_CONTROL_LOOP_OVER_50, "control_loop_over_50"),
        (ITEM_CONTROL_LOOP_OVER_70, "control_loop_over_70"),
        (ITEM_CONTROL_LOOP_OVER_85, "control_loop_over_85"),
        (ITEM_CYCLE_COUNTER_HZ, "cycle_counter_hz"),
        (ITEM_MAIN_SAMPLE_AGE_CYCLES, "main_sample_age_cycles"),
        (ITEM_MAIN_SAMPLE_AGE_MAX_CYCLES, "main_sample_age_max_cycles"),
        (ITEM_READINESS_FLAGS, "readiness_flags"),
        (ITEM_CHAIN_FAULT, "chain_fault"),
        (ITEM_VERNIER_BRANCH, "vernier_branch"),
        (ITEM_RUNTIME_UNIQUE_RANGE, "runtime_unique_range"),
        (ITEM_LUT_FLAGS, "lut_flags"),
    ]:
        data[name], _ = read_item_value(bus, args, item_id)

    for item_id, name in [
        (ITEM_RESOLVER_RESIDUAL, "resolver_residual_rad_compat"),
        (ITEM_VERNIER_POSITION_RAD, "vernier_position_rad"),
        (ITEM_ENCODER_POS_ESTIMATE, "encoder_pos_estimate"),
        (ITEM_ENCODER_VEL_ESTIMATE, "encoder_vel_estimate"),
        (ITEM_ENCODER_POS_CIRCULAR, "encoder_pos_circular"),
        (ITEM_PLL_ERROR_RAD, "pll_error_rad"),
        (ITEM_LUT_CORRECTION_RAD, "lut_correction_rad"),
        (ITEM_RAW_MAIN_PHASE_RAD, "raw_main_phase_rad"),
        (ITEM_RAW_AUX_PHASE_RAD, "raw_aux_phase_rad"),
        (ITEM_UNIQUE_POSITION_RAD, "unique_position_rad"),
        (ITEM_WRAPPED_OUTPUT_PHASE_RAD, "wrapped_output_phase_rad"),
        (ITEM_OUTPUT_POSITION_RAD, "output_position_rad"),
        (ITEM_OUTPUT_VELOCITY_RPM, "output_velocity_rpm"),
        (ITEM_RESOLVER_RESIDUAL_RAD, "resolver_residual_rad"),
        (ITEM_RESOLVER_MARGIN_RAD, "resolver_margin_rad"),
    ]:
        data[name], _ = read_item_value(bus, args, item_id, want_float=True)

    return data


def angle_to_turns(angle, cpr):
    return angle / cpr


def angle_to_deg(angle, cpr):
    return angle / cpr * 360.0


def fmt_raw(value):
    if value is None:
        return "?? ?? ?? ??"
    return " ".join(f"{(value >> shift) & 0xff:02X}" for shift in range(0, 32, 8))


def fmt_crc(value):
    if value is None:
        return "rx=?? calc=??"
    return f"rx={(value & 0xff):02X} calc={((value >> 8) & 0xff):02X}"


def fmt_hex(value, width):
    return "" if value is None else f"{value:0{width}X}"


def fmt_float(value, digits=6):
    return "nan" if value is None else f"{value:.{digits}f}"


def print_config_value(bus, args, param_id, name):
    resp = get_basic_config(bus, args.node_id, param_id, args.extended_id)
    if resp is None:
        print(f"  {name}: TIMEOUT")
        return None
    status = EXT_STATUS.get(resp["status"], resp["status"])
    value = resp["value_f"] if resp["type"] == EXT_TYPE_FLOAT32 else resp["value_i"]
    print(f"  {name}: {value} status={status} raw={resp['raw']}")
    return value if resp["status"] == 0 else None


def print_sample(data, cpr):
    if data.get("main_angle") is None:
        print("TIMEOUT: no response from vernier diagnostics")
        return

    missing = [
        key for key, value in data.items()
        if value is None and key not in DEBUG_FIELD_NAMES
    ]
    if missing:
        print(f"PARTIAL: missing {','.join(missing)}")
        return

    main = data["main_angle"]
    aux = data["aux_angle"]
    print(
        f"seq={data['pair_sequence']:6d} pair={'V' if data['pair_valid'] else 'X'} "
        f"main={main:5d} ({angle_to_turns(main, cpr):.6f}t {angle_to_deg(main, cpr):7.3f}deg) "
        f"M{'V' if data['main_valid'] else 'X'} "
        f"aux={aux:5d} ({angle_to_turns(aux, cpr):.6f}t {angle_to_deg(aux, cpr):7.3f}deg) "
        f"A{'V' if data['aux_valid'] else 'X'} "
        f"diff={main - aux:+6d} "
        f"out={data['vernier_position_rad']:.6f}rad "
        f"vcnt={data['vernier_virtual_count']} "
        f"res={data['resolver_residual_rad_compat']:+.6f}rad "
        f"pos={fmt_float(data.get('output_position_rad'))}rad "
        f"wrap={fmt_float(data.get('wrapped_output_phase_rad'))}rad "
        f"vel={fmt_float(data.get('output_velocity_rpm'))}rpm "
        f"branch={data.get('vernier_branch')} range={data.get('runtime_unique_range')} "
        f"vres={fmt_float(data.get('resolver_residual_rad'))}rad "
        f"margin={fmt_float(data.get('resolver_margin_rad'))}rad "
        f"pllerr={fmt_float(data.get('pll_error_rad'))}rad "
        f"lut=0x{data.get('lut_flags', 0):02X}/"
        f"{fmt_float(data.get('lut_correction_rad'))}rad "
        f"ready=0x{data.get('readiness_flags', 0):02X} "
        f"chain=0x{data.get('chain_fault', 0):08X} "
        f"state={data['resolver_state']} "
        f"err M={data['main_error_count']} A={data['aux_error_count']} P={data['pair_error_count']} "
        f"rawM=[{fmt_raw(data['main_raw'])}] crcM={fmt_crc(data['main_crc'])} "
        f"rawA=[{fmt_raw(data['aux_raw'])}] crcA={fmt_crc(data['aux_crc'])} "
        f"splitM(dma/crc/fix/stat/ok)={data.get('main_dma_error_count')}/"
        f"{data.get('main_crc_error_count')}/{data.get('main_fixed_bit_error_count')}/"
        f"{data.get('main_status_warning_count')}/{data.get('main_sample_count')} "
        f"splitA(dma/crc/fix/stat/ok)={data.get('aux_dma_error_count')}/"
        f"{data.get('aux_crc_error_count')}/{data.get('aux_fixed_bit_error_count')}/"
        f"{data.get('aux_status_warning_count')}/{data.get('aux_sample_count')} "
        f"seqM/A={data.get('main_sequence')}/{data.get('aux_sequence')}"
    )
    # Debug counters
    dbg = [
        f"FOC_BT={data.get('dbg_foc_bad_timing_cnt')}",
        f"ADCpre={data.get('dbg_cl_adc_fail_pre_cnt')}",
        f"ADCpost={data.get('dbg_cl_adc_fail_post_cnt')}",
        f"DLmiss={data.get('dbg_cl_deadline_miss_cnt')}",
        f"pairBsy={data.get('dbg_enc_pair_busy_cnt')}",
        f"pairOk={data.get('dbg_enc_pair_ok_cnt')}",
        f"crcM={data.get('main_crc_error_count')}",
        f"crcA={data.get('aux_crc_error_count')}",
    ]
    print(f"  DEBUG: {' '.join(dbg)}")
    cycle_hz = data.get("cycle_counter_hz")
    if cycle_hz:
        to_us = lambda cycles: None if cycles is None else cycles * 1e6 / cycle_hz
        print(
            "  TIMING: "
            f"loop={to_us(data.get('control_loop_cycles')):.2f}us "
            f"loopMax={to_us(data.get('control_loop_max_cycles')):.2f}us "
            f"isr={to_us(data.get('control_isr_cycles')):.2f}us "
            f"isrMax={to_us(data.get('control_isr_max_cycles')):.2f}us "
            f"pair={to_us(data.get('pair_transaction_cycles')):.2f}us "
            f"pairMax={to_us(data.get('pair_transaction_max_cycles')):.2f}us "
            f"age={to_us(data.get('main_sample_age_cycles')):.2f}us "
            f"ageMax={to_us(data.get('main_sample_age_max_cycles')):.2f}us "
            f"budgetCross(50/70/85)={data.get('control_loop_over_50')}/"
            f"{data.get('control_loop_over_70')}/{data.get('control_loop_over_85')}"
        )


def main():
    parser = argparse.ArgumentParser(description="Read raw dual MT6826S pair angles.")
    parser.add_argument("--channel", default="PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--node-id", type=int, default=0)
    parser.add_argument("--extended-id", action="store_true")
    parser.add_argument("--cpr", type=int, default=MT6826S_CPR)
    parser.add_argument("--period", type=float, default=0.1)
    parser.add_argument("--samples", type=int, default=0, help="0 = forever")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--clear-at-start", action="store_true")
    parser.add_argument("--request-state", type=int, help="Request an axis state before sampling")
    parser.add_argument("--post-request-delay", type=float, default=0.2)
    parser.add_argument("--csv", help="Write CSV to this path")
    parser.add_argument("--show-config", action="store_true")
    parser.add_argument("--show-status", action="store_true")
    args = parser.parse_args()

    csv_file = None
    bus = open_bus(args.channel, args.bitrate)
    try:
        print(f"Opened {bus.channel_info}")
        hb = wait_heartbeat(bus, args.node_id, args.extended_id, timeout=5.0)
        if hb is None:
            raise RuntimeError("No heartbeat received")
        print(f"Heartbeat: state={hb['axis_state']} axis_error=0x{hb['axis_error']:08X} raw={hb['raw']}")
        if args.show_status:
            print(f"Status_ex: {get_status_ex(bus, args.node_id, args.extended_id, timeout=args.timeout)}")

        if args.clear_at_start:
            clear_errors(bus, args.node_id, args.extended_id)
            time.sleep(0.2)

        if args.request_state is not None:
            set_requested_state(bus, args.node_id, args.request_state, args.extended_id)
            time.sleep(args.post_request_delay)

        if args.show_config:
            print("Encoder config:")
            mode = print_config_value(bus, args, 0x20, "mode")
            cpr = print_config_value(bus, args, 0x21, "cpr")
            print_config_value(bus, args, 0x22, "main_cs")
            print_config_value(bus, args, 0x24, "aux_cs")
            print_config_value(bus, args, 0x25, "vernier_virtual_cpr")
            print_config_value(bus, args, 0x26, "vernier_main_ratio")
            print_config_value(bus, args, 0x27, "vernier_aux_ratio")
            print_config_value(bus, args, 0x2A, "vernier_main_reversed")
            print_config_value(bus, args, 0x2B, "vernier_aux_reversed")
            print_config_value(bus, args, 0x33, "vernier_output_reversed")
            print_config_value(bus, args, 0x2C, "mt6826s_spi_mode")
            print_config_value(bus, args, 0x2F, "mt6826s_spi_prescaler")
            if mode is not None and int(mode) != ENCODER_MODE_SPI_ABS_MT6826S_VERNIER:
                print(f"  WARN: expected mode 0x{ENCODER_MODE_SPI_ABS_MT6826S_VERNIER:03X}")
            if cpr is not None:
                args.cpr = int(cpr)

        csv_writer = None
        if args.csv:
            csv_file = open(args.csv, "w", newline="")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "timestamp", "pair_sequence", "pair_valid",
                "main_angle", "main_valid", "aux_angle", "aux_valid",
                "main_turns", "aux_turns", "diff_counts",
                "vernier_position_rad", "virtual_count", "resolver_residual_rad", "resolver_state",
                "encoder_pos_rad", "encoder_velocity_rpm", "encoder_pos_circular_rad",
                "raw_main_phase_rad", "raw_aux_phase_rad", "unique_position_rad",
                "wrapped_output_phase_rad", "output_position_rad", "output_velocity_rpm",
                "vernier_branch", "runtime_unique_range", "resolver_residual_rad",
                "resolver_margin_rad", "pll_error_rad", "lut_correction_rad",
                "readiness_flags", "chain_fault", "lut_flags",
                "main_error_count", "aux_error_count", "pair_error_count",
                "main_raw", "aux_raw", "main_crc", "aux_crc",
                "main_dma_error_count", "main_crc_error_count",
                "main_fixed_bit_error_count", "main_status_warning_count",
                "main_sample_count", "aux_dma_error_count",
                "aux_crc_error_count", "aux_fixed_bit_error_count",
                "aux_status_warning_count", "aux_sample_count",
                "pair_transaction_cycles", "pair_transaction_max_cycles",
                "control_loop_cycles", "control_loop_max_cycles", "control_loop_count",
                "control_loop_over_50", "control_loop_over_70", "control_loop_over_85",
                "cycle_counter_hz", "main_sample_age_cycles", "main_sample_age_max_cycles",
                "control_isr_cycles", "control_isr_max_cycles", "control_isr_count",
                "dbg_foc_bad_timing_cnt", "dbg_cl_adc_fail_pre_cnt",
                "dbg_cl_adc_fail_post_cnt", "dbg_cl_deadline_miss_cnt",
                "dbg_enc_pair_busy_cnt", "dbg_enc_pair_ok_cnt",
            ])
            print(f"CSV logging to {args.csv}")

        n = 0
        while args.samples == 0 or n < args.samples:
            t0 = time.monotonic()
            data = read_full_pair(bus, args)
            print_sample(data, args.cpr)

            if csv_writer and data.get("main_angle") is not None and data.get("aux_angle") is not None:
                csv_writer.writerow([
                    time.time(), data["pair_sequence"], data["pair_valid"],
                    data["main_angle"], data["main_valid"],
                    data["aux_angle"], data["aux_valid"],
                    angle_to_turns(data["main_angle"], args.cpr),
                    angle_to_turns(data["aux_angle"], args.cpr),
                    data["main_angle"] - data["aux_angle"],
                    data["vernier_position_rad"], data["vernier_virtual_count"],
                    data["resolver_residual_rad_compat"], data["resolver_state"],
                    data["encoder_pos_estimate"], data["encoder_vel_estimate"],
                    data["encoder_pos_circular"],
                    data["raw_main_phase_rad"], data["raw_aux_phase_rad"],
                    data["unique_position_rad"], data["wrapped_output_phase_rad"],
                    data["output_position_rad"], data["output_velocity_rpm"],
                    data["vernier_branch"], data["runtime_unique_range"],
                    data["resolver_residual_rad"], data["resolver_margin_rad"],
                    data["pll_error_rad"], data["lut_correction_rad"],
                    data["readiness_flags"], data["chain_fault"], data["lut_flags"],
                    data["main_error_count"], data["aux_error_count"],
                    data["pair_error_count"],
                    fmt_hex(data["main_raw"], 8), fmt_hex(data["aux_raw"], 8),
                    fmt_hex(data["main_crc"], 4), fmt_hex(data["aux_crc"], 4),
                    data["main_dma_error_count"], data["main_crc_error_count"],
                    data["main_fixed_bit_error_count"], data["main_status_warning_count"],
                    data["main_sample_count"], data["aux_dma_error_count"],
                    data["aux_crc_error_count"], data["aux_fixed_bit_error_count"],
                    data["aux_status_warning_count"], data["aux_sample_count"],
                    data.get("pair_transaction_cycles"), data.get("pair_transaction_max_cycles"),
                    data.get("control_loop_cycles"), data.get("control_loop_max_cycles"),
                    data.get("control_loop_count"), data.get("control_loop_over_50"),
                    data.get("control_loop_over_70"), data.get("control_loop_over_85"),
                    data.get("cycle_counter_hz"), data.get("main_sample_age_cycles"),
                    data.get("main_sample_age_max_cycles"),
                    data.get("control_isr_cycles"), data.get("control_isr_max_cycles"),
                    data.get("control_isr_count"),
                    data.get("dbg_foc_bad_timing_cnt"), data.get("dbg_cl_adc_fail_pre_cnt"),
                    data.get("dbg_cl_adc_fail_post_cnt"), data.get("dbg_cl_deadline_miss_cnt"),
                    data.get("dbg_enc_pair_busy_cnt"), data.get("dbg_enc_pair_ok_cnt"),
                ])
                csv_file.flush()

            n += 1
            sleep_time = args.period - (time.monotonic() - t0)
            if sleep_time > 0 and (args.samples == 0 or n < args.samples):
                time.sleep(sleep_time)

        return 0
    finally:
        if csv_file:
            csv_file.close()
            print(f"CSV saved to {args.csv}")
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
