#!/usr/bin/env python3
"""Read raw dual MT6826S angles through CANSimple vernier diagnostics.

Requires firmware with encoder.mode = MODE_SPI_ABS_MT6826S_VERNIER (0x106).
This first-stage tool reads raw pair diagnostics only; resolver fields are
placeholders until the vernier resolver is connected to the control loop.
"""
import argparse
import csv
import time

from common import (
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
ITEM_VERNIER_POSITION_TURNS = 0x08
ITEM_MAIN_ERROR_COUNT = 0x09
ITEM_AUX_ERROR_COUNT = 0x0A
ITEM_RESOLVER_STATE = 0x0B
ITEM_PAIR_ERROR_COUNT = 0x0C
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

DEBUG_FIELD_NAMES = {
    "main_raw", "aux_raw", "main_crc", "aux_crc",
    "main_dma_error_count", "main_crc_error_count",
    "main_fixed_bit_error_count", "main_status_warning_count",
    "main_sample_count", "aux_dma_error_count",
    "aux_crc_error_count", "aux_fixed_bit_error_count",
    "aux_status_warning_count", "aux_sample_count",
    "main_sequence", "aux_sequence",
    "encoder_pos_estimate", "encoder_vel_estimate", "encoder_pos_circular",
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
        (0x31, "dbg_foc_bad_timing_delta"),
        (0x32, "dbg_foc_bad_timing_i_ts"),
        (0x33, "dbg_foc_bad_timing_ctrl_ts"),
        (0x34, "dbg_cl_adc_fail_pre_cnt"),
        (0x35, "dbg_cl_adc_fail_post_cnt"),
        (0x36, "dbg_cl_deadline_miss_cnt"),
        (0x37, "dbg_enc_pair_busy_cnt"),
        (0x38, "dbg_enc_pair_ok_cnt"),
        (0x39, "dbg_loop_alive_cnt"),
        (0x3A, "dbg_adc1_jeoc_fail"),
        (0x3B, "dbg_adc2_eoc_fail"),
        (0x3C, "dbg_adc2_jeoc_fail"),
        (0x3D, "dbg_adc3_eoc_fail"),
        (0x3E, "dbg_adc3_jeoc_fail"),
        (0x3F, "dbg_m0_adc1_jdr"),
        (0x40, "dbg_m0_adc2_jdr"),
        (0x41, "dbg_m0_adc3_jdr"),
        (0x42, "dbg_m0_current_sample_valid"),
        (0x46, "dbg_tim1_bdtr"),
        (0x47, "dbg_tim1_ccr1"),
        (0x48, "dbg_tim1_ccr2"),
        (0x49, "dbg_tim1_ccr3"),
        (0x4A, "dbg_m0_is_armed"),
        (0x4F, "dbg_m0_cm_current_present"),
        (0x50, "dbg_m0_cm_dc_calib_valid"),
        (0x51, "dbg_m0_cm_current_meas_valid"),
        (0x52, "dbg_m0_cm_armed_state"),
    ]:
        data[name], _ = read_item_value(bus, args, item_id)

    for item_id, name in [
        (ITEM_RESOLVER_RESIDUAL, "resolver_residual"),
        (ITEM_VERNIER_POSITION_TURNS, "vernier_position_turns"),
        (ITEM_ENCODER_POS_ESTIMATE, "encoder_pos_estimate"),
        (ITEM_ENCODER_VEL_ESTIMATE, "encoder_vel_estimate"),
        (ITEM_ENCODER_POS_CIRCULAR, "encoder_pos_circular"),
        (0x43, "dbg_m0_current_phA"),
        (0x44, "dbg_m0_current_phB"),
        (0x45, "dbg_m0_current_phC"),
        (0x4B, "dbg_resistance_actual_current"),
        (0x4C, "dbg_resistance_test_voltage"),
        (0x4D, "dbg_resistance_i_beta"),
        (0x4E, "dbg_resistance_test_mod"),
        (0x53, "dbg_m0_dc_calib_running_since"),
        (0x54, "dbg_m0_dc_calib_phA"),
        (0x55, "dbg_m0_dc_calib_phB"),
        (0x56, "dbg_m0_dc_calib_phC"),
        (0x57, "dbg_m0_cm_phA"),
        (0x58, "dbg_m0_cm_phB"),
        (0x59, "dbg_m0_cm_phC"),
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
        f"out={data['vernier_position_turns']:.6f}t "
        f"vcnt={data['vernier_virtual_count']} "
        f"res={data['resolver_residual']:+.6f} "
        f"enc={fmt_float(data.get('encoder_pos_estimate'))}t "
        f"vel={fmt_float(data.get('encoder_vel_estimate'))}t/s "
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
        f"loop={data.get('dbg_loop_alive_cnt')}",
    ]
    # Show BAD_TIMING details if any
    if data.get("dbg_foc_bad_timing_cnt"):
        dbg.append(
            f"BTdelta={data.get('dbg_foc_bad_timing_delta')}|"
            f"{data.get('dbg_foc_bad_timing_i_ts')}|"
            f"{data.get('dbg_foc_bad_timing_ctrl_ts')}"
        )
    print(f"  DEBUG: {' '.join(dbg)}")
    # Per-ADC flag breakdown
    adc_flag = [
        f"A1j={data.get('dbg_adc1_jeoc_fail')}",
        f"A2e={data.get('dbg_adc2_eoc_fail')}",
        f"A2j={data.get('dbg_adc2_jeoc_fail')}",
        f"A3e={data.get('dbg_adc3_eoc_fail')}",
        f"A3j={data.get('dbg_adc3_jeoc_fail')}",
    ]
    print(f"  ADC_FLAGS: {' '.join(adc_flag)}")
    m0_diag = [
        f"JDR={data.get('dbg_m0_adc1_jdr')}/{data.get('dbg_m0_adc2_jdr')}/{data.get('dbg_m0_adc3_jdr')}",
        f"Ivalid={data.get('dbg_m0_current_sample_valid')}",
        f"Iabc={fmt_float(data.get('dbg_m0_current_phA'), 3)}/"
        f"{fmt_float(data.get('dbg_m0_current_phB'), 3)}/"
        f"{fmt_float(data.get('dbg_m0_current_phC'), 3)}",
        f"TIM1=BDTR:{fmt_hex(data.get('dbg_tim1_bdtr'), 4)} "
        f"CCR:{data.get('dbg_tim1_ccr1')}/{data.get('dbg_tim1_ccr2')}/{data.get('dbg_tim1_ccr3')}",
        f"armed={data.get('dbg_m0_is_armed')}",
        f"Rcal=I:{fmt_float(data.get('dbg_resistance_actual_current'), 3)} "
        f"V:{fmt_float(data.get('dbg_resistance_test_voltage'), 3)} "
        f"Ib:{fmt_float(data.get('dbg_resistance_i_beta'), 3)} "
        f"mod:{fmt_float(data.get('dbg_resistance_test_mod'), 4)}",
        f"CM=present:{data.get('dbg_m0_cm_current_present')} "
        f"dcok:{data.get('dbg_m0_cm_dc_calib_valid')} "
        f"valid:{data.get('dbg_m0_cm_current_meas_valid')} "
        f"ast:{data.get('dbg_m0_cm_armed_state')}",
        f"DC=t:{fmt_float(data.get('dbg_m0_dc_calib_running_since'), 3)} "
        f"abc:{fmt_float(data.get('dbg_m0_dc_calib_phA'), 3)}/"
        f"{fmt_float(data.get('dbg_m0_dc_calib_phB'), 3)}/"
        f"{fmt_float(data.get('dbg_m0_dc_calib_phC'), 3)}",
        f"CMabc={fmt_float(data.get('dbg_m0_cm_phA'), 3)}/"
        f"{fmt_float(data.get('dbg_m0_cm_phB'), 3)}/"
        f"{fmt_float(data.get('dbg_m0_cm_phC'), 3)}",
    ]
    print(f"  M0_DIAG: {' '.join(m0_diag)}")


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
                "output_turns", "virtual_count", "resolver_residual", "resolver_state",
                "encoder_pos_estimate", "encoder_vel_estimate", "encoder_pos_circular",
                "main_error_count", "aux_error_count", "pair_error_count",
                "main_raw", "aux_raw", "main_crc", "aux_crc",
                "main_dma_error_count", "main_crc_error_count",
                "main_fixed_bit_error_count", "main_status_warning_count",
                "main_sample_count", "aux_dma_error_count",
                "aux_crc_error_count", "aux_fixed_bit_error_count",
                "aux_status_warning_count", "aux_sample_count",
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
                    data["vernier_position_turns"], data["vernier_virtual_count"],
                    data["resolver_residual"], data["resolver_state"],
                    data["encoder_pos_estimate"], data["encoder_vel_estimate"],
                    data["encoder_pos_circular"],
                    data["main_error_count"], data["aux_error_count"],
                    data["pair_error_count"],
                    fmt_hex(data["main_raw"], 8), fmt_hex(data["aux_raw"], 8),
                    fmt_hex(data["main_crc"], 4), fmt_hex(data["aux_crc"], 4),
                    data["main_dma_error_count"], data["main_crc_error_count"],
                    data["main_fixed_bit_error_count"], data["main_status_warning_count"],
                    data["main_sample_count"], data["aux_dma_error_count"],
                    data["aux_crc_error_count"], data["aux_fixed_bit_error_count"],
                    data["aux_status_warning_count"], data["aux_sample_count"],
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
