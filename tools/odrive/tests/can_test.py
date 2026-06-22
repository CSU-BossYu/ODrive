
import test_runner

import struct
import can
import asyncio
import time
import math

from fibre.utils import Logger
from odrive.enums import *
from test_runner import *

# Each argument is described as tuple (name, format, scale).
# Struct format codes: https://docs.python.org/2/library/struct.html
command_set = {
    'heartbeat': (0x001, [('error', 'I', 1), ('current_state', 'B', 1), ('reserved', 'H', 1), ('controller_state', 'B', 1)]), # tested
    'estop': (0x002, []), # tested
    'get_motor_error': (0x003, [('motor_error', 'I', 1)]), # untested
    'get_encoder_error': (0x004, [('encoder_error', 'I', 1)]), # untested
    'get_sensorless_error': (0x005, [('sensorless_error', 'I', 1)]), # untested
    'set_node_id': (0x006, [('node_id', 'I', 1)]), # tested
    'set_requested_state': (0x007, [('requested_state', 'I', 1)]), # tested
    # 0x008 not yet implemented
    'get_encoder_estimates': (0x009, [('encoder_pos_estimate', 'f', 1), ('encoder_vel_estimate', 'f', 1)]), # partially tested
    'get_encoder_count': (0x00a, [('encoder_shadow_count', 'i', 1), ('encoder_count', 'i', 1)]), # partially tested
    'set_controller_modes': (0x00b, [('control_mode', 'i', 1), ('input_mode', 'i', 1)]), # tested
    'set_input_pos': (0x00c, [('input_pos', 'f', 1), ('vel_ff', 'h', 0.001), ('torque_ff', 'h', 0.001)]), # tested
    'set_input_vel': (0x00d, [('input_vel', 'f', 1), ('torque_ff', 'f', 1)]), # tested
    'set_input_torque': (0x00e, [('input_torque', 'f', 1)]), # tested
    'set_velocity_limit': (0x00f, [('velocity_limit', 'f', 1)]), # tested
    'start_anticogging': (0x010, []), # untested
    'set_traj_vel_limit': (0x011, [('traj_vel_limit', 'f', 1)]), # tested
    'set_traj_accel_limits': (0x012, [('traj_accel_limit', 'f', 1), ('traj_decel_limit', 'f', 1)]), # tested
    'set_traj_inertia': (0x013, [('inertia', 'f', 1)]), # tested
    'get_iq': (0x014, [('iq_setpoint', 'f', 1), ('iq_measured', 'f', 1)]), # untested
    'get_sensorless_estimates': (0x015, [('sensorless_pos_estimate', 'f', 1), ('sensorless_vel_estimate', 'f', 1)]), # untested
    'reboot': (0x016, []), # tested
    'get_vbus_voltage': (0x017, [('vbus_voltage', 'f', 1)]), # tested
    'clear_errors': (0x018, []), # partially tested
}

def command(bus, node_id_, extended_id, cmd_name, **kwargs):
    cmd_spec = command_set[cmd_name]
    cmd_id = cmd_spec[0]
    fmt = '<' + ''.join([f for (n, f, s) in cmd_spec[1]]) # all little endian

    if (sorted([n for (n, f, s) in cmd_spec[1]]) != sorted(kwargs.keys())):
        raise Exception("expected arguments: " + str([n for (n, f, s) in cmd_spec[1]]))

    fields = [((kwargs[n] / s) if f == 'f' else int(kwargs[n] / s)) for (n, f, s) in cmd_spec[1]]
    data = struct.pack(fmt, *fields)
    msg = can.Message(arbitration_id=((node_id_ << 5) | cmd_id), extended_id=extended_id, data=data)
    bus.send(msg)

async def record_messages(bus, node_id, extended_id, cmd_name, timeout = 5.0):
    """
    Returns an async generator that yields a dictionary for each CAN message that
    is received, provided that the CAN ID matches the expected value.
    """

    cmd_spec = command_set[cmd_name]
    cmd_id = cmd_spec[0]
    fmt = '<' + ''.join([f for (n, f, s) in cmd_spec[1]]) # all little endian

    reader = can.AsyncBufferedReader()
    notifier = can.Notifier(bus, [reader], timeout = timeout, loop = asyncio.get_event_loop())

    try:
        # The timeout in can.Notifier only triggers if no new messages are received at all,
        # so we need a second monitoring method.
        start = time.monotonic()
        while True:
            msg = await reader.get_message()
            if ((msg.arbitration_id == ((node_id << 5) | cmd_id)) and (msg.is_extended_id == extended_id) and not msg.is_remote_frame):
                fields = struct.unpack(fmt, msg.data[:(struct.calcsize(fmt))]) 
                res = {n: (fields[i] * s) for (i, (n, f, s)) in enumerate(cmd_spec[1])}
                res['t'] = time.monotonic()
                yield res
            if (time.monotonic() - start) > timeout:
                break
    finally:
        notifier.stop()

async def request(bus, node_id, extended_id, cmd_name, timeout = 1.0):
    cmd_spec = command_set[cmd_name]
    cmd_id = cmd_spec[0]

    msg_generator = record_messages(bus, node_id, extended_id, cmd_name, timeout)

    msg = can.Message(arbitration_id=((node_id << 5) | cmd_id), extended_id=extended_id, data=[], is_remote_frame=True)
    bus.send(msg)

    async for msg in msg_generator:
        return msg

    raise TimeoutError()

async def get_all(async_iterator):
    return [x async for x in async_iterator]


class TestSimpleCAN():
    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            can_interfaces = list(testrig.get_connected_components(odrive.can, CanInterfaceComponent))
            yield AnyTestCase(*[(odrive, intf, 0, False, tf) for intf, tf in can_interfaces]) # standard ID
            yield AnyTestCase(*[(odrive, intf, 0xfedcba, True, tf) for intf, tf in can_interfaces]) # extended ID

    def run_test(self, odrive: ODriveComponent, canbus: CanInterfaceComponent, node_id: int, extended_id: bool, logger: Logger):
        odrive.disable_mappings()
        if odrive.yaml['board-version'].startswith("v3."):
            odrive.handle.config.gpio15_mode = GPIO_MODE_CAN_A
            odrive.handle.config.gpio16_mode = GPIO_MODE_CAN_A
        elif odrive.yaml['board-version'].startswith("v4."):
            pass # CAN pin configuration is hardcoded
        else:
            raise Exception("unknown board version {}".format(odrive.yaml['board-version']))
        odrive.handle.config.enable_can_a = True
        odrive.save_config_and_reboot()

        axis = odrive.handle.axis0
        axis.config.enable_watchdog = False
        odrive.handle.clear_errors()
        axis.config.can.node_id = node_id
        axis.config.can.is_extended = extended_id
        time.sleep(0.1)
        
        def my_cmd(cmd_name, **kwargs): command(canbus.handle, node_id, extended_id, cmd_name, **kwargs)
        def my_req(cmd_name, **kwargs): return asyncio.run(request(canbus.handle, node_id, extended_id, cmd_name, **kwargs))
        def fence(): my_req('get_vbus_voltage') # fence to ensure the CAN command was sent
        def flush_rx():
            while not canbus.handle.recv(timeout = 0) is None: pass

        logger.debug('sending request...')
        test_assert_eq(my_req('get_vbus_voltage')['vbus_voltage'], odrive.handle.vbus_voltage, accuracy=0.01)

        my_cmd('set_node_id', node_id=node_id+20)
        time.sleep(0.1) # TODO: remove this hack (see note in firmware)
        asyncio.run(request(canbus.handle, node_id+20, extended_id, 'get_vbus_voltage'))
        test_assert_eq(axis.config.can.node_id, node_id+20)

        # Reset node ID to default value
        command(canbus.handle, node_id+20, extended_id, 'set_node_id', node_id=node_id)
        fence()
        test_assert_eq(axis.config.can.node_id, node_id)

        # Check that extended node IDs are not carelessly projected to 6-bit IDs
        extended_id = not extended_id
        my_cmd('estop') # should not be accepted
        extended_id = not extended_id
        fence()
        test_assert_eq(axis.error, AXIS_ERROR_NONE)

        axis.encoder.set_linear_count(123)
        flush_rx() # drop previous encoder estimates that were sent by the ODrive at a constant rate
        test_assert_eq(my_req('get_encoder_estimates')['encoder_pos_estimate'], 123.0 / axis.encoder.config.cpr, accuracy=0.01)
        test_assert_eq(my_req('get_encoder_count')['encoder_shadow_count'], 123.0, accuracy=0.01)

        my_cmd('clear_errors')
        fence()
        test_assert_eq(axis.error, 0)

        my_cmd('estop')
        fence()
        test_assert_eq(axis.error, AXIS_ERROR_ESTOP_REQUESTED)

        my_cmd('set_requested_state', requested_state=42) # illegal state - should assert axis error
        fence()
        test_assert_eq(axis.current_state, 1) # idle
        test_assert_eq(axis.error, AXIS_ERROR_ESTOP_REQUESTED | AXIS_ERROR_INVALID_STATE)

        my_cmd('clear_errors')
        fence()
        test_assert_eq(axis.error, 0)

        my_cmd('set_controller_modes', control_mode=1, input_mode=5) # current conrol, traprzoidal trajectory
        fence()
        test_assert_eq(axis.controller.config.control_mode, 1)
        test_assert_eq(axis.controller.config.input_mode, 5)

        # Reset to safe values
        my_cmd('set_controller_modes', control_mode=3, input_mode=1) # position control, passthrough
        fence()
        test_assert_eq(axis.controller.config.control_mode, 3)
        test_assert_eq(axis.controller.config.input_mode, 1)

        axis.controller.input_pos = 1234
        axis.controller.input_vel = 1234
        axis.controller.input_torque = 1234
        my_cmd('set_input_pos', input_pos=1.23, vel_ff=1.2, torque_ff=3.4)
        fence()
        test_assert_eq(axis.controller.input_pos, 1.23, range=0.1)
        test_assert_eq(axis.controller.input_vel, 1.2, range=0.01)
        test_assert_eq(axis.controller.input_torque, 3.4, range=0.001)

        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        my_cmd('set_input_vel', input_vel=-10.5, torque_ff=0.1234)
        fence()
        test_assert_eq(axis.controller.input_vel, -10.5, range=0.01)
        test_assert_eq(axis.controller.input_torque, 0.1234, range=0.01)

        axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        my_cmd('set_input_torque', input_torque=0.1)
        fence()
        test_assert_eq(axis.controller.input_torque, 0.1, range=0.01)

        my_cmd('set_velocity_limit', velocity_limit=2.345678)
        fence()
        test_assert_eq(axis.controller.config.vel_limit, 2.345678, range=0.001)

        my_cmd('set_traj_vel_limit', traj_vel_limit=123.456)
        fence()
        test_assert_eq(axis.trap_traj.config.vel_limit, 123.456, range=0.0001)

        my_cmd('set_traj_accel_limits', traj_accel_limit=98.231, traj_decel_limit=-12.234)
        fence()
        test_assert_eq(axis.trap_traj.config.accel_limit, 98.231, range=0.0001)
        test_assert_eq(axis.trap_traj.config.decel_limit, -12.234, range=0.0001)

        my_cmd('set_traj_inertia', inertia=55.086)
        fence()
        test_assert_eq(axis.controller.config.inertia, 55.086, range=0.0001)

        # any CAN cmd will feed the watchdog
        test_watchdog(axis, lambda: my_cmd('set_input_torque', input_torque=0.0), logger)

        logger.debug('testing heartbeat...')
        flush_rx() # Flush RX buffer to get a clean state for the heartbeat test
        heartbeats = asyncio.run(get_all(record_messages(canbus.handle, node_id, extended_id, 'heartbeat', timeout = 2.0)))
        test_assert_eq(len(heartbeats), 2.0 / 0.1, accuracy=0.05)
        test_assert_eq([msg['error'] for msg in heartbeats], [AXIS_ERROR_WATCHDOG_TIMER_EXPIRED] * len(heartbeats))
        test_assert_eq([msg['current_state'] for msg in heartbeats], [1] * len(heartbeats))

        logger.debug('testing reboot...')
        test_assert_eq(odrive.handle._on_lost.done(), False)
        my_cmd('reboot')
        time.sleep(0.5)
        if not odrive.handle._on_lost is None:
            raise TestFailed("device didn't seem to reboot")
        odrive.handle = None
        time.sleep(2.0)
        odrive.prepare(logger)

# Extended Command helpers (CMD 0x1E)
EXTENDED_CMD_ID = 0x01E

def extended_command(bus, node_id, extended_id, sub_cmd, flags=0, value=0, param2=0, param3=0, value_float=None):
    """
    Send an extended command (CMD 0x1E) to the ODrive.
    Payload: byte0=sub_cmd, byte1=item/param, byte2=type for SET_BASIC_CONFIG,
    byte3=reserved, byte4-7=value (uint32 LE or float32 LE)
    If value_float is not None, it overrides value with a float32 encoding.
    """
    if value_float is not None:
        data = struct.pack('<BBBBf', sub_cmd, flags, param2, param3, value_float)
    else:
        data = struct.pack('<BBBBi', sub_cmd, flags, param2, param3, value)
    msg = can.Message(arbitration_id=((node_id << 5) | EXTENDED_CMD_ID), extended_id=extended_id, data=data)
    bus.send(msg)

async def extended_request(bus, node_id, extended_id, sub_cmd, flags=0, value=0, param2=0, param3=0, timeout=1.0, value_float=None):
    """
    Send an extended command and wait for the response on the same CAN ID.
    Returns dict with keys: sub_cmd, item, status, type, value, value_float.
    Raw response bytes 1..3 are also available as byte1, byte2, byte3.
    """
    reader = can.AsyncBufferedReader()
    notifier = can.Notifier(bus, [reader], timeout=timeout, loop=asyncio.get_event_loop())

    try:
        extended_command(bus, node_id, extended_id, sub_cmd, flags, value, param2, param3, value_float)

        start = time.monotonic()
        while True:
            msg = await reader.get_message()
            expected_id = ((node_id << 5) | EXTENDED_CMD_ID)
            if (msg.arbitration_id == expected_id and msg.is_extended_id == extended_id and not msg.is_remote_frame):
                sub_cmd_r, byte1_r, byte2_r, byte3_r, value_r = struct.unpack('<BBBBi', msg.data[:8])
                if sub_cmd_r == sub_cmd:  # match sub_cmd
                    return {
                        'sub_cmd': sub_cmd_r,
                        'byte1': byte1_r,
                        'byte2': byte2_r,
                        'byte3': byte3_r,
                        'item': byte1_r,
                        'status': byte2_r,
                        'type': byte3_r,
                        'param2': byte2_r,  # compatibility alias
                        'param3': byte3_r,  # compatibility alias
                        'value': value_r,
                        'value_float': struct.unpack('<f', msg.data[4:8])[0],
                    }
            if (time.monotonic() - start) > timeout:
                raise TimeoutError(f"extended_request sub_cmd=0x{sub_cmd:02X} timed out")
    finally:
        notifier.stop()


class TestExtendedCAN():
    """Tests for Extended CAN Command (CMD 0x1E)"""

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            can_interfaces = list(testrig.get_connected_components(odrive.can, CanInterfaceComponent))
            yield AnyTestCase(*[(odrive, intf, 0, False, tf) for intf, tf in can_interfaces])
            yield AnyTestCase(*[(odrive, intf, 0xfedcba, True, tf) for intf, tf in can_interfaces])

    def run_test(self, odrive: ODriveComponent, canbus: CanInterfaceComponent, node_id: int, extended_id: bool, logger: Logger):
        odrive.disable_mappings()
        if odrive.yaml['board-version'].startswith("v3."):
            odrive.handle.config.gpio15_mode = GPIO_MODE_CAN_A
            odrive.handle.config.gpio16_mode = GPIO_MODE_CAN_A
        elif odrive.yaml['board-version'].startswith("v4."):
            pass
        else:
            raise Exception("unknown board version {}".format(odrive.yaml['board-version']))
        odrive.handle.config.enable_can_a = True
        odrive.save_config_and_reboot()

        axis = odrive.handle.axis0
        axis.config.enable_watchdog = False
        odrive.handle.clear_errors()
        axis.config.can.node_id = node_id
        axis.config.can.is_extended = extended_id
        time.sleep(0.1)

        def ext_cmd(sub_cmd, flags=0, value=0, param2=0, param3=0):
            extended_command(canbus.handle, node_id, extended_id, sub_cmd, flags, value, param2, param3)

        def ext_req(sub_cmd, flags=0, value=0, param2=0, param3=0, value_float=None, **kwargs):
            return asyncio.run(extended_request(canbus.handle, node_id, extended_id, sub_cmd, flags, value, param2, param3,
                                                value_float=value_float, **kwargs))

        # -----------------------------------------------------------
        # Test 1: GET_AXIS_STATUS_EX (sub_cmd 0x01)
        # -----------------------------------------------------------
        logger.debug('Testing GET_AXIS_STATUS_EX...')
        status_ex = ext_req(0x01, timeout=2.0)
        test_assert_eq(status_ex['sub_cmd'], 0x01)
        test_assert_eq(status_ex['byte1'], 0x00)  # OK

        # Verify axis.current_state
        test_assert_eq(status_ex['byte2'], axis.current_state)

        # Verify flags
        flags = status_ex['byte3']
        test_assert_eq(bool(flags & 0x01), axis.motor.is_calibrated)
        test_assert_eq(bool(flags & 0x02), axis.encoder.is_ready)
        test_assert_eq(bool(flags & 0x04), axis.motor.error != 0)
        test_assert_eq(bool(flags & 0x08), axis.encoder.error != 0)
        test_assert_eq(bool(flags & 0x10), axis.controller.error != 0)

        # Verify axis.error
        test_assert_eq(status_ex['value'], axis.error)

        # -----------------------------------------------------------
        # Test 2: SET_PRECALIBRATED (sub_cmd 0x02)
        # -----------------------------------------------------------
        logger.debug('Testing SET_PRECALIBRATED...')

        # Save original values to restore later
        orig_motor_pre_cal = axis.motor.config.pre_calibrated
        orig_encoder_pre_cal = axis.encoder.config.pre_calibrated

        try:
            # Test clear both
            resp = ext_req(0x02, flags=0x30)  # bit4=clear motor, bit5=clear encoder
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            test_assert_eq(axis.motor.config.pre_calibrated, False)
            test_assert_eq(axis.motor.is_calibrated, False)
            test_assert_eq(axis.encoder.config.pre_calibrated, False)
            test_assert_eq(axis.encoder.is_ready, False)

            # Test set both
            resp = ext_req(0x02, flags=0x03)  # bit0=set motor, bit1=set encoder
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            test_assert_eq(axis.motor.config.pre_calibrated, True)
            test_assert_eq(axis.motor.is_calibrated, True)
            test_assert_eq(axis.encoder.config.pre_calibrated, True)
            test_assert_eq(axis.encoder.is_ready, True)

            # Test invalid flags (conflicting set+clear)
            resp = ext_req(0x02, flags=0x11)  # bit0=set motor, bit4=clear motor (conflict)
            test_assert_eq(resp['status'], 0x01)  # invalid_flags
            test_assert_eq(axis.motor.config.pre_calibrated, True)
            test_assert_eq(axis.motor.is_calibrated, True)

            # Test empty flags
            resp = ext_req(0x02, flags=0x00)
            test_assert_eq(resp['status'], 0x01)  # invalid_flags
            test_assert_eq(axis.motor.config.pre_calibrated, True)
            test_assert_eq(axis.encoder.config.pre_calibrated, True)

        finally:
            # Restore original state
            restore_flags = 0
            restore_flags |= 0x01 if orig_motor_pre_cal else 0x10
            restore_flags |= 0x02 if orig_encoder_pre_cal else 0x20
            ext_req(0x02, flags=restore_flags)

        # -----------------------------------------------------------
        # Test 3: GET_CALIB_RESULT (sub_cmd 0x04)
        # -----------------------------------------------------------
        logger.debug('Testing GET_CALIB_RESULT...')

        # Save originals and set known values for testing
        orig_phase_resistance = axis.motor.config.phase_resistance
        orig_phase_inductance = axis.motor.config.phase_inductance
        orig_phase_offset = axis.encoder.config.phase_offset
        orig_direction = axis.encoder.config.direction

        axis.motor.config.phase_resistance = 1.234
        axis.motor.config.phase_inductance = 5.678e-5
        axis.encoder.config.phase_offset = 12345
        axis.encoder.config.direction = -1
        time.sleep(0.05)

        # Test phase_resistance (item_id=0x01, float32)
        resp = ext_req(0x04, flags=0x01)
        test_assert_eq(resp['status'], 0x00)
        test_assert_eq(resp['type'], 1)  # type = float32
        test_assert_eq(resp['value_float'], 1.234, accuracy=0.001)

        # Test phase_inductance (item_id=0x02, float32)
        resp = ext_req(0x04, flags=0x02)
        test_assert_eq(resp['status'], 0x00)
        test_assert_eq(resp['type'], 1)  # type = float32
        test_assert_eq(resp['value_float'], 5.678e-5, accuracy=1e-7)

        # Test phase_offset (item_id=0x03, int32)
        resp = ext_req(0x04, flags=0x03)
        test_assert_eq(resp['status'], 0x00)
        test_assert_eq(resp['type'], 2)  # type = int32
        test_assert_eq(resp['value'], 12345)

        # Test direction (item_id=0x04, int32)
        resp = ext_req(0x04, flags=0x04)
        test_assert_eq(resp['status'], 0x00)
        test_assert_eq(resp['type'], 2)  # type = int32
        test_assert_eq(resp['value'], -1)

        # Test unknown item
        resp = ext_req(0x04, flags=0xFF)
        test_assert_eq(resp['status'], 0x01)  # unknown_item

        # Restore original values
        axis.motor.config.phase_resistance = orig_phase_resistance
        axis.motor.config.phase_inductance = orig_phase_inductance
        axis.encoder.config.phase_offset = orig_phase_offset
        axis.encoder.config.direction = orig_direction

        # -----------------------------------------------------------
        # Test 4: SAVE_CONFIGURATION (sub_cmd 0x03)
        # -----------------------------------------------------------
        logger.debug('Testing SAVE_CONFIGURATION...')

        # Set a known pre_calibrated value
        axis.motor.config.pre_calibrated = True
        axis.encoder.config.pre_calibrated = True
        time.sleep(0.05)

        # Send save command (device will reboot)
        try:
            resp = ext_req(0x03, timeout=2.0)
            # The ACK should be received before reboot, but if the device reboots
            # fast enough, we might not get it. Either way is acceptable.
            if resp is not None:
                test_assert_eq(resp['sub_cmd'], 0x03)
                test_assert_eq(resp['status'], 0x00)
        except TimeoutError:
            logger.debug('SAVE_CONFIGURATION: ACK not received (device rebooted before TX) - this is expected')

        # Wait for reboot
        time.sleep(3.0)
        odrive.handle = None
        time.sleep(2.0)
        odrive.prepare(logger)
        axis = odrive.handle.axis0

        # Verify pre_calibrated was persisted
        test_assert_eq(axis.motor.config.pre_calibrated, True)
        test_assert_eq(axis.encoder.config.pre_calibrated, True)

        # Restore persisted pre_calibrated values so this test doesn't leave
        # the device with a different saved configuration.
        if (orig_motor_pre_cal != True) or (orig_encoder_pre_cal != True):
            axis.motor.config.pre_calibrated = orig_motor_pre_cal
            axis.encoder.config.pre_calibrated = orig_encoder_pre_cal
            odrive.save_config_and_reboot()
            axis = odrive.handle.axis0

        # Re-configure CAN
        axis.config.can.node_id = node_id
        axis.config.can.is_extended = extended_id
        axis.config.enable_watchdog = False
        time.sleep(0.1)

        # -----------------------------------------------------------
        # Test 5: Compatibility regression
        # -----------------------------------------------------------
        logger.debug('Testing compatibility: existing CAN commands...')

        def my_cmd(cmd_name, **kwargs):
            command(canbus.handle, node_id, extended_id, cmd_name, **kwargs)

        def my_req(cmd_name, **kwargs):
            return asyncio.run(request(canbus.handle, node_id, extended_id, cmd_name, **kwargs))

        # Heartbeat
        heartbeats = asyncio.run(get_all(record_messages(canbus.handle, node_id, extended_id, 'heartbeat', timeout=2.0)))
        test_assert_eq(len(heartbeats), 2.0 / 0.1, accuracy=0.5)
        logger.debug(f'  Got {len(heartbeats)} heartbeats')

        # Set requested state
        my_cmd('set_requested_state', requested_state=AXIS_STATE_IDLE)
        time.sleep(0.1)
        test_assert_eq(axis.current_state, AXIS_STATE_IDLE)

        # Get encoder estimates
        resp = my_req('get_encoder_estimates')
        logger.debug(f'  Encoder estimates: pos={resp["encoder_pos_estimate"]:.4f}, vel={resp["encoder_vel_estimate"]:.4f}')

        # Clear errors
        my_cmd('clear_errors')
        time.sleep(0.05)

        # -----------------------------------------------------------
        # Test 6: GET_DEVICE_INFO (sub_cmd 0x05)
        # -----------------------------------------------------------
        logger.debug('Testing GET_DEVICE_INFO...')

        # Protocol version
        resp = ext_req(0x05, flags=0x01)
        test_assert_eq(resp['status'], 0x00)
        test_assert_eq(resp['type'], 3)  # type = uint32
        test_assert_eq(resp['value'], 0x00000100)  # v1.0

        # FW version
        resp = ext_req(0x05, flags=0x02)
        test_assert_eq(resp['status'], 0x00)
        fw = resp['value']
        logger.debug(f'  FW version: {(fw>>24)&0xFF}.{(fw>>16)&0xFF}.{(fw>>8)&0xFF}.{fw&0xFF}')

        # HW version
        resp = ext_req(0x05, flags=0x03)
        test_assert_eq(resp['status'], 0x00)
        hw = resp['value']
        logger.debug(f'  HW version: {(hw>>16)&0xFF}.{(hw>>8)&0xFF}.{hw&0xFF}')

        # Serial number
        resp_lo = ext_req(0x05, flags=0x04)
        resp_hi = ext_req(0x05, flags=0x05)
        test_assert_eq(resp_lo['status'], 0x00)
        test_assert_eq(resp_hi['status'], 0x00)
        serial = (resp_hi['value'] << 32) | resp_lo['value']
        logger.debug(f'  Serial: {serial:#018x}')

        # Unknown item
        resp = ext_req(0x05, flags=0xFF)
        test_assert_eq(resp['status'], 0x01)  # unknown

        # -----------------------------------------------------------
        # Test 7: SET_BASIC_CONFIG / GET_BASIC_CONFIG (sub_cmd 0x06/0x07)
        # -----------------------------------------------------------
        logger.debug('Testing SET_BASIC_CONFIG / GET_BASIC_CONFIG...')

        # Save originals
        orig_motor_type = axis.motor.config.motor_type
        orig_pole_pairs = axis.motor.config.pole_pairs
        orig_calib_current = axis.motor.config.calibration_current
        orig_current_lim = axis.motor.config.current_lim
        orig_encoder_mode = axis.encoder.config.mode
        orig_encoder_cpr = axis.encoder.config.cpr

        try:
            # --- SET motor_type (uint32) ---
            resp = ext_req(0x07, flags=0x10, param2=3, value=0)  # type=uint32, value=HIGH_CURRENT
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            resp = ext_req(0x06, flags=0x10)
            test_assert_eq(resp['status'], 0x00)
            test_assert_eq(resp['type'], 3)  # type=uint32
            test_assert_eq(resp['value'], 0)  # HIGH_CURRENT

            # --- SET pole_pairs (int32) ---
            resp = ext_req(0x07, flags=0x11, param2=2, value=21)  # type=int32
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            test_assert_eq(axis.motor.config.pole_pairs, 21)

            # --- SET calibration_current (float32) ---
            resp = ext_req(0x07, flags=0x12, param2=1, value_float=5.0)  # type=float32
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            test_assert_eq(axis.motor.config.calibration_current, 5.0, accuracy=0.01)

            # --- SET current_lim (float32) ---
            resp = ext_req(0x07, flags=0x14, param2=1, value_float=15.0)  # type=float32
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            test_assert_eq(axis.motor.config.current_lim, 15.0, accuracy=0.01)

            # --- SET encoder cpr (int32) ---
            resp = ext_req(0x07, flags=0x21, param2=2, value=8192)  # type=int32
            test_assert_eq(resp['status'], 0x00)
            time.sleep(0.05)
            test_assert_eq(axis.encoder.config.cpr, 8192)

            # --- GET readback ---
            resp = ext_req(0x06, flags=0x15)  # torque_constant (float32)
            test_assert_eq(resp['status'], 0x00)
            test_assert_eq(resp['type'], 1)  # type=float32

            # --- invalid type ---
            resp = ext_req(0x07, flags=0x11, param2=1, value_float=0.0)  # pole_pairs expects int32, sending float32
            test_assert_eq(resp['status'], 0x03)  # invalid_type

            # --- invalid value ---
            resp = ext_req(0x07, flags=0x11, param2=2, value=0)  # pole_pairs must be > 0
            test_assert_eq(resp['status'], 0x04)  # invalid_value

            # --- unknown param ---
            resp = ext_req(0x07, flags=0xFF, param2=3, value=0)
            test_assert_eq(resp['status'], 0x01)  # unknown_param

        finally:
            # Restore originals
            axis.motor.config.motor_type = orig_motor_type
            axis.motor.config.pole_pairs = orig_pole_pairs
            axis.motor.config.calibration_current = orig_calib_current
            axis.motor.config.current_lim = orig_current_lim
            axis.encoder.config.mode = orig_encoder_mode
            axis.encoder.config.cpr = orig_encoder_cpr

        logger.debug('All extended CAN tests passed!')


tests = [TestSimpleCAN(), TestExtendedCAN()]

if __name__ == '__main__':
    test_runner.run(tests)
