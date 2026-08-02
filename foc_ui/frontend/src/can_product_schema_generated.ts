// Generated from docs/can_product_protocol_schema.json; do not edit.
export const CAN_PRODUCT_PROTOCOL_VERSION = 0x0000010C as const
export enum ProductCanMessageId {
  NMT = 0x00,
  HEARTBEAT = 0x01,
  ESTOP = 0x02,
  GET_MOTOR_ERROR = 0x03,
  GET_ENCODER_ERROR = 0x04,
  COMMAND_ACK = 0x05,
  SET_AXIS_NODE_ID = 0x06,
  SET_AXIS_STATE = 0x07,
  MANAGEMENT_COMMAND = 0x08,
  ENCODER_ESTIMATES = 0x09,
  GET_ENCODER_COUNT = 0x0A,
  SET_CONTROLLER_MODE = 0x0B,
  SET_INPUT_POS = 0x0C,
  SET_INPUT_VEL = 0x0D,
  SET_INPUT_TORQUE = 0x0E,
  SET_LIMITS = 0x0F,
  PRODUCT_STATUS = 0x10,
  SET_TRAJ_VEL_LIMIT = 0x11,
  SET_TRAJ_ACCEL_LIMITS = 0x12,
  SET_TRAJ_INERTIA = 0x13,
  IQ = 0x14,
  REBOOT = 0x16,
  BUS_VOLTAGE_CURRENT = 0x17,
  CLEAR_ERRORS = 0x18,
  SET_LINEAR_COUNT = 0x19,
  SET_POS_GAIN = 0x1A,
  SET_VEL_GAINS = 0x1B,
  GET_ADC_VOLTAGE = 0x1C,
  GET_CONTROLLER_ERROR = 0x1D,
  LEGACY_EXTENDED = 0x1E,
  SET_MIT_CONTROL = 0x1F,
}
export interface CanCommandAck {
  requestId: number
  status: number
  stateEpochLow: number
  reason: number
}
