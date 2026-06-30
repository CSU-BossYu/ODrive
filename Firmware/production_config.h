/*
 * Production profile for the trimmed ODrive v3 single-axis firmware.
 *
 * Keep this file C-compatible: it is included by both C and C++ firmware code.
 * Values can still be overridden from the build system with -D..., but the
 * defaults below define the production-safe profile used by this branch.
 */

#ifndef __ODRIVE_PRODUCTION_CONFIG_H
#define __ODRIVE_PRODUCTION_CONFIG_H

#ifndef ODRIVE_PRODUCTION_SINGLE_AXIS
#define ODRIVE_PRODUCTION_SINGLE_AXIS 1
#endif

#ifndef ODRIVE_PRODUCTION_CAN_BAUD
#define ODRIVE_PRODUCTION_CAN_BAUD 1000000UL
#endif

#ifndef ODRIVE_PRODUCTION_CAN_NODE_ID
#define ODRIVE_PRODUCTION_CAN_NODE_ID 0UL
#endif

#ifndef ODRIVE_PRODUCTION_MOTOR_CALIBRATION_CURRENT
#define ODRIVE_PRODUCTION_MOTOR_CALIBRATION_CURRENT 2.0f
#endif

#ifndef ODRIVE_PRODUCTION_MOTOR_CURRENT_LIMIT
#define ODRIVE_PRODUCTION_MOTOR_CURRENT_LIMIT 3.0f
#endif

#ifndef ODRIVE_PRODUCTION_AXIS_CALIBRATION_CURRENT
#define ODRIVE_PRODUCTION_AXIS_CALIBRATION_CURRENT 2.0f
#endif

#ifndef ODRIVE_PRODUCTION_PROFILE_VERSION
#define ODRIVE_PRODUCTION_PROFILE_VERSION 0x0001u
#endif

#endif /* __ODRIVE_PRODUCTION_CONFIG_H */
