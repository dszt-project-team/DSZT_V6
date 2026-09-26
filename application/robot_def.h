#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>

typedef enum
{
  ROBOT_MODE_LOCKED = 0,
  ROBOT_MODE_MANUAL = 1,
  ROBOT_MODE_AUTO_FC = 2
} RobotMode;

typedef enum
{
  ROBOT_GATE_READY = 0,
  ROBOT_GATE_RC_LOST,
  ROBOT_GATE_STARTUP_LOCK_REQUIRED,
  ROBOT_GATE_MODE_CONFIRMING,
  ROBOT_GATE_THROTTLE_CENTERING
} RobotGate;

typedef struct
{
  int16_t steering_permille;
  int16_t throttle_permille;
  uint16_t speed_limit_erpm;
  uint8_t mode;
  uint8_t gate;
  uint8_t rc_online;
  uint8_t failsafe;
  uint8_t throttle_centered;
  uint32_t frame_count;
  uint32_t rc_error_count;
  uint16_t rc_pulse_us[16]; /* Last snapshot; meaningful only when rc_online && !failsafe. */
  uint16_t steering_filtered_us;
} RobotCommand;

typedef struct
{
  int32_t left_target_erpm;
  int32_t right_target_erpm;
  int32_t left_speed_erpm;
  int32_t right_speed_erpm;
  uint16_t left_fault;
  uint16_t right_fault;
  uint16_t steer_raw[2];
  int16_t steer_actual_x10[2];
  int16_t steer_target_x10[2];
  int16_t steer_duty[2];
  uint8_t steer_healthy[2];
  uint8_t steer_zero_valid[2];
  uint8_t steer_brake[2];
  uint8_t steer_reverse[2];
  uint8_t steer_calibration_side;
  uint8_t steer_released;
  uint8_t steer_fault;
  uint16_t fc_drive_raw_us;
  uint16_t fc_drive_filtered_us;
  uint16_t fc_steer_raw_us;
  uint16_t fc_steer_filtered_us;
  uint16_t left_timeout_count;
  uint16_t right_timeout_count;
  uint16_t left_crc_error_count;
  uint16_t right_crc_error_count;
  uint16_t rs485_overflow_count;
  uint16_t rs485_uart_error_count;
  uint32_t left_status_age_ms;
  uint32_t right_status_age_ms;
  uint32_t left_heartbeat_age_ms;
  uint32_t right_heartbeat_age_ms;
  uint32_t status_preempt_count;
  uint16_t oid_read_mode[2];
  int32_t oid_read_target[2];
  uint32_t oid_read_age[2];
  uint16_t oid_read_timeouts[2];
  uint16_t oid_zero_writes[2];
  uint16_t oid_speed_acks[2];
  uint16_t oid_write_unconfirmed[2];
  uint16_t oid_exceptions[2];
  uint8_t oid_last_exception[2];
  uint8_t left_online;
  uint8_t right_online;
  uint8_t fc_drive_online;
  uint8_t fc_steer_online;
  uint8_t fc_drive_fault;
  uint8_t fc_steer_fault;
  uint8_t fc_release_ready;
  uint8_t oid_safety_state;
  uint8_t oid_command_pending;
  uint8_t oid_command_step;
  uint8_t oid_command_urgent;
  uint8_t oid_stop_fault; /* 旧诊断字段保留为0，不再参与行走许可。 */
  uint8_t oid_reverse_wait;
  uint8_t oid_reverse_zero_pairs;
  uint8_t oid_read_active;
  int32_t oid_requested_base_erpm;
  uint32_t oid_heartbeat_max_gap[2];
  int32_t oid_tx_target[2];
  uint32_t oid_tx_ms[2];
  uint8_t parameters_confirmed;
  uint8_t motion_enabled;
  int16_t steering_scheduled_permille;
  uint16_t steering_speed_gain_permille;
} RobotChassisState;

extern RobotCommand g_robot_command;
extern RobotChassisState g_robot_chassis;

#endif
