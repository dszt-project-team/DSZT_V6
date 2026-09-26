#include "chassis_app.h"

#include "bsp_rs485.h"
#include "dual_steer.h"
#include "chassis_config.h"
#include "oid_stop_guard.h"
#include "oid_reverse_guard.h"
#include "front_drive.h"
#include "pwm_input.h"
#include "robot_def.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static BSP_RS485_Bus_t s_oid_bus;
static FrontDrive_Handle_t s_front_drive;
static uint32_t s_last_control_ms;
static OidStopGuard s_stop_guard;
static OidReverseGuard s_reverse_guard;
static PwmInput_Handle_t *s_fc_drive_input;
static PwmInput_Handle_t *s_fc_steer_input;
static PwmInput_Snapshot_t s_fc_drive;
static PwmInput_Snapshot_t s_fc_steer;
static uint32_t s_fc_center_start_ms;
static uint8_t s_fc_release_ready;

static int16_t ChassisApp_MapAsymmetricPulse(uint16_t pulse_us,
                                             uint16_t low_full_us,
                                             uint16_t neutral_us,
                                             uint16_t high_full_us,
                                             uint16_t deadband_us)
{
  int32_t value;

  if ((pulse_us >= (uint16_t)(neutral_us - deadband_us)) &&
      (pulse_us <= (uint16_t)(neutral_us + deadband_us)))
  {
    return 0;
  }
  if (pulse_us < neutral_us)
  {
    if (pulse_us <= low_full_us)
    {
      return -1000;
    }
    value = -((int32_t)((neutral_us - deadband_us) - pulse_us) * 1000) /
            (int32_t)((neutral_us - deadband_us) - low_full_us);
  }
  else
  {
    if (pulse_us >= high_full_us)
    {
      return 1000;
    }
    value = ((int32_t)pulse_us - (int32_t)(neutral_us + deadband_us)) * 1000 /
            (int32_t)(high_full_us - (neutral_us + deadband_us));
  }
  if (value < -1000)
  {
    value = -1000;
  }
  if (value > 1000)
  {
    value = 1000;
  }
  return (int16_t)value;
}

static int16_t ChassisApp_GetFcDrivePermille(void)
{
  return ChassisApp_MapAsymmetricPulse(
      s_fc_drive.filtered_us,
      CHASSIS_FC_DRIVE_REVERSE_FULL_US,
      CHASSIS_FC_DRIVE_NEUTRAL_US,
      CHASSIS_FC_DRIVE_FORWARD_FULL_US,
      CHASSIS_FC_DRIVE_DEADBAND_US);
}

static int16_t ChassisApp_GetFcSteerPermille(void)
{
  return ChassisApp_MapAsymmetricPulse(
      s_fc_steer.filtered_us,
      CHASSIS_FC_STEER_LEFT_FULL_US,
      CHASSIS_FC_STEER_NEUTRAL_US,
      CHASSIS_FC_STEER_RIGHT_FULL_US,
      CHASSIS_FC_STEER_DEADBAND_US);
}

static uint8_t ChassisApp_FcInputsHealthy(void)
{
  return (uint8_t)(((s_fc_drive.online != 0U) &&
                    (s_fc_drive.fault == PWM_INPUT_FAULT_NONE) &&
                    (s_fc_steer.online != 0U) &&
                    (s_fc_steer.fault == PWM_INPUT_FAULT_NONE)) ? 1U : 0U);
}

static void ChassisApp_UpdateFcRelease(uint32_t now_ms)
{
  uint8_t centered;

  if ((CHASSIS_FC_CONTROL_ENABLE == 0U) ||
      (g_robot_command.mode != ROBOT_MODE_AUTO_FC) ||
      (ChassisApp_FcInputsHealthy() == 0U))
  {
    s_fc_center_start_ms = 0U;
    s_fc_release_ready = 0U;
    return;
  }
  if (s_fc_release_ready != 0U)
  {
    return;
  }

  centered = (uint8_t)(((ChassisApp_GetFcDrivePermille() == 0) &&
                         (ChassisApp_GetFcSteerPermille() == 0)) ? 1U : 0U);
  if (centered == 0U)
  {
    s_fc_center_start_ms = 0U;
    s_fc_release_ready = 0U;
    return;
  }
  if (s_fc_center_start_ms == 0U)
  {
    s_fc_center_start_ms = now_ms;
  }
  if ((now_ms - s_fc_center_start_ms) >= CHASSIS_FC_RELEASE_CENTER_MS)
  {
    s_fc_release_ready = 1U;
  }
}

static void ChassisApp_CalculateTargets(int16_t steering_permille,
                                        int16_t throttle_permille,
                                        uint16_t speed_limit_erpm,
                                        int32_t *left_erpm, int32_t *right_erpm)
{
  float left_angle, right_angle, left_scale, right_scale;
  int32_t base = (int32_t)throttle_permille * speed_limit_erpm / 1000;
  DualSteer_Geometry(steering_permille, &left_angle, &right_angle, &left_scale, &right_scale);
  *left_erpm = (int32_t)((float)base * left_scale);
  *right_erpm = (int32_t)((float)base * right_scale);
}

void ChassisApp_Init(void)
{
  PwmInput_Config_t pwm_config;

  DualSteer_Init();
  memset(&s_stop_guard, 0, sizeof(s_stop_guard));
  memset(&s_reverse_guard, 0, sizeof(s_reverse_guard));

  memset(&pwm_config, 0, sizeof(pwm_config));
  pwm_config.timer = &htim3;
  pwm_config.minimum_valid_us = CHASSIS_FC_PWM_MIN_VALID_US;
  pwm_config.maximum_valid_us = CHASSIS_FC_PWM_MAX_VALID_US;
  pwm_config.timeout_ms = CHASSIS_FC_PWM_TIMEOUT_MS;
  pwm_config.transient_fault_hold_ms = CHASSIS_FC_PWM_TRANSIENT_HOLD_MS;
  pwm_config.average_window = CHASSIS_FC_DRIVE_AVERAGE_WINDOW;
  pwm_config.valid_samples_to_online = CHASSIS_FC_PWM_VALID_TO_ONLINE;
  pwm_config.channel = TIM_CHANNEL_3;
  s_fc_drive_input = PwmInput_Register(&pwm_config);
  pwm_config.average_window = CHASSIS_FC_STEER_AVERAGE_WINDOW;
  pwm_config.channel = TIM_CHANNEL_4;
  s_fc_steer_input = PwmInput_Register(&pwm_config);
  memset(&s_fc_drive, 0, sizeof(s_fc_drive));
  memset(&s_fc_steer, 0, sizeof(s_fc_steer));
  s_fc_drive.fault = PWM_INPUT_FAULT_NOT_READY;
  s_fc_steer.fault = PWM_INPUT_FAULT_NOT_READY;

  BSP_RS485_Init(&s_oid_bus, &huart3, 0, 0U);
  FrontDrive_Init(&s_front_drive, &s_oid_bus,
                  CHASSIS_OID_LEFT_ID, CHASSIS_OID_RIGHT_ID,
                  CHASSIS_OID_MOTOR_POLE_PAIRS);
  s_front_drive.control_read_enabled = CHASSIS_OID_READBACK_ENABLE;
  g_robot_chassis.parameters_confirmed = CHASSIS_PARAMETERS_CONFIRMED;
}

void ChassisApp_Task(uint32_t now_ms)
{
  const OID_ESC_Status_t *left_status;
  const OID_ESC_Status_t *right_status;
  int32_t left_target = 0;
  int32_t right_target = 0;
  uint8_t permit_motion;
  uint8_t source_ready;
  uint8_t rearm_centered;
  int16_t active_steering = 0;
  int16_t active_throttle = 0;
  uint16_t active_speed_limit = CHASSIS_FC_SPEED_LIMIT_ERPM;
  uint8_t diagnostic_side;

  /* Decide before servicing the bus: do not start extra reads while unlocked.
   * An already-started transaction is allowed to finish without colliding. */
  s_front_drive.control_read_allowed = (uint8_t)(g_robot_command.mode == ROBOT_MODE_LOCKED &&
      s_front_drive.left.status.last_update_ms != 0U &&
      s_front_drive.right.status.last_update_ms != 0U &&
      (now_ms - s_front_drive.left.status.last_update_ms) <= CHASSIS_OID_REVERSE_STATUS_MAX_AGE_MS &&
      (now_ms - s_front_drive.right.status.last_update_ms) <= CHASSIS_OID_REVERSE_STATUS_MAX_AGE_MS &&
      s_front_drive.left.status.speed_erpm >= -CHASSIS_OID_REVERSE_ZERO_ERPM &&
      s_front_drive.left.status.speed_erpm <= CHASSIS_OID_REVERSE_ZERO_ERPM &&
      s_front_drive.right.status.speed_erpm >= -CHASSIS_OID_REVERSE_ZERO_ERPM &&
      s_front_drive.right.status.speed_erpm <= CHASSIS_OID_REVERSE_ZERO_ERPM);
  FrontDrive_Task(&s_front_drive, now_ms);
  left_status = FrontDrive_GetStatus(&s_front_drive, FRONT_DRIVE_SIDE_LEFT);
  right_status = FrontDrive_GetStatus(&s_front_drive, FRONT_DRIVE_SIDE_RIGHT);

  for (diagnostic_side = 0U; diagnostic_side < 2U; diagnostic_side++)
  {
    const OID_ESC_Diagnostic_t *d = (diagnostic_side == 0U) ?
        &s_front_drive.left.diagnostic : &s_front_drive.right.diagnostic;
    g_robot_chassis.oid_read_mode[diagnostic_side] = d->control_mode;
    g_robot_chassis.oid_read_target[diagnostic_side] = d->target_erpm;
    g_robot_chassis.oid_read_age[diagnostic_side] = (d->control_update_ms == 0U) ?
        UINT32_MAX : now_ms - d->control_update_ms;
    g_robot_chassis.oid_read_timeouts[diagnostic_side] = d->control_timeouts;
    g_robot_chassis.oid_zero_writes[diagnostic_side] = d->zero_writes;
    g_robot_chassis.oid_speed_acks[diagnostic_side] = d->speed_acks;
    g_robot_chassis.oid_write_unconfirmed[diagnostic_side] = d->speed_unconfirmed;
    g_robot_chassis.oid_exceptions[diagnostic_side] = d->exceptions;
    g_robot_chassis.oid_last_exception[diagnostic_side] = d->last_exception;
  }

  g_robot_chassis.left_speed_erpm = left_status->speed_erpm;
  g_robot_chassis.right_speed_erpm = right_status->speed_erpm;
  g_robot_chassis.left_fault = left_status->fault;
  g_robot_chassis.right_fault = right_status->fault;
  g_robot_chassis.left_timeout_count = left_status->timeout_count;
  g_robot_chassis.right_timeout_count = right_status->timeout_count;
  g_robot_chassis.left_crc_error_count = left_status->crc_error_count;
  g_robot_chassis.right_crc_error_count = right_status->crc_error_count;
  g_robot_chassis.rs485_overflow_count = s_oid_bus.rx_overflow_count;
  g_robot_chassis.rs485_uart_error_count = s_oid_bus.uart_error_count;
  g_robot_chassis.left_status_age_ms =
      (left_status->last_update_ms == 0U) ? UINT32_MAX :
      (now_ms - left_status->last_update_ms);
  g_robot_chassis.right_status_age_ms =
      (right_status->last_update_ms == 0U) ? UINT32_MAX :
      (now_ms - right_status->last_update_ms);
  g_robot_chassis.left_heartbeat_age_ms =
      now_ms - s_front_drive.left_last_heartbeat_ms;
  g_robot_chassis.right_heartbeat_age_ms =
      now_ms - s_front_drive.right_last_heartbeat_ms;
  g_robot_chassis.status_preempt_count = s_front_drive.status_preempt_count;
  g_robot_chassis.oid_safety_state = s_front_drive.safety_state;
  g_robot_chassis.oid_command_pending = s_front_drive.command_pending;
  g_robot_chassis.oid_command_step = s_front_drive.command_step;
  g_robot_chassis.oid_command_urgent = s_front_drive.command_urgent;
  g_robot_chassis.oid_read_active = s_front_drive.control_read_enabled && s_front_drive.control_read_allowed;
  g_robot_chassis.oid_heartbeat_max_gap[0] = s_front_drive.left_heartbeat_max_gap_ms;
  g_robot_chassis.oid_heartbeat_max_gap[1] = s_front_drive.right_heartbeat_max_gap_ms;
  g_robot_chassis.oid_tx_target[0] = s_front_drive.left_speed_tx_erpm;
  g_robot_chassis.oid_tx_target[1] = s_front_drive.right_speed_tx_erpm;
  g_robot_chassis.oid_tx_ms[0] = s_front_drive.left_speed_tx_ms;
  g_robot_chassis.oid_tx_ms[1] = s_front_drive.right_speed_tx_ms;
  g_robot_chassis.left_online =
      FrontDrive_IsOnline(&s_front_drive, FRONT_DRIVE_SIDE_LEFT, now_ms);
  g_robot_chassis.right_online =
      FrontDrive_IsOnline(&s_front_drive, FRONT_DRIVE_SIDE_RIGHT, now_ms);

  if (PwmInput_GetSnapshot(s_fc_drive_input, &s_fc_drive, now_ms) == 0U)
  {
    memset(&s_fc_drive, 0, sizeof(s_fc_drive));
    s_fc_drive.fault = PWM_INPUT_FAULT_NOT_READY;
  }
  if (PwmInput_GetSnapshot(s_fc_steer_input, &s_fc_steer, now_ms) == 0U)
  {
    memset(&s_fc_steer, 0, sizeof(s_fc_steer));
    s_fc_steer.fault = PWM_INPUT_FAULT_NOT_READY;
  }
  ChassisApp_UpdateFcRelease(now_ms);
  g_robot_chassis.fc_drive_raw_us = s_fc_drive.raw_us;
  g_robot_chassis.fc_drive_filtered_us = s_fc_drive.filtered_us;
  g_robot_chassis.fc_steer_raw_us = s_fc_steer.raw_us;
  g_robot_chassis.fc_steer_filtered_us = s_fc_steer.filtered_us;
  g_robot_chassis.fc_drive_online = s_fc_drive.online;
  g_robot_chassis.fc_steer_online = s_fc_steer.online;
  g_robot_chassis.fc_drive_fault = (uint8_t)s_fc_drive.fault;
  g_robot_chassis.fc_steer_fault = (uint8_t)s_fc_steer.fault;
  g_robot_chassis.fc_release_ready = s_fc_release_ready;

  if ((now_ms - s_last_control_ms) < CHASSIS_CONTROL_PERIOD_MS)
  {
    return;
  }
  s_last_control_ms = now_ms;

  rearm_centered = (g_robot_command.mode == ROBOT_MODE_AUTO_FC) ?
      (uint8_t)(((ChassisApp_FcInputsHealthy() != 0U) &&
                 (ChassisApp_GetFcDrivePermille() == 0) &&
                 (ChassisApp_GetFcSteerPermille() == 0)) ? 1U : 0U) :
      g_robot_command.throttle_centered;
  if ((FrontDrive_IsSafetyRearmRequired(&s_front_drive) != 0U) &&
      (g_robot_command.rc_online != 0U) &&
      (g_robot_command.failsafe == 0U) &&
      (rearm_centered != 0U))
  {
    FrontDrive_RearmSafety(&s_front_drive, now_ms);
  }

  if (g_robot_command.mode == ROBOT_MODE_MANUAL)
  {
    active_steering = g_robot_command.steering_permille;
    active_throttle = g_robot_command.throttle_permille;
    active_speed_limit = g_robot_command.speed_limit_erpm;
    source_ready = (uint8_t)((g_robot_command.gate == ROBOT_GATE_READY) ? 1U : 0U);
  }
  else if (g_robot_command.mode == ROBOT_MODE_AUTO_FC)
  {
    active_steering = ChassisApp_GetFcSteerPermille();
    active_throttle = ChassisApp_GetFcDrivePermille();
    active_speed_limit = CHASSIS_FC_SPEED_LIMIT_ERPM;
    source_ready = s_fc_release_ready;
  }
  else
  {
    source_ready = 0U;
  }

  if (CHASSIS_OID_COMMISSION_MAX_ERPM != 0U &&
      active_speed_limit > CHASSIS_OID_COMMISSION_MAX_ERPM)
    active_speed_limit = CHASSIS_OID_COMMISSION_MAX_ERPM;

  /* Same gain reference as V4: effective base command including the CH7 limit.
     The returned command is the separate V4-style OID differential ramp. */
  active_steering = DualSteer_Task(active_steering,
      (int16_t)(((int32_t)active_throttle * active_speed_limit) /
                (int32_t)CHASSIS_ACKERMANN_SPEED_REFERENCE_ERPM), source_ready, now_ms);

  permit_motion = (uint8_t)((CHASSIS_PARAMETERS_CONFIRMED != 0U) &&
                            (CHASSIS_OID_OUTPUT_ENABLE != 0U) &&
                            (source_ready != 0U) &&
                            (DualSteer_MotionReady() != 0U) &&
                            (FrontDrive_IsMotionReady(&s_front_drive, now_ms) != 0U));
  if (permit_motion != 0U)
  {
    ChassisApp_CalculateTargets(active_steering,
                                active_throttle,
                                active_speed_limit,
                                &left_target, &right_target);
    left_target *= CHASSIS_OID_LEFT_DIRECTION;
    right_target *= CHASSIS_OID_RIGHT_DIRECTION;
  }

  g_robot_chassis.oid_requested_base_erpm = permit_motion ?
      (int32_t)active_throttle * active_speed_limit / 1000 : 0;
  if (CHASSIS_OID_REVERSE_GUARD_ENABLE != 0U &&
      OidReverseGuard_Update(&s_reverse_guard, g_robot_chassis.oid_requested_base_erpm,
      (uint8_t)(g_robot_chassis.left_online && g_robot_chassis.right_online &&
                left_status->fault == 0U && right_status->fault == 0U),
      (uint8_t)(s_front_drive.left_speed_tx_erpm == 0 && s_front_drive.right_speed_tx_erpm == 0 &&
                (now_ms - s_front_drive.left_speed_tx_ms) < (now_ms - s_reverse_guard.begin_ms) &&
                (now_ms - s_front_drive.right_speed_tx_ms) < (now_ms - s_reverse_guard.begin_ms)),
      left_status->speed_erpm * CHASSIS_OID_LEFT_DIRECTION,
      right_status->speed_erpm * CHASSIS_OID_RIGHT_DIRECTION,
      left_status->last_update_ms, right_status->last_update_ms, now_ms) == 0U)
  {
    left_target = right_target = 0;
    permit_motion = 0U;
  }
  g_robot_chassis.oid_reverse_wait = s_reverse_guard.waiting;
  g_robot_chassis.oid_reverse_zero_pairs = s_reverse_guard.zero_pairs;

  OidStopGuard_Update(&s_stop_guard,
      (uint8_t)(left_target == 0 && right_target == 0));
  if (OidStopGuard_RequestDue(&s_stop_guard, now_ms) != 0U)
  {
    if (FrontDrive_StopUrgent(&s_front_drive) == HAL_OK)
      OidStopGuard_MarkRequest(&s_stop_guard, now_ms);
  }
  else if (s_stop_guard.active == 0U)
  {
    (void)FrontDrive_SetTargetErpm(&s_front_drive, left_target, right_target);
  }

  g_robot_chassis.oid_stop_fault = 0U; /* UART7兼容字段：停车永久锁存已移除。 */
  g_robot_chassis.left_target_erpm = left_target;
  g_robot_chassis.right_target_erpm = right_target;
  g_robot_chassis.motion_enabled = permit_motion;
}
