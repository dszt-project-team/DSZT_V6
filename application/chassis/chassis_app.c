#include "chassis_app.h"

#include "bsp_rs485.h"
#include "dual_steer.h"
#include "chassis_config.h"
#include "oid_stop_guard.h"
#include "oid_reverse_guard.h"
#include "front_drive.h"
#include "robot_def.h"
#include "usart.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static BSP_RS485_Bus_t s_oid_bus;
static FrontDrive_Handle_t s_front_drive;
static uint32_t s_last_control_ms;
static OidStopGuard s_stop_guard;
static OidReverseGuard s_reverse_guard;
static uint32_t s_rearm_center_ms;
static uint8_t s_rearm_center_timing;
static uint8_t s_rearm_was_required;

static void ChassisApp_RequestStop(uint32_t now_ms)
{
  OidStopGuard_Update(&s_stop_guard, 1U);
  if (OidStopGuard_RequestDue(&s_stop_guard, now_ms) != 0U &&
      FrontDrive_StopUrgent(&s_front_drive) == HAL_OK)
    OidStopGuard_MarkRequest(&s_stop_guard, now_ms);
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
  DualSteer_Init();
  memset(&s_stop_guard, 0, sizeof(s_stop_guard));
  memset(&s_reverse_guard, 0, sizeof(s_reverse_guard));

  s_last_control_ms = 0U;
  s_rearm_center_ms = 0U;
  s_rearm_center_timing = 0U;
  s_rearm_was_required = 0U;

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
  uint8_t rearm_ready = 0U;
  uint8_t rearm_required;
  int16_t active_steering = 0;
  int16_t active_throttle = 0;
  uint16_t active_speed_limit = CHASSIS_FC_SPEED_LIMIT_ERPM;
  uint8_t diagnostic_side;

  source_ready = (uint8_t)(g_robot_command.source_online &&
      g_robot_command.released && g_robot_command.gate == ROBOT_GATE_READY &&
      (g_robot_command.mode == ROBOT_MODE_AUTO_FC ||
       g_robot_command.mode == ROBOT_MODE_CALIBRATION));

  /* 输入异常在1ms服务层先撤销待发非零目标，再服务总线，避免先发送上一轮油门。
   * 已在途的RTU事务仍由FrontDrive正常收尾，不截断接收帧。 */
  if (!source_ready || CHASSIS_STEER_CALIBRATION_SIDE != 0U ||
      CHASSIS_PARAMETERS_CONFIRMED == 0U || CHASSIS_OID_OUTPUT_ENABLE == 0U ||
      g_robot_chassis.steer_fault != 0U)
  {
    ChassisApp_RequestStop(now_ms);
    g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 0;
    g_robot_chassis.oid_requested_base_erpm = 0;
    g_robot_chassis.motion_enabled = 0U;
  }
  if (!source_ready)
    (void)DualSteer_Task(0, 0, 0U, now_ms);

  /* FC无锁车档。额外只读事务仅在双轴中位、目标零且反馈新鲜低速时发起。 */
  s_front_drive.control_read_allowed = (uint8_t)(g_robot_command.source_online &&
      g_robot_command.centered &&
      g_robot_chassis.left_target_erpm == 0 && g_robot_chassis.right_target_erpm == 0 &&
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

  /* 心跳断档重臂需要一次新的持续双回中，不复用运行前的授权时间。 */
  rearm_required = FrontDrive_IsSafetyRearmRequired(&s_front_drive);
  if (!rearm_required || !s_rearm_was_required)
    s_rearm_center_timing = 0U;
  s_rearm_was_required = rearm_required;
  if (rearm_required && source_ready && g_robot_command.centered)
  {
    if (!s_rearm_center_timing)
    {
      s_rearm_center_timing = 1U;
      s_rearm_center_ms = now_ms;
    }
    rearm_ready = (uint8_t)((now_ms - s_rearm_center_ms) >= CHASSIS_FC_RELEASE_CENTER_MS);
  }
  else
  {
    s_rearm_center_timing = 0U;
  }

  if ((now_ms - s_last_control_ms) < CHASSIS_CONTROL_PERIOD_MS)
  {
    return;
  }
  s_last_control_ms = now_ms;

  if (rearm_required != 0U && rearm_ready)
    FrontDrive_RearmSafety(&s_front_drive, now_ms);

  active_steering = g_robot_command.steering_permille;
  active_throttle = g_robot_command.throttle_permille;
  active_speed_limit = g_robot_command.speed_limit_erpm;

  if (CHASSIS_OID_COMMISSION_MAX_ERPM != 0U &&
      active_speed_limit > CHASSIS_OID_COMMISSION_MAX_ERPM)
    active_speed_limit = CHASSIS_OID_COMMISSION_MAX_ERPM;

  /* Same gain reference as V4: effective base command including the FC speed limit.
     The returned command is the separate V4-style OID differential ramp. */
  active_steering = DualSteer_Task(active_steering,
      (int16_t)(((int32_t)active_throttle * active_speed_limit) /
                (int32_t)CHASSIS_ACKERMANN_SPEED_REFERENCE_ERPM), source_ready, now_ms);

  permit_motion = (uint8_t)((CHASSIS_PARAMETERS_CONFIRMED != 0U) &&
                            (CHASSIS_OID_OUTPUT_ENABLE != 0U) &&
                            (CHASSIS_STEER_CALIBRATION_SIDE == 0U) &&
                            (g_robot_command.mode == ROBOT_MODE_AUTO_FC) &&
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
