#include "dual_steer.h"
#include "chassis_config.h"
#include "robot_def.h"
#include "steer_angle_ctrl.h"
#include "tim.h"
#include <math.h>
#include <stdlib.h>

/* Physical V6 left = old V4 right (J31); V6 right = old V4 left (J26). */
static SteerMotor_Handle_t s_motor[2];
static Mt6826sPwm_Handle_t s_encoder[2];
static Mt6826sPwm_Snapshot_t s_feedback[2];
static SteerAngleCtrl_Handle_t s_ctrl[2];
static uint8_t s_released, s_center_timing, s_fault;
static uint32_t s_center_ms, s_direction_ms;
static uint8_t s_direction_stage, s_pending_reverse;
static float s_oid_command;
static float s_target[2];
static uint32_t s_slew_ms;
static uint8_t s_slew_valid;
static uint8_t s_previous_mode;

int16_t DualSteer_ScheduleCommand(int16_t command, int16_t drive_command_permille)
{
  float ratio, gain = 1.0f, scheduled;
  if (command > 1000) command = 1000;
  if (command < -1000) command = -1000;
  if (CHASSIS_ACKERMANN_SPEED_GAIN_ENABLE != 0U)
  {
    ratio = fabsf((float)drive_command_permille) / CHASSIS_ACKERMANN_SPEED_GAIN_FULL_COMMAND;
    if (ratio > 1.0f) ratio = 1.0f;
    gain = 1.0f - (1.0f - CHASSIS_ACKERMANN_SPEED_GAIN_MIN) * ratio;
  }
  g_robot_chassis.steering_speed_gain_permille = (uint16_t)(gain * 1000.0f + 0.5f);
  scheduled = (float)command * gain;
  if (fabsf((float)command) <= CHASSIS_ACKERMANN_COMMAND_DEADBAND * 1000.0f ||
      fabsf(scheduled) <= CHASSIS_ACKERMANN_COMMAND_DEADBAND * 1000.0f) return 0;
  return (int16_t)((scheduled < 0.0f) ? scheduled - 0.5f : scheduled + 0.5f);
}

void DualSteer_Geometry(int16_t command, float *left_deg, float *right_deg,
                        float *left_scale, float *right_scale)
{
  float inner, outer, radius, inner_path, outer_path, sign;
  *left_deg = *right_deg = 0.0f;
  *left_scale = *right_scale = 1.0f;
  if (command == 0) return;
  if (command > 1000) command = 1000;
  if (command < -1000) command = -1000;
  inner = (float)abs(command) * CHASSIS_MAX_INNER_WHEEL_DEG / 1000.0f;
  radius = CHASSIS_WHEELBASE_M / tanf(inner * 0.01745329252f);
  outer = atanf(CHASSIS_WHEELBASE_M / (radius + CHASSIS_FRONT_TRACK_M)) * 57.29577951f;
  inner_path = sqrtf(CHASSIS_WHEELBASE_M * CHASSIS_WHEELBASE_M + radius * radius);
  outer_path = sqrtf(CHASSIS_WHEELBASE_M * CHASSIS_WHEELBASE_M +
                    (radius + CHASSIS_FRONT_TRACK_M) * (radius + CHASSIS_FRONT_TRACK_M));
  /* Wheel angles use V6 coordinates: negative left. Shaft sign agrees with
     V4 reverse gear after swapping physical sides, without a runtime switch. */
  sign = (command < 0) ? -1.0f : 1.0f;
  *left_deg = sign * ((command < 0) ? inner : outer);
  *right_deg = sign * ((command < 0) ? outer : inner);
  if (command < 0) *left_scale = inner_path / outer_path;
  else *right_scale = inner_path / outer_path;
}

void DualSteer_Init(void)
{
  unsigned i;
  /* 只有控制板初始化清除转向硬故障；飞控输入短暂丢失不会代替硬故障复位。 */
  s_released = s_center_timing = s_fault = 0U;
  s_direction_stage = s_pending_reverse = s_slew_valid = 0U;
  s_center_ms = s_direction_ms = s_slew_ms = 0U;
  s_oid_command = s_target[0] = s_target[1] = 0.0f;
  s_previous_mode = 0U;
  if (!(CHASSIS_ACKERMANN_SPEED_GAIN_MIN > 0.0f && CHASSIS_ACKERMANN_SPEED_GAIN_MIN <= 1.0f) ||
      !(CHASSIS_ACKERMANN_SPEED_GAIN_FULL_COMMAND > 0.0f) ||
      !(CHASSIS_ACKERMANN_COMMAND_DEADBAND >= 0.0f && CHASSIS_ACKERMANN_COMMAND_DEADBAND < 1.0f) ||
      !(CHASSIS_STEER_TARGET_SLEW_DEG_PER_S > 0.0f) ||
      !(CHASSIS_OID_ACKERMANN_CMD_SLEW_PER_S > 0.0f))
    Error_Handler();
  if (SteerMotor_Init(&s_motor[0], &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2,
      STEER_LEFT_DIR_GPIO_Port, STEER_LEFT_DIR_Pin,
      STEER_LEFT_BRAKE_GPIO_Port, STEER_LEFT_BRAKE_Pin, TIM8_CC_IRQn) != HAL_OK ||
      SteerMotor_Init(&s_motor[1], &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2,
      STEER_RIGHT_DIR_GPIO_Port, STEER_RIGHT_DIR_Pin,
      STEER_RIGHT_BRAKE_GPIO_Port, STEER_RIGHT_BRAKE_Pin, TIM4_IRQn) != HAL_OK)
    Error_Handler();
  if (Mt6826sPwm_Init(&s_encoder[0], &htim2, TIM_CHANNEL_1,
      MT6826S_LEFT_PWM_GPIO_Port, MT6826S_LEFT_PWM_Pin, GPIO_AF1_TIM2) != HAL_OK ||
      Mt6826sPwm_Init(&s_encoder[1], &htim5, TIM_CHANNEL_2,
      MT6826S_RIGHT_PWM_GPIO_Port, MT6826S_RIGHT_PWM_Pin, GPIO_AF2_TIM5) != HAL_OK)
    Error_Handler();
  Mt6826sPwm_SetDirectionInverted(&s_encoder[0], CHASSIS_STEER_LEFT_SENSOR_INVERT);
  Mt6826sPwm_SetDirectionInverted(&s_encoder[1], CHASSIS_STEER_RIGHT_SENSOR_INVERT);
  if (CHASSIS_STEER_LEFT_ZERO_CONFIRMED != 0U)
    Mt6826sPwm_SetZeroRaw(&s_encoder[0], CHASSIS_STEER_LEFT_ZERO_RAW);
  if (CHASSIS_STEER_RIGHT_ZERO_CONFIRMED != 0U)
    Mt6826sPwm_SetZeroRaw(&s_encoder[1], CHASSIS_STEER_RIGHT_ZERO_RAW);
  for (i = 0U; i < 2U; i++)
  {
    SteerAngleCtrl_Init(&s_ctrl[i], &s_motor[i], &s_encoder[i]);
    SteerAngleCtrl_Configure(&s_ctrl[i], CHASSIS_STEER_POSITION_TOLERANCE_DEG,
        CHASSIS_STEER_POSITION_RESTART_DEG, CHASSIS_STEER_POSITION_MAX_ABS_DEG,
        CHASSIS_STEER_POSITION_KP_PER_DEG, CHASSIS_STEER_RETURN_CENTER_TARGET_DEG,
        CHASSIS_STEER_RETURN_CENTER_TOLERANCE_DEG, CHASSIS_STEER_RETURN_CENTER_RESTART_DEG,
        CHASSIS_STEER_POSITION_MIN_DUTY, CHASSIS_STEER_POSITION_MAX_DUTY,
        CHASSIS_STEER_RETURN_CENTER_MAX_DUTY, CHASSIS_STEER_DUTY_RAMP_STEP,
        (i == 0U) ? CHASSIS_STEER_LEFT_POSITIVE_REVERSE : CHASSIS_STEER_RIGHT_POSITIVE_REVERSE);
  }
}

static void DualSteer_Stop(void)
{
  SteerAngleCtrl_Disable(&s_ctrl[0]);
  SteerAngleCtrl_Disable(&s_ctrl[1]);
  s_oid_command = 0.0f;
  s_target[0] = s_target[1] = 0.0f;
  s_slew_valid = 0U;
  s_direction_stage = 0U;
}

static void DualSteer_Calibrate(int16_t command, uint32_t now_ms)
{
  unsigned side = (CHASSIS_STEER_CALIBRATION_SIDE == 2U) ? 1U : 0U;
  uint8_t reverse;
  int16_t duty;
  SteerMotor_Stop(&s_motor[1U - side]);
  if (command == 0)
  {
    SteerMotor_Stop(&s_motor[side]);
    s_direction_stage = 0U;
    return;
  }
  reverse = (command > 0) ?
      ((side == 0U) ? CHASSIS_STEER_LEFT_POSITIVE_REVERSE : CHASSIS_STEER_RIGHT_POSITIVE_REVERSE) :
      (uint8_t)(((side == 0U) ? CHASSIS_STEER_LEFT_POSITIVE_REVERSE : CHASSIS_STEER_RIGHT_POSITIVE_REVERSE) == 0U);
  /* Stop and wait BEFORE changing FR, then wait again before applying PWM.
     All waits are timestamps, never delays in the RTOS/OID service task. */
  if ((s_motor[side].reverse != reverse) || (s_direction_stage != 0U))
  {
    SteerMotor_Stop(&s_motor[side]);
    if ((s_direction_stage == 0U) || (s_pending_reverse != reverse))
    {
      s_pending_reverse = reverse;
      s_direction_stage = 1U;
      s_direction_ms = now_ms;
      return;
    }
    if ((now_ms - s_direction_ms) < CHASSIS_STEER_CAL_DIRECTION_MS) return;
    if (s_direction_stage == 1U)
    {
      SteerMotor_SetDirection(&s_motor[side], reverse);
      s_direction_stage = 2U;
      s_direction_ms = now_ms;
      return;
    }
    s_direction_stage = 0U;
  }
  duty = (int16_t)(CHASSIS_STEER_CAL_MIN_DUTY +
         abs(command) * (CHASSIS_STEER_CAL_MAX_DUTY - CHASSIS_STEER_CAL_MIN_DUTY) / 1000);
  SteerMotor_SetDuty(&s_motor[side], duty);
  SteerMotor_SetBrake(&s_motor[side], 0U);
}

static void DualSteer_Publish(void)
{
  unsigned i;
  g_robot_chassis.steer_calibration_side = CHASSIS_STEER_CALIBRATION_SIDE;
  g_robot_chassis.steer_released = s_released;
  g_robot_chassis.steer_fault = s_fault;
  for (i = 0U; i < 2U; i++)
  {
    g_robot_chassis.steer_raw[i] = s_feedback[i].raw_angle;
    g_robot_chassis.steer_actual_x10[i] = (int16_t)s_feedback[i].relative_deg_x10;
    g_robot_chassis.steer_target_x10[i] = (int16_t)(s_ctrl[i].target_angle_deg * 10.0f);
    g_robot_chassis.steer_healthy[i] = s_feedback[i].healthy;
    g_robot_chassis.steer_zero_valid[i] = s_feedback[i].zero_valid;
    g_robot_chassis.steer_duty[i] = s_motor[i].duty_permille;
    g_robot_chassis.steer_brake[i] = s_motor[i].brake_on;
    g_robot_chassis.steer_reverse[i] = s_motor[i].reverse;
  }
}

int16_t DualSteer_Task(int16_t command, int16_t drive_command_permille,
                       uint8_t source_ready, uint32_t now_ms)
{
  unsigned i;
  uint8_t healthy;
  float wheel[2], scale[2], target;
  int16_t scheduled = 0, differential = 0;
  uint32_t elapsed_ms;
  float max_step, delta;
  uint8_t first_target;
  g_robot_chassis.steering_scheduled_permille = 0;
  g_robot_chassis.steering_speed_gain_permille = 1000U;
  if (command > 1000) command = 1000;
  if (command < -1000) command = -1000;
  if (g_robot_command.mode != s_previous_mode)
  {
    s_previous_mode = g_robot_command.mode;
    s_released = s_center_timing = 0U;
    DualSteer_Stop();
  }
  for (i = 0U; i < 2U; i++)
  {
    Mt6826sPwm_Task(&s_encoder[i], now_ms);
    Mt6826sPwm_GetSnapshot(&s_encoder[i], now_ms, &s_feedback[i]);
  }
  if (CHASSIS_STEER_CALIBRATION_SIDE != 0U &&
      CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH == 0U)
    healthy = 1U;
  else if (CHASSIS_STEER_CALIBRATION_SIDE != 0U)
    healthy = s_feedback[(CHASSIS_STEER_CALIBRATION_SIDE == 2U) ? 1U : 0U].healthy;
  else
    healthy = (uint8_t)(s_feedback[0].healthy && s_feedback[1].healthy);
  if (CHASSIS_STEER_CALIBRATION_SIDE != 0U && g_robot_command.mode != ROBOT_MODE_CALIBRATION)
    source_ready = 0U;
  if (CHASSIS_STEER_CALIBRATION_SIDE == 0U && g_robot_command.mode != ROBOT_MODE_AUTO_FC)
    source_ready = 0U;
  if (g_robot_command.source_online == 0U || g_robot_command.released == 0U ||
      g_robot_command.gate != ROBOT_GATE_READY)
    source_ready = 0U;
  if (source_ready == 0U || healthy == 0U)
  {
    if (CHASSIS_STEER_CALIBRATION_SIDE == 0U && s_released && !healthy)
      s_fault = 1U;
    /* FC没有CH5锁车。编码器运行中失效、角度越界或内环硬故障保持到板重启。
     * 单纯输入异常只撤销释放；输入恢复后必须重新双回中才允许再次动作。 */
    s_released = s_center_timing = 0U;
    DualSteer_Stop();
    DualSteer_Publish();
    return 0;
  }
  if (s_fault != 0U)
  {
    s_released = s_center_timing = 0U;
    DualSteer_Stop();
    DualSteer_Publish();
    return 0;
  }
  if (s_released == 0U)
  {
    if (command == 0 && g_robot_command.centered != 0U)
    {
      if (s_center_timing == 0U) { s_center_ms = now_ms; s_center_timing = 1U; }
      if ((now_ms - s_center_ms) >= CHASSIS_STEER_RELEASE_CENTER_MS) s_released = 1U;
    }
    else s_center_timing = 0U;
  }
  if (s_released == 0U || s_fault != 0U)
    DualSteer_Stop();
  else if (CHASSIS_STEER_CALIBRATION_SIDE != 0U)
    DualSteer_Calibrate(command, now_ms);
  else if (CHASSIS_STEER_ZERO_CONFIRMED == 0U || CHASSIS_STEER_CLOSED_LOOP_ENABLE == 0U)
    DualSteer_Stop();
  else
  {
    /* Match V4: independent OID command ramp and output-shaft angle ramp.
     * First released target seeds both paths; release requires centered inputs. */
    first_target = (uint8_t)(s_slew_valid == 0U);
    elapsed_ms = first_target ? 0U : now_ms - s_slew_ms;
    s_slew_ms = now_ms;
    s_slew_valid = 1U;
    if (first_target) s_oid_command = (float)command;
    else
    {
      max_step = CHASSIS_OID_ACKERMANN_CMD_SLEW_PER_S *
                 (float)((elapsed_ms > 100U) ? 100U : elapsed_ms);
      delta = (float)command - s_oid_command;
      if (delta > max_step) delta = max_step;
      if (delta < -max_step) delta = -max_step;
      s_oid_command += delta;
    }
    differential = DualSteer_ScheduleCommand((int16_t)s_oid_command, drive_command_permille);
    scheduled = DualSteer_ScheduleCommand(command, drive_command_permille);
    DualSteer_Geometry(scheduled, &wheel[0], &wheel[1], &scale[0], &scale[1]);
    max_step = CHASSIS_STEER_TARGET_SLEW_DEG_PER_S * 0.001f *
               (float)((elapsed_ms > 200U) ? 200U : elapsed_ms);
    for (i = 0U; i < 2U; i++)
    {
      uint8_t inner = (uint8_t)((i == 0U) ? (scheduled < 0) : (scheduled > 0));
      target = wheel[i] * (inner ? CHASSIS_STEER_INNER_OUTPUT_DEG / CHASSIS_STEER_INNER_REFERENCE_DEG :
                                  CHASSIS_STEER_OUTER_OUTPUT_DEG / CHASSIS_STEER_OUTER_REFERENCE_DEG);
      if (fabsf(target) > CHASSIS_STEER_TARGET_MAX_DEG)
      { s_fault = 1U; break; }
      if (first_target) s_target[i] = target;
      else
      {
        delta = target - s_target[i];
        if (delta > max_step) delta = max_step;
        if (delta < -max_step) delta = -max_step;
        s_target[i] += delta;
      }
      if (SteerAngleCtrl_SetTargetDeg(&s_ctrl[i], s_target[i]) != HAL_OK)
      { s_fault = 1U; break; }
      if (s_ctrl[i].state == STEER_ANGLE_CTRL_DISABLED && SteerAngleCtrl_Enable(&s_ctrl[i]) != HAL_OK)
      { s_fault = 1U; break; }
      SteerAngleCtrl_Task(&s_ctrl[i], now_ms);
      if (s_ctrl[i].state == STEER_ANGLE_CTRL_FAULT) { s_fault = 1U; break; }
    }
    if (s_fault != 0U) { s_released = 0U; DualSteer_Stop(); scheduled = differential = 0; }
  }
  DualSteer_Publish();
  g_robot_chassis.steering_scheduled_permille = scheduled;
  return differential;
}

uint8_t DualSteer_MotionReady(void)
{
  return (uint8_t)(CHASSIS_STEER_CALIBRATION_SIDE == 0U &&
      CHASSIS_STEER_CLOSED_LOOP_ENABLE != 0U &&
      CHASSIS_STEER_ZERO_CONFIRMED != 0U && CHASSIS_STEER_LINKAGE_CONFIRMED != 0U &&
      s_released != 0U && s_fault == 0U && s_feedback[0].healthy && s_feedback[1].healthy &&
      s_ctrl[0].state == STEER_ANGLE_CTRL_HOLD && s_ctrl[1].state == STEER_ANGLE_CTRL_HOLD);
}
