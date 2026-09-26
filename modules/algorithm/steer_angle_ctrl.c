/**
  ******************************************************************************
  * @file    steer_angle_ctrl.c
  * @brief   基于输出轴角度反馈的转向位置控制算法实现。
  ******************************************************************************
  */

#include "steer_angle_ctrl.h"
#include <string.h>

/**
  * @brief 返回浮点绝对值，避免为简单运算额外引入库依赖。
  */
static float SteerAngleCtrl_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

/**
  * @brief 清除未完成的换向等待状态。
  */
static void SteerAngleCtrl_ResetDirectionChange(SteerAngleCtrl_Handle_t *ctrl)
{
  if (ctrl == 0)
  {
    return;
  }

  ctrl->direction_change_pending = 0U;
  ctrl->pending_reverse = 0U;
  ctrl->direction_change_started_ms = 0U;
}

/**
  * @brief 清空位置环输出斜坡状态。
  */
static void SteerAngleCtrl_ResetOutputRamp(SteerAngleCtrl_Handle_t *ctrl)
{
  if (ctrl == 0)
  {
    return;
  }

  ctrl->output_duty_permille = 0;
  ctrl->last_duty_ramp_ms = 0U;
}

/**
  * @brief 停止电机，同时把闭环内部斜坡输出清零。
  */
static void SteerAngleCtrl_StopMotor(SteerAngleCtrl_Handle_t *ctrl)
{
  if (ctrl == 0)
  {
    return;
  }

  SteerAngleCtrl_ResetOutputRamp(ctrl);
  SteerMotor_Stop(ctrl->motor);
}

/**
  * @brief 按每个调度周期最大变化量限制闭环 PWM 占空比。
  */
static int16_t SteerAngleCtrl_RampDuty(SteerAngleCtrl_Handle_t *ctrl,
                                       int16_t target_duty,
                                       uint32_t now_ms)
{
  int16_t delta;

  if (ctrl == 0)
  {
    return 0;
  }

  if (target_duty <= 0)
  {
    SteerAngleCtrl_ResetOutputRamp(ctrl);
    return 0;
  }

  if ((ctrl->last_duty_ramp_ms == 0U) ||
      (ctrl->duty_ramp_step_permille <= 0))
  {
    ctrl->last_duty_ramp_ms = now_ms;
  }

  delta = (int16_t)(target_duty - ctrl->output_duty_permille);
  if (delta > ctrl->duty_ramp_step_permille)
  {
    delta = ctrl->duty_ramp_step_permille;
  }
  else if (delta < (int16_t)(-ctrl->duty_ramp_step_permille))
  {
    delta = (int16_t)(-ctrl->duty_ramp_step_permille);
  }

  ctrl->output_duty_permille = (int16_t)(ctrl->output_duty_permille + delta);
  ctrl->last_duty_ramp_ms = now_ms;
  return ctrl->output_duty_permille;
}

/**
  * @brief 进入故障锁存状态并停止转向电机。
  */
static void SteerAngleCtrl_EnterFault(SteerAngleCtrl_Handle_t *ctrl)
{
  if (ctrl == 0)
  {
    return;
  }

  ctrl->state = STEER_ANGLE_CTRL_FAULT;
  ctrl->fault_count++;
  SteerAngleCtrl_ResetDirectionChange(ctrl);
  ctrl->stopped_in_tolerance = 0U;
  SteerAngleCtrl_StopMotor(ctrl);
}

/**
  * @brief 初始化转向角内环对象。
  * @note  上电后始终保持禁用。输出轴零点、硬故障线和方向没有完成实车标定前，不会主动转向。
  */
void SteerAngleCtrl_Init(SteerAngleCtrl_Handle_t *ctrl,
                         SteerMotor_Handle_t *motor,
                         Mt6826sPwm_Handle_t *angle)
{
  if (ctrl == 0)
  {
    return;
  }

  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->motor = motor;
  ctrl->angle = angle;
  ctrl->state = STEER_ANGLE_CTRL_DISABLED;
  ctrl->duty_scale_permille = 1000U;
  SteerAngleCtrl_StopMotor(ctrl);
}

/**
  * @brief 写入装车标定后的转向输出轴位置闭环参数。
  * @param center_tolerance_deg 允许停止电机的角度误差。
  * @param restart_error_deg    停机后重新启动所需的误差阈值，需大于停止阈值。
  * @param max_abs_angle_deg    输出轴允许的右左最大绝对角度。
  * @param kp_permille_per_deg  比例控制系数。
  * @param return_center_target_deg  目标角接近 0 时启用回中软着陆的阈值。
  * @param return_center_tolerance_deg 回中软着陆停止误差带。
  * @param return_center_restart_deg 回中软着陆重新启动误差带。
  * @param min_duty_permille    闭环启动电机时使用的最小低有效 PWM 千分比。
  * @param max_duty_permille    允许的最大低有效 PWM 千分比。
  * @param return_center_max_duty_permille 回中软着陆允许的最大低有效 PWM 千分比。
  * @param duty_ramp_step_permille 每个 10ms 调度周期允许的最大占空比变化量。
  * @param positive_error_reverse 正角度误差对应的 5840 方向电平。
  */
void SteerAngleCtrl_Configure(SteerAngleCtrl_Handle_t *ctrl,
                              float center_tolerance_deg,
                              float restart_error_deg,
                              float max_abs_angle_deg,
                              float kp_permille_per_deg,
                              float return_center_target_deg,
                              float return_center_tolerance_deg,
                              float return_center_restart_deg,
                              int16_t min_duty_permille,
                              int16_t max_duty_permille,
                              int16_t return_center_max_duty_permille,
                              int16_t duty_ramp_step_permille,
                              uint8_t positive_error_reverse)
{
  if ((ctrl == 0) ||
      (center_tolerance_deg <= 0.0f) ||
      (restart_error_deg <= center_tolerance_deg) ||
      (max_abs_angle_deg <= center_tolerance_deg) ||
      (restart_error_deg >= max_abs_angle_deg) ||
      (kp_permille_per_deg <= 0.0f) ||
      (return_center_target_deg < 0.0f) ||
      (return_center_tolerance_deg < center_tolerance_deg) ||
      (return_center_restart_deg <= return_center_tolerance_deg) ||
      (return_center_restart_deg >= max_abs_angle_deg) ||
      (min_duty_permille < 0) ||
      (max_duty_permille <= 0) ||
      (min_duty_permille > max_duty_permille) ||
      (return_center_max_duty_permille < min_duty_permille) ||
      (return_center_max_duty_permille > max_duty_permille) ||
      (duty_ramp_step_permille <= 0) ||
      (duty_ramp_step_permille > 1000) ||
      (max_duty_permille > 1000))
  {
    return;
  }

  ctrl->center_tolerance_deg = center_tolerance_deg;
  ctrl->restart_error_deg = restart_error_deg;
  ctrl->max_abs_angle_deg = max_abs_angle_deg;
  ctrl->kp_permille_per_deg = kp_permille_per_deg;
  ctrl->return_center_target_deg = return_center_target_deg;
  ctrl->return_center_tolerance_deg = return_center_tolerance_deg;
  ctrl->return_center_restart_deg = return_center_restart_deg;
  ctrl->min_duty_permille = min_duty_permille;
  ctrl->runtime_min_duty_permille = min_duty_permille;
  ctrl->max_duty_permille = max_duty_permille;
  ctrl->return_center_max_duty_permille = return_center_max_duty_permille;
  ctrl->duty_ramp_step_permille = duty_ramp_step_permille;
  ctrl->duty_scale_permille = 1000U;
  ctrl->positive_error_reverse = (positive_error_reverse != 0U) ? 1U : 0U;
  ctrl->stopped_in_tolerance = 0U;
  SteerAngleCtrl_ResetOutputRamp(ctrl);
  ctrl->calibrated = 1U;
}

/**
  * @brief 显式使能转向角内环。
  * @return 已标定输出轴参数、MT6826S 零点且不在故障状态时返回 HAL_OK。
  */
HAL_StatusTypeDef SteerAngleCtrl_Enable(SteerAngleCtrl_Handle_t *ctrl)
{
  if ((ctrl == 0) ||
      (ctrl->calibrated == 0U) ||
      (ctrl->angle == 0) ||
      (ctrl->angle->zero_valid == 0U) ||
      (ctrl->state == STEER_ANGLE_CTRL_FAULT))
  {
    return HAL_ERROR;
  }

  ctrl->state = STEER_ANGLE_CTRL_HOLD;
  ctrl->stopped_in_tolerance = 0U;
  SteerAngleCtrl_ResetOutputRamp(ctrl);
  ctrl->enable_count++;
  return HAL_OK;
}

/**
  * @brief 禁用角度内环并立即停止转向电机。
  */
void SteerAngleCtrl_Disable(SteerAngleCtrl_Handle_t *ctrl)
{
  if (ctrl == 0)
  {
    return;
  }

  ctrl->state = STEER_ANGLE_CTRL_DISABLED;
  SteerAngleCtrl_ResetDirectionChange(ctrl);
  ctrl->stopped_in_tolerance = 0U;
  SteerAngleCtrl_StopMotor(ctrl);
}

/**
  * @brief 清除故障锁存，回到默认禁用状态。
  * @note  清故障后仍需由上层重新检查安全条件并显式使能。
  */
void SteerAngleCtrl_ClearFault(SteerAngleCtrl_Handle_t *ctrl)
{
  if (ctrl == 0)
  {
    return;
  }

  ctrl->state = STEER_ANGLE_CTRL_DISABLED;
  SteerAngleCtrl_ResetDirectionChange(ctrl);
  ctrl->stopped_in_tolerance = 0U;
  SteerAngleCtrl_StopMotor(ctrl);
}

/**
  * @brief 设置目标电机输出轴角度。
  * @return 目标未超过输出轴硬故障线时返回 HAL_OK。
  */
HAL_StatusTypeDef SteerAngleCtrl_SetTargetDeg(SteerAngleCtrl_Handle_t *ctrl,
                                              float target_angle_deg)
{
  if ((ctrl == 0) ||
      (ctrl->calibrated == 0U) ||
      (SteerAngleCtrl_Abs(target_angle_deg) > ctrl->max_abs_angle_deg))
  {
    return HAL_ERROR;
  }

  ctrl->target_angle_deg = target_angle_deg;
  return HAL_OK;
}

/**
  * @brief 设置运行期 duty 调节比例。
  * @note  比例只缩放最小起转 duty 以上的可调区间，保留机械克服静摩擦所需的
  *        min_duty_permille；1000 表示保持位置环原始输出，超范围输入按 1000 处理。
  */
void SteerAngleCtrl_SetDutyScalePermille(SteerAngleCtrl_Handle_t *ctrl,
                                          uint16_t scale_permille)
{
  if (ctrl == 0)
  {
    return;
  }

  ctrl->duty_scale_permille =
    (scale_permille <= 1000U) ? scale_permille : 1000U;
}

/**
  * @brief 设置本周期使用的最小起转 duty。
  * @note  用于内/外侧机构行程同步；允许低于常规最小值，但不会低于 0 或超过
  *        已配置的最大 duty。调用方必须结合装车实测确认该值仍能稳定驱动电机。
  */
void SteerAngleCtrl_SetRuntimeMinDutyPermille(SteerAngleCtrl_Handle_t *ctrl,
                                               int16_t min_duty_permille)
{
  if (ctrl == 0)
  {
    return;
  }

  if (min_duty_permille < 0)
  {
    min_duty_permille = 0;
  }
  else if (min_duty_permille > ctrl->max_duty_permille)
  {
    min_duty_permille = ctrl->max_duty_permille;
  }
  ctrl->runtime_min_duty_permille = min_duty_permille;
}

/**
  * @brief 执行一次转向电机输出轴角度比例闭环。
  * @param now_ms 当前 HAL tick。
  * @note  反馈来自转向电机输出轴上的 MT6826S，因此限制的是输出轴角度，不是轮胎真实转角。
  *        若上位机或行走控制需要轮胎角度，必须使用后续标定得到的输出轴角度到轮胎角度映射。
  *        转向机构与输出轴当前按刚性无打滑处理；结构松动仍需机械检查。
  */
void SteerAngleCtrl_Task(SteerAngleCtrl_Handle_t *ctrl, uint32_t now_ms)
{
  Mt6826sPwm_Snapshot_t angle_snapshot;
  float current_angle;
  float error;
  float abs_error;
  float stop_tolerance_deg;
  float restart_error_deg;
  float duty_float;
  int16_t max_duty_permille;
  int16_t min_duty_permille;
  int16_t duty;
  int32_t duty_span;
  uint8_t reverse;

  if ((ctrl == 0) || (ctrl->motor == 0) || (ctrl->angle == 0))
  {
    return;
  }

  if (ctrl->state != STEER_ANGLE_CTRL_HOLD)
  {
    ctrl->stopped_in_tolerance = 0U;
    SteerAngleCtrl_StopMotor(ctrl);
    return;
  }

  if ((ctrl->calibrated == 0U) ||
      (Mt6826sPwm_IsHealthy(ctrl->angle, now_ms) == 0U))
  {
    SteerAngleCtrl_EnterFault(ctrl);
    return;
  }

  Mt6826sPwm_GetSnapshot(ctrl->angle, now_ms, &angle_snapshot);
  if ((angle_snapshot.healthy == 0U) || (angle_snapshot.zero_valid == 0U))
  {
    SteerAngleCtrl_EnterFault(ctrl);
    return;
  }

  current_angle = angle_snapshot.relative_angle_deg;
  if (SteerAngleCtrl_Abs(current_angle) > ctrl->max_abs_angle_deg)
  {
    SteerAngleCtrl_EnterFault(ctrl);
    return;
  }

  error = ctrl->target_angle_deg - current_angle;
  abs_error = SteerAngleCtrl_Abs(error);
  stop_tolerance_deg = ctrl->center_tolerance_deg;
  restart_error_deg = ctrl->restart_error_deg;
  max_duty_permille = ctrl->max_duty_permille;
  min_duty_permille = ctrl->runtime_min_duty_permille;
  if ((ctrl->return_center_target_deg > 0.0f) &&
      (SteerAngleCtrl_Abs(ctrl->target_angle_deg) <=
       ctrl->return_center_target_deg))
  {
    stop_tolerance_deg = ctrl->return_center_tolerance_deg;
    restart_error_deg = ctrl->return_center_restart_deg;
    max_duty_permille = ctrl->return_center_max_duty_permille;
  }

  if (abs_error <= stop_tolerance_deg)
  {
    SteerAngleCtrl_ResetDirectionChange(ctrl);
    if (ctrl->stopped_in_tolerance == 0U)
    {
      ctrl->tolerance_stop_count++;
    }
    ctrl->stopped_in_tolerance = 1U;
    SteerAngleCtrl_StopMotor(ctrl);
    return;
  }

  if (ctrl->stopped_in_tolerance != 0U)
  {
    if (abs_error <= restart_error_deg)
    {
      SteerAngleCtrl_ResetDirectionChange(ctrl);
      SteerAngleCtrl_StopMotor(ctrl);
      return;
    }
    ctrl->tolerance_restart_count++;
    ctrl->stopped_in_tolerance = 0U;
  }

  duty_float = abs_error * ctrl->kp_permille_per_deg;
  if ((min_duty_permille > 0) &&
      (duty_float < (float)min_duty_permille))
  {
    duty_float = (float)min_duty_permille;
  }
  if (duty_float > (float)max_duty_permille)
  {
    duty_float = (float)max_duty_permille;
  }
  duty = (int16_t)duty_float;

  /*
   * 运行期同步补偿只压缩最小起转 duty 以上的区间。例如 min=300、原输出=1000、
   * scale=850 时得到 300 + (700 * 850 / 1000) = 895。这样可以减慢领先侧，
   * 又不会把低速输出压到无法克服减速机构静摩擦的范围。
   */
  if ((duty > min_duty_permille) &&
      (ctrl->duty_scale_permille < 1000U))
  {
    duty_span = (int32_t)duty - (int32_t)min_duty_permille;
    duty_span = (duty_span * (int32_t)ctrl->duty_scale_permille + 500) / 1000;
    duty = (int16_t)((int32_t)min_duty_permille + duty_span);
  }

  reverse = (error > 0.0f) ? ctrl->positive_error_reverse :
                             (uint8_t)(ctrl->positive_error_reverse == 0U);

  /*
   * 厂家要求切换正反转前必须先停机。首次发现方向变化时先输出停止和刹车，
   * 修改方向脚后立即返回；后续调度周期只检查时间，不阻塞其他裸机任务。
   */
  if (ctrl->motor->reverse != reverse)
  {
    SteerAngleCtrl_StopMotor(ctrl);
    SteerMotor_SetDirection(ctrl->motor, reverse);
    ctrl->direction_change_count++;
    ctrl->pending_reverse = reverse;
    ctrl->direction_change_started_ms = now_ms;
    ctrl->direction_change_pending = 1U;
    return;
  }

  if (ctrl->direction_change_pending != 0U)
  {
    if ((now_ms - ctrl->direction_change_started_ms) <
        STEER_ANGLE_CTRL_DIRECTION_DEADTIME_MS)
    {
      SteerAngleCtrl_StopMotor(ctrl);
      return;
    }

    SteerAngleCtrl_ResetDirectionChange(ctrl);
  }

  duty = SteerAngleCtrl_RampDuty(ctrl, duty, now_ms);
  SteerMotor_SetBrake(ctrl->motor, 0U);
  SteerMotor_SetDuty(ctrl->motor, duty);
}
