/**
  ******************************************************************************
  * @file    steer_angle_ctrl.h
  * @brief   基于输出轴角度反馈的转向位置控制算法接口。
  ******************************************************************************
  * @attention
  *
 * 该控制器上电默认禁用，只有上层完成输出轴零点、硬故障线和正反方向配置后，
 * 才允许显式使能。这样复位或传感器异常时，固件不会主动驱动转向电机。
  *
  ******************************************************************************
  */

#ifndef STEER_ANGLE_CTRL_H
#define STEER_ANGLE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mt6826s_pwm.h"
#include "steer_motor.h"
#include <stdint.h>

/**
  * @brief 切换转向电机方向后保持刹车的非阻塞等待时间。
  * @note  厂家要求必须在电机停止转动后再切换方向。该时间是保守起点，
  *        实车台架联调时还需要结合转向机构惯量确认是否需要加长。
  */
/* 转向换向后保持刹车的等待时间，单位 ms。 */
#define STEER_ANGLE_CTRL_DIRECTION_DEADTIME_MS 50U

/**
  * @brief 转向角内环运行状态。
  */
typedef enum
{
  STEER_ANGLE_CTRL_DISABLED = 0,  /* 默认状态：停止电机，不执行闭环。 */
  STEER_ANGLE_CTRL_HOLD,          /* 已使能：按目标电机输出轴角度闭环。 */
  STEER_ANGLE_CTRL_FAULT          /* 传感器异常或越界：锁存停止。 */
} SteerAngleCtrl_State_t;

/**
  * @brief 转向角内环对象。
  */
typedef struct
{
  SteerMotor_Handle_t *motor;        /* 5840 转向电机基础驱动。 */
  Mt6826sPwm_Handle_t *angle;        /* 单个 MT6826S，转向电机输出轴角度反馈。 */
  SteerAngleCtrl_State_t state;      /* 当前闭环状态。 */
  float target_angle_deg;            /* 目标电机输出轴角度，单位 deg。 */
  float center_tolerance_deg;        /* 进入停止区的误差阈值，单位 deg。 */
  float restart_error_deg;           /* 已停机后允许重新启动的误差阈值，单位 deg。 */
  float max_abs_angle_deg;           /* 右左两侧允许的绝对输出轴角度上限。 */
  float kp_permille_per_deg;         /* 比例系数：每度误差对应的 PWM 千分比。 */
  float return_center_target_deg;     /* 目标角接近 0 时启用回中软着陆参数。 */
  float return_center_tolerance_deg;  /* 回中软着陆停止误差带，单位 deg。 */
  float return_center_restart_deg;    /* 回中软着陆重新启动误差带，单位 deg。 */
  int16_t min_duty_permille;         /* 克服减速电机静摩擦的最小低有效 PWM 千分比。 */
  int16_t runtime_min_duty_permille; /* 上层按机构角色设置的运行期最小 duty。 */
  int16_t max_duty_permille;         /* 闭环允许使用的最大低有效 PWM 千分比。 */
  int16_t return_center_max_duty_permille; /* 回中软着陆允许使用的最大 PWM 千分比。 */
  int16_t duty_ramp_step_permille;   /* 每个 10ms 任务周期允许变化的最大 PWM 千分比。 */
  int16_t output_duty_permille;      /* 当前经斜坡后的闭环 PWM 输出。 */
  uint16_t duty_scale_permille;      /* 运行期 duty 调节比例；1000 表示不缩放。 */
  uint8_t positive_error_reverse;    /* 正误差对应的电机方向，需装车标定。 */
  uint8_t calibrated;                /* 机械参数已标定时为 1。 */
  uint8_t stopped_in_tolerance;      /* 已进入停止误差带，等待误差超过重启阈值。 */
  uint8_t direction_change_pending;  /* 换向等待期间为 1，等待结束前保持刹车。 */
  uint8_t pending_reverse;           /* 当前等待生效的目标方向。 */
  uint32_t direction_change_started_ms; /* 最近一次切换方向的 HAL tick。 */
  uint32_t last_duty_ramp_ms;        /* 上一次闭环占空比斜坡更新的 HAL tick。 */
  uint32_t fault_count;              /* 闭环保护触发次数。 */
  uint32_t enable_count;             /* 控制器重新使能次数，用于定位上层门控抖动。 */
  uint32_t tolerance_stop_count;     /* 首次进入停止误差带的次数。 */
  uint32_t tolerance_restart_count;  /* 误差越过重启阈值后恢复运行的次数。 */
  uint32_t direction_change_count;   /* 因误差极性变化而切换方向的次数。 */
} SteerAngleCtrl_Handle_t;

void SteerAngleCtrl_Init(SteerAngleCtrl_Handle_t *ctrl,
                         SteerMotor_Handle_t *motor,
                         Mt6826sPwm_Handle_t *angle);
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
                              uint8_t positive_error_reverse);
HAL_StatusTypeDef SteerAngleCtrl_Enable(SteerAngleCtrl_Handle_t *ctrl);
void SteerAngleCtrl_Disable(SteerAngleCtrl_Handle_t *ctrl);
void SteerAngleCtrl_ClearFault(SteerAngleCtrl_Handle_t *ctrl);
HAL_StatusTypeDef SteerAngleCtrl_SetTargetDeg(SteerAngleCtrl_Handle_t *ctrl,
                                               float target_angle_deg);
void SteerAngleCtrl_SetDutyScalePermille(SteerAngleCtrl_Handle_t *ctrl,
                                          uint16_t scale_permille);
void SteerAngleCtrl_SetRuntimeMinDutyPermille(SteerAngleCtrl_Handle_t *ctrl,
                                               int16_t min_duty_permille);
void SteerAngleCtrl_Task(SteerAngleCtrl_Handle_t *ctrl, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STEER_ANGLE_CTRL_H */
