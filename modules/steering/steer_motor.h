/**
  ******************************************************************************
  * @file    steer_motor.h
  * @brief   5840-3650-2840 右左转向电机设备模块接口。
  ******************************************************************************
  */

#ifndef STEER_MOTOR_H
#define STEER_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
  TIM_HandleTypeDef *htim;       /* PWM 与 FG 输入捕获共用的定时器。右路 TIM4，左路 TIM8。 */
  uint32_t pwm_channel;          /* PWM 通道。右路 TIM4_CH1，左路 TIM8_CH1。 */
  uint32_t fg_channel;           /* FG 输入捕获通道。右路 TIM4_CH2，左路 TIM8_CH2。 */
  HAL_TIM_ActiveChannel active_fg_channel; /* HAL 回调里的 FG 活动通道。 */
  GPIO_TypeDef *dir_gpio_port;   /* FR/方向脚端口。 */
  uint16_t dir_gpio_pin;         /* FR/方向脚引脚。 */
  GPIO_TypeDef *brake_gpio_port; /* BRK/刹车脚端口。 */
  uint16_t brake_gpio_pin;       /* BRK/刹车脚引脚。 */
  IRQn_Type tim_irqn;            /* 该电机 FG 输入捕获所属中断。 */
  volatile uint32_t fg_pulse_count; /* FG 累计脉冲数；具体每转脉冲数以当前电机资料和实测为准。 */
  volatile uint32_t last_fg_ms;  /* 最近一次 FG 上升沿到达时的 HAL tick。 */
  int16_t duty_permille;         /* 当前低电平运行占空比，0~1000。 */
  uint8_t reverse;               /* 方向标志，0=CW，1=CCW；实际右左转方向需装车标定。 */
  uint8_t brake_on;              /* 刹车标志，1=刹车。 */
  uint8_t fg_capture_started;    /* FG 输入捕获中断已经启动时为 1。 */
} SteerMotor_Handle_t;

HAL_StatusTypeDef SteerMotor_Init(SteerMotor_Handle_t *motor,
                                  TIM_HandleTypeDef *htim,
                                  uint32_t pwm_channel,
                                  uint32_t fg_channel,
                                  GPIO_TypeDef *dir_gpio_port,
                                  uint16_t dir_gpio_pin,
                                  GPIO_TypeDef *brake_gpio_port,
                                  uint16_t brake_gpio_pin,
                                  IRQn_Type tim_irqn);
void SteerMotor_SetDuty(SteerMotor_Handle_t *motor, int16_t duty_permille);
void SteerMotor_SetDirection(SteerMotor_Handle_t *motor, uint8_t reverse);
void SteerMotor_SetBrake(SteerMotor_Handle_t *motor, uint8_t brake_on);
void SteerMotor_Stop(SteerMotor_Handle_t *motor);
void SteerMotor_FgCaptureCallback(SteerMotor_Handle_t *motor,
                                  TIM_HandleTypeDef *htim,
                                  uint32_t now_ms);
uint32_t SteerMotor_GetFgPulseCount(const SteerMotor_Handle_t *motor);
uint32_t SteerMotor_GetLastFgMs(const SteerMotor_Handle_t *motor);
uint8_t SteerMotor_IsFgActive(const SteerMotor_Handle_t *motor,
                              uint32_t now_ms,
                              uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* STEER_MOTOR_H */
