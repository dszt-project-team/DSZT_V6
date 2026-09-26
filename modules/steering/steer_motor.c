/**
  ******************************************************************************
  * @file    steer_motor.c
  * @brief   5840-3650-2840 右左转向电机设备模块实现。
  ******************************************************************************
  */

#include "steer_motor.h"
#include "bsp_callback.h"
#include <string.h>

static void SteerMotor_DispatchFgCapture(void *parent, TIM_HandleTypeDef *htim)
{
  SteerMotor_FgCaptureCallback((SteerMotor_Handle_t *)parent, htim, HAL_GetTick());
}

static HAL_TIM_ActiveChannel SteerMotor_ChannelToActive(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1:
      return HAL_TIM_ACTIVE_CHANNEL_1;
    case TIM_CHANNEL_2:
      return HAL_TIM_ACTIVE_CHANNEL_2;
    case TIM_CHANNEL_3:
      return HAL_TIM_ACTIVE_CHANNEL_3;
    case TIM_CHANNEL_4:
      return HAL_TIM_ACTIVE_CHANNEL_4;
    default:
      return HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  }
}

/**
  * @brief 初始化转向电机控制对象。
  * @param motor       电机控制对象。
  * @param htim        PWM 定时器句柄。
  * @param pwm_channel PWM 通道。
  * @param fg_channel  FG 输入捕获通道。
  * @return HAL 状态。
  * @note  上电后默认 0% 运行占空比、保持 BRK 刹车、默认 CW 方向，避免转向机构突然动作。
  *        当前右左 5840-3650-2840 蓝线 PWM 均按低电平有效运行；右路 TIM4_CH1/J26
  *        为 25kHz，左路 TIM8_CH1/J31 恢复 V4 实际配置 50kHz。黄线 FG 电平形式以当前批次资料和
  *        台架实测为准，整车接线仍建议串联限流电阻并预留施密特整形和 ESD 焊盘。
  */
HAL_StatusTypeDef SteerMotor_Init(SteerMotor_Handle_t *motor,
                                  TIM_HandleTypeDef *htim,
                                  uint32_t pwm_channel,
                                  uint32_t fg_channel,
                                  GPIO_TypeDef *dir_gpio_port,
                                  uint16_t dir_gpio_pin,
                                  GPIO_TypeDef *brake_gpio_port,
                                  uint16_t brake_gpio_pin,
                                  IRQn_Type tim_irqn)
{
  BspTimCallbackConfig callback_config;
  HAL_TIM_ActiveChannel active_fg_channel;

  if ((motor == 0) || (htim == 0) ||
      (dir_gpio_port == 0) || (brake_gpio_port == 0))
  {
    return HAL_ERROR;
  }
  active_fg_channel = SteerMotor_ChannelToActive(fg_channel);
  if (active_fg_channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED)
  {
    return HAL_ERROR;
  }

  memset(motor, 0, sizeof(*motor));
  motor->htim = htim;
  motor->pwm_channel = pwm_channel;
  motor->fg_channel = fg_channel;
  motor->active_fg_channel = active_fg_channel;
  motor->dir_gpio_port = dir_gpio_port;
  motor->dir_gpio_pin = dir_gpio_pin;
  motor->brake_gpio_port = brake_gpio_port;
  motor->brake_gpio_pin = brake_gpio_pin;
  motor->tim_irqn = tim_irqn;

  SteerMotor_SetDirection(motor, 0U);
  SteerMotor_SetDuty(motor, 0);
  if (HAL_TIM_PWM_Start(motor->htim, motor->pwm_channel) != HAL_OK)
  {
    return HAL_ERROR;
  }
  SteerMotor_SetBrake(motor, 1U);

  /*
   * NVIC 仅服务本驱动的 FG 输入捕获。右路为 TIM4_IRQn，左路为 TIM8_CC_IRQn。
   * 中断入口由 CubeMX 生成在 Core/Src/stm32f4xx_it.c，应用层只保留 HAL 捕获回调。
   */
  HAL_NVIC_SetPriority(motor->tim_irqn, 5U, 0U);
  HAL_NVIC_EnableIRQ(motor->tim_irqn);
  memset(&callback_config, 0, sizeof(callback_config));
  callback_config.handle = htim;
  callback_config.parent = motor;
  callback_config.input_capture = SteerMotor_DispatchFgCapture;
  if (BspCallback_RegisterTim(&callback_config) == 0U)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_IC_Start_IT(motor->htim, motor->fg_channel) != HAL_OK)
  {
    return HAL_ERROR;
  }
  motor->fg_capture_started = 1U;
  return HAL_OK;
}

/**
  * @brief 设置转向电机 PWM 运行占空比。
  * @param motor         电机控制对象。
  * @param duty_permille 低电平运行占空比千分比，0~1000；0=全高停止，1000=全低运行。
  */
void SteerMotor_SetDuty(SteerMotor_Handle_t *motor, int16_t duty_permille)
{
  uint32_t period;
  uint32_t active_low_ticks;
  uint32_t compare_high_ticks;

  if ((motor == 0) || (motor->htim == 0))
  {
    return;
  }

  if (duty_permille < 0)
  {
    duty_permille = 0;
  }
  if (duty_permille > 1000)
  {
    duty_permille = 1000;
  }

  /*
   * 低电平有效试验版：外部参数仍按“运行占空比”理解，但真正输出到蓝线的是低电平占空比。
   * TIM4 使用 PWM1 + 高极性，CCR 表示高电平时间；因此 0% 运行占空比要输出全高，
   * 30% 运行占空比要输出 70% 高 + 30% 低。
   */
  period = __HAL_TIM_GET_AUTORELOAD(motor->htim) + 1U;
  active_low_ticks = (period * (uint32_t)duty_permille) / 1000U;
  compare_high_ticks = period - active_low_ticks;
  __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, compare_high_ticks);
  motor->duty_permille = duty_permille;
}

/**
  * @brief 设置转向电机方向。
  * @param motor   电机控制对象。
  * @param reverse 0=CW 电气方向；1=CCW 电气方向。
  * @note  官方参数图说明白线方向脚：悬空=CCW，低电平(<=0.6V)=CW。
  *        PD14 配置为开漏输出；写 SET 表示释放线路，由电机内部上拉形成高电平。
  *        实际右左转方向仍需要结合装车机构标定。
  */
void SteerMotor_SetDirection(SteerMotor_Handle_t *motor, uint8_t reverse)
{
  if (motor == 0)
  {
    return;
  }

  /* 官方参数图：白线悬空=CCW，低电平(<=0.6V)=CW；实际右左转方向仍由机构标定。 */
  HAL_GPIO_WritePin(motor->dir_gpio_port,
                    motor->dir_gpio_pin,
                    reverse ? GPIO_PIN_SET : GPIO_PIN_RESET);
  motor->reverse = reverse ? 1U : 0U;
}

/**
  * @brief 设置转向电机刹车。
  * @param motor    电机控制对象。
  * @param brake_on 1=刹车；0=释放刹车。
  * @note  线序图说明绿线刹车脚：高电平或悬空=运行，低电平=停止/刹车。
  *        PD15 配置为开漏输出；写 SET 表示释放线路，由电机内部上拉形成高电平。
  */
void SteerMotor_SetBrake(SteerMotor_Handle_t *motor, uint8_t brake_on)
{
  if (motor == 0)
  {
    return;
  }

  HAL_GPIO_WritePin(motor->brake_gpio_port,
                    motor->brake_gpio_pin,
                    brake_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  motor->brake_on = brake_on ? 1U : 0U;
}

/**
  * @brief 停止转向电机输出。
  * @param motor 电机控制对象。
  * @note  先把蓝线 PWM 置为全高停止，再按电机刹车脚逻辑拉低绿线 BRK。
  */
void SteerMotor_Stop(SteerMotor_Handle_t *motor)
{
  if (motor == 0)
  {
    return;
  }

  SteerMotor_SetDuty(motor, 0);
  SteerMotor_SetBrake(motor, 1U);
}

/**
  * @brief 在 HAL 输入捕获回调中记录转向电机黄线 FG 上升沿。
  * @param motor  转向电机控制对象。
  * @param htim   触发输入捕获中断的定时器句柄。
  * @param now_ms 当前 HAL tick。
  * @note  中断中只做常量时间记录，不进行浮点计算、日志输出或阻塞调用。
  */
void SteerMotor_FgCaptureCallback(SteerMotor_Handle_t *motor,
                                  TIM_HandleTypeDef *htim,
                                  uint32_t now_ms)
{
  if ((motor == 0) || (htim != motor->htim) ||
      (htim->Channel != motor->active_fg_channel))
  {
    return;
  }

  motor->fg_pulse_count++;
  motor->last_fg_ms = now_ms;
}

/**
  * @brief 获取转向电机黄线 FG 累计脉冲数。
  */
uint32_t SteerMotor_GetFgPulseCount(const SteerMotor_Handle_t *motor)
{
  return (motor == 0) ? 0U : motor->fg_pulse_count;
}

/**
  * @brief 获取最近一次 FG 上升沿到达时间。
  */
uint32_t SteerMotor_GetLastFgMs(const SteerMotor_Handle_t *motor)
{
  return (motor == 0) ? 0U : motor->last_fg_ms;
}

/**
  * @brief 判断最近一段时间内是否观察到 FG 脉冲。
  * @param motor      转向电机控制对象。
  * @param now_ms     当前 HAL tick。
  * @param timeout_ms 允许的 FG 静默时间。
  * @note  本函数只提供观测结果。堵转判定仍需结合占空比、动作方向、
  *        外部输出轴角度反馈和台架标定阈值，由上层安全策略统一处理。
  */
uint8_t SteerMotor_IsFgActive(const SteerMotor_Handle_t *motor,
                              uint32_t now_ms,
                              uint32_t timeout_ms)
{
  if ((motor == 0) || (motor->fg_capture_started == 0U) ||
      (motor->fg_pulse_count == 0U))
  {
    return 0U;
  }

  return ((now_ms - motor->last_fg_ms) <= timeout_ms) ? 1U : 0U;
}
