/**
  ******************************************************************************
  * @file    mt6826s_pwm.c
  * @brief   MT6826S OUT/PWM 绝对角度编码器模块实现。
  ******************************************************************************
  */

#include "mt6826s_pwm.h"
#include "bsp_callback.h"
#include <string.h>

static void Mt6826sPwm_DispatchCapture(void *parent, TIM_HandleTypeDef *htim)
{
  Mt6826sPwm_IcCaptureCallback((Mt6826sPwm_Handle_t *)parent, htim);
}

static HAL_TIM_ActiveChannel Mt6826sPwm_ChannelToActive(uint32_t channel)
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

static void Mt6826sPwm_EnableGpioClock(GPIO_TypeDef *gpio_port)
{
  if (gpio_port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (gpio_port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (gpio_port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (gpio_port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (gpio_port == GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  else if (gpio_port == GPIOF)
  {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  }
  else if (gpio_port == GPIOG)
  {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
  else if (gpio_port == GPIOH)
  {
    __HAL_RCC_GPIOH_CLK_ENABLE();
  }
  else if (gpio_port == GPIOI)
  {
    __HAL_RCC_GPIOI_CLK_ENABLE();
  }
}

static int32_t Mt6826sPwm_ComputeRelativeDegX10(uint16_t raw_angle,
                                                 uint16_t zero_raw_angle,
                                                 uint8_t direction_inverted)
{
  int32_t relative_count;
  int32_t scaled;

  relative_count = (int32_t)((uint16_t)((raw_angle - zero_raw_angle) & 0x0FFFU));
  if (relative_count > (int32_t)(MT6826S_PWM_FULL_SCALE_COUNT / 2U - 1U))
  {
    relative_count -= (int32_t)MT6826S_PWM_FULL_SCALE_COUNT;
  }
  if (direction_inverted != 0U)
  {
    relative_count = -relative_count;
  }

  scaled = relative_count * 3600;
  if (scaled >= 0)
  {
    return (scaled + (int32_t)(MT6826S_PWM_FULL_SCALE_COUNT / 2U)) /
           (int32_t)MT6826S_PWM_FULL_SCALE_COUNT;
  }

  return (scaled - (int32_t)(MT6826S_PWM_FULL_SCALE_COUNT / 2U)) /
         (int32_t)MT6826S_PWM_FULL_SCALE_COUNT;
}

/**
  * @brief 计算两个 12 位绝对角之间的最短圆周距离。
  * @note  4095 -> 0 的正常环回距离为 1，而不是 4095。
  */
static uint32_t Mt6826sPwm_CircularDistance(uint16_t first,
                                            uint16_t second)
{
  int32_t delta;

  delta = (int32_t)first - (int32_t)second;
  if (delta > (int32_t)(MT6826S_PWM_FULL_SCALE_COUNT / 2U))
  {
    delta -= (int32_t)MT6826S_PWM_FULL_SCALE_COUNT;
  }
  else if (delta < -(int32_t)(MT6826S_PWM_FULL_SCALE_COUNT / 2U))
  {
    delta += (int32_t)MT6826S_PWM_FULL_SCALE_COUNT;
  }

  return (uint32_t)((delta < 0) ? -delta : delta);
}

/**
  * @brief 检查候选角度是否满足转向输出轴的物理连续性。
  * @note  只比较最近 100ms 内的有效样本。若输入曾经真正超时，则允许首个恢复帧
  *        重新建立基准，避免机构被外力移动后永久拒绝新位置。
  */
static uint8_t Mt6826sPwm_IsCandidateContinuous(const Mt6826sPwm_Handle_t *sensor,
                                                 uint16_t candidate_raw,
                                                 uint32_t now_ms)
{
  uint32_t elapsed_ms;
  uint32_t allowed_count;

  if ((sensor == 0) ||
      (sensor->sample_count == 0U))
  {
    return 1U;
  }

  elapsed_ms = now_ms - sensor->last_update_ms;
  if (elapsed_ms > MT6826S_PWM_HEALTH_TIMEOUT_MS)
  {
    return 1U;
  }

  allowed_count = MT6826S_PWM_CONTINUITY_MARGIN_COUNT +
                  MT6826S_PWM_CONTINUITY_MAX_COUNT_PER_MS * elapsed_ms;
  if (allowed_count > (MT6826S_PWM_FULL_SCALE_COUNT / 2U))
  {
    allowed_count = MT6826S_PWM_FULL_SCALE_COUNT / 2U;
  }

  return (Mt6826sPwm_CircularDistance(candidate_raw, sensor->raw_angle) <=
          allowed_count) ? 1U : 0U;
}

static uint8_t Mt6826sPwm_IsTimestampFresh(uint32_t now_ms,
                                           uint32_t timestamp_ms,
                                           uint32_t timeout_ms)
{
  /* 调用方在临界区内同步采样当前 tick 与反馈；无符号差正确跨越 tick
   * 回绕，不把回绕前的旧数据误认为“未来新数据”。online 判断是否收到过帧，
   * 因此时间戳 0 仍是有效时刻，不作为无数据标志。 */
  return ((uint32_t)(now_ms - timestamp_ms) <= timeout_ms) ? 1U : 0U;
}

static void Mt6826sPwm_ProcessFrame(Mt6826sPwm_Handle_t *sensor,
                                    uint32_t high_ticks_us,
                                    uint32_t period_ticks_us,
                                    uint32_t now_ms)
{
  uint32_t encoded_ticks;
  uint32_t raw_angle;

  if ((period_ticks_us < MT6826S_PWM_PERIOD_MIN_US) ||
      (period_ticks_us > MT6826S_PWM_PERIOD_MAX_US) ||
      (high_ticks_us == 0U) ||
      (high_ticks_us >= period_ticks_us))
  {
    sensor->invalid_count++;
    return;
  }

  /*
   * MT6826S 默认高电平有效：高电平宽度 = 固定 16 ticks + 12bit 角度数据。
   * 周期由实测 period 归一化，这样 497Hz/994Hz 两档都可用同一套公式。
   */
  encoded_ticks = ((high_ticks_us * MT6826S_PWM_FRAME_TICKS) +
                   (period_ticks_us / 2U)) / period_ticks_us;
  if ((encoded_ticks < MT6826S_PWM_START_HIGH_TICKS) ||
      (encoded_ticks > (MT6826S_PWM_START_HIGH_TICKS +
                        MT6826S_PWM_DATA_TICKS)))
  {
    sensor->invalid_count++;
    return;
  }

  raw_angle = encoded_ticks - MT6826S_PWM_START_HIGH_TICKS;
  if (Mt6826sPwm_IsCandidateContinuous(sensor,
                                        (uint16_t)(raw_angle & 0x0FFFU),
                                        now_ms) == 0U)
  {
    sensor->invalid_count++;
    sensor->continuity_reject_count++;
    return;
  }
  sensor->high_ticks_us = high_ticks_us;
  sensor->period_ticks_us = period_ticks_us;
  sensor->raw_angle = (uint16_t)(raw_angle & 0x0FFFU);
  if (sensor->zero_valid != 0U)
  {
    sensor->relative_deg_x10 =
      Mt6826sPwm_ComputeRelativeDegX10(sensor->raw_angle,
                                       sensor->zero_raw_angle,
                                       sensor->direction_inverted);
  }
  else
  {
    sensor->relative_deg_x10 = 0;
  }
  sensor->duty_permille = (uint16_t)(((high_ticks_us * 1000U) +
                                      (period_ticks_us / 2U)) /
                                     period_ticks_us);
  sensor->last_update_ms = now_ms;
  sensor->sample_count++;
  sensor->online = 1U;
}

HAL_StatusTypeDef Mt6826sPwm_Init(Mt6826sPwm_Handle_t *sensor,
                                  TIM_HandleTypeDef *htim,
                                  uint32_t channel,
                                  GPIO_TypeDef *gpio_port,
                                  uint16_t gpio_pin,
                                  uint32_t gpio_alternate)
{
  BspTimCallbackConfig callback_config;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  HAL_TIM_ActiveChannel active_channel;

  if ((sensor == 0) || (htim == 0) || (gpio_port == 0))
  {
    return HAL_ERROR;
  }

  active_channel = Mt6826sPwm_ChannelToActive(channel);
  if (active_channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED)
  {
    return HAL_ERROR;
  }

  memset(sensor, 0, sizeof(*sensor));
  sensor->htim = htim;
  sensor->channel = channel;
  sensor->active_channel = active_channel;
  sensor->gpio_port = gpio_port;
  sensor->gpio_pin = gpio_pin;

  /*
   * 本驱动不写死具体接口：右路由 J26-6/PH11/TIM5_CH2 采集，左路由 J31-1/PA0/TIM2_CH1 采集。
   * 这里仅按调用方传入的 GPIO/AF/定时器通道完成 PWM 输入捕获初始化。
   */
  /* 当前接线约定：右路 J26-6/PH11/TIM5_CH2，左路 J31-1/PA0/TIM2_CH1。 */
  Mt6826sPwm_EnableGpioClock(gpio_port);
  GPIO_InitStruct.Pin = gpio_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = gpio_alternate;
  HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0U;
  if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, channel) != HAL_OK)
  {
    return HAL_ERROR;
  }

  memset(&callback_config, 0, sizeof(callback_config));
  callback_config.handle = htim;
  callback_config.parent = sensor;
  callback_config.input_capture = Mt6826sPwm_DispatchCapture;
  if (BspCallback_RegisterTim(&callback_config) == 0U)
  {
    return HAL_ERROR;
  }

  return HAL_TIM_IC_Start_IT(htim, channel);
}

void Mt6826sPwm_Task(Mt6826sPwm_Handle_t *sensor, uint32_t now_ms)
{
  uint32_t primask;

  if (sensor == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  now_ms = HAL_GetTick();
  if ((sensor->online != 0U) &&
      (Mt6826sPwm_IsTimestampFresh(now_ms,
                                   sensor->last_update_ms,
                                   MT6826S_PWM_HEALTH_TIMEOUT_MS) == 0U))
  {
    sensor->online = 0U;
    sensor->timeout_count++;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Mt6826sPwm_IcCaptureCallback(Mt6826sPwm_Handle_t *sensor,
                                  TIM_HandleTypeDef *htim)
{
  GPIO_PinState level;
  uint32_t capture;
  uint32_t high_ticks_us;
  uint32_t period_ticks_us;
  uint32_t now_ms;

  if ((sensor == 0) || (htim != sensor->htim) ||
      (htim->Channel != sensor->active_channel))
  {
    return;
  }

  capture = HAL_TIM_ReadCapturedValue(sensor->htim, sensor->channel);
  now_ms = HAL_GetTick();
  level = HAL_GPIO_ReadPin(sensor->gpio_port, sensor->gpio_pin);
  sensor->last_edge_ms = now_ms;
  sensor->edge_count++;

  if (level == GPIO_PIN_SET)
  {
    if ((sensor->have_rise != 0U) && (sensor->have_fall != 0U))
    {
      period_ticks_us = capture - sensor->rise_tick_us;
      high_ticks_us = sensor->fall_tick_us - sensor->rise_tick_us;
      Mt6826sPwm_ProcessFrame(sensor, high_ticks_us, period_ticks_us, now_ms);
    }

    sensor->rise_tick_us = capture;
    sensor->have_rise = 1U;
    sensor->have_fall = 0U;
  }
  else if (sensor->have_rise != 0U)
  {
    sensor->fall_tick_us = capture;
    sensor->have_fall = 1U;
  }
}

uint8_t Mt6826sPwm_IsHealthy(const Mt6826sPwm_Handle_t *sensor,
                             uint32_t now_ms)
{
  uint32_t primask;
  uint8_t healthy;

  if (sensor == 0)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  now_ms = HAL_GetTick();
  healthy = (uint8_t)((sensor->online != 0U) &&
      (Mt6826sPwm_IsTimestampFresh(now_ms, sensor->last_update_ms,
                                  MT6826S_PWM_HEALTH_TIMEOUT_MS) != 0U));
  if (primask == 0U)
  {
    __enable_irq();
  }
  return healthy;
}

void Mt6826sPwm_SetDirectionInverted(Mt6826sPwm_Handle_t *sensor,
                                     uint8_t inverted)
{
  uint32_t primask;

  if (sensor == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  sensor->direction_inverted = (inverted != 0U) ? 1U : 0U;
  if (sensor->zero_valid != 0U)
  {
    sensor->relative_deg_x10 =
      Mt6826sPwm_ComputeRelativeDegX10(sensor->raw_angle,
                                       sensor->zero_raw_angle,
                                       sensor->direction_inverted);
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Mt6826sPwm_SetZeroRaw(Mt6826sPwm_Handle_t *sensor,
                           uint16_t raw_angle)
{
  uint32_t primask;

  if (sensor == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  sensor->zero_raw_angle = (uint16_t)(raw_angle & 0x0FFFU);
  sensor->zero_valid = 1U;
  sensor->relative_deg_x10 =
    Mt6826sPwm_ComputeRelativeDegX10(sensor->raw_angle,
                                     sensor->zero_raw_angle,
                                     sensor->direction_inverted);
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Mt6826sPwm_SetZeroAtCurrent(Mt6826sPwm_Handle_t *sensor)
{
  uint16_t raw_angle;
  uint32_t primask;

  if (sensor == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  raw_angle = sensor->raw_angle;
  if (primask == 0U)
  {
    __enable_irq();
  }

  Mt6826sPwm_SetZeroRaw(sensor, raw_angle);
}

uint8_t Mt6826sPwm_HasZero(const Mt6826sPwm_Handle_t *sensor)
{
  if ((sensor == 0) || (sensor->zero_valid == 0U))
  {
    return 0U;
  }

  return 1U;
}

void Mt6826sPwm_GetSnapshot(const Mt6826sPwm_Handle_t *sensor,
                            uint32_t now_ms,
                            Mt6826sPwm_Snapshot_t *snapshot)
{
  uint32_t primask;
  uint16_t raw_angle;

  if (snapshot == 0)
  {
    return;
  }

  memset(snapshot, 0, sizeof(*snapshot));
  if (sensor == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  now_ms = HAL_GetTick();
  snapshot->online = sensor->online;
  snapshot->raw_angle = sensor->raw_angle;
  snapshot->zero_valid = sensor->zero_valid;
  snapshot->direction_inverted = sensor->direction_inverted;
  snapshot->zero_raw_angle = sensor->zero_raw_angle;
  snapshot->relative_deg_x10 = sensor->relative_deg_x10;
  snapshot->duty_permille = sensor->duty_permille;
  snapshot->high_ticks_us = sensor->high_ticks_us;
  snapshot->period_ticks_us = sensor->period_ticks_us;
  snapshot->sample_count = sensor->sample_count;
  snapshot->edge_count = sensor->edge_count;
  snapshot->invalid_count = sensor->invalid_count;
  snapshot->continuity_reject_count = sensor->continuity_reject_count;
  snapshot->timeout_count = sensor->timeout_count;
  snapshot->last_update_ms = sensor->last_update_ms;
  snapshot->last_edge_ms = sensor->last_edge_ms;
  if (primask == 0U)
  {
    __enable_irq();
  }
  snapshot->pin_level =
    (HAL_GPIO_ReadPin(sensor->gpio_port, sensor->gpio_pin) == GPIO_PIN_SET) ? 1U : 0U;

  raw_angle = snapshot->raw_angle;
  snapshot->healthy =
    ((snapshot->online != 0U) &&
     (Mt6826sPwm_IsTimestampFresh(now_ms,
                                  snapshot->last_update_ms,
                                  MT6826S_PWM_HEALTH_TIMEOUT_MS) != 0U)) ? 1U : 0U;
  snapshot->raw_deg_x10 =
    (int32_t)(((uint32_t)raw_angle * 3600U +
               (MT6826S_PWM_FULL_SCALE_COUNT / 2U)) /
              MT6826S_PWM_FULL_SCALE_COUNT);
  snapshot->relative_angle_deg = (float)snapshot->relative_deg_x10 * 0.1f;
}
