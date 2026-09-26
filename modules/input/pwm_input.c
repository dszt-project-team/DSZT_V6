/**
  ******************************************************************************
  * @file    pwm_input.c
  * @brief   飞控 Servo PWM 输入捕获、滤波和超时诊断实现。
  ******************************************************************************
  */

#include "pwm_input.h"

#include "bsp_callback.h"

#include <stddef.h>
#include <string.h>

#define PWM_INPUT_MAX_INSTANCES       2U
#define PWM_INPUT_MAX_AVERAGE_WINDOW  8U

struct PwmInput_Handle
{
  PwmInput_Config_t config;
  volatile uint16_t rising_capture;
  volatile uint16_t raw_us;
  volatile uint16_t filtered_us;
  volatile uint16_t last_invalid_us;
  volatile uint16_t samples[PWM_INPUT_MAX_AVERAGE_WINDOW];
  volatile uint32_t sample_sum;
  volatile uint32_t rising_edge_ms;
  volatile uint32_t last_valid_ms;
  volatile uint32_t last_invalid_ms;
  volatile uint32_t valid_pulse_count;
  volatile uint32_t invalid_pulse_count;
  volatile PwmInput_Fault_t fault;
  volatile uint8_t sample_index;
  volatile uint8_t sample_count;
  volatile uint8_t valid_streak;
  volatile uint8_t waiting_for_falling_edge;
  uint8_t initialized;
};

static PwmInput_Handle_t s_inputs[PWM_INPUT_MAX_INSTANCES];

static void PwmInput_ResetFilter(PwmInput_Handle_t *handle)
{
  handle->sample_sum = 0U;
  handle->sample_index = 0U;
  handle->sample_count = 0U;
  handle->valid_streak = 0U;
}

static HAL_TIM_ActiveChannel PwmInput_GetActiveChannel(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1: return HAL_TIM_ACTIVE_CHANNEL_1;
    case TIM_CHANNEL_2: return HAL_TIM_ACTIVE_CHANNEL_2;
    case TIM_CHANNEL_3: return HAL_TIM_ACTIVE_CHANNEL_3;
    case TIM_CHANNEL_4: return HAL_TIM_ACTIVE_CHANNEL_4;
    default: return HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  }
}

static uint8_t PwmInput_ConfigIsValid(const PwmInput_Config_t *config)
{
  if ((config == 0) || (config->timer == 0) ||
      (PwmInput_GetActiveChannel(config->channel) == HAL_TIM_ACTIVE_CHANNEL_CLEARED) ||
      (config->minimum_valid_us >= config->maximum_valid_us) ||
      (config->timeout_ms == 0U) ||
      (config->transient_fault_hold_ms == 0U) ||
      (config->transient_fault_hold_ms >= config->timeout_ms) ||
      (config->average_window == 0U) ||
      (config->average_window > PWM_INPUT_MAX_AVERAGE_WINDOW) ||
      (config->valid_samples_to_online == 0U))
  {
    return 0U;
  }
  return 1U;
}

static void PwmInput_PublishValidPulse(PwmInput_Handle_t *handle,
                                       uint16_t pulse_us,
                                       uint32_t now_ms)
{
  uint8_t window = handle->config.average_window;

  /* 断流后首帧可能先于任务侧超时检查到达，不能沿用断流前的均值与预热。 */
  if ((handle->valid_pulse_count != 0U) &&
      ((uint32_t)(now_ms - handle->last_valid_ms) > handle->config.timeout_ms))
  {
    PwmInput_ResetFilter(handle);
  }

  if (handle->sample_count < window)
  {
    handle->samples[handle->sample_index] = pulse_us;
    handle->sample_sum += pulse_us;
    handle->sample_count++;
  }
  else
  {
    handle->sample_sum -= handle->samples[handle->sample_index];
    handle->samples[handle->sample_index] = pulse_us;
    handle->sample_sum += pulse_us;
  }
  handle->sample_index++;
  if (handle->sample_index >= window)
  {
    handle->sample_index = 0U;
  }
  handle->raw_us = pulse_us;
  handle->filtered_us =
    (uint16_t)((handle->sample_sum + ((uint32_t)handle->sample_count / 2U)) /
               (uint32_t)handle->sample_count);
  handle->last_valid_ms = now_ms;
  handle->valid_pulse_count++;
  /* 在线预热与均值窗独立：MAIN1 可取最新值，但仍须连续有效帧确认。 */
  if (handle->valid_streak < handle->config.valid_samples_to_online)
  {
    handle->valid_streak++;
  }
  handle->fault = PWM_INPUT_FAULT_NONE;
}

static void PwmInput_PublishInvalidPulse(PwmInput_Handle_t *handle,
                                         uint16_t pulse_us,
                                         uint32_t now_ms)
{
  handle->raw_us = pulse_us;
  handle->last_invalid_us = pulse_us;
  handle->last_invalid_ms = now_ms;
  handle->invalid_pulse_count++;
  PwmInput_ResetFilter(handle);
  handle->fault = PWM_INPUT_FAULT_RANGE;
}

static void PwmInput_CaptureCallback(void *parent, TIM_HandleTypeDef *htim)
{
  PwmInput_Handle_t *handle = (PwmInput_Handle_t *)parent;
  uint16_t captured;
  uint16_t pulse_us;
  uint32_t now_ms;

  if ((handle == 0) || (handle->initialized == 0U) ||
      (handle->config.timer != htim) ||
      (PwmInput_GetActiveChannel(handle->config.channel) != htim->Channel))
  {
    return;
  }

  captured = (uint16_t)HAL_TIM_ReadCapturedValue(htim, handle->config.channel);
  now_ms = HAL_GetTick();
  if (handle->waiting_for_falling_edge == 0U)
  {
    handle->rising_capture = captured;
    handle->rising_edge_ms = now_ms;
    handle->waiting_for_falling_edge = 1U;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                 handle->config.channel,
                                 TIM_INPUTCHANNELPOLARITY_FALLING);
    return;
  }

  pulse_us = (uint16_t)(captured - handle->rising_capture);
  handle->waiting_for_falling_edge = 0U;
  __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                               handle->config.channel,
                               TIM_INPUTCHANNELPOLARITY_RISING);
  if ((pulse_us < handle->config.minimum_valid_us) ||
      (pulse_us > handle->config.maximum_valid_us) ||
      /* 防止高电平跨过 16 位计数器整圈后，截断差值伪装成合法短脉冲。 */
      ((uint32_t)(now_ms - handle->rising_edge_ms) >
       (((uint32_t)handle->config.maximum_valid_us + 999U) / 1000U)))
  {
    PwmInput_PublishInvalidPulse(handle, pulse_us, now_ms);
  }
  else
  {
    PwmInput_PublishValidPulse(handle, pulse_us, now_ms);
  }
}

PwmInput_Handle_t *PwmInput_Register(const PwmInput_Config_t *config)
{
  BspTimCallbackConfig callback_config;
  PwmInput_Handle_t *free_handle = 0;
  uint8_t index;

  if (PwmInput_ConfigIsValid(config) == 0U)
  {
    return 0;
  }
  for (index = 0U; index < PWM_INPUT_MAX_INSTANCES; index++)
  {
    PwmInput_Handle_t *handle = &s_inputs[index];
    if ((handle->initialized != 0U) &&
        (handle->config.timer == config->timer) &&
        (handle->config.channel == config->channel))
    {
      return 0;
    }
    if ((free_handle == 0) && (handle->initialized == 0U))
    {
      free_handle = handle;
    }
  }
  if (free_handle == 0)
  {
    return 0;
  }

  memset(free_handle, 0, sizeof(*free_handle));
  free_handle->config = *config;
  free_handle->fault = PWM_INPUT_FAULT_NOT_READY;
  memset(&callback_config, 0, sizeof(callback_config));
  callback_config.handle = config->timer;
  callback_config.parent = free_handle;
  callback_config.input_capture = PwmInput_CaptureCallback;
  if (BspCallback_RegisterTim(&callback_config) == 0U)
  {
    memset(free_handle, 0, sizeof(*free_handle));
    return 0;
  }

  __HAL_TIM_SET_CAPTUREPOLARITY(config->timer,
                               config->channel,
                               TIM_INPUTCHANNELPOLARITY_RISING);
  if (HAL_TIM_IC_Start_IT(config->timer, config->channel) != HAL_OK)
  {
    memset(free_handle, 0, sizeof(*free_handle));
    return 0;
  }
  free_handle->initialized = 1U;
  return free_handle;
}

uint8_t PwmInput_GetSnapshot(PwmInput_Handle_t *handle,
                             PwmInput_Snapshot_t *snapshot,
                             uint32_t now_ms)
{
  uint32_t primask;
  uint32_t valid_age_ms;
  uint32_t invalid_age_ms;
  uint8_t valid_streak;

  if ((handle == 0) || (snapshot == 0) || (handle->initialized == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  /* 任务传入的时间可能早于刚到的 ISR，必须在同一快照临界区重新取时。 */
  now_ms = HAL_GetTick();
  snapshot->raw_us = handle->raw_us;
  snapshot->filtered_us = handle->filtered_us;
  snapshot->last_invalid_us = handle->last_invalid_us;
  snapshot->last_valid_ms = handle->last_valid_ms;
  snapshot->last_invalid_ms = handle->last_invalid_ms;
  snapshot->valid_pulse_count = handle->valid_pulse_count;
  snapshot->invalid_pulse_count = handle->invalid_pulse_count;
  snapshot->fault = handle->fault;
  valid_streak = handle->valid_streak;
  valid_age_ms = (uint32_t)(now_ms - snapshot->last_valid_ms);
  invalid_age_ms = (uint32_t)(now_ms - snapshot->last_invalid_ms);
  snapshot->online = 0U;
  if (snapshot->valid_pulse_count == 0U)
  {
    snapshot->fault = ((snapshot->invalid_pulse_count != 0U) &&
                       (invalid_age_ms <= handle->config.transient_fault_hold_ms)) ?
                        PWM_INPUT_FAULT_RANGE : PWM_INPUT_FAULT_NOT_READY;
  }
  else if (valid_age_ms > handle->config.timeout_ms)
  {
    snapshot->fault = PWM_INPUT_FAULT_TIMEOUT;
    /* 仅首次进入超时复位边沿；重复轮询不能打断恢复中的新脉冲。 */
    if (handle->fault != PWM_INPUT_FAULT_TIMEOUT)
    {
      PwmInput_ResetFilter(handle);
      handle->waiting_for_falling_edge = 0U;
      handle->fault = PWM_INPUT_FAULT_TIMEOUT;
      __HAL_TIM_SET_CAPTUREPOLARITY(handle->config.timer,
                                   handle->config.channel,
                                   TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
  else if ((snapshot->invalid_pulse_count != 0U) &&
           (invalid_age_ms <= handle->config.transient_fault_hold_ms))
  {
    snapshot->fault = PWM_INPUT_FAULT_RANGE;
  }
  else if (valid_streak < handle->config.valid_samples_to_online)
  {
    snapshot->fault = PWM_INPUT_FAULT_WARMUP;
  }
  else
  {
    snapshot->fault = PWM_INPUT_FAULT_NONE;
    snapshot->online = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1U;
}
