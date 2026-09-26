/**
  ******************************************************************************
  * @file    bsp_callback.c
  * @brief   HAL 回调到 BSP/Module 实例的统一分发实现。
  ******************************************************************************
  */

#include "bsp_callback.h"

#include <string.h>

#define BSP_CALLBACK_UART_COUNT 8U
#define BSP_CALLBACK_TIM_COUNT  12U

static BspUartCallbackConfig s_uart_callbacks[BSP_CALLBACK_UART_COUNT];
static BspTimCallbackConfig s_tim_callbacks[BSP_CALLBACK_TIM_COUNT];
static uint8_t s_uart_count;
static uint8_t s_tim_count;

void BspCallback_Init(void)
{
  memset(s_uart_callbacks, 0, sizeof(s_uart_callbacks));
  memset(s_tim_callbacks, 0, sizeof(s_tim_callbacks));
  s_uart_count = 0U;
  s_tim_count = 0U;
}

uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *config)
{
  uint8_t i;

  if ((config == 0) || (config->handle == 0))
  {
    return 0U;
  }

  for (i = 0U; i < s_uart_count; i++)
  {
    if (s_uart_callbacks[i].handle == config->handle)
    {
      return 0U;
    }
  }
  if (s_uart_count >= BSP_CALLBACK_UART_COUNT)
  {
    return 0U;
  }

  s_uart_callbacks[s_uart_count] = *config;
  s_uart_count++;
  return 1U;
}

uint8_t BspCallback_RegisterTim(const BspTimCallbackConfig *config)
{
  if ((config == 0) || (config->handle == 0) ||
      (s_tim_count >= BSP_CALLBACK_TIM_COUNT))
  {
    return 0U;
  }

  /* 同一个定时器允许多个通道实例，因此不按 handle 去重。 */
  s_tim_callbacks[s_tim_count] = *config;
  s_tim_count++;
  return 1U;
}

void BspCallback_DispatchUartIdle(UART_HandleTypeDef *huart)
{
  uint8_t i;

  for (i = 0U; i < s_uart_count; i++)
  {
    if ((s_uart_callbacks[i].handle == huart) &&
        (s_uart_callbacks[i].idle != 0))
    {
      s_uart_callbacks[i].idle(s_uart_callbacks[i].parent, huart);
      return;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t i;

  for (i = 0U; i < s_uart_count; i++)
  {
    if ((s_uart_callbacks[i].handle == huart) &&
        (s_uart_callbacks[i].rx_complete != 0))
    {
      s_uart_callbacks[i].rx_complete(s_uart_callbacks[i].parent, huart);
      return;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint8_t i;

  for (i = 0U; i < s_uart_count; i++)
  {
    if ((s_uart_callbacks[i].handle == huart) &&
        (s_uart_callbacks[i].error != 0))
    {
      s_uart_callbacks[i].error(s_uart_callbacks[i].parent, huart);
      return;
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint8_t i;

  for (i = 0U; i < s_tim_count; i++)
  {
    if ((s_tim_callbacks[i].handle == htim) &&
        (s_tim_callbacks[i].input_capture != 0))
    {
      s_tim_callbacks[i].input_capture(s_tim_callbacks[i].parent, htim);
    }
  }
}

/**
  * @brief 分发定时器输出比较边沿事件。
  * @note  各输出比较实例需要自行检查 htim->Channel；输入捕获回调也按活动通道分发。
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t i;

  for (i = 0U; i < s_tim_count; i++)
  {
    if ((s_tim_callbacks[i].handle == htim) &&
        (s_tim_callbacks[i].output_compare != 0))
    {
      s_tim_callbacks[i].output_compare(s_tim_callbacks[i].parent, htim);
    }
  }
}

/**
  * @brief 分发 PWM DMA 整帧发送完成事件。
  * @note  同一定时器可以挂多个 PWM 通道实例；实例自行检查 htim->Channel，
  *        因此这里必须遍历全部匹配项，不能在命中第一项后提前返回。
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t i;

  for (i = 0U; i < s_tim_count; i++)
  {
    if ((s_tim_callbacks[i].handle == htim) &&
        (s_tim_callbacks[i].pwm_pulse_finished != 0))
    {
      s_tim_callbacks[i].pwm_pulse_finished(s_tim_callbacks[i].parent, htim);
    }
  }
}
