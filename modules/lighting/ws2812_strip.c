/**
  ******************************************************************************
  * @file    ws2812_strip.c
  * @brief   双实例 WS2812 PWM-DMA 驱动实现。
  ******************************************************************************
  */

#include "ws2812_strip.h"

#include "bsp_callback.h"
#include <string.h>

#define WS2812_STRIP_DMA_TIMEOUT_MS 10U

static HAL_TIM_ActiveChannel WS2812Strip_ChannelToActive(uint32_t channel)
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

static uint32_t WS2812Strip_ChannelToDmaRequest(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1:
      return TIM_DMA_CC1;
    case TIM_CHANNEL_2:
      return TIM_DMA_CC2;
    case TIM_CHANNEL_3:
      return TIM_DMA_CC3;
    case TIM_CHANNEL_4:
      return TIM_DMA_CC4;
    default:
      return 0U;
  }
}

static uint32_t WS2812Strip_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void WS2812Strip_ExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/** DMA 完成中断只关闭本通道 DMA 请求并把比较值拉到 0，不阻塞另一路 TIM1 通道。 */
static void WS2812Strip_PwmFinishedDispatch(void *parent, TIM_HandleTypeDef *htim)
{
  WS2812Strip_Handle_t *strip = (WS2812Strip_Handle_t *)parent;

  if ((strip == 0) || (htim != strip->htim) ||
      (htim->Channel != strip->active_channel))
  {
    return;
  }

  __HAL_TIM_DISABLE_DMA(htim, strip->dma_request);
  __HAL_TIM_SET_COMPARE(htim, strip->channel, 0U);
  strip->dma_busy = 0U;
  strip->done_count++;
}

HAL_StatusTypeDef WS2812Strip_Init(WS2812Strip_Handle_t *strip,
                                   TIM_HandleTypeDef *htim,
                                   uint32_t channel,
                                   uint16_t led_count,
                                   uint8_t reversed,
                                   uint16_t zero_compare,
                                   uint16_t one_compare)
{
  BspTimCallbackConfig callback_config;
  HAL_TIM_ActiveChannel active_channel;
  uint32_t dma_request;

  if ((strip == 0) || (htim == 0) || (led_count == 0U) ||
      (led_count > WS2812_STRIP_MAX_LED_COUNT) ||
      (zero_compare == 0U) || (one_compare <= zero_compare))
  {
    return HAL_ERROR;
  }

  active_channel = WS2812Strip_ChannelToActive(channel);
  dma_request = WS2812Strip_ChannelToDmaRequest(channel);
  if ((active_channel == HAL_TIM_ACTIVE_CHANNEL_CLEARED) || (dma_request == 0U))
  {
    return HAL_ERROR;
  }

  memset(strip, 0, sizeof(*strip));
  strip->htim = htim;
  strip->channel = channel;
  strip->active_channel = active_channel;
  strip->dma_request = dma_request;
  strip->led_count = led_count;
  strip->reversed = (reversed != 0U) ? 1U : 0U;
  strip->zero_compare = zero_compare;
  strip->one_compare = one_compare;

  memset(&callback_config, 0, sizeof(callback_config));
  callback_config.handle = htim;
  callback_config.parent = strip;
  callback_config.pwm_pulse_finished = WS2812Strip_PwmFinishedDispatch;
  if (BspCallback_RegisterTim(&callback_config) == 0U)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COMPARE(htim, channel, 0U);
  return HAL_OK;
}

void WS2812Strip_Task(WS2812Strip_Handle_t *strip, uint32_t now_ms)
{
  uint32_t primask;
  uint8_t timed_out = 0U;

  if ((strip == 0) || (strip->htim == 0))
  {
    return;
  }

  primask = WS2812Strip_EnterCritical();
  if ((strip->dma_busy != 0U) &&
      ((now_ms - strip->dma_start_ms) >= WS2812_STRIP_DMA_TIMEOUT_MS))
  {
    strip->dma_busy = 0U;
    strip->error_count++;
    timed_out = 1U;
  }
  WS2812Strip_ExitCritical(primask);

  if (timed_out != 0U)
  {
    /* 超时恢复放在主循环，允许 HAL 安全终止对应 DMA；正常路径不进入这里。 */
    (void)HAL_TIM_PWM_Stop_DMA(strip->htim, strip->channel);
    __HAL_TIM_SET_COMPARE(strip->htim, strip->channel, 0U);
  }
}

void WS2812Strip_SetPixelRgb(WS2812Strip_Handle_t *strip,
                             uint16_t index,
                             uint8_t r,
                             uint8_t g,
                             uint8_t b)
{
  if ((strip == 0) || (index >= strip->led_count))
  {
    return;
  }

  strip->pixels[index].r = r;
  strip->pixels[index].g = g;
  strip->pixels[index].b = b;
}

void WS2812Strip_FillRgb(WS2812Strip_Handle_t *strip,
                         uint8_t r,
                         uint8_t g,
                         uint8_t b)
{
  uint16_t i;

  if (strip == 0)
  {
    return;
  }

  for (i = 0U; i < strip->led_count; i++)
  {
    WS2812Strip_SetPixelRgb(strip, i, r, g, b);
  }
}

HAL_StatusTypeDef WS2812Strip_Refresh(WS2812Strip_Handle_t *strip)
{
  uint16_t physical_index;
  uint16_t logical_index;
  uint16_t *write_ptr;
  uint8_t bytes[3];
  uint8_t byte_index;
  uint8_t bit_index;
  uint32_t primask;
  HAL_StatusTypeDef status;

  if ((strip == 0) || (strip->htim == 0))
  {
    return HAL_ERROR;
  }

  WS2812Strip_Task(strip, HAL_GetTick());
  primask = WS2812Strip_EnterCritical();
  if (strip->dma_busy != 0U)
  {
    strip->busy_skip_count++;
    WS2812Strip_ExitCritical(primask);
    return HAL_BUSY;
  }
  strip->dma_busy = 1U;
  strip->dma_start_ms = HAL_GetTick();
  WS2812Strip_ExitCritical(primask);

  memset(strip->dma_buffer, 0, sizeof(strip->dma_buffer));
  write_ptr = &strip->dma_buffer[WS2812_STRIP_RESET_SLOT_COUNT];
  for (physical_index = 0U; physical_index < strip->led_count; physical_index++)
  {
    logical_index = (strip->reversed != 0U) ?
      (uint16_t)(strip->led_count - 1U - physical_index) : physical_index;
    bytes[0] = strip->pixels[logical_index].g;
    bytes[1] = strip->pixels[logical_index].r;
    bytes[2] = strip->pixels[logical_index].b;
    for (byte_index = 0U; byte_index < 3U; byte_index++)
    {
      for (bit_index = 0U; bit_index < 8U; bit_index++)
      {
        *write_ptr++ = ((bytes[byte_index] & (uint8_t)(0x80U >> bit_index)) != 0U) ?
          strip->one_compare : strip->zero_compare;
      }
    }
  }

  status = HAL_TIM_PWM_Start_DMA(strip->htim,
                                 strip->channel,
                                 (uint32_t *)strip->dma_buffer,
                                 (uint16_t)((WS2812_STRIP_RESET_SLOT_COUNT * 2U) +
                                            (strip->led_count * 24U)));
  if (status != HAL_OK)
  {
    primask = WS2812Strip_EnterCritical();
    strip->dma_busy = 0U;
    strip->error_count++;
    WS2812Strip_ExitCritical(primask);
    __HAL_TIM_SET_COMPARE(strip->htim, strip->channel, 0U);
  }
  return status;
}
