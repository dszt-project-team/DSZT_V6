/**
  ******************************************************************************
  * @file    ws2812_strip.h
  * @brief   基于定时器 PWM + DMA 的可复用 WS2812 灯带实例驱动。
  ******************************************************************************
  */

#ifndef WS2812_STRIP_H
#define WS2812_STRIP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define WS2812_STRIP_MAX_LED_COUNT 21U
#define WS2812_STRIP_RESET_SLOT_COUNT 80U
#define WS2812_STRIP_DMA_WORD_COUNT \
  ((WS2812_STRIP_RESET_SLOT_COUNT * 2U) + (WS2812_STRIP_MAX_LED_COUNT * 24U))

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} WS2812Strip_Color_t;

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  HAL_TIM_ActiveChannel active_channel;
  uint32_t dma_request;
  uint16_t led_count;
  uint16_t zero_compare;
  uint16_t one_compare;
  uint8_t reversed;
  WS2812Strip_Color_t pixels[WS2812_STRIP_MAX_LED_COUNT];
  uint16_t dma_buffer[WS2812_STRIP_DMA_WORD_COUNT];
  volatile uint8_t dma_busy;
  volatile uint8_t dma_recovering; /* 异步终止期间保持忙，DMA READY 且 EN=0 后才可复用缓存。 */
  volatile uint32_t dma_start_ms;
  volatile uint32_t dma_recovery_ms; /* 最近异步终止尝试时间，用于无阻塞的有界重试。 */
  volatile uint32_t done_count;
  volatile uint32_t busy_skip_count;
  volatile uint32_t error_count;
} WS2812Strip_Handle_t;

HAL_StatusTypeDef WS2812Strip_Init(WS2812Strip_Handle_t *strip,
                                   TIM_HandleTypeDef *htim,
                                   uint32_t channel,
                                   uint16_t led_count,
                                   uint8_t reversed,
                                   uint16_t zero_compare,
                                   uint16_t one_compare);
void WS2812Strip_Task(WS2812Strip_Handle_t *strip, uint32_t now_ms);
void WS2812Strip_SetPixelRgb(WS2812Strip_Handle_t *strip,
                             uint16_t index,
                             uint8_t r,
                             uint8_t g,
                             uint8_t b);
void WS2812Strip_FillRgb(WS2812Strip_Handle_t *strip,
                         uint8_t r,
                         uint8_t g,
                         uint8_t b);
HAL_StatusTypeDef WS2812Strip_Refresh(WS2812Strip_Handle_t *strip);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_STRIP_H */
