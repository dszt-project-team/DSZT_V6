#ifndef WS2812_TEST_MAIN_H
#define WS2812_TEST_MAIN_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0, HAL_TIM_ACTIVE_CHANNEL_1,
               HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3,
               HAL_TIM_ACTIVE_CHANNEL_4 } HAL_TIM_ActiveChannel;
typedef enum { HAL_DMA_STATE_RESET = 0, HAL_DMA_STATE_READY, HAL_DMA_STATE_BUSY,
               HAL_DMA_STATE_TIMEOUT, HAL_DMA_STATE_ERROR, HAL_DMA_STATE_ABORT } HAL_DMA_StateTypeDef;
typedef struct { uint32_t CR; } DMA_Stream_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; volatile HAL_DMA_StateTypeDef State; } DMA_HandleTypeDef;
typedef struct { uint32_t dummy; } UART_HandleTypeDef;
typedef struct { HAL_TIM_ActiveChannel Channel; DMA_HandleTypeDef *hdma[7];
                 uint32_t dma_requests; uint32_t compare[4]; } TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_DMA_ID_CC1 1U
#define TIM_DMA_ID_CC2 2U
#define TIM_DMA_ID_CC3 3U
#define TIM_DMA_ID_CC4 4U
#define TIM_DMA_CC1 1U
#define TIM_DMA_CC2 2U
#define TIM_DMA_CC3 4U
#define TIM_DMA_CC4 8U
#define DMA_SxCR_EN 1U
#define __HAL_TIM_DISABLE_DMA(timer, mask) ((timer)->dma_requests &= ~(mask))
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare[(channel) / 4U] = (value))
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t *data, uint16_t length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
