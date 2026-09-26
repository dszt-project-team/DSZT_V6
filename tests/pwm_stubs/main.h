#ifndef PWM_TEST_MAIN_H
#define PWM_TEST_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0,
  HAL_TIM_ACTIVE_CHANNEL_1 = 1,
  HAL_TIM_ACTIVE_CHANNEL_2 = 2,
  HAL_TIM_ACTIVE_CHANNEL_3 = 4,
  HAL_TIM_ACTIVE_CHANNEL_4 = 8
} HAL_TIM_ActiveChannel;
typedef struct { HAL_TIM_ActiveChannel Channel; } TIM_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_INPUTCHANNELPOLARITY_RISING 0U
#define TIM_INPUTCHANNELPOLARITY_FALLING 1U
#define __HAL_TIM_SET_CAPTUREPOLARITY(timer, channel, polarity) \
  test_set_polarity((timer), (channel), (polarity))

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel);
void test_set_polarity(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t polarity);

#endif
