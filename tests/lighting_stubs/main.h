#ifndef LIGHTING_TEST_MAIN_H
#define LIGHTING_TEST_MAIN_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0, HAL_TIM_ACTIVE_CHANNEL_1,
               HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3,
               HAL_TIM_ACTIVE_CHANNEL_4 } HAL_TIM_ActiveChannel;
typedef struct { HAL_TIM_ActiveChannel Channel; } TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_4 12U
#endif
