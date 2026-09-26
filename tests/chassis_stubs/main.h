#ifndef CHASSIS_TEST_MAIN_H
#define CHASSIS_TEST_MAIN_H
#include <stdint.h>
typedef enum { HAL_OK=0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
#endif
