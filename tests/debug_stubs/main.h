#ifndef TEST_DEBUG_MAIN_H
#define TEST_DEBUG_MAIN_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void Error_Handler(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
