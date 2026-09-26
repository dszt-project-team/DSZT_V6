#ifndef TEST_IRQ_MAIN_H
#define TEST_IRQ_MAIN_H
#include <stdint.h>
typedef struct { uint32_t flags, enabled; } UART_HandleTypeDef;
typedef struct { unsigned unused; } TIM_HandleTypeDef;
typedef struct { unsigned unused; } DMA_HandleTypeDef;
#define RESET 0U
#define UART_FLAG_IDLE 16U
#define UART_IT_IDLE 16U
#define __HAL_UART_GET_FLAG(h, f) ((h)->flags & (f))
#define __HAL_UART_GET_IT_SOURCE(h, f) ((h)->enabled & (f))
#define __HAL_UART_CLEAR_IDLEFLAG(h) ((h)->flags &= ~UART_FLAG_IDLE)
void HAL_UART_IRQHandler(UART_HandleTypeDef *h);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *h);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h);
#endif
