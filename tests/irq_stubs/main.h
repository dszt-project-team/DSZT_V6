#ifndef TEST_IRQ_MAIN_H
#define TEST_IRQ_MAIN_H
#include <stdint.h>
typedef struct { uint32_t SR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; uint32_t enabled; } UART_HandleTypeDef;
typedef struct { unsigned unused; } TIM_HandleTypeDef;
typedef struct { unsigned unused; } DMA_HandleTypeDef;
#define RESET 0U
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define UART_FLAG_IDLE 16U
#define UART_IT_IDLE 16U
#define TEST_UART_ERROR_FLAGS (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)
#define __HAL_UART_GET_FLAG(h, f) (((h)->Instance->SR & (f)) == (f))
#define __HAL_UART_GET_IT_SOURCE(h, f) ((h)->enabled & (f))
#define __HAL_UART_DISABLE_IT(h, f) ((h)->enabled &= ~(f))
/* 模拟 STM32F4 的读 SR/DR：清 IDLE 同时清 PE/FE/NE/ORE。 */
#define __HAL_UART_CLEAR_IDLEFLAG(h) ((h)->Instance->SR &= ~(UART_FLAG_IDLE | TEST_UART_ERROR_FLAGS))
void HAL_UART_IRQHandler(UART_HandleTypeDef *h);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *h);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h);
#endif
