#ifndef RS485_TEST_MAIN_H
#define RS485_TEST_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum { RESET = 0, SET } FlagStatus;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;

#define UART_FLAG_TC 1U
#define __HAL_UART_GET_FLAG(handle, flag) ((void)(handle), (void)(flag), SET)

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t len, uint32_t timeout_ms);

#endif
