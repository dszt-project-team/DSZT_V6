/** @file bsp_uart.h @brief STM32 UART 发送的最小 BSP 封装。 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdint.h>

typedef struct
{
  UART_HandleTypeDef *huart;
  uint32_t timeout_ms;
} BspUartPort;

HAL_StatusTypeDef BspUart_Init(BspUartPort *port,
                               UART_HandleTypeDef *huart,
                               uint32_t timeout_ms);
HAL_StatusTypeDef BspUart_Write(BspUartPort *port,
                                const uint8_t *data,
                                uint16_t length);
uint8_t BspUart_IsTxReady(const BspUartPort *port);
HAL_StatusTypeDef BspUart_WriteAsync(BspUartPort *port,
                                     const uint8_t *data,
                                     uint16_t length);

#endif /* BSP_UART_H */
