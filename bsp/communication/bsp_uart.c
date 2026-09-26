/** @file bsp_uart.c @brief STM32 UART 发送的最小 BSP 封装。 */
#include "bsp_uart.h"

HAL_StatusTypeDef BspUart_Init(BspUartPort *port,
                               UART_HandleTypeDef *huart,
                               uint32_t timeout_ms)
{
  if ((port == 0) || (huart == 0))
  {
    return HAL_ERROR;
  }
  port->huart = huart;
  port->timeout_ms = timeout_ms;
  return HAL_OK;
}

HAL_StatusTypeDef BspUart_Write(BspUartPort *port,
                                const uint8_t *data,
                                uint16_t length)
{
  if ((port == 0) || (port->huart == 0) || (data == 0) || (length == 0U))
  {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit(port->huart, (uint8_t *)data, length, port->timeout_ms);
}

uint8_t BspUart_IsTxReady(const BspUartPort *port)
{
  return (uint8_t)(((port != 0) &&
                    (port->huart != 0) &&
                    (port->huart->gState == HAL_UART_STATE_READY)) ? 1U : 0U);
}

HAL_StatusTypeDef BspUart_WriteAsync(BspUartPort *port,
                                     const uint8_t *data,
                                     uint16_t length)
{
  if ((port == 0) || (port->huart == 0) ||
      (data == 0) || (length == 0U))
  {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit_IT(port->huart, (uint8_t *)data, length);
}
