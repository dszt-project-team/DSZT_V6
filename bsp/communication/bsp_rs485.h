/**
  ******************************************************************************
  * @file    bsp_rs485.h
  * @brief   RS485 半双工片上 UART/方向控制抽象层。
  ******************************************************************************
  */

#ifndef BSP_RS485_H
#define BSP_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 单条 RS485 总线的软件接收帧缓冲长度。 */
#define BSP_RS485_RX_BUFFER_SIZE 128U

typedef struct
{
  UART_HandleTypeDef *huart;                 /* 绑定的 HAL UART 句柄。 */
  GPIO_TypeDef *de_port;                     /* 可选：手动换向模块 DE/RE 端口；自动换向模块传 NULL。 */
  uint16_t de_pin;                           /* 可选：手动换向模块 DE/RE 引脚；自动换向模块传 0。 */
  uint8_t rx_byte;                           /* HAL 单字节中断接收缓冲。 */
  volatile uint8_t rx_buffer[BSP_RS485_RX_BUFFER_SIZE]; /* 当前正在接收的一帧数据。 */
  volatile uint16_t rx_len;                  /* 当前帧已接收长度。 */
  volatile uint32_t last_rx_tick;            /* 最近一次收到字节的系统 tick。 */
  volatile uint8_t frame_ready;              /* 帧完成标志，由空闲间隔轮询置位。 */
  uint16_t rx_overflow_count;                /* 接收溢出次数，用于调试线束或协议异常。 */
  uint16_t uart_error_count;                 /* UART 硬件错误次数，用于区分总线噪声、帧错误和软件调度问题。 */
} BSP_RS485_Bus_t;

void BSP_RS485_Init(BSP_RS485_Bus_t *bus, UART_HandleTypeDef *huart,
                    GPIO_TypeDef *de_port, uint16_t de_pin);
void BSP_RS485_StartReceive(BSP_RS485_Bus_t *bus);
void BSP_RS485_PollFrameTimeout(BSP_RS485_Bus_t *bus, uint32_t now_ms, uint32_t idle_gap_ms);
uint16_t BSP_RS485_GetFrame(BSP_RS485_Bus_t *bus, uint8_t *out, uint16_t out_size);
HAL_StatusTypeDef BSP_RS485_Send(BSP_RS485_Bus_t *bus, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
void BSP_RS485_UartRxCpltCallback(UART_HandleTypeDef *huart);
void BSP_RS485_UartErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BSP_RS485_H */
