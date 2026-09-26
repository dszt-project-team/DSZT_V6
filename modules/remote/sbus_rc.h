/**
  ******************************************************************************
  * @file    sbus_rc.h
  * @brief   MC7/MC8RE 接收机 SBUS 遥控输入模块接口。
  ******************************************************************************
  */

#ifndef SBUS_RC_H
#define SBUS_RC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* SBUS 固定帧长度，包含帧头、16 路通道、标志位和帧尾。 */
#define SBUS_RC_FRAME_SIZE       25U
/* USART1 SBUS DMA 循环接收缓冲长度。 */
#define SBUS_RC_DMA_BUFFER_SIZE  64U
/* 当前解析的 SBUS 标准通道数量。 */
#define SBUS_RC_CHANNEL_COUNT    16U
/* MC7/MC8RE 常见原始通道下限，用于映射到 1000us。 */
#define SBUS_RC_MIN_RAW          200U
/* MC7/MC8RE 常见原始通道中位，用于判断摇杆居中。 */
#define SBUS_RC_MID_RAW          1000U
/* MC7/MC8RE 常见原始通道上限，用于映射到 2000us。 */
#define SBUS_RC_MAX_RAW          1800U
/* 超过该时间没有解析到合法 SBUS 帧，则遥控在线状态清零。 */
#define SBUS_RC_TIMEOUT_MS       300U

typedef struct
{
  uint16_t raw[SBUS_RC_CHANNEL_COUNT];        /* 16 路 SBUS 原始通道值，MC7 资料给出的常见范围为 200~1800。 */
  uint16_t pulse_us[SBUS_RC_CHANNEL_COUNT];   /* 映射后的 1000~2000 数值，便于后续油门/转向控制使用。 */
  uint8_t frame_lost;                         /* 接收机上报的丢帧标志。 */
  uint8_t failsafe;                           /* 接收机上报的失控保护标志。 */
  uint8_t online;                             /* 软件在线状态，超过 SBUS_RC_TIMEOUT_MS 未更新会清零。 */
  uint32_t last_update_ms;                    /* 最近一次解析到合法帧的系统 tick。 */
  uint32_t frame_count;                       /* 合法帧计数，用于调试链路是否稳定。 */
  uint32_t error_count;                       /* 帧头/帧尾/串口错误累计次数，用于排查线序和反相电路。 */
} SBusRc_Data_t;

typedef struct
{
  UART_HandleTypeDef *huart;                  /* 绑定的 UART，当前 MC7 SBUS 经板载反相电路使用 USART1_RX / PB7。 */
  uint8_t dma_buffer[SBUS_RC_DMA_BUFFER_SIZE];/* DMA 循环接收缓冲区，硬件持续写入，软件按位置差取新数据。 */
  uint16_t dma_last_pos;                      /* 上一次已经处理到的 DMA 写入位置。 */
  uint8_t frame[SBUS_RC_FRAME_SIZE];          /* 当前正在拼接的 25 字节 SBUS 帧。 */
  uint8_t index;                              /* 当前帧写入位置。 */
  SBusRc_Data_t data;                         /* 最近一次解析出的通道和状态。 */
} SBusRc_Handle_t;

void SBusRc_Init(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart);
void SBusRc_StartReceive(SBusRc_Handle_t *rc);
void SBusRc_Task(SBusRc_Handle_t *rc, uint32_t now_ms);
void SBusRc_UartIdleCallback(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart, uint32_t now_ms);
void SBusRc_UartRxCpltCallback(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart, uint32_t now_ms);
void SBusRc_UartErrorCallback(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart);
const SBusRc_Data_t *SBusRc_GetData(const SBusRc_Handle_t *rc);
uint8_t SBusRc_GetSnapshot(const SBusRc_Handle_t *rc, SBusRc_Data_t *snapshot);
uint16_t SBusRc_GetChannelPulseUs(const SBusRc_Handle_t *rc, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_RC_H */
