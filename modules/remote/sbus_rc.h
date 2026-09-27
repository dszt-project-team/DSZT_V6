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
/* 连续第 3 个 frame_lost 帧触发停车；范围 1~65535，前 2 帧只保持最近健康通道。 */
#define SBUS_RC_FRAME_LOST_TRIP_FRAMES 3U
/* 丢帧容忍上限，单位 ms，范围 1~TIMEOUT_MS；距健康帧达到此值停车，丢帧不能续期。 */
#define SBUS_RC_FRAME_LOST_HOLD_MS 50U
/* UART 恢复重试间隔，单位 ms，范围 1~TIMEOUT_MS；仅在任务中恢复，不在中断内阻塞。 */
#define SBUS_RC_RX_RETRY_MS       20U
/* 失联原因位图：连续丢帧或丢帧保持时间到限。 */
#define SBUS_RC_REASON_FRAME_LOST 0x01U
/* 失联原因位图：接收机标准或 MC7 failsafe 位，任何一帧置位都立即停车。 */
#define SBUS_RC_REASON_FAILSAFE   0x02U
/* 失联原因位图：超过 300 ms 没有合法 SBUS 帧。 */
#define SBUS_RC_REASON_TIMEOUT    0x04U
/* 失联原因位图：UART/DMA 接收错误。 */
#define SBUS_RC_REASON_UART       0x08U
/* 失联原因位图：DMA 接收启动或恢复失败。 */
#define SBUS_RC_REASON_RX_START   0x10U

/* 参数边界约束：防止把有限位宽计数或恢复周期改成永远无法达到的门槛。 */
#if (SBUS_RC_FRAME_LOST_TRIP_FRAMES < 1U) || (SBUS_RC_FRAME_LOST_TRIP_FRAMES > 65535U)
#error "SBUS frame-lost trip frames must be in 1..65535"
#endif
#if (SBUS_RC_FRAME_LOST_HOLD_MS < 1U) || (SBUS_RC_FRAME_LOST_HOLD_MS > SBUS_RC_TIMEOUT_MS)
#error "SBUS frame-lost hold must be in 1..SBUS_RC_TIMEOUT_MS"
#endif
#if (SBUS_RC_RX_RETRY_MS < 1U) || (SBUS_RC_RX_RETRY_MS > SBUS_RC_TIMEOUT_MS)
#error "SBUS receive retry interval must be in 1..SBUS_RC_TIMEOUT_MS"
#endif

typedef struct
{
  uint16_t raw[SBUS_RC_CHANNEL_COUNT];        /* 16 路 SBUS 原始通道值，MC7 资料给出的常见范围为 200~1800。 */
  uint16_t pulse_us[SBUS_RC_CHANNEL_COUNT];   /* 映射后的 1000~2000 数值，便于后续油门/转向控制使用。 */
  uint8_t frame_lost;                         /* 接收机上报的丢帧标志。 */
  uint8_t failsafe;                           /* 接收机上报的失控保护标志。 */
  uint8_t online;                             /* 可用状态：健康帧或有界丢帧保持为 1，已确认故障为 0。 */
  uint32_t last_update_ms;                    /* 最近一次解析到合法帧的系统 tick。 */
  uint32_t frame_count;                       /* 合法帧计数，用于调试链路是否稳定。 */
  uint32_t error_count;                       /* 帧头/帧尾/串口错误累计次数，用于排查线序和反相电路。 */
  uint8_t raw_flags;                          /* 最新合法帧原始 flags；0x04 丢帧、0x08/0x10 失控。 */
  uint8_t guard_reason;                       /* 当前已确认故障位图；健康帧清零。 */
  uint8_t last_guard_reason;                  /* 最近故障原因位图，恢复后仍保留，便于捕获瞬时故障。 */
  uint16_t lost_streak;                       /* 连续丢帧帧数，饱和至 65535，健康帧清零。 */
  uint32_t good_frame_count;                  /* 健康帧计数；开关确认只以新的健康帧推进。 */
  uint32_t last_good_ms;                      /* 最近健康帧时间，tick 为 0 也有效。 */
  uint32_t lost_count;                        /* frame_lost 标志帧累计次数，独立于 failsafe。 */
  uint32_t failsafe_count;                    /* 任意 failsafe 标志帧累计次数。 */
  uint32_t uart_error_count;                  /* UART/DMA 错误回调累计次数。 */
  uint32_t timeout_count;                     /* 无合法帧超时的故障进入次数，不按任务周期累加。 */
  uint32_t rx_start_error_count;              /* DMA 启动或接收恢复失败累计次数。 */
  uint32_t guard_event_count;                 /* 新故障/新增原因序号，健康帧不清零，防止瞬时故障被覆盖。 */
  uint32_t last_guard_ms;                     /* 最近一次新增故障原因的时间。 */
} SBusRc_Data_t;

typedef struct
{
  UART_HandleTypeDef *huart;                  /* 绑定的 UART，当前 MC7 SBUS 经板载反相电路使用 USART1_RX / PB7。 */
  uint8_t dma_buffer[SBUS_RC_DMA_BUFFER_SIZE];/* DMA 循环接收缓冲区，硬件持续写入，软件按位置差取新数据。 */
  uint16_t dma_last_pos;                      /* 上一次已经处理到的 DMA 写入位置。 */
  uint8_t frame[SBUS_RC_FRAME_SIZE];          /* 当前正在拼接的 25 字节 SBUS 帧。 */
  uint8_t index;                              /* 当前帧写入位置。 */
  uint8_t has_good_frame;                     /* 独立于计数判断健康基线，避免 tick=0 或计数回绕被当作无效。 */
  uint8_t rx_active;                          /* DMA 接收成功启动后置位；恢复期间丢弃旧回调缓冲。 */
  uint8_t rx_restart_pending;                 /* UART 故障/启动失败后等待任务恢复，禁止在中断内阻塞。 */
  uint32_t rx_retry_ms;                       /* 最近一次恢复尝试时间，用于有界重试间隔。 */
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
