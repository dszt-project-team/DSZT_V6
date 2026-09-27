/**
  ******************************************************************************
  * @file    sbus_rc.c
  * @brief   MC7/MC8RE 接收机 SBUS 遥控输入模块实现。
  ******************************************************************************
  */

#include "sbus_rc.h"
#include "bsp_callback.h"
#include <string.h>

/* SBUS 固定帧头，MC7/MC8RE-V2 每帧第 0 字节应为 0x0F。 */
#define SBUS_RC_HEADER_BYTE       0x0FU
/* SBUS 固定帧尾，当前接收机正常帧第 24 字节为 0x00。 */
#define SBUS_RC_FOOTER_BYTE       0x00U
/* SBUS flags.bit2：接收机声明上一帧丢失。 */
#define SBUS_RC_FLAG_FRAME_LOST   0x04U
/* SBUS flags.bit3：标准 failsafe 标志，置位时禁止执行器输出。 */
#define SBUS_RC_FLAG_FAILSAFE     0x08U
/* MC7/MC8RE-V2 兼容 failsafe 标志位，与标准 failsafe 合并判断。 */
#define SBUS_RC_FLAG_FAILSAFE_MC7 0x10U

static void SBusRc_DispatchRxComplete(void *parent, UART_HandleTypeDef *huart)
{
  SBusRc_UartRxCpltCallback((SBusRc_Handle_t *)parent, huart, HAL_GetTick());
}

static void SBusRc_DispatchIdle(void *parent, UART_HandleTypeDef *huart)
{
  SBusRc_UartIdleCallback((SBusRc_Handle_t *)parent, huart, HAL_GetTick());
}

static void SBusRc_DispatchError(void *parent, UART_HandleTypeDef *huart)
{
  SBusRc_UartErrorCallback((SBusRc_Handle_t *)parent, huart);
}

/**
  * @brief 把 SBUS 原始通道值限制并映射到 1000~2000。
  * @param raw SBUS 11 位原始通道值。
  * @return 约等于航模 PWM 语义的 1000~2000 数值。
  * @note 当前按 MC7/MC8RE 资料里的 200~1800 标定；接收机端点配置变化时，
  *       只需调整头文件里的 MIN/MID/MAX 宏。
  */
static uint16_t SBusRc_NormalizePulseUs(uint16_t raw)
{
  uint32_t scaled;

  if (raw <= SBUS_RC_MIN_RAW)
  {
    return 1000U;
  }

  if (raw >= SBUS_RC_MAX_RAW)
  {
    return 2000U;
  }

  scaled = ((uint32_t)(raw - SBUS_RC_MIN_RAW) * 1000U) /
           (uint32_t)(SBUS_RC_MAX_RAW - SBUS_RC_MIN_RAW);
  return (uint16_t)(1000U + scaled);
}

/* 任务调用者须持有临界区；ISR 与 USART/DMA 回调同优先级，不会互相重入。
 * 只记录新进入/新增原因，避免连续坏帧或周期维护刷掉现场事件时间。 */
static void SBusRc_EnterGuard(SBusRc_Handle_t *rc, uint8_t reason, uint32_t now_ms)
{
  if ((reason & (uint8_t)~rc->data.guard_reason) != 0U)
  {
    if (((reason & SBUS_RC_REASON_TIMEOUT) != 0U) &&
        ((rc->data.guard_reason & SBUS_RC_REASON_TIMEOUT) == 0U))
    {
      rc->data.timeout_count++;
    }
    rc->data.guard_reason |= reason;
    rc->data.last_guard_reason = rc->data.guard_reason;
    rc->data.last_guard_ms = now_ms;
    rc->data.guard_event_count++;
  }
  rc->data.online = 0U;
}

static void SBusRc_CheckAge(SBusRc_Handle_t *rc, uint32_t now_ms)
{
  if ((uint32_t)(now_ms - rc->data.last_update_ms) > SBUS_RC_TIMEOUT_MS)
  {
    SBusRc_EnterGuard(rc, SBUS_RC_REASON_TIMEOUT, now_ms);
  }
  if ((rc->data.frame_lost != 0U) &&
      ((rc->has_good_frame == 0U) ||
       ((uint32_t)(now_ms - rc->data.last_good_ms) >= SBUS_RC_FRAME_LOST_HOLD_MS)))
  {
    SBusRc_EnterGuard(rc, SBUS_RC_REASON_FRAME_LOST, now_ms);
  }
}

/**
  * @brief 从 25 字节 SBUS 帧中解包 16 路 11 位通道值。
  * @param rc     SBUS 驱动实例。
  * @param now_ms 当前 HAL tick，用于记录数据新鲜度。
  */
static void SBusRc_ParseFrame(SBusRc_Handle_t *rc, uint32_t now_ms)
{
  const uint8_t *b = rc->frame;
  uint16_t *ch = rc->data.raw;
  uint8_t i;

  if ((b[0] != SBUS_RC_HEADER_BYTE) || (b[24] != SBUS_RC_FOOTER_BYTE))
  {
    rc->data.error_count++;
    return;
  }

  /* 先结算旧帧的时限，再接纳恢复帧；即使任务尚未轮询，故障事件也不会消失。 */
  SBusRc_CheckAge(rc, now_ms);
  rc->data.raw_flags = b[23];
  rc->data.frame_lost = ((b[23] & SBUS_RC_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
  rc->data.failsafe =
      ((b[23] & (SBUS_RC_FLAG_FAILSAFE | SBUS_RC_FLAG_FAILSAFE_MC7)) != 0U) ? 1U : 0U;
  rc->data.last_update_ms = now_ms;
  rc->data.frame_count++;
  if (rc->data.frame_lost != 0U)
  {
    rc->data.lost_count++;
    if (rc->data.lost_streak < UINT16_MAX) rc->data.lost_streak++;
  }
  else
  {
    rc->data.lost_streak = 0U;
  }
  if (rc->data.failsafe != 0U)
  {
    rc->data.failsafe_count++;
    SBusRc_EnterGuard(rc, SBUS_RC_REASON_FAILSAFE, now_ms);
    return;
  }
  if (rc->data.frame_lost != 0U)
  {
    if ((rc->has_good_frame == 0U) ||
        (rc->data.lost_streak >= SBUS_RC_FRAME_LOST_TRIP_FRAMES) ||
        ((uint32_t)(now_ms - rc->data.last_good_ms) >= SBUS_RC_FRAME_LOST_HOLD_MS))
    {
      SBusRc_EnterGuard(rc, SBUS_RC_REASON_FRAME_LOST, now_ms);
    }
    /* 丢帧的通道内容不可用于新运动目标或 CH5 确认；只有近期健康基线才允许短暂保持。 */
    return;
  }

  ch[0]  = (uint16_t)(((uint16_t)b[1]        | ((uint16_t)b[2] << 8)) & 0x07FFU);
  ch[1]  = (uint16_t)((((uint16_t)b[2] >> 3) | ((uint16_t)b[3] << 5)) & 0x07FFU);
  ch[2]  = (uint16_t)((((uint16_t)b[3] >> 6) | ((uint16_t)b[4] << 2) | ((uint16_t)b[5] << 10)) & 0x07FFU);
  ch[3]  = (uint16_t)((((uint16_t)b[5] >> 1) | ((uint16_t)b[6] << 7)) & 0x07FFU);
  ch[4]  = (uint16_t)((((uint16_t)b[6] >> 4) | ((uint16_t)b[7] << 4)) & 0x07FFU);
  ch[5]  = (uint16_t)((((uint16_t)b[7] >> 7) | ((uint16_t)b[8] << 1) | ((uint16_t)b[9] << 9)) & 0x07FFU);
  ch[6]  = (uint16_t)((((uint16_t)b[9] >> 2) | ((uint16_t)b[10] << 6)) & 0x07FFU);
  ch[7]  = (uint16_t)((((uint16_t)b[10] >> 5) | ((uint16_t)b[11] << 3)) & 0x07FFU);
  ch[8]  = (uint16_t)(((uint16_t)b[12]        | ((uint16_t)b[13] << 8)) & 0x07FFU);
  ch[9]  = (uint16_t)((((uint16_t)b[13] >> 3) | ((uint16_t)b[14] << 5)) & 0x07FFU);
  ch[10] = (uint16_t)((((uint16_t)b[14] >> 6) | ((uint16_t)b[15] << 2) | ((uint16_t)b[16] << 10)) & 0x07FFU);
  ch[11] = (uint16_t)((((uint16_t)b[16] >> 1) | ((uint16_t)b[17] << 7)) & 0x07FFU);
  ch[12] = (uint16_t)((((uint16_t)b[17] >> 4) | ((uint16_t)b[18] << 4)) & 0x07FFU);
  ch[13] = (uint16_t)((((uint16_t)b[18] >> 7) | ((uint16_t)b[19] << 1) | ((uint16_t)b[20] << 9)) & 0x07FFU);
  ch[14] = (uint16_t)((((uint16_t)b[20] >> 2) | ((uint16_t)b[21] << 6)) & 0x07FFU);
  ch[15] = (uint16_t)((((uint16_t)b[21] >> 5) | ((uint16_t)b[22] << 3)) & 0x07FFU);

  for (i = 0U; i < SBUS_RC_CHANNEL_COUNT; i++)
  {
    rc->data.pulse_us[i] = SBusRc_NormalizePulseUs(ch[i]);
  }

  rc->has_good_frame = 1U;
  rc->data.good_frame_count++;
  rc->data.last_good_ms = now_ms;
  rc->data.guard_reason = 0U;
  rc->data.online = 1U;
}

/**
  * @brief 输入 1 个 SBUS 字节并维护帧同步。
  * @param rc     SBUS 驱动实例。
  * @param byte   新收到的字节。
  * @param now_ms 当前 HAL tick。
  * @note DMA 只负责搬运字节，真正的帧同步仍在这里完成；遇到 0x0F 才开始收帧。
  */
static void SBusRc_InputByte(SBusRc_Handle_t *rc, uint8_t byte, uint32_t now_ms)
{
  if (rc->index == 0U)
  {
    if (byte == SBUS_RC_HEADER_BYTE)
    {
      rc->frame[rc->index] = byte;
      rc->index++;
    }
    return;
  }

  rc->frame[rc->index] = byte;
  rc->index++;

  if (rc->index >= SBUS_RC_FRAME_SIZE)
  {
    SBusRc_ParseFrame(rc, now_ms);
    rc->index = 0U;
  }
}

/**
  * @brief 根据 DMA 当前写指针，把环形缓冲区里的新字节喂给 SBUS 解析器。
  * @param rc     SBUS 驱动实例。
  * @param now_ms 当前 HAL tick。
  * @note 这个函数通常在 UART IDLE 中断里调用，也可在 DMA 满缓冲回调里兜底调用。
  */
static void SBusRc_ProcessDmaNewBytes(SBusRc_Handle_t *rc, uint32_t now_ms)
{
  uint16_t pos;
  uint16_t i;

  if ((rc == 0) || (rc->huart == 0) || (rc->huart->hdmarx == 0) ||
      (rc->rx_active == 0U) || (rc->rx_restart_pending != 0U) ||
      (rc->huart->ErrorCode != HAL_UART_ERROR_NONE))
  {
    return;
  }

  pos = (uint16_t)(SBUS_RC_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(rc->huart->hdmarx));
  if (pos >= SBUS_RC_DMA_BUFFER_SIZE)
  {
    pos = 0U;
  }

  if (pos == rc->dma_last_pos)
  {
    return;
  }

  if (pos > rc->dma_last_pos)
  {
    for (i = rc->dma_last_pos; i < pos; i++)
    {
      SBusRc_InputByte(rc, rc->dma_buffer[i], now_ms);
    }
  }
  else
  {
    for (i = rc->dma_last_pos; i < SBUS_RC_DMA_BUFFER_SIZE; i++)
    {
      SBusRc_InputByte(rc, rc->dma_buffer[i], now_ms);
    }
    for (i = 0U; i < pos; i++)
    {
      SBusRc_InputByte(rc, rc->dma_buffer[i], now_ms);
    }
  }

  rc->dma_last_pos = pos;
}

/**
  * @brief 初始化 SBUS 驱动实例并启动 UART DMA 循环接收。
  * @param rc    SBUS 驱动实例内存，由上层静态分配。
  * @param huart 绑定的 UART 句柄，MC7 当前为 huart1。
  */
void SBusRc_Init(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart)
{
  BspUartCallbackConfig callback_config;
  uint8_t i;

  if ((rc == 0) || (huart == 0))
  {
    return;
  }

  memset(rc, 0, sizeof(*rc));
  rc->huart = huart;
  rc->data.last_update_ms = HAL_GetTick();
  memset(&callback_config, 0, sizeof(callback_config));
  callback_config.handle = huart;
  callback_config.parent = rc;
  callback_config.rx_complete = SBusRc_DispatchRxComplete;
  callback_config.idle = SBusRc_DispatchIdle;
  callback_config.error = SBusRc_DispatchError;
  if (BspCallback_RegisterUart(&callback_config) == 0U)
  {
    rc->data.error_count++;
    return;
  }
  for (i = 0U; i < SBUS_RC_CHANNEL_COUNT; i++)
  {
    rc->data.raw[i] = SBUS_RC_MID_RAW;
    rc->data.pulse_us[i] = 1500U;
  }
  SBusRc_StartReceive(rc);
}

/**
  * @brief 启动或重新启动 SBUS 的 DMA 循环接收和 IDLE 中断。
  * @param rc SBUS 驱动实例。
  */
void SBusRc_StartReceive(SBusRc_Handle_t *rc)
{
  uint32_t primask;
  uint32_t now_ms;
  HAL_StatusTypeDef status;

  if ((rc == 0) || (rc->huart == 0))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  now_ms = HAL_GetTick();
  if (rc->rx_active == 0U)
  {
    rc->dma_last_pos = 0U;
    rc->index = 0U;
    rc->rx_retry_ms = now_ms;
    status = (rc->huart->hdmarx == 0) ? HAL_ERROR :
             HAL_UART_Receive_DMA(rc->huart, rc->dma_buffer, SBUS_RC_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
      rc->rx_active = 1U;
      rc->rx_restart_pending = 0U;
      __HAL_DMA_DISABLE_IT(rc->huart->hdmarx, DMA_IT_HT);
      __HAL_UART_ENABLE_IT(rc->huart, UART_IT_IDLE);
    }
    else
    {
      rc->rx_restart_pending = 1U;
      rc->data.error_count++;
      rc->data.rx_start_error_count++;
      SBusRc_EnterGuard(rc, SBUS_RC_REASON_RX_START, now_ms);
    }
  }
  if (primask == 0U) __enable_irq();
}

/**
  * @brief 周期性维护 SBUS 在线状态。
  * @param rc     SBUS 驱动实例。
  * @param now_ms 当前 HAL tick。
  */
void SBusRc_Task(SBusRc_Handle_t *rc, uint32_t now_ms)
{
  uint32_t primask;
  uint8_t retry_rx = 0U;
  HAL_StatusTypeDef status;

  if (rc == 0)
  {
    return;
  }

  /* 同一临界区采样当前 tick、检查最后一帧并更新 online，防止新 ISR 帧
   * 比任务传入的 now_ms 更晚而造成无符号下溢，也防止超时判断覆盖新帧。
   * 初始 online 已为 0；合法帧可以恰好在 tick 回绕为 0 时到达。 */
  primask = __get_PRIMASK();
  __disable_irq();
  now_ms = HAL_GetTick();
  SBusRc_CheckAge(rc, now_ms);
  if ((primask == 0U) && (rc->rx_restart_pending != 0U) &&
      ((uint32_t)(now_ms - rc->rx_retry_ms) >= SBUS_RC_RX_RETRY_MS))
  {
    rc->rx_retry_ms = now_ms;
    retry_rx = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  if ((retry_rx != 0U) && (rc->huart != 0))
  {
    /* HAL 的 DMA Abort 可能轮询等待硬件，因此仅在任务、开中断状态下执行。 */
    status = HAL_UART_AbortReceive(rc->huart);
    if (status == HAL_OK)
    {
      SBusRc_StartReceive(rc);
    }
    else
    {
      primask = __get_PRIMASK();
      __disable_irq();
      rc->data.error_count++;
      rc->data.rx_start_error_count++;
      SBusRc_EnterGuard(rc, SBUS_RC_REASON_RX_START, HAL_GetTick());
      if (primask == 0U) __enable_irq();
    }
  }
}

/**
  * @brief UART IDLE 空闲中断回调入口。
  * @param rc     SBUS 驱动实例。
  * @param huart  触发 IDLE 的 UART。
  * @param now_ms 当前 HAL tick。
  */
void SBusRc_UartIdleCallback(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart, uint32_t now_ms)
{
  if ((rc == 0) || (huart != rc->huart))
  {
    return;
  }

  SBusRc_ProcessDmaNewBytes(rc, now_ms);
}

/**
  * @brief HAL UART 接收完成回调入口，主要用于 DMA 环形缓冲区绕回时兜底处理。
  * @param rc     SBUS 驱动实例。
  * @param huart  触发回调的 UART。
  * @param now_ms 当前 HAL tick。
  */
void SBusRc_UartRxCpltCallback(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart, uint32_t now_ms)
{
  if ((rc == 0) || (huart != rc->huart))
  {
    return;
  }

  SBusRc_ProcessDmaNewBytes(rc, now_ms);
}

/**
  * @brief HAL UART 错误回调中的 SBUS 恢复入口。
  * @param rc    SBUS 驱动实例。
  * @param huart 触发错误的 UART。
  */
void SBusRc_UartErrorCallback(SBusRc_Handle_t *rc, UART_HandleTypeDef *huart)
{
  if ((rc == 0) || (huart != rc->huart))
  {
    return;
  }

  rc->index = 0U;
  rc->dma_last_pos = 0U;
  rc->rx_active = 0U;
  rc->rx_restart_pending = 1U;
  rc->has_good_frame = 0U;
  rc->rx_retry_ms = HAL_GetTick();
  rc->data.error_count++;
  rc->data.uart_error_count++;
  SBusRc_EnterGuard(rc, SBUS_RC_REASON_UART, rc->rx_retry_ms);
}

/**
  * @brief 获取最近一次解析出的 SBUS 状态。
  * @param rc SBUS 驱动实例。
  * @return 数据指针；参数非法时返回空指针。
  */
const SBusRc_Data_t *SBusRc_GetData(const SBusRc_Handle_t *rc)
{
  if (rc == 0)
  {
    return 0;
  }

  return &rc->data;
}

/**
  * @brief 原子复制最近一帧 SBUS 数据，避免任务与 UART/DMA 回调跨帧混读。
  */
uint8_t SBusRc_GetSnapshot(const SBusRc_Handle_t *rc, SBusRc_Data_t *snapshot)
{
  uint32_t primask;

  if ((rc == 0) || (snapshot == 0))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(snapshot, &rc->data, sizeof(*snapshot));
  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

/**
  * @brief 按通道号读取 1000~2000 映射值。
  * @param rc      SBUS 驱动实例。
  * @param channel 通道序号，0 对应 CH1，15 对应 CH16。
  * @return 对应通道映射值；参数非法时返回中位 1500。
  */
uint16_t SBusRc_GetChannelPulseUs(const SBusRc_Handle_t *rc, uint8_t channel)
{
  if ((rc == 0) || (channel >= SBUS_RC_CHANNEL_COUNT))
  {
    return 1500U;
  }

  return rc->data.pulse_us[channel];
}
