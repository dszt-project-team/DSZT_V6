/**
  ******************************************************************************
  * @file    bsp_rs485.c
  * @brief   RS485 半双工片上 UART/方向控制抽象层实现。
  ******************************************************************************
  */

#include "bsp_rs485.h"
#include "bsp_callback.h"
#include <string.h>

/* 最多登记的 RS485 总线实例数量，用于 UART 回调分发。 */
#define BSP_RS485_MAX_BUS_NUM 4U

static BSP_RS485_Bus_t *s_bus_list[BSP_RS485_MAX_BUS_NUM];

static void BSP_RS485_DispatchRxComplete(void *parent, UART_HandleTypeDef *huart)
{
  (void)parent;
  BSP_RS485_UartRxCpltCallback(huart);
}

static void BSP_RS485_DispatchError(void *parent, UART_HandleTypeDef *huart)
{
  (void)parent;
  BSP_RS485_UartErrorCallback(huart);
}

/**
  * @brief 按需切换 RS485 收发方向。
  * @param bus   RS485 总线实例。
  * @param state 手动换向模块的目标电平。
  * @note  当前 OID 总线使用自动双向 TTL-RS485 模块，不需要 MCU 控制 DE/RE。
  *        当端口为空时直接返回；保留此兼容层便于以后接入手动换向模块。
  */
static void BSP_RS485_SetDirection(BSP_RS485_Bus_t *bus, GPIO_PinState state)
{
  if ((bus != 0) && (bus->de_port != 0))
  {
    HAL_GPIO_WritePin(bus->de_port, bus->de_pin, state);
  }
}

/**
  * @brief 把 RS485 总线实例登记到 UART 回调分发表。
  * @param bus 需要登记的总线实例。
  */
static void BSP_RS485_RegisterBus(BSP_RS485_Bus_t *bus)
{
  uint8_t i;

  for (i = 0; i < BSP_RS485_MAX_BUS_NUM; i++)
  {
    if ((s_bus_list[i] == bus) || (s_bus_list[i] == 0))
    {
      s_bus_list[i] = bus;
      return;
    }
  }
}

/**
  * @brief 按 UART 句柄查找 RS485 总线实例。
  * @param huart HAL UART 句柄。
  * @return 找到的总线实例；未找到返回空指针。
  */
static BSP_RS485_Bus_t *BSP_RS485_FindBus(UART_HandleTypeDef *huart)
{
  uint8_t i;

  for (i = 0; i < BSP_RS485_MAX_BUS_NUM; i++)
  {
    if ((s_bus_list[i] != 0) && (s_bus_list[i]->huart == huart))
    {
      return s_bus_list[i];
    }
  }

  return 0;
}

/**
  * @brief 初始化一条 RS485 半双工总线。
  * @param bus     总线实例内存，由上层静态分配。
  * @param huart   绑定的 UART。
  * @param de_port 可选的 DE/RE 方向脚端口；自动换向模块传 NULL。
  * @param de_pin  可选的 DE/RE 方向脚引脚；自动换向模块传 0。
  * @note  自动换向模块只启动 1 字节中断接收；手动换向模块还会先切回接收状态。
  */
void BSP_RS485_Init(BSP_RS485_Bus_t *bus, UART_HandleTypeDef *huart,
                    GPIO_TypeDef *de_port, uint16_t de_pin)
{
  BspUartCallbackConfig callback_config;

  if ((bus == 0) || (huart == 0))
  {
    return;
  }

  memset(bus, 0, sizeof(*bus));
  bus->huart = huart;
  bus->de_port = de_port;
  bus->de_pin = de_pin;

  BSP_RS485_SetDirection(bus, GPIO_PIN_RESET);
  BSP_RS485_RegisterBus(bus);
  memset(&callback_config, 0, sizeof(callback_config));
  callback_config.handle = huart;
  callback_config.parent = bus;
  callback_config.rx_complete = BSP_RS485_DispatchRxComplete;
  callback_config.error = BSP_RS485_DispatchError;
  if (BspCallback_RegisterUart(&callback_config) == 0U)
  {
    bus->uart_error_count++;
    return;
  }
  BSP_RS485_StartReceive(bus);
}

/**
  * @brief 启动单字节 UART 中断接收。
  * @param bus RS485 总线实例。
  * @note  接收完成后会在 HAL_UART_RxCpltCallback 中自动再次启动，形成持续接收。
  */
void BSP_RS485_StartReceive(BSP_RS485_Bus_t *bus)
{
  if ((bus == 0) || (bus->huart == 0))
  {
    return;
  }

  (void)HAL_UART_Receive_IT(bus->huart, &bus->rx_byte, 1U);
}

/**
  * @brief 轮询接收帧空闲超时。
  * @param bus         RS485 总线实例。
  * @param now_ms      当前 HAL tick。
  * @param idle_gap_ms 多久未收到新字节认为一帧完成。
  * @note  Modbus RTU 标准用 3.5 字符时间作为帧间隔；这里用毫秒级空闲检测，便于裸机主循环调度。
  */
void BSP_RS485_PollFrameTimeout(BSP_RS485_Bus_t *bus, uint32_t now_ms, uint32_t idle_gap_ms)
{
  uint16_t len_snapshot;
  uint32_t last_tick_snapshot;
  uint32_t primask;

  if ((bus == 0) || (bus->frame_ready != 0U))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  len_snapshot = bus->rx_len;
  last_tick_snapshot = bus->last_rx_tick;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((len_snapshot > 0U) && ((now_ms - last_tick_snapshot) >= idle_gap_ms))
  {
    /* 重新关中断核对快照，避免刚到的新字节与旧帧边界竞争。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((bus->rx_len == len_snapshot) &&
        (bus->last_rx_tick == last_tick_snapshot) &&
        (bus->frame_ready == 0U))
    {
      bus->frame_ready = 1U;
    }
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
}

/**
  * @brief 取出一帧已完成的 RS485 数据。
  * @param bus      RS485 总线实例。
  * @param out      输出缓存。
  * @param out_size 输出缓存大小。
  * @return 复制出的帧长度；没有完整帧时返回 0。
  */
uint16_t BSP_RS485_GetFrame(BSP_RS485_Bus_t *bus, uint8_t *out, uint16_t out_size)
{
  uint16_t len;
  uint32_t primask;

  if ((bus == 0) || (out == 0) || (out_size == 0U) || (bus->frame_ready == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  len = bus->rx_len;
  if (len > out_size)
  {
    len = out_size;
  }
  memcpy(out, (const void *)bus->rx_buffer, len);
  bus->rx_len = 0U;
  bus->frame_ready = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return len;
}

/**
  * @brief 通过 RS485 半双工总线发送数据。
  * @param bus        RS485 总线实例。
  * @param data       待发送数据。
  * @param len        待发送长度。
  * @param timeout_ms HAL 发送超时。
  * @return HAL 状态。
  * @note  自动换向模块无需 MCU 控制方向；若传入 DE/RE 引脚，则发送前拉高、等待 TC 后拉低。
  */
HAL_StatusTypeDef BSP_RS485_Send(BSP_RS485_Bus_t *bus, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t start_tick;
  uint32_t primask;

  if ((bus == 0) || (bus->huart == 0) || (data == 0) || (len == 0U))
  {
    return HAL_ERROR;
  }

  BSP_RS485_SetDirection(bus, GPIO_PIN_SET);
  status = HAL_UART_Transmit(bus->huart, (uint8_t *)data, len, timeout_ms);

  start_tick = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - start_tick) > timeout_ms)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }

  /*
   * 自动换向 TTL-RS485 模块常会让 MCU 收到自己刚发出的 Modbus 请求。
   * 发送结束后清掉这段本机回显，避免请求帧和从站响应粘在一起导致 CRC 错误。
   * 手动 DE/RE 模块一般不会回显，但清空当前半帧也不会影响后续响应接收。
   */
  primask = __get_PRIMASK();
  __disable_irq();
  bus->rx_len = 0U;
  bus->frame_ready = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  BSP_RS485_SetDirection(bus, GPIO_PIN_RESET);
  return status;
}

/**
  * @brief HAL UART 接收完成回调分发入口。
  * @param huart 产生回调的 UART 句柄。
  * @note  ISR 内只做入缓冲和重新挂接收，不做协议解析，保证中断足够短。
  */
void BSP_RS485_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  BSP_RS485_Bus_t *bus = BSP_RS485_FindBus(huart);

  if (bus == 0)
  {
    return;
  }

  if (bus->frame_ready != 0U)
  {
    bus->rx_overflow_count++;
    bus->rx_len = 0U;
    bus->frame_ready = 0U;
  }

  if (bus->rx_len < BSP_RS485_RX_BUFFER_SIZE)
  {
    bus->rx_buffer[bus->rx_len] = bus->rx_byte;
    bus->rx_len++;
    bus->last_rx_tick = HAL_GetTick();
  }
  else
  {
    bus->rx_overflow_count++;
    bus->rx_len = 0U;
  }

  BSP_RS485_StartReceive(bus);
}

/**
  * @brief HAL UART 错误回调分发入口。
  * @param huart 产生错误的 UART 句柄。
  * @note  调试阶段常见错误来自线束噪声或波特率不一致，清空当前半帧后重新挂接收。
  */
void BSP_RS485_UartErrorCallback(UART_HandleTypeDef *huart)
{
  BSP_RS485_Bus_t *bus = BSP_RS485_FindBus(huart);

  if (bus == 0)
  {
    return;
  }

  bus->uart_error_count++;
  bus->rx_len = 0U;
  bus->frame_ready = 0U;
  BSP_RS485_StartReceive(bus);
}
