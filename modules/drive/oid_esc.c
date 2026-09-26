/**
  ******************************************************************************
  * @file    oid_esc.c
  * @brief   OID-RS485-500W 单驱电调设备模块实现。
  ******************************************************************************
  */

#include "oid_esc.h"
#include "modbus_rtu.h"
#include <string.h>

/* 单帧 OID Modbus 请求的 UART 发送超时，单位 ms。 */
#define OID_ESC_TX_TIMEOUT_MS 10U

/**
  * @brief 初始化 OID 电调对象。
  * @param esc        电调对象。
  * @param id         Modbus 从站 ID。
  * @param pole_pairs 电机极对数；V6 当前沿用 14 对极占位值，实车适配前需确认。
  */
void OID_ESC_Init(OID_ESC_Handle_t *esc, uint8_t id, uint8_t pole_pairs)
{
  if (esc == 0)
  {
    return;
  }

  memset(esc, 0, sizeof(*esc));
  esc->id = id;
  esc->pole_pairs = pole_pairs;
  esc->heartbeat_value = 1U;
}

/**
  * @brief 发送一条 Modbus 帧。
  * @param bus RS485 总线。
  * @param frame 完整 Modbus 帧。
  * @param len 帧长度。
  * @return HAL 发送状态。
  */
static HAL_StatusTypeDef OID_ESC_SendFrame(BSP_RS485_Bus_t *bus, const uint8_t *frame, uint16_t len)
{
  return BSP_RS485_Send(bus, frame, len, OID_ESC_TX_TIMEOUT_MS);
}

/**
  * @brief 更新电调心跳寄存器。
  * @param bus OID RS485 总线。
  * @param esc 电调对象。
  * @return HAL 发送状态。
  * @note  OID 手册要求周期性改变 0x1770，否则心跳超时后电机会被放空。
  */
HAL_StatusTypeDef OID_ESC_SendHeartbeat(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc)
{
  uint8_t frame[8];
  uint16_t len;
  HAL_StatusTypeDef status;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  len = Modbus_RtuBuildWriteSingle(esc->id, OID_ESC_REG_HEARTBEAT, esc->heartbeat_value, frame, sizeof(frame));
  status = OID_ESC_SendFrame(bus, frame, len);
  if (status == HAL_OK)
  {
    /* 只有真正交给 UART 后才提交下一值，失败重试时仍发送尚未成功的心跳值。 */
    esc->heartbeat_value = (esc->heartbeat_value == 1U) ? 2U : 1U;
  }

  return status;
}

/**
  * @brief 设置 OID 控制模式。
  * @param bus  OID RS485 总线。
  * @param esc  电调对象。
  * @param mode 控制模式，见 OID_ESC_Mode_t。
  * @return HAL 发送状态。
  */
HAL_StatusTypeDef OID_ESC_SetControlMode(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, OID_ESC_Mode_t mode)
{
  uint8_t frame[8];
  uint16_t len;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  len = Modbus_RtuBuildWriteSingle(esc->id, OID_ESC_REG_CONTROL_MODE, (uint16_t)mode, frame, sizeof(frame));
  return OID_ESC_SendFrame(bus, frame, len);
}

/**
  * @brief 设置目标电角速度。
  * @param bus        OID RS485 总线。
  * @param esc        电调对象。
  * @param speed_erpm 目标电角速度，单位 erpm，正负号决定方向。
  * @return HAL 发送状态。
  * @note  目标速度是 32 位寄存器，按厂家手册的大端字节顺序写入 0x1773/0x1774。
  */
HAL_StatusTypeDef OID_ESC_SetSpeedErpm(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, int32_t speed_erpm)
{
  HAL_StatusTypeDef status;
  uint8_t frame[16];
  uint16_t regs[2];
  uint16_t len;
  uint32_t raw = (uint32_t)speed_erpm;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  regs[0] = (uint16_t)(raw >> 16);
  regs[1] = (uint16_t)(raw & 0xFFFFU);
  len = Modbus_RtuBuildWriteMultipleU16(esc->id, OID_ESC_REG_TARGET_SPEED, regs, 2U, frame, sizeof(frame));

  status = OID_ESC_SendFrame(bus, frame, len);
  if (status == HAL_OK)
  {
    /* 0x10 replies echo address/count, NOT the written value. Diagnostic only. */
    if (esc->diagnostic.speed_write_pending != 0U)
      esc->diagnostic.speed_unconfirmed++;
    esc->diagnostic.speed_write_pending = 1U;
    esc->diagnostic.speed_write_ms = HAL_GetTick();
    if (speed_erpm == 0) esc->diagnostic.zero_writes++;
  }
  return status;
}

void OID_ESC_PollDiagnostic(OID_ESC_Handle_t *esc, uint32_t now_ms)
{
  if (esc == 0) return;
  if (esc->diagnostic.control_pending != 0U &&
      (now_ms - esc->diagnostic.control_request_ms) >= 50U)
  {
    esc->diagnostic.control_pending = 0U;
    esc->diagnostic.control_timeouts++;
  }
  if (esc->diagnostic.speed_write_pending != 0U &&
      (now_ms - esc->diagnostic.speed_write_ms) >= 50U)
  {
    esc->diagnostic.speed_write_pending = 0U;
    esc->diagnostic.speed_unconfirmed++;
  }
}

/* Only this 0x03 range is requested, so an 8-byte data reply is unambiguous. */
HAL_StatusTypeDef OID_ESC_RequestControl(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, uint32_t now_ms)
{
  uint8_t frame[8];
  uint16_t len;
  HAL_StatusTypeDef status;
  if (bus == 0 || esc == 0) return HAL_ERROR;
  if (esc->diagnostic.control_pending != 0U) return HAL_BUSY;
  len = Modbus_RtuBuildRead(esc->id, MODBUS_RTU_FUNC_READ_HOLDING,
                           OID_ESC_REG_CONTROL_MODE, 4U, frame, sizeof(frame));
  status = OID_ESC_SendFrame(bus, frame, len);
  if (status == HAL_OK)
  {
    esc->diagnostic.control_pending = 1U;
    esc->diagnostic.control_request_ms = now_ms;
  }
  return status;
}

/**
  * @brief 按机械转速 rpm 设置目标速度。
  * @param bus       OID RS485 总线。
  * @param esc       电调对象。
  * @param speed_rpm 目标输出机械转速，单位 rpm。
  * @return HAL 发送状态。
  * @note  OID 速度寄存器使用 erpm，因此这里用电机极对数换算；本接口参数明确是行走电机本体机械 rpm，
  *        若上层持有的是减速箱输出轴或车轮 rpm，必须先按减速比换算，不能与转向 MT6826S 反馈混用。
  */
HAL_StatusTypeDef OID_ESC_SetSpeedRpm(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, int32_t speed_rpm)
{
  if (esc == 0)
  {
    return HAL_ERROR;
  }

  return OID_ESC_SetSpeedErpm(bus, esc, speed_rpm * (int32_t)esc->pole_pairs);
}

/**
  * @brief 设置目标占空比。
  * @param bus           OID RS485 总线。
  * @param esc           电调对象。
  * @param duty_permille 目标占空比，单位千分比，50 表示 5%，-50 表示 -5%。
  * @return HAL 发送状态。
  * @note  找 ABZ 编码器 Z 信号时用小占空比让电机短暂转动，找到后必须及时写 0。
  */
HAL_StatusTypeDef OID_ESC_SetDutyPermille(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, int16_t duty_permille)
{
  uint8_t frame[8];
  uint16_t len;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  len = Modbus_RtuBuildWriteSingle(esc->id,
                                   OID_ESC_REG_TARGET_DUTY,
                                   (uint16_t)duty_permille,
                                   frame,
                                   sizeof(frame));
  return OID_ESC_SendFrame(bus, frame, len);
}

/**
  * @brief 设置电子刹车电流。
  * @param bus          OID RS485 总线。
  * @param esc          电调对象。
  * @param current_10ma 刹车电流，单位 10mA。
  * @return HAL 发送状态。
  */
HAL_StatusTypeDef OID_ESC_SetBrakeCurrent(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, uint16_t current_10ma)
{
  uint8_t frame[8];
  uint16_t len;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  len = Modbus_RtuBuildWriteSingle(esc->id, OID_ESC_REG_BRAKE_CURRENT, current_10ma, frame, sizeof(frame));
  return OID_ESC_SendFrame(bus, frame, len);
}

/**
  * @brief 请求读取 OID 基础状态。
  * @param bus OID RS485 总线。
  * @param esc 电调对象。
  * @return HAL 发送状态。
  * @note  一次读取 5000~5008：故障、实时速度、占空比、功率、电压、电流、温度。
  */
HAL_StatusTypeDef OID_ESC_RequestStatus(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc)
{
  uint8_t frame[8];
  uint16_t len;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  len = Modbus_RtuBuildRead(esc->id, MODBUS_RTU_FUNC_READ_INPUT, OID_ESC_REG_FAULT, 9U, frame, sizeof(frame));
  return OID_ESC_SendFrame(bus, frame, len);
}

/**
  * @brief 请求读取 ABZ 编码器 Z 信号状态。
  * @param bus OID RS485 总线。
  * @param esc 电调对象。
  * @return HAL 发送状态。
  * @note  读取输入寄存器 5013(0x1395)，返回 1 表示本次上电已经找到 Z 信号。
  */
HAL_StatusTypeDef OID_ESC_RequestZSignal(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc)
{
  uint8_t frame[8];
  uint16_t len;

  if ((bus == 0) || (esc == 0))
  {
    return HAL_ERROR;
  }

  len = Modbus_RtuBuildRead(esc->id,
                            MODBUS_RTU_FUNC_READ_INPUT,
                            OID_ESC_REG_ENCODER_Z_FOUND,
                            1U,
                            frame,
                            sizeof(frame));
  return OID_ESC_SendFrame(bus, frame, len);
}

/**
  * @brief 解析一帧可能属于该电调的 Modbus 响应。
  * @param esc    电调对象。
  * @param frame  收到的完整帧。
  * @param len    帧长度。
  * @param now_ms 当前系统 tick。
  * @return 1 表示本帧已被该电调消费；0 表示不是该电调状态帧。
  */
uint8_t OID_ESC_HandleFrame(OID_ESC_Handle_t *esc, const uint8_t *frame, uint16_t len, uint32_t now_ms)
{
  const uint8_t *data;
  uint16_t expected_len;

  if ((esc == 0) || (frame == 0) || (len < 5U) || (frame[0] != esc->id))
  {
    return 0U;
  }

  if (Modbus_RtuCheckFrame(frame, len) == 0U)
  {
    esc->status.crc_error_count++;
    return 1U;
  }

  OID_ESC_PollDiagnostic(esc, now_ms);
  if ((frame[1] & 0x80U) != 0U && len == 5U)
  {
    esc->diagnostic.exceptions++;
    esc->diagnostic.last_exception = frame[2];
    if (frame[1] == (MODBUS_RTU_FUNC_READ_HOLDING | 0x80U))
      esc->diagnostic.control_pending = 0U;
    if (frame[1] == (MODBUS_RTU_FUNC_WRITE_MULTIPLE | 0x80U))
    {
      if (esc->diagnostic.speed_write_pending != 0U)
        esc->diagnostic.speed_unconfirmed++;
      esc->diagnostic.speed_write_pending = 0U;
    }
    return 1U;
  }
  if (frame[1] == MODBUS_RTU_FUNC_WRITE_MULTIPLE && len == 8U &&
      Modbus_RtuReadU16Be(&frame[2]) == OID_ESC_REG_TARGET_SPEED &&
      Modbus_RtuReadU16Be(&frame[4]) == 2U)
  {
    esc->diagnostic.speed_acks++;
    esc->diagnostic.speed_write_pending = 0U;
    return 1U;
  }
  if (frame[1] == MODBUS_RTU_FUNC_READ_HOLDING)
  {
    if (esc->diagnostic.control_pending != 0U && len == 13U && frame[2] == 8U)
    {
      esc->diagnostic.control_mode = Modbus_RtuReadU16Be(&frame[3]);
      /* Skip 0x1772 current target. */
      esc->diagnostic.target_erpm = Modbus_RtuReadI32Be(&frame[7]);
      esc->diagnostic.control_update_ms = now_ms;
      esc->diagnostic.control_reads++;
      esc->diagnostic.control_pending = 0U;
    }
    return 1U;
  }

  if (frame[1] != MODBUS_RTU_FUNC_READ_INPUT)
  {
    return 1U;
  }

  expected_len = (uint16_t)frame[2] + 5U;
  if (len != expected_len)
  {
    return 1U;
  }

  data = &frame[3];
  if (frame[2] == 2U)
  {
    esc->status.encoder_z_found = (Modbus_RtuReadU16Be(data) != 0U) ? 1U : 0U;
    esc->status.last_z_update_ms = now_ms;
    return 1U;
  }

  if (frame[2] != 18U)
  {
    return 1U;
  }

  esc->status.fault = Modbus_RtuReadU16Be(&data[0]);
  esc->status.speed_erpm = Modbus_RtuReadI32Be(&data[2]);
  esc->status.duty_permille = Modbus_RtuReadI16Be(&data[6]);
  esc->status.power_w = Modbus_RtuReadI16Be(&data[8]);
  esc->status.input_voltage_v = Modbus_RtuReadI16Be(&data[10]);
  esc->status.motor_current_10ma = Modbus_RtuReadI16Be(&data[12]);
  esc->status.bus_current_10ma = Modbus_RtuReadI16Be(&data[14]);
  esc->status.temperature_c = Modbus_RtuReadI16Be(&data[16]);
  esc->status.last_update_ms = now_ms;

  return 1U;
}
