/**
  ******************************************************************************
  * @file    modbus_rtu.c
  * @brief   Modbus RTU 组帧、校验与解析协议模块实现。
  ******************************************************************************
  */

#include "modbus_rtu.h"
#include <string.h>

/**
  * @brief 计算 Modbus RTU CRC16。
  * @param data 指向待校验数据，不包含 CRC 字段。
  * @param len  待校验字节数。
  * @return CRC16 结果。发送时低字节先放入帧尾，可匹配 OID/WIT 示例帧。
  */
uint16_t Modbus_RtuCrc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/**
  * @brief 检查一帧 Modbus RTU 数据的 CRC 是否正确。
  * @param frame 完整帧，最后 2 字节为 CRC。
  * @param len   完整帧长度。
  * @return 1 表示 CRC 正确，0 表示长度不足或 CRC 错误。
  */
uint8_t Modbus_RtuCheckFrame(const uint8_t *frame, uint16_t len)
{
  uint16_t crc_calc;
  uint16_t crc_recv;

  if ((frame == 0) || (len < 4U))
  {
    return 0U;
  }

  crc_calc = Modbus_RtuCrc16(frame, (uint16_t)(len - 2U));
  crc_recv = (uint16_t)frame[len - 2U] | ((uint16_t)frame[len - 1U] << 8);

  return (crc_calc == crc_recv) ? 1U : 0U;
}

/**
  * @brief 追加 CRC 到帧尾。
  * @param frame    待追加 CRC 的帧缓存。
  * @param data_len 当前不含 CRC 的长度。
  * @return 追加 CRC 后的完整长度。
  */
static uint16_t Modbus_RtuAppendCrc(uint8_t *frame, uint16_t data_len)
{
  uint16_t crc = Modbus_RtuCrc16(frame, data_len);

  frame[data_len] = (uint8_t)(crc & 0xFFU);
  frame[data_len + 1U] = (uint8_t)(crc >> 8);

  return (uint16_t)(data_len + 2U);
}

/**
  * @brief 组功能码 0x06 写单个保持寄存器帧。
  * @param id       从站地址。
  * @param reg      寄存器地址。
  * @param value    写入值。
  * @param out      输出帧缓存。
  * @param out_size 输出帧缓存大小，至少 8 字节。
  * @return 完整帧长度，0 表示缓存不足或参数错误。
  */
uint16_t Modbus_RtuBuildWriteSingle(uint8_t id, uint16_t reg, uint16_t value,
                                    uint8_t *out, uint16_t out_size)
{
  if ((out == 0) || (out_size < 8U))
  {
    return 0U;
  }

  out[0] = id;
  out[1] = MODBUS_RTU_FUNC_WRITE_SINGLE;
  out[2] = (uint8_t)(reg >> 8);
  out[3] = (uint8_t)(reg & 0xFFU);
  out[4] = (uint8_t)(value >> 8);
  out[5] = (uint8_t)(value & 0xFFU);

  return Modbus_RtuAppendCrc(out, 6U);
}

/**
  * @brief 组功能码 0x10 连续写多个 16 位保持寄存器帧。
  * @param id        从站地址。
  * @param reg       起始寄存器地址。
  * @param values    待写入的 16 位寄存器数组，按寄存器顺序排列。
  * @param reg_count 寄存器个数。
  * @param out       输出帧缓存。
  * @param out_size  输出帧缓存大小。
  * @return 完整帧长度，0 表示缓存不足或参数错误。
  */
uint16_t Modbus_RtuBuildWriteMultipleU16(uint8_t id, uint16_t reg, const uint16_t *values,
                                         uint16_t reg_count, uint8_t *out, uint16_t out_size)
{
  uint16_t i;
  uint16_t data_len;

  if ((values == 0) || (out == 0) || (reg_count == 0U) || (reg_count > 120U))
  {
    return 0U;
  }

  data_len = (uint16_t)(7U + (reg_count * 2U));
  if (out_size < (uint16_t)(data_len + 2U))
  {
    return 0U;
  }

  out[0] = id;
  out[1] = MODBUS_RTU_FUNC_WRITE_MULTIPLE;
  out[2] = (uint8_t)(reg >> 8);
  out[3] = (uint8_t)(reg & 0xFFU);
  out[4] = (uint8_t)(reg_count >> 8);
  out[5] = (uint8_t)(reg_count & 0xFFU);
  out[6] = (uint8_t)(reg_count * 2U);

  for (i = 0; i < reg_count; i++)
  {
    out[7U + (i * 2U)] = (uint8_t)(values[i] >> 8);
    out[8U + (i * 2U)] = (uint8_t)(values[i] & 0xFFU);
  }

  return Modbus_RtuAppendCrc(out, data_len);
}

/**
  * @brief 组读寄存器帧。
  * @param id        从站地址。
  * @param function  读功能码，通常为 0x03 或 0x04。
  * @param reg       起始寄存器地址。
  * @param reg_count 读取寄存器数量。
  * @param out       输出帧缓存。
  * @param out_size  输出帧缓存大小，至少 8 字节。
  * @return 完整帧长度，0 表示参数错误。
  */
uint16_t Modbus_RtuBuildRead(uint8_t id, uint8_t function, uint16_t reg, uint16_t reg_count,
                             uint8_t *out, uint16_t out_size)
{
  if ((out == 0) || (out_size < 8U) || (reg_count == 0U))
  {
    return 0U;
  }

  out[0] = id;
  out[1] = function;
  out[2] = (uint8_t)(reg >> 8);
  out[3] = (uint8_t)(reg & 0xFFU);
  out[4] = (uint8_t)(reg_count >> 8);
  out[5] = (uint8_t)(reg_count & 0xFFU);

  return Modbus_RtuAppendCrc(out, 6U);
}

/**
  * @brief 从大端字节流读取有符号 16 位数。
  */
int16_t Modbus_RtuReadI16Be(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/**
  * @brief 从大端字节流读取无符号 16 位数。
  */
uint16_t Modbus_RtuReadU16Be(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/**
  * @brief 从大端字节流读取有符号 32 位数。
  */
int32_t Modbus_RtuReadI32Be(const uint8_t *data)
{
  uint32_t value = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   ((uint32_t)data[3]);

  return (int32_t)value;
}
