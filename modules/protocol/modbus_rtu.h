/**
  ******************************************************************************
  * @file    modbus_rtu.h
  * @brief   Modbus RTU 组帧、校验与解析协议模块。
  ******************************************************************************
  */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Modbus 功能码 0x03：读取保持寄存器。 */
#define MODBUS_RTU_FUNC_READ_HOLDING     0x03U
/* Modbus 功能码 0x04：读取输入寄存器。 */
#define MODBUS_RTU_FUNC_READ_INPUT       0x04U
/* Modbus 功能码 0x06：写单个保持寄存器。 */
#define MODBUS_RTU_FUNC_WRITE_SINGLE     0x06U
/* Modbus 功能码 0x10：连续写多个保持寄存器。 */
#define MODBUS_RTU_FUNC_WRITE_MULTIPLE   0x10U

uint16_t Modbus_RtuCrc16(const uint8_t *data, uint16_t len);
uint8_t Modbus_RtuCheckFrame(const uint8_t *frame, uint16_t len);
uint16_t Modbus_RtuBuildWriteSingle(uint8_t id, uint16_t reg, uint16_t value,
                                    uint8_t *out, uint16_t out_size);
uint16_t Modbus_RtuBuildWriteMultipleU16(uint8_t id, uint16_t reg, const uint16_t *values,
                                         uint16_t reg_count, uint8_t *out, uint16_t out_size);
uint16_t Modbus_RtuBuildRead(uint8_t id, uint8_t function, uint16_t reg, uint16_t reg_count,
                             uint8_t *out, uint16_t out_size);
int16_t Modbus_RtuReadI16Be(const uint8_t *data);
uint16_t Modbus_RtuReadU16Be(const uint8_t *data);
int32_t Modbus_RtuReadI32Be(const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
