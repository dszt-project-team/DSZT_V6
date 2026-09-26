/**
  ******************************************************************************
  * @file    oid_esc.h
  * @brief   OID-RS485-500W 单驱电调设备模块。
  ******************************************************************************
  */

#ifndef OID_ESC_H
#define OID_ESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_rs485.h"
#include <stdint.h>

/* 故障码寄存器；0 表示无故障，非 0 需结合 OID 手册排查。 */
#define OID_ESC_REG_FAULT              0x1388U
/* ABZ 兼容寄存器；V6 霍尔版本不读取，也不据此门控速度闭环。 */
#define OID_ESC_REG_ENCODER_Z_FOUND    0x1395U
/* 心跳寄存器；整车运行时按 1/2 交替写入，防止电调通信超时停机。 */
#define OID_ESC_REG_HEARTBEAT          0x1770U
/* 控制模式寄存器；写入 OID_ESC_Mode_t 中的模式值。 */
#define OID_ESC_REG_CONTROL_MODE       0x1771U
/* 目标电流寄存器，当前项目未作为常规行走控制入口。 */
#define OID_ESC_REG_TARGET_CURRENT     0x1772U
/* 目标速度寄存器，单位 ERPM；CH3 前后走最终写这里。 */
#define OID_ESC_REG_TARGET_SPEED       0x1773U
/* 目标占空比寄存器；仅供禁用的 ABZ 兼容路径使用，V6 正常行走不使用。 */
#define OID_ESC_REG_TARGET_DUTY        0x1775U
/* 电子刹车电流寄存器，单位 10mA；V6 整车控制逻辑不调用。 */
#define OID_ESC_REG_BRAKE_CURRENT      0x177EU

/* OID 故障码 19：本次上电尚未找到编码器 Z 信号。 */
#define OID_ESC_FAULT_ENCODER_Z_NOT_FOUND 19U

typedef enum
{
  OID_ESC_MODE_CURRENT = 0,       /* 电流闭环模式，直接控制相电流。 */
  OID_ESC_MODE_SPEED = 1,         /* 速度闭环模式，写 TARGET_SPEED 后用于正常前进/后退。 */
  OID_ESC_MODE_DUTY = 2,          /* 占空比模式；V6 霍尔行走不使用。 */
  OID_ESC_MODE_ABS_POS = 3,       /* 绝对位置模式，当前前驱不使用。 */
  OID_ESC_MODE_REL_LAST_POS = 4,  /* 以上次目标为基准的相对位置模式，当前不使用。 */
  OID_ESC_MODE_REL_NOW_POS = 5,   /* 以当前位置为基准的相对位置模式，当前不使用。 */
  OID_ESC_MODE_BRAKE = 6,         /* 电子刹车诊断模式，V6 整车逻辑不进入该模式。 */
  OID_ESC_MODE_HAND_BRAKE = 7,    /* 手刹模式，当前整车逻辑不使用。 */
  OID_ESC_MODE_HOMING = 8,        /* 回零模式，当前前驱不使用。 */
  OID_ESC_MODE_HOMING_STOP = 9,   /* 回零停止模式，当前前驱不使用。 */
  OID_ESC_MODE_CURRENT_RAMP = 10, /* 电流斜坡模式，当前前驱不使用。 */
  OID_ESC_MODE_IDLE = 0xFFFF      /* 软件内部占位值，不直接写入电调。 */
} OID_ESC_Mode_t;

typedef struct
{
  uint16_t fault;            /* 故障码，0 表示无故障。 */
  int32_t speed_erpm;        /* 实时电角速度，erpm=rpm*磁极对数。 */
  int16_t duty_permille;     /* 实时占空比，-1000~1000。 */
  int16_t power_w;           /* 实时功率，单位 W。 */
  int16_t input_voltage_v;   /* 输入电压，单位 V。 */
  int16_t motor_current_10ma;/* 电机相电流，单位 10mA。 */
  int16_t bus_current_10ma;  /* 总线电流，单位 10mA。 */
  int16_t temperature_c;     /* 驱动器温度，单位摄氏度。 */
  uint8_t encoder_z_found;   /* ABZ 兼容状态；V6 霍尔版本忽略。 */
  uint32_t last_update_ms;   /* 最近一次解析到状态帧的 tick。 */
  uint32_t last_z_update_ms; /* 最近一次解析到 Z 信号查询响应的 tick。 */
  uint16_t crc_error_count;  /* CRC 错误累计，便于排查 A/B 接反、干扰或波特率错误。 */
  uint16_t timeout_count;    /* 通信超时累计。 */
} OID_ESC_Status_t;

typedef struct
{
  uint16_t control_mode;       /* 0x1771 readback, not the local mode cache. */
  int32_t target_erpm;         /* 0x1773/4 readback, not the MCU requested target. */
  uint32_t control_update_ms;
  uint32_t control_request_ms;
  uint32_t speed_write_ms;
  uint16_t control_reads;
  uint16_t control_timeouts;
  uint16_t zero_writes;
  uint16_t speed_acks;
  uint16_t speed_unconfirmed;
  uint16_t exceptions;
  uint8_t last_exception;
  uint8_t control_pending;
  uint8_t speed_write_pending;
} OID_ESC_Diagnostic_t;

typedef struct
{
  uint8_t id;                /* Modbus 从站 ID。 */
  uint8_t pole_pairs;        /* 电机极对数，用于 rpm 与 erpm 换算。 */
  uint16_t heartbeat_value;  /* 心跳寄存器轮询值，当前按 1/2 循环。 */
  OID_ESC_Status_t status;   /* 最近一次状态快照。 */
  OID_ESC_Diagnostic_t diagnostic; /* Evidence only; never grants motion permission. */
} OID_ESC_Handle_t;

void OID_ESC_Init(OID_ESC_Handle_t *esc, uint8_t id, uint8_t pole_pairs);
HAL_StatusTypeDef OID_ESC_SendHeartbeat(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc);
HAL_StatusTypeDef OID_ESC_SetControlMode(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, OID_ESC_Mode_t mode);
HAL_StatusTypeDef OID_ESC_SetSpeedErpm(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, int32_t speed_erpm);
HAL_StatusTypeDef OID_ESC_SetSpeedRpm(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, int32_t speed_rpm);
HAL_StatusTypeDef OID_ESC_SetDutyPermille(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, int16_t duty_permille);
HAL_StatusTypeDef OID_ESC_SetBrakeCurrent(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, uint16_t current_10ma);
HAL_StatusTypeDef OID_ESC_RequestStatus(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc);
HAL_StatusTypeDef OID_ESC_RequestControl(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc, uint32_t now_ms);
void OID_ESC_PollDiagnostic(OID_ESC_Handle_t *esc, uint32_t now_ms);
HAL_StatusTypeDef OID_ESC_RequestZSignal(BSP_RS485_Bus_t *bus, OID_ESC_Handle_t *esc);
uint8_t OID_ESC_HandleFrame(OID_ESC_Handle_t *esc, const uint8_t *frame, uint16_t len, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* OID_ESC_H */
