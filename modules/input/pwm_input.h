/**
  ******************************************************************************
  * @file    pwm_input.h
  * @brief   飞控 Servo PWM 输入捕获、滤波和超时诊断。
  ******************************************************************************
  */

#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  PWM_INPUT_FAULT_NONE = 0,
  PWM_INPUT_FAULT_NOT_READY,
  PWM_INPUT_FAULT_TIMEOUT,
  PWM_INPUT_FAULT_RANGE,
  PWM_INPUT_FAULT_WARMUP
} PwmInput_Fault_t;

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint16_t minimum_valid_us;
  uint16_t maximum_valid_us;
  uint32_t timeout_ms;
  uint32_t transient_fault_hold_ms;
  uint8_t average_window;          /* 均值窗 1..8；1 表示直接使用最新有效脉宽。 */
  uint8_t valid_samples_to_online; /* 连续有效帧预热 1..255，与均值窗独立。 */
} PwmInput_Config_t;

typedef struct
{
  uint16_t raw_us;
  uint16_t filtered_us;
  uint16_t last_invalid_us;
  uint32_t last_valid_ms;
  uint32_t last_invalid_ms;
  uint32_t valid_pulse_count;
  uint32_t invalid_pulse_count;
  /* 累计断流事件；好帧恢复不清零，任务据此撤销断流前的控制释放。 */
  uint32_t timeout_event_count;
  PwmInput_Fault_t fault;
  uint8_t online;
} PwmInput_Snapshot_t;

typedef struct PwmInput_Handle PwmInput_Handle_t;

PwmInput_Handle_t *PwmInput_Register(const PwmInput_Config_t *config);
/* now_ms 保留兼容调用接口；实际健康判定在原子快照内重新读取 HAL 时基。 */
uint8_t PwmInput_GetSnapshot(PwmInput_Handle_t *handle,
                             PwmInput_Snapshot_t *snapshot,
                             uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* PWM_INPUT_H */
