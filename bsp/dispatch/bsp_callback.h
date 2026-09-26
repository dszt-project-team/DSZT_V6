/**
  ******************************************************************************
  * @file    bsp_callback.h
  * @brief   HAL 回调到 BSP/Module 实例的统一分发接口。
  ******************************************************************************
  */

#ifndef BSP_CALLBACK_H
#define BSP_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef void (*BspUartCallback)(void *parent, UART_HandleTypeDef *huart);
typedef void (*BspTimCallback)(void *parent, TIM_HandleTypeDef *htim);

typedef struct
{
  UART_HandleTypeDef *handle;
  void *parent;
  BspUartCallback rx_complete;
  BspUartCallback idle;
  BspUartCallback error;
} BspUartCallbackConfig;

typedef struct
{
  TIM_HandleTypeDef *handle;
  void *parent;
  BspTimCallback input_capture;
  BspTimCallback output_compare;
  BspTimCallback pwm_pulse_finished;
} BspTimCallbackConfig;

void BspCallback_Init(void);
uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *config);
uint8_t BspCallback_RegisterTim(const BspTimCallbackConfig *config);
void BspCallback_DispatchUartIdle(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CALLBACK_H */
