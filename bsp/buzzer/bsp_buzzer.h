#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

/** A 板板载无源蜂鸣器的提示类型；枚举值同时表示抢占优先级。 */
typedef enum
{
  BSP_BUZZER_CUE_SILENT = 0,
  BSP_BUZZER_CUE_MODE_CHANGE,
  BSP_BUZZER_CUE_RC_CONNECTED,
  BSP_BUZZER_CUE_BRAKE,
  BSP_BUZZER_CUE_OID_LEFT_FAULT,
  BSP_BUZZER_CUE_OID_RIGHT_FAULT,
  BSP_BUZZER_CUE_OID_DUAL_FAULT,
  BSP_BUZZER_CUE_FC_MAIN1_TIMEOUT,
  BSP_BUZZER_CUE_FC_MAIN2_TIMEOUT,
  BSP_BUZZER_CUE_FC_DUAL_TIMEOUT
} BSP_BuzzerCue;

uint8_t BSP_Buzzer_Init(TIM_HandleTypeDef *timer, uint32_t channel);
void BSP_Buzzer_Play(BSP_BuzzerCue cue, uint32_t now_ms);
void BSP_Buzzer_Stop(void);
void BSP_Buzzer_Task(uint32_t now_ms);
BSP_BuzzerCue BSP_Buzzer_GetCue(void);

#endif /* BSP_BUZZER_H */
