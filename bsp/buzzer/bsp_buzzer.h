#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
/* 固定短音型；优先级显式定义，不依赖枚举顺序。 */
typedef enum
{
  BSP_BUZZER_CUE_SILENT = 0,  /* 静音。 */
  BSP_BUZZER_CUE_BOOT,        /* 软件启动上升三音，不表示放行。 */
  BSP_BUZZER_CUE_AUTO_READY,  /* 飞控自动实际放行短三音。 */
  BSP_BUZZER_CUE_RELEASE_WAIT,/* 等待回中/重臂：每5 s低双短音。 */
  BSP_BUZZER_CUE_OID_FAULT,   /* OID故障：每4 s一短音。 */
  BSP_BUZZER_CUE_FC_FAULT,    /* 飞控输入故障：每4 s两短音。 */
  BSP_BUZZER_CUE_STEER_FAULT  /* 转向故障：每4 s三短音。 */
} BSP_BuzzerCue;
uint8_t BSP_Buzzer_Init(TIM_HandleTypeDef *timer, uint32_t channel);
/* 低优先级事件直接丢弃，不积压过时的就绪音。 */
void BSP_Buzzer_Play(BSP_BuzzerCue cue, uint32_t now_ms);
void BSP_Buzzer_Stop(void);
void BSP_Buzzer_Task(uint32_t now_ms);
BSP_BuzzerCue BSP_Buzzer_GetCue(void);
#endif
