#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 板载无源蜂鸣器音型；优先级由实现显式分配，不依赖枚举顺序。 */
typedef enum
{
  BSP_BUZZER_CUE_SILENT = 0,        /* 静音。 */
  BSP_BUZZER_CUE_MODE_CHANGE,       /* 兼容旧调用：等同手动实际放行音。 */
  BSP_BUZZER_CUE_RC_CONNECTED,      /* 接收机首次健康上线短双音。 */
  BSP_BUZZER_CUE_BRAKE,             /* 主动锁车下降双音。 */
  BSP_BUZZER_CUE_OID_LEFT_FAULT,    /* 兼容左OID故障，使用统一OID音型。 */
  BSP_BUZZER_CUE_OID_RIGHT_FAULT,   /* 兼容右OID故障，使用统一OID音型。 */
  BSP_BUZZER_CUE_OID_DUAL_FAULT,    /* OID故障：每4 s一短音。 */
  BSP_BUZZER_CUE_FC_MAIN1_TIMEOUT,  /* 兼容MAIN1故障，使用统一飞控音型。 */
  BSP_BUZZER_CUE_FC_MAIN2_TIMEOUT,  /* 兼容MAIN2故障，使用统一飞控音型。 */
  BSP_BUZZER_CUE_FC_DUAL_TIMEOUT,   /* 飞控故障：每4 s两短音。 */
  BSP_BUZZER_CUE_BOOT,              /* 软件开机上升三音，不代表放行。 */
  BSP_BUZZER_CUE_MANUAL_READY,      /* 手动实际放行上升双音。 */
  BSP_BUZZER_CUE_AUTO_READY,        /* 自动实际放行短三音。 */
  BSP_BUZZER_CUE_STARTUP_WAIT,      /* 等待重新锁车：每5 s低双短音。 */
  BSP_BUZZER_CUE_STEER_FAULT,       /* 转向故障：每4 s三短音。 */
  BSP_BUZZER_CUE_RC_LOST            /* 遥控失联：每3 s高低双短音。 */
} BSP_BuzzerCue;

uint8_t BSP_Buzzer_Init(TIM_HandleTypeDef *timer, uint32_t channel);
/* 较低优先级事件直接丢弃，不排队，不会在故障恢复后补播。 */
void BSP_Buzzer_Play(BSP_BuzzerCue cue, uint32_t now_ms);
void BSP_Buzzer_Stop(void);
void BSP_Buzzer_Task(uint32_t now_ms);
BSP_BuzzerCue BSP_Buzzer_GetCue(void);

#endif
