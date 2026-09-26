#include "bsp_buzzer.h"

#include <stddef.h>

typedef struct
{
  uint16_t frequency_hz;
  uint16_t duration_ms;
} BSP_BuzzerStep;

typedef struct
{
  TIM_HandleTypeDef *timer;
  const BSP_BuzzerStep *pattern;
  uint32_t channel;
  uint32_t phase_started_ms;
  BSP_BuzzerCue cue;
  BSP_BuzzerCue pending_cue;
  uint8_t phase_count;
  uint8_t phase;
  uint8_t repeating;
  uint8_t initialized;
} BSP_BuzzerContext;

static BSP_BuzzerContext s_buzzer;

/* 蜂鸣器-01：接收机上线采用三音上升，模拟飞控就绪提示。 */
static const BSP_BuzzerStep s_rc_connected_pattern[] =
{
  {2200U,  70U},
  {   0U,  35U},
  {2700U,  70U},
  {   0U,  35U},
  {3300U, 140U}
};

/* 蜂鸣器-02：除刹车外的有效模式切换统一使用同一组短促双音。 */
static const BSP_BuzzerStep s_mode_change_pattern[] =
{
  {2500U, 60U},
  {   0U, 30U},
  {3000U, 90U}
};

/* 蜂鸣器-03：进入任一刹车模式时使用三音下降，和普通模式切换明确区分。 */
static const BSP_BuzzerStep s_brake_pattern[] =
{
  {3300U,  70U},
  {   0U,  25U},
  {2700U,  70U},
  {   0U,  25U},
  {2200U, 140U}
};

/* 蜂鸣器-04：仅 左 OID 故障，低频长音每 2 秒重复一次。 */
static const BSP_BuzzerStep s_oid_left_fault_pattern[] =
{
  {2200U,  400U},
  {   0U, 1600U}
};

/* 蜂鸣器-05：仅 右 OID 故障，高频双短音每 2 秒重复一次。 */
static const BSP_BuzzerStep s_oid_right_fault_pattern[] =
{
  {3300U,  120U},
  {   0U,  100U},
  {3300U,  120U},
  {   0U, 1660U}
};

/* 蜂鸣器-06：双 OID 故障，高低交替四短音每 2 秒重复一次。 */
static const BSP_BuzzerStep s_oid_dual_fault_pattern[] =
{
  {2200U, 100U},
  {   0U,  70U},
  {3300U, 100U},
  {   0U,  70U},
  {2200U, 100U},
  {   0U,  70U},
  {3300U, 100U},
  {   0U, 1390U}
};

static uint32_t BSP_Buzzer_GetCounterClockHz(void)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

  /* STM32F4 的 APB1 分频不为 1 时，TIM12 时钟自动乘 2。 */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
  {
    timer_clock_hz *= 2U;
  }
  return timer_clock_hz / (s_buzzer.timer->Init.Prescaler + 1U);
}

static void BSP_Buzzer_SetTone(uint16_t frequency_hz)
{
  uint32_t period_counts;

  if ((s_buzzer.initialized == 0U) || (s_buzzer.timer == NULL))
  {
    return;
  }
  if (frequency_hz == 0U)
  {
    __HAL_TIM_SET_COMPARE(s_buzzer.timer, s_buzzer.channel, 0U);
    return;
  }

  period_counts =
      (BSP_Buzzer_GetCounterClockHz() + ((uint32_t)frequency_hz / 2U)) /
      (uint32_t)frequency_hz;
  if (period_counts < 2U)
  {
    period_counts = 2U;
  }
  if (period_counts > 65536U)
  {
    period_counts = 65536U;
  }

  __HAL_TIM_SET_AUTORELOAD(s_buzzer.timer, period_counts - 1U);
  __HAL_TIM_SET_COUNTER(s_buzzer.timer, 0U);
  __HAL_TIM_SET_COMPARE(s_buzzer.timer,
                        s_buzzer.channel,
                        period_counts / 2U);
}

static const BSP_BuzzerStep *BSP_Buzzer_GetPattern(BSP_BuzzerCue cue,
                                                    uint8_t *phase_count,
                                                    uint8_t *repeating)
{
  *phase_count = 0U;
  *repeating = 0U;
  switch (cue)
  {
    case BSP_BUZZER_CUE_RC_CONNECTED:
      *phase_count = (uint8_t)(sizeof(s_rc_connected_pattern) /
                               sizeof(s_rc_connected_pattern[0]));
      return s_rc_connected_pattern;
    case BSP_BUZZER_CUE_BRAKE:
      *phase_count = (uint8_t)(sizeof(s_brake_pattern) /
                               sizeof(s_brake_pattern[0]));
      return s_brake_pattern;
    case BSP_BUZZER_CUE_MODE_CHANGE:
      *phase_count = (uint8_t)(sizeof(s_mode_change_pattern) /
                               sizeof(s_mode_change_pattern[0]));
      return s_mode_change_pattern;
    case BSP_BUZZER_CUE_FC_MAIN1_TIMEOUT:
    case BSP_BUZZER_CUE_OID_LEFT_FAULT:
      *phase_count = (uint8_t)(sizeof(s_oid_left_fault_pattern) /
                               sizeof(s_oid_left_fault_pattern[0]));
      *repeating = 1U;
      return s_oid_left_fault_pattern;
    case BSP_BUZZER_CUE_FC_MAIN2_TIMEOUT:
    case BSP_BUZZER_CUE_OID_RIGHT_FAULT:
      *phase_count = (uint8_t)(sizeof(s_oid_right_fault_pattern) /
                               sizeof(s_oid_right_fault_pattern[0]));
      *repeating = 1U;
      return s_oid_right_fault_pattern;
    case BSP_BUZZER_CUE_FC_DUAL_TIMEOUT:
    case BSP_BUZZER_CUE_OID_DUAL_FAULT:
      *phase_count = (uint8_t)(sizeof(s_oid_dual_fault_pattern) /
                               sizeof(s_oid_dual_fault_pattern[0]));
      *repeating = 1U;
      return s_oid_dual_fault_pattern;
    case BSP_BUZZER_CUE_SILENT:
    default:
      return NULL;
  }
}

uint8_t BSP_Buzzer_Init(TIM_HandleTypeDef *timer, uint32_t channel)
{
  if (timer == NULL)
  {
    return 0U;
  }

  s_buzzer.timer = timer;
  s_buzzer.channel = channel;
  s_buzzer.pattern = NULL;
  s_buzzer.phase_started_ms = 0U;
  s_buzzer.cue = BSP_BUZZER_CUE_SILENT;
  s_buzzer.pending_cue = BSP_BUZZER_CUE_SILENT;
  s_buzzer.phase_count = 0U;
  s_buzzer.phase = 0U;
  s_buzzer.repeating = 0U;
  s_buzzer.initialized = 0U;

  __HAL_TIM_SET_COMPARE(timer, channel, 0U);
  if (HAL_TIM_PWM_Start(timer, channel) != HAL_OK)
  {
    return 0U;
  }

  s_buzzer.initialized = 1U;
  BSP_Buzzer_SetTone(0U);
  return 1U;
}

void BSP_Buzzer_Play(BSP_BuzzerCue cue, uint32_t now_ms)
{
  const BSP_BuzzerStep *pattern;
  uint8_t phase_count;
  uint8_t repeating;

  if ((s_buzzer.initialized == 0U) || (cue == BSP_BUZZER_CUE_SILENT))
  {
    return;
  }

  /* 蜂鸣器-07：高优先级提示可抢占低优先级节奏。 */
  if ((s_buzzer.cue != BSP_BUZZER_CUE_SILENT) && (cue < s_buzzer.cue))
  {
    /* 连接提示期间发生普通模式切换时保留一次，当前节奏结束后补播。 */
    s_buzzer.pending_cue = cue;
    return;
  }

  pattern = BSP_Buzzer_GetPattern(cue, &phase_count, &repeating);
  if ((pattern == NULL) || (phase_count == 0U))
  {
    return;
  }

  s_buzzer.cue = cue;
  s_buzzer.pending_cue = BSP_BUZZER_CUE_SILENT;
  s_buzzer.pattern = pattern;
  s_buzzer.phase_count = phase_count;
  s_buzzer.phase = 0U;
  s_buzzer.repeating = repeating;
  s_buzzer.phase_started_ms = now_ms;
  BSP_Buzzer_SetTone(pattern[0].frequency_hz);
}

void BSP_Buzzer_Stop(void)
{
  s_buzzer.cue = BSP_BUZZER_CUE_SILENT;
  s_buzzer.pending_cue = BSP_BUZZER_CUE_SILENT;
  s_buzzer.pattern = NULL;
  s_buzzer.phase_count = 0U;
  s_buzzer.phase = 0U;
  s_buzzer.repeating = 0U;
  BSP_Buzzer_SetTone(0U);
}

void BSP_Buzzer_Task(uint32_t now_ms)
{
  BSP_BuzzerCue pending_cue;
  uint16_t duration_ms;

  if ((s_buzzer.initialized == 0U) ||
      (s_buzzer.cue == BSP_BUZZER_CUE_SILENT) ||
      (s_buzzer.pattern == NULL))
  {
    return;
  }

  duration_ms = s_buzzer.pattern[s_buzzer.phase].duration_ms;
  while ((uint32_t)(now_ms - s_buzzer.phase_started_ms) >= duration_ms)
  {
    s_buzzer.phase_started_ms += duration_ms;
    s_buzzer.phase++;
    if (s_buzzer.phase >= s_buzzer.phase_count)
    {
      if (s_buzzer.repeating != 0U)
      {
        s_buzzer.phase = 0U;
        BSP_Buzzer_SetTone(s_buzzer.pattern[0].frequency_hz);
        duration_ms = s_buzzer.pattern[0].duration_ms;
        continue;
      }
      pending_cue = s_buzzer.pending_cue;
      BSP_Buzzer_Stop();
      if (pending_cue != BSP_BUZZER_CUE_SILENT)
      {
        BSP_Buzzer_Play(pending_cue, now_ms);
      }
      return;
    }
    BSP_Buzzer_SetTone(s_buzzer.pattern[s_buzzer.phase].frequency_hz);
    duration_ms = s_buzzer.pattern[s_buzzer.phase].duration_ms;
  }
}

BSP_BuzzerCue BSP_Buzzer_GetCue(void)
{
  return s_buzzer.cue;
}
