#include "bsp_buzzer.h"

#include <stddef.h>
#include <string.h>

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
  uint32_t started_ms;
  uint32_t period_ms;
  BSP_BuzzerCue cue;
  uint8_t phase_count;
  uint8_t phase;
  uint8_t repeating;
  uint8_t initialized;
} BSP_BuzzerContext;

static BSP_BuzzerContext s_buzzer;

/* 音型仅用于简洁状态辨识；频率为1500～3300 Hz范围内的固定值。 */
static const BSP_BuzzerStep s_boot[] =
  {{1800U,60U},{0U,30U},{2400U,60U},{0U,30U},{3000U,80U}};
static const BSP_BuzzerStep s_auto[] =
  {{2200U,50U},{0U,30U},{2700U,50U},{0U,30U},{3100U,70U}};
static const BSP_BuzzerStep s_release_wait[] =
  {{1800U,55U},{0U,45U},{1800U,55U},{0U,4845U}};
static const BSP_BuzzerStep s_oid_fault[] =
  {{1700U,120U},{0U,3880U}};
static const BSP_BuzzerStep s_fc_fault[] =
  {{2600U,90U},{0U,90U},{2600U,90U},{0U,3730U}};
static const BSP_BuzzerStep s_steer_fault[] =
  {{2000U,90U},{0U,90U},{2000U,90U},{0U,90U},{2000U,90U},{0U,3550U}};

static uint8_t BSP_Buzzer_Priority(BSP_BuzzerCue cue)
{
  switch (cue)
  {
    case BSP_BUZZER_CUE_STEER_FAULT: return 90U;
    case BSP_BUZZER_CUE_FC_FAULT: return 100U;
    case BSP_BUZZER_CUE_OID_FAULT: return 80U;
    case BSP_BUZZER_CUE_RELEASE_WAIT: return 60U;
    case BSP_BUZZER_CUE_AUTO_READY: return 30U;
    case BSP_BUZZER_CUE_BOOT: return 10U;
    default: return 0U;
  }
}

static uint32_t BSP_Buzzer_GetCounterClockHz(void)
{
  uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
  /* APB1分频不为1时，STM32F4的TIM12时钟乘2；不修改CubeMX预分频配置。 */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) clock_hz *= 2U;
  return clock_hz / (s_buzzer.timer->Init.Prescaler + 1U);
}

static void BSP_Buzzer_SetTone(uint16_t frequency_hz)
{
  uint32_t counts;
  if (s_buzzer.initialized == 0U || s_buzzer.timer == NULL) return;
  if (frequency_hz == 0U)
  {
    __HAL_TIM_SET_COMPARE(s_buzzer.timer, s_buzzer.channel, 0U);
    return;
  }
  counts = (BSP_Buzzer_GetCounterClockHz() + frequency_hz / 2U) / frequency_hz;
  if (counts < 2U) counts = 2U;
  if (counts > 65536U) counts = 65536U;
  __HAL_TIM_SET_AUTORELOAD(s_buzzer.timer, counts - 1U);
  __HAL_TIM_SET_COUNTER(s_buzzer.timer, 0U);
  __HAL_TIM_SET_COMPARE(s_buzzer.timer, s_buzzer.channel, counts / 2U);
}

static const BSP_BuzzerStep *BSP_Buzzer_GetPattern(BSP_BuzzerCue cue,
                                                  uint8_t *count,
                                                  uint8_t *repeating)
{
  *count = 0U;
  *repeating = 0U;
  switch (cue)
  {
    case BSP_BUZZER_CUE_BOOT: *count=5U; return s_boot;
    case BSP_BUZZER_CUE_AUTO_READY: *count=5U; return s_auto;
    case BSP_BUZZER_CUE_RELEASE_WAIT: *count=4U; *repeating=1U; return s_release_wait;
    case BSP_BUZZER_CUE_OID_FAULT: *count=2U; *repeating=1U; return s_oid_fault;
    case BSP_BUZZER_CUE_FC_FAULT: *count=4U; *repeating=1U; return s_fc_fault;
    case BSP_BUZZER_CUE_STEER_FAULT: *count=6U; *repeating=1U; return s_steer_fault;
    default: return NULL;
  }
}

uint8_t BSP_Buzzer_Init(TIM_HandleTypeDef *timer, uint32_t channel)
{
  if (s_buzzer.initialized != 0U) BSP_Buzzer_Stop();
  memset(&s_buzzer, 0, sizeof(s_buzzer));
  if (timer == NULL) return 0U;
  s_buzzer.timer = timer;
  s_buzzer.channel = channel;
  __HAL_TIM_SET_COMPARE(timer, channel, 0U);
  if (HAL_TIM_PWM_Start(timer, channel) != HAL_OK) return 0U;
  s_buzzer.initialized = 1U;
  return 1U;
}

void BSP_Buzzer_Play(BSP_BuzzerCue cue, uint32_t now_ms)
{
  const BSP_BuzzerStep *pattern;
  uint8_t count, repeating, i;
  uint32_t period_ms = 0U;
  if (s_buzzer.initialized == 0U || cue == BSP_BUZZER_CUE_SILENT) return;
  /* 不保留待播队列：过时的就绪音不能在故障恢复后误播。 */
  if (BSP_Buzzer_Priority(cue) < BSP_Buzzer_Priority(s_buzzer.cue)) return;
  if (cue == s_buzzer.cue) return;
  pattern = BSP_Buzzer_GetPattern(cue, &count, &repeating);
  if (pattern == NULL || count == 0U) return;
  for (i=0U; i<count; i++) period_ms += pattern[i].duration_ms;
  if (period_ms == 0U) return;
  s_buzzer.pattern = pattern;
  s_buzzer.cue = cue;
  s_buzzer.started_ms = now_ms;
  s_buzzer.period_ms = period_ms;
  s_buzzer.phase_count = count;
  s_buzzer.phase = 0U;
  s_buzzer.repeating = repeating;
  BSP_Buzzer_SetTone(pattern[0].frequency_hz);
}

void BSP_Buzzer_Stop(void)
{
  s_buzzer.cue = BSP_BUZZER_CUE_SILENT;
  s_buzzer.pattern = NULL;
  s_buzzer.phase_count = 0U;
  s_buzzer.phase = 0U;
  s_buzzer.repeating = 0U;
  s_buzzer.period_ms = 0U;
  BSP_Buzzer_SetTone(0U);
}

void BSP_Buzzer_Task(uint32_t now_ms)
{
  uint32_t elapsed, position;
  uint8_t phase;
  if (s_buzzer.initialized == 0U || s_buzzer.pattern == NULL || s_buzzer.period_ms == 0U) return;
  elapsed = now_ms - s_buzzer.started_ms;
  if (s_buzzer.repeating == 0U && elapsed >= s_buzzer.period_ms)
  {
    BSP_Buzzer_Stop();
    return;
  }
  if (s_buzzer.repeating != 0U)
  {
    position = elapsed % s_buzzer.period_ms;
    s_buzzer.started_ms = now_ms - position;
  }
  else position = elapsed;
  /* 最多检查六个固定音段；时间长跳时直接跳到当下，不追赶或补播旧节拍。 */
  for (phase=0U; phase<s_buzzer.phase_count; phase++)
  {
    if (position < s_buzzer.pattern[phase].duration_ms) break;
    position -= s_buzzer.pattern[phase].duration_ms;
  }
  if (phase < s_buzzer.phase_count && phase != s_buzzer.phase)
  {
    s_buzzer.phase = phase;
    BSP_Buzzer_SetTone(s_buzzer.pattern[phase].frequency_hz);
  }
}

BSP_BuzzerCue BSP_Buzzer_GetCue(void)
{
  return s_buzzer.cue;
}
