/* Real nonblocking TIM buzzer scheduler; only HAL registers are mocked. */
#include <assert.h>
#include <stdio.h>
#include "../bsp/buzzer/bsp_buzzer.c"
RCC_TypeDef test_rcc;
unsigned test_tone_writes;
static uint32_t tick_ms;
static HAL_StatusTypeDef start_result;
TIM_HandleTypeDef htim12;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c) {(void)t;(void)c;return start_result;}
uint32_t HAL_RCC_GetPCLK1Freq(void) {return 42000000U;}
uint32_t HAL_GetTick(void) {return tick_ms;}

int main(void)
{
  uint8_t i,count,repeat;
  uint32_t duration;
  unsigned before;
  const BSP_BuzzerStep *pattern;
  htim12.Init.Prescaler=83U;
  test_rcc.CFGR=RCC_CFGR_PPRE1;
  assert(BSP_Buzzer_Init(NULL,TIM_CHANNEL_1)==0);
  start_result=HAL_ERROR;
  assert(BSP_Buzzer_Init(&htim12,TIM_CHANNEL_1)==0);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_BOOT,0);assert(htim12.compare==0);
  start_result=HAL_OK;
  assert(BSP_Buzzer_Init(&htim12,TIM_CHANNEL_1)==1);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_BOOT,0);
  assert(htim12.autoreload==555U && htim12.compare==278U);
  BSP_Buzzer_Task(60);assert(htim12.compare==0);
  BSP_Buzzer_Task(90);assert(htim12.compare>0);
  BSP_Buzzer_Task(260);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT && htim12.compare==0);

  for(i=BSP_BUZZER_CUE_MODE_CHANGE;i<=BSP_BUZZER_CUE_RC_LOST;i++)
  {
    pattern=BSP_Buzzer_GetPattern((BSP_BuzzerCue)i,&count,&repeat);
    assert(pattern && count>0 && count<=6);
    duration=0;
    for(unsigned j=0;j<count;j++)
    {
      assert(pattern[j].duration_ms>0);
      assert(pattern[j].frequency_hz==0 || (pattern[j].frequency_hz>=1500 && pattern[j].frequency_hz<=3300));
      duration+=pattern[j].duration_ms;
    }
    BSP_Buzzer_Stop();BSP_Buzzer_Play((BSP_BuzzerCue)i,1000);
    assert(s_buzzer.period_ms==duration && s_buzzer.repeating==repeat);
    before=test_tone_writes;
    BSP_Buzzer_Task(0x7FFFFFF0U);
    assert(test_tone_writes-before<=1); /* No catch-up loop, regardless of elapsed time. */
    assert(repeat || BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  }
  BSP_Buzzer_Stop();BSP_Buzzer_Play(BSP_BUZZER_CUE_MANUAL_READY,100);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_OID_DUAL_FAULT,110);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_OID_DUAL_FAULT);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_AUTO_READY,120); /* Lower event discarded, never queued. */
  BSP_Buzzer_Play(BSP_BUZZER_CUE_RC_LOST,130);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RC_LOST);
  BSP_Buzzer_Stop();BSP_Buzzer_Task(10000);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT && htim12.compare==0);

  BSP_Buzzer_Play(BSP_BUZZER_CUE_RC_LOST,UINT32_MAX-40U);
  BSP_Buzzer_Task(48U);assert(s_buzzer.phase==0 && htim12.compare>0);
  BSP_Buzzer_Task(49U);assert(s_buzzer.phase==1 && htim12.compare==0);
  BSP_Buzzer_Task(139U);assert(s_buzzer.phase==2 && htim12.compare>0);
  BSP_Buzzer_Task(2959U);assert(s_buzzer.phase==0 && htim12.compare>0);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_RC_LOST,3000U);
  assert(s_buzzer.started_ms==2959U); /* Same request cannot restart the repeat pattern. */
  BSP_Buzzer_Stop();BSP_Buzzer_Play(BSP_BUZZER_CUE_BOOT,UINT32_MAX-40U);
  BSP_Buzzer_Task(219U);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  puts("PASS: buzzer PWM init/tone, bounded patterns/catch-up, priorities/no stale queue, stop, wrap");
  return 0;
}
