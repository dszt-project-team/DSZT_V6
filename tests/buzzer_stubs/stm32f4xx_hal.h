#ifndef BUZZER_TEST_HAL_H
#define BUZZER_TEST_HAL_H
#include <stdint.h>
typedef struct { uint32_t Prescaler; } TIM_Base_InitTypeDef;
typedef struct {
  TIM_Base_InitTypeDef Init;
  uint32_t autoreload, counter, compare;
} TIM_HandleTypeDef;
typedef struct { uint32_t CFGR; } RCC_TypeDef;
extern RCC_TypeDef test_rcc;
extern unsigned test_tone_writes;
#define RCC (&test_rcc)
#define RCC_CFGR_PPRE1 0x1C00U
#define TIM_CHANNEL_1 0U
#define HAL_OK 0
#define HAL_ERROR 1
typedef int HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_GetTick(void);
#define __HAL_TIM_SET_COMPARE(t,c,v) do { (void)(c); (t)->compare=(v); test_tone_writes++; } while (0)
#define __HAL_TIM_SET_AUTORELOAD(t,v) ((t)->autoreload=(v))
#define __HAL_TIM_SET_COUNTER(t,v) ((t)->counter=(v))
#endif
