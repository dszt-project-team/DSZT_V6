/* Host regression: real MT6826S module, hardware primitives only are mocked. */
#include <stdio.h>
#include <string.h>
#include "../modules/encoder/mt6826s_pwm.c"

GPIO_TypeDef test_gpio_ports[9];
static uint32_t tick, irq_mask;
static unsigned failures, sampled_without_mask;
static TIM_HandleTypeDef timer;
static Mt6826sPwm_Handle_t encoder;

uint32_t HAL_GetTick(void)
{ if (irq_mask == 0U) sampled_without_mask++; return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{ (void)port; (void)config; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)pin; return port->level; }
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *handle, TIM_IC_InitTypeDef *config, uint32_t channel)
{ (void)handle; (void)config; (void)channel; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *handle, uint32_t channel)
{ (void)handle; (void)channel; return HAL_OK; }
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *handle, uint32_t channel)
{ (void)channel; return handle->capture; }
uint8_t BspCallback_RegisterTim(const BspTimCallbackConfig *config)
{ (void)config; return 1U; }

static void check(int pass, const char *name)
{ printf("%s: %s\n", pass ? "PASS" : "FAIL", name); if (!pass) failures++; }

static void fresh_frame(uint32_t now)
{
  tick = now;
  /* 50% duty, 1 ms valid encoder period; exercise the real decoder. */
  Mt6826sPwm_ProcessFrame(&encoder, 500U, 1000U, now);
}

int main(void)
{
  Mt6826sPwm_Snapshot_t snapshot;
  Mt6826sPwm_Init(&encoder, &timer, TIM_CHANNEL_1, GPIOA, 1U, 0U);
  check(Mt6826sPwm_IsHealthy(&encoder, 0U) == 0U, "uninitialized feedback is never healthy");
  fresh_frame(1001U);
  Mt6826sPwm_Task(&encoder, 1000U);
  check(Mt6826sPwm_IsHealthy(&encoder, 1000U) != 0U, "new ISR timestamp is not rejected by stale caller tick");
  tick = 1101U;
  check(Mt6826sPwm_IsHealthy(&encoder, tick) != 0U, "100 ms age boundary remains healthy");
  tick = 1102U;
  check(Mt6826sPwm_IsHealthy(&encoder, tick) == 0U, "101 ms age expires feedback");
  Mt6826sPwm_Task(&encoder, tick);
  check(encoder.online == 0U && encoder.timeout_count == 1U, "timeout clears online and counts once");
  Mt6826sPwm_Task(&encoder, tick);
  check(encoder.timeout_count == 1U, "repeated offline polling does not recount timeout");

  fresh_frame(UINT32_MAX - 50U);
  tick = 49U;
  check(Mt6826sPwm_IsHealthy(&encoder, tick) != 0U, "100 ms across rollover remains healthy");
  tick = 50U;
  check(Mt6826sPwm_IsHealthy(&encoder, tick) == 0U, "101 ms across rollover expires feedback");
  Mt6826sPwm_GetSnapshot(&encoder, tick, &snapshot);
  check(snapshot.healthy == 0U, "snapshot cannot keep pre-rollover stale feedback healthy");
  Mt6826sPwm_Task(&encoder, tick);
  check(encoder.online == 0U, "maintenance clears expired pre-rollover feedback");
  fresh_frame(0U);
  Mt6826sPwm_GetSnapshot(&encoder, UINT32_MAX, &snapshot);
  check(snapshot.healthy != 0U, "valid frame timestamp zero is not a missing-sample sentinel");

  fresh_frame(500U);
  irq_mask = 1U;
  Mt6826sPwm_Task(&encoder, tick);
  Mt6826sPwm_IsHealthy(&encoder, tick);
  Mt6826sPwm_GetSnapshot(&encoder, tick, &snapshot);
  check(irq_mask == 1U, "all feedback APIs preserve an already masked IRQ state");
  irq_mask = 0U;
  check(sampled_without_mask == 0U, "feedback APIs sample tick with the state atomically");
  printf("RESULT: %u failure(s)\n", failures);
  return failures ? 1 : 0;
}
