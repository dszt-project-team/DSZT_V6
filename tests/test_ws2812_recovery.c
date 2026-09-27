/* Real strip driver: DMA abort acknowledgement is deliberately asynchronous. */
#include <stdio.h>
#include <string.h>
#include "../modules/lighting/ws2812_strip.c"

static uint32_t tick, irq_mask;
static unsigned failures, starts[4], stops[4];
static uint8_t reject_abort, fail_start_active, fail_start_idle, channel_busy[4];
static TIM_HandleTypeDef timer;
static DMA_HandleTypeDef dma[4];
static DMA_Stream_TypeDef stream[4];
static WS2812Strip_Handle_t left, right;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
uint8_t BspCallback_RegisterTim(const BspTimCallbackConfig *config) { (void)config; return 1U; }
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t channel,
                                       uint32_t *data, uint16_t length)
{
  unsigned i = channel / 4U;
  if (channel_busy[i]) return HAL_BUSY;
  (void)data; (void)length; starts[i]++;
  channel_busy[i] = 1U;
  if (fail_start_idle) return HAL_ERROR;
  dma[i].State = HAL_DMA_STATE_BUSY; stream[i].CR |= DMA_SxCR_EN;
  htim->dma_requests |= (1U << i);
  return fail_start_active ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t channel)
{
  unsigned i = channel / 4U;
  stops[i]++; htim->dma_requests &= ~(1U << i); channel_busy[i] = 0U;
  if (!reject_abort && dma[i].State == HAL_DMA_STATE_BUSY) dma[i].State = HAL_DMA_STATE_ABORT;
  /* Models HAL Stop_DMA returning OK before hardware actually stops, including rejected abort. */
  return HAL_OK;
}
static void check(int pass, const char *name)
{ printf("%s: %s\n", pass ? "PASS" : "FAIL", name); if (!pass) failures++; }
static void reset(uint32_t now)
{
  unsigned i;
  tick = now; irq_mask = 0U; reject_abort = fail_start_active = fail_start_idle = 0U;
  memset(&timer, 0, sizeof(timer)); memset(dma, 0, sizeof(dma)); memset(stream, 0, sizeof(stream));
  memset(starts, 0, sizeof(starts)); memset(stops, 0, sizeof(stops));
  memset(channel_busy, 0, sizeof(channel_busy));
  for (i = 0; i < 4U; i++)
  { dma[i].Instance = &stream[i]; dma[i].State = HAL_DMA_STATE_READY; timer.hdma[i + 1U] = &dma[i]; }
  (void)WS2812Strip_Init(&left, &timer, TIM_CHANNEL_1, 21U, 0U, 70U, 140U);
  (void)WS2812Strip_Init(&right, &timer, TIM_CHANNEL_4, 21U, 0U, 70U, 140U);
}
static void complete(WS2812Strip_Handle_t *strip)
{
  unsigned i = strip->channel / 4U;
  dma[i].State = HAL_DMA_STATE_READY; stream[i].CR = 0U;
  channel_busy[i] = 0U;
  timer.Channel = strip->active_channel;
  WS2812Strip_PwmFinishedDispatch(strip, &timer);
}

int main(void)
{
  uint16_t old_buffer[WS2812_STRIP_DMA_WORD_COUNT];
  reset(0U);
  WS2812Strip_FillRgb(&left, 1U, 2U, 3U);
  check(WS2812Strip_Refresh(&left) == HAL_OK && left.dma_busy == 1U, "normal DMA launch marks busy");
  memcpy(old_buffer, left.dma_buffer, sizeof(old_buffer));
  WS2812Strip_FillRgb(&left, 30U, 31U, 32U);
  tick = 9U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY && stops[0] == 0U &&
        memcmp(old_buffer, left.dma_buffer, sizeof(old_buffer)) == 0,
        "active DMA buffer remains immutable before timeout");
  tick = 10U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY && left.dma_recovering == 1U &&
        left.dma_busy == 1U && stops[0] == 1U && dma[0].State == HAL_DMA_STATE_ABORT &&
        memcmp(old_buffer, left.dma_buffer, sizeof(old_buffer)) == 0,
        "timeout starts asynchronous abort but cannot release or rewrite buffer");
  timer.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
  WS2812Strip_PwmFinishedDispatch(&left, &timer);
  check(left.dma_busy == 1U && left.done_count == 0U,
        "late completion cannot clear busy while recovering");
  check(WS2812Strip_Refresh(&right) == HAL_OK && (timer.dma_requests & TIM_DMA_CC4) != 0U,
        "other TIM1 channel can launch during left recovery");
  tick = 29U;
  WS2812Strip_Task(&left, tick);
  check(stops[0] == 1U, "abort retry is not repeated before 20 ms interval");
  tick = 30U;
  WS2812Strip_Task(&left, tick);
  check(stops[0] == 2U && (timer.dma_requests & TIM_DMA_CC4) != 0U && stops[3] == 0U,
        "bounded retry touches only its own DMA request and channel");
  dma[0].State = HAL_DMA_STATE_READY; stream[0].CR = DMA_SxCR_EN;
  tick = 31U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY && left.dma_busy == 1U,
        "READY state alone cannot release buffer while stream EN is set");
  dma[0].State = HAL_DMA_STATE_ABORT; stream[0].CR = 0U;
  tick = 32U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY,
        "EN clear alone cannot release buffer while abort callback is pending");
  dma[0].State = HAL_DMA_STATE_READY;
  timer.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
  WS2812Strip_PwmFinishedDispatch(&left, &timer);
  check(left.dma_busy == 1U, "even quiescent late callback leaves recovery ownership to task");
  tick = 33U;
  check(WS2812Strip_Refresh(&left) == HAL_OK && starts[0] == 2U && left.dma_recovering == 0U &&
        memcmp(old_buffer, left.dma_buffer, sizeof(old_buffer)) != 0,
        "READY plus EN clear lets task finish abort before next encoding");
  timer.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
  WS2812Strip_PwmFinishedDispatch(&left, &timer);
  check(left.dma_busy == 1U && (timer.dma_requests & TIM_DMA_CC1) != 0U,
        "stale callback during new active frame cannot shut its request off");
  complete(&left);
  check(left.dma_busy == 0U && left.done_count == 1U && timer.compare[0] == 0U,
        "normal completed frame safely clears busy without abort");
  complete(&right);
  check(right.dma_busy == 0U && right.done_count == 1U, "other channel completes normally");

  reset(0U); reject_abort = 1U;
  (void)WS2812Strip_Refresh(&left); memcpy(old_buffer, left.dma_buffer, sizeof(old_buffer));
  tick = 10U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY && dma[0].State == HAL_DMA_STATE_BUSY &&
        stream[0].CR == DMA_SxCR_EN && left.dma_busy == 1U,
        "HAL Stop OK with rejected abort does not imply hardware stopped");
  tick = 30U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY && stops[0] == 2U &&
        memcmp(old_buffer, left.dma_buffer, sizeof(old_buffer)) == 0,
        "rejected abort retries without touching in-flight buffer");
  dma[0].State = HAL_DMA_STATE_READY; stream[0].CR = 0U;
  tick = 31U;
  check(WS2812Strip_Refresh(&left) == HAL_OK, "eventual genuine quiescence recovers rejected abort");

  reset(UINT32_MAX - 5U);
  (void)WS2812Strip_Refresh(&left);
  tick = 4U; WS2812Strip_Task(&left, tick);
  check(left.dma_recovering == 1U && stops[0] == 1U, "DMA timeout handles tick rollover");
  tick = 24U; WS2812Strip_Task(&left, tick);
  check(stops[0] == 2U, "asynchronous abort retry handles tick rollover");

  reset(0U); fail_start_active = 1U;
  check(WS2812Strip_Refresh(&left) == HAL_ERROR && left.dma_busy == 1U,
        "failed start cannot clear busy if hardware nevertheless became active");
  memcpy(old_buffer, left.dma_buffer, sizeof(old_buffer));
  tick = 1U;
  check(WS2812Strip_Refresh(&left) == HAL_BUSY &&
        memcmp(old_buffer, left.dma_buffer, sizeof(old_buffer)) == 0,
        "failed-start active buffer remains protected during asynchronous recovery");
  reset(0U); fail_start_idle = 1U;
  check(WS2812Strip_Refresh(&left) == HAL_ERROR && left.dma_recovering == 1U &&
        left.dma_busy == 1U && stops[0] == 1U && channel_busy[0] == 0U,
        "failed start with idle DMA still resets HAL TIM channel BUSY state");
  fail_start_idle = 0U; tick = 1U;
  check(WS2812Strip_Refresh(&left) == HAL_OK && starts[0] == 2U,
        "idle-DMA start failure can launch again after channel and DMA recovery");
  reset(0U);
  timer.hdma[TIM_DMA_ID_CC1] = 0;
  check(WS2812Strip_Init(&left, &timer, TIM_CHANNEL_1, 21U, 0U, 70U, 140U) == HAL_ERROR &&
        left.htim == 0, "missing channel DMA handle fails initialization safely");
  printf("RESULT: %u failure(s)\n", failures);
  return failures ? 1 : 0;
}
