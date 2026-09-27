/* 直接编译实际 CubeMX USER CODE 中断路径，不复制一份待测处理函数。 */
#include <assert.h>
#include <stdio.h>
#include "../Core/Src/stm32f4xx_it.c"

DMA_HandleTypeDef hdma_tim1_ch1, hdma_tim1_ch4_trig_com, hdma_usart1_rx;
TIM_HandleTypeDef htim2, htim3, htim4, htim5, htim8, htim6;
UART_HandleTypeDef huart7, huart1, huart3;
static USART_TypeDef usart1_regs;
static unsigned dispatch_count, hal_count;
static unsigned error_count, abort_pending, synchronous_restart;
static uint32_t hal_error_flags;

static void CompleteAbortAndRestart(UART_HandleTypeDef *h)
{
  /* 对应错误回调中重新启动 DMA 时清 ORE/IDLE，并重新开启 IDLE 中断。 */
  __HAL_UART_CLEAR_IDLEFLAG(h);
  h->enabled |= UART_IT_IDLE;
  abort_pending = 0U;
}

void BspCallback_DispatchUartIdle(UART_HandleTypeDef *h)
{
  assert(h == &huart1);
  assert((h->Instance->SR & UART_FLAG_IDLE) == 0U);
  assert(abort_pending == 0U);
  dispatch_count++;
}
void HAL_UART_IRQHandler(UART_HandleTypeDef *h)
{
  const uint32_t errors = h->Instance->SR & TEST_UART_ERROR_FLAGS;
  hal_count++;
  if (errors != 0U)
  {
    /* 必须在读 SR/DR 清错误之前进入 HAL，且旧缓冲不得经 IDLE 分发。 */
    assert((h->enabled & UART_IT_IDLE) == 0U);
    hal_error_flags = errors;
    error_count++;
    abort_pending = 1U;
    if (synchronous_restart != 0U)
    {
      CompleteAbortAndRestart(h);
    }
  }
  else if ((h->enabled & UART_IT_IDLE) != 0U)
  {
    /* 正常 IDLE 仍须在 HAL 之前清除。 */
    assert((h->Instance->SR & UART_FLAG_IDLE) == 0U);
  }
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *h) { (void)h; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; }
int main(void)
{
  static const uint32_t errors[] = {USART_SR_PE, USART_SR_FE, USART_SR_NE, USART_SR_ORE,
                                     TEST_UART_ERROR_FLAGS};
  unsigned i;
  unsigned sync;
  huart1.Instance = &usart1_regs;
  usart1_regs.SR = UART_FLAG_IDLE;
  USART1_IRQHandler(); /* 未使能 IDLE 时不得分发或清除它。 */
  assert(dispatch_count == 0U && usart1_regs.SR == UART_FLAG_IDLE);
  huart1.enabled = UART_IT_IDLE;
  USART1_IRQHandler();
  assert(dispatch_count == 1U && usart1_regs.SR == 0U);
  USART1_IRQHandler(); /* 没有新 IDLE 时不应重复回调。 */
  assert(dispatch_count == 1U);
  usart1_regs.SR = UART_FLAG_IDLE;
  USART1_IRQHandler();
  assert(dispatch_count == 2U && usart1_regs.SR == 0U && hal_count == 4U);

  for (sync = 0U; sync < 2U; sync++)
  {
    synchronous_restart = sync;
    for (i = 0U; i < sizeof(errors) / sizeof(errors[0]); i++)
    {
      unsigned previous_dispatch = dispatch_count;
      unsigned previous_hal = hal_count;
      unsigned previous_errors = error_count;
      usart1_regs.SR = UART_FLAG_IDLE | errors[i];
      USART1_IRQHandler();
      assert(dispatch_count == previous_dispatch);
      assert(hal_count == previous_hal + 1U);
      assert(error_count == previous_errors + 1U && hal_error_flags == errors[i]);
      assert(usart1_regs.SR == 0U);
      if (sync == 0U)
      {
        /* 异步 Abort 尚未完成时，即使再有 IDLE，也不能解析旧 DMA 缓冲。 */
        assert(abort_pending == 1U && huart1.enabled == 0U);
        usart1_regs.SR = UART_FLAG_IDLE;
        USART1_IRQHandler();
        assert(dispatch_count == previous_dispatch);
        CompleteAbortAndRestart(&huart1);
      }
      assert(abort_pending == 0U && huart1.enabled == UART_IT_IDLE);
      usart1_regs.SR = UART_FLAG_IDLE;
      USART1_IRQHandler();
      assert(dispatch_count == previous_dispatch + 1U && usart1_regs.SR == 0U);
    }
  }

  /* 无 IDLE 的单独错误同样只经 HAL 恢复，不分发 DMA 数据。 */
  {
    unsigned previous_dispatch = dispatch_count;
    unsigned previous_hal = hal_count;
    usart1_regs.SR = USART_SR_ORE;
    USART1_IRQHandler();
    assert(dispatch_count == previous_dispatch && hal_count == previous_hal + 1U);
    assert(usart1_regs.SR == 0U && huart1.enabled == UART_IT_IDLE);
  }
  puts("PASS: USART1 IRQ preserves HAL errors, suppresses tainted IDLE, and recovers after DMA abort");
  return 0;
}
