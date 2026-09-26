/* Compile the real CubeMX USER CODE interrupt path, not a copied handler. */
#include <assert.h>
#include <stdio.h>
#include "../Core/Src/stm32f4xx_it.c"

DMA_HandleTypeDef hdma_tim1_ch1, hdma_tim1_ch4_trig_com, hdma_usart1_rx;
TIM_HandleTypeDef htim2, htim3, htim4, htim5, htim8, htim6;
UART_HandleTypeDef huart7, huart1, huart3;
static unsigned dispatch_count, hal_count;
void BspCallback_DispatchUartIdle(UART_HandleTypeDef *h)
{
  assert(h == &huart1);
  assert((h->flags & UART_FLAG_IDLE) == 0U); /* Must clear before parsing DMA. */
  dispatch_count++;
}
void HAL_UART_IRQHandler(UART_HandleTypeDef *h) { (void)h; hal_count++; }
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *h) { (void)h; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; }
int main(void)
{
  huart1.flags = UART_FLAG_IDLE;
  USART1_IRQHandler(); /* Disabled IDLE must not dispatch or clear it. */
  assert(dispatch_count == 0U && huart1.flags == UART_FLAG_IDLE);
  huart1.enabled = UART_IT_IDLE;
  USART1_IRQHandler();
  assert(dispatch_count == 1U && huart1.flags == 0U);
  USART1_IRQHandler(); /* No new IDLE: no repeated callback. */
  assert(dispatch_count == 1U);
  huart1.flags = UART_FLAG_IDLE;
  USART1_IRQHandler();
  assert(dispatch_count == 2U && huart1.flags == 0U && hal_count == 4U);
  puts("PASS: actual USART1 IRQ gates and clears IDLE before DMA dispatch");
  return 0;
}
