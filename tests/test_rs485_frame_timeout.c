/* Host-only test: execute the real RS485 receive callback and frame timer.
 * UART/IRQ primitives are mocked, not the frame-boundary implementation.
 * Default: regression assertions (must pass after the defect is repaired).
 * --expect-stale-tick-defect: prove the original stale-now failure explicitly.
 * This test and its stubs must not be added to the MDK firmware project. */
#include <stdio.h>
#include <string.h>
#include "../bsp/communication/bsp_rs485.c"

static uint32_t tick;
static uint32_t primask;
static unsigned failures;
static UART_HandleTypeDef uart;
static BSP_RS485_Bus_t bus;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *data, uint16_t len)
{ (void)handle; (void)data; (void)len; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, uint8_t *data,
                                  uint16_t len, uint32_t timeout_ms)
{ (void)handle; (void)data; (void)len; (void)timeout_ms; return HAL_OK; }
uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *config)
{ (void)config; return 1U; }

static void check(int result, const char *name)
{
  printf("%s: %s\n", result ? "PASS" : "FAIL", name);
  if (!result) failures++;
}

static void reset_bus(uint32_t now)
{
  tick = now;
  primask = 0U;
  BSP_RS485_Init(&bus, &uart, 0, 0U);
}

static void receive_byte(uint8_t value, uint32_t now)
{
  tick = now;
  bus.rx_byte = value;
  BSP_RS485_UartRxCpltCallback(&uart);
}

static void stale_now_case(uint32_t sampled_now, uint32_t received_at, int expect_defect)
{
  uint8_t output[8];
  uint16_t len;
  reset_bus(sampled_now);
  /* RobotTask samples now first; a later UART ISR updates last_rx_tick. */
  receive_byte(1U, received_at);
  BSP_RS485_PollFrameTimeout(&bus, sampled_now, 2U);
  printf("  sampled_now=%lu last_rx=%lu unsigned_delta=%lu ready=%u\n",
         (unsigned long)sampled_now, (unsigned long)received_at,
         (unsigned long)(sampled_now - received_at), (unsigned)bus.frame_ready);
  if (expect_defect)
  {
    check(bus.frame_ready == 1U, "original bug prematurely marks a one-byte fragment ready");
    len = BSP_RS485_GetFrame(&bus, output, sizeof(output));
    check(len == 1U && output[0] == 1U && bus.rx_len == 0U,
          "real GetFrame consumes the incomplete fragment");
  }
  else
  {
    check(bus.frame_ready == 0U, "an ISR timestamp newer than caller now is not an idle gap");
    receive_byte(4U, received_at);
    tick = received_at + 1U;
    BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
    check(bus.frame_ready == 0U, "remaining bytes stay buffered before the full idle gap");
    tick = received_at + 2U;
    BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
    len = BSP_RS485_GetFrame(&bus, output, sizeof(output));
    check(len == 2U && output[0] == 1U && output[1] == 4U,
          "complete buffered bytes are returned only after the true idle gap");
  }
}

int main(int argc, char **argv)
{
  int expect_defect = argc == 2 && strcmp(argv[1], "--expect-stale-tick-defect") == 0;
  reset_bus(100U);
  receive_byte(1U, 100U);
  BSP_RS485_PollFrameTimeout(&bus, 100U, 2U);
  check(bus.frame_ready == 0U, "same-tick receive is not ready");
  tick = 101U;
  BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
  check(bus.frame_ready == 0U, "one-ms idle is not ready");
  tick = 102U;
  BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
  check(bus.frame_ready == 1U, "two-ms idle is ready");

  reset_bus(UINT32_MAX - 1U);
  receive_byte(1U, UINT32_MAX - 1U);
  tick = UINT32_MAX;
  BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
  check(bus.frame_ready == 0U, "one-ms idle before tick rollover is not ready");
  tick = 0U;
  BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
  check(bus.frame_ready == 1U, "legitimate two-ms idle across tick rollover is ready");

  reset_bus(300U);
  receive_byte(1U, 300U);
  primask = 1U;
  BSP_RS485_PollFrameTimeout(&bus, tick, 2U);
  check(primask == 1U, "frame polling preserves an already masked IRQ state");
  primask = 0U;

  stale_now_case(1000U, 1001U, expect_defect);
  stale_now_case(UINT32_MAX, 0U, expect_defect);
  printf("RESULT: %u failure(s); mode=%s\n", failures,
         expect_defect ? "original-defect-reproduction" : "regression");
  return failures ? 1 : 0;
}
