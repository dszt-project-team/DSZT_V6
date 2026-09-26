/* Host regression: real SBUS parser/task, hardware primitives only are mocked. */
#include <stdio.h>
#include <string.h>
#include "../modules/remote/sbus_rc.c"

static uint32_t tick, irq_mask;
static unsigned failures;
static SBusRc_Handle_t receiver;
static UART_HandleTypeDef uart;
static unsigned sampled_without_mask;

uint32_t HAL_GetTick(void)
{ if (irq_mask == 0U) sampled_without_mask++; return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{ (void)handle; (void)data; (void)length; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{ (void)handle; return HAL_OK; }
uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *config)
{ (void)config; return 1U; }

static void check(int pass, const char *name)
{ printf("%s: %s\n", pass ? "PASS" : "FAIL", name); if (!pass) failures++; }

static void receive_frame(uint32_t now, uint8_t flags)
{
  memset(receiver.frame, 0, sizeof(receiver.frame));
  receiver.frame[0] = 0x0FU;
  receiver.frame[23] = flags;
  tick = now;
  SBusRc_ParseFrame(&receiver, now);
}

int main(void)
{
  SBusRc_Init(&receiver, &uart);
  SBusRc_Task(&receiver, 0U);
  check(receiver.data.online == 0U, "no frame never becomes online");
  receive_frame(1001U, 0U);
  SBusRc_Task(&receiver, 1000U);
  check(receiver.data.online == 1U, "new ISR frame cannot be expired by stale task tick");
  tick = 1301U;
  SBusRc_Task(&receiver, tick);
  check(receiver.data.online == 1U, "300 ms boundary remains online");
  tick = 1302U;
  SBusRc_Task(&receiver, tick);
  check(receiver.data.online == 0U, "301 ms silence expires normally");
  receive_frame(UINT32_MAX - 50U, 0U);
  tick = 249U;
  SBusRc_Task(&receiver, tick);
  check(receiver.data.online == 1U, "300 ms boundary survives tick rollover");
  tick = 250U;
  SBusRc_Task(&receiver, tick);
  check(receiver.data.online == 0U, "301 ms silence across rollover expires");
  receive_frame(0U, 0U);
  SBusRc_Task(&receiver, UINT32_MAX);
  check(receiver.data.online == 1U, "frame received at tick zero is valid");
  receive_frame(100U, 0x08U);
  SBusRc_Task(&receiver, tick);
  check(receiver.data.online == 0U && receiver.data.failsafe != 0U,
        "time maintenance never revives a failsafe frame");
  receive_frame(200U, 0U);
  irq_mask = 1U;
  SBusRc_Task(&receiver, tick);
  check(irq_mask == 1U, "already masked IRQ state is preserved");
  irq_mask = 0U;
  check(sampled_without_mask == 0U, "maintenance samples HAL tick only while IRQ masked");
  printf("RESULT: %u failure(s)\n", failures);
  return failures ? 1 : 0;
}
