/* Real SBUS parser, loss qualification and DMA recovery; only HAL is mocked. */
#include <stdio.h>
#include <string.h>
#include "../modules/remote/sbus_rc.c"

static uint32_t tick, irq_mask;
static unsigned failures, sampled_without_mask, starts, aborts;
static HAL_StatusTypeDef start_result, abort_result;
static SBusRc_Handle_t receiver;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart;

uint32_t HAL_GetTick(void)
{ if (irq_mask == 0U) sampled_without_mask++; return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
  (void)data; (void)length; starts++;
  if (start_result == HAL_OK) handle->ErrorCode = HAL_UART_ERROR_NONE;
  return start_result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{ (void)handle; aborts++; return abort_result; }
uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *config)
{ (void)config; return 1U; }

static void check(int pass, const char *name)
{ printf("%s: %s\n", pass ? "PASS" : "FAIL", name); if (!pass) failures++; }

static void reset(uint32_t now)
{
  tick = now; irq_mask = 0U; starts = aborts = 0U;
  start_result = abort_result = HAL_OK;
  memset(&uart, 0, sizeof(uart));
  memset(&dma, 0, sizeof(dma));
  uart.hdmarx = &dma;
  dma.remaining = SBUS_RC_DMA_BUFFER_SIZE;
  SBusRc_Init(&receiver, &uart);
  sampled_without_mask = 0U;
}

static void receive_frame(uint32_t now, uint8_t flags, uint16_t channel0)
{
  memset(receiver.frame, 0, sizeof(receiver.frame));
  receiver.frame[0] = 0x0FU;
  receiver.frame[1] = (uint8_t)channel0;
  receiver.frame[2] = (uint8_t)(channel0 >> 8);
  receiver.frame[23] = flags;
  tick = now;
  SBusRc_ParseFrame(&receiver, now);
}

static void task(uint32_t now)
{ tick = now; SBusRc_Task(&receiver, now); }

static void test_time(void)
{
  reset(1000U);
  SBusRc_Task(&receiver, 0U);
  check(receiver.data.online == 0U, "no frame never becomes online");
  receive_frame(1001U, 0U, 1000U);
  SBusRc_Task(&receiver, 1000U);
  check(receiver.data.online == 1U, "new ISR frame cannot be expired by stale task tick");
  task(1301U);
  check(receiver.data.online == 1U, "300 ms boundary remains online");
  task(1302U);
  check(receiver.data.online == 0U && receiver.data.timeout_count == 1U,
        "301 ms silence expires and records one timeout");
  task(1310U);
  check(receiver.data.timeout_count == 1U && receiver.data.guard_event_count == 1U,
        "continued timeout does not repeatedly increment event counters");
  reset(UINT32_MAX - 50U);
  receive_frame(UINT32_MAX - 50U, 0U, 1000U);
  task(249U);
  check(receiver.data.online == 1U, "300 ms boundary survives tick rollover");
  task(250U);
  check(receiver.data.online == 0U, "301 ms silence across rollover expires");
  reset(0U);
  receive_frame(0U, 0U, 1000U);
  SBusRc_Task(&receiver, UINT32_MAX);
  check(receiver.data.online == 1U, "frame received at tick zero is valid");
  receive_frame(100U, 0x08U, 1800U);
  SBusRc_Task(&receiver, tick);
  check(receiver.data.online == 0U && receiver.data.failsafe != 0U,
        "time maintenance never revives a failsafe frame");
  receive_frame(200U, 0U, 1000U);
  irq_mask = 1U;
  SBusRc_Task(&receiver, tick);
  check(irq_mask == 1U, "already masked IRQ state is preserved");
  irq_mask = 0U;
  check(sampled_without_mask == 0U, "maintenance samples HAL tick only while IRQ masked");
}

static void test_loss(void)
{
  uint32_t event;
  reset(0U);
  receive_frame(0U, 0U, 1000U);
  receive_frame(14U, 0x04U, 1800U);
  check(receiver.data.online == 1U && receiver.data.pulse_us[0] == 1500U &&
        receiver.data.good_frame_count == 1U && receiver.data.lost_count == 1U,
        "one lost frame holds only last good channels without advancing good count");
  receive_frame(28U, 0x04U, 1800U);
  check(receiver.data.online == 1U && receiver.data.lost_streak == 2U,
        "two lost frames remain inside bounded tolerance");
  receive_frame(42U, 0x04U, 1800U);
  check(receiver.data.online == 0U && receiver.data.guard_reason == SBUS_RC_REASON_FRAME_LOST &&
        receiver.data.guard_event_count == 1U,
        "third consecutive lost frame trips immediately");
  receive_frame(56U, 0x04U, 1800U);
  check(receiver.data.guard_event_count == 1U, "continued frame loss retains one event");
  receive_frame(70U, 0U, 1800U);
  check(receiver.data.online == 1U && receiver.data.guard_reason == 0U &&
        receiver.data.lost_streak == 0U && receiver.data.last_guard_reason == SBUS_RC_REASON_FRAME_LOST &&
        receiver.data.pulse_us[0] == 2000U,
        "healthy frame resets current loss but retains diagnostic event");
  receive_frame(84U, 0x04U, 200U);
  receive_frame(98U, 0U, 1000U);
  receive_frame(112U, 0x04U, 200U);
  check(receiver.data.lost_streak == 1U && receiver.data.online == 1U,
        "healthy frame resets consecutive-loss qualification");
  task(147U);
  check(receiver.data.online == 1U, "49 ms good-frame age keeps bounded hold");
  task(148U);
  check(receiver.data.online == 0U && receiver.data.guard_event_count == 2U,
        "50 ms good-frame age trips even if lost-frame stream then goes silent");
  reset(UINT32_MAX - 20U);
  receive_frame(UINT32_MAX - 20U, 0U, 1000U);
  receive_frame(UINT32_MAX - 6U, 0x04U, 1800U);
  task(28U);
  check(receiver.data.online == 1U, "49 ms dropped hold survives tick rollover");
  task(29U);
  check(receiver.data.online == 0U, "50 ms dropped hold trips across tick rollover");
  reset(0U);
  receive_frame(0U, 0x04U, 1800U);
  check(receiver.data.online == 0U && receiver.data.good_frame_count == 0U &&
        receiver.data.pulse_us[0] == 1500U,
        "startup dropped frame cannot invent a healthy motion baseline");
  receive_frame(14U, 0U, 1000U);
  event = receiver.data.guard_event_count;
  receive_frame(28U, 0x08U, 1800U);
  check(receiver.data.online == 0U && receiver.data.pulse_us[0] == 1500U,
        "standard failsafe immediately trips without accepting channels");
  receive_frame(42U, 0U, 1000U);
  check(receiver.data.online == 1U && receiver.data.guard_event_count == event + 1U &&
        receiver.data.last_guard_reason == SBUS_RC_REASON_FAILSAFE &&
        receiver.data.last_guard_ms == 28U,
        "failsafe then recovery before task still leaves observable event");
  receive_frame(56U, 0x10U, 1800U);
  check(receiver.data.online == 0U && receiver.data.failsafe_count == 2U &&
        receiver.data.raw_flags == 0x10U, "MC7 failsafe is immediate and independently counted");
  reset(0U);
  receive_frame(0U, 0U, 1000U);
  receive_frame(14U, 0x04U, 1800U);
  receive_frame(50U, 0U, 1000U);
  check(receiver.data.online == 1U && receiver.data.guard_event_count == 1U &&
        receiver.data.last_guard_reason == SBUS_RC_REASON_FRAME_LOST,
        "late good frame records expired dropped hold before clearing it");
  receive_frame(351U, 0U, 1000U);
  check(receiver.data.online == 1U && receiver.data.timeout_count == 1U &&
        receiver.data.last_guard_reason == SBUS_RC_REASON_TIMEOUT,
        "late good frame records unpolled 301 ms timeout before recovery");
  event = receiver.data.frame_count;
  receiver.frame[24] = 0x55U;
  SBusRc_ParseFrame(&receiver, 365U);
  check(receiver.data.frame_count == event && receiver.data.error_count == 1U,
        "invalid footer cannot refresh valid or good-frame clocks");
}

static void test_recovery(void)
{
  uint32_t event;
  reset(0U);
  receive_frame(0U, 0U, 1000U);
  tick = 10U;
  SBusRc_UartErrorCallback(&receiver, &uart);
  check(receiver.data.online == 0U && receiver.data.uart_error_count == 1U &&
        receiver.data.guard_reason == SBUS_RC_REASON_UART && starts == 1U && aborts == 0U,
        "UART error immediately guards but never blocks on abort in ISR");
  memcpy(receiver.dma_buffer, receiver.frame, SBUS_RC_FRAME_SIZE);
  dma.remaining = SBUS_RC_DMA_BUFFER_SIZE - SBUS_RC_FRAME_SIZE;
  SBusRc_UartRxCpltCallback(&receiver, &uart, 11U);
  check(receiver.data.frame_count == 1U, "pending recovery rejects old DMA callbacks");
  task(29U);
  check(aborts == 0U, "receive recovery respects retry spacing");
  abort_result = HAL_TIMEOUT;
  task(30U);
  check(aborts == 1U && starts == 1U && receiver.data.rx_start_error_count == 1U &&
        receiver.data.online == 0U,
        "failed abort remains guarded and never starts a second active DMA");
  event = receiver.data.guard_event_count;
  abort_result = HAL_OK; start_result = HAL_BUSY;
  task(50U);
  check(starts == 2U && receiver.rx_restart_pending != 0U && receiver.rx_active == 0U &&
        receiver.data.rx_start_error_count == 2U && receiver.data.guard_event_count == event,
        "failed start retries without repeated same-cause guard events");
  start_result = HAL_OK;
  irq_mask = 1U;
  task(70U);
  check(starts == 2U && irq_mask == 1U, "masked task defers potentially blocking abort");
  irq_mask = 0U;
  task(70U);
  check(starts == 3U && receiver.rx_active != 0U && receiver.rx_restart_pending == 0U &&
        receiver.data.online == 0U, "successful DMA restart still waits for a new good frame");
  receive_frame(84U, 0U, 1000U);
  check(receiver.data.online == 1U && receiver.data.guard_reason == 0U,
        "new healthy frame recovers UART guard");
  event = receiver.data.frame_count;
  memcpy(receiver.dma_buffer, receiver.frame, SBUS_RC_FRAME_SIZE);
  dma.remaining = SBUS_RC_DMA_BUFFER_SIZE - SBUS_RC_FRAME_SIZE;
  uart.ErrorCode = 1U;
  SBusRc_UartIdleCallback(&receiver, &uart, 98U);
  SBusRc_UartRxCpltCallback(&receiver, &uart, 98U);
  check(receiver.data.frame_count == event,
        "HAL error code blocks DMA parsing even before asynchronous error callback");
  reset(0U);
  receiver.rx_active = 0U;
  start_result = HAL_ERROR;
  SBusRc_StartReceive(&receiver);
  check(receiver.data.online == 0U && receiver.data.guard_reason == SBUS_RC_REASON_RX_START &&
        receiver.rx_restart_pending != 0U, "initial receive failure is visible and retryable");
}

int main(void)
{
  test_time();
  test_loss();
  test_recovery();
  printf("RESULT: %u failure(s)\n", failures);
  return failures ? 1 : 0;
}
