/* 联合编译实际 SBUS DMA 解析与命令门控；仅硬件原语使用桩，不接触执行器。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../modules/remote/sbus_rc.c"
#include "../application/command/command_app.c"

RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
UART_HandleTypeDef huart1;
static DMA_HandleTypeDef rx_dma;
static uint32_t test_tick, irq_mask;
static uint16_t write_pos;
static unsigned starts, aborts;
static HAL_StatusTypeDef start_status, abort_status;

uint32_t HAL_GetTick(void) { return test_tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __enable_irq(void) { irq_mask = 0U; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
  (void)data;
  assert(uart == &huart1 && length == SBUS_RC_DMA_BUFFER_SIZE);
  starts++;
  if (start_status == HAL_OK)
  {
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    uart->hdmarx->remaining = length;
    write_pos = 0U;
  }
  return start_status;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
  assert(uart == &huart1 && irq_mask == 0U);
  aborts++;
  return abort_status;
}
uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *config)
{
  assert(config->handle == &huart1 && config->parent == &s_rc);
  return 1U;
}

static void task_at(uint32_t now)
{
  test_tick = now;
  CommandApp_Task(now);
}

static void inject_frame(uint32_t now, uint16_t ch3, uint16_t ch5, uint8_t flags)
{
  uint8_t frame[SBUS_RC_FRAME_SIZE] = {0};
  unsigned channel, bit, i;
  frame[0] = 0x0FU;
  frame[23] = flags;
  for (channel = 0U; channel < 16U; channel++)
  {
    uint16_t pulse = channel == COMMAND_CH_THROTTLE ? ch3 :
                     channel == COMMAND_CH_MODE ? ch5 :
                     channel == COMMAND_CH_SPEED_LIMIT ? 2000U : 1500U;
    uint16_t raw = (uint16_t)(200U + (uint32_t)(pulse - 1000U) * 1600U / 1000U);
    for (bit = 0U; bit < 11U; bit++)
    {
      unsigned index = channel * 11U + bit;
      if ((raw & (1U << bit)) != 0U) frame[1U + index / 8U] |= (uint8_t)(1U << (index % 8U));
    }
  }
  test_tick = now;
  for (i = 0U; i < sizeof(frame); i++)
  {
    s_rc.dma_buffer[write_pos] = frame[i];
    write_pos = (uint16_t)((write_pos + 1U) % SBUS_RC_DMA_BUFFER_SIZE);
  }
  rx_dma.remaining = (uint32_t)(SBUS_RC_DMA_BUFFER_SIZE - write_pos);
  irq_mask = 1U;
  SBusRc_UartIdleCallback(&s_rc, &huart1, now);
  irq_mask = 0U;
}

static void frame_task(uint32_t now, uint16_t ch3, uint16_t ch5, uint8_t flags)
{
  inject_frame(now, ch3, ch5, flags);
  task_at(now);
}

static uint32_t ready_manual(void)
{
  uint32_t now;
  test_tick = 100U;
  irq_mask = 0U;
  starts = aborts = 0U;
  start_status = abort_status = HAL_OK;
  memset(&huart1, 0, sizeof(huart1));
  huart1.hdmarx = &rx_dma;
  CommandApp_Init();
  assert(starts == 1U && s_rc.rx_active != 0U);
  frame_task(100U, 1500U, 1000U, 0U);
  frame_task(114U, 1500U, 1000U, 0U);
  for (now = 128U; now <= 352U; now += 14U) frame_task(now, 1500U, 1500U, 0U);
  assert(g_robot_command.mode == ROBOT_MODE_MANUAL && g_robot_command.gate == ROBOT_GATE_READY);
  return 352U;
}

static void test_bounded_soft_loss(void)
{
  uint32_t now = ready_manual() + 14U;
  uint32_t good_count;
  int16_t throttle;
  frame_task(now, 1750U, 1500U, 0U);
  good_count = s_rc.data.good_frame_count;
  throttle = g_robot_command.throttle_permille;
  assert(throttle > 0);
  frame_task(now + 14U, 1000U, 2000U, 4U);
  frame_task(now + 28U, 1000U, 2000U, 4U);
  assert(s_rc.data.good_frame_count == good_count && s_rc.data.pulse_us[2] == 1750U);
  assert(g_robot_command.mode == ROBOT_MODE_MANUAL && g_robot_command.gate == ROBOT_GATE_READY);
  assert(g_robot_command.throttle_permille == throttle && g_robot_command.failsafe == 0U);
  frame_task(now + 42U, 1000U, 2000U, 4U);
  assert(g_robot_command.gate == ROBOT_GATE_RC_LOST && g_robot_command.throttle_permille == 0);
  assert(s_rc.data.lost_count == 3U && s_rc.data.last_guard_reason == SBUS_RC_REASON_FRAME_LOST);

  now = ready_manual() + 14U;
  frame_task(now, 1750U, 1500U, 0U);
  frame_task(now + 14U, 1000U, 2000U, 4U);
  task_at(now + 49U);
  assert(g_robot_command.gate == ROBOT_GATE_READY);
  task_at(now + 50U);
  assert(g_robot_command.gate == ROBOT_GATE_RC_LOST);
}

static void test_transient_failsafe_and_mode_glitches(void)
{
  uint32_t now = ready_manual();
  uint32_t offset;
  inject_frame(now + 14U, 1000U, 2000U, 0x10U);
  inject_frame(now + 28U, 1500U, 1500U, 0U);
  assert(s_rc.data.online == 1U && s_rc.data.guard_reason == 0U);
  task_at(now + 28U);
  assert(g_robot_command.gate == ROBOT_GATE_RC_LOST && g_robot_command.failsafe != 0U);
  task_at(now + 29U);
  assert(g_robot_command.gate == ROBOT_GATE_MODE_CONFIRMING);
  frame_task(now + 42U, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_MODE_CONFIRMING);
  frame_task(now + 56U, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_THROTTLE_CENTERING);
  for (offset = 70U; offset <= 266U; offset += 14U) frame_task(now + offset, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_READY);
  frame_task(now + 280U, 1500U, 1300U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_READY);
  frame_task(now + 294U, 1500U, 2000U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_MODE_CONFIRMING);
  assert(s_rc_diagnostics.mode_reject_count == 1U);
}

static void test_lost_frame_breaks_mode_confirmation(void)
{
  uint32_t now = ready_manual();
  frame_task(now + 14U, 1500U, 2000U, 0U);
  assert(g_robot_command.mode == ROBOT_MODE_MANUAL);
  inject_frame(now + 28U, 1500U, 2000U, 4U);
  inject_frame(now + 42U, 1500U, 2000U, 0U);
  /* 丢帧已被健康帧覆盖，但累计计数仍应中断 AUTO 的连续确认。 */
  task_at(now + 42U);
  assert(g_robot_command.mode == ROBOT_MODE_LOCKED && g_robot_command.gate == ROBOT_GATE_MODE_CONFIRMING);
  frame_task(now + 56U, 1500U, 2000U, 0U);
  assert(g_robot_command.mode == ROBOT_MODE_AUTO_FC && g_robot_command.gate == ROBOT_GATE_READY);
}

static void test_long_loss_reauthorization(void)
{
  uint32_t now = ready_manual() + 14U;
  uint32_t offset;
  frame_task(now, 1500U, 1500U, 8U);
  for (offset = 14U; offset < 699U; offset += 14U) frame_task(now + offset, 1500U, 1500U, 8U);
  frame_task(now + 699U, 1500U, 1500U, 8U);
  assert(s_rc_diagnostics.revoke_count == 0U);
  frame_task(now + 700U, 1500U, 1500U, 0U);
  assert(s_rc_diagnostics.revoke_count == 1U && s_rc_diagnostics.last_loss_ms == 700U);
  frame_task(now + 714U, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_STARTUP_LOCK_REQUIRED);
  frame_task(now + 728U, 1500U, 1000U, 0U);
  frame_task(now + 742U, 1500U, 1000U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_READY && g_robot_command.mode == ROBOT_MODE_LOCKED);
}

static void test_soft_loss_cannot_release(void)
{
  uint32_t now = ready_manual();
  uint32_t offset;
  frame_task(now + 14U, 1500U, 1000U, 0U);
  frame_task(now + 28U, 1500U, 1000U, 0U);
  frame_task(now + 42U, 1500U, 1500U, 0U);
  frame_task(now + 56U, 1500U, 1500U, 0U);
  for (offset = 70U; offset <= 238U; offset += 14U) frame_task(now + offset, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_THROTTLE_CENTERING);
  /* 回中时间恰满 200 ms 时只有丢帧保持值：不得将未释放状态变成可驱动。 */
  frame_task(now + 252U, 2000U, 2000U, 4U);
  task_at(now + 256U);
  assert(g_robot_command.gate == ROBOT_GATE_THROTTLE_CENTERING && s_release_ready == 0U);
  frame_task(now + 266U, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_READY);
}

static void test_uart_deferred_restart(void)
{
  uint32_t now = ready_manual();
  uint32_t frames = s_rc.data.frame_count;
  unsigned prior_starts = starts;
  huart1.ErrorCode = 1U;
  test_tick = now + 1U;
  irq_mask = 1U;
  SBusRc_UartErrorCallback(&s_rc, &huart1);
  irq_mask = 0U;
  assert(aborts == 0U && starts == prior_starts);
  assert(s_rc.rx_active == 0U && s_rc.rx_restart_pending != 0U);
  inject_frame(now + 2U, 2000U, 2000U, 0U);
  assert(s_rc.data.frame_count == frames);
  task_at(now + 2U);
  assert(g_robot_command.gate == ROBOT_GATE_RC_LOST);
  start_status = HAL_ERROR;
  task_at(now + 21U);
  assert(aborts == 1U && s_rc.data.rx_start_error_count == 1U);
  start_status = HAL_OK;
  task_at(now + 41U);
  assert(aborts == 2U && s_rc.rx_active != 0U && s_rc.rx_restart_pending == 0U);
  frame_task(now + 55U, 1500U, 1500U, 0U);
  frame_task(now + 69U, 1500U, 1500U, 0U);
  assert(g_robot_command.gate == ROBOT_GATE_THROTTLE_CENTERING);
  assert(s_rc.data.uart_error_count == 1U && s_rc.data.rx_start_error_count == 1U);
}

int main(void)
{
  test_bounded_soft_loss();
  test_transient_failsafe_and_mode_glitches();
  test_lost_frame_breaks_mode_confirmation();
  test_long_loss_reauthorization();
  test_soft_loss_cannot_release();
  test_uart_deferred_restart();
  puts("PASS: real SBUS DMA + command integration: bounded hold, hard/transient loss, CH5 glitches, 700 ms reauthorization, deferred UART recovery");
  return 0;
}
