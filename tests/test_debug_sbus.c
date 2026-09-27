#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../application/debug/debug_app.c"

RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
UART_HandleTypeDef huart7;
static CommandRcDiagnostics s_test_diag;
static char s_captured[1024];
static uint8_t s_ready = 1U;
static uint32_t s_tick, s_writes;

uint32_t HAL_GetTick(void) { return s_tick; }
uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __enable_irq(void) {}
void Error_Handler(void) { assert(0); }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ (void)u; (void)b; (void)n; return HAL_OK; }
uint8_t BspCallback_RegisterUart(const BspUartCallbackConfig *c) { (void)c; return 1U; }
HAL_StatusTypeDef BspUart_Init(BspUartPort *p, UART_HandleTypeDef *u, uint32_t t)
{ (void)p; (void)u; (void)t; return HAL_OK; }
uint8_t BspUart_IsTxReady(const BspUartPort *p) { (void)p; return s_ready; }
HAL_StatusTypeDef BspUart_WriteAsync(BspUartPort *p, const uint8_t *b, uint16_t n)
{
  (void)p;
  assert(n < sizeof(s_captured));
  memcpy(s_captured, b, n);
  s_captured[n] = '\0';
  ++s_writes;
  return HAL_OK;
}
void CommandApp_GetRcDiagnostics(CommandRcDiagnostics *d) { *d = s_test_diag; }

static void CheckCompleteLine(void)
{
  size_t n = strlen(s_captured);
  assert(n >= 2U && n < sizeof(s_debug_line) - 1U);
  assert(s_captured[n - 2U] == '\r' && s_captured[n - 1U] == '\n');
}

int main(void)
{
  uint32_t count;
  DebugApp_Init();
  s_output_mode = DEBUG_APP_OUTPUT_SBUS;
  s_tick = 100U;
  s_test_diag.sbus.raw_flags = 4U;
  s_test_diag.sbus.frame_lost = 1U;
  s_test_diag.sbus.good_frame_count = 7U;
  s_test_diag.sbus.last_good_ms = 90U;
  s_test_diag.sbus.lost_count = 1U;
  s_test_diag.sbus.lost_streak = 1U;
  DebugApp_Task(s_tick);
  assert(strstr(s_captured, "flags=04 fl=1 rf=0 reason=0/0") != NULL);
  assert(strstr(s_captured, "lost=1 streak=1") != NULL);
  assert(strstr(s_captured, "good=7 goodage=10") != NULL);
  CheckCompleteLine();

  /* 全部整数取最大位宽，诊断尾部仍不能被截断。 */
  memset(&s_test_diag, 0xff, sizeof(s_test_diag));
  memset(&g_robot_command, 0xff, sizeof(g_robot_command));
  s_tick = 0xfffffffeU;
  s_test_diag.sbus.last_good_ms = 0U;
  DebugApp_Task(s_tick);
  assert(strstr(s_captured, "lossms=4294967295/4294967295\r\n") != NULL);
  CheckCompleteLine();
  count = s_writes;
  s_ready = 0U;
  s_tick += 100U;
  DebugApp_Task(s_tick);
  assert(s_writes == count);
  s_ready = 1U;
  s_test_diag.sbus.good_frame_count = 0U;
  DebugApp_Task(s_tick);
  assert(strstr(s_captured, "goodage=4294967295") != NULL);
  CheckCompleteLine();
  puts("PASS: SBUS diagnostic reasons, counters, bounded formatting and TX busy.");
  return 0;
}
