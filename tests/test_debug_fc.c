/* Compile real readonly diagnostic parser/formatter with UART IO mocked. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../application/debug/debug_app.c"
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
UART_HandleTypeDef huart7;
static CommandFcDiagnostics s_test_diag;
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
  (void)p; assert(n < sizeof(s_captured));
  memcpy(s_captured,b,n); s_captured[n]='\0'; ++s_writes; return HAL_OK;
}
void CommandApp_GetFcDiagnostics(CommandFcDiagnostics *d) { *d=s_test_diag; }
static void task(void)
{
  RobotCommand command=g_robot_command;
  RobotChassisState chassis=g_robot_chassis;
  DebugApp_Task(s_tick);
  assert(memcmp(&command,&g_robot_command,sizeof(command))==0);
  assert(memcmp(&chassis,&g_robot_chassis,sizeof(chassis))==0);
}
static void inject(const char *command)
{
  while (*command) { s_rx_byte=(uint8_t)*command++; DebugApp_Rx(NULL,&huart7); }
  task();
}
static void complete(void)
{
  size_t n=strlen(s_captured);
  assert(n>=2U && n<sizeof(s_debug_line)-1U);
  assert(s_captured[n-2U]=='\r' && s_captured[n-1U]=='\n');
}
int main(void)
{
  uint32_t count;
  const char *names[]={"GENERIC","OID","MT6826S","FC","OFF"};
  const uint8_t modes[]={DEBUG_APP_OUTPUT_GENERIC,DEBUG_APP_OUTPUT_OID,
    DEBUG_APP_OUTPUT_MT6826S,DEBUG_APP_OUTPUT_FC,DEBUG_APP_OUTPUT_OFF};
  char command[32];
  DebugApp_Init(); assert(s_output_mode==DEBUG_APP_OUTPUT_FC);
  s_tick=200;
  s_test_diag.drive.valid_pulse_count=7;
  s_test_diag.steer.valid_pulse_count=8;
  s_test_diag.drive.last_valid_ms=190;
  s_test_diag.steer.last_valid_ms=180;
  s_test_diag.drive.invalid_pulse_count=1;
  s_test_diag.drive.timeout_event_count=3;
  s_test_diag.drive.last_invalid_us=3000;
  s_test_diag.center_elapsed_ms=100;
  g_robot_command.fault_event_count=2;
  task(); complete();
  assert(strstr(s_captured,"FC t=200 ") && strstr(s_captured,"events=2") &&
    strstr(s_captured,"good=7/8 bad=1/0 gap=3/0 age=10/20 badus=3000/0"));
  inject("DBG SBUS\r\n"); assert(s_output_mode==DEBUG_APP_OUTPUT_FC);
  assert(strstr(s_captured,"DBG ERR") && !strstr(s_captured,"SBUS"));
  for (unsigned i=0;i<5;i++)
  {
    snprintf(command,sizeof(command),"DBG %s\n",names[i]);
    inject(command); assert(s_output_mode==modes[i]);
    assert(strstr(s_captured,"DBG OK") && strstr(s_captured,"readonly=1"));
    complete(); s_tick+=200; task();
  }
  count=s_writes; s_tick+=1000; task(); assert(s_writes==count);
  inject("DBG?\n"); assert(strstr(s_captured,"GENERIC|OID|MT6826S|FC|OFF"));
  inject("MOTOR 1000\n"); assert(strstr(s_captured,"DBG ERR") && s_output_mode==DEBUG_APP_OUTPUT_OFF);
  inject("DBG HELP\n"); assert(strstr(s_captured,"DBG OK"));
  /* Overlength and overflow may never execute a suffix as a command. */
  inject("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"); task();
  assert(strstr(s_captured,"DBG ERR") && s_output_mode==DEBUG_APP_OUTPUT_OFF);
  for(unsigned i=0;i<140;i++) { s_rx_byte='X'; DebugApp_Rx(NULL,&huart7); }
  task(); inject("DBG FC\n"); assert(strstr(s_captured,"DBG ERR"));
  assert(s_output_mode==DEBUG_APP_OUTPUT_OFF);
  inject("DBG FC\n"); assert(s_output_mode==DEBUG_APP_OUTPUT_FC);
  memset(&s_test_diag,0xff,sizeof(s_test_diag));
  memset(&g_robot_command,0xff,sizeof(g_robot_command));
  memset(&g_robot_chassis,0xff,sizeof(g_robot_chassis));
  for(unsigned i=0;i<4;i++)
  {
    s_output_mode=modes[i]; s_tick+=200; task(); complete();
  }
  assert(strstr(s_captured,"events=4294967295"));
  count=s_writes; s_ready=0; s_tick+=200; task(); assert(s_writes==count);
  s_ready=1; s_test_diag.drive.valid_pulse_count=s_test_diag.steer.valid_pulse_count=0;
  task(); complete(); assert(strstr(s_captured,"age=4294967295/4294967295"));
  s_rx_overflow=1; s_reply=3; s_output_mode=DEBUG_APP_OUTPUT_OFF;
  DebugApp_Init(); assert(s_output_mode==DEBUG_APP_OUTPUT_FC && s_rx_overflow==0 && s_reply==0);
  puts("PASS: FC-only diagnostic fields, readonly five-mode parser, retired/invalid commands, overflow, worst width, TX busy, init");
  return 0;
}
