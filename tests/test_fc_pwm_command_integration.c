/* 真实PWM边沿捕获+真实FC命令模块，验证断流后先恢复4帧不能继承旧授权。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../modules/input/pwm_input.c"
#include "../application/command/command_app.c"

TIM_HandleTypeDef htim3;
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
static uint32_t tick, primask, capture;
static BspTimCallbackConfig callbacks[2];
static unsigned callback_count;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return capture; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }
void test_set_polarity(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t polarity)
{ (void)timer; (void)channel; (void)polarity; }
uint8_t BspCallback_RegisterTim(const BspTimCallbackConfig *c)
{ assert(callback_count < 2U); callbacks[callback_count++] = *c; return 1U; }
static void edge_fc(uint32_t channel, uint16_t value, uint32_t time)
{
  unsigned i;
  htim3.Channel = PwmInput_GetActiveChannel(channel);
  capture = value; tick = time;
  for (i = 0U; i < callback_count; i++)
    callbacks[i].input_capture(callbacks[i].parent, &htim3);
}
static void pulse_pair(uint32_t time, uint16_t drive, uint16_t steer, uint8_t poll)
{
  edge_fc(TIM_CHANNEL_3, 10000U, time - 2U);
  edge_fc(TIM_CHANNEL_4, 10000U, time - 2U);
  edge_fc(TIM_CHANNEL_3, (uint16_t)(10000U + drive), time);
  edge_fc(TIM_CHANNEL_4, (uint16_t)(10000U + steer), time);
  if (poll) CommandApp_Task(time);
}
int main(void)
{
  uint32_t t;
  CommandFcDiagnostics d;
  CommandApp_Init();
  for (t = 20U; t <= 280U; t += 20U)
  {
    pulse_pair(t, 1502U, 1500U, 1U);
    assert(g_robot_command.released == (t == 280U));
  }
  pulse_pair(300U, 1770U, 1500U, 1U);
  assert(g_robot_command.throttle_permille == 1000 && g_robot_command.released);
  for (t = 420U; t <= 480U; t += 20U)
    pulse_pair(t, 1770U, 1500U, 0U);
  CommandApp_Task(480U);
  CommandApp_GetFcDiagnostics(&d);
  assert(d.drive.online && d.steer.online);
  assert(d.drive.timeout_event_count == 1U && d.steer.timeout_event_count == 1U);
  assert(!g_robot_command.released && !g_robot_command.source_online && !g_robot_command.throttle_permille);
  pulse_pair(500U, 1770U, 1500U, 1U);
  assert(g_robot_command.source_online && !g_robot_command.released && !g_robot_command.throttle_permille);
  for (t = 520U; t <= 720U; t += 20U)
    pulse_pair(t, 1502U, 1500U, 1U);
  assert(g_robot_command.released);
  /* 非法脉冲也先由真实驱动恢复4帧，再让命令任务观察累计事件。 */
  pulse_pair(740U, 799U, 1500U, 0U);
  for (t = 760U; t <= 820U; t += 20U)
    pulse_pair(t, 1770U, 1500U, 0U);
  CommandApp_Task(820U);
  CommandApp_GetFcDiagnostics(&d);
  assert(d.drive.online && d.drive.invalid_pulse_count == 1U);
  assert(!g_robot_command.released && !g_robot_command.throttle_permille);
  assert(g_robot_command.fault_event_count == 2U);
  puts("PASS: real PWM + FC command, four-frame warmup, initial center, latent timeout and invalid recovery cannot reuse nonzero authorization");
  return 0;
}
