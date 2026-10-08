/* 真实飞控命令状态机；PWM捕获另由test_pwm_input验证，本测试不接硬件。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../application/chassis/chassis_config.h"
#include "../application/command/command_app.c"

TIM_HandleTypeDef htim3;
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
struct PwmInput_Handle { unsigned side; };
static struct PwmInput_Handle handles[2];
static PwmInput_Snapshot_t inputs[2];
static PwmInput_Config_t configs[2];
static unsigned registered;
static int failed_registration = -1;

PwmInput_Handle_t *PwmInput_Register(const PwmInput_Config_t *c)
{
  unsigned side = registered++;
  assert(side < 2U);
  configs[side] = *c;
  handles[side].side = side;
  return ((int)side == failed_registration) ? NULL : &handles[side];
}
uint8_t PwmInput_GetSnapshot(PwmInput_Handle_t *h, PwmInput_Snapshot_t *s, uint32_t now)
{
  (void)now;
  if (h == NULL) return 0U;
  *s = inputs[h->side];
  return 1U;
}
static void reset(void)
{
  registered = 0U;
  memset(inputs, 0, sizeof(inputs));
  memset(&g_robot_chassis, 0, sizeof(g_robot_chassis));
  CommandApp_Init();
}
static void sample(uint32_t now, uint16_t drive, uint16_t steer)
{
  unsigned i;
  for (i = 0U; i < 2U; i++)
  {
    inputs[i].raw_us = inputs[i].filtered_us = i ? steer : drive;
    inputs[i].online = 1U;
    inputs[i].fault = PWM_INPUT_FAULT_NONE;
    inputs[i].last_valid_ms = now;
    inputs[i].valid_pulse_count++;
  }
  CommandApp_Task(now);
}
static void release(uint32_t start)
{
  sample(start, 1502U, 1500U);
  sample(start + 199U, 1502U, 1500U);
  assert(!g_robot_command.released);
  sample(start + 200U, 1502U, 1500U);
  assert(g_robot_command.released && g_robot_command.centered);
  assert(g_robot_command.gate == ROBOT_GATE_READY);
}
int main(void)
{
  CommandFcDiagnostics d;
  uint32_t event_count;
  reset();
  assert(g_robot_command.mode == (CHASSIS_STEER_CALIBRATION_SIDE ? ROBOT_MODE_CALIBRATION : ROBOT_MODE_AUTO_FC));
  assert(!g_robot_command.source_online && !g_robot_command.released);
  assert(g_robot_command.gate == ROBOT_GATE_FC_INPUT_INVALID);
  assert(configs[0].timer == &htim3 && configs[1].timer == &htim3);
  assert(configs[0].channel == TIM_CHANNEL_3 && configs[1].channel == TIM_CHANNEL_4);
  assert(configs[0].average_window == 1U && configs[1].average_window == 8U);
  assert(configs[0].valid_samples_to_online == 4U && configs[1].valid_samples_to_online == 4U);
  assert(configs[0].minimum_valid_us == 800U && configs[0].maximum_valid_us == 2200U);
  assert(configs[0].timeout_ms == 30U && configs[0].transient_fault_hold_ms == 5U);
  sample(10U, 1770U, 2000U); sample(1000U, 1770U, 2000U);
  assert(!g_robot_command.released && !g_robot_command.throttle_permille && !g_robot_command.steering_permille);

  /* MAIN2均值中位而原始已离中不能启动回中计时。 */
  sample(1100U, 1502U, 1500U);
  inputs[1].raw_us = 1516U;
  CommandApp_Task(1300U);
  assert(!g_robot_command.released && !g_robot_command.centered);
  release(1400U);
  inputs[1].raw_us = 2000U;
  inputs[1].filtered_us = 1500U;
  CommandApp_Task(1610U);
  assert(!g_robot_command.centered && g_robot_command.released);
  assert(g_robot_command.steering_permille == (CHASSIS_STEER_CALIBRATION_SIDE ? 1000 : 0));
  sample(1620U, 1770U, 1000U);
  if (CHASSIS_STEER_CALIBRATION_SIDE)
  {
    assert(!g_robot_command.released && !g_robot_command.throttle_permille && !g_robot_command.steering_permille);
    assert(g_robot_command.gate == ROBOT_GATE_CAL_DRIVE_NOT_CENTERED);
    release(1700U);
    sample(1910U, 1502U, 1000U);
    assert(g_robot_command.steering_permille == -1000 && !g_robot_command.throttle_permille);
  }
  else
  {
    assert(g_robot_command.throttle_permille == 1000 && g_robot_command.steering_permille == -1000);
    sample(1630U, 1260U, 2000U);
    assert(g_robot_command.throttle_permille == -1000 && g_robot_command.steering_permille == 1000);
    sample(1640U, 1502U, 1500U);
    assert(!g_robot_command.throttle_permille && !g_robot_command.steering_permille);
  }

  /* 即使快照已恢复健康，累计坏脉冲变化也必须撤销授权一次。 */
  inputs[1].invalid_pulse_count++;
  CommandApp_Task(2000U);
  assert(!g_robot_command.source_online && !g_robot_command.released);
  event_count = g_robot_command.fault_event_count;
  assert(event_count == 1U);
  sample(2010U, 1502U, 1500U);
  CommandApp_Task(2300U); /* 没有两路新样本不能凭任务轮询放行。 */
  assert(!g_robot_command.released);
  inputs[0].valid_pulse_count++;
  CommandApp_Task(2301U); assert(!g_robot_command.released);
  inputs[1].valid_pulse_count++;
  CommandApp_Task(2302U); assert(g_robot_command.released);

  inputs[0].online = 0U; inputs[0].fault = PWM_INPUT_FAULT_TIMEOUT;
  CommandApp_Task(2310U);
  assert(!g_robot_command.released && g_robot_command.fault_event_count == event_count + 1U);
  CommandApp_Task(2400U);
  assert(g_robot_command.fault_event_count == event_count + 1U);
  sample(2410U, 1502U, 1516U); assert(!g_robot_command.released);
  release(2500U);
  CommandApp_GetFcDiagnostics(&d);
  assert(d.drive.raw_us == 1502U && d.steer.raw_us == 1500U && d.center_elapsed_ms == 200U);
  assert(g_robot_chassis.fc_release_ready && g_robot_chassis.fc_drive_online && g_robot_chassis.fc_steer_online);
  CommandApp_GetFcDiagnostics(NULL);

  /* 任务漏过断流，底层已收4好帧恢复，也不能继续旧授权的非零目标。 */
  inputs[0].timeout_event_count++;
  inputs[0].valid_pulse_count += 4U;
  inputs[1].valid_pulse_count += 4U;
  inputs[1].raw_us = inputs[1].filtered_us = 2000U;
  event_count = g_robot_command.fault_event_count;
  CommandApp_Task(2800U);
  assert(!g_robot_command.source_online && !g_robot_command.released);
  assert(!g_robot_command.throttle_permille && !g_robot_command.steering_permille);
  assert(g_robot_command.fault_event_count == event_count + 1U);
  sample(2810U, 1502U, 2000U);
  assert(g_robot_command.source_online && !g_robot_command.released);
  release(2900U);

  reset();
  release(UINT32_MAX - 100U); /* 回中计时跨HAL tick回绕。 */
  reset();
  sample(10U, 1502U, 1500U);
  inputs[1].online = 0U; inputs[1].fault = PWM_INPUT_FAULT_WARMUP;
  CommandApp_Task(300U); assert(!g_robot_command.released);
  failed_registration = 1;
  reset();
  sample(10U, 1502U, 1500U); sample(500U, 1502U, 1500U);
  assert(!g_robot_command.source_online && !g_robot_command.released);
  puts("PASS: FC-only command mapping, raw+filtered neutral, fresh pair release, latent invalid, timeout/recovery, calibration MAIN1 stop, tick wrap, registration failure");
  return 0;
}
