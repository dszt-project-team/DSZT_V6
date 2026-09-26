/* 真实ChassisApp控制链；UART、PWM采集及转向位置环由桩提供。
 * PWM捕获/均值另由test_pwm_input使用真实模块验证。 */
#include <assert.h>
#include <stdio.h>
#include "../application/chassis/chassis_app.c"
#include "../application/debug/debug_config.h"

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart3;
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
struct PwmInput_Handle { unsigned side; };
static struct PwmInput_Handle handles[2];
static PwmInput_Snapshot_t inputs[2];
static PwmInput_Config_t configs[2];
static unsigned registered, zero_requests;
static uint8_t steer_ready = 1U, drive_ready = 1U;
static int32_t sent_left, sent_right;

PwmInput_Handle_t *PwmInput_Register(const PwmInput_Config_t *c)
{
  unsigned side = registered++;
  assert(side < 2U);
  configs[side] = *c;
  handles[side].side = side;
  return &handles[side];
}
uint8_t PwmInput_GetSnapshot(PwmInput_Handle_t *h, PwmInput_Snapshot_t *s, uint32_t now)
{ (void)now; *s = inputs[h->side]; return 1U; }
void BSP_RS485_Init(BSP_RS485_Bus_t *b, UART_HandleTypeDef *u, GPIO_TypeDef *p, uint16_t pin)
{ (void)b; (void)u; (void)p; (void)pin; }
void FrontDrive_Init(FrontDrive_Handle_t *d, BSP_RS485_Bus_t *b, uint8_t l, uint8_t r, uint8_t p)
{ (void)b; (void)l; (void)r; (void)p; memset(d, 0, sizeof(*d)); }
void FrontDrive_Task(FrontDrive_Handle_t *d, uint32_t now)
{ d->left.status.last_update_ms = d->right.status.last_update_ms = now; }
const OID_ESC_Status_t *FrontDrive_GetStatus(const FrontDrive_Handle_t *d, FrontDrive_Side_t side)
{ return side == FRONT_DRIVE_SIDE_LEFT ? &d->left.status : &d->right.status; }
uint8_t FrontDrive_IsOnline(const FrontDrive_Handle_t *d, FrontDrive_Side_t s, uint32_t now)
{ (void)d; (void)s; (void)now; return drive_ready; }
uint8_t FrontDrive_IsMotionReady(const FrontDrive_Handle_t *d, uint32_t now)
{ (void)now; return (uint8_t)(drive_ready && !d->left.status.fault && !d->right.status.fault); }
uint8_t FrontDrive_IsSafetyRearmRequired(const FrontDrive_Handle_t *d) { (void)d; return 0U; }
void FrontDrive_RearmSafety(FrontDrive_Handle_t *d, uint32_t now) { (void)d; (void)now; }
HAL_StatusTypeDef FrontDrive_StopUrgent(FrontDrive_Handle_t *d)
{ (void)d; zero_requests++; sent_left = sent_right = 0; return HAL_OK; }
HAL_StatusTypeDef FrontDrive_SetTargetErpm(FrontDrive_Handle_t *d, int32_t l, int32_t r)
{ (void)d; sent_left = l; sent_right = r; return HAL_OK; }
void DualSteer_Init(void) {}
int16_t DualSteer_Task(int16_t c, int16_t drive, uint8_t ready, uint32_t now)
{ (void)drive; (void)ready; (void)now; return c; }
uint8_t DualSteer_MotionReady(void) { return steer_ready; }
void DualSteer_Geometry(int16_t c, float *l, float *r, float *ls, float *rs)
{ (void)c; *l = *r = 0.0f; *ls = *rs = 1.0f; }

static void set_fc(uint16_t drive, uint8_t online, PwmInput_Fault_t fault)
{
  inputs[0].raw_us = inputs[0].filtered_us = drive;
  inputs[0].online = online; inputs[0].fault = fault;
  inputs[1].raw_us = inputs[1].filtered_us = 1500U;
  inputs[1].online = 1U; inputs[1].fault = PWM_INPUT_FAULT_NONE;
}
int main(void)
{
  uint32_t t;
  ChassisApp_Init();
  assert(configs[0].average_window == 1U && configs[1].average_window == 8U);
  assert(configs[0].valid_samples_to_online == 4U && configs[1].valid_samples_to_online == 4U);
  assert(configs[0].minimum_valid_us == 800U && configs[0].maximum_valid_us == 2200U);
  assert(DEBUG_APP_OUTPUT_MODE == DEBUG_APP_OUTPUT_OID);
  g_robot_command.rc_online = 1U;
  g_robot_command.mode = ROBOT_MODE_AUTO_FC;
  set_fc(1502U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(100U); assert(!g_robot_chassis.motion_enabled);
  ChassisApp_Task(299U); assert(!g_robot_chassis.motion_enabled);
  ChassisApp_Task(309U); assert(g_robot_chassis.fc_release_ready && g_robot_chassis.motion_enabled);
  set_fc(1770U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(319U); assert(sent_left == 4900 && sent_right == 4900);
  set_fc(1260U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(329U); assert(sent_left == -4900 && sent_right == -4900);
  set_fc(1502U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(339U); assert(sent_left == 0 && sent_right == 0);
  for (t = 349U; t < 60349U; t += 10U)
  {
    /* 停车期间模拟越界反馈，停车维护不得产生永久锁存。 */
    s_front_drive.left.status.speed_erpm = 80;
    ChassisApp_Task(t);
    assert(g_robot_chassis.oid_stop_fault == 0U);
  }
  assert(zero_requests >= 601U);
  set_fc(1770U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(t); assert(sent_left == 4900 && g_robot_chassis.motion_enabled);
  set_fc(1770U, 0U, PWM_INPUT_FAULT_TIMEOUT);
  ChassisApp_Task(t += 10U); assert(sent_left == 0 && !g_robot_chassis.fc_release_ready);
  set_fc(1770U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled);
  set_fc(1502U, 1U, PWM_INPUT_FAULT_NONE);
  ChassisApp_Task(t += 10U);
  ChassisApp_Task(t += 200U); assert(g_robot_chassis.motion_enabled);
  set_fc(1770U, 1U, PWM_INPUT_FAULT_NONE);
  drive_ready = 0U;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled && !sent_left);
  drive_ready = 1U; steer_ready = 0U;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled);
  steer_ready = 1U; s_front_drive.left.status.fault = 1U;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled);
  s_front_drive.left.status.fault = 0U;
  g_robot_command.mode = ROBOT_MODE_LOCKED;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled && !sent_left);
  puts("PASS: real chassis MAIN1 direct mapping/reversal/neutral; 60s idle resumes; FC/steer/OID/lock gates retained");
  return 0;
}
