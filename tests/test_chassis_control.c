/* 真实ChassisApp，模拟OID总线与转向内环；不代表硬件验收。 */
#include <assert.h>
#include <stdio.h>
#include "../application/chassis/chassis_app.c"

UART_HandleTypeDef huart3;
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
static unsigned zero_requests, rearm_requests, steer_stop_calls;
static uint8_t steer_ready = 1U, drive_ready = 1U, must_service_zero;
static int32_t sent_left, sent_right;

void BSP_RS485_Init(BSP_RS485_Bus_t *b, UART_HandleTypeDef *u, GPIO_TypeDef *p, uint16_t pin)
{ (void)b; (void)u; (void)p; (void)pin; }
void FrontDrive_Init(FrontDrive_Handle_t *d, BSP_RS485_Bus_t *b, uint8_t l, uint8_t r, uint8_t p)
{ (void)b; (void)l; (void)r; (void)p; memset(d, 0, sizeof(*d)); }
void FrontDrive_Task(FrontDrive_Handle_t *d, uint32_t now)
{
  if (must_service_zero) assert(!d->pending_left_erpm && !d->pending_right_erpm);
  d->left.status.last_update_ms = d->right.status.last_update_ms = now;
}
const OID_ESC_Status_t *FrontDrive_GetStatus(const FrontDrive_Handle_t *d, FrontDrive_Side_t side)
{ return side == FRONT_DRIVE_SIDE_LEFT ? &d->left.status : &d->right.status; }
uint8_t FrontDrive_IsOnline(const FrontDrive_Handle_t *d, FrontDrive_Side_t s, uint32_t now)
{ (void)d; (void)s; (void)now; return drive_ready; }
uint8_t FrontDrive_IsMotionReady(const FrontDrive_Handle_t *d, uint32_t now)
{ (void)now; return (uint8_t)(drive_ready && !d->safety_state && !d->left.status.fault && !d->right.status.fault); }
uint8_t FrontDrive_IsSafetyRearmRequired(const FrontDrive_Handle_t *d)
{ return (uint8_t)(d->safety_state == FRONT_DRIVE_SAFETY_WAIT_REARM || d->safety_state == FRONT_DRIVE_SAFETY_WAIT_VERIFY); }
void FrontDrive_RearmSafety(FrontDrive_Handle_t *d, uint32_t now)
{ (void)now; rearm_requests++; d->safety_state = FRONT_DRIVE_SAFETY_IDLE; }
HAL_StatusTypeDef FrontDrive_StopUrgent(FrontDrive_Handle_t *d)
{
  zero_requests++;
  d->pending_left_erpm = d->pending_right_erpm = 0;
  sent_left = sent_right = 0;
  return HAL_OK;
}
HAL_StatusTypeDef FrontDrive_SetTargetErpm(FrontDrive_Handle_t *d, int32_t l, int32_t r)
{
  d->pending_left_erpm = l; d->pending_right_erpm = r;
  sent_left = l; sent_right = r;
  return HAL_OK;
}
void DualSteer_Init(void) {}
int16_t DualSteer_Task(int16_t c, int16_t drive, uint8_t ready, uint32_t now)
{ (void)drive; (void)now; if (!ready) steer_stop_calls++; return c; }
uint8_t DualSteer_MotionReady(void) { return steer_ready; }
void DualSteer_Geometry(int16_t c, float *l, float *r, float *ls, float *rs)
{ (void)c; *l = *r = 0.0f; *ls = *rs = 1.0f; }

static void command(int16_t throttle, uint8_t released)
{
  g_robot_command.mode = CHASSIS_STEER_CALIBRATION_SIDE ? ROBOT_MODE_CALIBRATION : ROBOT_MODE_AUTO_FC;
  g_robot_command.source_online = 1U;
  g_robot_command.released = released;
  g_robot_command.centered = (uint8_t)(throttle == 0);
  g_robot_command.throttle_permille = throttle;
  g_robot_command.speed_limit_erpm = 4900U;
  g_robot_command.gate = released ? ROBOT_GATE_READY : ROBOT_GATE_FC_CENTERING;
}
int main(void)
{
  uint32_t t;
  unsigned stops;
  ChassisApp_Init();
  command(0, 0U);
  ChassisApp_Task(100U); assert(!g_robot_chassis.motion_enabled);
  command(1000, 1U);
  ChassisApp_Task(110U);
  assert(sent_left == (CHASSIS_STEER_CALIBRATION_SIDE ? 0 : 4900));
  assert(sent_right == sent_left);
  /* 还没到10ms控制周期，也必须先清掉待发旧油门再服务OID并停止转向。 */
  stops = steer_stop_calls;
  g_robot_command.source_online = 0U;
  must_service_zero = 1U;
  ChassisApp_Task(111U);
  assert(!sent_left && !sent_right && !g_robot_chassis.motion_enabled && steer_stop_calls > stops);
  must_service_zero = 0U;
  command(-1000, 1U);
  ChassisApp_Task(120U); assert(sent_left == (CHASSIS_STEER_CALIBRATION_SIDE ? 0 : -4900));
  g_robot_command.mode = 255U;
  must_service_zero = 1U;
  ChassisApp_Task(121U); assert(!sent_left && !sent_right && !g_robot_chassis.motion_enabled);
  must_service_zero = 0U;
  command(0, 1U);
  ChassisApp_Task(130U);
  for (t = 140U; t < 60140U; t += 10U)
  {
    s_front_drive.left.status.speed_erpm = 80;
    ChassisApp_Task(t);
    assert(!g_robot_chassis.oid_stop_fault && !sent_left && !sent_right);
  }
  assert(zero_requests >= 601U);
  command(1000, 1U);
  ChassisApp_Task(t += 10U);
  assert(sent_left == (CHASSIS_STEER_CALIBRATION_SIDE ? 0 : 4900));
  drive_ready = 0U;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled && !sent_left);
  drive_ready = 1U; steer_ready = 0U;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled);
  steer_ready = 1U; s_front_drive.left.status.fault = 1U;
  ChassisApp_Task(t += 10U); assert(!g_robot_chassis.motion_enabled);
  s_front_drive.left.status.fault = 0U;

  /* 不得拿故障前的长期中位抵扣OID重臂的200ms。 */
  command(0, 1U);
  ChassisApp_Task(t += 10U);
  ChassisApp_Task(t += 1000U);
  assert(rearm_requests == 0U);
  s_front_drive.safety_state = FRONT_DRIVE_SAFETY_WAIT_REARM;
  ChassisApp_Task(t += 10U);
  ChassisApp_Task(t += 190U);
  assert(rearm_requests == 0U);
  ChassisApp_Task(t += 10U);
  assert(rearm_requests == 1U);
  /* 后续新的WAIT_VERIFY阶段重新计时；中途离中会再次清计时。 */
  ChassisApp_Task(t += 1U);
  s_front_drive.safety_state = FRONT_DRIVE_SAFETY_WAIT_VERIFY;
  ChassisApp_Task(t += 10U);
  ChassisApp_Task(t += 190U); assert(rearm_requests == 1U);
  g_robot_command.centered = 0U;
  ChassisApp_Task(t += 10U);
  g_robot_command.centered = 1U;
  ChassisApp_Task(t += 10U);
  ChassisApp_Task(t += 190U); assert(rearm_requests == 1U);
  ChassisApp_Task(t += 10U); assert(rearm_requests == 2U);
  /* 无RC锁车档仍保留低速中位额外回读；高反馈、离中、陈旧反馈均不许可。 */
  s_front_drive.left.status.speed_erpm = 0;
  ChassisApp_Task(t += 10U); assert(s_front_drive.control_read_allowed);
  s_front_drive.left.status.speed_erpm = 51;
  ChassisApp_Task(t += 10U); assert(!s_front_drive.control_read_allowed);
  s_front_drive.left.status.speed_erpm = 0;
  g_robot_command.centered = 0U;
  ChassisApp_Task(t += 10U); assert(!s_front_drive.control_read_allowed);
  g_robot_command.centered = 1U;
  s_front_drive.left.status.last_update_ms = t - 300U;
  ChassisApp_Task(t += 10U); assert(!s_front_drive.control_read_allowed);
  puts("PASS: FC-only chassis, pre-service zero, invalid-mode stop, 60s idle, OID/steer gating, fresh recovery center, diagnostic readback, calibration hard OID lock");
  return 0;
}
