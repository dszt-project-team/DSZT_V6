/* Actual closed-loop application with mocked hardware and inner controller.
 * Reuse the same hardware mock definitions; the renamed calibration test is not run. */
#include "../application/chassis/chassis_config.h"
#undef CHASSIS_STEER_CALIBRATION_SIDE
#undef CHASSIS_STEER_LEFT_ZERO_CONFIRMED
#undef CHASSIS_STEER_RIGHT_ZERO_CONFIRMED
#define CHASSIS_STEER_CALIBRATION_SIDE 0U
#define CHASSIS_STEER_LEFT_ZERO_CONFIRMED 1U
#define CHASSIS_STEER_RIGHT_ZERO_CONFIRMED 1U
#define TEST_USE_CONFIG_SIDE 1
#define main calibration_reference_main
#include "test_dual_steer.c"
#undef main
int main(void)
{
  uint32_t t;
  int16_t common=0;
  float left, right, left_scale, right_scale;
  /* FC branch preserves accepted drive and geometry parameters. */
  assert(CHASSIS_PARAMETERS_CONFIRMED == 1U && CHASSIS_OID_OUTPUT_ENABLE == 1U);
  assert(CHASSIS_OID_COMMISSION_MAX_ERPM == 0U && CHASSIS_OID_READBACK_ENABLE == 1U);
  assert(CHASSIS_STEER_LINKAGE_CONFIRMED == 1U);
  assert(CHASSIS_OID_LEFT_ID == 1U && CHASSIS_OID_RIGHT_ID == 2U);
  assert(CHASSIS_OID_LEFT_DIRECTION == 1 && CHASSIS_OID_RIGHT_DIRECTION == 1);
  assert(CHASSIS_FC_SPEED_LIMIT_ERPM == 4900U);
  assert(CHASSIS_STEER_LEFT_ZERO_RAW == 3189U && CHASSIS_STEER_RIGHT_ZERO_RAW == 3073U);
  assert(CHASSIS_MAX_INNER_WHEEL_DEG == 28.0f);
  assert(CHASSIS_STEER_TARGET_MAX_DEG == 80.0f);
  assert(CHASSIS_STEER_POSITION_MAX_ABS_DEG == 82.0f);
  assert(CHASSIS_STEER_TARGET_MAX_DEG < CHASSIS_STEER_POSITION_MAX_ABS_DEG);
  DualSteer_Geometry(-1000, &left, &right, &left_scale, &right_scale);
  assert(fabsf(left * CHASSIS_STEER_INNER_OUTPUT_DEG / CHASSIS_STEER_INNER_REFERENCE_DEG + 76.125212f) < 0.001f);
  assert(fabsf(right * CHASSIS_STEER_OUTER_OUTPUT_DEG / CHASSIS_STEER_OUTER_REFERENCE_DEG + 66.915019f) < 0.001f);
  assert(76.125212f < CHASSIS_STEER_TARGET_MAX_DEG);
  assert(76.125212f + CHASSIS_STEER_POSITION_RESTART_DEG < CHASSIS_STEER_POSITION_MAX_ABS_DEG);
  DualSteer_Init();
  assert(sensor_invert[0]==1 && sensor_invert[1]==1);
  assert(positive_reverse[0]==1 && positive_reverse[1]==1);
  /* Observed plant: DIR=0 increases raw. Positive error must increase the
     signed feedback; inversion and DIR must change together, on both sides. */
  assert((positive_reverse[0] ? -1 : 1)*(sensor_invert[0] ? -1 : 1)>0);
  assert((positive_reverse[1] ? -1 : 1)*(sensor_invert[1] ? -1 : 1)>0);
  s_encoder[0].sample.healthy=s_encoder[1].sample.healthy=1;
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;
  g_robot_command.source_online=g_robot_command.released=g_robot_command.centered=1;
  DualSteer_Task(0,0,1,10);DualSteer_Task(0,0,1,210);
  for(t=220;t<=850;t+=10) common=DualSteer_Task(-1000,1000,1,t);
  assert(common==-700 && g_robot_chassis.steering_scheduled_permille==-700);
  assert(g_robot_chassis.steering_speed_gain_permille==700);
  assert(fabsf(s_ctrl[0].target_angle_deg+89.719f*28.0f/33.0f*0.7f)<0.001f);
  assert(s_ctrl[0].state==STEER_ANGLE_CTRL_HOLD && s_ctrl[1].state==STEER_ANGLE_CTRL_HOLD);
  assert(DualSteer_MotionReady()==(CHASSIS_STEER_LINKAGE_CONFIRMED!=0U));
  common=DualSteer_Task(-1000,-1000,1,860);assert(common==-700);
  s_encoder[1].sample.healthy=0;
  assert(DualSteer_Task(-1000,1000,1,870)==0 && !DualSteer_MotionReady());
  assert(g_robot_chassis.steer_fault && s_motor[0].brake_on && s_motor[1].brake_on);
  /* Verify both settled full-lock directions at zero throttle, not only the
     high-speed gain case. Every legitimate new shaft target stays protected. */
  s_encoder[1].sample.healthy=1;
  g_robot_command.source_online=0;
  DualSteer_Task(0,0,0,880);
  g_robot_command.source_online=1;
  DualSteer_Task(0,0,1,890); DualSteer_Task(0,0,1,1090);
  assert(g_robot_chassis.steer_fault && !DualSteer_MotionReady());
  /* Only a control-board initialization clears the hard steering latch. */
  DualSteer_Init();
  DualSteer_Task(0,0,1,890); DualSteer_Task(0,0,1,1090);
  for(t=1100;t<=2100;t+=10) DualSteer_Task(-1000,0,1,t);
  assert(!g_robot_chassis.steer_fault && DualSteer_MotionReady());
  assert(fabsf(s_ctrl[0].target_angle_deg+76.125212f)<0.001f);
  assert(fabsf(s_ctrl[1].target_angle_deg+66.915019f)<0.001f);
  for(t=2110;t<=3110;t+=10) DualSteer_Task(1000,0,1,t);
  assert(!g_robot_chassis.steer_fault && DualSteer_MotionReady());
  assert(fabsf(s_ctrl[0].target_angle_deg-66.915019f)<0.001f);
  assert(fabsf(s_ctrl[1].target_angle_deg-76.125212f)<0.001f);
  puts("PASS: settled steering/differential targets, reverse gain symmetry, dual-encoder fault stop (mocked inner loop)");
  return 0;
}
