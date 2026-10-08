#include "../application/chassis/chassis_config.h"
#define TEST_USE_CONFIG_SIDE 1
#define main calibration_reference_main
#include "test_dual_steer.c"
#undef main
int main(void)
{
  uint32_t t;
  float before;
  int16_t drive;
  DualSteer_Init();
  s_encoder[0].sample.healthy=s_encoder[1].sample.healthy=1;
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;
  g_robot_command.source_online=g_robot_command.released=g_robot_command.centered=1;
  DualSteer_Task(0,0,1,10); DualSteer_Task(0,0,1,210);
  drive=DualSteer_Task(-1000,0,1,220);
  assert(drive==0); /* first 16 permille lies within differential deadband */
  assert(fabsf(s_ctrl[0].target_angle_deg+3.0f)<0.001f);
  assert(fabsf(s_ctrl[1].target_angle_deg+3.0f)<0.001f);
  assert(g_robot_chassis.steering_scheduled_permille==-1000);
  for(t=230;t<=510;t+=10) drive=DualSteer_Task(-1000,0,1,t);
  assert(fabsf(s_ctrl[0].target_angle_deg+89.719f*28.0f/33.0f)<0.001f);
  assert(drive==-480); /* steering arrives at 300ms, OID ramp is independent */
  before=s_ctrl[0].target_angle_deg;
  DualSteer_Task(1000,0,1,520);
  assert(fabsf(s_ctrl[0].target_angle_deg-before-3.0f)<0.001f);
  before=s_ctrl[0].target_angle_deg;
  DualSteer_Task(1000,0,1,520);
  assert(s_ctrl[0].target_angle_deg==before);
  DualSteer_Task(1000,0,1,540);
  assert(fabsf(s_ctrl[0].target_angle_deg-before-6.0f)<0.001f);
  before=s_ctrl[0].target_angle_deg;
  DualSteer_Task(1000,0,1,1000);
  assert(fabsf(s_ctrl[0].target_angle_deg-before-60.0f)<0.001f);
  g_robot_command.source_online=0;
  assert(DualSteer_Task(1000,0,0,1010)==0);
  assert(!s_slew_valid && s_motor[0].brake_on && s_motor[1].brake_on);
  g_robot_command.source_online=1;
  DualSteer_Task(0,0,1,1020); DualSteer_Task(0,0,1,1220);
  assert(s_ctrl[0].target_angle_deg==0 && s_ctrl[1].target_angle_deg==0);
  DualSteer_Task(1000,0,1,1230);
  assert(fabsf(s_ctrl[0].target_angle_deg-3.0f)<0.001f);
  s_encoder[0].sample.healthy=0;
  assert(DualSteer_Task(1000,0,1,1240)==0 && !s_slew_valid);
  assert(s_motor[0].brake_on && s_motor[1].brake_on);
  puts("PASS: V4 300deg/s shaft ramp, independent 1.6/s OID, reversal, elapsed cap, stop/rearm/fault");
  return 0;
}
