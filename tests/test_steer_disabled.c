/* Compile-time stop switch must prevent either closed-loop controller enabling. */
#include "../application/chassis/chassis_config.h"
#undef CHASSIS_STEER_CLOSED_LOOP_ENABLE
#define CHASSIS_STEER_CLOSED_LOOP_ENABLE 0U
#define TEST_USE_CONFIG_SIDE 1
#define main calibration_reference_main
#include "test_dual_steer.c"
#undef main
int main(void)
{
  uint32_t t;
  DualSteer_Init();
  s_encoder[0].sample.healthy=s_encoder[1].sample.healthy=1;
  g_robot_command.mode=ROBOT_MODE_MANUAL;
  g_robot_command.rc_online=g_robot_command.throttle_centered=1;
  DualSteer_Task(0,0,1,10);DualSteer_Task(0,0,1,210);
  for(t=220;t<1000;t+=10) assert(DualSteer_Task(-1000,0,1,t)==0);
  assert(s_ctrl[0].state==STEER_ANGLE_CTRL_DISABLED && s_ctrl[1].state==STEER_ANGLE_CTRL_DISABLED);
  assert(s_motor[0].brake_on && s_motor[1].brake_on && !DualSteer_MotionReady());
  puts("PASS: closed-loop stop switch keeps both motors disabled");
  return 0;
}
