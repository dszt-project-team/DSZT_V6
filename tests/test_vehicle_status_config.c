/* Exercise each configuration gate independently without editing firmware macros. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "chassis/chassis_config.h"
static unsigned cfg_parameters=1, cfg_fc=1, cfg_oid=1, cfg_loop=1, cfg_zero=1, cfg_linkage=1, cfg_cal_health=0;
#undef CHASSIS_PARAMETERS_CONFIRMED
#undef CHASSIS_FC_CONTROL_ENABLE
#undef CHASSIS_OID_OUTPUT_ENABLE
#undef CHASSIS_STEER_CLOSED_LOOP_ENABLE
#undef CHASSIS_STEER_ZERO_CONFIRMED
#undef CHASSIS_STEER_LINKAGE_CONFIRMED
#undef CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH
#define CHASSIS_PARAMETERS_CONFIRMED cfg_parameters
#define CHASSIS_FC_CONTROL_ENABLE cfg_fc
#define CHASSIS_OID_OUTPUT_ENABLE cfg_oid
#define CHASSIS_STEER_CLOSED_LOOP_ENABLE cfg_loop
#define CHASSIS_STEER_ZERO_CONFIRMED cfg_zero
#define CHASSIS_STEER_LINKAGE_CONFIRMED cfg_linkage
#define CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH cfg_cal_health
#include "vehicle_status.h"
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
int main(void)
{
  unsigned *flags[]={&cfg_parameters,&cfg_fc,&cfg_oid,&cfg_loop,&cfg_zero,&cfg_linkage};
  memset(&g_robot_command,0,sizeof(g_robot_command));
  memset(&g_robot_chassis,0,sizeof(g_robot_chassis));
  g_robot_command.source_online=g_robot_command.released=1;
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;
  g_robot_chassis.fc_drive_online=g_robot_chassis.fc_steer_online=1;
  g_robot_chassis.left_online=g_robot_chassis.right_online=1;
  g_robot_chassis.steer_healthy[0]=g_robot_chassis.steer_healthy[1]=1;
  g_robot_chassis.steer_zero_valid[0]=g_robot_chassis.steer_zero_valid[1]=1;
  g_robot_chassis.fc_release_ready=g_robot_chassis.steer_released=1;
  g_robot_chassis.parameters_confirmed=g_robot_chassis.motion_enabled=1;
  assert(VehicleStatus_Get()==VEHICLE_STATUS_AUTO_READY);
  for(unsigned i=0;i<sizeof(flags)/sizeof(flags[0]);i++)
  {
    *flags[i]=0; assert(VehicleStatus_Get()==VEHICLE_STATUS_CONFIG_HOLD);
    *flags[i]=1; assert(VehicleStatus_Get()==VEHICLE_STATUS_AUTO_READY);
  }
  g_robot_command.mode=ROBOT_MODE_CALIBRATION;
  assert(VehicleStatus_Get()==VEHICLE_STATUS_CONFIG_HOLD); /* Mode/side mismatch. */
  g_robot_chassis.steer_calibration_side=1;
  cfg_oid=cfg_loop=cfg_zero=cfg_linkage=0;
  assert(VehicleStatus_Get()==VEHICLE_STATUS_CALIBRATION);
  cfg_fc=0; assert(VehicleStatus_Get()==VEHICLE_STATUS_CONFIG_HOLD); cfg_fc=1;
  g_robot_chassis.steer_healthy[0]=g_robot_chassis.steer_healthy[1]=0;
  assert(VehicleStatus_Get()==VEHICLE_STATUS_CALIBRATION); /* Optional encoder bypass. */
  cfg_cal_health=1; assert(VehicleStatus_Get()==VEHICLE_STATUS_STEER_FAULT);
  g_robot_chassis.steer_healthy[0]=1;
  assert(VehicleStatus_Get()==VEHICLE_STATUS_CALIBRATION); /* Unselected encoder ignored. */
  g_robot_chassis.steer_calibration_side=3;
  assert(VehicleStatus_Get()==VEHICLE_STATUS_CONFIG_HOLD);
  puts("PASS: disabled FC/OID/loop/linkage/zero/parameters are configuration hold, maintenance bypass and selected encoder gate");
  return 0;
}
