#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vehicle_status.h"

RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;

static void Healthy(void)
{
  memset(&g_robot_command, 0, sizeof(g_robot_command));
  memset(&g_robot_chassis, 0, sizeof(g_robot_chassis));
  g_robot_command.rc_online = 1U;
  g_robot_command.mode = ROBOT_MODE_MANUAL;
  g_robot_chassis.left_online = g_robot_chassis.right_online = 1U;
  g_robot_chassis.steer_healthy[0] = g_robot_chassis.steer_healthy[1] = 1U;
  g_robot_chassis.steer_zero_valid[0] = g_robot_chassis.steer_zero_valid[1] = 1U;
  g_robot_chassis.parameters_confirmed = g_robot_chassis.motion_enabled = 1U;
  g_robot_chassis.steer_released = 1U;
}

int main(void)
{
  RobotCommand command_before;
  RobotChassisState chassis_before;
  Healthy();
  assert(VehicleStatus_Get() == VEHICLE_STATUS_MANUAL_READY);
  command_before = g_robot_command;
  chassis_before = g_robot_chassis;
  (void)VehicleStatus_Get();
  assert(memcmp(&command_before, &g_robot_command, sizeof(command_before)) == 0);
  assert(memcmp(&chassis_before, &g_robot_chassis, sizeof(chassis_before)) == 0);
  g_robot_chassis.oid_safety_state = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  g_robot_chassis.oid_safety_state = 0U;
  g_robot_command.mode = 255U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_MODE_WAIT);
  g_robot_command.mode = ROBOT_MODE_AUTO_FC;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_FC_FAULT);
  g_robot_chassis.fc_drive_online = g_robot_chassis.fc_steer_online = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  g_robot_chassis.fc_release_ready = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_AUTO_READY);
  g_robot_chassis.steer_fault = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_STEER_FAULT);
  g_robot_command.failsafe = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RC_LOST);
  Healthy();
  g_robot_command.gate = ROBOT_GATE_STARTUP_LOCK_REQUIRED;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_STARTUP_WAIT);
  g_robot_command.gate = ROBOT_GATE_MODE_CONFIRMING;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_MODE_WAIT);
  g_robot_command.gate = ROBOT_GATE_THROTTLE_CENTERING;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  g_robot_command.mode = ROBOT_MODE_LOCKED;
  g_robot_command.gate = ROBOT_GATE_READY;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_LOCKED);
  g_robot_chassis.right_online = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_OID_FAULT);
  Healthy();
  g_robot_chassis.steer_healthy[1] = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_STEER_FAULT);
  Healthy();
  g_robot_chassis.parameters_confirmed = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CONFIG_HOLD);
  Healthy();
  g_robot_chassis.steer_zero_valid[1] = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CONFIG_HOLD);
  Healthy();
  g_robot_chassis.steer_calibration_side = 1U;
  g_robot_chassis.right_online = g_robot_chassis.steer_healthy[1] = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CALIBRATION);
  puts("PASS: shared read-only vehicle status priorities, gates, automatic and calibration.");
  return 0;
}
