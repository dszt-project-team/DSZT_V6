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
  g_robot_command.source_online = g_robot_command.released = 1U;
  g_robot_command.mode = ROBOT_MODE_AUTO_FC;
  g_robot_chassis.left_online = g_robot_chassis.right_online = 1U;
  g_robot_chassis.fc_drive_online = g_robot_chassis.fc_steer_online = 1U;
  g_robot_chassis.fc_release_ready = 1U;
  g_robot_chassis.steer_healthy[0] = g_robot_chassis.steer_healthy[1] = 1U;
  g_robot_chassis.steer_zero_valid[0] = g_robot_chassis.steer_zero_valid[1] = 1U;
  g_robot_chassis.parameters_confirmed = g_robot_chassis.motion_enabled = 1U;
  g_robot_chassis.steer_released = 1U;
}
int main(void)
{
  RobotCommand before_command;
  RobotChassisState before_chassis;
  Healthy();
  assert(VehicleStatus_Get() == VEHICLE_STATUS_AUTO_READY);
  before_command = g_robot_command; before_chassis = g_robot_chassis;
  (void)VehicleStatus_Get();
  assert(memcmp(&before_command, &g_robot_command, sizeof(before_command)) == 0);
  assert(memcmp(&before_chassis, &g_robot_chassis, sizeof(before_chassis)) == 0);
  g_robot_chassis.oid_safety_state = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  g_robot_chassis.oid_safety_state = 0U;
  g_robot_command.mode = 255U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CONFIG_HOLD);
  Healthy(); g_robot_command.released = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  Healthy(); g_robot_chassis.fc_release_ready = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  Healthy(); g_robot_chassis.steer_released = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  Healthy(); g_robot_chassis.motion_enabled = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  Healthy(); g_robot_chassis.left_online = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_OID_FAULT);
  g_robot_chassis.steer_healthy[1] = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_STEER_FAULT);
  g_robot_command.source_online = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_FC_FAULT);
  Healthy(); g_robot_chassis.fc_drive_fault = 2U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_FC_FAULT);
  Healthy(); g_robot_command.gate = ROBOT_GATE_FC_INPUT_INVALID;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_FC_FAULT);
  Healthy(); g_robot_chassis.parameters_confirmed = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CONFIG_HOLD);
  Healthy(); g_robot_chassis.steer_zero_valid[1] = 0U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CONFIG_HOLD);
  Healthy(); g_robot_command.mode = ROBOT_MODE_CALIBRATION;
  g_robot_chassis.steer_calibration_side = 1U;
  g_robot_chassis.left_online = g_robot_chassis.right_online = 0U;
  g_robot_chassis.steer_healthy[0] = g_robot_chassis.steer_healthy[1] = 0U;
  g_robot_chassis.steer_zero_valid[0] = g_robot_chassis.steer_zero_valid[1] = 0U;
  g_robot_chassis.motion_enabled = 0U; g_robot_chassis.oid_safety_state = 2U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_CALIBRATION);
  g_robot_command.gate = ROBOT_GATE_CAL_DRIVE_NOT_CENTERED;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  g_robot_command.gate = ROBOT_GATE_FC_CENTERING;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_RELEASE_WAIT);
  g_robot_chassis.steer_fault = 1U;
  assert(VehicleStatus_Get() == VEHICLE_STATUS_STEER_FAULT);
  puts("PASS: FC-only readonly status, fault priorities, centering/rearm and isolated calibration");
  return 0;
}
