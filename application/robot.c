#include "robot.h"

#include "bsp_callback.h"
#include "buzzer_app.h"
#include "chassis_app.h"
#include "command_app.h"
#include "debug_app.h"
#include "lighting_app.h"
#include "robot_def.h"

#include "main.h"

#include <string.h>

RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;

void RobotInit(void)
{
  memset(&g_robot_command, 0, sizeof(g_robot_command));
  memset(&g_robot_chassis, 0, sizeof(g_robot_chassis));
  g_robot_command.mode = ROBOT_MODE_LOCKED;
  g_robot_command.gate = ROBOT_GATE_STARTUP_LOCK_REQUIRED;

  BspCallback_Init();
  CommandApp_Init();
  ChassisApp_Init();
  LightingApp_Init();
  BuzzerApp_Init();
  DebugApp_Init();
}

void RobotTask(void)
{
  uint32_t now_ms = HAL_GetTick();

  CommandApp_Task(now_ms);
  ChassisApp_Task(now_ms);
  LightingApp_Task(now_ms);
  BuzzerApp_Task(now_ms);
  DebugApp_Task(now_ms);
}
