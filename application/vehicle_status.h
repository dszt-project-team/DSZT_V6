/* 灯带与蜂鸣器共用的只读状态，不参与运动授权。编号同时用于 UART7 ui 字段。 */
#ifndef VEHICLE_STATUS_H
#define VEHICLE_STATUS_H
#include "robot_def.h"
#include "chassis/chassis_config.h"
typedef enum
{
  VEHICLE_STATUS_FC_FAULT = 0,   /* MAIN1/MAIN2 未就绪、超时或脉宽非法。 */
  VEHICLE_STATUS_STEER_FAULT,    /* 转向硬故障或闭环所需编码器不健康。 */
  VEHICLE_STATUS_OID_FAULT,      /* OID 离线或驱动器故障。 */
  VEHICLE_STATUS_CONFIG_HOLD,   /* 参数、零点或模式配置不满足要求。 */
  VEHICLE_STATUS_RELEASE_WAIT,  /* 等待双中位、转向释放或 OID 重臂。 */
  VEHICLE_STATUS_CALIBRATION,   /* 单轮开环维护已释放；不表示整车可行走。 */
  VEHICLE_STATUS_AUTO_READY     /* 自动链真正就绪；不代表车轮正在转动。 */
} VehicleStatus;
/* RobotTask 完成命令/底盘刷新后调用；只读取快照，故障压制所有运动灯效。 */
static inline VehicleStatus VehicleStatus_Get(void)
{
  uint8_t calibration = g_robot_chassis.steer_calibration_side;
  if ((g_robot_command.source_online == 0U) || (g_robot_command.gate == ROBOT_GATE_FC_INPUT_INVALID) ||
      (g_robot_chassis.fc_drive_online == 0U) || (g_robot_chassis.fc_steer_online == 0U) ||
      (g_robot_chassis.fc_drive_fault != 0U) || (g_robot_chassis.fc_steer_fault != 0U))
    return VEHICLE_STATUS_FC_FAULT;
  if ((g_robot_chassis.steer_fault != 0U) || ((calibration == 0U) &&
      ((g_robot_chassis.steer_healthy[0] == 0U) || (g_robot_chassis.steer_healthy[1] == 0U))) ||
      ((calibration >= 1U && calibration <= 2U) && CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH != 0U &&
       g_robot_chassis.steer_healthy[calibration - 1U] == 0U))
    return VEHICLE_STATUS_STEER_FAULT;
  /* 维护模式本来就禁止行走，不要求 OID 或未选中的编码器在线。 */
  if ((calibration == 0U) && ((g_robot_chassis.left_fault != 0U) || (g_robot_chassis.right_fault != 0U) ||
      (g_robot_chassis.left_online == 0U) || (g_robot_chassis.right_online == 0U)))
    return VEHICLE_STATUS_OID_FAULT;
  if ((CHASSIS_PARAMETERS_CONFIRMED == 0U) || (CHASSIS_FC_CONTROL_ENABLE == 0U) ||
      (g_robot_chassis.parameters_confirmed == 0U) || (calibration > 2U) ||
      ((g_robot_command.mode != ROBOT_MODE_AUTO_FC) && (g_robot_command.mode != ROBOT_MODE_CALIBRATION)) ||
      ((calibration == 0U) != (g_robot_command.mode == ROBOT_MODE_AUTO_FC)) ||
      ((calibration == 0U) && ((CHASSIS_OID_OUTPUT_ENABLE == 0U) ||
       (CHASSIS_STEER_CLOSED_LOOP_ENABLE == 0U) || (CHASSIS_STEER_ZERO_CONFIRMED == 0U) ||
       (CHASSIS_STEER_LINKAGE_CONFIRMED == 0U) || (g_robot_chassis.steer_zero_valid[0] == 0U) ||
       (g_robot_chassis.steer_zero_valid[1] == 0U)))) return VEHICLE_STATUS_CONFIG_HOLD;
  if ((g_robot_command.gate != ROBOT_GATE_READY) || (g_robot_command.released == 0U) ||
      (g_robot_chassis.fc_release_ready == 0U) || (g_robot_chassis.steer_released == 0U))
    return VEHICLE_STATUS_RELEASE_WAIT;
  if (calibration != 0U) return VEHICLE_STATUS_CALIBRATION;
  if ((g_robot_chassis.motion_enabled == 0U) || (g_robot_chassis.oid_safety_state != 0U))
    return VEHICLE_STATUS_RELEASE_WAIT;
  return VEHICLE_STATUS_AUTO_READY;
}
#endif
