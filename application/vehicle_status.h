/* 灯带与蜂鸣器共用的只读状态分类，不参与控制授权，不修改任何运动目标。 */
#ifndef VEHICLE_STATUS_H
#define VEHICLE_STATUS_H

#include "robot_def.h"

typedef enum
{
  VEHICLE_STATUS_RC_LOST = 0,     /* 遥控保护已生效或接收机尚未可用。 */
  VEHICLE_STATUS_STEER_FAULT,     /* 转向故障，或正常闭环所需编码器不健康。 */
  VEHICLE_STATUS_OID_FAULT,       /* 行走驱动器离线或报告故障。 */
  VEHICLE_STATUS_FC_FAULT,        /* 自动档的飞控输入未就绪/超时/越界。 */
  VEHICLE_STATUS_CONFIG_HOLD,    /* 整车参数或闭环零点未确认。 */
  VEHICLE_STATUS_STARTUP_WAIT,   /* 等待 CH5 锁车完成启动授权。 */
  VEHICLE_STATUS_MODE_WAIT,      /* CH5 档位还未通过连续健康帧确认。 */
  VEHICLE_STATUS_LOCKED,         /* 主动锁车，未触发上面的故障/配置门。 */
  VEHICLE_STATUS_RELEASE_WAIT,   /* 工作档等待回中、转向释放或 OID 重臂。 */
  VEHICLE_STATUS_CALIBRATION,    /* 单轮开环维护，不表示整车行走可用。 */
  VEHICLE_STATUS_MANUAL_READY,  /* 手动控制链已允许行走，不表示轮子正在转动。 */
  VEHICLE_STATUS_AUTO_READY     /* 自动控制链已允许行走，不表示轮子正在转动。 */
} VehicleStatus;

/* 由 RobotTask 完成命令/底盘刷新后调用；故障优先于模式、运动灯效和普通音。 */
static inline VehicleStatus VehicleStatus_Get(void)
{
  uint8_t calibration = g_robot_chassis.steer_calibration_side;
  if ((g_robot_command.rc_online == 0U) || (g_robot_command.failsafe != 0U) ||
      (g_robot_command.gate == ROBOT_GATE_RC_LOST))
    return VEHICLE_STATUS_RC_LOST;
  if ((g_robot_chassis.steer_fault != 0U) ||
      ((calibration == 0U) &&
       ((g_robot_chassis.steer_healthy[0] == 0U) ||
        (g_robot_chassis.steer_healthy[1] == 0U))))
    return VEHICLE_STATUS_STEER_FAULT;
  /* 单轮维护本来就禁止行走，未接 OID 不应遮蔽标定状态。 */
  if ((calibration == 0U) &&
      ((g_robot_chassis.left_fault != 0U) || (g_robot_chassis.right_fault != 0U) ||
       (g_robot_chassis.left_online == 0U) || (g_robot_chassis.right_online == 0U)))
    return VEHICLE_STATUS_OID_FAULT;
  if ((g_robot_command.mode == ROBOT_MODE_AUTO_FC) &&
      ((g_robot_chassis.fc_drive_online == 0U) ||
       (g_robot_chassis.fc_steer_online == 0U) ||
       (g_robot_chassis.fc_drive_fault != 0U) || (g_robot_chassis.fc_steer_fault != 0U)))
    return VEHICLE_STATUS_FC_FAULT;
  if ((g_robot_chassis.parameters_confirmed == 0U) ||
      ((calibration == 0U) &&
       ((g_robot_chassis.steer_zero_valid[0] == 0U) ||
        (g_robot_chassis.steer_zero_valid[1] == 0U))))
    return VEHICLE_STATUS_CONFIG_HOLD;
  if (g_robot_command.gate == ROBOT_GATE_STARTUP_LOCK_REQUIRED)
    return VEHICLE_STATUS_STARTUP_WAIT;
  if (g_robot_command.gate == ROBOT_GATE_MODE_CONFIRMING)
    return VEHICLE_STATUS_MODE_WAIT;
  if ((g_robot_command.mode != ROBOT_MODE_LOCKED) &&
      (g_robot_command.mode != ROBOT_MODE_MANUAL) &&
      (g_robot_command.mode != ROBOT_MODE_AUTO_FC))
    return VEHICLE_STATUS_MODE_WAIT;
  if (g_robot_command.mode == ROBOT_MODE_LOCKED)
    return VEHICLE_STATUS_LOCKED;
  if (calibration != 0U)
    return VEHICLE_STATUS_CALIBRATION;
  if ((g_robot_command.gate != ROBOT_GATE_READY) ||
      (g_robot_chassis.motion_enabled == 0U) || (g_robot_chassis.steer_released == 0U) ||
      (g_robot_chassis.oid_safety_state != 0U) ||
      ((g_robot_command.mode == ROBOT_MODE_AUTO_FC) &&
       (g_robot_chassis.fc_release_ready == 0U)))
    return VEHICLE_STATUS_RELEASE_WAIT; /* OID 安全状态0才就绪，不沿用10 ms前的放行快照。 */
  return (g_robot_command.mode == ROBOT_MODE_AUTO_FC) ?
      VEHICLE_STATUS_AUTO_READY : VEHICLE_STATUS_MANUAL_READY;
}

#endif
