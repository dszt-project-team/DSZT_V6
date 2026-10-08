/**
  ******************************************************************************
  * @file    front_drive.h
  * @brief   前轮双 OID 电调组合驱动模块。
  ******************************************************************************
  */

#ifndef FRONT_DRIVE_H
#define FRONT_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "oid_esc.h"

/* 左 OID 电调是否参与心跳、状态轮询和速度控制。 */
#define FRONT_DRIVE_LEFT_PRESENT     1U
/* 右 OID 电调是否参与心跳、状态轮询和速度控制。 */
#define FRONT_DRIVE_RIGHT_PRESENT    1U
/*
 * DSZT_V6 延续霍尔 HU/HV/HW 接口，不使用 ABZ/Z 信号。
 * 旧版往返找 Z 状态机保留在模块内部作为历史兼容路径，但本项目永久关闭。
 */
#define FRONT_DRIVE_AUTO_Z_SEARCH    0U

typedef enum
{
  FRONT_DRIVE_Z_STATE_QUERY = 0,  /* 上电后先读取一次 Z 状态，避免重复找 Z。 */
  FRONT_DRIVE_Z_STATE_POSITIVE_START, /* 下发 +15% 占空比和占空比模式。 */
  FRONT_DRIVE_Z_STATE_POSITIVE_RUN,   /* 保持 +15% 占空比 1 秒并读取 Z。 */
  FRONT_DRIVE_Z_STATE_NEGATIVE_START, /* 下发 -15% 占空比和占空比模式。 */
  FRONT_DRIVE_Z_STATE_NEGATIVE_RUN,   /* 保持 -15% 占空比 1 秒并读取 Z。 */
  FRONT_DRIVE_Z_STATE_STOPPING,   /* 找到后占空比归零、零速入速度闭环；超时则占空比归零并切空模式。 */
  FRONT_DRIVE_Z_STATE_READY,      /* 已找到 Z，可以进入速度闭环。 */
  FRONT_DRIVE_Z_STATE_FAILED      /* 多次往返仍未找到 Z，保持禁止速度闭环。 */
} FrontDrive_ZState_t;

typedef enum
{
  FRONT_DRIVE_SAFETY_IDLE = 0,              /* 无心跳断档保护流程。 */
  FRONT_DRIVE_SAFETY_PRE_CLEAR,             /* 心跳恢复前先尝试清零旧速度/占空比目标。 */
  FRONT_DRIVE_SAFETY_RESTORE_HEARTBEAT,     /* 发一次心跳让驱动器恢复通信响应。 */
  FRONT_DRIVE_SAFETY_POST_CLEAR,            /* 心跳恢复后再次清零，覆盖被保留的旧目标。 */
  FRONT_DRIVE_SAFETY_WAIT_REARM,            /* 持续心跳和零目标，等待双侧重连且上层确认安全。 */
  FRONT_DRIVE_SAFETY_WAIT_VERIFY            /* 重连清零完成，等待新的双侧零速状态再放行。 */
} FrontDrive_SafetyState_t;

typedef enum
{
  FRONT_DRIVE_SIDE_LEFT = 0,   /* 左前轮电调，ID 由 chassis_config.h 配置。 */
  FRONT_DRIVE_SIDE_RIGHT = 1   /* 右前轮电调，ID 由 chassis_config.h 配置。 */
} FrontDrive_Side_t;

typedef enum
{
  FRONT_DRIVE_COMMAND_SPEED = 0, /* 写目标速度后切入速度闭环模式。 */
  FRONT_DRIVE_COMMAND_BRAKE = 1  /* 写电子刹车电流后切入刹车电流控制模式，严禁用于手刹模式。 */
} FrontDrive_CommandType_t;

typedef struct
{
  BSP_RS485_Bus_t *bus;          /* OID 左/右电调共用的半双工 RS485 总线。 */
  OID_ESC_Handle_t left;         /* 左前轮电调对象，Modbus ID=1。 */
  OID_ESC_Handle_t right;        /* 右前轮电调对象，Modbus ID=2。 */

  uint32_t last_tx_ms;           /* 最近一次成功发帧的 tick，用于控制总线帧间隔。 */
  uint32_t last_status_ms;       /* 最近一次状态轮询 tick，左右轮交替读取。 */
  uint32_t last_control_read_ms;
  uint8_t control_read_enabled; /* Readonly diagnostics, opt-in by application. */
  uint8_t control_read_side;
  uint8_t control_read_allowed; /* Application grants only with centered FC inputs, zero targets and fresh low-speed feedback. */
  uint32_t left_heartbeat_max_gap_ms;
  uint32_t right_heartbeat_max_gap_ms;
  uint8_t heartbeat_sent_mask;
  uint32_t left_speed_tx_ms;
  uint32_t right_speed_tx_ms;
  int32_t left_speed_tx_erpm; /* UART submission evidence, not driver ACK/readback. */
  int32_t right_speed_tx_erpm;
  uint32_t safety_zero_refresh_ms;
  uint32_t left_last_heartbeat_ms;  /* 左电调最近一次心跳发送 tick。 */
  uint32_t right_last_heartbeat_ms; /* 右电调最近一次心跳发送 tick。 */
  uint32_t left_status_request_ms;  /* 左电调最近一次状态请求 tick。 */
  uint32_t right_status_request_ms; /* 右电调最近一次状态请求 tick。 */

  uint8_t heartbeat_side;        /* 心跳轮询方向，0=左，1=右。 */
  uint8_t status_side;           /* 状态轮询方向，0=左，1=右。 */
  uint8_t status_pair_pending;   /* 1=一组配对状态读取还需发送第二侧。 */
  uint8_t status_pair_first_side;/* 下一组配对状态读取的首发侧，逐组交替。 */
  uint8_t left_wait_status;      /* 左电调状态请求等待响应标志。 */
  uint8_t right_wait_status;     /* 右电调状态请求等待响应标志。 */
  uint32_t status_preempt_count; /* Legacy diagnostic field; no RX transaction cancellation. */

  int32_t pending_left_erpm;     /* 待下发的左轮目标 erpm。 */
  int32_t pending_right_erpm;    /* 待下发的右轮目标 erpm。 */
  uint16_t pending_left_brake_10ma;  /* 待下发的左电调电子刹车电流，单位 10mA。 */
  uint16_t pending_right_brake_10ma; /* 待下发的右电调电子刹车电流，单位 10mA。 */
  uint8_t command_type;          /* FrontDrive_CommandType_t，当前挂起命令类型。 */
  uint8_t command_pending;       /* 速度命令挂起标志。 */
  uint8_t command_step;          /* 双电调速度命令分帧步骤。 */
  uint8_t command_urgent;        /* 双零速优先于新查询和普通命令；到期心跳保留时隙。 */
  uint8_t speed_first_side;      /* 速度目标首发侧，完成一组后在左右之间交替。 */
  uint16_t left_control_mode_cache;  /* 最近成功写入左 OID 的控制模式；0xFFFF=未知。 */
  uint16_t right_control_mode_cache; /* 最近成功写入右 OID 的控制模式；0xFFFF=未知。 */

  uint32_t z_phase_start_ms;     /* 当前找 Z 阶段开始 tick。 */
  uint32_t z_last_request_ms;    /* 最近一次 Z 状态读取 tick。 */
  uint8_t z_state;               /* FrontDrive_ZState_t，记录上电找 Z 状态。 */
  uint8_t z_command_step;        /* 找 Z 期间多电调分帧步骤。 */
  uint8_t z_found_stop_step;      /* 某侧先找到 Z 后，单独补发 0% 占空比的分帧步骤。 */
  uint8_t z_cycle_count;         /* +15%/-15% 往返找 Z 的循环次数。 */
  uint8_t left_z_duty_active;     /* 左侧找 Z 占空比是否仍可能有效，需要找到 Z 后及时清零。 */
  uint8_t right_z_duty_active;    /* 右侧找 Z 占空比是否仍可能有效，需要找到 Z 后及时清零。 */
  uint8_t z_fault_rearm_required; /* 1=运行中丢 Z，必须油门回中重找 Z 并完成双侧零速验证。 */

  uint8_t safety_state;          /* FrontDrive_SafetyState_t，心跳断档恢复保护状态。 */
  uint8_t safety_clear_step;     /* 心跳恢复保护分帧步骤。 */
  uint8_t safety_clear_left;     /* 左电调需要清零保护。 */
  uint8_t safety_clear_right;    /* 右电调需要清零保护。 */
  uint8_t safety_rearm_in_progress; /* 1=重连解锁触发的清零序列，结束后还要验双侧零速。 */
} FrontDrive_Handle_t;

void FrontDrive_Init(FrontDrive_Handle_t *drive, BSP_RS485_Bus_t *bus,
                     uint8_t left_id, uint8_t right_id, uint8_t pole_pairs);
void FrontDrive_Task(FrontDrive_Handle_t *drive, uint32_t now_ms);
HAL_StatusTypeDef FrontDrive_SetTargetErpm(FrontDrive_Handle_t *drive, int32_t left_erpm, int32_t right_erpm);
HAL_StatusTypeDef FrontDrive_SetTargetRpm(FrontDrive_Handle_t *drive, int32_t left_rpm, int32_t right_rpm);
HAL_StatusTypeDef FrontDrive_SetBrakeCurrent(FrontDrive_Handle_t *drive,
                                            uint16_t left_current_10ma,
                                            uint16_t right_current_10ma);
HAL_StatusTypeDef FrontDrive_Stop(FrontDrive_Handle_t *drive);
HAL_StatusTypeDef FrontDrive_StopUrgent(FrontDrive_Handle_t *drive);
const OID_ESC_Status_t *FrontDrive_GetStatus(const FrontDrive_Handle_t *drive, FrontDrive_Side_t side);
uint8_t FrontDrive_IsOnline(const FrontDrive_Handle_t *drive, FrontDrive_Side_t side, uint32_t now_ms);
uint8_t FrontDrive_IsZReady(const FrontDrive_Handle_t *drive);
uint8_t FrontDrive_IsMotionReady(const FrontDrive_Handle_t *drive, uint32_t now_ms);
uint8_t FrontDrive_IsSafetyRearmRequired(const FrontDrive_Handle_t *drive);
void FrontDrive_AbortZSearch(FrontDrive_Handle_t *drive, uint32_t now_ms);
void FrontDrive_RetryZSearch(FrontDrive_Handle_t *drive, uint32_t now_ms);
void FrontDrive_RearmSafety(FrontDrive_Handle_t *drive, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FRONT_DRIVE_H */
