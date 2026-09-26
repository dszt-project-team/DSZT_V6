/**
  ******************************************************************************
  * @file    front_drive.c
  * @brief   前轮双 OID 电调组合驱动模块实现。
  ******************************************************************************
  */

#include "front_drive.h"

/* 485 接收帧空闲判定时间。115200bps 下 Modbus 3.5 字符约 0.35ms，2ms 可保证
 * 状态帧先置 frame_ready，再进入下一次发送调度。 */
#define FRONT_DRIVE_FRAME_IDLE_MS       2U
/* 同一条 RS485 总线上两次主机请求之间的最小间隔，避免命令、心跳、状态查询挤在一起。 */
#define FRONT_DRIVE_TX_GAP_MS           8U
/* OID 应用配置里的心跳超时为 1000ms，固件每 100ms 轮询一次 1/2 心跳值。 */
#define FRONT_DRIVE_HEARTBEAT_MS        100U
/* 若本机连续 800ms 未能刷新某侧心跳，恢复心跳前必须先清零旧目标，避免电调沿用断档前指令。 */
#define FRONT_DRIVE_HEARTBEAT_LOST_MS   800U
/* 每 100ms 启动一组左右配对状态读取，正常响应下两侧样本只相差一个帧间隔。 */
#define FRONT_DRIVE_STATUS_PAIR_MS      100U
/* 状态请求发出后超过该时间还没有读响应，就记录一次通信超时。 */
#define FRONT_DRIVE_RESPONSE_TIMEOUT_MS 50U
/* 找 Z 进入 READY 后留出首轮完整状态读取时间；宽限内仍禁止非零目标。 */
#define FRONT_DRIVE_STATUS_STARTUP_GRACE_MS 1000U
/* 连续多个 100ms 配对状态周期没有有效回包，判为真实离线并联动清零两侧。 */
#define FRONT_DRIVE_STATUS_SAFETY_TIMEOUT_MS 500U
/* 重连清零后，两侧实际速度都进入该窗口才允许结束安全重臂。 */
#define FRONT_DRIVE_REARM_MAX_ABS_ERPM 50L
/* Reserve a full read timeout and two heartbeat slots before starting a read. */
#define FRONT_DRIVE_READ_BUDGET_MS (FRONT_DRIVE_RESPONSE_TIMEOUT_MS + 2U * FRONT_DRIVE_TX_GAP_MS)
/* ABZ 编码器上电后需要先找到 Z 信号，15% 占空比正反各保持 1 秒。 */
#define FRONT_DRIVE_Z_DUTY_PERMILLE     150
/* 找 Z 单方向保持时间，单位 ms。 */
#define FRONT_DRIVE_Z_PHASE_MS          1000U
/* 找 Z 期间周期读取 5013，尽快在 Z=1 时把占空比归零。 */
#define FRONT_DRIVE_Z_POLL_MS           200U
/* 最多往返 5 次，仍未找到 Z 则保持禁止速度闭环，避免车辆持续自走。 */
#define FRONT_DRIVE_Z_MAX_CYCLES        5U

static uint8_t FrontDrive_HasPendingStatusResponse(const FrontDrive_Handle_t *drive);
static uint8_t FrontDrive_ArePresentStatusesHealthy(const FrontDrive_Handle_t *drive,
                                                    uint32_t now_ms);
static uint8_t FrontDrive_ArePresentStatusesStopped(const FrontDrive_Handle_t *drive);

static uint8_t FrontDrive_ReadHasHeartbeatBudget(const FrontDrive_Handle_t *drive, uint32_t now)
{
  return (uint8_t)(
      (FRONT_DRIVE_LEFT_PRESENT == 0U ||
       (now - drive->left_last_heartbeat_ms) < FRONT_DRIVE_HEARTBEAT_MS - FRONT_DRIVE_READ_BUDGET_MS) &&
      (FRONT_DRIVE_RIGHT_PRESENT == 0U ||
       (now - drive->right_last_heartbeat_ms) < FRONT_DRIVE_HEARTBEAT_MS - FRONT_DRIVE_READ_BUDGET_MS));
}

static void FrontDrive_RecordHeartbeat(FrontDrive_Handle_t *drive, uint8_t side, uint32_t now)
{
  uint32_t *last = side ? &drive->right_last_heartbeat_ms : &drive->left_last_heartbeat_ms;
  uint32_t *maximum = side ? &drive->right_heartbeat_max_gap_ms : &drive->left_heartbeat_max_gap_ms;
  uint32_t gap = now - *last;
  if ((drive->heartbeat_sent_mask & (1U << side)) != 0U && gap > *maximum) *maximum = gap;
  drive->heartbeat_sent_mask |= (uint8_t)(1U << side);
  *last = now;
}

/**
  * @brief 判断当前是否允许在 OID RS485 总线上发送下一帧。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示可以发送，0 表示需要继续等待帧间隔。
  */
static uint8_t FrontDrive_CanSend(const FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  uint8_t rx_busy = 0U;
  uint32_t primask;

  if (drive == 0)
  {
    return 0U;
  }

  if (drive->bus != 0)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    rx_busy = ((drive->bus->rx_len != 0U) || (drive->bus->frame_ready != 0U)) ? 1U : 0U;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }

  if (rx_busy != 0U)
  {
    return 0U;
  }

  return ((now_ms - drive->last_tx_ms) >= FRONT_DRIVE_TX_GAP_MS) ? 1U : 0U;
}

/**
  * @brief 判断指定侧电调当前是否实际接入。
  * @param side 左/右侧选择。
  * @return 1 表示参与心跳、找 Z、状态轮询和速度控制；0 表示只保留对象和 ID。
  */
static uint8_t FrontDrive_IsSidePresent(FrontDrive_Side_t side)
{
  if (side == FRONT_DRIVE_SIDE_RIGHT)
  {
    return (FRONT_DRIVE_RIGHT_PRESENT != 0U) ? 1U : 0U;
  }

  return (FRONT_DRIVE_LEFT_PRESENT != 0U) ? 1U : 0U;
}

/**
  * @brief 判断所有已接入的电调是否都找到了编码器 Z 信号。
  * @param drive 前轮驱动对象。
  * @return 1 表示已接入电调全部 Z=1；0 表示至少还有一侧未找到。
  */
static uint8_t FrontDrive_AllPresentZFound(const FrontDrive_Handle_t *drive)
{
  if (drive == 0)
  {
    return 0U;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) != 0U) &&
      (drive->left.status.encoder_z_found == 0U))
  {
    return 0U;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) != 0U) &&
      (drive->right.status.encoder_z_found == 0U))
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief 记录找 Z 占空比是否可能仍在某侧电调上生效。
  * @param drive          前轮驱动对象。
  * @param side           左/右侧选择。
  * @param duty_permille  最近成功写入的占空比。
  * @note  找到 Z 的单侧需要尽快写 0%，否则另一侧继续找 Z 时，已找到的一侧也会陪着慢转。
  */
static void FrontDrive_MarkZDutyActive(FrontDrive_Handle_t *drive,
                                      FrontDrive_Side_t side,
                                      int16_t duty_permille)
{
  uint8_t active = (duty_permille != 0) ? 1U : 0U;

  if (drive == 0)
  {
    return;
  }

  if (side == FRONT_DRIVE_SIDE_RIGHT)
  {
    drive->right_z_duty_active = active;
  }
  else
  {
    drive->left_z_duty_active = active;
  }
}

/**
  * @brief 判断某侧是否需要在已找到 Z 后单独补发 0% 占空比。
  * @param drive 前轮驱动对象。
  * @param side  左/右侧选择。
  * @return 1 表示该侧已经找到 Z，且最近找 Z 占空比可能仍未清零。
  */
static uint8_t FrontDrive_NeedStopFoundZSide(const FrontDrive_Handle_t *drive,
                                            FrontDrive_Side_t side)
{
  const OID_ESC_Handle_t *esc;
  uint8_t active;

  if ((drive == 0) || (FrontDrive_IsSidePresent(side) == 0U))
  {
    return 0U;
  }

  esc = (side == FRONT_DRIVE_SIDE_RIGHT) ? &drive->right : &drive->left;
  active = (side == FRONT_DRIVE_SIDE_RIGHT) ?
             drive->right_z_duty_active :
             drive->left_z_duty_active;

  return ((esc != 0) &&
          (esc->status.encoder_z_found != 0U) &&
          (active != 0U)) ? 1U : 0U;
}

/**
  * @brief 切换找 Z 状态，并清零该状态内部的分帧步骤。
  * @param drive  前轮驱动对象。
  * @param state  新状态。
  * @param now_ms 当前系统 tick。
  */
static void FrontDrive_EnterZState(FrontDrive_Handle_t *drive, FrontDrive_ZState_t state, uint32_t now_ms)
{
  if (drive == 0)
  {
    return;
  }

  drive->z_state = (uint8_t)state;
  drive->z_command_step = 0U;
  drive->z_found_stop_step = 0U;
  drive->z_phase_start_ms = now_ms;
  drive->z_last_request_ms = 0U;
  if (state == FRONT_DRIVE_Z_STATE_READY)
  {
    /* 必须在 READY 后重新取得两侧完整状态，不能拿找 Z 前的旧快照放行运动。 */
    drive->left.status.last_update_ms = 0U;
    drive->right.status.last_update_ms = 0U;
  }
}

/**
  * @brief 请求执行心跳断档后的安全清零流程。
  * @param drive       前轮驱动对象。
  * @param clear_left  1 表示清零左侧电调。
  * @param clear_right 1 表示清零右侧电调。
  * @note  清零流程会丢弃挂起的非零速度命令，避免心跳恢复后旧目标继续生效。
  */
static void FrontDrive_RequestSafetyClear(FrontDrive_Handle_t *drive, uint8_t clear_left, uint8_t clear_right)
{
  if ((drive == 0) || ((clear_left == 0U) && (clear_right == 0U)))
  {
    return;
  }

  /* 双驱车辆任一侧心跳或编码器安全异常时必须联动清零，禁止另一侧维持旧目标。 */
  clear_left = FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT);
  clear_right = FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT);

  drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_PRE_CLEAR;
  drive->safety_clear_step = 0U;
  drive->safety_clear_left = (clear_left != 0U) ? 1U : drive->safety_clear_left;
  drive->safety_clear_right = (clear_right != 0U) ? 1U : drive->safety_clear_right;
  drive->pending_left_erpm = 0;
  drive->pending_right_erpm = 0;
  drive->pending_left_brake_10ma = 0U;
  drive->pending_right_brake_10ma = 0U;
  drive->command_type = (uint8_t)FRONT_DRIVE_COMMAND_SPEED;
  drive->command_pending = 0U;
  drive->command_step = 0U;
  drive->command_urgent = 0U;
  drive->status_pair_pending = 0U;
  drive->safety_rearm_in_progress = 0U;
}

/**
  * @brief 检查心跳是否发生异常断档。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @note  检测在心跳发送前执行，一旦触发，下一帧优先清零而不是恢复心跳。
  */
static void FrontDrive_CheckHeartbeatLoss(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  uint8_t clear_left = 0U;
  uint8_t clear_right = 0U;

  if (drive == 0)
  {
    return;
  }

  /*
   * 上电找 Z 期间 FrontDrive_Task 会优先跑占空比找 Z 流程，普通心跳还没有开始发送。
   * 这时不能用心跳断档保护去重置状态机，否则双电调找 Z 超过 800ms 时会反复回到 +15% 阶段，
   * 表现为电机一直慢速转动且迟迟无法进入速度闭环。
   */
  if (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_READY)
  {
    return;
  }

  if (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE)
  {
    return;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) != 0U) &&
      (drive->left_last_heartbeat_ms != 0U) &&
      ((now_ms - drive->left_last_heartbeat_ms) >= FRONT_DRIVE_HEARTBEAT_LOST_MS))
  {
    clear_left = 1U;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) != 0U) &&
      (drive->right_last_heartbeat_ms != 0U) &&
      ((now_ms - drive->right_last_heartbeat_ms) >= FRONT_DRIVE_HEARTBEAT_LOST_MS))
  {
    clear_right = 1U;
  }

  FrontDrive_RequestSafetyClear(drive, clear_left, clear_right);
}

/** 任一已接入 OID 的完整状态必须新鲜且无故障，才能作为双驱运动反馈。 */
static uint8_t FrontDrive_ArePresentStatusesHealthy(const FrontDrive_Handle_t *drive,
                                                    uint32_t now_ms)
{
  const OID_ESC_Status_t *status;

  if (drive == 0)
  {
    return 0U;
  }
  if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) != 0U)
  {
    status = &drive->left.status;
    if ((status->last_update_ms == 0U) ||
        ((now_ms - status->last_update_ms) > FRONT_DRIVE_STATUS_SAFETY_TIMEOUT_MS) ||
        (status->fault != 0U))
    {
      return 0U;
    }
  }
  if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) != 0U)
  {
    status = &drive->right.status;
    if ((status->last_update_ms == 0U) ||
        ((now_ms - status->last_update_ms) > FRONT_DRIVE_STATUS_SAFETY_TIMEOUT_MS) ||
        (status->fault != 0U))
    {
      return 0U;
    }
  }
  return 1U;
}

/** 重连安全序列后必须从新状态帧确认两侧都已经接近静止。 */
static uint8_t FrontDrive_ArePresentStatusesStopped(const FrontDrive_Handle_t *drive)
{
  if (drive == 0)
  {
    return 0U;
  }
  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) != 0U) &&
      ((drive->left.status.speed_erpm < -FRONT_DRIVE_REARM_MAX_ABS_ERPM) ||
       (drive->left.status.speed_erpm > FRONT_DRIVE_REARM_MAX_ABS_ERPM)))
  {
    return 0U;
  }
  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) != 0U) &&
      ((drive->right.status.speed_erpm < -FRONT_DRIVE_REARM_MAX_ABS_ERPM) ||
       (drive->right.status.speed_erpm > FRONT_DRIVE_REARM_MAX_ABS_ERPM)))
  {
    return 0U;
  }
  return 1U;
}

/** READY 后若任一侧连续多个状态周期无有效回包或报告故障，联动清零两侧并锁存重臂。 */
static void FrontDrive_CheckStatusHealth(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if ((drive == 0) ||
      (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_READY) ||
      (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE))
  {
    return;
  }
  if (drive->z_phase_start_ms == 0U)
  {
    drive->z_phase_start_ms = now_ms;
    return;
  }
  if ((now_ms - drive->z_phase_start_ms) < FRONT_DRIVE_STATUS_STARTUP_GRACE_MS)
  {
    return;
  }
  if (FrontDrive_ArePresentStatusesHealthy(drive, now_ms) == 0U)
  {
    FrontDrive_RequestSafetyClear(drive, 1U, 1U);
  }
}

/**
  * @brief 按左右侧获取 OID 电调对象。
  * @param drive 前轮驱动对象。
  * @param side  左/右侧选择。
  * @return 对应电调对象；参数非法时返回空指针。
  */
static OID_ESC_Handle_t *FrontDrive_GetEsc(FrontDrive_Handle_t *drive, FrontDrive_Side_t side)
{
  if (drive == 0)
  {
    return 0;
  }

  if (side == FRONT_DRIVE_SIDE_LEFT)
  {
    return &drive->left;
  }
  if (side == FRONT_DRIVE_SIDE_RIGHT)
  {
    return &drive->right;
  }
  return 0;
}

/**
  * @brief 按左右侧获取只读 OID 电调对象。
  * @param drive 前轮驱动对象。
  * @param side  左/右侧选择。
  * @return 对应电调对象；参数非法时返回空指针。
  */
static const OID_ESC_Handle_t *FrontDrive_GetEscConst(const FrontDrive_Handle_t *drive, FrontDrive_Side_t side)
{
  if (drive == 0)
  {
    return 0;
  }

  if (side == FRONT_DRIVE_SIDE_LEFT)
  {
    return &drive->left;
  }
  if (side == FRONT_DRIVE_SIDE_RIGHT)
  {
    return &drive->right;
  }
  return 0;
}

static uint16_t FrontDrive_GetCachedMode(const FrontDrive_Handle_t *drive,
                                        FrontDrive_Side_t side)
{
  if (drive == 0)
  {
    return (uint16_t)OID_ESC_MODE_IDLE;
  }
  return (side == FRONT_DRIVE_SIDE_RIGHT) ?
    drive->right_control_mode_cache : drive->left_control_mode_cache;
}

static void FrontDrive_SetCachedMode(FrontDrive_Handle_t *drive,
                                    FrontDrive_Side_t side,
                                    OID_ESC_Mode_t mode)
{
  if (drive == 0)
  {
    return;
  }
  if (side == FRONT_DRIVE_SIDE_RIGHT)
  {
    drive->right_control_mode_cache = (uint16_t)mode;
  }
  else
  {
    drive->left_control_mode_cache = (uint16_t)mode;
  }
}

static uint8_t FrontDrive_IsSpeedModeReady(const FrontDrive_Handle_t *drive,
                                          FrontDrive_Side_t side)
{
  if (FrontDrive_IsSidePresent(side) == 0U)
  {
    return 1U;
  }
  return (FrontDrive_GetCachedMode(drive, side) ==
          (uint16_t)OID_ESC_MODE_SPEED) ? 1U : 0U;
}

/* An empty RX buffer does NOT prove a slave will not reply later. Finish the
 * bounded transaction before sending another request, including urgent zero. */
static uint8_t FrontDrive_PreemptStatusWait(FrontDrive_Handle_t *drive)
{
  return (uint8_t)(FrontDrive_HasPendingStatusResponse(drive) == 0U);
}

/**
  * @brief 检查状态请求是否超时。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @note  左右两台电调均已接入。若某侧超时，可据此排查总线、ID 和线序问题。
  */
static void FrontDrive_CheckStatusTimeout(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if (drive == 0)
  {
    return;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) != 0U) &&
      (drive->left_wait_status != 0U) &&
      ((now_ms - drive->left_status_request_ms) >= FRONT_DRIVE_RESPONSE_TIMEOUT_MS))
  {
    drive->left.status.timeout_count++;
    drive->left_wait_status = 0U;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) != 0U) &&
      (drive->right_wait_status != 0U) &&
      ((now_ms - drive->right_status_request_ms) >= FRONT_DRIVE_RESPONSE_TIMEOUT_MS))
  {
    drive->right.status.timeout_count++;
    drive->right_wait_status = 0U;
  }
}

/**
  * @brief 判断是否仍在等待某侧电调的状态读响应。
  * @param drive 前轮驱动对象。
  * @return 1 表示状态读请求尚未闭环，0 表示总线可以继续排下一类请求。
  * @note  OID 使用半双工 Modbus RTU，同一时刻只能有一个主站请求在途。
  *        如果状态读请求后立刻插入心跳或速度写入，慢返回的一侧可能被下一次发送清掉，
  *        日志上表现为某一侧 timeout 持续累加、online 偶尔掉线。
  */
static uint8_t FrontDrive_HasPendingStatusResponse(const FrontDrive_Handle_t *drive)
{
  if (drive == 0)
  {
    return 0U;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) != 0U) &&
      (drive->left_wait_status != 0U))
  {
    return 1U;
  }

  if ((FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) != 0U) &&
      (drive->right_wait_status != 0U))
  {
    return 1U;
  }

  return 0U;
}

/**
  * @brief 状态帧报告“未找到 Z”时，将该侧重新拉回找 Z 流程。
  * @param drive  前轮驱动对象。
  * @param side   左/右侧选择。
  * @param now_ms 当前系统 tick。
  * @note  OID 断电、重启或内部丢失 ABZ Z 状态后，之前缓存的 encoder_z_found 不能继续作为速度闭环依据。
  *        一旦读到故障 19，先联动清零双侧并锁存在 FAILED/WAIT_REARM；只有油门回中后才允许重新找 Z。
  */
static void FrontDrive_HandleEncoderZFault(FrontDrive_Handle_t *drive,
                                          FrontDrive_Side_t side,
                                          uint32_t now_ms)
{
  OID_ESC_Handle_t *esc;
  uint8_t clear_left = 0U;
  uint8_t clear_right = 0U;

  if ((drive == 0) || (FrontDrive_IsSidePresent(side) == 0U))
  {
    return;
  }

  esc = FrontDrive_GetEsc(drive, side);
  if ((esc == 0) || (esc->status.fault != OID_ESC_FAULT_ENCODER_Z_NOT_FOUND))
  {
    return;
  }

  /*
   * 故障 19 的状态帧会周期性刷新；第一次触发后 z_state 已不再是 READY，
   * 这里直接返回，避免反复把安全清零步骤重置到第 0 帧。
   */
  if ((esc->status.encoder_z_found == 0U) &&
      (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_READY))
  {
    return;
  }

  esc->status.encoder_z_found = 0U;
  esc->status.last_z_update_ms = 0U;
  FrontDrive_SetCachedMode(drive, side, OID_ESC_MODE_IDLE);

  if (side == FRONT_DRIVE_SIDE_RIGHT)
  {
    drive->right_z_duty_active = 0U;
    clear_right = 1U;
  }
  else
  {
    drive->left_z_duty_active = 0U;
    clear_left = 1U;
  }

  drive->z_cycle_count = 0U;
  drive->z_fault_rearm_required = 1U;
  FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_FAILED, now_ms);
  FrontDrive_RequestSafetyClear(drive, clear_left, clear_right);
}

/** 仅在锁车低速的合法新回读确认模式异常时，进入既有清零重臂。 */
static void FrontDrive_CheckControlModeReadback(FrontDrive_Handle_t *drive,
                                               const OID_ESC_Handle_t *esc,
                                               uint32_t previous_reads)
{
  /* 只使用本次合法且未超时的请求回包；旧诊断快照不能触发恢复。
   * 应用仅在锁车、双侧反馈新鲜且低速时授予 control_read_allowed。
   * 若等待回包期间已经解锁，则不在线改写模式，留到下次安全回读处理。 */
  if ((esc->diagnostic.control_reads == previous_reads) ||
      (drive->control_read_allowed == 0U) ||
      (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE) ||
      (esc->diagnostic.control_mode == (uint16_t)OID_ESC_MODE_SPEED))
  {
    return;
  }

  /* UART 发出模式帧不等于驱动器已保持该模式。新回读与预期速度模式
   * 不符时，先废弃缓存及挂起目标，再走已有双侧清零/心跳/回中重臂，
   * 不直接恢复原非零速度，也不增加运动期间的诊断轮询。 */
  drive->left_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
  drive->right_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
  FrontDrive_RequestSafetyClear(drive, 1U, 1U);
}

/**
  * @brief 解析 RS485 总线上收到的一帧，并清除对应电调的状态等待标志。
  * @param drive  前轮驱动对象。
  * @param frame  收到的完整 Modbus 帧。
  * @param len    帧长度。
  * @param now_ms 当前系统 tick。
  */
static void FrontDrive_HandleRxFrame(FrontDrive_Handle_t *drive, const uint8_t *frame, uint16_t len, uint32_t now_ms)
{
  uint32_t before_update;
  uint32_t before_control_reads;

  if ((drive == 0) || (frame == 0))
  {
    return;
  }

  /* Unsolicited/expired status replies must not count as fresh stop evidence. */
  if (len >= 2U && frame[1] == 0x04U &&
      ((frame[0] == drive->left.id && drive->left_wait_status == 0U) ||
       (frame[0] == drive->right.id && drive->right_wait_status == 0U))) return;

  before_update = drive->left.status.last_update_ms;
  before_control_reads = drive->left.diagnostic.control_reads;
  if (OID_ESC_HandleFrame(&drive->left, frame, len, now_ms) != 0U)
  {
    FrontDrive_CheckControlModeReadback(drive, &drive->left, before_control_reads);
    if (drive->left.status.last_update_ms != before_update)
    {
      drive->left_wait_status = 0U;
      if (FRONT_DRIVE_AUTO_Z_SEARCH != 0U)
      {
        FrontDrive_HandleEncoderZFault(drive, FRONT_DRIVE_SIDE_LEFT, now_ms);
      }
    }
    return;
  }

  before_update = drive->right.status.last_update_ms;
  before_control_reads = drive->right.diagnostic.control_reads;
  if (OID_ESC_HandleFrame(&drive->right, frame, len, now_ms) != 0U)
  {
    FrontDrive_CheckControlModeReadback(drive, &drive->right, before_control_reads);
    if (drive->right.status.last_update_ms != before_update)
    {
      drive->right_wait_status = 0U;
      if (FRONT_DRIVE_AUTO_Z_SEARCH != 0U)
      {
        FrontDrive_HandleEncoderZFault(drive, FRONT_DRIVE_SIDE_RIGHT, now_ms);
      }
    }
  }
}

/**
  * @brief 发送一帧后统一更新时间戳。
  * @param drive  前轮驱动对象。
  * @param status HAL 发送结果。
  * @param now_ms 当前系统 tick。
  * @return 1 表示本次确实成功占用了总线，0 表示没有成功发送。
  */
static uint8_t FrontDrive_MarkTxResult(FrontDrive_Handle_t *drive, HAL_StatusTypeDef status, uint32_t now_ms)
{
  if ((drive != 0) && (status == HAL_OK))
  {
    drive->last_tx_ms = now_ms;
    return 1U;
  }

  return 0U;
}

static HAL_StatusTypeDef FrontDrive_WriteSpeed(FrontDrive_Handle_t *drive,
    FrontDrive_Side_t side, int32_t value, uint32_t now)
{
  HAL_StatusTypeDef result = OID_ESC_SetSpeedErpm(drive->bus, FrontDrive_GetEsc(drive, side), value);
  if (result == HAL_OK)
  {
    if (side == FRONT_DRIVE_SIDE_LEFT)
    { drive->left_speed_tx_erpm = value; drive->left_speed_tx_ms = now; }
    else
    { drive->right_speed_tx_erpm = value; drive->right_speed_tx_ms = now; }
  }
  return result;
}

/**
  * @brief 心跳断档保护中的清零分帧步骤。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已经发送一帧或正在等待帧间隔；0 表示本轮清零帧已全部排完。
  * @note  每侧依次写 0 速度、0 占空比，再按 Z 状态进入速度零锁或占空比零输出。
  */
static uint8_t FrontDrive_SendSafetyClearStep(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;
  uint8_t side_selected;
  uint8_t sub_step;

  if (drive == 0)
  {
    return 0U;
  }

  while (drive->safety_clear_step < 6U)
  {
    if (drive->safety_clear_step < 3U)
    {
      side = FRONT_DRIVE_SIDE_LEFT;
      side_selected = drive->safety_clear_left;
      sub_step = drive->safety_clear_step;
    }
    else
    {
      side = FRONT_DRIVE_SIDE_RIGHT;
      side_selected = drive->safety_clear_right;
      sub_step = (uint8_t)(drive->safety_clear_step - 3U);
    }

    if ((side_selected == 0U) || (FrontDrive_IsSidePresent(side) == 0U))
    {
      drive->safety_clear_step += (sub_step == 0U) ? 3U : 1U;
      continue;
    }

    if (FrontDrive_CanSend(drive, now_ms) == 0U)
    {
      return 1U;
    }

    esc = FrontDrive_GetEsc(drive, side);
    if (sub_step == 0U)
    {
      status = FrontDrive_WriteSpeed(drive, side, 0, now_ms);
    }
    else if (sub_step == 1U)
    {
      status = OID_ESC_SetDutyPermille(drive->bus, esc, 0);
    }
    else if (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY)
    {
      status = OID_ESC_SetControlMode(drive->bus, esc, OID_ESC_MODE_SPEED);
    }
    else
    {
      status = OID_ESC_SetControlMode(drive->bus, esc, OID_ESC_MODE_DUTY);
    }

    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      if (sub_step == 2U)
      {
        FrontDrive_SetCachedMode(
          drive,
          side,
          (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY) ?
            OID_ESC_MODE_SPEED : OID_ESC_MODE_DUTY);
      }
      drive->safety_clear_step++;
    }
    return 1U;
  }

  drive->safety_clear_step = 0U;
  return 0U;
}

/**
  * @brief 心跳断档保护中的心跳恢复分帧步骤。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已经发送一帧或正在等待帧间隔；0 表示本轮心跳帧已全部排完。
  */
static uint8_t FrontDrive_SendSafetyHeartbeatStep(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;
  uint8_t side_selected;

  if (drive == 0)
  {
    return 0U;
  }

  while (drive->safety_clear_step < 2U)
  {
    if (drive->safety_clear_step == 0U)
    {
      side = FRONT_DRIVE_SIDE_LEFT;
      side_selected = drive->safety_clear_left;
    }
    else
    {
      side = FRONT_DRIVE_SIDE_RIGHT;
      side_selected = drive->safety_clear_right;
    }

    if ((side_selected == 0U) || (FrontDrive_IsSidePresent(side) == 0U))
    {
      drive->safety_clear_step++;
      continue;
    }

    if (FrontDrive_CanSend(drive, now_ms) == 0U)
    {
      return 1U;
    }

    esc = FrontDrive_GetEsc(drive, side);
    status = OID_ESC_SendHeartbeat(drive->bus, esc);
    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      if (side == FRONT_DRIVE_SIDE_LEFT)
      {
        FrontDrive_RecordHeartbeat(drive, 0U, now_ms);
      }
      else
      {
        FrontDrive_RecordHeartbeat(drive, 1U, now_ms);
      }
      drive->safety_clear_step++;
    }
    return 1U;
  }

  drive->safety_clear_step = 0U;
  return 0U;
}

/**
  * @brief 心跳断档恢复保护主流程。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示保护流程正在占用总线优先级；0 表示无保护流程或保护已进入等待解锁态。
  * @note  流程为：恢复心跳前清零 -> 恢复心跳 -> 恢复心跳后再清零；若由重连解锁触发，
  *        还必须读取新的双侧零速状态后才能回到正常运动态。
  */
static uint8_t FrontDrive_ServiceSafetyClear(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if (drive == 0)
  {
    return 0U;
  }

  switch ((FrontDrive_SafetyState_t)drive->safety_state)
  {
    case FRONT_DRIVE_SAFETY_PRE_CLEAR:
      if (FrontDrive_SendSafetyClearStep(drive, now_ms) != 0U)
      {
        return 1U;
      }
      drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_RESTORE_HEARTBEAT;
      return 1U;

    case FRONT_DRIVE_SAFETY_RESTORE_HEARTBEAT:
      if (FrontDrive_SendSafetyHeartbeatStep(drive, now_ms) != 0U)
      {
        return 1U;
      }
      drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_POST_CLEAR;
      return 1U;

    case FRONT_DRIVE_SAFETY_POST_CLEAR:
      if (FrontDrive_SendSafetyClearStep(drive, now_ms) != 0U)
      {
        return 1U;
      }

      drive->safety_clear_left = 0U;
      drive->safety_clear_right = 0U;
      if ((drive->z_fault_rearm_required != 0U) &&
          (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_READY))
      {
        /*
         * 运行中丢 Z 后，只要尚未重新 READY，任何安全清零收尾都必须回到
         * FAILED/WAIT_REARM。即使状态帧短暂恢复 fault=0，也不能借普通重臂路径
         * 自动进入 QUERY/IDLE，必须等上层确认 CH3 回中后显式重试。
         */
        drive->safety_rearm_in_progress = 0U;
        drive->z_cycle_count = 0U;
        FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_FAILED, now_ms);
        drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM;
      }
      else if (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY)
      {
        if (drive->safety_rearm_in_progress != 0U)
        {
          /* 重连侧可能漏掉上一轮清零；本轮完整序列后强制等新的双侧零速状态。 */
          drive->safety_rearm_in_progress = 0U;
          drive->left.status.last_update_ms = 0U;
          drive->right.status.last_update_ms = 0U;
          drive->left_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
          drive->right_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
          drive->last_status_ms = now_ms - FRONT_DRIVE_STATUS_PAIR_MS;
          drive->status_pair_pending = 0U;
          drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_WAIT_VERIFY;
        }
        else
        {
          drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM;
        }
      }
      else
      {
        drive->safety_rearm_in_progress = 0U;
        FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_QUERY, now_ms);
        drive->z_cycle_count = 0U;
        drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_IDLE;
      }
      return 0U;

    default:
      return 0U;
  }
}

/**
  * @brief 找 Z 阶段按左右已接入电调分帧读取 5013。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已发送一帧或正在等待帧间隔；0 表示本轮所有读取帧已排完。
  */
static uint8_t FrontDrive_SendZRequestStep(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;

  if (drive == 0)
  {
    return 0U;
  }

  while (drive->z_command_step < 2U)
  {
    side = (drive->z_command_step == 0U) ? FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
    if (FrontDrive_IsSidePresent(side) == 0U)
    {
      drive->z_command_step++;
      continue;
    }

    if (FrontDrive_CanSend(drive, now_ms) == 0U)
    {
      return 1U;
    }

    esc = FrontDrive_GetEsc(drive, side);
    status = OID_ESC_RequestZSignal(drive->bus, esc);
    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      drive->z_command_step++;
    }
    return 1U;
  }

  drive->z_command_step = 0U;
  drive->z_last_request_ms = now_ms;
  return 0U;
}

/**
  * @brief 找 Z 阶段按左右已接入电调分帧设置占空比，并按需切入占空比模式。
  * @param drive          前轮驱动对象。
  * @param now_ms         当前系统 tick。
  * @param duty_permille  目标占空比，100=10%，-100=-10%，0=停止占空比输出。
  * @param enter_duty_mode 是否同时切入占空比控制模式。
  * @return 1 表示已发送一帧或正在等待帧间隔；0 表示本轮所有设置帧已排完。
  */
static uint8_t FrontDrive_SendDutyStep(FrontDrive_Handle_t *drive,
                                      uint32_t now_ms,
                                      int16_t duty_permille,
                                      uint8_t enter_duty_mode)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;
  uint8_t sub_step;

  if (drive == 0)
  {
    return 0U;
  }

  while (drive->z_command_step < 4U)
  {
    side = (drive->z_command_step < 2U) ? FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
    sub_step = (uint8_t)(drive->z_command_step & 0x01U);
    esc = FrontDrive_GetEsc(drive, side);

    if ((FrontDrive_IsSidePresent(side) == 0U) ||
        ((esc != 0) && (esc->status.encoder_z_found != 0U)) ||
        ((sub_step == 1U) && (enter_duty_mode == 0U)))
    {
      if ((sub_step == 0U) && (esc != 0) && (esc->status.encoder_z_found != 0U))
      {
        FrontDrive_MarkZDutyActive(drive, side, 0);
      }
      drive->z_command_step++;
      continue;
    }

    if (FrontDrive_CanSend(drive, now_ms) == 0U)
    {
      return 1U;
    }

    if (sub_step == 0U)
    {
      status = OID_ESC_SetDutyPermille(drive->bus, esc, duty_permille);
    }
    else
    {
      status = OID_ESC_SetControlMode(drive->bus, esc, OID_ESC_MODE_DUTY);
    }

    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      if (sub_step == 0U)
      {
        FrontDrive_MarkZDutyActive(drive, side, duty_permille);
      }
      else
      {
        FrontDrive_SetCachedMode(drive, side, OID_ESC_MODE_DUTY);
      }
      drive->z_command_step++;
    }
    return 1U;
  }

  drive->z_command_step = 0U;
  return 0U;
}

/**
  * @brief 某一侧先找到 Z 后，只给这一侧补发 0% 占空比。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已发送一帧或正在等待帧间隔；0 表示没有需要单独清零的侧。
  * @note  另一侧仍可继续找 Z，但已找到的一侧不再被找 Z 占空比拖着慢转。
  */
static uint8_t FrontDrive_StopFoundZSideStep(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;

  if (drive == 0)
  {
    return 0U;
  }

  while (drive->z_found_stop_step < 2U)
  {
    side = (drive->z_found_stop_step == 0U) ? FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
    if (FrontDrive_NeedStopFoundZSide(drive, side) == 0U)
    {
      drive->z_found_stop_step++;
      continue;
    }

    if (FrontDrive_CanSend(drive, now_ms) == 0U)
    {
      return 1U;
    }

    esc = FrontDrive_GetEsc(drive, side);
    status = OID_ESC_SetDutyPermille(drive->bus, esc, 0);
    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      FrontDrive_MarkZDutyActive(drive, side, 0);
      drive->z_found_stop_step++;
    }
    return 1U;
  }

  drive->z_found_stop_step = 0U;
  return 0U;
}

/**
  * @brief 找 Z 结束时按左右已接入电调分帧执行安全收尾。
  * @param drive            前轮驱动对象。
  * @param now_ms           当前系统 tick。
  * @param enter_speed_mode 1=Z 已找到，写 0 占空比后再写 0 速度并进入速度闭环；0=只停占空比。
  * @return 1 表示已发送一帧或正在等待帧间隔；0 表示停止帧已全部排完。
  */
static uint8_t FrontDrive_SendZStopStep(FrontDrive_Handle_t *drive, uint32_t now_ms, uint8_t enter_speed_mode)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;
  uint8_t sub_step;
  uint8_t max_step;

  if (drive == 0)
  {
    return 0U;
  }

  max_step = (enter_speed_mode != 0U) ? 6U : 2U;
  while (drive->z_command_step < max_step)
  {
    if (enter_speed_mode != 0U)
    {
      side = (drive->z_command_step < 3U) ? FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
      sub_step = (uint8_t)(drive->z_command_step % 3U);
    }
    else
    {
      side = (drive->z_command_step == 0U) ? FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
      sub_step = 0U;
    }

    if (FrontDrive_IsSidePresent(side) == 0U)
    {
      drive->z_command_step += (enter_speed_mode != 0U) ? 3U : 1U;
      continue;
    }

    if (FrontDrive_CanSend(drive, now_ms) == 0U)
    {
      return 1U;
    }

    esc = FrontDrive_GetEsc(drive, side);
    if (sub_step == 0U)
    {
      status = OID_ESC_SetDutyPermille(drive->bus, esc, 0);
    }
    else if ((enter_speed_mode != 0U) && (sub_step == 1U))
    {
      status = FrontDrive_WriteSpeed(drive, side, 0, now_ms);
    }
    else
    {
      status = OID_ESC_SetControlMode(drive->bus, esc, OID_ESC_MODE_SPEED);
    }

    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      if (sub_step == 0U)
      {
        FrontDrive_MarkZDutyActive(drive, side, 0);
      }
      else if (sub_step == 2U)
      {
        FrontDrive_SetCachedMode(drive, side, OID_ESC_MODE_SPEED);
      }
      drive->z_command_step++;
    }
    return 1U;
  }

  drive->z_command_step = 0U;
  return 0U;
}

/**
  * @brief 上电后自动找 ABZ 编码器 Z 信号。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示找 Z 流程仍占用控制优先级；0 表示已经 ready，可执行速度命令。
  * @note  流程为：读取 Z -> +15% 1 秒 -> -15% 1 秒，期间持续读 5013；找到后立刻占空比归零。
  */
static uint8_t FrontDrive_ServiceZFinder(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if (drive == 0)
  {
    return 0U;
  }

  if (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY)
  {
    return 0U;
  }

  if (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_FAILED)
  {
    return 1U;
  }

  if ((FrontDrive_AllPresentZFound(drive) != 0U) &&
      (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_STOPPING))
  {
    FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_STOPPING, now_ms);
  }
  else if (FrontDrive_StopFoundZSideStep(drive, now_ms) != 0U)
  {
    return 1U;
  }

  switch ((FrontDrive_ZState_t)drive->z_state)
  {
    case FRONT_DRIVE_Z_STATE_QUERY:
      if (FrontDrive_SendZRequestStep(drive, now_ms) != 0U)
      {
        return 1U;
      }
      FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_POSITIVE_START, now_ms);
      return 1U;

    case FRONT_DRIVE_Z_STATE_POSITIVE_START:
      if (FrontDrive_SendDutyStep(drive, now_ms, FRONT_DRIVE_Z_DUTY_PERMILLE, 1U) != 0U)
      {
        return 1U;
      }
      FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_POSITIVE_RUN, now_ms);
      return 1U;

    case FRONT_DRIVE_Z_STATE_POSITIVE_RUN:
      if ((now_ms - drive->z_phase_start_ms) >= FRONT_DRIVE_Z_PHASE_MS)
      {
        FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_NEGATIVE_START, now_ms);
        return 1U;
      }
      if ((now_ms - drive->z_last_request_ms) >= FRONT_DRIVE_Z_POLL_MS)
      {
        (void)FrontDrive_SendZRequestStep(drive, now_ms);
      }
      return 1U;

    case FRONT_DRIVE_Z_STATE_NEGATIVE_START:
      if (FrontDrive_SendDutyStep(drive, now_ms, -FRONT_DRIVE_Z_DUTY_PERMILLE, 0U) != 0U)
      {
        return 1U;
      }
      FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_NEGATIVE_RUN, now_ms);
      return 1U;

    case FRONT_DRIVE_Z_STATE_NEGATIVE_RUN:
      if ((now_ms - drive->z_phase_start_ms) >= FRONT_DRIVE_Z_PHASE_MS)
      {
        drive->z_cycle_count++;
        if (drive->z_cycle_count >= FRONT_DRIVE_Z_MAX_CYCLES)
        {
          FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_STOPPING, now_ms);
        }
        else
        {
          FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_POSITIVE_START, now_ms);
        }
        return 1U;
      }
      if ((now_ms - drive->z_last_request_ms) >= FRONT_DRIVE_Z_POLL_MS)
      {
        (void)FrontDrive_SendZRequestStep(drive, now_ms);
      }
      return 1U;

    case FRONT_DRIVE_Z_STATE_STOPPING:
      if (FrontDrive_SendZStopStep(drive, now_ms, FrontDrive_AllPresentZFound(drive)) != 0U)
      {
        return 1U;
      }
      if (FrontDrive_AllPresentZFound(drive) != 0U)
      {
        FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_READY, now_ms);
      }
      else
      {
        FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_FAILED, now_ms);
      }
      return 1U;

    default:
      return 1U;
  }
}

/**
  * @brief 执行挂起的左右轮独立速度命令。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已经发送一帧或正在等待帧间隔；0 表示没有挂起命令。
  * @note  同一条 RS485 总线不能同时给两个电调发帧。已确认两侧均在速度模式时只发送
  *        两帧目标；模式未知时先双侧安全清零并切回速度模式，再发送同一代目标。
  */
static uint8_t FrontDrive_ProcessPendingCommand(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t side;
  int32_t target_erpm;

  if ((drive == 0) || (drive->command_pending == 0U))
  {
    return 0U;
  }

  if (FrontDrive_CanSend(drive, now_ms) == 0U)
  {
    return 1U;
  }

  if (drive->command_type == (uint8_t)FRONT_DRIVE_COMMAND_BRAKE)
  {
    switch (drive->command_step)
    {
      case 0U:
        if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) == 0U)
        {
          drive->command_step = 2U;
          break;
        }
        status = OID_ESC_SetBrakeCurrent(drive->bus,
                                         &drive->left,
                                         drive->pending_left_brake_10ma);
        if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
        {
          drive->command_step = 1U;
        }
        break;

      case 1U:
        if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) == 0U)
        {
          drive->command_step = 2U;
          break;
        }
        /*
         * OID 电子刹车模式为控制模式 6。模式 7 是厂家明确禁止应用中使用的手刹，
         * 这里不提供任何进入手刹模式的路径。
         */
        status = OID_ESC_SetControlMode(drive->bus, &drive->left, OID_ESC_MODE_BRAKE);
        if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
        {
          FrontDrive_SetCachedMode(drive, FRONT_DRIVE_SIDE_LEFT, OID_ESC_MODE_BRAKE);
          drive->command_step = 2U;
        }
        break;

      case 2U:
        if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) == 0U)
        {
          drive->command_step = 0U;
          drive->command_pending = 0U;
          break;
        }
        status = OID_ESC_SetBrakeCurrent(drive->bus,
                                         &drive->right,
                                         drive->pending_right_brake_10ma);
        if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
        {
          drive->command_step = 3U;
        }
        break;

      default:
        if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) == 0U)
        {
          drive->command_step = 0U;
          drive->command_pending = 0U;
          break;
        }
        status = OID_ESC_SetControlMode(drive->bus, &drive->right, OID_ESC_MODE_BRAKE);
        if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
        {
          FrontDrive_SetCachedMode(drive, FRONT_DRIVE_SIDE_RIGHT, OID_ESC_MODE_BRAKE);
          drive->command_step = 0U;
          drive->command_pending = 0U;
          drive->command_urgent = 0U;
        }
        break;
    }

    return 1U;
  }

  /*
   * 模式未知或刚从占空比/刹车模式退出时，先双侧写 0，再切速度模式，
   * 最后才发送本代非零目标，避免恢复 OID 寄存器中残留的旧速度。
   */
  while (drive->command_step < 4U)
  {
    side = ((drive->command_step == 0U) || (drive->command_step == 2U)) ?
      FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
    if (FrontDrive_IsSidePresent(side) == 0U)
    {
      drive->command_step++;
      continue;
    }

    esc = FrontDrive_GetEsc(drive, side);
    if (drive->command_step < 2U)
    {
      /* 只要任一侧模式未知，两侧都先明确写 0，不能让已在速度模式的一侧保留旧目标。 */
      status = FrontDrive_WriteSpeed(drive, side, 0, now_ms);
    }
    else
    {
      if (FrontDrive_IsSpeedModeReady(drive, side) != 0U)
      {
        drive->command_step++;
        continue;
      }
      status = OID_ESC_SetControlMode(drive->bus, esc, OID_ESC_MODE_SPEED);
    }
    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      if (drive->command_step >= 2U)
      {
        FrontDrive_SetCachedMode(drive, side, OID_ESC_MODE_SPEED);
      }
      drive->command_step++;
      if ((drive->command_step >= 4U) &&
          (drive->pending_left_erpm == 0) &&
          (drive->pending_right_erpm == 0))
      {
        drive->command_step = 0U;
        drive->command_pending = 0U;
        drive->command_urgent = 0U;
      }
    }
    return 1U;
  }

  /* 缓存若被异步失效，回到安全建模步骤，不能在模式未知时直接写非零目标。 */
  if ((FrontDrive_IsSpeedModeReady(drive, FRONT_DRIVE_SIDE_LEFT) == 0U) ||
      (FrontDrive_IsSpeedModeReady(drive, FRONT_DRIVE_SIDE_RIGHT) == 0U))
  {
    drive->command_step = 0U;
    return 1U;
  }

  while (drive->command_step < 6U)
  {
    side = (FrontDrive_Side_t)((drive->speed_first_side +
                              (drive->command_step - 4U)) & 0x01U);
    if (FrontDrive_IsSidePresent(side) == 0U)
    {
      drive->command_step++;
      continue;
    }

    esc = FrontDrive_GetEsc(drive, side);
    target_erpm = (side == FRONT_DRIVE_SIDE_RIGHT) ?
      drive->pending_right_erpm : drive->pending_left_erpm;
    status = FrontDrive_WriteSpeed(drive, side, target_erpm, now_ms);
    if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
    {
      drive->command_step++;
    }
    if (drive->command_step >= 6U)
    {
      drive->speed_first_side ^= 1U;
      drive->command_step = 0U;
      drive->command_pending = 0U;
      drive->command_urgent = 0U;
    }
    return 1U;
  }

  return 0U;
}

/**
  * @brief 轮询左右电调心跳。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已经发送心跳帧，0 表示本轮没有发送。
  */
static uint8_t FrontDrive_ServiceHeartbeat(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  uint8_t i;
  uint8_t side;

  if ((drive == 0) || (FrontDrive_CanSend(drive, now_ms) == 0U))
  {
    return 0U;
  }

  for (i = 0U; i < 2U; i++)
  {
    side = (uint8_t)((drive->heartbeat_side + i) & 0x01U);
    if (side == 0U)
    {
      if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_LEFT) == 0U)
      {
        continue;
      }

      if ((now_ms - drive->left_last_heartbeat_ms) >= FRONT_DRIVE_HEARTBEAT_MS)
      {
        status = OID_ESC_SendHeartbeat(drive->bus, &drive->left);
        if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
        {
          FrontDrive_RecordHeartbeat(drive, 0U, now_ms);
          drive->heartbeat_side = 1U;
          return 1U;
        }
      }
    }
    else
    {
      if (FrontDrive_IsSidePresent(FRONT_DRIVE_SIDE_RIGHT) == 0U)
      {
        continue;
      }

      if ((now_ms - drive->right_last_heartbeat_ms) >= FRONT_DRIVE_HEARTBEAT_MS)
      {
        status = OID_ESC_SendHeartbeat(drive->bus, &drive->right);
        if (FrontDrive_MarkTxResult(drive, status, now_ms) != 0U)
        {
          FrontDrive_RecordHeartbeat(drive, 1U, now_ms);
          drive->heartbeat_side = 0U;
          return 1U;
        }
      }
    }
  }

  return 0U;
}

/**
  * @brief 轮询左右电调基础状态。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @return 1 表示已经发送状态读取帧，0 表示本轮没有发送。
  */
static uint8_t FrontDrive_ServiceStatus(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  OID_ESC_Handle_t *esc;
  FrontDrive_Side_t selected_side;
  uint8_t side;
  uint8_t sending_second;

  if ((drive == 0) ||
      (FrontDrive_CanSend(drive, now_ms) == 0U) ||
      (FrontDrive_ReadHasHeartbeatBudget(drive, now_ms) == 0U))
  {
    return 0U;
  }

  sending_second = drive->status_pair_pending;
  if ((sending_second == 0U) &&
      ((now_ms - drive->last_status_ms) < FRONT_DRIVE_STATUS_PAIR_MS))
  {
    return 0U;
  }

  side = (sending_second != 0U) ?
    drive->status_side : drive->status_pair_first_side;
  selected_side = (side == 0U) ? FRONT_DRIVE_SIDE_LEFT : FRONT_DRIVE_SIDE_RIGHT;
  if (FrontDrive_IsSidePresent(selected_side) == 0U)
  {
    selected_side = (selected_side == FRONT_DRIVE_SIDE_LEFT) ?
      FRONT_DRIVE_SIDE_RIGHT : FRONT_DRIVE_SIDE_LEFT;
    side ^= 1U;
    if (FrontDrive_IsSidePresent(selected_side) == 0U)
    {
      return 0U;
    }
  }

  esc = FrontDrive_GetEsc(drive, selected_side);
  status = OID_ESC_RequestStatus(drive->bus, esc);
  if (FrontDrive_MarkTxResult(drive, status, now_ms) == 0U)
  {
    return 0U;
  }

  if (sending_second == 0U)
  {
    drive->last_status_ms = now_ms;
    drive->status_pair_pending = 1U;
    drive->status_side = side ^ 1U;
  }
  else
  {
    drive->status_pair_pending = 0U;
    drive->status_pair_first_side ^= 1U;
    drive->status_side = drive->status_pair_first_side;
  }

  if (side == 0U)
  {
    drive->left_status_request_ms = now_ms;
    drive->left_wait_status = 1U;
  }
  else
  {
    drive->right_status_request_ms = now_ms;
    drive->right_wait_status = 1U;
  }
  return 1U;
}

/**
  * @brief 初始化前轮驱动管理层。
  * @param drive 前轮驱动对象。
  * @param bus   OID 左/右电调共用 RS485 总线。
  * @note  默认只初始化对象和通信维护状态，不主动下发运动速度。
  */
void FrontDrive_Init(FrontDrive_Handle_t *drive, BSP_RS485_Bus_t *bus,
                     uint8_t left_id, uint8_t right_id, uint8_t pole_pairs)
{
  if (drive == 0)
  {
    return;
  }

  drive->bus = bus;
  drive->last_control_read_ms = 0U;
  drive->control_read_enabled = 0U;
  drive->control_read_side = 0U;
  drive->control_read_allowed = 0U;
  drive->left_heartbeat_max_gap_ms = drive->right_heartbeat_max_gap_ms = 0U;
  drive->heartbeat_sent_mask = 0U;
  drive->left_speed_tx_ms = drive->right_speed_tx_ms = 0U;
  drive->left_speed_tx_erpm = drive->right_speed_tx_erpm = 0;
  drive->safety_zero_refresh_ms = 0U;
  drive->last_tx_ms = 0U - FRONT_DRIVE_TX_GAP_MS;
  drive->last_status_ms = 0U - FRONT_DRIVE_STATUS_PAIR_MS;
  drive->left_last_heartbeat_ms = 0U - FRONT_DRIVE_HEARTBEAT_LOST_MS;
  drive->right_last_heartbeat_ms = 0U - FRONT_DRIVE_HEARTBEAT_LOST_MS;
  /*
   * 霍尔版本虽无需找 Z，仍故意从“心跳已断档”状态启动：首轮任务先执行
   * 零目标/零占空比 -> 心跳 -> 再清零，避免 MCU 复位后恢复心跳时释放 OID 内旧目标。
   */
  drive->left_status_request_ms = 0U;
  drive->right_status_request_ms = 0U;
  drive->heartbeat_side = 0U;
  drive->status_side = 0U;
  drive->status_pair_pending = 0U;
  drive->status_pair_first_side = 0U;
  drive->left_wait_status = 0U;
  drive->right_wait_status = 0U;
  drive->status_preempt_count = 0U;
  drive->pending_left_erpm = 0;
  drive->pending_right_erpm = 0;
  drive->pending_left_brake_10ma = 0U;
  drive->pending_right_brake_10ma = 0U;
  drive->command_type = (uint8_t)FRONT_DRIVE_COMMAND_SPEED;
  drive->command_pending = 0U;
  drive->command_step = 0U;
  drive->command_urgent = 0U;
  drive->speed_first_side = 0U;
  drive->left_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
  drive->right_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
  drive->z_phase_start_ms = 0U;
  drive->z_last_request_ms = 0U;
#if (FRONT_DRIVE_AUTO_Z_SEARCH != 0U)
  drive->z_state = (uint8_t)FRONT_DRIVE_Z_STATE_QUERY;
#else
  drive->z_state = (uint8_t)FRONT_DRIVE_Z_STATE_READY;
#endif
  drive->z_command_step = 0U;
  drive->z_found_stop_step = 0U;
  drive->z_cycle_count = 0U;
  drive->left_z_duty_active = 0U;
  drive->right_z_duty_active = 0U;
  drive->z_fault_rearm_required = 0U;
  drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_IDLE;
  drive->safety_clear_step = 0U;
  drive->safety_clear_left = 0U;
  drive->safety_clear_right = 0U;
  drive->safety_rearm_in_progress = 0U;

  OID_ESC_Init(&drive->left, left_id, pole_pairs);
  OID_ESC_Init(&drive->right, right_id, pole_pairs);
}

/**
  * @brief 前轮双电调周期任务。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @note  收包/有界事务等待 -> 心跳断档安全清零 -> 到期心跳 -> 紧急双零速
  *        -> 配对状态/普通速度 -> 仅锁车停止时的额外回读。所有模式保持心跳。
  */
void FrontDrive_Task(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  uint8_t frame[BSP_RS485_RX_BUFFER_SIZE];
  uint16_t len;

  if ((drive == 0) || (drive->bus == 0))
  {
    return;
  }

  BSP_RS485_PollFrameTimeout(drive->bus, now_ms, FRONT_DRIVE_FRAME_IDLE_MS);
  len = BSP_RS485_GetFrame(drive->bus, frame, sizeof(frame));
  if (len > 0U)
  {
    FrontDrive_HandleRxFrame(drive, frame, len, now_ms);
  }

  FrontDrive_CheckStatusTimeout(drive, now_ms);
  FrontDrive_CheckHeartbeatLoss(drive, now_ms);
  FrontDrive_CheckStatusHealth(drive, now_ms);

  OID_ESC_PollDiagnostic(&drive->left, now_ms);
  OID_ESC_PollDiagnostic(&drive->right, now_ms);
  /* Finish the diagnostic response (50 ms bound) before another request;
     never mix it into a status frame. Pending RX is consumed above. */
  if (drive->left.diagnostic.control_pending != 0U ||
      drive->right.diagnostic.control_pending != 0U)
  {
    return;
  }

  if (((drive->command_urgent != 0U) ||
       ((drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE) &&
        (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM) &&
        (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_WAIT_VERIFY))) &&
      (FrontDrive_PreemptStatusWait(drive) == 0U))
  {
    return;
  }

  if (FrontDrive_ServiceSafetyClear(drive, now_ms) != 0U)
  {
    return;
  }

  /* 普通控制和心跳不截断已经开始的 Modbus 状态响应。 */
  if (FrontDrive_HasPendingStatusResponse(drive) != 0U)
  {
    return;
  }

  /*
   * 手册要求心跳与控制模式无关，必须独立周期刷新。
   * 因此心跳优先级高于速度命令和状态查询，避免连续油门命令把 6000 心跳寄存器挤掉。
   */
  if (FrontDrive_ServiceHeartbeat(drive, now_ms) != 0U)
  {
    return;
  }

  /* Never use loss of heartbeat as a brake. WAIT_REARM keeps both the link
   * and zero requests alive, but SetTargetErpm still rejects nonzero output. */
  if ((drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM ||
       drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_WAIT_VERIFY) &&
      (now_ms - drive->safety_zero_refresh_ms) >= 100U &&
      drive->command_pending == 0U)
  {
    if (FrontDrive_StopUrgent(drive) == HAL_OK)
      drive->safety_zero_refresh_ms = now_ms;
  }
  if (drive->command_urgent != 0U &&
      FrontDrive_ProcessPendingCommand(drive, now_ms) != 0U) return;

  if ((FRONT_DRIVE_AUTO_Z_SEARCH != 0U) &&
      (FrontDrive_ServiceZFinder(drive, now_ms) != 0U))
  {
    return;
  }

  /* Complete a started speed pair coherently. At pair boundaries, overdue
   * status gets a slot even under continuous 10 ms target changes. */
  if ((drive->command_pending == 0U || drive->command_step != 5U) &&
      (FrontDrive_ServiceStatus(drive, now_ms) != 0U))
  {
    return;
  }

  if (FrontDrive_ProcessPendingCommand(drive, now_ms) != 0U)
  {
    return;
  }

  if (FrontDrive_ServiceStatus(drive, now_ms) != 0U) return;
  if (drive->control_read_enabled != 0U && drive->control_read_allowed != 0U &&
      drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_IDLE &&
      drive->command_pending == 0U &&
      FrontDrive_ReadHasHeartbeatBudget(drive, now_ms) != 0U &&
      (now_ms - drive->last_control_read_ms) >= 250U &&
      FrontDrive_CanSend(drive, now_ms) != 0U)
  {
    OID_ESC_Handle_t *esc = (drive->control_read_side == 0U) ? &drive->left : &drive->right;
    if (OID_ESC_RequestControl(drive->bus, esc, now_ms) == HAL_OK)
    {
      drive->last_tx_ms = now_ms;
      drive->last_control_read_ms = now_ms;
      drive->control_read_side ^= 1U;
    }
  }
}

/**
  * @brief 设置左右前轮目标电角速度。
  * @param drive      前轮驱动对象。
  * @param left_erpm  左前轮目标 erpm，正负号决定方向。
  * @param right_erpm 右前轮目标 erpm，正负号决定方向。
  * @return HAL_OK 表示命令已挂起，实际发送由 FrontDrive_Task 分帧完成。
  */
HAL_StatusTypeDef FrontDrive_SetTargetErpm(FrontDrive_Handle_t *drive, int32_t left_erpm, int32_t right_erpm)
{
  if ((drive == 0) || (drive->bus == 0))
  {
    return HAL_ERROR;
  }

  if ((drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE) ||
      (((left_erpm != 0) || (right_erpm != 0)) &&
       ((FrontDrive_IsZReady(drive) == 0U) ||
        (drive->z_fault_rearm_required != 0U))))
  {
    return HAL_BUSY;
  }

  /*
   * 电子刹车同样是左右双电调 4 帧序列。若诊断调用者立刻用零速/速度命令覆盖，
   * 可能只完成左侧刹车而右侧仍停留在旧速度模式，所以必须先等刹车序列完整落地。
   */
  if ((drive->command_pending != 0U) &&
      (drive->command_type == (uint8_t)FRONT_DRIVE_COMMAND_BRAKE))
  {
    return HAL_BUSY;
  }

  /*
   * 双电调目标需要顺序写入；速度模式已确认时为两帧，模式未知时还需先安全清零并切模式。
   * 当前序列必须使用同一组目标快照。
   * 若摇杆在左侧已发送、右侧尚未发送时覆盖 pending 值，会造成左右电调拿到不同一代目标，
   * 表现为一侧正常、一侧不动或方向/速度短时间不协调。首侧目标未发送前可合并最新值；
   * 首侧已发送则完成当前双侧快照，上层下一周期重试最新值，不建立旧目标队列。
   */
  if ((drive->command_pending != 0U) &&
      (drive->command_type == (uint8_t)FRONT_DRIVE_COMMAND_SPEED))
  {
    if ((drive->pending_left_erpm == left_erpm) &&
        (drive->pending_right_erpm == right_erpm))
    {
      return HAL_OK;
    }

    if (drive->command_urgent == 0U && drive->command_step <= 4U)
    {
      drive->pending_left_erpm = left_erpm;
      drive->pending_right_erpm = right_erpm;
      return HAL_OK;
    }

    return HAL_BUSY;
  }

  drive->pending_left_erpm = left_erpm;
  drive->pending_right_erpm = right_erpm;
  drive->pending_left_brake_10ma = 0U;
  drive->pending_right_brake_10ma = 0U;
  drive->command_type = (uint8_t)FRONT_DRIVE_COMMAND_SPEED;
  drive->command_step =
    ((FrontDrive_IsSpeedModeReady(drive, FRONT_DRIVE_SIDE_LEFT) != 0U) &&
     (FrontDrive_IsSpeedModeReady(drive, FRONT_DRIVE_SIDE_RIGHT) != 0U)) ? 4U : 0U;
  drive->command_pending = 1U;
  drive->command_urgent = 0U;

  return HAL_OK;
}

/**
  * @brief 按电机机械 rpm 设置左右前轮目标速度。
  * @param drive     前轮驱动对象。
  * @param left_rpm  左电机目标机械转速，单位 rpm。
  * @param right_rpm 右电机目标机械转速，单位 rpm。
  * @return HAL_OK 表示命令已挂起。
  * @note  OID 速度寄存器使用 erpm，换算关系为 erpm = rpm * 极对数。
  */
HAL_StatusTypeDef FrontDrive_SetTargetRpm(FrontDrive_Handle_t *drive, int32_t left_rpm, int32_t right_rpm)
{
  if (drive == 0)
  {
    return HAL_ERROR;
  }

  return FrontDrive_SetTargetErpm(drive,
                                 left_rpm * (int32_t)drive->left.pole_pairs,
                                 right_rpm * (int32_t)drive->right.pole_pairs);
}

/**
  * @brief 设置左右前轮电子刹车电流并切入刹车控制模式。
  * @param drive              前轮驱动对象。
  * @param left_current_10ma  左电调电子刹车电流，单位 10mA，100 表示 1A。
  * @param right_current_10ma 右电调电子刹车电流，单位 10mA。
  * @return HAL_OK 表示刹车命令已挂起，实际发送由 FrontDrive_Task 分帧完成。
  * @note  该函数只使用 OID 手册 7.9 的电子刹车模式，不使用 7.10 手刹模式。
  *        电子刹车为再生制动，台架测试时需要观察母线电压和电源是否报警。
  */
HAL_StatusTypeDef FrontDrive_SetBrakeCurrent(FrontDrive_Handle_t *drive,
                                            uint16_t left_current_10ma,
                                            uint16_t right_current_10ma)
{
  if ((drive == 0) || (drive->bus == 0))
  {
    return HAL_ERROR;
  }

  if ((drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE) ||
      (drive->z_fault_rearm_required != 0U))
  {
    return HAL_BUSY;
  }

  /*
   * 同一条电子刹车命令正在分帧发送时直接复用，避免周期刷新把步骤反复打回 0。
   */
  if ((drive->command_pending != 0U) &&
      (drive->command_type == (uint8_t)FRONT_DRIVE_COMMAND_BRAKE) &&
      (drive->pending_left_brake_10ma == left_current_10ma) &&
      (drive->pending_right_brake_10ma == right_current_10ma))
  {
    return HAL_OK;
  }

  if ((drive->command_pending != 0U) &&
      (drive->command_type == (uint8_t)FRONT_DRIVE_COMMAND_BRAKE))
  {
    return HAL_BUSY;
  }

  drive->pending_left_erpm = 0;
  drive->pending_right_erpm = 0;
  drive->pending_left_brake_10ma = left_current_10ma;
  drive->pending_right_brake_10ma = right_current_10ma;
  drive->command_type = (uint8_t)FRONT_DRIVE_COMMAND_BRAKE;
  drive->command_step = 0U;
  drive->command_pending = 1U;
  drive->command_urgent = 0U;

  return HAL_OK;
}

/**
  * @brief 停止左右前轮速度输出。
  * @param drive 前轮驱动对象。
  * @return HAL_OK 表示零速命令已挂起。
  */
HAL_StatusTypeDef FrontDrive_Stop(FrontDrive_Handle_t *drive)
{
  return FrontDrive_SetTargetErpm(drive, 0, 0);
}

/**
  * @brief 挂起可覆盖普通命令的双侧 0ERPM 安全命令。
  * @note  已发起的 Modbus 事务先完成或超时，不插帧；不再启动新查询抢占零速。
  */
HAL_StatusTypeDef FrontDrive_StopUrgent(FrontDrive_Handle_t *drive)
{
  if ((drive == 0) || (drive->bus == 0))
  {
    return HAL_ERROR;
  }

  if ((drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE) &&
      (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM) &&
      (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_WAIT_VERIFY))
  {
    return HAL_OK;
  }
  if (FrontDrive_IsZReady(drive) == 0U)
  {
    return HAL_BUSY;
  }
  if ((drive->command_pending != 0U) &&
      (drive->command_urgent != 0U) &&
      (drive->pending_left_erpm == 0) &&
      (drive->pending_right_erpm == 0))
  {
    return HAL_OK;
  }

  drive->pending_left_erpm = 0;
  drive->pending_right_erpm = 0;
  drive->pending_left_brake_10ma = 0U;
  drive->pending_right_brake_10ma = 0U;
  drive->command_type = (uint8_t)FRONT_DRIVE_COMMAND_SPEED;
  drive->command_step =
    ((FrontDrive_IsSpeedModeReady(drive, FRONT_DRIVE_SIDE_LEFT) != 0U) &&
     (FrontDrive_IsSpeedModeReady(drive, FRONT_DRIVE_SIDE_RIGHT) != 0U)) ? 4U : 0U;
  drive->command_pending = 1U;
  drive->command_urgent = 1U;
  return HAL_OK;
}

/**
  * @brief 获取指定侧电调的最近状态快照。
  * @param drive 前轮驱动对象。
  * @param side  左/右侧选择。
  * @return 状态快照指针；参数非法时返回空指针。
  */
const OID_ESC_Status_t *FrontDrive_GetStatus(const FrontDrive_Handle_t *drive, FrontDrive_Side_t side)
{
  const OID_ESC_Handle_t *esc = FrontDrive_GetEscConst(drive, side);

  if (esc == 0)
  {
    return 0;
  }

  return &esc->status;
}

/**
  * @brief 判断指定侧电调状态是否在线新鲜。
  * @param drive  前轮驱动对象。
  * @param side   左/右侧选择。
  * @param now_ms 当前系统 tick。
  * @return 1 表示最近 500ms 内读到过该电调状态，0 表示离线或尚未读到状态。
  */
uint8_t FrontDrive_IsOnline(const FrontDrive_Handle_t *drive, FrontDrive_Side_t side, uint32_t now_ms)
{
  const OID_ESC_Status_t *status = FrontDrive_GetStatus(drive, side);

  if ((status == 0) || (status->last_update_ms == 0U))
  {
    return 0U;
  }

  return ((now_ms - status->last_update_ms) <= FRONT_DRIVE_STATUS_SAFETY_TIMEOUT_MS) ? 1U : 0U;
}

/**
  * @brief 判断传感器启动条件是否满足，可以接受非零速度闭环命令。
  * @param drive 前轮驱动对象。
  * @return 霍尔版本恒由初始化置 READY；ABZ 兼容版本表示找 Z 已完成。
  */
uint8_t FrontDrive_IsZReady(const FrontDrive_Handle_t *drive)
{
  if (drive == 0)
  {
    return 0U;
  }

  return (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY) ? 1U : 0U;
}

/** 双侧完整状态新鲜且无故障、传感器启动条件满足且安全状态空闲时才允许非零运动。 */
uint8_t FrontDrive_IsMotionReady(const FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if ((drive == 0) ||
      (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_READY) ||
      (drive->z_fault_rearm_required != 0U) ||
      (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_IDLE))
  {
    return 0U;
  }
  return FrontDrive_ArePresentStatusesHealthy(drive, now_ms);
}

/**
  * @brief 判断心跳断档保护是否已经清零完成并等待上层重新解锁。
  * @param drive 前轮驱动对象。
  * @return 1 表示仍在等待重连清零或零速验证；0 表示不需要安全重臂。
  */
uint8_t FrontDrive_IsSafetyRearmRequired(const FrontDrive_Handle_t *drive)
{
  if (drive == 0)
  {
    return 0U;
  }

  return ((drive->z_fault_rearm_required != 0U) ||
          (drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM) ||
          (drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_WAIT_VERIFY)) ? 1U : 0U;
}

/**
  * @brief 上层撤销解锁时，中止正在进行的找 Z 流程并进入安全收尾。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @note  未解锁状态不应该继续用找 Z 占空比转动；收尾阶段只写 0% 占空比并保持速度闭环禁用。
  */
void FrontDrive_AbortZSearch(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if (drive == 0)
  {
    return;
  }

  if ((drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY) ||
      (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_FAILED) ||
      (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_STOPPING))
  {
    return;
  }

  FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_STOPPING, now_ms);
}

/**
  * @brief 在找 Z 失败后，由上层确认油门回中再重新发起一次找 Z。
  * @param drive  前轮驱动对象。
  * @param now_ms 当前系统 tick。
  * @note  保留“速度闭环前必须找 Z”的硬保护，只允许从 FAILED 回到 QUERY。
  *        上电找 Z 失败沿用原重试；运行中丢 Z 则只允许在 WAIT_REARM 中重试，
  *        且找回后仍须完成清零和双侧实测零速验证。
  */
void FrontDrive_RetryZSearch(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if (drive == 0)
  {
    return;
  }

  if (drive->z_state != (uint8_t)FRONT_DRIVE_Z_STATE_FAILED)
  {
    return;
  }
  if ((drive->z_fault_rearm_required != 0U) &&
      (drive->safety_state != (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM))
  {
    /* 初次故障清零尚未完成时，禁止上层提前启动非零占空比找 Z。 */
    return;
  }

  drive->pending_left_erpm = 0;
  drive->pending_right_erpm = 0;
  drive->pending_left_brake_10ma = 0U;
  drive->pending_right_brake_10ma = 0U;
  drive->command_type = (uint8_t)FRONT_DRIVE_COMMAND_SPEED;
  drive->command_pending = 0U;
  drive->command_step = 0U;
  if (drive->z_fault_rearm_required == 0U)
  {
    drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_IDLE;
  }
  drive->safety_clear_step = 0U;
  drive->safety_clear_left = 0U;
  drive->safety_clear_right = 0U;
  drive->safety_rearm_in_progress = 0U;
  drive->left_z_duty_active = 0U;
  drive->right_z_duty_active = 0U;
  drive->z_cycle_count = 0U;
  FrontDrive_EnterZState(drive, FRONT_DRIVE_Z_STATE_QUERY, now_ms);
}

/**
  * @brief 上层确认安全后启动重连清零，并在新状态确认双侧零速后解除锁存。
  * @param drive 前轮驱动对象。
  * @note  只应在油门回中、急停未触发、SBUS 在线时调用；WAIT_REARM 不直接放行。
  */
void FrontDrive_RearmSafety(FrontDrive_Handle_t *drive, uint32_t now_ms)
{
  if (drive == 0)
  {
    return;
  }

  if ((drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_WAIT_REARM) &&
      (drive->z_state == (uint8_t)FRONT_DRIVE_Z_STATE_READY) &&
      (FrontDrive_ArePresentStatusesHealthy(drive, now_ms) != 0U))
  {
    /* Heartbeats continue while locked; rearm still requires a complete
     * clear -> heartbeat -> clear sequence and fresh stopped feedback. */
    FrontDrive_RequestSafetyClear(drive, 1U, 1U);
    drive->left_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
    drive->right_control_mode_cache = (uint16_t)OID_ESC_MODE_IDLE;
    drive->safety_rearm_in_progress = 1U;
  }
  else if ((drive->safety_state == (uint8_t)FRONT_DRIVE_SAFETY_WAIT_VERIFY) &&
           (FrontDrive_ArePresentStatusesHealthy(drive, now_ms) != 0U) &&
           (FrontDrive_ArePresentStatusesStopped(drive) != 0U))
  {
    drive->z_fault_rearm_required = 0U;
    drive->safety_state = (uint8_t)FRONT_DRIVE_SAFETY_IDLE;
  }
}
