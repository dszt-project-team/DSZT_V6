#include "buzzer_app.h"
#include "buzzer_config.h"
#include "bsp_buzzer.h"
#include "robot_def.h"
#include "vehicle_status.h"
#include "tim.h"

static VehicleStatus s_status;
static VehicleStatus s_previous_status;
static uint32_t s_status_since_ms;
static uint32_t s_connected_ms;
static uint8_t s_status_valid;
static uint8_t s_event_handled;
static uint8_t s_online_seen;
static uint8_t s_ready_seen;
static uint8_t s_connect_pending;
static BSP_BuzzerCue s_alarm;

static uint8_t BuzzerApp_IsReady(VehicleStatus status)
{
  return (uint8_t)(status == VEHICLE_STATUS_MANUAL_READY || status == VEHICLE_STATUS_AUTO_READY);
}

static uint8_t BuzzerApp_IsFault(VehicleStatus status)
{
  return (uint8_t)(status == VEHICLE_STATUS_RC_LOST || status == VEHICLE_STATUS_STEER_FAULT ||
                   status == VEHICLE_STATUS_OID_FAULT || status == VEHICLE_STATUS_FC_FAULT);
}

static BSP_BuzzerCue BuzzerApp_AlarmCue(uint32_t now_ms)
{
  uint32_t age = now_ms - s_status_since_ms;
  if (BUZZER_APP_ALARM_ENABLE == 0U) return BSP_BUZZER_CUE_SILENT;
  if (s_status == VEHICLE_STATUS_STARTUP_WAIT && s_online_seen != 0U &&
      BUZZER_APP_STARTUP_WAIT_ENABLE != 0U && age >= BUZZER_APP_STARTUP_WAIT_MS)
    return BSP_BUZZER_CUE_STARTUP_WAIT;
  if (age < BUZZER_APP_FAULT_STABLE_MS) return BSP_BUZZER_CUE_SILENT;
  switch (s_status)
  {
    case VEHICLE_STATUS_RC_LOST:
      if (s_online_seen != 0U && BUZZER_APP_RC_FAULT_ENABLE != 0U) return BSP_BUZZER_CUE_RC_LOST;
      break;
    case VEHICLE_STATUS_STEER_FAULT:
      return (BUZZER_APP_STEER_FAULT_ENABLE != 0U) ?
          BSP_BUZZER_CUE_STEER_FAULT : BSP_BUZZER_CUE_SILENT;
    case VEHICLE_STATUS_OID_FAULT:
      return (BUZZER_APP_OID_FAULT_ENABLE != 0U) ?
          BSP_BUZZER_CUE_OID_DUAL_FAULT : BSP_BUZZER_CUE_SILENT;
    case VEHICLE_STATUS_FC_FAULT:
      return (BUZZER_APP_FC_FAULT_ENABLE != 0U) ?
          BSP_BUZZER_CUE_FC_DUAL_TIMEOUT : BSP_BUZZER_CUE_SILENT;
    default: break;
  }
  return BSP_BUZZER_CUE_SILENT;
}

void BuzzerApp_Init(void)
{
  s_status = VEHICLE_STATUS_LOCKED;
  s_previous_status = VEHICLE_STATUS_LOCKED;
  s_status_since_ms = 0U;
  s_connected_ms = 0U;
  s_status_valid = 0U;
  s_event_handled = 0U;
  s_online_seen = 0U;
  s_ready_seen = 0U;
  s_connect_pending = 0U;
  s_alarm = BSP_BUZZER_CUE_SILENT;
  if (BUZZER_APP_ENABLE == 0U) return;
  if (BSP_Buzzer_Init(&htim12, TIM_CHANNEL_1) == 0U) return;
  if (BUZZER_APP_BOOT_CUE_ENABLE != 0U) BSP_Buzzer_Play(BSP_BUZZER_CUE_BOOT, HAL_GetTick());
}

void BuzzerApp_Task(uint32_t now_ms)
{
  VehicleStatus status;
  BSP_BuzzerCue alarm, event = BSP_BUZZER_CUE_SILENT;
  uint8_t healthy_rc;
  if (BUZZER_APP_ENABLE == 0U) return;
  status = VehicleStatus_Get();
  healthy_rc = (uint8_t)(g_robot_command.rc_online != 0U && g_robot_command.failsafe == 0U);
  if (s_online_seen == 0U && healthy_rc != 0U)
  {
    s_online_seen = 1U;
    s_connect_pending = BUZZER_APP_RC_CUE_ENABLE;
    s_connected_ms = now_ms;
  }
  if (s_status_valid == 0U || status != s_status)
  {
    s_previous_status = s_status;
    if (s_status_valid != 0U && BuzzerApp_IsReady(s_status) != 0U && BuzzerApp_IsReady(status) == 0U)
      BSP_Buzzer_Stop();
    /* 普通连接/锁车音在故障出现时立即取消；开机音只表示启动，可正常完成。
     * 真正的持续故障仍在300 ms稳定后以报警音抢占，不等待普通音排队。
     */
    if (BuzzerApp_IsFault(status) != 0U && BSP_Buzzer_GetCue() != BSP_BUZZER_CUE_BOOT)
      BSP_Buzzer_Stop();
    if (s_alarm != BSP_BUZZER_CUE_SILENT)
    {
      BSP_Buzzer_Stop();
      s_alarm = BSP_BUZZER_CUE_SILENT;
    }
    s_status = status;
    s_status_valid = 1U;
    s_status_since_ms = now_ms;
    s_event_handled = 0U;
  }
  if (healthy_rc == 0U || BuzzerApp_IsFault(status) != 0U ||
      (now_ms - s_connected_ms) >= BUZZER_APP_CONNECT_EVENT_TTL_MS)
    s_connect_pending = 0U;

  alarm = BuzzerApp_AlarmCue(now_ms);
  if (alarm != s_alarm)
  {
    BSP_Buzzer_Stop();
    s_alarm = alarm;
    if (alarm != BSP_BUZZER_CUE_SILENT) BSP_Buzzer_Play(alarm, now_ms);
  }
  /* 故障期间不积压模式提示；恢复后不会补播已作废的解锁事件。 */
  if (BuzzerApp_IsFault(status) != 0U) s_event_handled = 1U;
  else if (s_alarm == BSP_BUZZER_CUE_SILENT && s_event_handled == 0U &&
           (now_ms - s_status_since_ms) >= BUZZER_APP_EVENT_STABLE_MS)
  {
    s_event_handled = 1U;
    if (BUZZER_APP_MODE_CUE_ENABLE != 0U)
    {
      if (status == VEHICLE_STATUS_MANUAL_READY && BuzzerApp_IsReady(s_previous_status) == 0U &&
          BuzzerApp_IsFault(s_previous_status) == 0U) event = BSP_BUZZER_CUE_MANUAL_READY;
      else if (status == VEHICLE_STATUS_AUTO_READY && BuzzerApp_IsFault(s_previous_status) == 0U)
        event = BSP_BUZZER_CUE_AUTO_READY;
      else if (status == VEHICLE_STATUS_LOCKED && s_ready_seen != 0U) event = BSP_BUZZER_CUE_BRAKE;
    }
    if (BuzzerApp_IsReady(status) != 0U) s_ready_seen = 1U;
  }
  BSP_Buzzer_Task(now_ms);
  if (event != BSP_BUZZER_CUE_SILENT)
  {
    s_connect_pending = 0U;
    BSP_Buzzer_Play(event, now_ms);
  }
  else if (s_connect_pending != 0U && s_alarm == BSP_BUZZER_CUE_SILENT &&
           BSP_Buzzer_GetCue() == BSP_BUZZER_CUE_SILENT)
  {
    s_connect_pending = 0U;
    BSP_Buzzer_Play(BSP_BUZZER_CUE_RC_CONNECTED, now_ms);
  }
}
