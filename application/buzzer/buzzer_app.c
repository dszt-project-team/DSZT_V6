#include "buzzer_app.h"
#include "buzzer_config.h"
#include "bsp_buzzer.h"
#include "robot_def.h"
#include "vehicle_status.h"
#include "tim.h"

static VehicleStatus s_status;
static uint32_t s_status_since_ms;
static uint8_t s_status_valid, s_event_handled, s_online_seen;
static BSP_BuzzerCue s_alarm;

static uint8_t BuzzerApp_IsFault(VehicleStatus status)
{
  return (uint8_t)(status == VEHICLE_STATUS_STEER_FAULT ||
                  status == VEHICLE_STATUS_OID_FAULT || status == VEHICLE_STATUS_FC_FAULT);
}
static BSP_BuzzerCue BuzzerApp_AlarmCue(uint32_t now_ms)
{
  uint32_t age = now_ms - s_status_since_ms;
  if (BUZZER_APP_ALARM_ENABLE == 0U) return BSP_BUZZER_CUE_SILENT;
  if (s_status == VEHICLE_STATUS_RELEASE_WAIT && BUZZER_APP_RELEASE_WAIT_ENABLE != 0U &&
      age >= BUZZER_APP_RELEASE_WAIT_MS) return BSP_BUZZER_CUE_RELEASE_WAIT;
  if (age < BUZZER_APP_FAULT_STABLE_MS) return BSP_BUZZER_CUE_SILENT;
  switch (s_status)
  {
    case VEHICLE_STATUS_STEER_FAULT:
      return (BUZZER_APP_STEER_FAULT_ENABLE != 0U) ? BSP_BUZZER_CUE_STEER_FAULT : BSP_BUZZER_CUE_SILENT;
    case VEHICLE_STATUS_OID_FAULT:
      return (BUZZER_APP_OID_FAULT_ENABLE != 0U) ? BSP_BUZZER_CUE_OID_FAULT : BSP_BUZZER_CUE_SILENT;
    case VEHICLE_STATUS_FC_FAULT:
      return (s_online_seen != 0U && BUZZER_APP_FC_FAULT_ENABLE != 0U) ?
          BSP_BUZZER_CUE_FC_FAULT : BSP_BUZZER_CUE_SILENT;
    default: return BSP_BUZZER_CUE_SILENT;
  }
}
void BuzzerApp_Init(void)
{
  s_status = VEHICLE_STATUS_FC_FAULT;
  s_status_since_ms = 0U;
  s_status_valid = s_event_handled = s_online_seen = 0U;
  s_alarm = BSP_BUZZER_CUE_SILENT;
  if (BUZZER_APP_ENABLE == 0U) return;
  if (BSP_Buzzer_Init(&htim12, TIM_CHANNEL_1) == 0U) return;
  if (BUZZER_APP_BOOT_CUE_ENABLE != 0U) BSP_Buzzer_Play(BSP_BUZZER_CUE_BOOT, HAL_GetTick());
}
void BuzzerApp_Task(uint32_t now_ms)
{
  VehicleStatus status;
  BSP_BuzzerCue alarm;
  if (BUZZER_APP_ENABLE == 0U) return;
  status = VehicleStatus_Get();
  if (g_robot_command.source_online != 0U && g_robot_chassis.fc_drive_online != 0U &&
      g_robot_chassis.fc_steer_online != 0U && g_robot_chassis.fc_drive_fault == 0U &&
      g_robot_chassis.fc_steer_fault == 0U) s_online_seen = 1U;
  if (s_status_valid == 0U || status != s_status)
  {
    /* 离开就绪或出现故障取消普通音；开机音只表示启动，可正常完成。 */
    if ((s_status_valid != 0U && s_status == VEHICLE_STATUS_AUTO_READY) ||
        (BuzzerApp_IsFault(status) != 0U && BSP_Buzzer_GetCue() != BSP_BUZZER_CUE_BOOT) ||
        s_alarm != BSP_BUZZER_CUE_SILENT) BSP_Buzzer_Stop();
    s_alarm = BSP_BUZZER_CUE_SILENT;
    s_status = status;
    s_status_valid = 1U;
    s_status_since_ms = now_ms;
    s_event_handled = 0U;
  }
  alarm = BuzzerApp_AlarmCue(now_ms);
  if (alarm != s_alarm)
  {
    BSP_Buzzer_Stop();
    s_alarm = alarm;
    if (alarm != BSP_BUZZER_CUE_SILENT) BSP_Buzzer_Play(alarm, now_ms);
  }
  BSP_Buzzer_Task(now_ms);
  if (s_event_handled == 0U && (now_ms - s_status_since_ms) >= BUZZER_APP_EVENT_STABLE_MS)
  {
    s_event_handled = 1U;
    /* 不积压事件：只有当前真正就绪才提示；故障恢复并回中放行后可再次提示。 */
    if (status == VEHICLE_STATUS_AUTO_READY && s_alarm == BSP_BUZZER_CUE_SILENT &&
        BUZZER_APP_READY_CUE_ENABLE != 0U) BSP_Buzzer_Play(BSP_BUZZER_CUE_AUTO_READY, now_ms);
  }
}
