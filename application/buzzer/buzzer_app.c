#include "buzzer_app.h"
#include "buzzer_config.h"

#include "bsp_buzzer.h"
#include "pwm_input.h"
#include "robot_def.h"
#include "tim.h"

static uint8_t s_previous_online;
static uint8_t s_previous_mode;
static uint8_t s_fc_healthy_seen;
static BSP_BuzzerCue s_persistent_cue;

static BSP_BuzzerCue BuzzerApp_GetOidCue(void)
{
  uint8_t mask = 0U;

  if (g_robot_chassis.left_fault != 0U)
  {
    mask |= 1U;
  }
  if (g_robot_chassis.right_fault != 0U)
  {
    mask |= 2U;
  }
  if (mask == 1U)
  {
    return BSP_BUZZER_CUE_OID_LEFT_FAULT;
  }
  if (mask == 2U)
  {
    return BSP_BUZZER_CUE_OID_RIGHT_FAULT;
  }
  if (mask == 3U)
  {
    return BSP_BUZZER_CUE_OID_DUAL_FAULT;
  }
  return BSP_BUZZER_CUE_SILENT;
}

static BSP_BuzzerCue BuzzerApp_GetFcCue(void)
{
  uint8_t timeout_mask = 0U;

  if (g_robot_command.mode != ROBOT_MODE_AUTO_FC)
  {
    s_fc_healthy_seen = 0U;
    return BSP_BUZZER_CUE_SILENT;
  }
  if ((g_robot_chassis.fc_drive_online != 0U) &&
      (g_robot_chassis.fc_steer_online != 0U) &&
      (g_robot_chassis.fc_drive_fault == PWM_INPUT_FAULT_NONE) &&
      (g_robot_chassis.fc_steer_fault == PWM_INPUT_FAULT_NONE))
  {
    s_fc_healthy_seen = 1U;
  }
  if (s_fc_healthy_seen == 0U)
  {
    return BSP_BUZZER_CUE_SILENT;
  }
  if (g_robot_chassis.fc_drive_fault == PWM_INPUT_FAULT_TIMEOUT)
  {
    timeout_mask |= 1U;
  }
  if (g_robot_chassis.fc_steer_fault == PWM_INPUT_FAULT_TIMEOUT)
  {
    timeout_mask |= 2U;
  }
  if (timeout_mask == 1U)
  {
    return BSP_BUZZER_CUE_FC_MAIN1_TIMEOUT;
  }
  if (timeout_mask == 2U)
  {
    return BSP_BUZZER_CUE_FC_MAIN2_TIMEOUT;
  }
  if (timeout_mask == 3U)
  {
    return BSP_BUZZER_CUE_FC_DUAL_TIMEOUT;
  }
  return BSP_BUZZER_CUE_SILENT;
}

void BuzzerApp_Init(void)
{
  if (BUZZER_APP_ENABLE == 0U) return;
  (void)BSP_Buzzer_Init(&htim12, TIM_CHANNEL_1);
  s_previous_mode = ROBOT_MODE_LOCKED;
  s_persistent_cue = BSP_BUZZER_CUE_SILENT;
}

void BuzzerApp_Task(uint32_t now_ms)
{
  BSP_BuzzerCue desired_cue;
  BSP_BuzzerCue fc_cue;
  BSP_BuzzerCue oid_cue;
  if (BUZZER_APP_ENABLE == 0U) return;
  fc_cue = BUZZER_APP_FC_FAULT_ENABLE ? BuzzerApp_GetFcCue() : BSP_BUZZER_CUE_SILENT;
  oid_cue = BUZZER_APP_OID_FAULT_ENABLE ? BuzzerApp_GetOidCue() : BSP_BUZZER_CUE_SILENT;

  desired_cue = (fc_cue != BSP_BUZZER_CUE_SILENT) ? fc_cue : oid_cue;
  if (desired_cue != s_persistent_cue)
  {
    BSP_Buzzer_Stop();
    s_persistent_cue = desired_cue;
    if (desired_cue != BSP_BUZZER_CUE_SILENT)
    {
      BSP_Buzzer_Play(desired_cue, now_ms);
    }
  }

  if (s_persistent_cue == BSP_BUZZER_CUE_SILENT)
  {
    if (BUZZER_APP_RC_CUE_ENABLE && (s_previous_online == 0U) && (g_robot_command.rc_online != 0U))
    {
      BSP_Buzzer_Play(BSP_BUZZER_CUE_RC_CONNECTED, now_ms);
    }
    if (BUZZER_APP_MODE_CUE_ENABLE && s_previous_mode != g_robot_command.mode)
    {
      BSP_Buzzer_Play(BSP_BUZZER_CUE_MODE_CHANGE, now_ms);
    }
  }

  s_previous_online = g_robot_command.rc_online;
  s_previous_mode = g_robot_command.mode;
  BSP_Buzzer_Task(now_ms);
}
