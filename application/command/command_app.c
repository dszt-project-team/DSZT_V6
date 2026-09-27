#include "command_app.h"
#include "command_config.h"
#include "chassis_config.h"

#include "robot_def.h"
#include "sbus_rc.h"
#include "usart.h"

#include <string.h>

#define COMMAND_MODE_INVALID               0xFFU

static SBusRc_Handle_t s_rc;
static uint32_t s_last_good_frame_count;
static uint32_t s_last_lost_count;
static uint32_t s_last_guard_event_count;
static CommandRcDiagnostics s_rc_diagnostics;
static uint32_t s_center_start_ms;
static uint32_t s_loss_start_ms;
static uint8_t s_startup_lock_seen;
static uint8_t s_stable_mode;
static uint8_t s_candidate_mode;
static uint8_t s_candidate_frames;
static uint8_t s_unconfirmed_frames;
static uint8_t s_release_ready;
static uint8_t s_loss_active;
static uint16_t s_steer_samples[COMMAND_STEER_FILTER_WINDOW];
static uint32_t s_steer_sum, s_steer_filter_ms;
static uint8_t s_steer_index, s_steer_filter_valid;

static uint16_t CommandApp_FilterSteering(uint16_t pulse, uint32_t now_ms)
{
  unsigned i;
  if (COMMAND_STEER_FILTER_ENABLE == 0U) return pulse;
  if (s_steer_filter_valid == 0U)
  {
    for (i = 0U; i < COMMAND_STEER_FILTER_WINDOW; i++) s_steer_samples[i] = pulse;
    s_steer_sum = (uint32_t)pulse * COMMAND_STEER_FILTER_WINDOW;
    s_steer_index = 0U;
    s_steer_filter_ms = now_ms;
    s_steer_filter_valid = 1U;
  }
  else if ((now_ms - s_steer_filter_ms) >= COMMAND_STEER_FILTER_PERIOD_MS)
  {
    s_steer_filter_ms = now_ms;
    s_steer_sum -= s_steer_samples[s_steer_index];
    s_steer_samples[s_steer_index] = pulse;
    s_steer_sum += pulse;
    s_steer_index = (uint8_t)((s_steer_index + 1U) % COMMAND_STEER_FILTER_WINDOW);
  }
  return (uint16_t)((s_steer_sum + COMMAND_STEER_FILTER_WINDOW / 2U) / COMMAND_STEER_FILTER_WINDOW);
}

static uint8_t CommandApp_PulseInRange(uint16_t pulse_us,
                                       uint16_t minimum_us,
                                       uint16_t maximum_us)
{
  return (uint8_t)(((pulse_us >= minimum_us) &&
                    (pulse_us <= maximum_us)) ? 1U : 0U);
}

static int16_t CommandApp_PulseToPermille(uint16_t pulse_us)
{
  int32_t delta = (int32_t)pulse_us - (int32_t)COMMAND_CENTER_US;
  int32_t magnitude;

  if ((delta >= -(int32_t)COMMAND_DEADBAND_US) &&
      (delta <= (int32_t)COMMAND_DEADBAND_US))
  {
    return 0;
  }
  magnitude = (delta < 0) ? -delta : delta;
  if (magnitude > (int32_t)COMMAND_THROTTLE_MAX_OFFSET_US) magnitude = COMMAND_THROTTLE_MAX_OFFSET_US;
  magnitude = (magnitude - (int32_t)COMMAND_DEADBAND_US) * 1000 /
              (int32_t)(COMMAND_THROTTLE_MAX_OFFSET_US - COMMAND_DEADBAND_US);
  return (int16_t)((delta < 0) ? -magnitude : magnitude);
}

/* V4 MC7 CH1 endpoints 1257/1500/1757 us; deadband 45 us. */
static int16_t CommandApp_SteeringPermille(uint16_t pulse_us)
{
  int32_t axis = (int32_t)pulse_us - COMMAND_STEER_CENTER_US;
  int32_t magnitude = (axis < 0) ? -axis : axis;
  int32_t endpoint = (axis < 0) ? COMMAND_STEER_LEFT_OFFSET_US : COMMAND_STEER_RIGHT_OFFSET_US;
  if (magnitude <= COMMAND_STEER_DEADBAND_US) return 0;
  if (magnitude > endpoint) magnitude = endpoint;
  magnitude = (magnitude - COMMAND_STEER_DEADBAND_US) * 1000 / (endpoint - COMMAND_STEER_DEADBAND_US);
  return (int16_t)((axis < 0) ? -magnitude : magnitude);
}

static uint8_t CommandApp_DecodeMode(uint16_t pulse_us)
{
  if (CommandApp_PulseInRange(pulse_us,
                              COMMAND_SWITCH_LOCK_MIN_US,
                              COMMAND_SWITCH_LOCK_MAX_US) != 0U)
  {
    return ROBOT_MODE_LOCKED;
  }
  if (CommandApp_PulseInRange(pulse_us,
                              COMMAND_SWITCH_MANUAL_MIN_US,
                              COMMAND_SWITCH_MANUAL_MAX_US) != 0U)
  {
    return ROBOT_MODE_MANUAL;
  }
  if (CommandApp_PulseInRange(pulse_us,
                              COMMAND_SWITCH_AUTO_MIN_US,
                              COMMAND_SWITCH_AUTO_MAX_US) != 0U)
  {
    return ROBOT_MODE_AUTO_FC;
  }
  return COMMAND_MODE_INVALID;
}

static uint16_t CommandApp_SpeedLimit(uint16_t pulse_us)
{
  uint32_t span = COMMAND_MAX_SPEED_LIMIT_ERPM - COMMAND_MIN_SPEED_LIMIT_ERPM;

  if (pulse_us < COMMAND_LIMIT_KNOB_MIN_US)
  {
    pulse_us = COMMAND_LIMIT_KNOB_MIN_US;
  }
  if (pulse_us > COMMAND_LIMIT_KNOB_MAX_US)
  {
    pulse_us = COMMAND_LIMIT_KNOB_MAX_US;
  }
  return (uint16_t)(COMMAND_MIN_SPEED_LIMIT_ERPM +
                    ((span * (uint32_t)(pulse_us - COMMAND_LIMIT_KNOB_MIN_US)) /
                     (COMMAND_LIMIT_KNOB_MAX_US - COMMAND_LIMIT_KNOB_MIN_US)));
}

static void CommandApp_ResetSwitchQualification(void)
{
  s_stable_mode = COMMAND_MODE_INVALID;
  s_candidate_mode = COMMAND_MODE_INVALID;
  s_candidate_frames = 0U;
  s_unconfirmed_frames = 0U;
}

static void CommandApp_QualifyMode(uint8_t raw_mode)
{
  if ((raw_mode == s_stable_mode) && (raw_mode != COMMAND_MODE_INVALID))
  {
    s_candidate_mode = raw_mode;
    s_candidate_frames = 0U;
    s_unconfirmed_frames = 0U;
    return;
  }

  if (s_stable_mode != COMMAND_MODE_INVALID)
  {
    if (s_unconfirmed_frames < COMMAND_SWITCH_CONFIRM_FRAMES)
    {
      s_unconfirmed_frames++;
    }
  }

  /* 非法值和其他合法档位共用偏离计数，二者交替不能无限保持原档。 */
  if (raw_mode == COMMAND_MODE_INVALID)
  {
    s_candidate_mode = COMMAND_MODE_INVALID;
    s_candidate_frames = 0U;
  }
  else if (raw_mode != s_candidate_mode)
  {
    s_candidate_mode = raw_mode;
    s_candidate_frames = 1U;
  }
  else if (s_candidate_frames < COMMAND_SWITCH_CONFIRM_FRAMES)
  {
    s_candidate_frames++;
  }

  if (s_candidate_frames >= COMMAND_SWITCH_CONFIRM_FRAMES)
  {
    s_stable_mode = raw_mode;
    s_candidate_frames = 0U;
    s_unconfirmed_frames = 0U;
  }
  else if (s_unconfirmed_frames >= COMMAND_SWITCH_CONFIRM_FRAMES)
  {
    s_rc_diagnostics.mode_reject_count++;
    s_stable_mode = COMMAND_MODE_INVALID;
    s_unconfirmed_frames = 0U;
  }
}

static void CommandApp_UpdateLossDuration(uint32_t now_ms)
{
  uint32_t elapsed = now_ms - s_loss_start_ms;
  s_rc_diagnostics.last_loss_ms = elapsed;
  if (elapsed > s_rc_diagnostics.max_loss_ms)
  {
    s_rc_diagnostics.max_loss_ms = elapsed;
  }
  if ((elapsed >= COMMAND_REVOKE_ARM_MS) && (s_startup_lock_seen != 0U))
  {
    s_startup_lock_seen = 0U;
    s_rc_diagnostics.revoke_count++;
  }
}

static void CommandApp_RecordLoss(uint32_t now_ms)
{
  if (s_loss_active == 0U)
  {
    s_loss_active = 1U;
    s_loss_start_ms = now_ms;
  }
  CommandApp_UpdateLossDuration(now_ms);
}

static void CommandApp_Lock(uint8_t gate)
{
  g_robot_command.mode = ROBOT_MODE_LOCKED;
  g_robot_command.gate = gate;
  g_robot_command.throttle_permille = 0;
  g_robot_command.steering_permille = 0;
  s_steer_filter_valid = 0U;
  s_release_ready = 0U;
  s_center_start_ms = 0U;
}

void CommandApp_Init(void)
{
  memset(&s_rc, 0, sizeof(s_rc));
  memset(&s_rc_diagnostics, 0, sizeof(s_rc_diagnostics));
  memset(&g_robot_command, 0, sizeof(g_robot_command));
  memset(s_steer_samples, 0, sizeof(s_steer_samples));
  s_last_good_frame_count = 0U;
  s_last_lost_count = 0U;
  s_last_guard_event_count = 0U;
  s_center_start_ms = 0U;
  s_loss_start_ms = 0U;
  s_startup_lock_seen = 0U;
  s_release_ready = 0U;
  s_loss_active = 0U;
  s_steer_sum = 0U;
  s_steer_filter_ms = 0U;
  s_steer_index = 0U;
  s_steer_filter_valid = 0U;
  CommandApp_ResetSwitchQualification();
  g_robot_command.gate = ROBOT_GATE_STARTUP_LOCK_REQUIRED;
  g_robot_command.speed_limit_erpm = COMMAND_MIN_SPEED_LIMIT_ERPM;
  SBusRc_Init(&s_rc, &huart1);
}

void CommandApp_Task(uint32_t now_ms)
{
  SBusRc_Data_t data_snapshot;
  const SBusRc_Data_t *data = &data_snapshot;
  uint8_t raw_mode;
  uint8_t new_frame;
  uint8_t new_guard_event;
  uint8_t new_lost_frame;
  int16_t throttle;
  uint16_t steer_filtered;

  SBusRc_Task(&s_rc, now_ms);
  if (SBusRc_GetSnapshot(&s_rc, &data_snapshot) == 0U)
  {
    g_robot_command.rc_online = 0U;
    g_robot_command.failsafe = 1U;
    CommandApp_RecordLoss(now_ms);
    CommandApp_ResetSwitchQualification();
    CommandApp_Lock(ROBOT_GATE_RC_LOST);
    return;
  }
  s_rc_diagnostics.sbus = data_snapshot;
  new_frame = (uint8_t)((data->good_frame_count != s_last_good_frame_count) &&
                         (data->frame_lost == 0U));
  s_last_good_frame_count = data->good_frame_count;
  new_guard_event = (uint8_t)(data->guard_event_count != s_last_guard_event_count);
  s_last_guard_event_count = data->guard_event_count;
  new_lost_frame = (uint8_t)(data->lost_count != s_last_lost_count);
  s_last_lost_count = data->lost_count;

  g_robot_command.rc_online = data->online;
  g_robot_command.failsafe = (uint8_t)((data->failsafe != 0U) ||
                                       (data->guard_reason != 0U) ||
                                       (new_guard_event != 0U));
  g_robot_command.frame_count = data->frame_count;
  g_robot_command.rc_error_count = data->error_count;
  memcpy(g_robot_command.rc_pulse_us, data->pulse_us, sizeof(g_robot_command.rc_pulse_us));

  if ((data->online == 0U) || (g_robot_command.failsafe != 0U))
  {
    CommandApp_RecordLoss(now_ms);
    CommandApp_ResetSwitchQualification();
    CommandApp_Lock(ROBOT_GATE_RC_LOST);
    return;
  }
  if (s_loss_active != 0U)
  {
    /* 恢复帧也检查完整时长，避免 699 ms 异常、710 ms 恢复漏掉撤权。 */
    CommandApp_UpdateLossDuration(now_ms);
    s_loss_active = 0U;
  }

  throttle = CommandApp_PulseToPermille(data->pulse_us[COMMAND_CH_THROTTLE]);
  g_robot_command.throttle_centered = (uint8_t)((throttle == 0) ? 1U : 0U);
  g_robot_command.speed_limit_erpm =
      CommandApp_SpeedLimit(data->pulse_us[COMMAND_CH_SPEED_LIMIT]);

  raw_mode = CommandApp_DecodeMode(data->pulse_us[COMMAND_CH_MODE]);
  if ((data->frame_lost != 0U) || (new_lost_frame != 0U))
  {
    /* 丢帧中断新档连续确认；累计计数也能发现已被健康帧覆盖的短丢帧。
     * 保留偏离原档计数，非法值/丢帧/其他档交替不能无限保持原档。
     */
    s_candidate_mode = COMMAND_MODE_INVALID;
    s_candidate_frames = 0U;
  }
  if (new_frame != 0U)
  {
    CommandApp_QualifyMode(raw_mode);
  }

  if ((s_stable_mode == ROBOT_MODE_LOCKED) && (throttle == 0))
  {
    s_startup_lock_seen = 1U;
  }
  if (s_startup_lock_seen == 0U)
  {
    CommandApp_Lock(ROBOT_GATE_STARTUP_LOCK_REQUIRED);
    return;
  }
  if (s_stable_mode == COMMAND_MODE_INVALID)
  {
    CommandApp_Lock(ROBOT_GATE_MODE_CONFIRMING);
    return;
  }
  if (s_stable_mode == ROBOT_MODE_LOCKED)
  {
    CommandApp_Lock(ROBOT_GATE_READY);
    return;
  }

  if (g_robot_command.mode != s_stable_mode)
  {
    g_robot_command.mode = s_stable_mode;
    s_center_start_ms = 0U;
    s_release_ready = 0U;
    s_steer_filter_valid = 0U;
  }

  if (g_robot_command.mode == ROBOT_MODE_AUTO_FC)
  {
    s_steer_filter_valid = 0U;
    g_robot_command.steering_permille = 0;
    g_robot_command.throttle_permille = 0;
    g_robot_command.gate = ROBOT_GATE_READY;
    return;
  }

  steer_filtered = CommandApp_FilterSteering(data->pulse_us[COMMAND_CH_STEERING], now_ms);
  g_robot_command.steering_filtered_us = steer_filtered;
  g_robot_command.steering_permille = CommandApp_SteeringPermille(
      (CHASSIS_STEER_CALIBRATION_SIDE != 0U && COMMAND_CAL_USE_RAW_STEERING != 0U) ?
      data->pulse_us[COMMAND_CH_STEERING] : steer_filtered);

  if (s_release_ready == 0U)
  {
    if (throttle == 0)
    {
      if (s_center_start_ms == 0U)
      {
        s_center_start_ms = now_ms;
      }
      /* 短丢帧只能保持既有释放；新放行必须由健康新帧确认回中。 */
      if (((now_ms - s_center_start_ms) >= COMMAND_CENTER_RELEASE_MS) &&
          (new_frame != 0U))
      {
        s_release_ready = 1U;
      }
    }
    else
    {
      s_center_start_ms = 0U;
    }
  }

  if (s_release_ready == 0U)
  {
    g_robot_command.gate = ROBOT_GATE_THROTTLE_CENTERING;
    g_robot_command.throttle_permille = 0;
  }
  else
  {
    g_robot_command.gate = ROBOT_GATE_READY;
    g_robot_command.throttle_permille = throttle;
  }
}

void CommandApp_GetRcDiagnostics(CommandRcDiagnostics *out)
{
  if (out != NULL)
  {
    *out = s_rc_diagnostics;
  }
}
