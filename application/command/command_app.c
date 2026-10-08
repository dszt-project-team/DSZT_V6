#include "command_app.h"
#include "command_config.h"
#include "chassis_config.h"
#include "robot_def.h"
#include "tim.h"

#include <string.h>

static PwmInput_Handle_t *s_drive_input;
static PwmInput_Handle_t *s_steer_input;
static CommandFcDiagnostics s_diagnostics;
static uint32_t s_invalid_count[2];
static uint32_t s_timeout_count[2];
static uint32_t s_center_pair_count[2];
static uint32_t s_center_start_ms;
static uint8_t s_center_timing;

static int16_t CommandApp_MapPulse(uint16_t pulse, uint16_t low,
    uint16_t neutral, uint16_t high, uint16_t deadband)
{
  if (pulse < (uint16_t)(neutral - deadband))
  {
    if (pulse <= low) return -1000;
    return (int16_t)(-((int32_t)(neutral - deadband - pulse) * 1000) /
                     (int32_t)(neutral - deadband - low));
  }
  if (pulse > (uint16_t)(neutral + deadband))
  {
    if (pulse >= high) return 1000;
    return (int16_t)(((int32_t)(pulse - neutral - deadband) * 1000) /
                     (int32_t)(high - neutral - deadband));
  }
  return 0;
}

static int16_t CommandApp_Drive(uint16_t pulse)
{
  return CommandApp_MapPulse(pulse, CHASSIS_FC_DRIVE_REVERSE_FULL_US,
      CHASSIS_FC_DRIVE_NEUTRAL_US, CHASSIS_FC_DRIVE_FORWARD_FULL_US,
      CHASSIS_FC_DRIVE_DEADBAND_US);
}

static int16_t CommandApp_Steer(uint16_t pulse)
{
  return CommandApp_MapPulse(pulse, CHASSIS_FC_STEER_LEFT_FULL_US,
      CHASSIS_FC_STEER_NEUTRAL_US, CHASSIS_FC_STEER_RIGHT_FULL_US,
      CHASSIS_FC_STEER_DEADBAND_US);
}

/* 回中检查直接使用脉宽死区，不能把整数映射舍入到0的边缘值当作回中。 */
static uint8_t CommandApp_InDeadband(uint16_t pulse, uint16_t neutral, uint16_t half)
{
  return (uint8_t)(pulse >= neutral - half && pulse <= neutral + half);
}

static void CommandApp_ResetRelease(uint8_t gate)
{
  g_robot_command.released = 0U;
  g_robot_command.gate = gate;
  g_robot_command.throttle_permille = 0;
  g_robot_command.steering_permille = 0;
  s_center_timing = 0U;
  s_diagnostics.center_elapsed_ms = 0U;
}

static void CommandApp_PublishInputs(void)
{
  g_robot_chassis.fc_drive_raw_us = s_diagnostics.drive.raw_us;
  g_robot_chassis.fc_drive_filtered_us = s_diagnostics.drive.filtered_us;
  g_robot_chassis.fc_steer_raw_us = s_diagnostics.steer.raw_us;
  g_robot_chassis.fc_steer_filtered_us = s_diagnostics.steer.filtered_us;
  g_robot_chassis.fc_drive_online = s_diagnostics.drive.online;
  g_robot_chassis.fc_steer_online = s_diagnostics.steer.online;
  g_robot_chassis.fc_drive_fault = (uint8_t)s_diagnostics.drive.fault;
  g_robot_chassis.fc_steer_fault = (uint8_t)s_diagnostics.steer.fault;
  g_robot_chassis.fc_release_ready = g_robot_command.released;
}

void CommandApp_Init(void)
{
  PwmInput_Config_t config;
  memset(&g_robot_command, 0, sizeof(g_robot_command));
  memset(&s_diagnostics, 0, sizeof(s_diagnostics));
  memset(s_invalid_count, 0, sizeof(s_invalid_count));
  memset(s_timeout_count, 0, sizeof(s_timeout_count));
  memset(s_center_pair_count, 0, sizeof(s_center_pair_count));
  s_center_start_ms = 0U;
  s_center_timing = 0U;
  g_robot_command.mode = (CHASSIS_STEER_CALIBRATION_SIDE == 0U) ?
      ROBOT_MODE_AUTO_FC : ROBOT_MODE_CALIBRATION;
  g_robot_command.gate = ROBOT_GATE_FC_INPUT_INVALID;
  g_robot_command.speed_limit_erpm = CHASSIS_FC_SPEED_LIMIT_ERPM;
  s_diagnostics.drive.fault = s_diagnostics.steer.fault = PWM_INPUT_FAULT_NOT_READY;

  memset(&config, 0, sizeof(config));
  config.timer = &htim3;
  config.minimum_valid_us = CHASSIS_FC_PWM_MIN_VALID_US;
  config.maximum_valid_us = CHASSIS_FC_PWM_MAX_VALID_US;
  config.timeout_ms = CHASSIS_FC_PWM_TIMEOUT_MS;
  config.transient_fault_hold_ms = CHASSIS_FC_PWM_TRANSIENT_HOLD_MS;
  config.valid_samples_to_online = CHASSIS_FC_PWM_VALID_TO_ONLINE;
  config.channel = TIM_CHANNEL_3;
  config.average_window = CHASSIS_FC_DRIVE_AVERAGE_WINDOW;
  s_drive_input = PwmInput_Register(&config);
  config.channel = TIM_CHANNEL_4;
  config.average_window = CHASSIS_FC_STEER_AVERAGE_WINDOW;
  s_steer_input = PwmInput_Register(&config);
  /* 注册失败保持未就绪，不能退化成单通道运行。 */
  CommandApp_PublishInputs();
}

void CommandApp_Task(uint32_t now_ms)
{
  uint8_t healthy, new_fault, drive_centered, steer_centered;
  uint8_t previously_online = g_robot_command.source_online;
  uint8_t new_pair;
  uint16_t steer_pulse;

  if (PwmInput_GetSnapshot(s_drive_input, &s_diagnostics.drive, now_ms) == 0U)
  {
    memset(&s_diagnostics.drive, 0, sizeof(s_diagnostics.drive));
    s_diagnostics.drive.fault = PWM_INPUT_FAULT_NOT_READY;
  }
  if (PwmInput_GetSnapshot(s_steer_input, &s_diagnostics.steer, now_ms) == 0U)
  {
    memset(&s_diagnostics.steer, 0, sizeof(s_diagnostics.steer));
    s_diagnostics.steer.fault = PWM_INPUT_FAULT_NOT_READY;
  }
  /* 累计事件在底层恢复后仍保留，不能因任务漏过瞬时异常而沿用旧的非零授权。 */
  new_fault = (uint8_t)(s_diagnostics.drive.invalid_pulse_count != s_invalid_count[0] ||
                       s_diagnostics.steer.invalid_pulse_count != s_invalid_count[1] ||
                       s_diagnostics.drive.timeout_event_count != s_timeout_count[0] ||
                       s_diagnostics.steer.timeout_event_count != s_timeout_count[1]);
  s_invalid_count[0] = s_diagnostics.drive.invalid_pulse_count;
  s_invalid_count[1] = s_diagnostics.steer.invalid_pulse_count;
  s_timeout_count[0] = s_diagnostics.drive.timeout_event_count;
  s_timeout_count[1] = s_diagnostics.steer.timeout_event_count;
  healthy = (uint8_t)(s_diagnostics.drive.online && s_diagnostics.steer.online &&
      s_diagnostics.drive.fault == PWM_INPUT_FAULT_NONE &&
      s_diagnostics.steer.fault == PWM_INPUT_FAULT_NONE && !new_fault);
  g_robot_command.source_online = healthy;
  drive_centered = (uint8_t)(CommandApp_InDeadband(s_diagnostics.drive.raw_us,
      CHASSIS_FC_DRIVE_NEUTRAL_US, CHASSIS_FC_DRIVE_DEADBAND_US) &&
      CommandApp_InDeadband(s_diagnostics.drive.filtered_us,
      CHASSIS_FC_DRIVE_NEUTRAL_US, CHASSIS_FC_DRIVE_DEADBAND_US));
  steer_centered = (uint8_t)(CommandApp_InDeadband(s_diagnostics.steer.raw_us,
      CHASSIS_FC_STEER_NEUTRAL_US, CHASSIS_FC_STEER_DEADBAND_US) &&
      CommandApp_InDeadband(s_diagnostics.steer.filtered_us,
      CHASSIS_FC_STEER_NEUTRAL_US, CHASSIS_FC_STEER_DEADBAND_US));
  g_robot_command.centered = (uint8_t)(healthy && drive_centered && steer_centered);
  if (!healthy)
  {
    if (previously_online || new_fault) g_robot_command.fault_event_count++;
    CommandApp_ResetRelease(ROBOT_GATE_FC_INPUT_INVALID);
  }
  else if (CHASSIS_FC_CONTROL_ENABLE == 0U)
  {
    CommandApp_ResetRelease(ROBOT_GATE_FC_CENTERING);
  }
  else if (CHASSIS_STEER_CALIBRATION_SIDE != 0U && !drive_centered)
  {
    /* 开环维护的MAIN1是持续停止门，而不只是初次授权条件。 */
    CommandApp_ResetRelease(ROBOT_GATE_CAL_DRIVE_NOT_CENTERED);
  }
  else
  {
    if (!g_robot_command.released)
    {
      if (!g_robot_command.centered)
      {
        CommandApp_ResetRelease(ROBOT_GATE_FC_CENTERING);
      }
      else
      {
        if (!s_center_timing)
        {
          s_center_timing = 1U;
          s_center_start_ms = now_ms;
          s_center_pair_count[0] = s_diagnostics.drive.valid_pulse_count;
          s_center_pair_count[1] = s_diagnostics.steer.valid_pulse_count;
        }
        s_diagnostics.center_elapsed_ms = now_ms - s_center_start_ms;
        new_pair = (uint8_t)(s_diagnostics.drive.valid_pulse_count != s_center_pair_count[0] &&
                            s_diagnostics.steer.valid_pulse_count != s_center_pair_count[1]);
        if (new_pair)
        {
          s_center_pair_count[0] = s_diagnostics.drive.valid_pulse_count;
          s_center_pair_count[1] = s_diagnostics.steer.valid_pulse_count;
          if (s_diagnostics.center_elapsed_ms >= CHASSIS_FC_RELEASE_CENTER_MS)
            g_robot_command.released = 1U;
        }
        g_robot_command.gate = g_robot_command.released ? ROBOT_GATE_READY : ROBOT_GATE_FC_CENTERING;
      }
    }
    if (g_robot_command.released)
    {
      steer_pulse = (CHASSIS_STEER_CALIBRATION_SIDE != 0U && COMMAND_FC_CAL_USE_RAW_STEERING != 0U) ?
          s_diagnostics.steer.raw_us : s_diagnostics.steer.filtered_us;
      g_robot_command.steering_permille = CommandApp_Steer(steer_pulse);
      g_robot_command.throttle_permille = (CHASSIS_STEER_CALIBRATION_SIDE == 0U) ?
          CommandApp_Drive(s_diagnostics.drive.filtered_us) : 0;
      g_robot_command.gate = ROBOT_GATE_READY;
    }
  }
  CommandApp_PublishInputs();
}

void CommandApp_GetFcDiagnostics(CommandFcDiagnostics *out)
{
  if (out != 0) *out = s_diagnostics;
}
