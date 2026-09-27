#include "debug_app.h"
#include "debug_config.h"
#include "bsp_callback.h"
#include "command_config.h"
#include "command_app.h"
#include "chassis_config.h"

#include "bsp_uart.h"
#include "robot_def.h"
#include "vehicle_status.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

static BspUartPort s_debug_port;
static uint32_t s_last_output_ms;
/* 预留所有 32 位诊断字段的最坏十进制长度，避免整行被 snprintf 截断。 */
static char s_debug_line[768];
static uint8_t s_output_mode = DEBUG_APP_OUTPUT_MODE;
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_buffer[DEBUG_APP_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;
static volatile uint8_t s_rx_overflow;
static char s_command_line[DEBUG_APP_COMMAND_LINE_SIZE];
static uint8_t s_command_length, s_discard_line, s_reply;

/* ISR only queues bytes; parsing and formatting stay in the task. */
static void DebugApp_Rx(void *parent, UART_HandleTypeDef *huart)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) % DEBUG_APP_RX_BUFFER_SIZE);
  (void)parent;
  if (next != s_rx_tail)
  {
    s_rx_buffer[s_rx_head] = s_rx_byte;
    s_rx_head = next;
  }
  else s_rx_overflow = 1U;
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1U);
}

static void DebugApp_RxError(void *parent, UART_HandleTypeDef *huart)
{
  (void)parent;
  s_rx_overflow = 1U; /* Discard through newline; never execute a truncated command. */
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1U);
}

static void DebugApp_Command(void)
{
  if (strcmp(s_command_line, "DBG GENERIC") == 0) s_output_mode = DEBUG_APP_OUTPUT_GENERIC;
  else if (strcmp(s_command_line, "DBG OID") == 0) s_output_mode = DEBUG_APP_OUTPUT_OID;
  else if (strcmp(s_command_line, "DBG SBUS") == 0) s_output_mode = DEBUG_APP_OUTPUT_SBUS;
  else if (strcmp(s_command_line, "DBG MT6826S") == 0) s_output_mode = DEBUG_APP_OUTPUT_MT6826S;
  else if (strcmp(s_command_line, "DBG FC") == 0) s_output_mode = DEBUG_APP_OUTPUT_FC;
  else if (strcmp(s_command_line, "DBG OFF") == 0) s_output_mode = DEBUG_APP_OUTPUT_OFF;
  else if (strcmp(s_command_line, "DBG?") == 0 || strcmp(s_command_line, "DBG HELP") == 0)
  { s_reply = 2U; return; }
  else { s_reply = 3U; return; }
  s_reply = 1U;
}

static void DebugApp_PollCommands(void)
{
  unsigned budget = DEBUG_APP_COMMAND_POLL_BUDGET;
  uint8_t ch;
  if (s_rx_overflow != 0U)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rx_tail = s_rx_head;
    s_rx_overflow = 0U;
    if (primask == 0U) __enable_irq();
    s_command_length = 0U;
    s_discard_line = 1U;
  }
  while (s_rx_tail != s_rx_head && budget-- != 0U && s_reply == 0U)
  {
    ch = s_rx_buffer[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % DEBUG_APP_RX_BUFFER_SIZE);
    if (ch == '\r' || ch == '\n')
    {
      if (s_discard_line != 0U) s_reply = 3U;
      else if (s_command_length != 0U)
      {
        s_command_line[s_command_length] = '\0';
        DebugApp_Command();
      }
      s_discard_line = s_command_length = 0U;
    }
    else if (ch < 32U || ch > 126U || s_command_length >= sizeof(s_command_line) - 1U)
      s_discard_line = 1U;
    else if (s_discard_line == 0U) s_command_line[s_command_length++] = (char)ch;
  }
}

void DebugApp_Init(void)
{
  BspUartCallbackConfig callback;
  if (DEBUG_APP_ENABLE == 0U) return;
  (void)BspUart_Init(&s_debug_port, &huart7, 30U);
  if (DEBUG_APP_COMMAND_ENABLE != 0U)
  {
    memset(&callback, 0, sizeof(callback));
    callback.handle = &huart7;
    callback.rx_complete = DebugApp_Rx;
    callback.error = DebugApp_RxError;
    if (BspCallback_RegisterUart(&callback) == 0U ||
        HAL_UART_Receive_IT(&huart7, &s_rx_byte, 1U) != HAL_OK) Error_Handler();
  }
}

void DebugApp_Task(uint32_t now_ms)
{
  int length;
  uint32_t period = (s_output_mode == DEBUG_APP_OUTPUT_SBUS) ?
      DEBUG_APP_SBUS_PRINT_PERIOD_MS : ((s_output_mode == DEBUG_APP_OUTPUT_OID) ?
      DEBUG_APP_OID_PRINT_PERIOD_MS : DEBUG_APP_TEXT_PRINT_PERIOD_MS);
  if (DEBUG_APP_ENABLE == 0U) return;
  if (DEBUG_APP_COMMAND_ENABLE != 0U) DebugApp_PollCommands();

  if (s_reply == 0U && (s_output_mode == DEBUG_APP_OUTPUT_OFF ||
      (now_ms - s_last_output_ms) < period))
  {
    return;
  }
  /* 静态缓冲在中断发送完成前不能改写；忙时跳过本帧，不阻塞 1 ms OID 调度。 */
  if (BspUart_IsTxReady(&s_debug_port) == 0U)
  {
    return;
  }
  s_last_output_ms = now_ms;

  if (s_reply != 0U)
  {
    length = snprintf(s_debug_line, sizeof(s_debug_line),
        "DBG %s mode=%u readonly=1 commands=DBG GENERIC|OID|SBUS|MT6826S|FC|OFF; DBG?\r\n",
        s_reply == 3U ? "ERR" : "OK", s_output_mode);
    s_reply = 0U;
  }
  else if (s_output_mode == DEBUG_APP_OUTPUT_SBUS)
  {
    CommandRcDiagnostics diag;
    uint32_t good_age;
    CommandApp_GetRcDiagnostics(&diag);
    /* HAL 当前时间须晚于命令快照，避免 ISR 更新晚于本轮入参而出现下溢。 */
    good_age = (diag.sbus.good_frame_count != 0U) ?
        (uint32_t)(HAL_GetTick() - diag.sbus.last_good_ms) : UINT32_MAX;
    length = snprintf(s_debug_line, sizeof(s_debug_line),
        "SBUS t=%lu rc=%u fs=%u mode=%u gate=%u frames=%lu err=%lu ch1=%u filt=%u ch3=%u ch5=%u ch7=%u thr=%d steer=%d lim=%u calraw=%u ui=%u "
        "flags=%02X fl=%u rf=%u reason=%u/%u events=%lu evtms=%lu lost=%lu streak=%u fsc=%lu uart=%lu tout=%lu good=%lu goodage=%lu revoke=%lu modebad=%lu lossms=%lu/%lu\r\n",
        (unsigned long)now_ms, g_robot_command.rc_online, g_robot_command.failsafe,
        g_robot_command.mode, g_robot_command.gate, (unsigned long)g_robot_command.frame_count,
        (unsigned long)g_robot_command.rc_error_count, g_robot_command.rc_pulse_us[COMMAND_CH_STEERING],
        g_robot_command.steering_filtered_us, g_robot_command.rc_pulse_us[COMMAND_CH_THROTTLE],
        g_robot_command.rc_pulse_us[COMMAND_CH_MODE], g_robot_command.rc_pulse_us[COMMAND_CH_SPEED_LIMIT],
        g_robot_command.throttle_permille, g_robot_command.steering_permille, g_robot_command.speed_limit_erpm,
        (unsigned)(CHASSIS_STEER_CALIBRATION_SIDE != 0U && COMMAND_CAL_USE_RAW_STEERING != 0U),
        (unsigned)VehicleStatus_Get(),
        (unsigned)diag.sbus.raw_flags, diag.sbus.frame_lost, diag.sbus.failsafe,
        diag.sbus.guard_reason, diag.sbus.last_guard_reason,
        (unsigned long)diag.sbus.guard_event_count, (unsigned long)diag.sbus.last_guard_ms,
        (unsigned long)diag.sbus.lost_count, diag.sbus.lost_streak,
        (unsigned long)diag.sbus.failsafe_count, (unsigned long)diag.sbus.uart_error_count,
        (unsigned long)diag.sbus.timeout_count, (unsigned long)diag.sbus.good_frame_count,
        (unsigned long)good_age, (unsigned long)diag.revoke_count,
        (unsigned long)diag.mode_reject_count, (unsigned long)diag.last_loss_ms,
        (unsigned long)diag.max_loss_ms);
  }
  else if (s_output_mode == DEBUG_APP_OUTPUT_MT6826S)
    length = snprintf(s_debug_line, sizeof(s_debug_line),
        "MT6826S t=%lu cal=%u encgate=%u sr=%u sf=%u raw=%u/%u sh=%u/%u zero=%u/%u ang=%d/%d at=%d/%d duty=%d/%d brk=%u/%u dir=%u/%u sched=%d gain=%u en=%u\r\n",
        (unsigned long)now_ms, g_robot_chassis.steer_calibration_side, CHASSIS_STEER_CAL_REQUIRE_ENCODER_HEALTH,
        g_robot_chassis.steer_released, g_robot_chassis.steer_fault,
        g_robot_chassis.steer_raw[0], g_robot_chassis.steer_raw[1],
        g_robot_chassis.steer_healthy[0], g_robot_chassis.steer_healthy[1],
        g_robot_chassis.steer_zero_valid[0], g_robot_chassis.steer_zero_valid[1],
        g_robot_chassis.steer_actual_x10[0], g_robot_chassis.steer_actual_x10[1],
        g_robot_chassis.steer_target_x10[0], g_robot_chassis.steer_target_x10[1],
        g_robot_chassis.steer_duty[0], g_robot_chassis.steer_duty[1],
        g_robot_chassis.steer_brake[0], g_robot_chassis.steer_brake[1],
        g_robot_chassis.steer_reverse[0], g_robot_chassis.steer_reverse[1],
        g_robot_chassis.steering_scheduled_permille, g_robot_chassis.steering_speed_gain_permille,
        g_robot_chassis.motion_enabled);
  else if (s_output_mode == DEBUG_APP_OUTPUT_FC)
    length = snprintf(s_debug_line, sizeof(s_debug_line),
        "FC t=%lu mode=%u rc=%u fs=%u main1=%u/%u/%u/%u main2=%u/%u/%u/%u rel=%u cal=%u en=%u\r\n",
        (unsigned long)now_ms, g_robot_command.mode, g_robot_command.rc_online, g_robot_command.failsafe,
        g_robot_chassis.fc_drive_raw_us, g_robot_chassis.fc_drive_filtered_us,
        g_robot_chassis.fc_drive_online, g_robot_chassis.fc_drive_fault,
        g_robot_chassis.fc_steer_raw_us, g_robot_chassis.fc_steer_filtered_us,
        g_robot_chassis.fc_steer_online, g_robot_chassis.fc_steer_fault,
        g_robot_chassis.fc_release_ready, g_robot_chassis.steer_calibration_side, g_robot_chassis.motion_enabled);
  else if (s_output_mode == DEBUG_APP_OUTPUT_OID)
    length = snprintf(s_debug_line, sizeof(s_debug_line),
        "OID t=%lu id=%u/%u param=%u en=%u tgt=%ld,%ld spd=%ld,%ld online=%u/%u fault=%u/%u age=%lu/%lu hb=%lu/%lu to=%u/%u crc=%u/%u bus=%u/%u sm=%u cmd=%u/%u/%u pre=%lu "
        "rmode=%u/%u rtgt=%ld/%ld rage=%lu/%lu rto=%u/%u zwr=%u/%u sack=%u/%u wmiss=%u/%u exc=%u/%u ec=%u/%u stopf=%u cap=%u "
        "ch3=%u req=%ld rev=%u/%u rd=%u tx=%ld/%ld txt=%lu/%lu hbmax=%lu/%lu\r\n",
        (unsigned long)now_ms, CHASSIS_OID_LEFT_ID, CHASSIS_OID_RIGHT_ID,
        g_robot_chassis.parameters_confirmed, g_robot_chassis.motion_enabled,
        (long)g_robot_chassis.left_target_erpm, (long)g_robot_chassis.right_target_erpm,
        (long)g_robot_chassis.left_speed_erpm, (long)g_robot_chassis.right_speed_erpm,
        g_robot_chassis.left_online, g_robot_chassis.right_online,
        g_robot_chassis.left_fault, g_robot_chassis.right_fault,
        (unsigned long)g_robot_chassis.left_status_age_ms, (unsigned long)g_robot_chassis.right_status_age_ms,
        (unsigned long)g_robot_chassis.left_heartbeat_age_ms, (unsigned long)g_robot_chassis.right_heartbeat_age_ms,
        g_robot_chassis.left_timeout_count, g_robot_chassis.right_timeout_count,
        g_robot_chassis.left_crc_error_count, g_robot_chassis.right_crc_error_count,
        g_robot_chassis.rs485_overflow_count, g_robot_chassis.rs485_uart_error_count,
        g_robot_chassis.oid_safety_state, g_robot_chassis.oid_command_pending,
        g_robot_chassis.oid_command_step, g_robot_chassis.oid_command_urgent,
        (unsigned long)g_robot_chassis.status_preempt_count,
        g_robot_chassis.oid_read_mode[0], g_robot_chassis.oid_read_mode[1],
        (long)g_robot_chassis.oid_read_target[0], (long)g_robot_chassis.oid_read_target[1],
        (unsigned long)g_robot_chassis.oid_read_age[0], (unsigned long)g_robot_chassis.oid_read_age[1],
        g_robot_chassis.oid_read_timeouts[0], g_robot_chassis.oid_read_timeouts[1],
        g_robot_chassis.oid_zero_writes[0], g_robot_chassis.oid_zero_writes[1],
        g_robot_chassis.oid_speed_acks[0], g_robot_chassis.oid_speed_acks[1],
        g_robot_chassis.oid_write_unconfirmed[0], g_robot_chassis.oid_write_unconfirmed[1],
        g_robot_chassis.oid_exceptions[0], g_robot_chassis.oid_exceptions[1],
        g_robot_chassis.oid_last_exception[0], g_robot_chassis.oid_last_exception[1],
        g_robot_chassis.oid_stop_fault, CHASSIS_OID_COMMISSION_MAX_ERPM,
        g_robot_command.rc_pulse_us[2], (long)g_robot_chassis.oid_requested_base_erpm,
        g_robot_chassis.oid_reverse_wait, g_robot_chassis.oid_reverse_zero_pairs,
        g_robot_chassis.oid_read_active,
        (long)g_robot_chassis.oid_tx_target[0], (long)g_robot_chassis.oid_tx_target[1],
        (unsigned long)g_robot_chassis.oid_tx_ms[0], (unsigned long)g_robot_chassis.oid_tx_ms[1],
        (unsigned long)g_robot_chassis.oid_heartbeat_max_gap[0], (unsigned long)g_robot_chassis.oid_heartbeat_max_gap[1]);
  else

  length = snprintf(s_debug_line, sizeof(s_debug_line),
                    "V6 t=%lu rc=%u mode=%u gate=%u fs=%u thr=%d steer=%d lim=%u "
                    "param=%u en=%u tgt=%ld,%ld spd=%ld,%ld oid=%u/%u fault=%u/%u "
                    "age=%lu/%lu hb=%lu/%lu to=%u/%u crc=%u/%u bus=%u/%u "
                    "sm=%u cmd=%u/%u/%u pre=%lu "
                    "fc1=%u/%u/%u/%u fc2=%u/%u/%u/%u rel=%u "
                    "cal=%u sr=%u sf=%u raw=%u/%u sh=%u/%u zero=%u/%u "
                    "ang=%d/%d at=%d/%d duty=%d/%d brk=%u/%u dir=%u/%u sched=%d gain=%u\r\n",
                    (unsigned long)now_ms,
                    g_robot_command.rc_online,
                    g_robot_command.mode,
                    g_robot_command.gate,
                    g_robot_command.failsafe,
                    g_robot_command.throttle_permille,
                    g_robot_command.steering_permille,
                    g_robot_command.speed_limit_erpm,
                    g_robot_chassis.parameters_confirmed,
                    g_robot_chassis.motion_enabled,
                    (long)g_robot_chassis.left_target_erpm,
                    (long)g_robot_chassis.right_target_erpm,
                    (long)g_robot_chassis.left_speed_erpm,
                    (long)g_robot_chassis.right_speed_erpm,
                    g_robot_chassis.left_online,
                    g_robot_chassis.right_online,
                    g_robot_chassis.left_fault,
                    g_robot_chassis.right_fault,
                    (unsigned long)g_robot_chassis.left_status_age_ms,
                    (unsigned long)g_robot_chassis.right_status_age_ms,
                    (unsigned long)g_robot_chassis.left_heartbeat_age_ms,
                    (unsigned long)g_robot_chassis.right_heartbeat_age_ms,
                    g_robot_chassis.left_timeout_count,
                    g_robot_chassis.right_timeout_count,
                    g_robot_chassis.left_crc_error_count,
                    g_robot_chassis.right_crc_error_count,
                    g_robot_chassis.rs485_overflow_count,
                    g_robot_chassis.rs485_uart_error_count,
                    g_robot_chassis.oid_safety_state,
                    g_robot_chassis.oid_command_pending,
                    g_robot_chassis.oid_command_step,
                    g_robot_chassis.oid_command_urgent,
                    (unsigned long)g_robot_chassis.status_preempt_count,
                    g_robot_chassis.fc_drive_raw_us,
                    g_robot_chassis.fc_drive_filtered_us,
                    g_robot_chassis.fc_drive_online,
                    g_robot_chassis.fc_drive_fault,
                    g_robot_chassis.fc_steer_raw_us,
                    g_robot_chassis.fc_steer_filtered_us,
                    g_robot_chassis.fc_steer_online,
                    g_robot_chassis.fc_steer_fault,
                    g_robot_chassis.fc_release_ready,
                    g_robot_chassis.steer_calibration_side,
                    g_robot_chassis.steer_released,
                    g_robot_chassis.steer_fault,
                    g_robot_chassis.steer_raw[0], g_robot_chassis.steer_raw[1],
                    g_robot_chassis.steer_healthy[0], g_robot_chassis.steer_healthy[1],
                    g_robot_chassis.steer_zero_valid[0], g_robot_chassis.steer_zero_valid[1],
                    g_robot_chassis.steer_actual_x10[0], g_robot_chassis.steer_actual_x10[1],
                    g_robot_chassis.steer_target_x10[0], g_robot_chassis.steer_target_x10[1],
                    g_robot_chassis.steer_duty[0], g_robot_chassis.steer_duty[1],
                    g_robot_chassis.steer_brake[0], g_robot_chassis.steer_brake[1],
                    g_robot_chassis.steer_reverse[0], g_robot_chassis.steer_reverse[1],
                    g_robot_chassis.steering_scheduled_permille, g_robot_chassis.steering_speed_gain_permille);
  if (length > 0)
  {
    if (length >= (int)sizeof(s_debug_line))
    {
      length = (int)sizeof(s_debug_line) - 1;
    }
    (void)BspUart_WriteAsync(&s_debug_port,
                             (const uint8_t *)s_debug_line,
                             (uint16_t)length);
  }
}
