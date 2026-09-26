#include "lighting_app.h"
#include "lighting_config.h"

#include "robot_def.h"
#include "tim.h"
#include "ws2812_strip.h"

static WS2812Strip_Handle_t s_left_strip;
static WS2812Strip_Handle_t s_right_strip;
static uint32_t s_last_refresh_ms;

static void LightingApp_SelectColor(uint32_t now_ms,
                                    uint8_t *red,
                                    uint8_t *green,
                                    uint8_t *blue)
{
  *red = 0U;
  *green = 0U;
  *blue = 0U;

  if (g_robot_command.rc_online == 0U)
  {
    *red = 24U;
    *green = (uint8_t)(((now_ms / 300U) & 1U) ? 8U : 0U);
  }
  else if ((g_robot_chassis.left_fault != 0U) ||
           (g_robot_chassis.right_fault != 0U))
  {
    *red = (uint8_t)(((now_ms / 150U) & 1U) ? 32U : 0U);
  }
  else if ((g_robot_command.mode == ROBOT_MODE_AUTO_FC) &&
           ((g_robot_chassis.fc_drive_online == 0U) ||
            (g_robot_chassis.fc_steer_online == 0U) ||
            (g_robot_chassis.fc_drive_fault != 0U) ||
            (g_robot_chassis.fc_steer_fault != 0U)))
  {
    *red = (uint8_t)(((now_ms / 150U) & 1U) ? 32U : 0U);
    *blue = *red;
  }
  else if (g_robot_chassis.parameters_confirmed == 0U)
  {
    *red = 12U;
    *blue = 24U;
  }
  else if (g_robot_command.mode == ROBOT_MODE_MANUAL)
  {
    *green = 24U;
  }
  else if (g_robot_command.mode == ROBOT_MODE_AUTO_FC)
  {
    *green = 16U;
    *blue = 24U;
  }
  else
  {
    *red = 24U;
  }
}

void LightingApp_Init(void)
{
  if (LIGHTING_APP_ENABLE == 0U) return;
  (void)WS2812Strip_Init(&s_left_strip, &htim1, TIM_CHANNEL_1,
                         LIGHTING_APP_LED_COUNT, LIGHTING_APP_LEFT_REVERSED,
                         LIGHTING_APP_WS2812_ZERO_COMPARE, LIGHTING_APP_WS2812_ONE_COMPARE);
  (void)WS2812Strip_Init(&s_right_strip, &htim1, TIM_CHANNEL_4,
                         LIGHTING_APP_LED_COUNT, LIGHTING_APP_RIGHT_REVERSED,
                         LIGHTING_APP_WS2812_ZERO_COMPARE, LIGHTING_APP_WS2812_ONE_COMPARE);
}

void LightingApp_Task(uint32_t now_ms)
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  if (LIGHTING_APP_ENABLE == 0U) return;

  WS2812Strip_Task(&s_left_strip, now_ms);
  WS2812Strip_Task(&s_right_strip, now_ms);
  if ((now_ms - s_last_refresh_ms) < LIGHTING_APP_FRAME_PERIOD_MS)
  {
    return;
  }
  s_last_refresh_ms = now_ms;

  LightingApp_SelectColor(now_ms, &red, &green, &blue);
  WS2812Strip_FillRgb(&s_left_strip, red, green, blue);
  WS2812Strip_FillRgb(&s_right_strip, red, green, blue);
  (void)WS2812Strip_Refresh(&s_left_strip);
  (void)WS2812Strip_Refresh(&s_right_strip);
}
