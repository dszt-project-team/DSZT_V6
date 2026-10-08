#include "lighting_app.h"
#include "lighting_config.h"

#include "robot_def.h"
#include "vehicle_status.h"
#include "tim.h"
#include "ws2812_strip.h"
#include <string.h>

#if (LIGHTING_APP_LED_COUNT < 1U) || (LIGHTING_APP_LED_COUNT > WS2812_STRIP_MAX_LED_COUNT)
#error "Lighting LED count must be within strip capacity"
#endif
#if (LIGHTING_APP_FRAME_PERIOD_MS < 20U) || (LIGHTING_APP_FRAME_PERIOD_MS > 1000U)
#error "Lighting refresh period must be in 20..1000 ms"
#endif
#if (LIGHTING_APP_MAX_BRIGHTNESS < 1U) || (LIGHTING_APP_MAX_BRIGHTNESS > 40U)
#error "Lighting brightness must be in 1..40"
#endif
#if (LIGHTING_APP_STATUS_END_PIXELS < 1U) || (LIGHTING_APP_STATUS_END_PIXELS > WS2812_STRIP_MAX_LED_COUNT)
#error "Lighting status ends must reserve 1..maximum pixel count"
#endif
#if (LIGHTING_APP_MOTION_SEGMENT_PIXELS < 1U) || (LIGHTING_APP_MOTION_SEGMENT_PIXELS > WS2812_STRIP_MAX_LED_COUNT)
#error "Lighting segment length must be within strip capacity"
#endif
#if (LIGHTING_APP_MOTION_DEADBAND_ERPM < 1L) || (LIGHTING_APP_MOTION_DEADBAND_ERPM > 65535L)
#error "Lighting motion deadband must be in 1..65535 ERPM"
#endif
#if (LIGHTING_APP_MOTION_CYCLE_MS < 100U) || (LIGHTING_APP_MOTION_CYCLE_MS > 60000U)
#error "Lighting motion cycle must be in 100..60000 ms"
#endif
#if (LIGHTING_APP_TURN_ENTER_PERMILLE < 1) || (LIGHTING_APP_TURN_ENTER_PERMILLE > 1000) || \
    (LIGHTING_APP_TURN_EXIT_PERMILLE < 0) || (LIGHTING_APP_TURN_EXIT_PERMILLE >= LIGHTING_APP_TURN_ENTER_PERMILLE)
#error "Lighting turn hysteresis must satisfy 0 <= exit < enter <= 1000"
#endif
#if (LIGHTING_APP_TURN_HALF_PERIOD_MS < 100U) || (LIGHTING_APP_TURN_HALF_PERIOD_MS > 5000U)
#error "Lighting turn half period must be in 100..5000 ms"
#endif
#if (LIGHTING_APP_TURN_MIN_BRIGHTNESS < 1U) || \
    (LIGHTING_APP_TURN_MIN_BRIGHTNESS > LIGHTING_APP_TURN_PEAK_BRIGHTNESS) || \
    (LIGHTING_APP_TURN_PEAK_BRIGHTNESS > 40U)
#error "Lighting turn brightness must satisfy 1 <= min <= peak <= 40"
#endif
#if (LIGHTING_APP_TURN_FEATHER_PIXELS < 1U) || \
    (LIGHTING_APP_TURN_FEATHER_PIXELS > WS2812_STRIP_MAX_LED_COUNT)
#error "Lighting turn feather must be within strip capacity"
#endif
#if (LIGHTING_APP_WAIT_CYCLE_MS < 200U) || (LIGHTING_APP_WAIT_CYCLE_MS > 60000U)
#error "Lighting wait cycle must be in 200..60000 ms"
#endif

static WS2812Strip_Handle_t s_left_strip;
static WS2812Strip_Handle_t s_right_strip;
static uint8_t s_left_ready;
static uint8_t s_right_ready;
static uint8_t s_refresh_started;
static VehicleStatus s_status;
static uint32_t s_last_refresh_ms;
static uint32_t s_status_start_ms;
static uint32_t s_motion_start_ms;
static uint32_t s_turn_start_ms;
static int8_t s_motion_direction;
static int8_t s_turn_side;

static uint8_t LightingApp_Limit(uint32_t value)
{
  return (uint8_t)((value > LIGHTING_APP_MAX_BRIGHTNESS) ? LIGHTING_APP_MAX_BRIGHTNESS : value);
}

static uint8_t LightingApp_DoublePulse(uint32_t elapsed_ms)
{
  uint32_t phase = elapsed_ms % 1200U;
  return ((phase < 100U) || ((phase >= 200U) && (phase < 300U))) ? 1U : 0U;
}

/* 0..1024定点平滑曲线：首尾斜率为0，避免呼吸峰谷和光带边缘突变；无需浮点或查表。 */
static uint32_t LightingApp_SmoothLevel(uint32_t value)
{
  if (value >= 1024U) return 1024U;
  return ((value * value / 1024U) * (3072U - 2U * value)) / 1024U;
}

static void LightingApp_RenderTurn(WS2812Strip_Handle_t *strip, uint16_t first,
                                  uint16_t span, uint32_t elapsed_ms)
{
  uint32_t phase = elapsed_ms % (2U * LIGHTING_APP_TURN_HALF_PERIOD_MS);
  uint32_t level;
  uint32_t feather;
  uint16_t i;
  uint16_t distance;
  uint8_t red;
  if (phase > LIGHTING_APP_TURN_HALF_PERIOD_MS)
    phase = 2U * LIGHTING_APP_TURN_HALF_PERIOD_MS - phase;
  level = LIGHTING_APP_TURN_MIN_BRIGHTNESS +
      ((LIGHTING_APP_TURN_PEAK_BRIGHTNESS - LIGHTING_APP_TURN_MIN_BRIGHTNESS) *
       LightingApp_SmoothLevel(phase * 1024U / LIGHTING_APP_TURN_HALF_PERIOD_MS)) / 1024U;
  for (i = 0U; i < span; i++)
  {
    /* 整段琥珀轮廓，两端对称羽化；不做中段矩形闪烁，也不覆盖安全状态端点。 */
    distance = (uint16_t)(i + 1U);
    if (distance > (uint16_t)(span - i)) distance = (uint16_t)(span - i);
    feather = LightingApp_SmoothLevel((uint32_t)distance * 1024U / LIGHTING_APP_TURN_FEATHER_PIXELS);
    red = LightingApp_Limit(level * feather / 1024U);
    WS2812Strip_SetPixelRgb(strip, (uint16_t)(first + i), red, (uint8_t)(red * 3U / 8U), 0U);
  }
}

static WS2812Strip_Color_t LightingApp_BaseColor(VehicleStatus status, uint32_t elapsed_ms)
{
  WS2812Strip_Color_t color = {0U, 0U, 0U};
  uint32_t phase;
  uint32_t level;
  switch (status)
  {
    case VEHICLE_STATUS_STEER_FAULT:
      if (LightingApp_DoublePulse(elapsed_ms) != 0U) { color.r = 32U; color.b = 20U; }
      break;
    case VEHICLE_STATUS_OID_FAULT:
      color.r = ((elapsed_ms % 1000U) < 500U) ? 32U : 0U;
      break;
    case VEHICLE_STATUS_FC_FAULT:
      if ((elapsed_ms % 1000U) < 500U) { color.r = 16U; color.b = 32U; }
      break;
    case VEHICLE_STATUS_CONFIG_HOLD:
      color.r = 12U; color.b = 24U;
      break;
    case VEHICLE_STATUS_RELEASE_WAIT:
      phase = elapsed_ms % LIGHTING_APP_WAIT_CYCLE_MS;
      if (phase > (LIGHTING_APP_WAIT_CYCLE_MS / 2U)) phase = LIGHTING_APP_WAIT_CYCLE_MS - phase;
      level = 4U + ((phase * 12U) / (LIGHTING_APP_WAIT_CYCLE_MS / 2U));
      color.r = (uint8_t)level; color.g = (uint8_t)(level * 2U / 5U);
      break;
    case VEHICLE_STATUS_CALIBRATION:
      color.r = 6U; color.b = 12U;
      break;
    case VEHICLE_STATUS_AUTO_READY:
      color.g = 3U; color.b = 16U;
      break;
    default:
      color.r = 12U; color.b = 24U;
      break;
  }
  color.r = LightingApp_Limit(color.r);
  color.g = LightingApp_Limit(color.g);
  color.b = LightingApp_Limit(color.b);
  return color;
}

static uint8_t LightingApp_IsReady(VehicleStatus status)
{
  return (status == VEHICLE_STATUS_AUTO_READY) ? 1U : 0U;
}

/* 只显示最终目标的方向，不把指令动画当作编码器实测速率。64 位求和避免诊断异常值溢出。 */
static int8_t LightingApp_MotionDirection(VehicleStatus status)
{
  int64_t mean;
  if ((LIGHTING_APP_MOTION_EFFECT_ENABLE == 0U) || (LightingApp_IsReady(status) == 0U) ||
      (g_robot_chassis.motion_enabled == 0U)) return 0;
  mean = ((int64_t)g_robot_chassis.left_target_erpm + (int64_t)g_robot_chassis.right_target_erpm) / 2;
  if (mean > LIGHTING_APP_MOTION_DEADBAND_ERPM) return 1;
  if (mean < -LIGHTING_APP_MOTION_DEADBAND_ERPM) return -1;
  return 0;
}

static int8_t LightingApp_TurnSide(VehicleStatus status)
{
  int16_t command = g_robot_chassis.steering_scheduled_permille;
  if ((LIGHTING_APP_TURN_SIGNAL_ENABLE == 0U) || (LightingApp_IsReady(status) == 0U) ||
      (g_robot_chassis.steer_released == 0U) || (command == 0)) return 0;
  if (command <= -LIGHTING_APP_TURN_ENTER_PERMILLE) return -1;
  if (command >= LIGHTING_APP_TURN_ENTER_PERMILLE) return 1;
  if ((s_turn_side < 0) && (command <= -LIGHTING_APP_TURN_EXIT_PERMILLE)) return -1;
  if ((s_turn_side > 0) && (command >= LIGHTING_APP_TURN_EXIT_PERMILLE)) return 1;
  return 0;
}

static void LightingApp_Render(WS2812Strip_Handle_t *strip, uint8_t ready,
                               int8_t physical_side, uint32_t now_ms)
{
  WS2812Strip_Color_t base;
  uint16_t first;
  uint16_t span;
  uint16_t step;
  uint16_t lead;
  uint16_t index;
  uint16_t i;
  uint8_t brightness;
  if (ready == 0U) return;
  /* 不等待 DMA，不修改在途 dma_buffer；像素缓存也等通道空闲后再重绘。 */
  if (strip->dma_busy != 0U) return;
  base = LightingApp_BaseColor(s_status, (uint32_t)(now_ms - s_status_start_ms));
  WS2812Strip_FillRgb(strip, base.r, base.g, base.b);
  if ((LightingApp_IsReady(s_status) != 0U) &&
      (strip->led_count > (2U * LIGHTING_APP_STATUS_END_PIXELS)))
  {
    first = LIGHTING_APP_STATUS_END_PIXELS;
    span = (uint16_t)(strip->led_count - (2U * first));
    if (s_turn_side == physical_side)
      LightingApp_RenderTurn(strip, first, span, (uint32_t)(now_ms - s_turn_start_ms));
    /* 白色光段最后叠加，转弯时也保留清楚的前后方向，不被琥珀色染色或遮挡。 */
    if (s_motion_direction != 0)
    {
      step = (uint16_t)((((uint32_t)(now_ms - s_motion_start_ms) % LIGHTING_APP_MOTION_CYCLE_MS) * span) /
                        LIGHTING_APP_MOTION_CYCLE_MS);
      lead = (s_motion_direction > 0) ? (uint16_t)(first + span - 1U - step) : (uint16_t)(first + step);
      for (i = 0U; (i < LIGHTING_APP_MOTION_SEGMENT_PIXELS) && (i < span); i++)
      {
        if (s_motion_direction > 0)
        {
          index = (uint16_t)(lead + i);
          if (index >= first + span) break;
        }
        else
        {
          if (lead < first + i) break;
          index = (uint16_t)(lead - i);
        }
        brightness = LightingApp_Limit((32U * (LIGHTING_APP_MOTION_SEGMENT_PIXELS - i)) /
                                       LIGHTING_APP_MOTION_SEGMENT_PIXELS);
        WS2812Strip_SetPixelRgb(strip, index, brightness, brightness, brightness);
      }
    }
  }
  (void)WS2812Strip_Refresh(strip);
}

void LightingApp_Init(void)
{
  memset(&s_left_strip, 0, sizeof(s_left_strip));
  memset(&s_right_strip, 0, sizeof(s_right_strip));
  s_left_ready = 0U;
  s_right_ready = 0U;
  s_refresh_started = 0U;
  s_motion_direction = 0;
  s_turn_side = 0;
  if (LIGHTING_APP_ENABLE == 0U) return;
  s_left_ready = (WS2812Strip_Init(&s_left_strip, &htim1, TIM_CHANNEL_1,
                    LIGHTING_APP_LED_COUNT, LIGHTING_APP_LEFT_REVERSED,
                    LIGHTING_APP_WS2812_ZERO_COMPARE, LIGHTING_APP_WS2812_ONE_COMPARE) == HAL_OK) ? 1U : 0U;
  s_right_ready = (WS2812Strip_Init(&s_right_strip, &htim1, TIM_CHANNEL_4,
                    LIGHTING_APP_LED_COUNT, LIGHTING_APP_RIGHT_REVERSED,
                    LIGHTING_APP_WS2812_ZERO_COMPARE, LIGHTING_APP_WS2812_ONE_COMPARE) == HAL_OK) ? 1U : 0U;
}

void LightingApp_Task(uint32_t now_ms)
{
  VehicleStatus status;
  int8_t direction;
  int8_t turn;
  if (LIGHTING_APP_ENABLE == 0U) return;
  if (s_left_ready != 0U) WS2812Strip_Task(&s_left_strip, now_ms);
  if (s_right_ready != 0U) WS2812Strip_Task(&s_right_strip, now_ms);
  if ((s_refresh_started != 0U) &&
      ((uint32_t)(now_ms - s_last_refresh_ms) < LIGHTING_APP_FRAME_PERIOD_MS)) return;
  s_last_refresh_ms = now_ms;
  status = VehicleStatus_Get();
  if ((s_refresh_started == 0U) || (status != s_status))
  {
    s_status = status;
    s_status_start_ms = now_ms;
  }
  s_refresh_started = 1U;
  direction = LightingApp_MotionDirection(status);
  turn = LightingApp_TurnSide(status);
  if (direction != s_motion_direction) { s_motion_direction = direction; s_motion_start_ms = now_ms; }
  if (turn != s_turn_side) { s_turn_side = turn; s_turn_start_ms = now_ms; }
  LightingApp_Render(&s_left_strip, s_left_ready, -1, now_ms);
  LightingApp_Render(&s_right_strip, s_right_ready, 1, now_ms);
}
