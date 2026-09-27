/* Real lighting application and shared state classifier; strip IO is mocked. */
#include <limits.h>
#include <stdio.h>
#include <string.h>
#ifdef LIGHTING_TEST_ZERO_EXIT
#include "../application/lighting/lighting_config.h"
#undef LIGHTING_APP_TURN_EXIT_PERMILLE
#define LIGHTING_APP_TURN_EXIT_PERMILLE 0
#endif
#include "../application/lighting/lighting_app.c"

RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
TIM_HandleTypeDef htim1;
static unsigned failures, out_of_bounds, refreshes[2], tasks[2];
static uint8_t fail_left_init, fail_right_init;

static unsigned side(const WS2812Strip_Handle_t *strip)
{ return strip == &s_left_strip ? 0U : 1U; }
HAL_StatusTypeDef WS2812Strip_Init(WS2812Strip_Handle_t *strip, TIM_HandleTypeDef *timer,
                                  uint32_t channel, uint16_t count, uint8_t reversed,
                                  uint16_t zero, uint16_t one)
{
  memset(strip, 0, sizeof(*strip));
  strip->htim = timer; strip->channel = channel; strip->led_count = count;
  strip->reversed = reversed; strip->zero_compare = zero; strip->one_compare = one;
  return ((side(strip) == 0U) ? fail_left_init : fail_right_init) ? HAL_ERROR : HAL_OK;
}
void WS2812Strip_Task(WS2812Strip_Handle_t *strip, uint32_t now)
{ (void)now; tasks[side(strip)]++; }
void WS2812Strip_SetPixelRgb(WS2812Strip_Handle_t *strip, uint16_t index,
                             uint8_t red, uint8_t green, uint8_t blue)
{
  if ((index >= strip->led_count) || (index >= WS2812_STRIP_MAX_LED_COUNT))
  { out_of_bounds++; return; }
  strip->pixels[index].r = red; strip->pixels[index].g = green; strip->pixels[index].b = blue;
}
void WS2812Strip_FillRgb(WS2812Strip_Handle_t *strip, uint8_t red, uint8_t green, uint8_t blue)
{
  uint16_t i;
  for (i = 0U; i < strip->led_count; i++) WS2812Strip_SetPixelRgb(strip, i, red, green, blue);
}
HAL_StatusTypeDef WS2812Strip_Refresh(WS2812Strip_Handle_t *strip)
{ refreshes[side(strip)]++; return strip->dma_busy ? HAL_BUSY : HAL_OK; }

static void check(int pass, const char *name)
{ printf("%s: %s\n", pass ? "PASS" : "FAIL", name); if (!pass) failures++; }
static int color_is(const WS2812Strip_Handle_t *strip, uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{ return strip->pixels[index].r == r && strip->pixels[index].g == g && strip->pixels[index].b == b; }
static int all_color(const WS2812Strip_Handle_t *strip, uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t i;
  for (i = 0U; i < strip->led_count; i++) if (!color_is(strip, i, r, g, b)) return 0;
  return 1;
}
static int find_white_lead(const WS2812Strip_Handle_t *strip)
{
  uint16_t i;
  for (i = 0U; i < strip->led_count; i++) if (color_is(strip, i, 32U, 32U, 32U)) return (int)i;
  return -1;
}
static void reset(void)
{
  memset(&g_robot_command, 0, sizeof(g_robot_command));
  memset(&g_robot_chassis, 0, sizeof(g_robot_chassis));
  memset(refreshes, 0, sizeof(refreshes)); memset(tasks, 0, sizeof(tasks));
  fail_left_init = fail_right_init = 0U;
  g_robot_command.rc_online = 1U;
  g_robot_command.mode = ROBOT_MODE_MANUAL;
  g_robot_command.gate = ROBOT_GATE_READY;
  g_robot_chassis.parameters_confirmed = 1U;
  g_robot_chassis.motion_enabled = 1U;
  g_robot_chassis.steer_released = 1U;
  g_robot_chassis.left_online = g_robot_chassis.right_online = 1U;
  g_robot_chassis.steer_healthy[0] = g_robot_chassis.steer_healthy[1] = 1U;
  g_robot_chassis.steer_zero_valid[0] = g_robot_chassis.steer_zero_valid[1] = 1U;
  g_robot_chassis.fc_drive_online = g_robot_chassis.fc_steer_online = 1U;
  g_robot_chassis.fc_release_ready = 1U;
  LightingApp_Init();
}

static void test_motion(void)
{
  reset();
  LightingApp_Task(0U);
  check(all_color(&s_left_strip, 0U, 12U, 8U) && all_color(&s_right_strip, 0U, 12U, 8U),
        "manual ready at zero target is stable low-brightness teal");
  check(s_left_strip.reversed == 0U && s_right_strip.reversed == 0U &&
        s_left_strip.channel == TIM_CHANNEL_1 && s_right_strip.channel == TIM_CHANNEL_4 &&
        s_left_strip.zero_compare == 70U && s_right_strip.one_compare == 140U,
        "both head-input strips use forward pixel order and unchanged electrical timing");
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 500;
  LightingApp_Task(50U);
  check(find_white_lead(&s_left_strip) == 18 && find_white_lead(&s_right_strip) == 18,
        "forward command starts white flow at rear interior on both sides");
  LightingApp_Task(150U);
  check(find_white_lead(&s_left_strip) == 17 && find_white_lead(&s_right_strip) == 17,
        "forward flow progresses rear to head with decreasing indices");
  check(color_is(&s_left_strip, 0U, 0U, 12U, 8U) && color_is(&s_left_strip, 20U, 0U, 12U, 8U),
        "movement never covers end status pixels");
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = -500;
  LightingApp_Task(200U);
  check(find_white_lead(&s_left_strip) == 2 && find_white_lead(&s_right_strip) == 2,
        "reverse command starts white flow at head interior on both sides");
  LightingApp_Task(300U);
  check(find_white_lead(&s_left_strip) == 3 && find_white_lead(&s_right_strip) == 3,
        "reverse white flow progresses head to rear with increasing indices");
  g_robot_chassis.left_target_erpm = 50; g_robot_chassis.right_target_erpm = 50;
  LightingApp_Task(350U);
  check(all_color(&s_left_strip, 0U, 12U, 8U), "50 ERPM command deadband suppresses movement flow");
  g_robot_chassis.left_target_erpm = 100; g_robot_chassis.right_target_erpm = 100;
  LightingApp_Task(400U);
  check(s_motion_direction == 1, "CH7 minimum 100 ERPM still displays movement direction");
  g_robot_chassis.left_target_erpm = 4900; g_robot_chassis.right_target_erpm = -4900;
  LightingApp_Task(450U);
  check(all_color(&s_left_strip, 0U, 12U, 8U), "opposed targets with zero mean do not imply translation");
  g_robot_chassis.left_target_erpm = INT32_MAX; g_robot_chassis.right_target_erpm = INT32_MAX;
  LightingApp_Task(500U);
  check(s_motion_direction == 1, "diagnostic extreme target sum does not overflow signed 32-bit");
  g_robot_command.mode = ROBOT_MODE_AUTO_FC;
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 0;
  LightingApp_Task(550U);
  check(all_color(&s_left_strip, 0U, 3U, 16U), "automatic ready at zero target is stable blue");
}

static void test_turn(void)
{
  uint8_t level[9];
  uint16_t i;
  int symmetric = 1;
  int amber = 1;
  reset();
  g_robot_chassis.steering_scheduled_permille = -130;
  LightingApp_Task(0U);
  check(color_is(&s_left_strip, 10U, 6U, 2U, 0U) && all_color(&s_right_strip, 0U, 12U, 8U),
        "negative steering starts dim amber breath only on left interior");
  check(color_is(&s_left_strip, 0U, 0U, 12U, 8U) && color_is(&s_left_strip, 1U, 0U, 12U, 8U) &&
        color_is(&s_left_strip, 19U, 0U, 12U, 8U) && color_is(&s_left_strip, 20U, 0U, 12U, 8U),
        "turn breath preserves all four reserved status pixels");
  level[0] = s_left_strip.pixels[10].r;
  for (i = 1U; i < 9U; i++)
  {
    LightingApp_Task((uint32_t)i * 150U);
    level[i] = s_left_strip.pixels[10].r;
    if (!all_color(&s_right_strip, 0U, 12U, 8U)) amber = 0;
    if (i == 4U)
    {
      uint16_t pixel;
      for (pixel = 2U; pixel <= 18U; pixel++)
      {
        WS2812Strip_Color_t a = s_left_strip.pixels[pixel];
        WS2812Strip_Color_t b = s_left_strip.pixels[20U - pixel];
        if ((a.r != b.r) || (a.g != b.g) || (a.b != b.b)) symmetric = 0;
        if ((a.r == 0U) || (a.g != (uint8_t)(a.r * 3U / 8U)) || (a.b != 0U)) amber = 0;
      }
      check(s_left_strip.pixels[2].r < s_left_strip.pixels[3].r &&
            s_left_strip.pixels[3].r < s_left_strip.pixels[4].r &&
            s_left_strip.pixels[4].r == 28U && s_left_strip.pixels[16].r == 28U,
            "three-pixel turn feather reaches full strength away from both ends");
    }
  }
  check(level[0] == 6U && level[4] == 28U && level[8] == 6U,
        "turn breath reaches 6 at start, 28 at 600 ms, and 6 at 1200 ms");
  check(level[0] < level[1] && level[1] < level[2] && level[2] < level[3] && level[3] < level[4] &&
        level[4] > level[5] && level[5] > level[6] && level[6] > level[7] && level[7] > level[8],
        "turn light rises and falls progressively without hard on-off phases");
  check(level[1] == level[7] && level[2] == level[6] && level[3] == level[5] &&
        (level[1] - level[0]) < (level[2] - level[1]) &&
        (level[4] - level[3]) < (level[3] - level[2]),
        "turn breathing is time-symmetric with eased minimum and maximum");
  check(symmetric && amber, "turn feather is spatially symmetric amber and never changes opposite strip");
  g_robot_chassis.steering_scheduled_permille = -90;
  LightingApp_Task(1250U);
  check(s_turn_side == -1, "turn signal holds through 60..120 hysteresis band");
  g_robot_chassis.steering_scheduled_permille =
      (LIGHTING_APP_TURN_EXIT_PERMILLE > 0) ? (1 - LIGHTING_APP_TURN_EXIT_PERMILLE) : 0;
  LightingApp_Task(1300U);
  check(s_turn_side == 0 && all_color(&s_left_strip, 0U, 12U, 8U),
        "turn signal exits to base below threshold or at exact zero with zero-exit config");
  g_robot_chassis.steering_scheduled_permille = 130;
  LightingApp_Task(1350U);
  check(color_is(&s_right_strip, 10U, 6U, 2U, 0U) && all_color(&s_left_strip, 0U, 12U, 8U),
        "positive steering starts right amber breath with left base restored");
  g_robot_command.mode = ROBOT_MODE_AUTO_FC;
  g_robot_chassis.steering_scheduled_permille = -130;
  LightingApp_Task(1400U);
  check(color_is(&s_left_strip, 10U, 6U, 2U, 0U) && all_color(&s_right_strip, 0U, 3U, 16U),
        "automatic steering reversal restarts new side gently and clears former side");
  g_robot_chassis.steer_released = 0U;
  LightingApp_Task(1450U);
  check(s_turn_side == 0 && s_status == VEHICLE_STATUS_RELEASE_WAIT,
        "unreleased steering cannot display turn-ready overlay");
}

static void test_turn_motion_layers(void)
{
  reset();
  g_robot_chassis.steering_scheduled_permille = -500;
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 500;
  LightingApp_Task(0U);
  LightingApp_Task(300U);
  check(color_is(&s_left_strip, 13U, 32U, 32U, 32U) &&
        color_is(&s_left_strip, 14U, 21U, 21U, 21U) &&
        color_is(&s_left_strip, 15U, 10U, 10U, 10U),
        "forward white gradient is drawn above amber turn layer without color tint");
  check(color_is(&s_left_strip, 10U, 17U, 6U, 0U) &&
        color_is(&s_right_strip, 10U, 0U, 12U, 8U) && find_white_lead(&s_right_strip) == 13,
        "turning while driving keeps amber elsewhere and white movement on opposite side");
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = -500;
  LightingApp_Task(350U);
  LightingApp_Task(500U);
  check(color_is(&s_left_strip, 4U, 32U, 32U, 32U) &&
        color_is(&s_left_strip, 3U, 21U, 21U, 21U) &&
        color_is(&s_left_strip, 2U, 10U, 10U, 10U) && find_white_lead(&s_right_strip) == 4,
        "reverse white gradient overrides feathered amber without changing opposite direction");
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 0;
  LightingApp_Task(550U);
  check(find_white_lead(&s_left_strip) == -1 && s_left_strip.pixels[10].r > 6U &&
        all_color(&s_right_strip, 0U, 12U, 8U),
        "stopping removes white segment without interrupting remaining turn breath");
  g_robot_chassis.steering_scheduled_permille = 0;
  LightingApp_Task(600U);
  check(all_color(&s_left_strip, 0U, 12U, 8U) && all_color(&s_right_strip, 0U, 12U, 8U),
        "centering after stopping removes all motion overlays");
}

static void test_turn_short_and_wrap(void)
{
  uint16_t count;
  uint16_t i;
  int ends_ok = 1;
  int symmetric = 1;
  int within_limit = 1;
  for (count = 1U; count <= WS2812_STRIP_MAX_LED_COUNT; count++)
  {
    reset();
    s_left_strip.led_count = s_right_strip.led_count = count;
    g_robot_chassis.steering_scheduled_permille = -500;
    LightingApp_Task(0U);
    LightingApp_Task(600U);
    if (!all_color(&s_right_strip, 0U, 12U, 8U)) ends_ok = 0;
    for (i = 0U; i < count; i++)
    {
      WS2812Strip_Color_t a = s_left_strip.pixels[i];
      WS2812Strip_Color_t b = s_left_strip.pixels[count - 1U - i];
      if ((count <= 4U) || (i < 2U) || (i >= count - 2U))
      {
        if (!color_is(&s_left_strip, i, 0U, 12U, 8U)) ends_ok = 0;
      }
      if ((a.r != b.r) || (a.g != b.g) || (a.b != b.b)) symmetric = 0;
      if ((a.r > LIGHTING_APP_MAX_BRIGHTNESS) || (a.g > LIGHTING_APP_MAX_BRIGHTNESS) ||
          (a.b > LIGHTING_APP_MAX_BRIGHTNESS)) within_limit = 0;
    }
  }
  check(ends_ok && symmetric && within_limit && out_of_bounds == 0U,
        "all strip lengths 1..21 preserve short-strip fallback, symmetric feather and brightness cap");
  reset();
  g_robot_chassis.steering_scheduled_permille = 500;
  LightingApp_Task(UINT32_MAX - 299U);
  LightingApp_Task(0U);
  check(color_is(&s_right_strip, 10U, 17U, 6U, 0U),
        "turn breath reaches midpoint continuously when tick crosses UINT32_MAX");
  LightingApp_Task(300U);
  check(color_is(&s_right_strip, 10U, 28U, 10U, 0U), "turn breath reaches peak after tick wrap");
  LightingApp_Task(900U);
  check(color_is(&s_right_strip, 10U, 6U, 2U, 0U) && all_color(&s_left_strip, 0U, 12U, 8U),
        "turn breath returns to minimum at full period after tick wrap");
}

static void test_status_priority(void)
{
  reset();
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 1000;
  g_robot_chassis.steering_scheduled_permille = -500;
  g_robot_command.rc_online = 0U;
  g_robot_chassis.steer_fault = 1U;
  g_robot_chassis.left_fault = 1U;
  LightingApp_Task(0U);
  check(all_color(&s_left_strip, 32U, 0U, 0U) && s_motion_direction == 0 && s_turn_side == 0,
        "RC fault overrides steering OID faults and all motion overlays");
  LightingApp_Task(100U);
  check(all_color(&s_left_strip, 0U, 0U, 0U), "RC first double pulse ends at 100 ms");
  LightingApp_Task(200U);
  check(all_color(&s_left_strip, 32U, 0U, 0U), "RC double pulse has second red flash");
  g_robot_command.rc_online = 1U;
  LightingApp_Task(300U);
  check(all_color(&s_left_strip, 32U, 0U, 20U), "steering fault is magenta double pulse above OID");
  g_robot_chassis.steer_fault = 0U;
  LightingApp_Task(350U);
  check(all_color(&s_left_strip, 32U, 0U, 0U), "OID fault is red slow flash");
  LightingApp_Task(850U);
  check(all_color(&s_left_strip, 0U, 0U, 0U), "OID slow flash has 500 ms off phase");
  g_robot_chassis.left_fault = 0U;
  g_robot_command.mode = ROBOT_MODE_AUTO_FC;
  g_robot_chassis.fc_drive_online = 0U;
  LightingApp_Task(900U);
  check(all_color(&s_left_strip, 16U, 0U, 32U), "automatic FC fault is purple slow flash");
  g_robot_chassis.fc_drive_online = 1U;
  g_robot_chassis.parameters_confirmed = 0U;
  LightingApp_Task(950U);
  check(all_color(&s_left_strip, 12U, 0U, 24U), "unconfirmed parameters remain steady purple");
  g_robot_chassis.parameters_confirmed = 1U;
  g_robot_command.mode = ROBOT_MODE_LOCKED;
  LightingApp_Task(1000U);
  check(all_color(&s_left_strip, 10U, 4U, 0U), "healthy intentional lock is dim amber not red");
  g_robot_command.gate = ROBOT_GATE_STARTUP_LOCK_REQUIRED;
  LightingApp_Task(1050U);
  check(all_color(&s_left_strip, 20U, 8U, 0U), "startup authorization wait is amber double pulse");
  g_robot_command.gate = ROBOT_GATE_MODE_CONFIRMING;
  LightingApp_Task(1100U);
  check(all_color(&s_left_strip, 4U, 1U, 0U), "mode-confirmation wait starts amber breathing");
  g_robot_command.mode = ROBOT_MODE_MANUAL;
  g_robot_command.gate = ROBOT_GATE_READY;
  g_robot_chassis.steer_calibration_side = 1U;
  LightingApp_Task(1150U);
  check(all_color(&s_left_strip, 6U, 0U, 12U), "open-loop calibration is dim purple without movement flow");
}

static void test_lifecycle(void)
{
  WS2812Strip_Color_t saved[WS2812_STRIP_MAX_LED_COUNT];
  uint16_t count;
  uint32_t now = 0U;
  reset();
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 1000;
  LightingApp_Task(UINT32_MAX - 25U);
  LightingApp_Task(24U);
  LightingApp_Task(74U);
  check(refreshes[0] == 3U && find_white_lead(&s_left_strip) == 17,
        "50 ms refresh and movement phase remain continuous through tick wrap");
  reset();
  fail_left_init = 1U;
  LightingApp_Init();
  LightingApp_Task(0U);
  check(tasks[0] == 0U && refreshes[0] == 0U && tasks[1] == 1U && refreshes[1] == 1U,
        "failed left init never services invalid handle while right side stays usable");
  reset();
  LightingApp_Task(0U);
  memcpy(saved, s_right_strip.pixels, sizeof(saved));
  s_right_strip.dma_busy = 1U;
  g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 1000;
  LightingApp_Task(50U);
  check(refreshes[0] == 2U && refreshes[1] == 1U &&
        memcmp(saved, s_right_strip.pixels, sizeof(saved)) == 0,
        "busy DMA channel skips redraw and refresh without stalling other channel");
  reset();
  for (count = 1U; count <= WS2812_STRIP_MAX_LED_COUNT; count++)
  {
    s_left_strip.led_count = s_right_strip.led_count = count;
    g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = 1000;
    g_robot_chassis.steering_scheduled_permille = -500;
    LightingApp_Task(now); now += 100U;
    g_robot_chassis.left_target_erpm = g_robot_chassis.right_target_erpm = -1000;
    g_robot_chassis.steering_scheduled_permille = 500;
    LightingApp_Task(now); now += 100U;
  }
  check(out_of_bounds == 0U, "all LED counts 1..21 preserve array and reserved-end boundaries");
}

int main(void)
{
  test_motion(); test_turn(); test_turn_motion_layers(); test_turn_short_and_wrap();
  test_status_priority(); test_lifecycle();
  printf("RESULT: %u failure(s)\n", failures);
  return failures ? 1 : 0;
}
