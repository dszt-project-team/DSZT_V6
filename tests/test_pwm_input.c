/* Host-only regression: executes the production PWM input module, including
 * edge capture, filtering, timeout recovery and snapshot IRQ protection.
 * HAL and callback registration are the only stubs; never link into firmware. */
#include <stdio.h>
#include <string.h>
#include "../modules/input/pwm_input.c"

static uint32_t tick;
static uint32_t primask;
static uint32_t capture_value;
static uint32_t polarity[4];
static unsigned failures;
static unsigned checks;
static TIM_HandleTypeDef timer;
static BspTimCallbackConfig callbacks[2];
static unsigned callback_count;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t channel)
{ (void)htim; (void)channel; return capture_value; }
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t channel)
{ (void)htim; (void)channel; return HAL_OK; }
void test_set_polarity(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t value)
{ (void)htim; polarity[channel / 4U] = value; }
uint8_t BspCallback_RegisterTim(const BspTimCallbackConfig *config)
{
  if (callback_count >= 2U) return 0U;
  callbacks[callback_count++] = *config;
  return 1U;
}

static void check(int result, const char *name)
{
  checks++;
  printf("%s: %s\n", result ? "PASS" : "FAIL", name);
  if (!result) failures++;
}

static void reset_inputs(void)
{
  memset(s_inputs, 0, sizeof(s_inputs));
  memset(callbacks, 0, sizeof(callbacks));
  memset(polarity, 0, sizeof(polarity));
  callback_count = 0U;
  tick = 0U;
  primask = 0U;
}

static PwmInput_Config_t make_config(uint8_t average_window, uint8_t warmup,
                                     uint32_t channel)
{
  PwmInput_Config_t config;
  memset(&config, 0, sizeof(config));
  config.timer = &timer;
  config.channel = channel;
  config.minimum_valid_us = 800U;
  config.maximum_valid_us = 2200U;
  config.timeout_ms = 30U;
  config.transient_fault_hold_ms = 5U;
  config.average_window = average_window;
  config.valid_samples_to_online = warmup;
  return config;
}

static PwmInput_Handle_t *register_input(uint8_t average_window, uint8_t warmup,
                                         uint32_t channel)
{
  PwmInput_Config_t config = make_config(average_window, warmup, channel);
  PwmInput_Handle_t *handle = PwmInput_Register(&config);
  check(handle != 0, "register independent filter window / warmup count");
  return handle;
}

static void edge(uint32_t channel, uint16_t captured, uint32_t now_ms)
{
  unsigned index;
  timer.Channel = PwmInput_GetActiveChannel(channel);
  capture_value = captured;
  tick = now_ms;
  for (index = 0U; index < callback_count; index++)
    callbacks[index].input_capture(callbacks[index].parent, &timer);
}

static void pulse(uint32_t channel, uint16_t width_us, uint32_t falling_ms)
{
  edge(channel, 10000U, falling_ms - 2U);
  edge(channel, (uint16_t)(10000U + width_us), falling_ms);
}

static PwmInput_Snapshot_t snapshot(PwmInput_Handle_t *handle, uint32_t now_ms,
                                     uint32_t caller_ms)
{
  PwmInput_Snapshot_t value;
  memset(&value, 0, sizeof(value));
  tick = now_ms;
  if (PwmInput_GetSnapshot(handle, &value, caller_ms) == 0U)
    check(0, "get initialized input snapshot");
  return value;
}

static void warm_four(uint32_t channel, uint16_t width_us, uint32_t last_ms)
{
  unsigned index;
  for (index = 0U; index < 4U; index++)
    pulse(channel, width_us, last_ms - 30U + index * 10U);
}

static void test_drive_latest_and_steer_average(void)
{
  PwmInput_Handle_t *drive;
  PwmInput_Handle_t *steer;
  PwmInput_Snapshot_t value;
  unsigned index;
  reset_inputs();
  drive = register_input(1U, 4U, TIM_CHANNEL_1);
  steer = register_input(8U, 4U, TIM_CHANNEL_2);
  for (index = 1U; index <= 4U; index++)
  {
    pulse(TIM_CHANNEL_1, 1502U, index * 10U);
    value = snapshot(drive, index * 10U, index * 10U);
    check(value.online == (index == 4U), "drive warmup still requires four fresh pulses");
  }
  pulse(TIM_CHANNEL_1, 1770U, 50U);
  value = snapshot(drive, 50U, 50U);
  check(value.online && value.filtered_us == 1770U, "drive full forward has no averaging delay");
  pulse(TIM_CHANNEL_1, 1260U, 60U);
  value = snapshot(drive, 60U, 60U);
  check(value.online && value.filtered_us == 1260U, "drive latest reverse replaces forward immediately");
  pulse(TIM_CHANNEL_1, 1502U, 70U);
  value = snapshot(drive, 70U, 70U);
  check(value.online && value.filtered_us == 1502U, "drive neutral replaces reverse immediately");
  for (index = 0U; index < 8U; index++) pulse(TIM_CHANNEL_2, 1500U, 80U + 10U * index);
  pulse(TIM_CHANNEL_2, 2000U, 160U);
  value = snapshot(steer, 160U, 160U);
  check(value.online && value.filtered_us == 1563U, "steering preserves rounded eight-sample average");
  check(snapshot(drive, 160U, 160U).fault == PWM_INPUT_FAULT_TIMEOUT,
        "steering pulses cannot keep drive input online");
}

static void test_range_and_rewarm(void)
{
  PwmInput_Handle_t *drive;
  PwmInput_Snapshot_t value;
  unsigned index;
  reset_inputs();
  drive = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, 40U);
  pulse(TIM_CHANNEL_1, 799U, 50U);
  value = snapshot(drive, 50U, 50U);
  check(!value.online && value.fault == PWM_INPUT_FAULT_RANGE,
        "too-short pulse denies input and resets warmup");
  for (index = 1U; index <= 4U; index++)
  {
    pulse(TIM_CHANNEL_1, 1502U, 50U + index * 10U);
    value = snapshot(drive, 50U + index * 10U, 50U + index * 10U);
    check(value.online == (index == 4U), "range recovery needs four consecutive valid pulses");
  }
  pulse(TIM_CHANNEL_1, 2201U, 100U);
  value = snapshot(drive, 100U, 100U);
  check(!value.online && value.fault == PWM_INPUT_FAULT_RANGE,
        "too-long pulse is not clamped into full throttle");
  pulse(TIM_CHANNEL_1, 800U, 110U);
  pulse(TIM_CHANNEL_1, 2200U, 120U);
  value = snapshot(drive, 120U, 120U);
  check(value.valid_pulse_count == 10U && value.invalid_pulse_count == 2U,
        "configured range endpoints remain valid");
}

static void test_gap_before_poll_and_recovery_edges(void)
{
  PwmInput_Handle_t *input;
  PwmInput_Snapshot_t value;
  reset_inputs();
  input = register_input(8U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 2000U, 40U);
  pulse(TIM_CHANNEL_1, 1000U, 100U); /* No snapshot during the outage. */
  value = snapshot(input, 100U, 100U);
  check(!value.online && value.fault == PWM_INPUT_FAULT_WARMUP && value.filtered_us == 1000U,
        "first frame after an unpolled gap discards old average and online streak");
  check(value.timeout_event_count == 1U,
        "ISR records an outage that the task never observed");
  pulse(TIM_CHANNEL_1, 1000U, 110U);
  pulse(TIM_CHANNEL_1, 1000U, 120U);
  pulse(TIM_CHANNEL_1, 1000U, 130U);
  value = snapshot(input, 130U, 130U);
  check(value.online && value.timeout_event_count == 1U,
        "four good recovery frames cannot erase the recorded outage");

  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, 40U);
  value = snapshot(input, 71U, 71U);
  check(!value.online && value.fault == PWM_INPUT_FAULT_TIMEOUT,
        "31-ms silence enters timeout");
  check(value.timeout_event_count == 1U,
        "first timeout snapshot includes the new event immediately");
  edge(TIM_CHANNEL_1, 1000U, 80U);
  (void)snapshot(input, 81U, 81U);
  check(polarity[0] == TIM_INPUTCHANNELPOLARITY_FALLING,
        "repeated timeout polling preserves an in-progress recovery pulse");
  edge(TIM_CHANNEL_1, 2502U, 82U);
  value = snapshot(input, 82U, 82U);
  check(value.valid_pulse_count == 5U && value.fault == PWM_INPUT_FAULT_WARMUP,
        "the recovery falling edge produces the first new warmup sample");
  check(value.timeout_event_count == 1U,
        "ISR recovery does not double-count a task-observed outage");
  pulse(TIM_CHANNEL_1, 1502U, 92U);
  pulse(TIM_CHANNEL_1, 1502U, 102U);
  pulse(TIM_CHANNEL_1, 1502U, 112U);
  check(snapshot(input, 112U, 112U).online, "timeout recovery becomes online after four new samples");
}

static void test_timeout_event_dedup_and_persistence(void)
{
  PwmInput_Handle_t *input;
  PwmInput_Snapshot_t value;
  unsigned index;
  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  value = snapshot(input, 100U, 100U);
  check(value.fault == PWM_INPUT_FAULT_NOT_READY && value.timeout_event_count == 0U,
        "never-seen input stays not-ready without invented timeout events");
  warm_four(TIM_CHANNEL_1, 1502U, 140U);
  value = snapshot(input, 170U, 170U);
  check(value.online && value.timeout_event_count == 0U,
        "exactly the configured timeout interval is still healthy");
  for (index = 171U; index <= 180U; index++)
    value = snapshot(input, index, index);
  check(value.timeout_event_count == 1U,
        "repeated timeout polling counts one continuous outage only");
  pulse(TIM_CHANNEL_1, 700U, 182U);
  value = snapshot(input, 183U, 183U);
  check(value.timeout_event_count == 1U && value.invalid_pulse_count == 1U,
        "invalid pulse during outage cannot re-arm its timeout counter");
  warm_four(TIM_CHANNEL_1, 1502U, 222U);
  value = snapshot(input, 222U, 222U);
  check(value.online && value.timeout_event_count == 1U,
        "task-observed outage remains single after invalid and good recovery frames");
  value = snapshot(input, 253U, 253U);
  check(value.timeout_event_count == 2U && value.fault == PWM_INPUT_FAULT_TIMEOUT,
        "a later independent outage increments the cumulative counter");

  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, 40U);
  /* The task is absent for the entire outage and four-frame recovery. */
  warm_four(TIM_CHANNEL_1, 1770U, 130U);
  value = snapshot(input, 130U, 130U);
  check(value.online && value.timeout_event_count == 1U && value.filtered_us == 1770U,
        "late task sees saved timeout even when the input is already healthy again");

  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, UINT32_MAX - 10U);
  value = snapshot(input, 19U, 19U);
  check(value.online && value.timeout_event_count == 0U,
        "tick rollover itself does not fabricate a timeout event");
  value = snapshot(input, 20U, 20U);
  check(value.fault == PWM_INPUT_FAULT_TIMEOUT && value.timeout_event_count == 1U,
        "first true post-rollover timeout is counted once");
  warm_four(TIM_CHANNEL_1, 1502U, 60U);
  value = snapshot(input, 60U, 60U);
  check(value.online && value.timeout_event_count == 1U,
        "post-rollover recovery does not double-count the task-observed event");

  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, UINT32_MAX - 10U);
  warm_four(TIM_CHANNEL_1, 1502U, 60U);
  value = snapshot(input, 60U, 60U);
  check(value.online && value.timeout_event_count == 1U,
        "ISR-only post-rollover recovery preserves an otherwise hidden timeout");
}

static void test_clock_capture_rollover_and_snapshot(void)
{
  PwmInput_Handle_t *input;
  PwmInput_Snapshot_t value;
  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, 40U);
  check(snapshot(input, 40U, 39U).online,
        "new ISR timestamp is not mistaken for timeout by an older caller tick");
  value = snapshot(input, 71U, 40U);
  check(value.fault == PWM_INPUT_FAULT_TIMEOUT,
        "stale caller time cannot disguise a real input timeout");
  primask = 1U;
  (void)snapshot(input, 72U, 72U);
  check(primask == 1U, "snapshot preserves already-disabled IRQ state");
  primask = 0U;
  (void)snapshot(input, 73U, 73U);
  check(primask == 0U, "snapshot restores enabled IRQ state");

  reset_inputs();
  input = register_input(1U, 4U, TIM_CHANNEL_1);
  warm_four(TIM_CHANNEL_1, 1502U, UINT32_MAX - 10U);
  check(snapshot(input, 5U, 5U).online, "HAL tick rollover preserves recent healthy input");
  check(snapshot(input, 21U, 21U).fault == PWM_INPUT_FAULT_TIMEOUT,
        "timeout is still detected across HAL tick rollover");

  reset_inputs();
  input = register_input(1U, 1U, TIM_CHANNEL_1);
  edge(TIM_CHANNEL_1, 65000U, 10U);
  edge(TIM_CHANNEL_1, 964U, 12U);
  value = snapshot(input, 12U, 12U);
  check(value.online && value.raw_us == 1500U, "ordinary 16-bit capture rollover preserves pulse width");
  edge(TIM_CHANNEL_1, 1000U, 100U);
  edge(TIM_CHANNEL_1, 2500U, 167U);
  value = snapshot(input, 167U, 167U);
  check(!value.online && value.invalid_pulse_count == 1U,
        "long high level cannot alias to a valid pulse after a full timer wrap");
}

static void test_configuration_and_long_warmup(void)
{
  PwmInput_Config_t config;
  PwmInput_Handle_t *input;
  unsigned index;
  reset_inputs();
  config = make_config(0U, 4U, TIM_CHANNEL_1);
  check(PwmInput_Register(&config) == 0, "zero average window rejected");
  config.average_window = 9U;
  check(PwmInput_Register(&config) == 0, "oversize average window rejected");
  config.average_window = 1U;
  config.valid_samples_to_online = 0U;
  check(PwmInput_Register(&config) == 0, "zero warmup rejected");
  input = register_input(1U, 255U, TIM_CHANNEL_1);
  for (index = 0U; index < 254U; index++) pulse(TIM_CHANNEL_1, 1502U, 10U + index * 10U);
  check(!snapshot(input, 2540U, 2540U).online, "warmup up to 255 frames is independent of averaging");
  pulse(TIM_CHANNEL_1, 1502U, 2550U);
  check(snapshot(input, 2550U, 2550U).online, "maximum configured warmup count reaches online");
  pulse(TIM_CHANNEL_1, 1502U, 2560U);
  check(snapshot(input, 2560U, 2560U).online, "warmup streak saturates instead of wrapping");
}

int main(void)
{
  test_drive_latest_and_steer_average();
  test_range_and_rewarm();
  test_gap_before_poll_and_recovery_edges();
  test_timeout_event_dedup_and_persistence();
  test_clock_capture_rollover_and_snapshot();
  test_configuration_and_long_warmup();
  printf("RESULT: %u/%u checks passed; %u failure(s)\n", checks - failures, checks, failures);
  return failures ? 1 : 0;
}
