/* Actual command application, mocked SBUS transport. No firmware/actuator IO. */
#include <assert.h>
#include <stdio.h>
#include "../application/command/command_app.c"
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
UART_HandleTypeDef huart1;
static SBusRc_Data_t sample;
void SBusRc_Init(SBusRc_Handle_t *s,UART_HandleTypeDef *u) {(void)s;(void)u;}
void SBusRc_Task(SBusRc_Handle_t *s,uint32_t t) {(void)s;(void)t;}
uint8_t SBusRc_GetSnapshot(SBusRc_Handle_t *s,SBusRc_Data_t *out) {(void)s;*out=sample;return 1;}
static void tick(uint32_t t)
{
  sample.frame_count++;
  if (!sample.frame_lost && !sample.failsafe && !sample.guard_reason && sample.online)
  {
    sample.good_frame_count++;
    sample.last_good_ms=t;
  }
  CommandApp_Task(t);
}
static void reset_sample(void)
{
  CommandApp_Init();
  memset(&sample,0,sizeof(sample));
  sample.online=1;
  sample.pulse_us[0]=1500;
  sample.pulse_us[2]=1500;
  sample.pulse_us[4]=1000;
  sample.pulse_us[6]=2000;
}
static void ready_manual(void)
{
  reset_sample();
  tick(200);tick(210);
  sample.pulse_us[4]=1500;tick(220);tick(230);tick(430);
  assert(g_robot_command.mode==ROBOT_MODE_MANUAL && g_robot_command.gate==ROBOT_GATE_READY);
}
static void start_fault(uint32_t t)
{
  sample.failsafe=1;
  sample.guard_reason=2;
  sample.last_guard_reason=2;
  sample.guard_event_count++;
  sample.last_guard_ms=t;
  tick(t);
  assert(g_robot_command.gate==ROBOT_GATE_RC_LOST);
  assert(g_robot_command.mode==ROBOT_MODE_LOCKED);
  assert(!g_robot_command.throttle_permille && !g_robot_command.steering_permille);
}
static void clear_fault(void)
{
  sample.failsafe=0;
  sample.frame_lost=0;
  sample.guard_reason=0;
  sample.online=1;
}
static void test_mapping(void)
{
  unsigned p;
  int16_t previous=-1000, value;
  assert(CommandApp_PulseToPermille(1450)==0);
  assert(CommandApp_PulseToPermille(1550)==0);
  assert(CommandApp_PulseToPermille(1551)==2);
  assert(CommandApp_PulseToPermille(1449)==-2);
  for(p=900;p<=2100;p++)
  {
    value=CommandApp_PulseToPermille((uint16_t)p);
    assert(value>=previous && value>=-1000 && value<=1000);
    assert(value==-CommandApp_PulseToPermille((uint16_t)(3000-p)));
    previous=value;
  }
  assert(CommandApp_SpeedLimit(1000)==100 && CommandApp_SpeedLimit(2000)==4900);
  assert(CommandApp_SteeringPermille(1257)==-1000 && CommandApp_SteeringPermille(1757)==1000);
  assert(CommandApp_FilterSteering(1500,100)==1500);
  assert(CommandApp_FilterSteering(1800,101)==1500);
  assert(CommandApp_FilterSteering(1800,110)==1600);
  assert(CommandApp_FilterSteering(1800,111)==1600);
  assert(CommandApp_FilterSteering(1800,120)==1700);
  assert(CommandApp_FilterSteering(1800,130)==1800);
  CommandApp_Lock(ROBOT_GATE_RC_LOST);
  assert(CommandApp_FilterSteering(1500,140)==1500);
  ready_manual();
  sample.pulse_us[0]=1757;sample.pulse_us[2]=1551;tick(440);
  assert(g_robot_command.throttle_permille==2);
  if (CHASSIS_STEER_CALIBRATION_SIDE != 0U && COMMAND_CAL_USE_RAW_STEERING != 0U)
  {
    assert(g_robot_command.steering_permille==1000);
    sample.pulse_us[0]=1500;tick(441);assert(g_robot_command.steering_permille==0);
  }
  else
  {
    assert(g_robot_command.steering_permille>0 && g_robot_command.steering_permille<1000);
    sample.pulse_us[0]=1500;tick(441);assert(g_robot_command.steering_permille>0);
    tick(450);tick(460);tick(470);assert(g_robot_command.steering_permille==0);
  }
}
static void test_ch5_qualification(void)
{
  unsigned i;
  CommandRcDiagnostics d;
  ready_manual();
  sample.pulse_us[4]=1300;tick(440);
  assert(g_robot_command.mode==ROBOT_MODE_MANUAL && g_robot_command.gate==ROBOT_GATE_READY);
  sample.pulse_us[4]=1500;tick(450);
  assert(g_robot_command.gate==ROBOT_GATE_READY);
  /* The old separate counters let INVALID/AUTO alternation remain READY forever. */
  sample.pulse_us[4]=1300;tick(460);
  sample.pulse_us[4]=2000;tick(470);
  assert(g_robot_command.mode==ROBOT_MODE_LOCKED && g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  for(i=0;i<20;i++)
  {
    sample.pulse_us[4]=(i%2==0)?1300:2000;
    tick(480+i*10);
    assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  }
  CommandApp_GetRcDiagnostics(&d);
  assert(d.mode_reject_count==1);
  sample.pulse_us[4]=1500;tick(700);
  assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  CommandApp_Task(705); /* Re-reading a snapshot cannot count as a fresh frame. */
  assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  tick(710);
  assert(g_robot_command.mode==ROBOT_MODE_MANUAL && g_robot_command.gate==ROBOT_GATE_THROTTLE_CENTERING);
  tick(910);assert(g_robot_command.gate==ROBOT_GATE_READY);
  /* Two healthy identical frames still switch directly to another valid mode. */
  sample.pulse_us[4]=2000;tick(920);
  assert(g_robot_command.mode==ROBOT_MODE_MANUAL);
  tick(930);assert(g_robot_command.mode==ROBOT_MODE_AUTO_FC);
  sample.pulse_us[4]=1300;tick(940);tick(950);
  assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
}
static void test_soft_loss_and_recovery(void)
{
  CommandRcDiagnostics d;
  ready_manual();
  sample.pulse_us[2]=1750;tick(440);
  assert(g_robot_command.throttle_permille>0);
  sample.frame_lost=1;sample.raw_flags=4;sample.lost_streak=1;
  tick(450); /* Driver retains last-good channel values on soft frame_lost. */
  assert(g_robot_command.gate==ROBOT_GATE_READY && !g_robot_command.failsafe);
  assert(g_robot_command.throttle_permille>0);
  sample.lost_streak=2;tick(460);
  assert(g_robot_command.gate==ROBOT_GATE_READY);
  sample.guard_reason=1;sample.guard_event_count++;sample.last_guard_reason=1;
  sample.last_guard_ms=470;tick(470);
  assert(g_robot_command.gate==ROBOT_GATE_RC_LOST && g_robot_command.failsafe);
  clear_fault();sample.pulse_us[2]=1500;
  CommandApp_Task(480); /* No fresh good frame: no mode reconfirmation. */
  assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  tick(490);assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  tick(500);assert(g_robot_command.gate==ROBOT_GATE_THROTTLE_CENTERING);
  tick(700);assert(g_robot_command.gate==ROBOT_GATE_READY);
  CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==0 && d.last_loss_ms==10 && d.max_loss_ms==10);
  assert(d.sbus.good_frame_count==sample.good_frame_count);
  /* Soft flagged frames cannot confirm a new CH5 position with held channels. */
  sample.pulse_us[4]=2000;tick(710);
  sample.frame_lost=1;tick(720);tick(730);
  assert(g_robot_command.mode==ROBOT_MODE_MANUAL);
  clear_fault();tick(740);assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  tick(750);assert(g_robot_command.mode==ROBOT_MODE_AUTO_FC);

  ready_manual();sample.pulse_us[4]=2000;tick(440);
  sample.lost_count++; /* A soft-lost frame followed by a good one between tasks. */
  tick(450);assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  tick(460);assert(g_robot_command.mode==ROBOT_MODE_AUTO_FC);
}
static void test_long_loss(void)
{
  CommandRcDiagnostics d;
  ready_manual();
  start_fault(1000);tick(1699);
  CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==0 && d.last_loss_ms==699);
  tick(1700);tick(1800);
  CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==1 && d.last_loss_ms==800 && d.max_loss_ms==800);
  clear_fault();tick(1900);tick(1910);
  assert(g_robot_command.gate==ROBOT_GATE_STARTUP_LOCK_REQUIRED);
  sample.pulse_us[4]=1000;tick(1920);tick(1930);
  assert(g_robot_command.mode==ROBOT_MODE_LOCKED && g_robot_command.gate==ROBOT_GATE_READY);
  sample.pulse_us[4]=1500;tick(1940);tick(1950);tick(2150);
  assert(g_robot_command.gate==ROBOT_GATE_READY);
  CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==1 && d.last_loss_ms==900);

  ready_manual();start_fault(1000);tick(1699);
  clear_fault();tick(1701); /* Recovery must not skip the 700 ms boundary. */
  CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==1 && d.last_loss_ms==701);
  assert(g_robot_command.gate==ROBOT_GATE_STARTUP_LOCK_REQUIRED);

  ready_manual();start_fault(UINT32_MAX-300U);tick((UINT32_MAX-300U)+699U);
  CommandApp_GetRcDiagnostics(&d);assert(d.revoke_count==0);
  clear_fault();tick((UINT32_MAX-300U)+700U);
  CommandApp_GetRcDiagnostics(&d);assert(d.revoke_count==1 && d.last_loss_ms==700);
}
static void test_center_release_requires_good_frame(void)
{
  reset_sample();tick(200);tick(210);
  sample.pulse_us[4]=1500;tick(220);tick(230);
  assert(g_robot_command.gate==ROBOT_GATE_THROTTLE_CENTERING);
  /* At 199 ms center dwell a soft-lost frame must not authorize a new release. */
  sample.frame_lost=1;sample.lost_count++;tick(429);
  CommandApp_Task(430);CommandApp_Task(440);
  assert(g_robot_command.gate==ROBOT_GATE_THROTTLE_CENTERING);
  assert(!g_robot_command.throttle_permille);
  sample.frame_lost=0;CommandApp_Task(441);
  assert(g_robot_command.gate==ROBOT_GATE_THROTTLE_CENTERING);
  tick(450);
  assert(g_robot_command.gate==ROBOT_GATE_READY);
  /* Once legitimately released, the same bounded soft loss does not re-lock. */
  sample.frame_lost=1;sample.lost_count++;tick(460);
  assert(g_robot_command.gate==ROBOT_GATE_READY);
}
static void test_transient_event_and_reset(void)
{
  CommandRcDiagnostics d;
  ready_manual();
  /* ISR saw a hard fault and then a healthy frame before the command task ran. */
  sample.guard_event_count++;sample.last_guard_reason=2;sample.last_guard_ms=445;
  tick(450);
  assert(g_robot_command.gate==ROBOT_GATE_RC_LOST && g_robot_command.failsafe);
  CommandApp_Task(455);
  assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING && !g_robot_command.failsafe);
  tick(460);assert(g_robot_command.gate==ROBOT_GATE_MODE_CONFIRMING);
  tick(470);tick(670);assert(g_robot_command.gate==ROBOT_GATE_READY);
  CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==0 && d.last_loss_ms==5);
  start_fault(700);tick(1400);
  CommandApp_Init();CommandApp_GetRcDiagnostics(&d);
  assert(d.revoke_count==0 && d.mode_reject_count==0 && d.max_loss_ms==0);
  assert(g_robot_command.gate==ROBOT_GATE_STARTUP_LOCK_REQUIRED && !g_robot_command.throttle_permille);
  reset_sample();sample.pulse_us[4]=1500;tick(1500);tick(1510);tick(2000);
  assert(g_robot_command.gate==ROBOT_GATE_STARTUP_LOCK_REQUIRED);
  CommandApp_GetRcDiagnostics(NULL);
}
int main(void)
{
  test_mapping();
  test_ch5_qualification();
  test_soft_loss_and_recovery();
  test_long_loss();
  test_center_release_requires_good_frame();
  test_transient_event_and_reset();
  puts("PASS: command mapping/filter, CH5 mixed glitches/fresh-frame qualification, soft/hard loss, recovery/revoke at 700 ms/wrap, transient event, diagnostics/reset");
  return 0;
}
