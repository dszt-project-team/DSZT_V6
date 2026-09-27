/* Real application + unified status + BSP; no board or actuator access. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../bsp/buzzer/bsp_buzzer.c"
#include "../application/buzzer/buzzer_app.c"
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
RCC_TypeDef test_rcc;
TIM_HandleTypeDef htim12;
unsigned test_tone_writes;
static uint32_t tick_ms;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c) {(void)t;(void)c;return HAL_OK;}
uint32_t HAL_RCC_GetPCLK1Freq(void) {return 42000000U;}
uint32_t HAL_GetTick(void) {return tick_ms;}

static void tick(uint32_t t)
{
  RobotCommand command=g_robot_command;
  RobotChassisState chassis=g_robot_chassis;
  tick_ms=t;BuzzerApp_Task(t);
  assert(memcmp(&command,&g_robot_command,sizeof(command))==0);
  assert(memcmp(&chassis,&g_robot_chassis,sizeof(chassis))==0);
}
static void healthy(void)
{
  memset(&g_robot_command,0,sizeof(g_robot_command));
  memset(&g_robot_chassis,0,sizeof(g_robot_chassis));
  g_robot_command.rc_online=1;
  g_robot_command.mode=ROBOT_MODE_LOCKED;
  g_robot_chassis.parameters_confirmed=1;
  g_robot_chassis.steer_healthy[0]=g_robot_chassis.steer_healthy[1]=1;
  g_robot_chassis.steer_zero_valid[0]=g_robot_chassis.steer_zero_valid[1]=1;
  g_robot_chassis.left_online=g_robot_chassis.right_online=1;
  g_robot_chassis.fc_drive_online=g_robot_chassis.fc_steer_online=1;
}
static void init(void)
{
  memset(&g_robot_command,0,sizeof(g_robot_command));
  memset(&g_robot_chassis,0,sizeof(g_robot_chassis));
  tick_ms=0;htim12.Init.Prescaler=83U;test_rcc.CFGR=RCC_CFGR_PPRE1;
  BuzzerApp_Init();
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BOOT);
}
static void locked_connected(void)
{
  init();healthy();tick(0);tick(260);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RC_CONNECTED);
  tick(415);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
}
int main(void)
{
  init();tick(0);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BOOT);
  tick(90);assert(s_buzzer.phase==2);
  tick(180);assert(s_buzzer.phase==4);
  tick(260);tick(300);tick(10000);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* No RX ever: no repeated loss alarm. */
  locked_connected();
  g_robot_command.mode=ROBOT_MODE_MANUAL;
  tick(500);tick(1000);
  assert(VehicleStatus_Get()==VEHICLE_STATUS_RELEASE_WAIT);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* Mode alone is not unlock. */
  g_robot_chassis.motion_enabled=1;g_robot_chassis.steer_released=1;
  tick(1100);tick(1199);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(1200);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_MANUAL_READY);
  tick(1380);g_robot_command.throttle_permille=1000;g_robot_command.steering_permille=-1000;tick(1400);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  g_robot_command.mode=ROBOT_MODE_LOCKED;tick(1500);tick(1600);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BRAKE);
  tick(1780);
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;g_robot_chassis.fc_release_ready=0;tick(1800);tick(2000);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  g_robot_chassis.fc_release_ready=1;tick(2100);tick(2200);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_AUTO_READY);
  g_robot_command.failsafe=1;tick(2210);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* READY event invalidated immediately. */
  tick(2509);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(2510);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RC_LOST);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_MANUAL_READY,2520);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RC_LOST);
  g_robot_command.failsafe=0;tick(2600);tick(2700);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* Recovery clears alarm, no old unlock. */
  g_robot_command.mode=ROBOT_MODE_LOCKED;g_robot_command.gate=ROBOT_GATE_STARTUP_LOCK_REQUIRED;
  tick(3000);tick(7999);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(8000);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_STARTUP_WAIT);
  g_robot_command.gate=ROBOT_GATE_READY;tick(8100);tick(8200);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BRAKE);
  tick(8400);

  g_robot_chassis.left_fault=1;tick(8500);tick(8799);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(8800);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_OID_DUAL_FAULT);
  g_robot_chassis.steer_fault=1;tick(8810);tick(9110);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_STEER_FAULT);
  g_robot_chassis.steer_fault=0;g_robot_chassis.left_fault=0;
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;g_robot_chassis.fc_drive_online=0;
  tick(9200);tick(9500);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_FC_DUAL_TIMEOUT);
  g_robot_chassis.fc_drive_online=1;tick(9600);tick(10000);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  g_robot_command.rc_online=0;tick(UINT32_MAX-100U);tick(198U);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(199U);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RC_LOST);
  healthy();tick(200);tick(500);
  assert(BSP_Buzzer_GetCue()!=BSP_BUZZER_CUE_RC_CONNECTED); /* First online only. */
  locked_connected();
  BSP_Buzzer_Play(BSP_BUZZER_CUE_BRAKE,500);tick(500);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BRAKE);
  g_robot_command.failsafe=1;tick(510);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* Also cancel ordinary cues when not READY. */
  tick(809);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(810);assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RC_LOST);
  puts("PASS: buzzer shared-status readonly, true READY events, quiet first boot, fault debounce/priority/recovery, startup reminder, wrap");
  return 0;
}
