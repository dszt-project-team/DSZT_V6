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
  RobotCommand command = g_robot_command;
  RobotChassisState chassis = g_robot_chassis;
  tick_ms=t; BuzzerApp_Task(t);
  assert(memcmp(&command,&g_robot_command,sizeof(command)) == 0);
  assert(memcmp(&chassis,&g_robot_chassis,sizeof(chassis)) == 0);
}
static void healthy(void)
{
  memset(&g_robot_command,0,sizeof(g_robot_command));
  memset(&g_robot_chassis,0,sizeof(g_robot_chassis));
  g_robot_command.source_online=1;
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;
  g_robot_command.gate=ROBOT_GATE_FC_CENTERING;
  g_robot_chassis.parameters_confirmed=1;
  g_robot_chassis.steer_healthy[0]=g_robot_chassis.steer_healthy[1]=1;
  g_robot_chassis.steer_zero_valid[0]=g_robot_chassis.steer_zero_valid[1]=1;
  g_robot_chassis.left_online=g_robot_chassis.right_online=1;
  g_robot_chassis.fc_drive_online=g_robot_chassis.fc_steer_online=1;
}
static void ready(void)
{
  g_robot_command.gate=ROBOT_GATE_READY;
  g_robot_command.released=1;
  g_robot_chassis.fc_release_ready=1;
  g_robot_chassis.steer_released=1;
  g_robot_chassis.motion_enabled=1;
}
static void init(void)
{
  memset(&g_robot_command,0,sizeof(g_robot_command));
  memset(&g_robot_chassis,0,sizeof(g_robot_chassis));
  tick_ms=0; htim12.Init.Prescaler=83U; test_rcc.CFGR=RCC_CFGR_PPRE1;
  BuzzerApp_Init(); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BOOT);
}
int main(void)
{
  init(); tick(0); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_BOOT);
  tick(90); assert(s_buzzer.phase==2);
  tick(180); assert(s_buzzer.phase==4);
  tick(260); tick(300); tick(10000);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* No FC ever: no repeated alarm. */
  healthy(); tick(10100); tick(15099);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(15100); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_RELEASE_WAIT);
  ready(); tick(15200); tick(15299); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(15300); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_AUTO_READY);
  tick(15530); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  g_robot_command.throttle_permille=1000; g_robot_command.steering_permille=-1000; tick(15600);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* No throttle/steer sound. */
  g_robot_command.source_online=0; tick(15700); tick(15999);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(16000); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_FC_FAULT);
  BSP_Buzzer_Play(BSP_BUZZER_CUE_AUTO_READY,16010);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_FC_FAULT);
  healthy(); tick(16100); tick(16200);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* Old READY event not queued. */
  ready(); tick(16300); tick(16400); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_AUTO_READY);
  g_robot_chassis.left_fault=1; tick(16410);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* Fault immediately cancels ordinary cue. */
  tick(16709); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(16710); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_OID_FAULT);
  g_robot_chassis.steer_fault=1; tick(16800); tick(17100);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_STEER_FAULT);
  g_robot_chassis.fc_steer_online=0; tick(17200); tick(17500);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_FC_FAULT);
  healthy(); g_robot_command.mode=ROBOT_MODE_CALIBRATION;
  g_robot_chassis.steer_calibration_side=1; ready(); tick(17600); tick(18000);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT); /* Calibration is not drive READY. */
  g_robot_command.source_online=0; tick(UINT32_MAX-100U); tick(198U);
  assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  tick(199U); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_FC_FAULT);
  healthy(); tick(200); tick(500); assert(BSP_Buzzer_GetCue()==BSP_BUZZER_CUE_SILENT);
  puts("PASS: FC-only buzzer readonly, boot, true READY, centering reminder, fault debounce/priority/recovery, wrap");
  return 0;
}
