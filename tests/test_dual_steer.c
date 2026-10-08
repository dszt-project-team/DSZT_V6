/* Host test: actual V6 steering application, mocked hardware and inner loop.
   Not a test of HAL, electrical levels or real mechanical motion. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "steer_angle_ctrl.h"
#include "robot_def.h"
#include "../application/chassis/chassis_config.h"
#ifndef TEST_USE_CONFIG_SIDE
#undef CHASSIS_STEER_CALIBRATION_SIDE
#define CHASSIS_STEER_CALIBRATION_SIDE 2U
#endif
#include "../application/chassis/dual_steer.c"
RobotCommand g_robot_command;
RobotChassisState g_robot_chassis;
int htim2, htim4, htim5, htim8;
static int sensor_invert[2], positive_reverse[2];
void Error_Handler(void) { abort(); }
HAL_StatusTypeDef SteerMotor_Init(SteerMotor_Handle_t *m,int *t,int a,int b,int c,int d,int e,int f,int g)
{ (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g; m->timer=t; SteerMotor_Stop(m); return HAL_OK; }
HAL_StatusTypeDef Mt6826sPwm_Init(Mt6826sPwm_Handle_t *s,int *t,int a,int b,int c,int d)
{ (void)a;(void)b;(void)c;(void)d;s->timer=t;return HAL_OK; }
void Mt6826sPwm_SetDirectionInverted(Mt6826sPwm_Handle_t *s,int v) {sensor_invert[s-&s_encoder[0]]=v;}
void Mt6826sPwm_SetZeroRaw(Mt6826sPwm_Handle_t *s,int v) {s->sample.zero_valid=1;s->sample.raw_angle=(uint16_t)v;}
void Mt6826sPwm_Task(Mt6826sPwm_Handle_t *s,uint32_t t) {(void)s;(void)t;}
void Mt6826sPwm_GetSnapshot(Mt6826sPwm_Handle_t *s,uint32_t t,Mt6826sPwm_Snapshot_t *o) {*o=s->sample;(void)t;}
void SteerAngleCtrl_Init(SteerAngleCtrl_Handle_t *c,SteerMotor_Handle_t *m,Mt6826sPwm_Handle_t *s) {c->motor=m;(void)s;}
void SteerAngleCtrl_Configure(SteerAngleCtrl_Handle_t *c,float a,float b,float d,float e,float f,float g,float h,int i,int j,int k,int l,int m)
{positive_reverse[c-&s_ctrl[0]]=m;(void)a;(void)b;(void)d;(void)e;(void)f;(void)g;(void)h;(void)i;(void)j;(void)k;(void)l;}
void SteerMotor_Stop(SteerMotor_Handle_t *m) {m->duty_permille=0;m->brake_on=1;}
void SteerMotor_SetDirection(SteerMotor_Handle_t *m,int r) {assert(m->brake_on && !m->duty_permille);m->reverse=(uint8_t)r;}
void SteerMotor_SetDuty(SteerMotor_Handle_t *m,int16_t d) {m->duty_permille=d;}
void SteerMotor_SetBrake(SteerMotor_Handle_t *m,int b) {m->brake_on=(uint8_t)b;}
void SteerAngleCtrl_Disable(SteerAngleCtrl_Handle_t *c) {c->state=STEER_ANGLE_CTRL_DISABLED;SteerMotor_Stop(c->motor);}
HAL_StatusTypeDef SteerAngleCtrl_SetTargetDeg(SteerAngleCtrl_Handle_t *c,float t) {c->target_angle_deg=t;return HAL_OK;}
HAL_StatusTypeDef SteerAngleCtrl_Enable(SteerAngleCtrl_Handle_t *c) {c->state=STEER_ANGLE_CTRL_HOLD;return HAL_OK;}
void SteerAngleCtrl_Task(SteerAngleCtrl_Handle_t *c,uint32_t t) {(void)c;(void)t;}

int main(void)
{
  int c;
  unsigned selected = (CHASSIS_STEER_CALIBRATION_SIDE == 2U) ? 1U : 0U;
  SteerMotor_Handle_t *active = &s_motor[selected];
  SteerMotor_Handle_t *stopped = &s_motor[1U-selected];
  unsigned positive_dir = selected ? CHASSIS_STEER_RIGHT_POSITIVE_REVERSE : CHASSIS_STEER_LEFT_POSITIVE_REVERSE;
  float l,r,ls,rs,pl,pr,pls,prs;
  assert(DualSteer_ScheduleCommand(1000,0)==1000);
  assert(DualSteer_ScheduleCommand(-1000,1000)==-700);
  assert(DualSteer_ScheduleCommand(1000,-1000)==700);
  assert(DualSteer_ScheduleCommand(1000,500)==850);
  assert(DualSteer_ScheduleCommand(20,0)==0);
  assert(DualSteer_ScheduleCommand(25,1000)==0);
  DualSteer_Geometry(DualSteer_ScheduleCommand(-1000,1000),&l,&r,&ls,&rs);
  assert(fabsf(l+19.6f)<0.001f && ls<1 && rs==1);
  for(c=1;c<=1000;c++)
  {
    DualSteer_Geometry((int16_t)-c,&l,&r,&ls,&rs);
    DualSteer_Geometry((int16_t)c,&pl,&pr,&pls,&prs);
    assert(l<r && r<0 && ls>0 && ls<1 && rs==1);
    assert(fabsf(l+pr)<0.0001f && fabsf(r+pl)<0.0001f);
    assert(fabsf(ls-prs)<0.0001f && pls==1);
    assert(fabsf(1/tanf(fabsf(r)*0.01745329252f)-1/tanf(fabsf(l)*0.01745329252f)-0.5f/0.61f)<0.002f);
  }
  assert(fabsf(l+28)<0.001f && fabsf(r+20.320382f)<0.001f);
  assert(fabsf(ls-0.739702466f)<0.0001f);
  DualSteer_Geometry(0,&l,&r,&ls,&rs);
  assert(l==0 && r==0 && ls==1 && rs==1);
  DualSteer_Init();
  active->reverse=(uint8_t)positive_dir;
  assert(s_motor[0].timer==&htim8 && s_motor[1].timer==&htim4);
  assert(s_encoder[0].timer==&htim2 && s_encoder[1].timer==&htim5);
  assert(s_encoder[0].sample.zero_valid == CHASSIS_STEER_LEFT_ZERO_CONFIRMED);
  assert(s_encoder[1].sample.zero_valid == CHASSIS_STEER_RIGHT_ZERO_CONFIRMED);
  if (CHASSIS_STEER_LEFT_ZERO_CONFIRMED) assert(s_encoder[0].sample.raw_angle == CHASSIS_STEER_LEFT_ZERO_RAW);
  if (CHASSIS_STEER_RIGHT_ZERO_CONFIRMED) assert(s_encoder[1].sample.raw_angle == CHASSIS_STEER_RIGHT_ZERO_RAW);
  g_robot_command.mode=ROBOT_MODE_CALIBRATION;
  g_robot_command.source_online=g_robot_command.released=g_robot_command.centered=1;
  /* Selected encoder deliberately remains unhealthy: the temporary open-loop
     hardware test must still run while publishing sh=0 for diagnosis. */
  s_encoder[selected].sample.healthy=0;
  DualSteer_Task(1000,0,1,10);assert(!active->duty_permille);
  DualSteer_Task(0,0,1,20);DualSteer_Task(0,0,1,210);assert(!s_released);
  DualSteer_Task(0,0,1,220);assert(s_released);
  DualSteer_Task(1000,0,1,230);assert(active->duty_permille==450 && !active->brake_on);
  assert(!stopped->duty_permille && stopped->brake_on && !DualSteer_MotionReady());
  DualSteer_Task(-1000,0,1,240);assert(!active->duty_permille && active->reverse==positive_dir);
  DualSteer_Task(-1000,0,1,530);assert(active->reverse==positive_dir);
  DualSteer_Task(-1000,0,1,540);assert(active->reverse!=positive_dir && !active->duty_permille);
  DualSteer_Task(-1000,0,1,830);assert(!active->duty_permille);
  DualSteer_Task(-1000,0,1,840);assert(active->duty_permille==450);
  DualSteer_Task(0,0,1,850);assert(!active->duty_permille && active->brake_on);
  DualSteer_Task(-1000,0,1,860);assert(s_released && active->duty_permille==450);
  assert(!g_robot_chassis.steer_healthy[selected]);
  g_robot_command.mode=ROBOT_MODE_AUTO_FC;
  DualSteer_Task(0,0,1,900);DualSteer_Task(-1000,0,1,1200);assert(!active->duty_permille);
  g_robot_command.mode=ROBOT_MODE_CALIBRATION;
  g_robot_command.centered=0;
  DualSteer_Task(0,0,1,1210);DualSteer_Task(0,0,1,1500);assert(!s_released);
  g_robot_command.centered=1;
  DualSteer_Task(0,0,1,1510);DualSteer_Task(0,0,1,1710);assert(s_released);
  DualSteer_Task(-1000,0,1,1720);assert(active->duty_permille==450);
  g_robot_command.released=0;
  DualSteer_Task(-1000,0,1,1730);assert(!active->duty_permille && !s_released);
  g_robot_command.released=1;g_robot_command.source_online=0;
  DualSteer_Task(0,0,1,1740);assert(!s_released);
  g_robot_command.source_online=0;
  DualSteer_Task(-1000,0,0,1750);assert(s_motor[0].brake_on && s_motor[1].brake_on);
  puts("PASS: 2000 geometry samples, side mapping, independent zero validity, selected-side encoder-health bypass, center gate, reversal waits, FC loss/recovery, normal-mode exclusion, OID lock");
  return 0;
}
