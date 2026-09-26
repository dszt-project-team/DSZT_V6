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
static void tick(uint32_t t) {sample.frame_count++;CommandApp_Task(t);}
int main(void)
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
  CommandApp_Init();
  sample.online=1;sample.pulse_us[0]=1500;sample.pulse_us[2]=1500;
  sample.pulse_us[4]=1000;sample.pulse_us[6]=2000;
  tick(200);tick(210);assert(g_robot_command.mode==ROBOT_MODE_LOCKED);
  sample.pulse_us[4]=1500;tick(220);tick(230);tick(430);
  assert(g_robot_command.mode==ROBOT_MODE_MANUAL && g_robot_command.gate==ROBOT_GATE_READY);
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
  sample.frame_lost=1;tick(480);
  assert(g_robot_command.mode==ROBOT_MODE_LOCKED && !g_robot_command.throttle_permille && !g_robot_command.steering_permille);
  puts("PASS: throttle deadband/monotonicity/symmetry, CH7, 10ms CH1 filter/reset, startup/center/loss, raw calibration neutral stop");
  return 0;
}
