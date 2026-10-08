#ifndef DUAL_STEER_H
#define DUAL_STEER_H
#include <stdint.h>
void DualSteer_Init(void);
/* 正常位置环10ms；输入禁止时可从1ms服务层立即调用停止。
 * 负值为车体左转；返回独立限速后的OID差速转向指令。 */
int16_t DualSteer_Task(int16_t command, int16_t drive_command_permille,
                       uint8_t source_ready, uint32_t now_ms);
int16_t DualSteer_ScheduleCommand(int16_t command, int16_t drive_command_permille);
uint8_t DualSteer_MotionReady(void);
void DualSteer_Geometry(int16_t command, float *left_deg, float *right_deg,
                        float *left_scale, float *right_scale);
#endif
