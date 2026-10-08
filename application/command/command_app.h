#ifndef COMMAND_APP_H
#define COMMAND_APP_H

#include <stdint.h>
#include "pwm_input.h"

typedef struct
{
  PwmInput_Snapshot_t drive;      /* MAIN1 最近控制决策使用的原子快照。 */
  PwmInput_Snapshot_t steer;      /* MAIN2 最近控制决策使用的原子快照。 */
  uint32_t center_elapsed_ms;     /* 最近一次/正在进行的回中时间；异常或离中重置。 */
} CommandFcDiagnostics;

void CommandApp_Init(void);
void CommandApp_Task(uint32_t now_ms);
/* 同一 RobotTask 在 CommandApp_Task 后调用，仅复制诊断，不允许中断中调用。 */
void CommandApp_GetFcDiagnostics(CommandFcDiagnostics *out);

#endif
