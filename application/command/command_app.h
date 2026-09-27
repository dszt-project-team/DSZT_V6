#ifndef COMMAND_APP_H
#define COMMAND_APP_H

#include <stdint.h>
#include "sbus_rc.h"

typedef struct
{
  SBusRc_Data_t sbus;          /* 与最近一次控制决策相同的原子 SBUS 快照。 */
  uint32_t revoke_count;      /* 持续失联导致既有启动授权被撤销的累计次数。 */
  uint32_t mode_reject_count; /* CH5 连续偏离原档且未确认新档的累计次数。 */
  uint32_t max_loss_ms;       /* 本次上电最长连续失联时长，含正在发生的失联。 */
  uint32_t last_loss_ms;      /* 正在发生或最近结束的一次连续失联时长。 */
} CommandRcDiagnostics;

void CommandApp_Init(void);
void CommandApp_Task(uint32_t now_ms);
/* 仅由同一 RobotTask 在 CommandApp_Task 之后调用，不用于中断。 */
void CommandApp_GetRcDiagnostics(CommandRcDiagnostics *out);

#endif
