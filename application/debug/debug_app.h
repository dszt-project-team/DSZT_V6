#ifndef DEBUG_APP_H
#define DEBUG_APP_H

#include <stdint.h>
#include "debug_config.h"

void DebugApp_Init(void);
void DebugApp_Task(uint32_t now_ms);

#endif
