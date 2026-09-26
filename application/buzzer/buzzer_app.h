#ifndef BUZZER_APP_H
#define BUZZER_APP_H

#include <stdint.h>

void BuzzerApp_Init(void);
void BuzzerApp_Task(uint32_t now_ms);

#endif
