#ifndef TEST_SBUS_H
#define TEST_SBUS_H
#include "usart.h"
typedef struct { int unused; } SBusRc_Handle_t;
typedef struct {
  uint16_t pulse_us[16];
  uint8_t online, failsafe, frame_lost;
  uint32_t frame_count, error_count;
} SBusRc_Data_t;
void SBusRc_Init(SBusRc_Handle_t *, UART_HandleTypeDef *);
void SBusRc_Task(SBusRc_Handle_t *, uint32_t);
uint8_t SBusRc_GetSnapshot(SBusRc_Handle_t *, SBusRc_Data_t *);
#endif
