#ifndef TEST_SBUS_H
#define TEST_SBUS_H
#include "usart.h"
typedef struct { int unused; } SBusRc_Handle_t;
typedef struct {
  uint16_t pulse_us[16];
  uint8_t online, failsafe, frame_lost;
  uint32_t frame_count, error_count;
  uint8_t raw_flags, guard_reason, last_guard_reason;
  uint16_t lost_streak;
  uint32_t good_frame_count, last_good_ms, lost_count, failsafe_count;
  uint32_t uart_error_count, timeout_count, guard_event_count, last_guard_ms;
} SBusRc_Data_t;
void SBusRc_Init(SBusRc_Handle_t *, UART_HandleTypeDef *);
void SBusRc_Task(SBusRc_Handle_t *, uint32_t);
uint8_t SBusRc_GetSnapshot(SBusRc_Handle_t *, SBusRc_Data_t *);
#endif
