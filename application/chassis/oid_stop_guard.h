#ifndef OID_STOP_GUARD_H
#define OID_STOP_GUARD_H
#include <stdint.h>

/* Not a mechanical brake. Keep transmitting zeros if a stop fault is latched.
 * Clearing the latch requires an MCU restart after physical inspection. */
typedef struct
{
  uint32_t begin_ms;
  uint32_t request_ms;
  uint8_t active;
  uint8_t requested;
  uint8_t fault;
} OidStopGuard;

static void OidStopGuard_Update(OidStopGuard *g, uint8_t stopping,
    uint8_t feedback_healthy, int32_t left, int32_t right, uint32_t now)
{
  if (stopping == 0U && g->fault == 0U)
  {
    g->active = g->requested = 0U;
    return;
  }
  if (g->active == 0U)
  {
    g->begin_ms = now;
    g->active = 1U;
    g->requested = 0U;
  }
  /* At most 4900 ERPM / 4900 ERPM/s = 1 s nominal deceleration;
     allow another 500 ms, then require fresh low-speed feedback. */
  if ((now - g->begin_ms) >= 1500U && feedback_healthy != 0U &&
      (left < -50 || left > 50 || right < -50 || right > 50))
    g->fault = 1U;
}

static uint8_t OidStopGuard_RequestDue(const OidStopGuard *g, uint32_t now)
{
  return (uint8_t)(g->active != 0U &&
      (g->requested == 0U || (now - g->request_ms) >= 100U));
}

static void OidStopGuard_MarkRequest(OidStopGuard *g, uint32_t now)
{
  g->requested = 1U;
  g->request_ms = now;
}
#endif
