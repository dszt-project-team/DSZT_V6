#ifndef OID_REVERSE_GUARD_H
#define OID_REVERSE_GUARD_H
#include <stdint.h>
#include "chassis_config.h"

/* No throttle filter/ramp. Opposite requests must pass a paired zero-speed
 * barrier; neutral or a new stick direction never cancels an active barrier. */
typedef struct
{
  int8_t direction;
  uint8_t waiting;
  uint8_t zero_pairs;
  uint32_t begin_ms;
  uint32_t left_sample_ms;
  uint32_t right_sample_ms;
} OidReverseGuard;

static uint8_t OidReverseGuard_Update(OidReverseGuard *g, int32_t requested,
    uint8_t healthy, uint8_t zero_sent, int32_t left, int32_t right,
    uint32_t left_ms, uint32_t right_ms, uint32_t now)
{
  int8_t desired = (requested > 0) ? 1 : ((requested < 0) ? -1 : 0);
  uint8_t stopped = (uint8_t)(healthy != 0U &&
      (now - left_ms) <= CHASSIS_OID_REVERSE_STATUS_MAX_AGE_MS &&
      (now - right_ms) <= CHASSIS_OID_REVERSE_STATUS_MAX_AGE_MS &&
      left >= -CHASSIS_OID_REVERSE_ZERO_ERPM && left <= CHASSIS_OID_REVERSE_ZERO_ERPM &&
      right >= -CHASSIS_OID_REVERSE_ZERO_ERPM && right <= CHASSIS_OID_REVERSE_ZERO_ERPM);
  uint8_t opposite_feedback = (uint8_t)(healthy != 0U &&
      ((desired > 0 && (left < -CHASSIS_OID_REVERSE_ZERO_ERPM || right < -CHASSIS_OID_REVERSE_ZERO_ERPM)) ||
       (desired < 0 && (left > CHASSIS_OID_REVERSE_ZERO_ERPM || right > CHASSIS_OID_REVERSE_ZERO_ERPM))));

  if (g->waiting == 0U && desired != 0 &&
      ((g->direction != 0 && desired != g->direction) || opposite_feedback != 0U))
  {
    g->waiting = 1U;
    g->begin_ms = now;
    g->zero_pairs = 0U;
    /* Only samples newer than the stop request may release a reversal. */
    g->left_sample_ms = left_ms;
    g->right_sample_ms = right_ms;
    return 0U;
  }

  if (g->waiting != 0U || desired == 0)
  {
    if (stopped == 0U || (g->waiting != 0U && zero_sent == 0U))
    {
      g->zero_pairs = 0U;
      g->left_sample_ms = left_ms;
      g->right_sample_ms = right_ms;
    }
    else if (left_ms != g->left_sample_ms && right_ms != g->right_sample_ms)
    {
      g->left_sample_ms = left_ms;
      g->right_sample_ms = right_ms;
      if (g->zero_pairs < CHASSIS_OID_REVERSE_ZERO_PAIRS) g->zero_pairs++;
      if (g->zero_pairs >= CHASSIS_OID_REVERSE_ZERO_PAIRS)
      {
        g->direction = 0;
        g->waiting = 0U;
      }
    }
    if (g->waiting != 0U) return 0U;
  }
  if (desired != 0)
  {
    g->direction = desired;
    g->zero_pairs = 0U;
  }
  return 1U;
}
#endif
