#ifndef OID_STOP_GUARD_H
#define OID_STOP_GUARD_H
#include <stdint.h>

/* 仅维护停车零速重发，不按反馈速度锁存行走故障。
 * 行走许可仍由飞控输入、转向和OID健康/安全恢复状态决定；不等于机械制动。 */
typedef struct
{
  uint32_t request_ms;
  uint8_t active;
  uint8_t requested;
} OidStopGuard;

static void OidStopGuard_Update(OidStopGuard *g, uint8_t stopping)
{
  if (stopping == 0U)
  {
    g->active = g->requested = 0U;
    return;
  }
  if (g->active == 0U)
  {
    g->active = 1U;
    g->requested = 0U;
  }
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
