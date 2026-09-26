#include <assert.h>
#include <stdio.h>
#include "../application/chassis/oid_stop_guard.h"

int main(void)
{
  OidStopGuard g = {0};
  OidStopGuard_Update(&g, 1U);
  assert(OidStopGuard_RequestDue(&g, 100U));
  OidStopGuard_MarkRequest(&g, 100U);
  assert(!OidStopGuard_RequestDue(&g, 199U));
  assert(OidStopGuard_RequestDue(&g, 200U));
  OidStopGuard_Update(&g, 1U);
  assert(g.active && g.requested && g.request_ms == 100U);
  OidStopGuard_Update(&g, 0U);
  assert(!g.active && !g.requested && !OidStopGuard_RequestDue(&g, 201U));
  OidStopGuard_Update(&g, 1U);
  assert(OidStopGuard_RequestDue(&g, 202U));
  OidStopGuard_MarkRequest(&g, UINT32_MAX - 50U);
  assert(!OidStopGuard_RequestDue(&g, 48U));
  assert(OidStopGuard_RequestDue(&g, 49U));
  puts("PASS: immediate zero, 100ms retries, resume without latch, new-stop reset, tick rollover");
  return 0;
}
