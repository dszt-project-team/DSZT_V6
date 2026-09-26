#include <assert.h>
#include <stdio.h>
#include "../application/chassis/oid_stop_guard.h"
int main(void)
{
  OidStopGuard g={0};
  OidStopGuard_Update(&g,1,1,400,400,100);
  assert(OidStopGuard_RequestDue(&g,100));
  OidStopGuard_MarkRequest(&g,100);
  assert(!OidStopGuard_RequestDue(&g,199));
  assert(OidStopGuard_RequestDue(&g,200));
  OidStopGuard_Update(&g,1,1,400,400,1599); assert(!g.fault);
  OidStopGuard_Update(&g,1,1,50,-50,1600); assert(!g.fault);
  OidStopGuard_Update(&g,1,1,51,0,1601); assert(g.fault);
  OidStopGuard_Update(&g,0,1,0,0,2000); assert(g.fault && g.active);
  g=(OidStopGuard){0};
  OidStopGuard_Update(&g,1,1,400,0,100);
  OidStopGuard_Update(&g,1,0,400,0,2000); assert(!g.fault); /* stale is not proof */
  OidStopGuard_Update(&g,1,1,-51,0,2001); assert(g.fault);
  g=(OidStopGuard){0};
  OidStopGuard_Update(&g,1,1,0,0,0);
  OidStopGuard_Update(&g,0,1,400,0,500); assert(!g.active);
  OidStopGuard_Update(&g,1,1,400,0,1000);
  OidStopGuard_Update(&g,1,1,0,0,2499); assert(!g.fault);
  g=(OidStopGuard){0};
  OidStopGuard_Update(&g,1,1,400,0,UINT32_MAX-1000U);
  OidStopGuard_MarkRequest(&g,UINT32_MAX-50U);
  assert(!OidStopGuard_RequestDue(&g,48)); assert(OidStopGuard_RequestDue(&g,49));
  OidStopGuard_Update(&g,1,1,400,0,499); assert(g.fault);
  puts("PASS: 100ms retry, 1500ms stop deadline, +/-50 window, stale feedback, restart-only fault latch, tick rollover");
}
