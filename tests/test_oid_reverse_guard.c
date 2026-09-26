#include <assert.h>
#include <stdio.h>
#include "../application/chassis/oid_reverse_guard.h"

int main(void)
{
  OidReverseGuard g={0};
  assert(OidReverseGuard_Update(&g,1000,1,0,0,0,100,100,100));
  assert(g.direction==1 && !g.waiting);
  assert(!OidReverseGuard_Update(&g,-1000,1,0,1000,1000,110,110,110));
  assert(g.waiting && !g.zero_pairs);
  /* Even fresh zeros cannot release before BOTH zero writes were submitted. */
  assert(!OidReverseGuard_Update(&g,-1000,1,0,0,0,210,210,210));
  assert(!OidReverseGuard_Update(&g,-1000,1,0,0,0,310,310,310));
  assert(!g.zero_pairs);
  assert(!OidReverseGuard_Update(&g,-1000,1,1,0,0,410,410,410));
  assert(g.zero_pairs==1);
  /* Re-reading the same samples, or only one updated side, cannot confirm. */
  assert(!OidReverseGuard_Update(&g,-1000,1,1,0,0,410,410,420));
  assert(!OidReverseGuard_Update(&g,-1000,1,1,0,0,510,410,510));
  assert(g.zero_pairs==1);
  assert(OidReverseGuard_Update(&g,-700,1,1,0,0,510,518,520));
  assert(!g.waiting && g.direction==-1);
  /* Neutral does not erase the preceding direction while wheels coast. */
  assert(OidReverseGuard_Update(&g,0,1,1,-300,-300,610,610,610));
  assert(!OidReverseGuard_Update(&g,900,1,0,-300,-300,620,620,620));
  /* Changing the stick again keeps the stop barrier, then takes latest intent. */
  assert(!OidReverseGuard_Update(&g,-200,1,1,0,0,710,710,710));
  assert(OidReverseGuard_Update(&g,-200,1,1,0,0,810,810,810));
  assert(g.direction==-1);
  assert(!OidReverseGuard_Update(&g,300,1,0,-200,-200,820,820,820));
  assert(!OidReverseGuard_Update(&g,300,1,1,0,0,910,910,910));
  /* Stale, bad or moving feedback resets consecutive confirmation. */
  assert(!OidReverseGuard_Update(&g,300,1,1,0,0,920,920,1200));
  assert(g.zero_pairs==0);
  assert(!OidReverseGuard_Update(&g,300,0,1,0,0,1210,1210,1210));
  assert(!OidReverseGuard_Update(&g,300,1,1,0,51,1310,1310,1310));
  assert(!OidReverseGuard_Update(&g,300,1,1,-50,50,1410,1410,1410));
  assert(OidReverseGuard_Update(&g,0,1,1,0,0,1510,1510,1510));
  assert(!g.waiting && g.direction==0); /* No queued old forward command. */
  /* Actual opposite motion also blocks a first command after reset. */
  g=(OidReverseGuard){0};
  assert(!OidReverseGuard_Update(&g,100,1,0,-80,0,100,100,100));
  g=(OidReverseGuard){0};
  assert(OidReverseGuard_Update(&g,100,1,0,0,0,UINT32_MAX-200,UINT32_MAX-200,UINT32_MAX-200));
  assert(!OidReverseGuard_Update(&g,-100,1,0,100,100,UINT32_MAX-100,UINT32_MAX-100,UINT32_MAX-100));
  assert(!OidReverseGuard_Update(&g,-100,1,1,0,0,20,25,30));
  assert(OidReverseGuard_Update(&g,-100,1,1,0,0,120,125,130));
  puts("PASS: reverse zero barrier, two fresh paired samples, zero TX prerequisite, neutral/changed intent, stale/fault/one-wheel motion, rollover");
  return 0;
}
