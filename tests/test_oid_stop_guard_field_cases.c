#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../application/chassis/oid_stop_guard.h"

/* 停车模块不再接收反馈速度或健康参数：它仅决定何时重发零速。
 * 反馈异常仍由OID通信健康及安全状态机处理，本测试不代替整车门控验证。 */
int main(void)
{
  OidStopGuard g = {0};
  uint32_t now;
  unsigned retries = 0U;
  for (now = 100U; now <= 60100U; now += 10U)
  {
    OidStopGuard_Update(&g, 1U);
    if (OidStopGuard_RequestDue(&g, now))
    {
      retries++;
      OidStopGuard_MarkRequest(&g, now);
    }
  }
  assert(retries == 601U && g.active);
  /* 长时间停止后，新的运动许可不会被停车维护对象锁住。 */
  OidStopGuard_Update(&g, 0U);
  assert(!g.active && !OidStopGuard_RequestDue(&g, 60110U));
  /* 重复停车/起步不需要初始化或重启。 */
  for (now = 0U; now < 1000U; now++)
  {
    OidStopGuard_Update(&g, 1U);
    assert(OidStopGuard_RequestDue(&g, now));
    OidStopGuard_MarkRequest(&g, now);
    OidStopGuard_Update(&g, 0U);
    assert(!g.active && !g.requested);
  }
  puts("PASS: 60s idle maintains 100ms zeros; 1000 stop/start cycles never require restart");
  return 0;
}
