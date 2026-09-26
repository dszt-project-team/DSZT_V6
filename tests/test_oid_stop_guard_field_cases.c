#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../application/chassis/oid_stop_guard.h"

/* 诊断测试：重现当前停车锁存的边界，不代表已证明外场故障根因。
 * 健康谓词复刻 ChassisApp_Task -> FrontDrive_IsOnline 的现有约束：
 * 双侧状态时间戳非零、年龄 <= 500 ms、驱动器 fault=0。
 * 此文件不进入 MDK 固件，也不改变任何参数或恢复策略。 */
static uint8_t StatusesHealthy(uint32_t now, uint32_t left_ms,
                              uint32_t right_ms, uint8_t left_fault,
                              uint8_t right_fault)
{
  return (uint8_t)(left_ms != 0U && right_ms != 0U &&
      now - left_ms <= 500U && now - right_ms <= 500U &&
      left_fault == 0U && right_fault == 0U);
}

static void TestOneSampleAfterLongStandstill(void)
{
  OidStopGuard g = {0};
  OidStopGuard_Update(&g, 1U, 1U, 0, 0, 100U);
  OidStopGuard_Update(&g, 1U, 1U, 0, 0, 60100U);
  assert(g.active != 0U && g.fault == 0U);
  /* 已停稳一分钟后，单次 +51 ERPM 已足以触发永久锁存。 */
  OidStopGuard_Update(&g, 1U, 1U, 51, 0, 60110U);
  assert(g.fault != 0U);
  assert(OidStopGuard_RequestDue(&g, 60110U) != 0U);
  OidStopGuard_MarkRequest(&g, 60110U);
  assert(OidStopGuard_RequestDue(&g, 60209U) == 0U);
  assert(OidStopGuard_RequestDue(&g, 60210U) != 0U);
  puts("PASS: one 51 ERPM sample after long standstill latches; zero retries remain active");
}

static void TestFeedbackRecoveryCannotClear(void)
{
  OidStopGuard g = {0};
  OidStopGuard_Update(&g, 1U, 1U, 0, 0, 100U);
  OidStopGuard_Update(&g, 1U, 1U, 0, -51, 1600U);
  assert(g.fault != 0U);
  OidStopGuard_Update(&g, 1U, 1U, 0, 0, 1610U);
  OidStopGuard_Update(&g, 1U, 1U, 0, 0, 10000U);
  assert(g.fault != 0U);
  /* 即使调用方随后请求运动，该对象也没有运行时清除入口。 */
  OidStopGuard_Update(&g, 0U, 1U, 0, 0, 10010U);
  assert(g.fault != 0U && g.active != 0U);
  puts("PASS: later zero feedback and a motion request do not clear the stop latch");
}

static void TestCachedStatusAgeBoundary(void)
{
  OidStopGuard at_limit = {0};
  OidStopGuard beyond_limit = {0};
  OidStopGuard_Update(&at_limit, 1U, 1U, 400, 400, 100U);
  OidStopGuard_Update(&beyond_limit, 1U, 1U, 400, 400, 100U);

  /* 最后一次状态在1100 ms；到停车期限1600 ms时，其年龄恰好500 ms。
   * 此时传入的并不是新帧，但上层仍会给出 healthy=1。 */
  assert(StatusesHealthy(1600U, 1100U, 1100U, 0U, 0U) != 0U);
  OidStopGuard_Update(&at_limit, 1U,
      StatusesHealthy(1600U, 1100U, 1100U, 0U, 0U), 120, 120, 1600U);
  assert(at_limit.fault != 0U);

  assert(StatusesHealthy(1600U, 1099U, 1099U, 0U, 0U) == 0U);
  OidStopGuard_Update(&beyond_limit, 1U,
      StatusesHealthy(1600U, 1099U, 1099U, 0U, 0U), 120, 120, 1600U);
  assert(beyond_limit.fault == 0U);
  puts("PASS: cached 500 ms-old speed can latch; 501 ms-old speed is excluded");
}

static void TestCommunicationRecoveryUsesOldDeadline(void)
{
  OidStopGuard g = {0};
  OidStopGuard_Update(&g, 1U, 1U, 400, 400, 100U);
  OidStopGuard_Update(&g, 1U, 0U, 400, 400, 1600U);
  OidStopGuard_Update(&g, 1U, 0U, 400, 400, 2990U);
  assert(g.fault == 0U && g.begin_ms == 100U);

  /* 通信恢复后首个健康状态仍为80 ERPM：沿用故障前停车期限，立即锁存。 */
  OidStopGuard_Update(&g, 1U,
      StatusesHealthy(3000U, 3000U, 3000U, 0U, 0U), 80, 80, 3000U);
  assert(g.fault != 0U && g.begin_ms == 100U);
  OidStopGuard_Update(&g, 1U, 1U, 0, 0, 3100U);
  assert(g.fault != 0U);
  puts("PASS: first healthy moving feedback after an outage uses the old deadline and latches");
}

int main(void)
{
  TestOneSampleAfterLongStandstill();
  TestFeedbackRecoveryCannotClear();
  TestCachedStatusAgeBoundary();
  TestCommunicationRecoveryUsesOldDeadline();
  puts("Diagnostic behavior reproduced; this is not a reproduction of the field incident.");
  return 0;
}
