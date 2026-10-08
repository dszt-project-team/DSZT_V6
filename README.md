# DSZT_V6_FC

基于 DSZT_V6 的飞控专用 A 板固件：四轮底盘，前轮双 OID 行走、双 5840 转向及 MT6826S 位置闭环。MAIN1 是唯一行走输入，MAIN2 是唯一转向输入；不使用 MC7、SBUS、CH5 或 CH7。

本版本位于 `dszt-project-team/DSZT_V6` 仓库的 **`codex/dszt-v6-fc`** 独立分支；`main` 保留 MC7 版 V6，不互相替换。FC 分支保留本版本的控制与接线说明，原 V6 厂商手册及硬件附件仍在 `main` 的 `Doc/` 中。

## 使用边界

- 上电进入飞控控制流程，但不是上电立即执行非零指令：两路 PWM 健康、完成预热及连续双回中后才释放。
- 任一路输入异常时请求双 OID 零速、转向停机刹车；恢复后重新双回中。零速请求不等于机械瞬时停止。
- 仅靠 MAIN1/MAIN2 无法识别飞控是否解锁，也无法识别持续重复的合法非零错误目标。飞控未解锁或失控时必须输出中位或停止脉冲；独立物理动力切断不能省略。
- 当前默认正常双转向闭环。单轮开环为编译期维护模式，使用 MAIN2 点动，禁止行走。

## 工程与说明

| 项目 | 内容 |
|---|---|
| MCU / 工具链 | STM32F427IIHx / Keil MDK ARMCC 5 |
| 工程 / 目标 | [MDK-ARM/DSZT_V6_FC.uvprojx](MDK-ARM/DSZT_V6_FC.uvprojx) / `DSZT_V6_FC` |
| 外设配置 | [DSZT_V6_FC.ioc](DSZT_V6_FC.ioc)，HSE 12 MHz、SYSCLK 168 MHz、HAL TIM6、RTOS SysTick |
| 控制与诊断手册 | [项目 Wiki](Doc/DSZT_V6_FC_Wiki.md) |
| 接线 | [A 板接线表](Doc/DSZT_V6_FC_接线表.md) |
| 功能边界与移植方案 | [飞控专用方案](Doc/DSZT_V6_FC_功能边界.md) |
| 参数入口 | [底盘配置](application/chassis/chassis_config.h)、[命令配置](application/command/command_config.h)、[诊断配置](application/debug/debug_config.h) |
| 主机回归 | [测试说明](tests/README.md)；不连接硬件 |
| 第三方说明 | [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) |

直接用 MDK 打开工程并 Rebuild。产物为 `MDK-ARM/DSZT_V6_FC/DSZT_V6_FC.axf` 和 `.hex`。重新生成 CubeMX 后，应核对自定义工程分组、相对路径及 `main.c` 的 `RobotInit()`、`freertos.c` 的 `RobotTask()` 用户区钩子；不得把另一个 DSZT 工程的源文件路径编入本工程。

`./tools/check_project.ps1` 可只读检查工程文件/包含路径、SBUS 外设移除及本地文档链接；应用回归运行 `./tests/run_host_tests.ps1`。

本项目继承 V6 的机械、编码器零点和 OID 参数，独立维护飞控专用控制链。2026-10-08，使用者确认 DSZT_V6_FC 已完成多次落地测试，当前功能效果满足使用要求。故障注入和极端工况不因日常功能通过而视为全部验证，安全边界见 Wiki；原 DSZT_V6 保持独立。
