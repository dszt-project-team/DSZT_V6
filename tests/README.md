# V6 主机自检

当前固件为双闭环；测试中的单轮模式仅在各自编译单元内覆盖，不修改固件配置。

| 测试 | 范围 |
|---|---|
| test_dual_steer | 右单轮开环、左右接口、独立零点、健康旁路、释放/换向/失控、2000点阿克曼几何 |
| test_left_calibration | 左单轮开环分支 |
| test_closed_steer | 零点3189/3073、28°对应76.125212°/66.915019°、目标80°/反馈82°、增益及故障停；OID ID1/2、方向+1/+1、4900上限、参数确认1、临时限速0 |
| test_command | CH3死区、CH7、10 ms CH1三点均值、模式与失控恢复、开环原始输入 |
| test_steer_disabled | 闭环开关关闭时双侧停止 |
| test_steer_slew | V4轴角300°/s、独立OID差速1.6/s、换向、调度时间上限、停止重臂 |
| test_oid_stop_guard | 即时零速、100 ms重发、恢复运动不锁存、tick回绕 |
| test_oid_reverse_guard | 双零速提交前不放行；连续两组新鲜双侧低速反馈、回中/改意图、单侧未停/离线/tick回绕 |
| test_oid_diagnostics | 真实组帧和调度器；丢零速、慢回包、诊断超时、同代目标合并、运动回读门、WAIT_REARM/VERIFY与连续命令下心跳/状态不饥饿 |
| test_oid_mode_recovery | 锁车低速新回读确认模式不符时双侧清零、持续心跳、等待重臂；拒绝未请求/CRC错误/迟到回包触发恢复，行走不增加回读 |
| test_uart_idle_irq | 真实USART1 IRQ的IDLE标志/使能判断及清除顺序 |
| test_rs485_frame_timeout | 真实RS485接收回调与分帧计时；调用时间早于ISR接收时间、正常空闲、tick回绕及中断状态保持 |
| test_oid_stop_guard_field_cases | 60秒停车持续重发零速、1000次停车/起步不要求重启 |
| test_chassis_control | 真实应用链：MAIN1即时正反/回中、长时间停止后起步、飞控/OID/转向/锁车门控 |
| test_pwm_input | 真实PWM捕获：MAIN1窗口1及4帧预热、MAIN2窗口8、非法范围/超时恢复、计数器回绕与长脉冲 |
| test_sbus_timestamp | 真实SBUS维护：新帧与旧任务时间竞争、300 ms边界、tick回绕与中断状态 |
| test_encoder_timestamp | 真实编码器健康判定：原子时间快照、100 ms边界、tick回绕和tick零合法反馈 |

安装提供 `gcc` 的 MinGW 工具链并加入 PATH，在DSZT_V6目录运行：

```powershell
./tests/run_host_tests.ps1
# 若gcc不在PATH，可通过 -Gcc 指定本机编译器完整路径。
```

脚本逐个重新编译并执行所列测试程序，遇到失败立即停止；产物位于 `tests/.build/`，不入库，实际数量由脚本汇总。

测试直接编译应用或BSP实现，硬件及位置内环为模拟接口；参数关系断言不等于真实反馈越界停机测试。未覆盖真实HAL时序、编码器动态解码、机械止挡或OID回包，不替代实车验证。测试及stubs不加入MDK固件，脚本不会访问串口或下载器。
