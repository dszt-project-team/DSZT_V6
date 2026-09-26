# V6 主机自检

当前固件为双闭环；测试中的单轮模式仅在各自编译单元内覆盖，不修改固件配置。

| 测试 | 范围 |
|---|---|
| test_dual_steer | 右单轮开环、左右接口、独立零点、健康旁路、释放/换向/失控、2000点阿克曼几何 |
| test_left_calibration | 左单轮开环分支 |
| test_closed_steer | 零点3148/3006、18°对应48.937636°/47.388730°、目标53°/反馈55°、增益及故障停；OID ID1/2、方向+1/+1、4900上限、参数确认1、临时限速0 |
| test_command | CH3死区、CH7、10 ms CH1三点均值、模式与失控恢复、开环原始输入 |
| test_steer_disabled | 闭环开关关闭时双侧停止 |
| test_steer_slew | V4轴角300°/s、独立OID差速1.6/s、换向、调度时间上限、停止重臂 |
| test_oid_stop_guard | 100 ms重发、1.5秒停车期限、±50窗口、新鲜度、故障锁存、tick回绕 |
| test_oid_reverse_guard | 双零速提交前不放行；连续两组新鲜双侧低速反馈、回中/改意图、单侧未停/离线/tick回绕 |
| test_oid_diagnostics | 真实组帧和调度器；丢零速、慢回包、诊断超时、同代目标合并、运动回读门、WAIT_REARM/VERIFY与连续命令下心跳/状态不饥饿 |
| test_uart_idle_irq | 真实USART1 IRQ的IDLE标志/使能判断及清除顺序 |
| test_rs485_frame_timeout | 真实RS485接收回调与分帧计时；调用时间早于ISR接收时间、正常空闲、tick回绕及中断状态保持 |
| test_oid_stop_guard_field_cases | 当前停车锁存边界：停稳后单样本、反馈恢复、500 ms缓存状态、通信恢复；不表示现场根因已确认 |

安装提供 `gcc` 的 MinGW 工具链并加入 PATH，在DSZT_V6目录运行：

```powershell
./tests/run_host_tests.ps1
# 若gcc不在PATH，可通过 -Gcc 指定本机编译器完整路径。
```

脚本逐个重新编译并执行12个测试程序，遇到失败立即停止；产物位于 `tests/.build/`，不入库。

测试直接编译应用或BSP实现，硬件及位置内环为模拟接口；参数关系断言不等于真实反馈越界停机测试。未覆盖真实HAL时序、编码器动态解码、机械止挡或OID回包，不替代实车验证。测试及stubs不加入MDK固件，脚本不会访问串口或下载器。
