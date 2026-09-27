# V6 主机自检

当前固件为双闭环；测试中的单轮模式仅在各自编译单元内覆盖，不修改固件配置。

| 测试 | 范围 |
|---|---|
| test_dual_steer | 右单轮开环、左右接口、独立零点、健康旁路、释放/换向/失控、2000点阿克曼几何 |
| test_left_calibration | 左单轮开环分支 |
| test_closed_steer | 零点3189/3073、28°对应76.125212°/66.915019°、目标80°/反馈82°、增益及故障停；OID ID1/2、方向+1/+1、4900上限、参数确认1、临时限速0 |
| test_command | CH3/CH7/CH1映射与均值；CH5健康帧确认、混合异常及隐藏丢帧；700 ms撤权/恢复边界、重复初始化、诊断计数 |
| test_debug_sbus | SBUS原因/累计字段、最坏位宽下不截断日志、发送忙时跳过 |
| test_vehicle_status | 共用提示状态的只读性；故障/授权/模式/就绪优先级，自动与单轮维护 |
| test_lighting_status / test_lighting_zero_exit | 前后白色渐变流水方向、单侧琥珀呼吸的峰谷/缓升缓降/对称羽化、白光覆盖不染色、另一侧隔离；状态优先级、双端保留区、亮度限幅、1～21灯珠边界、初始化失败/DMA忙/运动与呼吸时钟回绕、零退出阈值 |
| test_ws2812_recovery | 实际灯带驱动异步终止、DMA状态/EN双重确认、在途缓存保护、迟到回调、失败重试、另一通道隔离和tick回绕 |
| test_buzzer_status / test_buzzer_patterns | 软件启动/首次上线/真实就绪/锁车事件、持续故障稳定与抢占、故障恢复不补旧音、有限音段调度、长时间跳变和tick回绕 |
| test_steer_disabled | 闭环开关关闭时双侧停止 |
| test_steer_slew | V4轴角300°/s、独立OID差速1.6/s、换向、调度时间上限、停止重臂 |
| test_oid_stop_guard | 即时零速、100 ms重发、恢复运动不锁存、tick回绕 |
| test_oid_reverse_guard | 双零速提交前不放行；连续两组新鲜双侧低速反馈、回中/改意图、单侧未停/离线/tick回绕 |
| test_oid_diagnostics | 真实组帧和调度器；丢零速、慢回包、诊断超时、同代目标合并、运动回读门、WAIT_REARM/VERIFY与连续命令下心跳/状态不饥饿 |
| test_oid_mode_recovery | 锁车低速新回读确认模式不符时双侧清零、持续心跳、等待重臂；拒绝未请求/CRC错误/迟到回包触发恢复，行走不增加回读 |
| test_uart_idle_irq | 真实USART1 IRQ的IDLE标志/使能/清除顺序；PE/FE/NE/ORE与IDLE并发、同步/异步DMA中止恢复、拒绝旧缓冲分发 |
| test_rs485_frame_timeout | 真实RS485接收回调与分帧计时；调用时间早于ISR接收时间、正常空闲、tick回绕及中断状态保持 |
| test_oid_stop_guard_field_cases | 60秒停车持续重发零速、1000次停车/起步不要求重启 |
| test_chassis_control | 真实应用链：MAIN1即时正反/回中、长时间停止后起步、飞控/OID/转向/锁车门控 |
| test_pwm_input | 真实PWM捕获：MAIN1窗口1及4帧预热、MAIN2窗口8、非法范围/超时恢复、计数器回绕与长脉冲 |
| test_sbus_timestamp | 真实SBUS维护：时间竞争、300 ms边界、tick回绕、1/2/3丢帧及50 ms上限、瞬态failsafe事件、UART/DMA恢复失败重试 |
| test_sbus_command_integration | 实际DMA收帧→SBUS解析→命令门控；软丢帧保持最后健康通道、超限停车、隐藏失控不丢失、CH5混合异常、700 ms撤权与UART恢复 |
| test_encoder_timestamp | 真实编码器健康判定：原子时间快照、100 ms边界、tick回绕和tick零合法反馈 |

安装提供 `gcc` 的 MinGW 工具链并加入 PATH，在DSZT_V6目录运行：

```powershell
./tests/run_host_tests.ps1
# 若gcc不在PATH，可通过 -Gcc 指定本机编译器完整路径。
```

脚本逐个重新编译并执行所列测试程序，遇到失败立即停止；产物位于 `tests/.build/`，不入库，实际数量由脚本汇总。

测试直接编译应用或BSP实现，硬件及位置内环为模拟接口；参数关系断言不等于真实反馈越界停机测试。未覆盖真实HAL时序、编码器动态解码、机械止挡或OID回包，不替代实车验证。测试及stubs不加入MDK固件，脚本不会访问串口或下载器。
