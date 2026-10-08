# V6_FC 主机自检

固件默认双转向闭环；标定及配置关闭只在各测试编译单元覆盖宏，不改动正式固件配置。测试不会打开串口、下载器或写入控制板。

| 测试 | 范围 |
|---|---|
| test_fc_command / test_fc_command_calibration | 唯一飞控命令源、双轴原始+滤波回中、新样本释放、MAIN1正反映射、坏脉冲/超时累计事件、故障恢复、标定MAIN1持续停止门、时间回绕、输入注册失败 |
| test_fc_pwm_command_integration | 真实PWM捕获与真实命令层集成；4帧预热+回中、未被任务观察到的断流或非法脉冲不能保留旧授权 |
| test_chassis_control / test_chassis_calibration | 真实底盘应用、OID服务前清零、输入故障/模式错误、持续停车与起步、恢复时新200ms回中、只读回读门、标定OID硬禁止 |
| test_dual_steer / test_left_calibration | 右/左单轮开环、真实左右接口与独立零点、编码器健康旁路、换向等待、飞控失效、另一侧停止、2000点几何 |
| test_closed_steer / test_steer_disabled / test_steer_slew | 28°/80°/82°与当前零点参数、增益、双编码器故障锁存、关闭闭环、轴角300°/s、独立差速1.6/s及停止重臂 |
| test_pwm_input / test_encoder_timestamp | 实际捕获/健康代码、独立均值和预热、脉宽边界与长脉冲、累计断流去重、原子时间快照及tick回绕 |
| test_oid_stop_guard / test_oid_stop_guard_field_cases | 即时零速、100ms刷新、60s停车和1000次停车/启动不永久锁存 |
| test_oid_reverse_guard | 可选换向等待、新鲜配对反馈和零目标条件；默认固件关闭此功能 |
| test_oid_diagnostics / test_oid_mode_recovery | 真实Modbus组帧和调度器、连续命令与查询下心跳、慢回包、回读预算/门控、模式失配清零、WAIT_REARM/VERIFY恢复 |
| test_rs485_frame_timeout | 实际RS485回调与分帧；任务时间早于ISR、完整帧收取、临界区和tick回绕 |
| test_debug_fc | 五种只读视图、SBUS/未知命令拒绝、溢出和超长命令、最大字段宽度、忙时跳过 |
| test_vehicle_status / test_vehicle_status_config | FC故障优先级、配置禁用、真实就绪、标定隔离及所选编码器健康 |
| test_lighting_status / test_lighting_zero_exit | FC底光、前后白色光段、单侧羽化琥珀呼吸、状态端点、故障覆盖、1～21灯珠及DMA忙/时间回绕 |
| test_ws2812_recovery | DMA异步中止、缓存不可改写、迟到回调、失败重试、通道隔离 |
| test_buzzer_status / test_buzzer_patterns | 启动/真实自动就绪/等待及故障提示、声音去毛刺、优先级、无过期队列、有限调度与时间回绕 |

安装 MinGW GCC 后，在 DSZT_V6_FC 根目录执行：

```powershell
./tests/run_host_tests.ps1
# GCC 不在 PATH 时：
./tests/run_host_tests.ps1 -Gcc '你的工具链/bin/gcc.exe'
```

脚本使用 `-std=c99 -Wall -Wextra -Werror`，逐项重新编译并执行，失败即停止；程序数量由实际 runner 汇总。产物在 `tests/.build/`，不编入 MDK 固件。

测试使用硬件桩，部分位置内环为模拟接口；不是实际电机、编码器或 OID 的现场证明。FC固件的首次接线、PWM方向、断线/失控输出、机械限位和制动仍须单独硬件验证。
