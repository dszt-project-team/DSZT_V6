# DSZT_V6

基于 RoboMaster A 型开发板（STM32F427IIH6）的四轮金属探测车控制固件。前轴双 OID 轮毂驱动、双 5840 转向与双 MT6826S 反馈，后轴固定从动。

- STM32 HAL、FreeRTOS / CMSIS-RTOS v1、MDK-ARM / ARMCC。
- MC7/SBUS 手动控制；飞控 MAIN1 行走、MAIN2 转向。
- 双转向位置闭环、前轴阿克曼差速、OID 心跳与安全恢复。
- 双 WS2812、蜂鸣器、UART7 单源只读诊断。
- 金属探测 UART8、USART2、USART6 仅预留接口；探测盘协议尚未实现。IMU 业务关闭，无 A22、RK3566 遥测、前后定义切换及外八控制。

| 当前参数 | 配置 |
|---|---|
| 轴距 / 前后轮距 | 610 / 500、500 mm |
| 左 / 右软件零点 | 3148 / 3006 |
| 零油门满舵内 / 外轮目标 | 18° / 14.39° |
| 输出轴目标 / 反馈保护 | 53° / 55° |
| 左 / 右 OID ID | 1 / 2；方向由驱动器适配，MCU系数+1/+1 |
| OID 最大速度 / 加减速度 | 4900 ERPM / 4900 ERPM/s |
| 手动速度上限 | CH7选择100～4900 ERPM，无额外限速 |
| 转向模式 | 双闭环；单轮开环关闭 |
| 正反换向 | 不等待双轮低速反馈；由OID内部速度斜坡执行 |

## 文档与工程

- [项目 Wiki](Doc/DSZT_V6_项目总Wiki.md)：接线、控制链路、架构调度、关键时序、安全逻辑与诊断。
- [A 板接线手册](Doc/DSZT_V6_接线手册/DSZT_V6_A板接线手册.md)：详细针脚、供电和信号注意事项。
- [双转向与前驱差速参数](Doc/阿克曼参数/DSZT_V6_前驱转向差速标定.md)：角度映射、阿克曼公式及结构调整参考。
- [MDK 工程](MDK-ARM/DSZT_V6.uvprojx)：目标 `DSZT_V6`，产物位于 `MDK-ARM/DSZT_V6/`。
- [CubeMX 配置](DSZT_V6.ioc)、[主机自检](tests/README.md)。

UART7使用115200 / 8N1，默认每100 ms输出OID状态；支持 `DBG GENERIC/OID/SBUS/MT6826S/FC/OFF` 和 `DBG?`，命令以换行结束，只改变诊断视图。

上电先CH5锁车、CH1/CH3回中。参数开关不绕过遥控、编码器、OID健康和回中释放；四台电机使用独立动力供电并保持信号共地。软件角度与停车保护不替代机械止挡和断动力措施。

## 构建与维护

1. 使用 Windows + Keil MDK，安装 ARMCC 5.06 update 7（build 960）及 `Keil.STM32F4xx_DFP 2.15.0`。
2. 打开 `MDK-ARM/DSZT_V6.uvprojx`，选择 `DSZT_V6` 目标，执行 Rebuild。工程使用相对路径，HAL、CMSIS、FreeRTOS 源码已随仓库保留。
3. 生成 `MDK-ARM/DSZT_V6/DSZT_V6.axf` 和 `.hex`。编译与下载是独立操作；主机自检方法见 [tests/README.md](tests/README.md)。

当前 CubeMX 配置为 6.18.1 / STM32Cube F4 1.28.3。重新生成前备份工程，保留 USER CODE、自定义文件和 USART1 IDLE 中断清除逻辑。主要可调宏位于 `application/*/*_config.h`，均附中文作用说明和调整约束。

本仓库保存固件、工程、正式文档、测试源码及相关硬件参考资料；不上传构建产物、本机串口日志、IDE个人缓存和旧车体模型。

## 来源与使用边界

应用分层参考[跃鹿电控通用嵌入式框架](https://github.com/tangguZZZ/basic_framework)。第三方库和硬件资料保留各自版权及许可说明，见 [第三方说明](THIRD_PARTY_NOTICES.md)。仓库未设置覆盖全部项目文件的统一开源许可证。
