/* DSZT_V6_FC 唯一命令源为 MAIN1/MAIN2；脉宽标定与健康门见 chassis_config.h。 */
#ifndef COMMAND_CONFIG_H
/* 头文件重复包含保护，不是运行开关。 */
#define COMMAND_CONFIG_H

/* 单轮开环 MAIN2 指令是否旁路均值：当前1使用最新有效原始脉宽便于点动，0沿用8点均值。
 * 仅改变开环目标，不旁路两路输入健康、MAIN1回中或首次双轴回中检查；正常闭环始终使用均值。 */
#define COMMAND_FC_CAL_USE_RAW_STEERING 1U

#if COMMAND_FC_CAL_USE_RAW_STEERING > 1U
#error "FC calibration raw steering switch must be 0 or 1"
#endif
#endif
