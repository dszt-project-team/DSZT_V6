/* DSZT_V6 buzzer 配置：修改宏后需重新编译下载；中文注释说明当前作用及调整约束。
 * 0/1开关按各项适用范围使用，不代表绕过其他安全门。
 */
#ifndef BUZZER_CONFIG_H
/* 头文件重复包含保护，不是蜂鸣器开关，请勿修改。 */
#define BUZZER_CONFIG_H
/* 板载蜂鸣器总开关：当前1启用，0不初始化蜂鸣器业务且不更新提示；子开关受总开关约束。 */
#define BUZZER_APP_ENABLE           1U
/* OID故障提示：当前1开启，0静音该类提示；按左右非零驱动器fault选择音型，不覆盖所有安全门。 */
#define BUZZER_APP_OID_FAULT_ENABLE 1U
/* 飞控MAIN超时提示：当前1开启，0关闭；自动模式双路曾健康后才报警，提示优先于OID故障，不改变自动保护。 */
#define BUZZER_APP_FC_FAULT_ENABLE  1U
/* 遥控离线变在线提示音：当前1开启，0关闭；仅连接提示，不是持续失控报警。 */
#define BUZZER_APP_RC_CUE_ENABLE    1U
/* 车辆模式变化提示音：当前1开启，0关闭；无持续故障提示时播放，不改变模式确认或安全门。 */
#define BUZZER_APP_MODE_CUE_ENABLE  1U
#endif
