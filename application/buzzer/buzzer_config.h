/* 蜂鸣器只反馈软件状态，不参与授权/停车；修改宏后需重新编译下载。 */
#ifndef BUZZER_CONFIG_H
/* 头文件重复包含保护，不是功能开关。 */
#define BUZZER_CONFIG_H
/* 总开关：1启用蜂鸣器，0关闭；下列开关均受本项约束。 */
#define BUZZER_APP_ENABLE                 1U
/* 开机提示：1播放一次260 ms上升三音；只表示软件启动，不表示车辆放行。 */
#define BUZZER_APP_BOOT_CUE_ENABLE        1U
/* 就绪提示：1在自动链真正放行且稳定后播放短三音；标定状态不播放行走就绪音。 */
#define BUZZER_APP_READY_CUE_ENABLE       1U
/* 重复提示总开关：1启用持续故障/等待提示，0仅保留启动与就绪音。 */
#define BUZZER_APP_ALARM_ENABLE           1U
/* 转向故障提示：1启用；不改变编码器健康门或转向故障锁存。 */
#define BUZZER_APP_STEER_FAULT_ENABLE     1U
/* OID故障提示：1启用；左右及具体原因由UART7读取。 */
#define BUZZER_APP_OID_FAULT_ENABLE       1U
/* 飞控输入故障提示：1启用；两路曾健康上线后才报警，首次未接飞控不循环鸣叫。 */
#define BUZZER_APP_FC_FAULT_ENABLE        1U
/* 等待回中/重臂提示：1启用，持续等待5 s后低频短双音；不表示飞控已解锁。 */
#define BUZZER_APP_RELEASE_WAIT_ENABLE    1U
/* 就绪事件稳定时间，当前100 ms；仅抑制声音毛刺，不延迟运动控制。 */
#define BUZZER_APP_EVENT_STABLE_MS        100U
/* 故障声音稳定时间，当前300 ms；控制保护仍按原时序立即处理。 */
#define BUZZER_APP_FAULT_STABLE_MS        300U
/* 等待回中提示首次延迟，当前5000 ms；重复音型周期为5 s。 */
#define BUZZER_APP_RELEASE_WAIT_MS        5000U
#endif
