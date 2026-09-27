/* 蜂鸣器仅反馈软件状态，不参与解锁/停车控制；修改宏后需重新编译烧录。 */
#ifndef BUZZER_CONFIG_H
/* 头文件重复包含保护，不是功能开关。 */
#define BUZZER_CONFIG_H
/* 蜂鸣器总开关：1启用，0关闭；下列所有提示均受此开关约束。 */
#define BUZZER_APP_ENABLE                 1U
/* 开机提示：1播放一次260 ms上升三音，仅表示软件启动，不表示车辆解锁。 */
#define BUZZER_APP_BOOT_CUE_ENABLE        1U
/* 接收机首次健康上线提示：1播放短双音；重连不反复播报，也不表示已放行。 */
#define BUZZER_APP_RC_CUE_ENABLE          1U
/* 状态事件提示：1启用真实手动/自动放行及锁车音，0关闭；不按摇杆动作或仅切档发声。 */
#define BUZZER_APP_MODE_CUE_ENABLE        1U
/* 重复报警总开关：1启用以下持续故障/等待授权提示，0仅保留开机及事件音。 */
#define BUZZER_APP_ALARM_ENABLE           1U
/* 遥控失联报警：1启用；曾健康上线且失联持续达到稳定时间才提示，首次未连遥控不反复鸣叫。 */
#define BUZZER_APP_RC_FAULT_ENABLE        1U
/* 转向故障报警：1启用；使用统一车辆状态，不改变编码器或转向保护。 */
#define BUZZER_APP_STEER_FAULT_ENABLE     1U
/* OID故障报警：1启用；只做简短分组提示，左右及具体原因请看UART7。 */
#define BUZZER_APP_OID_FAULT_ENABLE       1U
/* 飞控故障报警：1启用；自动模式MAIN健康状态异常持续达到稳定时间才提示。 */
#define BUZZER_APP_FC_FAULT_ENABLE        1U
/* 等待启动授权提示：1启用；遥控曾上线后需重新锁车时，等待5 s再低频提示。 */
#define BUZZER_APP_STARTUP_WAIT_ENABLE    1U
/* 正常事件稳定时间，当前100 ms；仅抑制提示音跳变，不推迟控制响应。 */
#define BUZZER_APP_EVENT_STABLE_MS        100U
/* 故障提示稳定时间，当前300 ms；故障控制门立即生效，声音延后排除极短毛刺。 */
#define BUZZER_APP_FAULT_STABLE_MS        300U
/* 等待重新锁车的首次提醒延时，当前5000 ms；重复间隔由BSP音型固定为5 s。 */
#define BUZZER_APP_STARTUP_WAIT_MS        5000U
/* 首次上线音的有效等待期限，当前1000 ms；超时或故障时丢弃，避免事后补播旧事件。 */
#define BUZZER_APP_CONNECT_EVENT_TTL_MS   1000U

#endif
