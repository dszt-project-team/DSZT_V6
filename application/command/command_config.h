/* DSZT_V6 command 配置：修改宏后需重新编译下载；中文注释说明当前作用及调整约束。
 * 0/1开关按各项适用范围使用，不代表绕过其他安全门。
 */
#ifndef COMMAND_CONFIG_H
/* 头文件重复包含保护，不是遥控功能开关，请勿修改。 */
#define COMMAND_CONFIG_H

/* 手动转向通道下标，当前0对应CH1；SBUS下标从0开始，范围0～15，不是通道标签数字。 */
#define COMMAND_CH_STEERING               0U
/* 手动油门通道下标，当前2对应CH3；同时用于启动及手动回中授权，改动需同步遥控器通道。 */
#define COMMAND_CH_THROTTLE               2U
/* 模式开关通道下标，当前4对应CH5；选择锁车/手动/自动，范围0～15。 */
#define COMMAND_CH_MODE                   4U
/* 手动速度上限旋钮下标，当前6对应CH7；不改变飞控自动上限，范围0～15。 */
#define COMMAND_CH_SPEED_LIMIT            6U
/* CH3归一化等效中位，当前1500 μs；不是SBUS线实际PWM，与转向中位独立。 */
#define COMMAND_CENTER_US               1500U
/* CH3中位死区半宽，当前50 μs；中位±此值输出0，区外扣除死区再线性映射，加大更容易保持零速。 */
#define COMMAND_DEADBAND_US                50U
/* CH3满幅距中位的偏移量，当前500 μs；须>死区，当前端点1000/2000，超出钳位±1000‰。 */
#define COMMAND_THROTTLE_MAX_OFFSET_US    500U
/* CH1转向归一化等效中位，当前1500 μs；调整遥控中位映射，不调整编码器物理零点。 */
#define COMMAND_STEER_CENTER_US          1500
/* CH1转向死区半宽，当前45 μs；增大减少中位动作但损失细小输入，须小于左右满幅偏移。 */
#define COMMAND_STEER_DEADBAND_US          45
/* CH1左满幅相对中位的偏移量，当前243 μs，即1500−243=1257；按实际遥控端点标定。 */
#define COMMAND_STEER_LEFT_OFFSET_US      243
/* CH1右满幅相对中位的偏移量，当前257 μs，即1500+257=1757；左右不要求对称。 */
#define COMMAND_STEER_RIGHT_OFFSET_US     257

/* CH1滑动均值开关：当前1开启，0原值；仅滤转向，不滤CH3油门。 */
#define COMMAND_STEER_FILTER_ENABLE         1U
/* CH1均值样本数，当前3，范围1～16；加大更平滑也增加跟随延迟，1相当于单样本。 */
#define COMMAND_STEER_FILTER_WINDOW         3U
/* CH1均值更新间隔，当前10 ms；不是RTOS循环间隔，加大会延长滤波时间尺度。 */
#define COMMAND_STEER_FILTER_PERIOD_MS     10U

/* 单轮开环是否旁路CH1均值：当前1直接用原始CH1，0用滤波值；仅标零维护模式生效，便于回中及时停止。 */
#define COMMAND_CAL_USE_RAW_STEERING        1U
/* CH5锁车识别窗口下界，当前900 μs，含边界；须≤LOCK_MAX且不与其他档位重叠。 */
#define COMMAND_SWITCH_LOCK_MIN_US        900U
/* CH5锁车窗口上界，当前1200 μs，含边界；加宽可容忍端点偏差，但不能侵入手动窗口。 */
#define COMMAND_SWITCH_LOCK_MAX_US       1200U
/* CH5手动窗口下界，当前1400 μs，含边界；与锁车及自动窗口保持分离。 */
#define COMMAND_SWITCH_MANUAL_MIN_US     1400U
/* CH5手动窗口上界，当前1600 μs，含边界；与下界共同覆盖实际中档脉宽。 */
#define COMMAND_SWITCH_MANUAL_MAX_US     1600U
/* CH5飞控自动窗口下界，当前1800 μs，含边界；自动仍需MAIN健康和双轴回中。 */
#define COMMAND_SWITCH_AUTO_MIN_US       1800U
/* CH5飞控自动窗口上界，当前2100 μs，含边界；各窗口之外按非法档位处理。 */
#define COMMAND_SWITCH_AUTO_MAX_US       2100U
/* CH5新档位连续健康新SBUS帧确认数，当前2；连续偏离原档的非法/其他档位合并计数，未确认新档则锁车，须1～255。 */
#define COMMAND_SWITCH_CONFIRM_FRAMES       2U
/* 手动油门连续回中释放时间，当前200 ms；增大更保守但放行更慢，转向另有独立回中门。 */
#define COMMAND_CENTER_RELEASE_MS          200U
/* 持续失控后撤销启动授权的时间，当前700 ms；不是SBUS离线300 ms阈值。达到后须重新CH5锁车、CH3回中。 */
#define COMMAND_REVOKE_ARM_MS               700U
/* CH7最低手动速度上限，当前100 ERPM；不是最小起转命令，CH3回中仍为0，须≤最大上限。 */
#define COMMAND_MIN_SPEED_LIMIT_ERPM        100U
/* CH7最高手动速度上限，当前4900 ERPM；不应超过驱动器配置能力，不控制自动模式上限。 */
#define COMMAND_MAX_SPEED_LIMIT_ERPM       4900U
/* CH7旋钮最小归一化等效脉宽，当前1000 μs；对应最低速度上限，低于此值钳位。 */
#define COMMAND_LIMIT_KNOB_MIN_US          1000U
/* CH7旋钮最大归一化等效脉宽，当前2000 μs；对应最高速度上限，须>最小端点。 */
#define COMMAND_LIMIT_KNOB_MAX_US          2000U

#if COMMAND_STEER_FILTER_WINDOW < 1U || COMMAND_STEER_FILTER_WINDOW > 16U
#error "CH1 filter window must be 1..16"
#endif
#if COMMAND_SWITCH_CONFIRM_FRAMES < 1U || COMMAND_SWITCH_CONFIRM_FRAMES > 255U
#error "CH5 confirmation count must be 1..255"
#endif
#if COMMAND_THROTTLE_MAX_OFFSET_US <= COMMAND_DEADBAND_US
#error "Throttle endpoint must exceed its deadband"
#endif
#if COMMAND_STEER_LEFT_OFFSET_US <= COMMAND_STEER_DEADBAND_US || COMMAND_STEER_RIGHT_OFFSET_US <= COMMAND_STEER_DEADBAND_US
#error "Steering endpoints must exceed the deadband"
#endif
#if COMMAND_LIMIT_KNOB_MAX_US <= COMMAND_LIMIT_KNOB_MIN_US || COMMAND_MAX_SPEED_LIMIT_ERPM < COMMAND_MIN_SPEED_LIMIT_ERPM
#error "Invalid CH7 range"
#endif
#if COMMAND_CH_STEERING > 15U || COMMAND_CH_THROTTLE > 15U || COMMAND_CH_MODE > 15U || COMMAND_CH_SPEED_LIMIT > 15U
#error "SBUS channel index must be 0..15"
#endif
#endif
