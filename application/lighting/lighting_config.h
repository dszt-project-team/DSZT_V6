/* DSZT_V6_FC lighting 配置：修改宏后需重新编译下载；中文注释说明当前作用及调整约束。
 * 0/1开关按各项适用范围使用，不代表绕过其他安全门。
 */
#ifndef LIGHTING_CONFIG_H
/* 头文件重复包含保护，不是灯带开关，请勿修改。 */
#define LIGHTING_CONFIG_H

/* 双WS2812应用开关：当前1启用，0不初始化灯带业务且不刷新；不保证关闭前灯带已有颜色自动熄灭。 */
#define LIGHTING_APP_ENABLE                  1U
/* 每条灯带LED数量，当前21；两路相同，须1～WS2812_STRIP_MAX_LED_COUNT，增大会增加传输时间。 */
#define LIGHTING_APP_LED_COUNT              21U
/* 灯带应用刷新周期，当前50 ms；允许20~1000 ms，只影响灯效帧率，不修改WS2812位时序。 */
#define LIGHTING_APP_FRAME_PERIOD_MS        50U
/* 左灯带像素顺序：当前0正序；数据输入端在车头，逻辑索引0始终表示车头，1用于反向布线。 */
#define LIGHTING_APP_LEFT_REVERSED           0U
/* 右灯带像素顺序：当前0正序；数据输入端同样在车头，不因安装于右侧而镜像，也不交换接口。 */
#define LIGHTING_APP_RIGHT_REVERSED          0U
/* 单个RGB分量亮度上限，当前32/255；允许1~40，所有底色、故障色和叠加灯效均受此限幅。 */
#define LIGHTING_APP_MAX_BRIGHTNESS         32U
/* 每端保留的状态像素数，当前2；不被行进/转向覆盖，短灯带不足时仅显示状态底色。 */
#define LIGHTING_APP_STATUS_END_PIXELS       2U
/* 白色运动光段开关：1启用、0关闭；前进尾到头、后退头到尾，仅影响视觉，不改变控制或安全门。 */
#define LIGHTING_APP_MOTION_EFFECT_ENABLE    1U
/* 判断命令运动方向的平均目标死区，当前50 ERPM；允许1~65535，低速100仍可显示，不代表实测速率。 */
#define LIGHTING_APP_MOTION_DEADBAND_ERPM    50L
/* 光段走过内部像素的周期，当前900 ms；允许100~60000 ms，越小流动越快。 */
#define LIGHTING_APP_MOTION_CYCLE_MS        900U
/* 行进光段最大长度，当前3像素；允许1~WS2812_STRIP_MAX_LED_COUNT，短灯带自动截短。 */
#define LIGHTING_APP_MOTION_SEGMENT_PIXELS    3U
/* 左右转向灯开关：1启用、0关闭；只在转向已释放的飞控自动就绪状态显示，不影响转向控制。 */
#define LIGHTING_APP_TURN_SIGNAL_ENABLE      1U
/* 转向指示进入阈值，当前120/1000；允许1~1000，取飞控转向调度指令，负左正右。 */
#define LIGHTING_APP_TURN_ENTER_PERMILLE    120
/* 转向指示退出阈值，当前60/1000；允许0~ENTER-1，低于此值退出；精确回中始终退出，含阈值设0。 */
#define LIGHTING_APP_TURN_EXIT_PERMILLE      60
/* 转向琥珀光带渐亮/渐暗各自的时间，当前600 ms；允许100~5000 ms，完整呼吸周期1200 ms，不硬切亮灭。 */
#define LIGHTING_APP_TURN_HALF_PERIOD_MS    600U
/* 转向光带中央最低红色分量，当前6/255；允许1~PEAK，保持暗琥珀轮廓而非周期熄灭，仍受总亮度上限约束。 */
#define LIGHTING_APP_TURN_MIN_BRIGHTNESS      6U
/* 转向光带中央最高红色分量，当前28/255；允许MIN~40，绿色按红色3/8计算，蓝色为0，仍受总亮度上限约束。 */
#define LIGHTING_APP_TURN_PEAK_BRIGHTNESS    28U
/* 转向光带每端渐变羽化长度，当前3像素；允许1~WS2812_STRIP_MAX_LED_COUNT，短灯带自动重叠，状态端点不覆盖。 */
#define LIGHTING_APP_TURN_FEATHER_PIXELS      3U
/* 等待回中灯效的呼吸周期，当前1600 ms；允许200~60000 ms，仅用于琥珀等待状态。 */
#define LIGHTING_APP_WAIT_CYCLE_MS         1600U
/* WS2812逻辑0的定时器比较值，当前70个TIM1时钟计数；与168 MHz、ARR=209配套，影响电气脉宽而非亮度。 */
#define LIGHTING_APP_WS2812_ZERO_COMPARE    70U
/* WS2812逻辑1比较值，当前140个TIM1时钟计数；须>ZERO_COMPARE并匹配LED时序，不作普通亮度调节。 */
#define LIGHTING_APP_WS2812_ONE_COMPARE    140U
#endif
