/* DSZT_V6 lighting 配置：修改宏后需重新编译下载；中文注释说明当前作用及调整约束。
 * 0/1开关按各项适用范围使用，不代表绕过其他安全门。
 */
#ifndef LIGHTING_CONFIG_H
/* 头文件重复包含保护，不是灯带开关，请勿修改。 */
#define LIGHTING_CONFIG_H

/* 双WS2812应用开关：当前1启用，0不初始化灯带业务且不刷新；不保证关闭前灯带已有颜色自动熄灭。 */
#define LIGHTING_APP_ENABLE                  1U
/* 每条灯带LED数量，当前21；两路相同，须1～WS2812_STRIP_MAX_LED_COUNT，增大会增加传输时间。 */
#define LIGHTING_APP_LED_COUNT              21U
/* 灯带应用刷新周期，当前100 ms；减小刷新更密但增加DMA/任务负载，不是WS2812位时序。 */
#define LIGHTING_APP_FRAME_PERIOD_MS       100U
/* 左灯带像素顺序：当前0正序，1镜像；当前整条同色，改变此值通常无可见差异，不改变车头定义。 */
#define LIGHTING_APP_LEFT_REVERSED           0U
/* 右灯带像素顺序：当前1镜像，0正序；只翻转像素索引，不交换左右灯带接口。 */
#define LIGHTING_APP_RIGHT_REVERSED          1U
/* WS2812逻辑0的定时器比较值，当前70个TIM1时钟计数；与168 MHz、ARR=209配套，影响电气脉宽而非亮度。 */
#define LIGHTING_APP_WS2812_ZERO_COMPARE    70U
/* WS2812逻辑1比较值，当前140个TIM1时钟计数；须>ZERO_COMPARE并匹配LED时序，不作普通亮度调节。 */
#define LIGHTING_APP_WS2812_ONE_COMPARE    140U
#endif
