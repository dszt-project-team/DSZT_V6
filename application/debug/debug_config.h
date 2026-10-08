/* DSZT_V6_FC debug 配置：修改宏后需重新编译下载；中文注释说明当前作用及调整约束。
 * 0/1开关按各项适用范围使用，不代表绕过其他安全门。
 */
#ifndef DEBUG_CONFIG_H
/* 头文件重复包含保护，不是诊断开关，请勿修改。 */
#define DEBUG_CONFIG_H

/* 综合诊断固定编号0，行前缀V6_FC，对应DBG GENERIC；用于默认模式选择，不建议改编号。 */
#define DEBUG_APP_OUTPUT_GENERIC    0U
/* 综合诊断的兼容别名，等同GENERIC；不是独立视图，也没有DBG AGGREGATE命令。 */
#define DEBUG_APP_OUTPUT_AGGREGATE  DEBUG_APP_OUTPUT_GENERIC
/* OID诊断固定编号1，对应DBG OID；包含目标、反馈、心跳、回读及故障计数。 */
#define DEBUG_APP_OUTPUT_OID        1U
/* 编码器/转向诊断固定编号5，对应DBG MT6826S；轴角字段单位为0.1°。 */
#define DEBUG_APP_OUTPUT_MT6826S    5U
/* 飞控输入诊断固定编号7，对应DBG FC；显示MAIN1/2原始、滤波、在线和故障码。 */
#define DEBUG_APP_OUTPUT_FC         7U
/* 关闭周期诊断的固定编号255，对应DBG OFF；若命令开关开启，仍可接收查询/切换命令。 */
#define DEBUG_APP_OUTPUT_OFF      255U
/* 上电默认诊断视图，当前FC；可选上述GENERIC/OID/MT6826S/FC/OFF，运行期DBG切换不持久保存。 */
#define DEBUG_APP_OUTPUT_MODE      DEBUG_APP_OUTPUT_FC
/* OID周期输出间隔，当前100 ms；缩短增加串口/格式化负载，不改变OID控制周期，发送忙时跳过输出。 */
#define DEBUG_APP_OID_PRINT_PERIOD_MS 100U
/* UART7诊断总开关：当前1开启，0不初始化业务收发且任务直接返回；不关闭CubeMX基础UART初始化。 */
#define DEBUG_APP_ENABLE             1U
/* 只读DBG命令开关：当前1启用接收解析，0只输出默认视图；任何视图命令都不控制执行器或写参数。 */
#define DEBUG_APP_COMMAND_ENABLE     1U
/* 综合/编码器/飞控视图周期，当前200 ms；增大减少日志带宽，不改变控制调度。 */
#define DEBUG_APP_TEXT_PRINT_PERIOD_MS 200U
/* 单条ASCII命令缓存容量，当前32字节，含结尾零字符，最长31字符；超长行丢弃并报错，调大需考虑RAM。 */
#define DEBUG_APP_COMMAND_LINE_SIZE   32U
/* 每轮任务最多解析的接收字符数，当前32；增大可加快长命令处理但增加单轮工作量，应保持正数。 */
#define DEBUG_APP_COMMAND_POLL_BUDGET 32U
/* UART7中断接收环形缓存容量，当前128字节，可用127；调大增加抗突发能力和RAM占用，溢出会丢弃当前命令行。 */
#define DEBUG_APP_RX_BUFFER_SIZE     128U
#if DEBUG_APP_OUTPUT_MODE != DEBUG_APP_OUTPUT_GENERIC && DEBUG_APP_OUTPUT_MODE != DEBUG_APP_OUTPUT_OID && DEBUG_APP_OUTPUT_MODE != DEBUG_APP_OUTPUT_MT6826S && DEBUG_APP_OUTPUT_MODE != DEBUG_APP_OUTPUT_FC && DEBUG_APP_OUTPUT_MODE != DEBUG_APP_OUTPUT_OFF
#error "Unsupported V6_FC UART7 diagnostic mode"
#endif
#endif
