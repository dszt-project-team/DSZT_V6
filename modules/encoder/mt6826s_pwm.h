/**
  ******************************************************************************
  * @file    mt6826s_pwm.h
  * @brief   MT6826S OUT/PWM 绝对角度编码器模块接口。
  ******************************************************************************
  */

#ifndef MT6826S_PWM_H
#define MT6826S_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* MT6826S PWM 一帧为 4119 个最小时钟：16 个固定高电平 + 4095 个角度数据 + 8 个固定低电平。 */
#define MT6826S_PWM_FRAME_TICKS             4119U
#define MT6826S_PWM_START_HIGH_TICKS        16U
#define MT6826S_PWM_DATA_TICKS              4095U
#define MT6826S_PWM_FULL_SCALE_COUNT        4096U

/*
 * MT6826S 默认 PWM 为 994Hz，也可配置为 497Hz，规格给出 +/-8% 频率偏差。
 * 这里按 0.8ms~2.4ms 放宽窗口，既覆盖两档频率，也能滤掉接线悬空和异常脉冲。
 */
#define MT6826S_PWM_PERIOD_MIN_US           800U
#define MT6826S_PWM_PERIOD_MAX_US           2400U
#define MT6826S_PWM_HEALTH_TIMEOUT_MS       100U

/*
 * PWM 占空比在 4095 -> 0 环回时会从接近 100% 跳到接近 0%。当前单通道双边沿
 * 捕获通过 ISR 内读取 GPIO 电平区分边沿，极短高/低脉冲可能在 ISR 进入前已经结束，
 * 从而偶发形成“周期合法但角度突跳”的伪帧。5840 输出轴额定 41rpm，约等于
 * 2.8 count/ms；这里给到 8 count/ms 并叠加 32 count 固定余量，允许远高于实物
 * 极限的真实运动，同时拒绝会使位置环误换向的孤立大跳变。
 */
#define MT6826S_PWM_CONTINUITY_MARGIN_COUNT       32U
#define MT6826S_PWM_CONTINUITY_MAX_COUNT_PER_MS    8U

typedef struct
{
  uint8_t online;              /* 最近 MT6826S_PWM_HEALTH_TIMEOUT_MS 内有有效 PWM 帧。 */
  uint8_t healthy;             /* online 且最近有效帧未超时。 */
  uint16_t raw_angle;          /* 解码后的 12 位绝对角度，0~4095。 */
  int32_t raw_deg_x10;         /* 原始角度 * 10，单位 0.1 deg。 */
  int32_t relative_deg_x10;    /* 相对机械零点角度 * 10，单位 0.1 deg；未标零时为 0。 */
  float relative_angle_deg;    /* 相对机械零点角度，单位 deg；供闭环直接使用。 */
  uint8_t zero_valid;          /* 已写入机械中位零点。 */
  uint8_t direction_inverted;  /* 相对角软件方向取反。 */
  uint16_t zero_raw_angle;     /* 机械中位对应的 12 位原始角度。 */
  uint16_t duty_permille;      /* OUT 高电平占空比，千分比，仅用于诊断。 */
  uint8_t pin_level;           /* 当前 OUT 输入电平，仅用于接线/输出模式诊断。 */
  uint32_t high_ticks_us;      /* 最近一帧高电平宽度，当前 TIM2/TIM5 均按 1MHz 配置，单位等于 us。 */
  uint32_t period_ticks_us;    /* 最近一帧周期，当前 TIM2/TIM5 均按 1MHz 配置，单位等于 us。 */
  uint32_t sample_count;       /* 成功解码的 PWM 帧数量。 */
  uint32_t edge_count;         /* 捕获到的 OUT 边沿数量。 */
  uint32_t invalid_count;      /* 周期/高电平异常或编码越界的帧数量。 */
  uint32_t continuity_reject_count; /* 因角度连续性不可能而拒绝的伪帧数量。 */
  uint32_t timeout_count;      /* 从在线变为超时离线的次数。 */
  uint32_t last_update_ms;     /* 最近有效帧的 HAL tick。 */
  uint32_t last_edge_ms;       /* 最近边沿的 HAL tick。 */
} Mt6826sPwm_Snapshot_t;

typedef struct
{
  /* 当前接线约定：右路 J26-6/PH11/TIM5_CH2，左路 J31-1/PA0/TIM2_CH1。 */
  TIM_HandleTypeDef *htim;         /* 用作微秒时间戳的 32 位定时器；右路 TIM5，左路 TIM2。 */
  uint32_t channel;                /* 输入捕获通道；右路 J26-6/PH11/TIM5_CH2，左路 J31-1/PA0/TIM2_CH1。 */
  HAL_TIM_ActiveChannel active_channel; /* HAL 回调中的活动通道枚举。 */
  GPIO_TypeDef *gpio_port;         /* MT6826S OUT 接入 GPIO，由右左路初始化时分别传入。 */
  uint16_t gpio_pin;               /* MT6826S OUT 接入引脚，由右左路初始化时分别传入。 */
  volatile uint32_t rise_tick_us;  /* 最近一次上升沿定时器计数。 */
  volatile uint32_t fall_tick_us;  /* 最近一次下降沿定时器计数。 */
  volatile uint32_t high_ticks_us; /* 最近一帧高电平宽度。 */
  volatile uint32_t period_ticks_us; /* 最近一帧周期。 */
  volatile uint32_t last_edge_ms;  /* 最近边沿 tick。 */
  volatile uint32_t last_update_ms; /* 最近有效帧 tick。 */
  volatile uint32_t sample_count;  /* 成功解码帧计数。 */
  volatile uint32_t edge_count;    /* 边沿计数。 */
  volatile uint32_t invalid_count; /* 异常帧计数。 */
  volatile uint32_t continuity_reject_count; /* 角度连续性过滤拒绝计数。 */
  volatile uint32_t timeout_count; /* 超时离线计数。 */
  volatile uint16_t raw_angle;     /* 解码后的 12 位原始角度。 */
  volatile uint16_t zero_raw_angle; /* 机械中位对应的 12 位原始角度。 */
  volatile int32_t relative_deg_x10; /* 相对机械中位角度 * 10，单位 0.1 deg。 */
  volatile uint16_t duty_permille; /* 高电平占空比千分比。 */
  volatile uint8_t online;         /* 最近收到有效 PWM 帧。 */
  volatile uint8_t zero_valid;     /* 已写入机械中位零点。 */
  volatile uint8_t direction_inverted; /* 相对角软件方向取反。 */
  volatile uint8_t have_rise;      /* 已捕获上升沿，等待下降沿/下一上升沿。 */
  volatile uint8_t have_fall;      /* 已捕获下降沿，等待下一上升沿计算周期。 */
} Mt6826sPwm_Handle_t;

HAL_StatusTypeDef Mt6826sPwm_Init(Mt6826sPwm_Handle_t *sensor,
                                  TIM_HandleTypeDef *htim,
                                  uint32_t channel,
                                  GPIO_TypeDef *gpio_port,
                                  uint16_t gpio_pin,
                                  uint32_t gpio_alternate);
void Mt6826sPwm_Task(Mt6826sPwm_Handle_t *sensor, uint32_t now_ms);
void Mt6826sPwm_IcCaptureCallback(Mt6826sPwm_Handle_t *sensor,
                                  TIM_HandleTypeDef *htim);
uint8_t Mt6826sPwm_IsHealthy(const Mt6826sPwm_Handle_t *sensor,
                             uint32_t now_ms);
void Mt6826sPwm_SetDirectionInverted(Mt6826sPwm_Handle_t *sensor,
                                     uint8_t inverted);
void Mt6826sPwm_SetZeroRaw(Mt6826sPwm_Handle_t *sensor,
                           uint16_t raw_angle);
void Mt6826sPwm_SetZeroAtCurrent(Mt6826sPwm_Handle_t *sensor);
uint8_t Mt6826sPwm_HasZero(const Mt6826sPwm_Handle_t *sensor);
void Mt6826sPwm_GetSnapshot(const Mt6826sPwm_Handle_t *sensor,
                            uint32_t now_ms,
                            Mt6826sPwm_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* MT6826S_PWM_H */
