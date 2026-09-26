#ifndef TEST_STEER_H
#define TEST_STEER_H
#include <stdint.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_2 2
#define TIM8_CC_IRQn 8
#define TIM4_IRQn 4
#define GPIO_AF1_TIM2 1
#define GPIO_AF2_TIM5 2
#define STEER_LEFT_DIR_GPIO_Port 1
#define STEER_LEFT_DIR_Pin 7
#define STEER_LEFT_BRAKE_GPIO_Port 1
#define STEER_LEFT_BRAKE_Pin 2
#define STEER_RIGHT_DIR_GPIO_Port 2
#define STEER_RIGHT_DIR_Pin 14
#define STEER_RIGHT_BRAKE_GPIO_Port 2
#define STEER_RIGHT_BRAKE_Pin 15
#define MT6826S_LEFT_PWM_GPIO_Port 3
#define MT6826S_LEFT_PWM_Pin 0
#define MT6826S_RIGHT_PWM_GPIO_Port 4
#define MT6826S_RIGHT_PWM_Pin 11
extern int htim2, htim4, htim5, htim8;
typedef struct { int16_t duty_permille; uint8_t brake_on, reverse; int *timer; } SteerMotor_Handle_t;
typedef struct { uint8_t healthy, zero_valid; uint16_t raw_angle; int32_t relative_deg_x10; } Mt6826sPwm_Snapshot_t;
typedef struct { Mt6826sPwm_Snapshot_t sample; int *timer; } Mt6826sPwm_Handle_t;
enum { STEER_ANGLE_CTRL_DISABLED, STEER_ANGLE_CTRL_HOLD, STEER_ANGLE_CTRL_FAULT };
typedef struct { int state; float target_angle_deg; SteerMotor_Handle_t *motor; } SteerAngleCtrl_Handle_t;
void Error_Handler(void);
HAL_StatusTypeDef SteerMotor_Init(SteerMotor_Handle_t *, int *, int,int,int,int,int,int,int);
HAL_StatusTypeDef Mt6826sPwm_Init(Mt6826sPwm_Handle_t *, int *, int,int,int,int);
void Mt6826sPwm_SetDirectionInverted(Mt6826sPwm_Handle_t *, int);
void Mt6826sPwm_SetZeroRaw(Mt6826sPwm_Handle_t *, int);
void Mt6826sPwm_Task(Mt6826sPwm_Handle_t *, uint32_t);
void Mt6826sPwm_GetSnapshot(Mt6826sPwm_Handle_t *,uint32_t,Mt6826sPwm_Snapshot_t *);
void SteerAngleCtrl_Init(SteerAngleCtrl_Handle_t *,SteerMotor_Handle_t *,Mt6826sPwm_Handle_t *);
void SteerAngleCtrl_Configure(SteerAngleCtrl_Handle_t *,float,float,float,float,float,float,float,int,int,int,int,int);
void SteerAngleCtrl_Disable(SteerAngleCtrl_Handle_t *);
HAL_StatusTypeDef SteerAngleCtrl_SetTargetDeg(SteerAngleCtrl_Handle_t *,float);
HAL_StatusTypeDef SteerAngleCtrl_Enable(SteerAngleCtrl_Handle_t *);
void SteerAngleCtrl_Task(SteerAngleCtrl_Handle_t *,uint32_t);
void SteerMotor_Stop(SteerMotor_Handle_t *);
void SteerMotor_SetDirection(SteerMotor_Handle_t *,int);
void SteerMotor_SetDuty(SteerMotor_Handle_t *,int16_t);
void SteerMotor_SetBrake(SteerMotor_Handle_t *,int);
#endif
