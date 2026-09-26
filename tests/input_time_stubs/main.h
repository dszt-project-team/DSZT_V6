#ifndef INPUT_TIME_TEST_MAIN_H
#define INPUT_TIME_TEST_MAIN_H
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum {
  HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0, HAL_TIM_ACTIVE_CHANNEL_1,
  HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3, HAL_TIM_ACTIVE_CHANNEL_4
} HAL_TIM_ActiveChannel;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
typedef struct { HAL_TIM_ActiveChannel Channel; uint32_t capture; } TIM_HandleTypeDef;
typedef struct { GPIO_PinState level; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t ICPolarity, ICSelection, ICPrescaler, ICFilter; } TIM_IC_InitTypeDef;
extern GPIO_TypeDef test_gpio_ports[9];
#define GPIOA (&test_gpio_ports[0])
#define GPIOB (&test_gpio_ports[1])
#define GPIOC (&test_gpio_ports[2])
#define GPIOD (&test_gpio_ports[3])
#define GPIOE (&test_gpio_ports[4])
#define GPIOF (&test_gpio_ports[5])
#define GPIOG (&test_gpio_ports[6])
#define GPIOH (&test_gpio_ports[7])
#define GPIOI (&test_gpio_ports[8])
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOI_CLK_ENABLE() ((void)0)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define GPIO_MODE_AF_PP 0U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define TIM_INPUTCHANNELPOLARITY_BOTHEDGE 0U
#define TIM_ICSELECTION_DIRECTTI 0U
#define TIM_ICPSC_DIV1 0U
#define DMA_IT_HT 0U
#define UART_IT_IDLE 0U
#define __HAL_DMA_GET_COUNTER(handle) ((handle)->remaining)
#define __HAL_DMA_DISABLE_IT(handle, flag) ((void)(handle), (void)(flag))
#define __HAL_UART_ENABLE_IT(handle, flag) ((void)(handle), (void)(flag))
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *timer, TIM_IC_InitTypeDef *config, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
