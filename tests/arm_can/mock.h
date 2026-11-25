#ifndef ARM_CAN_MOCK_H
#define ARM_CAN_MOCK_H
#define __MAIN_H
#define __CAN_H__
#include <stdint.h>
#define __IO volatile
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t ErrorCode;} CAN_HandleTypeDef;
typedef struct {uint32_t unused;} UART_HandleTypeDef; /* Pi protocol declarations */
typedef struct {uint32_t unused;} TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
void mock_tim_set_compare(TIM_HandleTypeDef *,uint32_t,uint32_t);
#define __HAL_TIM_SET_COMPARE(timer,channel,value) mock_tim_set_compare(timer,channel,value)
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
extern CAN_HandleTypeDef hcan;
extern volatile HAL_StatusTypeDef can_last_tx_status;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
void __DMB(void);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
void can_SendCmd(volatile uint8_t *,uint8_t);
typedef enum {GPIO_PIN_RESET=0,GPIO_PIN_SET=1} GPIO_PinState;
#define GPIOA ((void *)1)
#define GPIOC ((void *)3)
#define GPIO_PIN_5 32U
#define GPIO_PIN_15 32768U
GPIO_PinState HAL_GPIO_ReadPin(void *,uint16_t);
#endif
