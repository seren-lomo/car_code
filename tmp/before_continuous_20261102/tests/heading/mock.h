#ifndef HEADING_HOST_MOCK_H
#define HEADING_HOST_MOCK_H
#define __MAIN_H
#define __CAN_H__
#define __EMM_V5_H
#define __USART_H__
#include <stdint.h>
#include <stdbool.h>
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct { uint32_t RxState, ErrorCode; } UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
extern UART_HandleTypeDef huart5;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
void __DMB(void);
typedef enum {S_VEL = 14, S_CPOS = 15, S_FLAG = 19} SysParams_t;
typedef struct {
    uint32_t ack_count, flags_count, flags_tick, speed_tick;
    uint32_t speed_count, position_count, position_tick;
    uint8_t ack_function, ack_status, flags;
    int16_t speed_rpm;
    int32_t position;
} MotorFeedback_t;
extern volatile MotorFeedback_t motor_feedback[5];
extern volatile HAL_StatusTypeDef can_last_tx_status;
#define GPIOC ((void *)0)
#define GPIO_PIN_5 32
#define GPIO_PIN_RESET 0
uint32_t HAL_GPIO_ReadPin(void *, uint32_t);
void Emm_V5_Stop_Now(uint8_t, bool);
void Emm_V5_En_Control(uint8_t, bool, bool);
void Emm_V5_Pos_Control(uint8_t, uint8_t, uint16_t, uint8_t, uint32_t, uint8_t, bool);
void Emm_V5_Vel_Control(uint8_t, uint8_t, uint16_t, uint8_t, bool);
void Emm_V5_Synchronous_motion(uint8_t);
void Emm_V5_Read_Sys_Params(uint8_t, SysParams_t);
#endif
