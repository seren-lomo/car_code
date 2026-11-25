#ifndef CHASSIS_HOST_MOCK_H
#define CHASSIS_HOST_MOCK_H
#define __MAIN_H
#define __CAN_H__
#define __EMM_V5_H
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef enum {false, true} bool;
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t Pin, Mode, Pull;} GPIO_InitTypeDef;
typedef struct {
    uint32_t ack_count, flags_count, flags_tick, speed_tick;
    uint8_t ack_function, ack_status, flags;
    int16_t speed_rpm;
} MotorFeedback_t;
extern volatile MotorFeedback_t motor_feedback[5];
extern volatile HAL_StatusTypeDef can_last_tx_status;
#define GPIOC ((void *)0)
#define GPIO_PIN_5 32
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP 1
#define GPIO_PIN_RESET 0
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
enum {S_VEL = 14, S_FLAG = 19};
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *init);
uint32_t HAL_GPIO_ReadPin(void *port, uint32_t pin);
void Emm_V5_Stop_Now(uint8_t addr, bool sync);
void Emm_V5_En_Control(uint8_t addr, bool enabled, bool sync);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, bool sync);
void Emm_V5_Read_Sys_Params(uint8_t addr, int param);
#endif
