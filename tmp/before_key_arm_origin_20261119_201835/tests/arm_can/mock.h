#ifndef ARM_CAN_MOCK_H
#define ARM_CAN_MOCK_H
#define __MAIN_H
#define __CAN_H__
#include <stdint.h>
#define __IO volatile
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t ErrorCode;} CAN_HandleTypeDef;
typedef struct {uint32_t unused;} UART_HandleTypeDef; /* Pi protocol declarations */
extern CAN_HandleTypeDef hcan;
extern volatile HAL_StatusTypeDef can_last_tx_status;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
void __DMB(void);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
void can_SendCmd(volatile uint8_t *,uint8_t);
#endif
