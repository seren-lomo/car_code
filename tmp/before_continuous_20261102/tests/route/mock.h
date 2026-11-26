#ifndef ROUTE_HOST_MOCK_H
#define ROUTE_HOST_MOCK_H
#define __MAIN_H
#define __weak
#include <stdint.h>
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct { struct { uint32_t Mode; } Init; uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; uint32_t ErrorCode; } UART_HandleTypeDef;
#define DMA_CIRCULAR 1U
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#define GPIOA ((void *)1)
#define GPIOC ((void *)2)
#define GPIO_PIN_5 32U
#define GPIO_PIN_15 32768U
#define GPIO_PIN_RESET 0U
uint32_t HAL_GPIO_ReadPin(void *, uint32_t);
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
void __DMB(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
#endif
