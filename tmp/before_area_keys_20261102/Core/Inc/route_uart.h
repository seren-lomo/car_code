#ifndef CAR_ROUTE_UART_H
#define CAR_ROUTE_UART_H
#include "main.h"

/* Selected UART needs circular byte RX DMA and its UART/DMA interrupts.
 * nullptr leaves the transport disconnected, while Route_* remains usable.
 * UART4: PC10 TX / PC11 RX, 115200 8N1 in the current CubeMX project.
 */
HAL_StatusTypeDef RouteUART_Init(UART_HandleTypeDef *uart);
void RouteUART_Task(void);  /* CommTask: retry RX and send queued replies. */
void RouteUART_RxEventCallback(UART_HandleTypeDef *uart, uint16_t position);
void RouteUART_ErrorCallback(UART_HandleTypeDef *uart);
void RouteUART_TxCpltCallback(UART_HandleTypeDef *uart);
uint8_t RouteUART_Send(const uint8_t *frame, uint16_t count); /* RobotTask only */

typedef struct {
    uint32_t rx_events, uart_errors, last_error, restart_errors;
    uint32_t tx_frames, tx_errors, tx_overflows;
    HAL_StatusTypeDef init_status;
    uint8_t enabled;
} RouteUARTDebug_t;
extern volatile RouteUARTDebug_t route_uart_debug;
#endif
