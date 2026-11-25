#include "route_uart.h"
#include "route.h"

#define RX_SIZE 256U
#define TX_SLOTS 16U
#define TX_BYTES 64U
#define TX_TIMEOUT_MS 100U
volatile RouteUARTDebug_t route_uart_debug;
static UART_HandleTypeDef *port;
static uint8_t rx_dma[RX_SIZE], tx_data[TX_SLOTS][TX_BYTES];
static uint16_t rx_position, tx_length[TX_SLOTS];
static volatile uint8_t tx_head, tx_tail, tx_done, rx_restart;
static uint8_t tx_active;
static uint32_t tx_started, retry_tick;

static HAL_StatusTypeDef start_rx(void)
{
    HAL_StatusTypeDef status;
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    rx_position=0U;
    status=HAL_UARTEx_ReceiveToIdle_DMA(port,rx_dma,RX_SIZE);
    __set_PRIMASK(key);
    return status;
}
HAL_StatusTypeDef RouteUART_Init(UART_HandleTypeDef *uart)
{
    port=uart;
    tx_head=tx_tail=tx_done=tx_active=rx_restart=0U;
    route_uart_debug.enabled=0U;
    if(port==0) return HAL_OK;
    if(port->hdmarx==0 || port->hdmarx->Init.Mode!=DMA_CIRCULAR) {
        route_uart_debug.init_status=HAL_ERROR; return HAL_ERROR;
    }
    route_uart_debug.enabled=1U;
    route_uart_debug.init_status=start_rx();
    if(route_uart_debug.init_status!=HAL_OK) rx_restart=1U;
    return route_uart_debug.init_status;
}
void RouteUART_RxEventCallback(UART_HandleTypeDef *uart, uint16_t position)
{
    uint16_t current;
    uint32_t key;
    (void)position;
    if(uart!=port || !route_uart_debug.enabled) return;
    key=__get_PRIMASK(); __disable_irq();
    if(!rx_restart) {
        /* Read the current DMA cursor. A pending lower-priority IDLE callback
         * can carry an older Size than a DMA callback that already ran. */
        current=(uint16_t)(RX_SIZE-__HAL_DMA_GET_COUNTER(port->hdmarx));
        if(current==RX_SIZE) current=0U;
        if(current<rx_position) {
            Route_ReceiveBytes(rx_dma+rx_position,(uint16_t)(RX_SIZE-rx_position));
            rx_position=0U;
        }
        if(current>rx_position) Route_ReceiveBytes(rx_dma+rx_position,(uint16_t)(current-rx_position));
        rx_position=current;
        route_uart_debug.rx_events++;
    }
    __set_PRIMASK(key);
}
void RouteUART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(uart!=port || !route_uart_debug.enabled) return;
    route_uart_debug.last_error=uart->ErrorCode;
    route_uart_debug.uart_errors++;
    rx_restart=1U;
    Route_ReceiveFault();
}
void RouteUART_TxCpltCallback(UART_HandleTypeDef *uart)
{ if(uart==port && route_uart_debug.enabled) tx_done=1U; }

uint8_t RouteUART_Send(const uint8_t *frame, uint16_t count)
{
    uint8_t next;
    uint16_t i;
    if(!route_uart_debug.enabled || count==0U || count>TX_BYTES) return 0U;
    next=(uint8_t)((tx_head+1U)%TX_SLOTS);
    if(next==tx_tail) { route_uart_debug.tx_overflows++; return 0U; }
    for(i=0U;i<count;i++) tx_data[tx_head][i]=frame[i];
    tx_length[tx_head]=count;
    __DMB(); tx_head=next;
    return 1U;
}
void RouteUART_Task(void)
{
    uint32_t now=HAL_GetTick();
    if(!route_uart_debug.enabled) return;
    if(rx_restart && (uint32_t)(now-retry_tick)>=10U) 
		{
        retry_tick=now;
        HAL_UART_AbortReceive(port);
        /* Discard any interrupted frame before accepting a fresh DMA stream. */
        Route_ReceiveFault();
        rx_restart=0U;
        if(start_rx()!=HAL_OK) { rx_restart=1U; route_uart_debug.restart_errors++; }
    }
    if(tx_active) {
        if(tx_done) {
            tx_active=tx_done=0U;
            tx_tail=(uint8_t)((tx_tail+1U)%TX_SLOTS);
            route_uart_debug.tx_frames++;
        } else if((uint32_t)(now-tx_started)>=TX_TIMEOUT_MS) {
            HAL_UART_AbortTransmit(port);
            tx_active=tx_done=0U;
            tx_tail=(uint8_t)((tx_tail+1U)%TX_SLOTS);
            route_uart_debug.tx_errors++;
        }
    }
    if(!tx_active && tx_tail!=tx_head) {
        tx_done=0U;
        tx_started=now;
        tx_active=1U;
        if(HAL_UART_Transmit_IT(port,tx_data[tx_tail],tx_length[tx_tail])!=HAL_OK) {
            /* A busy/broken transmitter cannot block reception or RobotTask.
             * Drop this reply; the peer may retry or query persistent state. */
            tx_active=0U;
            tx_tail=(uint8_t)((tx_tail+1U)%TX_SLOTS);
            route_uart_debug.tx_errors++;
        }
    }
}
