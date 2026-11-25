#include "imu.h"
int _fltused;
void *memset(void *out, int c, unsigned long long n)
{ unsigned char *p=out; while(n--) *p++=(unsigned char)c; return out; }
void *memcpy(void *out, const void *in, unsigned long long n)
{ unsigned char *p=out; const unsigned char *q=in; while(n--) *p++=*q++; return out; }
UART_HandleTypeDef huart5;
static uint32_t tick, primask;
static uint8_t *destination;
static uint8_t fail_next;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1; }
void __set_PRIMASK(uint32_t p) { primask=p; }
void __DMB(void) {}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
    if (u!=&huart5 || n!=1) return HAL_ERROR;
    if (fail_next) { fail_next=0; return HAL_ERROR; }
    if (u->RxState!=HAL_UART_STATE_READY) return HAL_BUSY;
    destination=p;
    u->RxState=HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}
__declspec(dllexport) void test_reset(void)
{
    tick=primask=fail_next=0;
    huart5.RxState=HAL_UART_STATE_READY;
    huart5.ErrorCode=0;
    IMU_Init();
}
__declspec(dllexport) void test_tick(uint32_t t) { tick=t; }
__declspec(dllexport) void test_feed(const uint8_t *p, uint32_t n)
{
    while(n--) {
        if (huart5.RxState!=HAL_UART_STATE_BUSY_RX) return;
        *destination=*p++;
        huart5.RxState=HAL_UART_STATE_READY;
        IMU_UART_RxCpltCallback(&huart5);
    }
}
__declspec(dllexport) void test_error(uint32_t error, uint32_t end_rx)
{
    huart5.ErrorCode=error;
    if(end_rx) huart5.RxState=HAL_UART_STATE_READY;
    IMU_UART_ErrorCallback(&huart5);
}
__declspec(dllexport) void test_fail_rearm(void) { fail_next=1; }
__declspec(dllexport) void test_other_uart(void)
{
    UART_HandleTypeDef other={0};
    IMU_UART_RxCpltCallback(&other);
    IMU_UART_ErrorCallback(&other);
}
