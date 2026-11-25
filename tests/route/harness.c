#include "route.h"
#include "route_uart.h"
#include "chassis_heading.h"
#include "imu.h"
#include "route_keys.h"
#include "chassis_debug.h"
#include "chassis_path.h"
volatile ChassisPathStatus_t chassis_path;
uint8_t Chassis_PathBuild(void) { return 0; }
uint8_t Chassis_PathAppend(float f,float l,uint16_t rpm,uint16_t i)
{ (void)f;(void)l;(void)rpm;(void)i;return 0; }
ChassisResult_t Chassis_PathStart(void) { return CHASSIS_INVALID; }
#define EX __declspec(dllexport)
int _fltused;
void *memset(void *out, int c, unsigned long long n)
{ uint8_t *p=out; while(n--) *p++=(uint8_t)c; return out; }
void *memcpy(void *out, const void *in, unsigned long long n)
{ uint8_t *p=out; const uint8_t *q=in; while(n--) *p++=*q++; return out; }
volatile ChassisMotion_t chassis_motion;
volatile ChassisHeading_t chassis_heading;
volatile ChassisHeadingConfig_t chassis_heading_config;
volatile ChassisDebug_t chassis_debug;
volatile uint32_t key0_level, key0_press_count;
static uint8_t key_levels[2]={1,1};
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart;
static uint32_t tick, mask, stops, motions, rx_pos, tx_used, tx_fail, rx_fail, tx_hang;
static uint8_t *rx_dest, online;
static uint16_t rx_size;
static float raw_yaw, amount;
static uint8_t action;
static uint8_t tx_capture[65536];

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask=1; }
void __set_PRIMASK(uint32_t value) { mask=value; }
void __DMB(void) {}
uint32_t HAL_GPIO_ReadPin(void *port, uint32_t pin)
{ (void)pin; return key_levels[port==GPIOC?0:1]; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
    if(rx_fail) return HAL_ERROR;
    rx_dest=p; rx_size=n; rx_pos=0; u->hdmarx->remaining=n; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n)
{
    uint16_t i;
    if(tx_fail) return HAL_BUSY;
    for(i=0;i<n && tx_used<sizeof(tx_capture);i++) tx_capture[tx_used++]=p[i];
    if(!tx_hang) RouteUART_TxCpltCallback(u);
    return HAL_OK;
}
uint8_t IMU_GetData(IMU_Data_t *out)
{ IMU_Data_t v={0}; v.angle_deg[2]=raw_yaw; v.online=online; *out=v; return online; }
uint8_t Chassis_IsBusy(void) { return chassis_motion.state==CHASSIS_RUNNING; }
void Chassis_Stop(void) { stops++; chassis_motion.state=CHASSIS_STOPPED; }
void Chassis_Task(void) {}
static ChassisResult_t start(uint8_t a, float value, uint16_t rpm)
{
    ChassisHeading_t empty={0};
    if(Chassis_IsBusy()) return CHASSIS_BUSY;
    if(rpm==0 || rpm>120 || (a!=1 && (value>4000 || value< -4000))) return CHASSIS_INVALID;
    action=a; amount=value; motions++; chassis_heading=empty;
    chassis_motion.state=CHASSIS_RUNNING; chassis_motion.fault=0;
    return CHASSIS_OK;
}
ChassisResult_t Chassis_TurnRelative(float degrees, uint16_t rpm) { return start(1,degrees,rpm); }
ChassisResult_t Chassis_MoveForwardHold(float mm, uint16_t rpm) { return start(2,mm,rpm); }
ChassisResult_t Chassis_MoveLeftHold(float mm, uint16_t rpm) { return start(3,mm,rpm); }

EX void test_init(void)
{
    RouteUARTDebug_t d={0}; ChassisMotion_t m={0};
    route_uart_debug=d; chassis_motion=m;
    tick=mask=stops=motions=tx_used=tx_fail=rx_fail=tx_hang=0;
    online=1; raw_yaw=0; action=0; amount=0;
    chassis_heading_config.imu_yaw_sign=-1;
    chassis_heading_config.distance_tolerance_mm=3;
    chassis_heading_config.yaw_tolerance_deg=1;
    route_config.position_tolerance_mm=6; route_config.heading_tolerance_deg=1.5f;
    route_config.work_timeout_ms=0;
    dma.Init.Mode=DMA_CIRCULAR; uart.hdmarx=&dma;
    Route_Init(); route_work_heading_config.enabled=1; route_work_heading_config.turn_rpm=15; route_continuous_enabled=0; RouteUART_Init(&uart);
    key_levels[0]=key_levels[1]=1; key0_press_count=0;
    RouteKeys_Init();
}
EX void test_step(uint32_t dt)
{ tick+=dt; RouteUART_Task(); Route_Task(); RouteUART_Task(); }
EX void test_feed(const uint8_t *p, uint16_t n) { Route_ReceiveBytes(p,n); }
EX void test_dma(const uint8_t *p, uint16_t n)
{
    uint16_t i;
    for(i=0;i<n;i++) {
        rx_dest[rx_pos++]=p[i];
        if(rx_pos==rx_size) rx_pos=0;
        dma.remaining=rx_pos==0?rx_size:(rx_size-rx_pos);
        if(rx_pos==0 || rx_pos==rx_size/2) RouteUART_RxEventCallback(&uart,(uint16_t)rx_pos);
    }
    RouteUART_RxEventCallback(&uart,(uint16_t)rx_pos);
}
EX void test_idle_stale(uint16_t position) { RouteUART_RxEventCallback(&uart,position); }
EX void test_finish(float forward, float left, float turn)
{
    chassis_heading.forward_mm=forward; chassis_heading.left_mm=left;
    chassis_heading.turned_deg=turn; raw_yaw-=turn;
    while(raw_yaw>=180) raw_yaw-=360;
    while(raw_yaw< -180) raw_yaw+=360;
    chassis_motion.state=CHASSIS_DONE;
}
EX void test_control(uint32_t key, uint32_t value)
{
    switch(key) {
    case 0: online=(uint8_t)value; break;
    case 1: route_config.work_timeout_ms=value; break;
    case 2: Route_SetMotionInhibit((uint8_t)value); break;
    case 3: Chassis_Stop(); break;
    case 4: chassis_motion.state=CHASSIS_FAULT; chassis_motion.fault=value; chassis_motion.fault_address=3; break;
    case 5: tx_fail=value; break;
    case 6: tx_hang=value; break;
    case 8: route_work_heading_config.enabled=(uint8_t)value; break;
    case 9: route_work_heading_config.turn_rpm=(uint16_t)value; break;
    case 7: rx_fail=value; uart.ErrorCode=8; RouteUART_ErrorCallback(&uart); break;
    }
}
EX void test_raw_yaw(float value) { raw_yaw=value; }
EX uint32_t test_get(uint32_t key)
{
    switch(key) {
    case 0:return route_status.state; case 1:return route_status.point_index;
    case 2:return route_status.route_id; case 3:return motions;
    case 4:return action; case 5:return Chassis_IsBusy();
    case 6:return route_status.crc_errors; case 7:return route_status.rx_overflows;
    case 8:return route_status.partial_timeouts; case 9:return route_status.fault;
    case 10:return route_status.completed_points; case 11:return stops;
    case 12:return route_vision.count; case 13:return route_uart_debug.rx_events;
    case 14:return route_uart_debug.tx_errors; case 15:return route_uart_debug.uart_errors;
    case 16:return route_uart_debug.restart_errors; case 17:return route_status.chassis_fault;
    case 18:return route_status.rx_bytes;
    case 19:return route_start_area.request_id;
    case 20:return route_start_area.area;
    case 21:return route_start_area.acknowledged;
    case 22:return route_start_area.pending;
    case 23:return route_start_area.tx_count;
    case 24:return route_keys_debug.area_requests;
    case 25:return key0_press_count;
    case 26:return route_keys_debug.key1_press_count;
    case 27:return route_status.phase; case 28:return route_work_heading_active;
    } return 0xffffffffU;
}
EX float test_float(uint32_t key)
{
    switch(key) {
    case 0:return amount; case 1:return route_status.estimated_x_mm;
    case 2:return route_status.estimated_y_mm; case 3:return route_status.map_yaw_deg;
    } return 0;
}
EX uint32_t test_read(uint8_t *out)
{ uint32_t i,n=tx_used; for(i=0;i<n;i++) out[i]=tx_capture[i]; tx_used=0; return n; }
EX void test_key(uint32_t index, uint32_t value) { if(index<2) key_levels[index]=(uint8_t)value; }
EX void test_keys_init(void) { RouteKeys_Init(); }
EX void test_robot_step(uint32_t dt)
{ tick+=dt; RouteUART_Task(); RouteKeys_Task(); Route_Task(); RouteUART_Task(); }
EX uint32_t test_request_area(uint32_t area) { return Route_RequestStartArea((uint8_t)area); }
