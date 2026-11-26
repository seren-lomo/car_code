#include "route_keys.h"
#include "route.h"
#include "arm_config.h"
#include "chassis_debug.h"

#define DEBOUNCE_MS 30U//软件滤波
typedef struct { uint32_t changed; uint8_t raw, stable, consumed; } Key;
static Key keys[2];
static uint8_t chord;//组合键,拒绝启动任何运动
volatile RouteKeysDebug_t route_keys_debug;

static uint8_t level(uint8_t index)//index == 0 代表 Key0（引脚 PC5）。index == 1 代表 Key1（引脚 PA15）。
{
    return HAL_GPIO_ReadPin(index==0U?GPIOC:GPIOA,
        index==0U?GPIO_PIN_5:GPIO_PIN_15)==GPIO_PIN_RESET?0U:1U;
}
void RouteKeys_Init(void)
{
    uint8_t i;
    RouteKeysDebug_t empty={0};
    route_keys_debug=empty;
    chord=0U;
    for(i=0U;i<2U;i++) {
        keys[i].raw=keys[i].stable=level(i);
        keys[i].changed=HAL_GetTick();
        /* A key held during boot must be released before it can select. */
        keys[i].consumed=(uint8_t)(keys[i].raw==0U);//清除所有的调试计数器，复位按键状态。
    }
    key0_level=keys[0].raw;
    route_keys_debug.key1_level=keys[1].raw;
}
void RouteKeys_Task(void)
{
    uint8_t raw[2], i;
    uint32_t now=HAL_GetTick();
    raw[0]=level(0U); raw[1]=level(1U);
    key0_level=raw[0]; route_keys_debug.key1_level=raw[1];
    /* Preserve the route application's immediate KEY0 stop, then consume
     * this press so it cannot also trigger a scan after stopping. */
    if(raw[0]==0U && (Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy())) {
        Route_Stop(); keys[0].consumed=1U; chassis_debug.stop_request++;
        route_keys_debug.pending_area=route_keys_debug.pose_preparing=0U;
    }//急停 
    if(raw[1]==0U && (Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy())) keys[1].consumed=1U;
    if(raw[0]==0U && raw[1]==0U) chord=1U;// 两个一起按了，触发 chord
    if(chord && raw[0]!=0U && raw[1]!=0U && keys[0].stable!=0U && keys[1].stable!=0U) chord=0U;// 必须两个都完全松手，才解除 chord
    route_keys_debug.both_pressed=chord;
    Route_SetMotionInhibit((uint8_t)(raw[0]==0U || raw[1]==0U || chord));
    for(i=0U;i<2U;i++) {
        if(raw[i]!=keys[i].raw) { keys[i].raw=raw[i]; keys[i].changed=now; }// 状态变了，重新计时
        if(raw[i]==keys[i].stable || (uint32_t)(now-keys[i].changed)<DEBOUNCE_MS) continue;
        keys[i].stable=raw[i];// 持续 30ms 没变，认定为稳定电平
        if(raw[i]!=0U) { keys[i].consumed=0U; continue; }
        if(i==0U) key0_press_count++; else route_keys_debug.key1_press_count++;
        if(!keys[i].consumed && !chord && raw[1U-i]!=0U && !Route_IsActive() && !Chassis_IsBusy() && !Route_AuxIsBusy()) {
            /* The physical key confirms the manual initial pose. Wait for
             * release before auto-zero, so held KEY0 cannot cancel itself. */
            route_keys_debug.pending_area=i==0U?2U:1U;
            route_keys_debug.pose_preparing=0U;
            route_keys_debug.pose_result=ROUTE_OK;
        } else route_keys_debug.ignored_presses++;
        keys[i].consumed=1U;// 处理完毕，标记为已消费
    }
    if(chord) route_keys_debug.pending_area=route_keys_debug.pose_preparing=0U;
    if(route_keys_debug.pending_area && raw[0]!=0U && raw[1]!=0U &&
       keys[0].stable!=0U && keys[1].stable!=0U) {
        RouteResult_t result;
        if(!route_keys_debug.pose_preparing) {
            result=Route_PrepareStartPose();
            route_keys_debug.pose_result=result;
            if(result!=ROUTE_OK) {
                route_keys_debug.ignored_presses++;route_keys_debug.pending_area=0U;return;
            }
            route_keys_debug.pose_preparing=1U;
        }
        result=Route_CheckStart();
        route_keys_debug.pose_result=result;
        if(result==ROUTE_BUSY) return;
#if ARM_VISION_TEST_MODE
        if(result==ROUTE_OK) route_keys_debug.arm_test_ready_count++;
#else
        if(result==ROUTE_OK) result=Route_RequestStartArea((uint8_t)route_keys_debug.pending_area);
        if(result==ROUTE_OK) route_keys_debug.area_requests++;
#endif
        route_keys_debug.pose_result=result;
        if(result!=ROUTE_OK) route_keys_debug.ignored_presses++;
        route_keys_debug.pending_area=route_keys_debug.pose_preparing=0U;
    }

}
