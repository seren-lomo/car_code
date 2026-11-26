#include "chassis_debug.h"

void *memset(void *out, int c, unsigned long long n)
{
    unsigned char *p = out;
    while (n--) *p++ = (unsigned char)c;
    return out;
}
void *memcpy(void *out, const void *in, unsigned long long n)
{
    unsigned char *p = out;
    const unsigned char *q = in;
    while (n--) *p++ = *q++;
    return out;
}

volatile MotorFeedback_t motor_feedback[5];
volatile HAL_StatusTypeDef can_last_tx_status;
static uint32_t tick, key_level = 1, stop_count;
static uint8_t missing_addr, reject_addr;
static int16_t applied[5];
static uint32_t nonzero_count;

uint32_t HAL_GetTick(void) { return tick++; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *i) { (void)p; (void)i; }
uint32_t HAL_GPIO_ReadPin(void *p, uint32_t pin)
{ (void)p; (void)pin; return key_level; }
static void ack(uint8_t addr, uint8_t func)
{
    can_last_tx_status = HAL_OK;
    if (addr == missing_addr) return;
    motor_feedback[addr].ack_function = func;
    motor_feedback[addr].ack_status = addr == reject_addr ? 0xE2 : 0x02;
    ++motor_feedback[addr].ack_count;
}
void Emm_V5_Stop_Now(uint8_t a, bool s)
{ (void)s; ++stop_count; applied[a] = 0; ack(a, 0xFE); }
void Emm_V5_En_Control(uint8_t a, bool e, bool s)
{ (void)s; motor_feedback[a].flags = e ? 1 : 0; ack(a, 0xF3); }
void Emm_V5_Vel_Control(uint8_t a, uint8_t d, uint16_t v, uint8_t acc, bool s)
{
    (void)acc; (void)s;
    applied[a] = d ? -(int16_t)v : (int16_t)v;
    if (v) ++nonzero_count;
    ack(a, 0xF6);
}
void Emm_V5_Read_Sys_Params(uint8_t a, int param)
{
    can_last_tx_status = HAL_OK;
    if (a == missing_addr) return;
    if (param == S_FLAG) {
        motor_feedback[a].flags_tick = tick;
        ++motor_feedback[a].flags_count;
    }
}

__declspec(dllexport) int run_case(int test)
{
    uint32_t i, before;
    Chassis_DebugInit();
    if (test == 1) { /* No motion at boot; debounce; one-shot; timeout. */
        if (nonzero_count || chassis_debug.state) return 11;
        key_level = 0;
        for (i = 0; i < 10; ++i) Chassis_DebugTask();
        key_level = 1;
        for (i = 0; i < 40; ++i) Chassis_DebugTask();
        if (nonzero_count || key0_press_count) return 12;
        key_level = 0;
        for (i = 0; i < 40; ++i) Chassis_DebugTask();
        key_level = 1;
        for (i = 0; i < 40; ++i) Chassis_DebugTask();
        if (key0_press_count != 1 || key0_level != 1 || !chassis_task_count)
            return 16;
        if (chassis_debug.state != 1 || applied[1] != 20 ||
            applied[2] || applied[3] || applied[4]) return 13;
        tick += 1001;
        Chassis_DebugTask();
        if (chassis_debug.state || applied[1]) return 14;
        before = nonzero_count;
        for (i = 0; i < 1100; ++i) Chassis_DebugTask();
        if (before != nonzero_count) return 15;
    } else if (test == 2) { /* Single-wheel order 1,2,4,3 and mounting signs. */
        uint8_t order[4] = {1, 2, 4, 3};
        int16_t expected[4] = {20, -20, 20, -20};
        for (i = 0; i < 4; ++i) {
            chassis_debug.command = i + 1;
            ++chassis_debug.trigger;
            Chassis_DebugTask();
            if (chassis_debug.state != 1 || applied[order[i]] != expected[i])
                return 21 + i;
            chassis_debug.command = 0;
            Chassis_DebugTask();
        }
    } else if (test == 3) { /* Unconfirmed geometry rejects whole chassis. */
        chassis_debug.command = 5;
        ++chassis_debug.trigger;
        Chassis_DebugTask();
        if (chassis_debug.fault != 1 || nonzero_count) return 31;
        chassis_debug.next_test = 5;
        key_level = 0;
        for (i = 0; i < 2100; ++i) Chassis_DebugTask();
        if (!chassis_debug.mapping_confirmed || nonzero_count) return 32;
        key_level = 1;
        for (i = 0; i < 40; ++i) Chassis_DebugTask();
        if (nonzero_count) return 33;
    } else if (test >= 4 && test <= 9) { /* Forward/back/left/right/CCW/CW. */
        int16_t expected[6][4] = {
            {20, -20, 20, -20}, {-20, 20, -20, 20},
            {-20, -20, 20, 20}, {20, 20, -20, -20},
            {-20, -20, -20, -20}, {20, 20, 20, 20}
        };
        uint8_t order[4] = {1, 2, 4, 3};
        chassis_debug.mapping_confirmed = 1;
        chassis_debug.command = test + 1;
        ++chassis_debug.trigger;
        Chassis_DebugTask();
        if (chassis_debug.state != 1) return 41;
        for (i = 0; i < 4; ++i)
            if (applied[order[i]] != expected[test - 4][i]) return 42 + i;
        key_level = 0;
        for (i = 0; i < 40; ++i) Chassis_DebugTask();
        if (chassis_debug.state || applied[1] || applied[2] || applied[3] ||
            applied[4]) return 46;
        before = nonzero_count;
        key_level = 1;
        for (i = 0; i < 40; ++i) Chassis_DebugTask();
        if (nonzero_count != before) return 47;
    } else if (test == 10 || test == 11) { /* Missing/rejected motor ACK. */
        if (test == 10) missing_addr = 4;
        else reject_addr = 4;
        chassis_debug.command = 1;
        ++chassis_debug.trigger;
        Chassis_DebugTask();
        if (chassis_debug.state != 2 || chassis_debug.fault != 3 ||
            chassis_debug.fault_address != 4 || nonzero_count) return 101;
    } else if (test == 12) { /* State timeout stops every wheel. */
        chassis_debug.duration_ms = 3000;
        chassis_debug.command = 1;
        ++chassis_debug.trigger;
        Chassis_DebugTask();
        missing_addr = 1;
        for (i = 0; i < 900; ++i) Chassis_DebugTask();
        if (chassis_debug.state != 2 || chassis_debug.fault != 5 || applied[1])
            return 121;
    } else if (test == 13) { /* Duplicate addresses rejected before motion. */
        chassis_debug.address[3] = 1;
        chassis_debug.command = 1;
        ++chassis_debug.trigger;
        Chassis_DebugTask();
        if (chassis_debug.fault != 1 || nonzero_count) return 131;
    } else if (test == 14) { /* Motor reports clog protection while running. */
        chassis_debug.command = 1;
        ++chassis_debug.trigger;
        Chassis_DebugTask();
        motor_feedback[2].flags = 9;
        Chassis_DebugTask();
        if (chassis_debug.fault != 4 || chassis_debug.state != 2 || applied[1])
            return 141;
    } else return 999;
    return 0;
}
