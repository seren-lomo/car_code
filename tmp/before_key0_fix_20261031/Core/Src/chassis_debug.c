#include "chassis_debug.h"
#include "can.h"
#include "Emm_V5.h"

#define WHEEL_COUNT 4U
#define TEST_RPM_LIMIT 60U
#define TEST_TIME_LIMIT_MS 3000U
#define ACK_TIMEOUT_MS 100U
#define FEEDBACK_TIMEOUT_MS 800U
#define EMM_ACC 100U

volatile ChassisDebug_t chassis_debug = {
    .address = {1, 2, 4, 3},
    .forward_sign = {CHASSIS_FL_FORWARD_SIGN, CHASSIS_FR_FORWARD_SIGN,
                     CHASSIS_RL_FORWARD_SIGN, CHASSIS_RR_FORWARD_SIGN},
    .mapping_confirmed = CHASSIS_DIRECTIONS_CONFIRMED,
    .rpm = 20,
    .single_sign = 1,
    .duration_ms = 1000,
    .next_test = 1
};

static uint32_t last_trigger;
static uint32_t run_started;
static uint32_t run_duration;
static uint32_t poll_started;
static uint8_t poll_wheel;
static uint8_t active_address[4];
static uint8_t active_wheel[4];
static uint8_t key_raw = 1;
static uint8_t key_stable = 1;
static uint8_t key_consumed;
static uint8_t key_confirm;
static uint32_t key_changed;
static uint32_t key_pressed;

static uint8_t TxOK(uint8_t addr)
{
    if (can_last_tx_status == HAL_OK) return 1;
    chassis_debug.fault = 2;//?? 2
    chassis_debug.fault_address = addr;
    return 0;
}

static uint8_t WaitAck(uint8_t addr, uint8_t function, uint32_t previous)
{
    uint32_t started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < ACK_TIMEOUT_MS) {
        if (motor_feedback[addr].ack_count != previous &&
            motor_feedback[addr].ack_function == function) {
            if (motor_feedback[addr].ack_status == 0x02U) {
                HAL_Delay(3);
                return 1;
            }
            break;
        }
    }
    chassis_debug.fault = 3;
    chassis_debug.fault_address = addr;
    return 0;
}

static uint8_t SetVelocity(uint8_t addr, int16_t rpm)
{
    uint32_t previous = motor_feedback[addr].ack_count;
    uint8_t direction = rpm < 0 ? 1U : 0U;
    uint16_t magnitude = (uint16_t)(rpm < 0 ? -rpm : rpm);
    Emm_V5_Vel_Control(addr, direction, magnitude, EMM_ACC, false);
    return TxOK(addr) && WaitAck(addr, 0xF6U, previous);
}

static uint8_t EnableWheel(uint8_t addr)
{
    uint32_t previous = motor_feedback[addr].ack_count;
    Emm_V5_En_Control(addr, true, false);
    return TxOK(addr) && WaitAck(addr, 0xF3U, previous);
}

static uint8_t ReadFlags(uint8_t addr)
{
    uint32_t previous = motor_feedback[addr].flags_count;
    uint32_t started;
    Emm_V5_Read_Sys_Params(addr, S_FLAG);
    if (!TxOK(addr)) return 0;
    started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < ACK_TIMEOUT_MS) {
        if (motor_feedback[addr].flags_count != previous) {
            if ((motor_feedback[addr].flags & 0x0DU) == 0x01U) return 1;
            chassis_debug.fault = 4;
            chassis_debug.fault_address = addr;
            return 0;
        }
    }
    chassis_debug.fault = 5;
    chassis_debug.fault_address = addr;
    return 0;
}

static void StopWheels(void)
{
    uint8_t i;
    /* Unicast only: this F103 bus may later contain other mechanisms. */
    for (i = 0; i < WHEEL_COUNT; ++i) {
        if (active_wheel[i]) {
            Emm_V5_Stop_Now(active_address[i], false);
            HAL_Delay(3);
        }
        active_wheel[i] = 0;
        chassis_debug.target_rpm[i] = 0;
    }
}

static uint8_t CheckConfig(uint32_t command)
{
    uint8_t i, j, addr;
    if (command == 0U || command > 10U || chassis_debug.rpm == 0U ||
        chassis_debug.rpm > TEST_RPM_LIMIT ||
        chassis_debug.duration_ms == 0U ||
        chassis_debug.duration_ms > TEST_TIME_LIMIT_MS ||
        (chassis_debug.single_sign != 1 && chassis_debug.single_sign != -1))
        return 0;
    if (command >= 5U && chassis_debug.mapping_confirmed != 1U) return 0;
    for (i = 0; i < WHEEL_COUNT; ++i) {
        addr = chassis_debug.address[i];
        if (addr == 0U || addr > 4U ||
            (chassis_debug.forward_sign[i] != 1 &&
             chassis_debug.forward_sign[i] != -1)) return 0;
        for (j = 0; j < i; ++j)
            if (addr == chassis_debug.address[j]) return 0;
    }
    return 1;
}

/* X roller layout; positive wheel RPM means rolling the vehicle forward.
 * +forward, +left, +CCW, each normalized to a motor-RPM contribution.
 * Physical units (m/s, rad/s) require wheel radius, geometry and gearing.
 */
static void Mix(int16_t forward, int16_t left, int16_t ccw, int16_t rpm[4])
{
    rpm[0] = forward - left - ccw;
    rpm[1] = forward + left + ccw;
    rpm[2] = forward + left - ccw;
    rpm[3] = forward - left + ccw;
}

static void StartTest(uint32_t command)
{
    int16_t rpm[4] = {0, 0, 0, 0};
    int16_t speed = (int16_t)chassis_debug.rpm;
    int16_t signed_rpm;
    uint8_t i;
    if (!CheckConfig(command)) {
        chassis_debug.fault = 1;
        chassis_debug.state = 2;
        return;
    }
    if (command <= 4U) rpm[command - 1U] = speed * chassis_debug.single_sign;
    else {
        switch (command) {
        case 5: Mix(speed, 0, 0, rpm); break;
        case 6: Mix(-speed, 0, 0, rpm); break;
        case 7: Mix(0, speed, 0, rpm); break;
        case 8: Mix(0, -speed, 0, rpm); break;
        case 9: Mix(0, 0, speed, rpm); break;
        case 10: Mix(0, 0, -speed, rpm); break;
        default: return;
        }
    }
    run_duration = chassis_debug.duration_ms;
    for (i = 0; i < WHEEL_COUNT; ++i) {
        active_address[i] = chassis_debug.address[i];
        active_wheel[i] = 1;
    }
    /* All 4 motors must be connected, calibrated and responding. */
    for (i = 0; i < WHEEL_COUNT; ++i) {
        uint32_t previous = motor_feedback[active_address[i]].ack_count;
        Emm_V5_Stop_Now(active_address[i], false);
        if (!TxOK(active_address[i]) ||
            !WaitAck(active_address[i], 0xFEU, previous) ||
            !EnableWheel(active_address[i]) ||
            !SetVelocity(active_address[i], 0) ||
            !ReadFlags(active_address[i]))
            goto failed;
    }
    for (i = 0; i < WHEEL_COUNT; ++i) {
        chassis_debug.target_rpm[i] = rpm[i];
        signed_rpm = rpm[i] * chassis_debug.forward_sign[i];
        if (!SetVelocity(active_address[i], signed_rpm)) goto failed;
    }
    run_started = HAL_GetTick();
    poll_started = run_started;
    poll_wheel = 0;
    chassis_debug.state = 1;
    return;
failed:
    StopWheels();
    chassis_debug.state = 2;
}

void Chassis_DebugInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    for (i = 0; i < WHEEL_COUNT; ++i) {
        active_address[i] = chassis_debug.address[i];
        active_wheel[i] = 1;
    }
    StopWheels();
    last_trigger = chassis_debug.trigger;
}

/* PC5 KEY0, active low. Press during a run stops without starting another.
 * Short release runs the next test; hold 800ms to return to test 1.
 * At unconfirmed test 5 only, hold 2s to confirm directions (no movement).
 */
static void PollKey(uint32_t now)
{
    uint8_t raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET ? 0U : 1U;
    if (raw != key_raw) {
        key_raw = raw;
        key_changed = now;
    }
    if (raw != key_stable && (uint32_t)(now - key_changed) >= 30U) {
        key_stable = raw;
        if (raw == 0U) {
            key_pressed = now;
            key_consumed = 0;
            key_confirm = (chassis_debug.state != 1U &&
                           chassis_debug.next_test == 5U &&
                           chassis_debug.mapping_confirmed == 0U);
            if (chassis_debug.state == 1U) {
                chassis_debug.command = 0;
                key_consumed = 1;
            }
        } else if (!key_consumed) {
            chassis_debug.command = chassis_debug.next_test;
            ++chassis_debug.trigger;
        }
    }
    if (key_stable == 0U && key_confirm && !key_consumed &&
        (uint32_t)(now - key_pressed) >= 2000U) {
        chassis_debug.mapping_confirmed = 1;
        chassis_debug.command = 0;
        key_consumed = 1;
    } else if (key_stable == 0U && !key_confirm &&
               (uint32_t)(now - key_pressed) >= 800U) {
        chassis_debug.command = 0;
        chassis_debug.next_test = 1;
        key_consumed = 1;
    }
}

void Chassis_DebugTask(void)
{
    uint8_t i, addr;
    uint32_t now = HAL_GetTick();
    uint32_t command;
    PollKey(now);
    if (chassis_debug.command == 0U) {
        if (chassis_debug.state == 1U) StopWheels();
        chassis_debug.state = 0;
        chassis_debug.fault = 0;
        chassis_debug.fault_address = 0;
        last_trigger = chassis_debug.trigger;
        return;
    }
    if (chassis_debug.trigger != last_trigger) {
        last_trigger = chassis_debug.trigger;
        StopWheels();
        chassis_debug.state = 0;
        chassis_debug.fault = 0;
        chassis_debug.fault_address = 0;
        command = chassis_debug.command;
        StartTest(command);
        if (chassis_debug.state == 1U)
            chassis_debug.next_test = command < 10U ? command + 1U : 1U;
        return;
    }
    if (chassis_debug.state != 1U) return;
    if ((uint32_t)(now - run_started) >= run_duration) {
        StopWheels();
        chassis_debug.state = 0;
        return;
    }
    for (i = 0; i < WHEEL_COUNT; ++i) {
        addr = active_address[i];
        if ((motor_feedback[addr].flags & 0x0DU) != 0x01U) {
            chassis_debug.fault = 4;
            chassis_debug.fault_address = addr;
            goto failed;
        }
        if ((uint32_t)(now - motor_feedback[addr].flags_tick) >
            FEEDBACK_TIMEOUT_MS) {
            chassis_debug.fault = 5;
            chassis_debug.fault_address = addr;
            goto failed;
        }
    }
    if ((uint32_t)(now - poll_started) >= 50U) {
        poll_started = now;
        addr = active_address[poll_wheel];
        Emm_V5_Read_Sys_Params(addr, S_FLAG);
        if (!TxOK(addr)) goto failed;
        HAL_Delay(3);
        Emm_V5_Read_Sys_Params(addr, S_VEL);
        if (!TxOK(addr)) goto failed;
        poll_wheel = (uint8_t)((poll_wheel + 1U) % WHEEL_COUNT);
    }
    return;
failed:
    StopWheels();
    chassis_debug.state = 2;
}
