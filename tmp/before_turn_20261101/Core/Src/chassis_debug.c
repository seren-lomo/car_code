#include "chassis_debug.h"
#include "chassis_heading.h"

volatile ChassisDebug_t chassis_debug = {
    .distance_mm = 500.0f, .rpm = 50, .acc = 100, .next_test = 1, .use_imu = 1
};
volatile uint32_t chassis_task_count, chassis_task_tick;
volatile uint32_t key0_level = 1, key0_press_count;
static uint32_t last_trigger, key_changed, key_pressed;
static uint32_t last_command;
static uint8_t key_raw = 1, key_stable = 1, key_consumed;

void Chassis_DebugInit(void)
{
    Chassis_Init();
    last_trigger = chassis_debug.trigger;
    last_command = chassis_debug.command;
}

/* KEY0 PC5 active low. A short RELEASE starts; press while moving cancels.
 * Holding 800 ms resets next_test to 1 without a move on release. */
static void PollKey(uint32_t now)
{
    uint8_t raw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET ? 0U : 1U;
    key0_level = raw;
    if (raw != key_raw) { key_raw = raw; key_changed = now; }
    if (raw != key_stable && (uint32_t)(now - key_changed) >= 30U) {
        key_stable = raw;
        if (raw == 0U) {
            ++key0_press_count;
            key_pressed = now;
            key_consumed = 0;
            if (Chassis_IsBusy()) {
                Chassis_Stop();
                chassis_debug.command = 0;
                key_consumed = 1;
            }
        } else if (!key_consumed) {
            chassis_debug.command = chassis_debug.next_test;
            ++chassis_debug.trigger;
        }
    }
    if (key_stable == 0U && (uint32_t)(now - key_pressed) >= 800U) {
        if (!key_consumed) Chassis_Stop();
        chassis_debug.command = 0;
        chassis_debug.next_test = 1;
        key_consumed = 1;
    }
}

void Chassis_DebugTask(void)
{
    uint32_t command, now = HAL_GetTick();
    ChassisResult_t result;
    ++chassis_task_count;
    chassis_task_tick = now;
    PollKey(now);
    if (chassis_debug.command == 0U) {
        /* An idle debug panel must not cancel moves started by business code.
         * Stop on an explicit command transition or a command=0 trigger. */
        if ((last_command != 0U || chassis_debug.trigger != last_trigger) &&
            Chassis_IsBusy()) Chassis_Stop();
        last_trigger = chassis_debug.trigger;
    } else if (chassis_debug.trigger != last_trigger) {
        last_trigger = chassis_debug.trigger;
        command = chassis_debug.command;
        /* Watch triggers during motion are ignored, never queued. */
        if (!Chassis_IsBusy()) {
            if (command < 1U || command > 4U ||
                !(chassis_debug.distance_mm > 0.0f &&
                  chassis_debug.distance_mm <= CHASSIS_MAX_DISTANCE_MM)) {
                chassis_motion.fault = CHASSIS_CONFIG_ERROR;
                chassis_motion.fault_address = 0;
                chassis_motion.state = CHASSIS_FAULT;
            } else {
                float mm = chassis_debug.distance_mm;
                if (command == 2U || command == 4U) mm = -mm;
                if (chassis_debug.use_imu) {
                    result = command <= 2U ?
                        Chassis_MoveForwardHold(mm, chassis_debug.rpm) :
                        Chassis_MoveLeftHold(mm, chassis_debug.rpm);
                } else {
                    result = command <= 2U ?
                        Chassis_MoveForward(mm, chassis_debug.rpm, chassis_debug.acc) :
                        Chassis_MoveLeft(mm, chassis_debug.rpm, chassis_debug.acc);
                }
                if (result == CHASSIS_OK)
                    chassis_debug.next_test = command < 4U ? command + 1U : 1U;
            }
        }
    }
    last_command = chassis_debug.command;
    Chassis_Task();
    chassis_debug.state = chassis_motion.state;
    chassis_debug.fault = chassis_motion.fault;
    chassis_debug.fault_address = chassis_motion.fault_address;
}
