#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#include "main.h"

#define CHASSIS_DIRECTIONS_CONFIRMED 0U
/* Or hold KEY0 for 2 seconds at test 5 after verifying wheel directions. */
#define CHASSIS_FL_FORWARD_SIGN (+1)
#define CHASSIS_FR_FORWARD_SIGN (-1)
#define CHASSIS_RL_FORWARD_SIGN (+1)
#define CHASSIS_RR_FORWARD_SIGN (-1)
/* Defaults assume opposite mounting on left/right; verify on the chassis. */

/* Wheel order throughout this module: FL, FR, RL, RR (viewed from above). */
typedef struct {
    uint8_t address[4];
    int8_t forward_sign[4]; /* +1: dir=0 drives that wheel forward; -1: dir=1. */
    uint8_t mapping_confirmed; /* Set to 1 only after single-wheel calibration. */
    uint16_t rpm;           /* Motor RPM, S_Vel_IS must be Disable. */
    int8_t single_sign;     /* +1 or -1 for single-wheel direction testing. */
    uint32_t duration_ms;
    uint32_t command;       /* See README_chassis_debug.md. 0 stops immediately. */
    uint32_t trigger;       /* Increment LAST to execute one timed test. */
    uint32_t next_test;     /* KEY0 short press starts this test, 1..10. */
    uint32_t state;         /* 0 idle, 1 running, 2 fault. */
    uint32_t fault;         /* 1 config, 2 TX, 3 ACK, 4 motor, 5 feedback timeout. */
    uint8_t fault_address;
    int16_t target_rpm[4];  /* Wheel-forward sign, before forward_sign mapping. */
} ChassisDebug_t;

extern volatile ChassisDebug_t chassis_debug;
/* Live diagnostics, separate from the chassis parameter structure. */
extern volatile uint32_t chassis_task_count;
extern volatile uint32_t chassis_task_tick;
extern volatile uint32_t key0_level;       /* Released=1, pressed=0. */
extern volatile uint32_t key0_press_count; /* Debounced presses. */
void Chassis_DebugInit(void);
void Chassis_DebugTask(void);

#endif
