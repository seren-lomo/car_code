#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H
#include "chassis_motion.h"
typedef struct {
    float distance_mm;      /* Positive test magnitude, default 500 mm. */
    uint16_t rpm;
    uint8_t acc;
    uint32_t next_test;     /* Translation 1..4; optional rotation 5..7. */
    uint32_t command;       /* 0 stop, 1..4 translate, 5 CCW, 6 CW, 7 turnaround. */
    uint32_t trigger;       /* Increment LAST to request one test. */
    uint32_t state, fault;  /* Mirrors chassis_motion; see chassis_motion.h. */
    uint8_t fault_address;
    uint8_t use_imu;        /* 1: distance + IMU velocity; 0: original position test. */
    float turn_deg;        /* Positive magnitude for commands 5/6, default 90. */
    uint16_t turn_rpm;      /* Wheel RPM limit for commands 5..7, default 20. */
    uint8_t include_turn_tests; /* 0 keeps KEY0 cycle 1..4; 1 enables 1..7. */
} ChassisDebug_t;
extern volatile ChassisDebug_t chassis_debug;
extern volatile uint32_t chassis_task_count, chassis_task_tick;
extern volatile uint32_t key0_level, key0_press_count;
void Chassis_DebugInit(void);
void Chassis_DebugTask(void);
#endif
