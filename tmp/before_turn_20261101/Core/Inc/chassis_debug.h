#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H
#include "chassis_motion.h"
typedef struct {
    float distance_mm;      /* Positive test magnitude, default 500 mm. */
    uint16_t rpm;
    uint8_t acc;
    uint32_t next_test;     /* 1 forward, 2 backward, 3 left, 4 right. */
    uint32_t command;       /* 0 stops; 1..4 selects a test. */
    uint32_t trigger;       /* Increment LAST to request one test. */
    uint32_t state, fault;  /* Mirrors chassis_motion; see chassis_motion.h. */
    uint8_t fault_address;
    uint8_t use_imu;        /* 1: distance + IMU velocity; 0: original position test. */
} ChassisDebug_t;
extern volatile ChassisDebug_t chassis_debug;
extern volatile uint32_t chassis_task_count, chassis_task_tick;
extern volatile uint32_t key0_level, key0_press_count;
void Chassis_DebugInit(void);
void Chassis_DebugTask(void);
#endif
