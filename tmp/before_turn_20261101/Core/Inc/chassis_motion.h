#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H
#include "main.h"

/* X mecanum. Wheel order FL, FR, RL, RR; distances in chassis-relative mm. */
#define CHASSIS_MAX_DISTANCE_MM 3000.0f
#define CHASSIS_MAX_RPM 120U
#define CHASSIS_MAX_TIMEOUT_MS 120000U
typedef enum {
    CHASSIS_IDLE = 0, CHASSIS_RUNNING = 1, CHASSIS_DONE = 2,
    CHASSIS_STOPPED = 3, CHASSIS_FAULT = 4
} ChassisState_t;
typedef enum {
    CHASSIS_OK = 0, CHASSIS_BUSY = 1, CHASSIS_INVALID = 2, CHASSIS_IO_ERROR = 3
} ChassisResult_t;
typedef enum {
    CHASSIS_NO_FAULT = 0, CHASSIS_CONFIG_ERROR = 1, CHASSIS_TX_ERROR = 2,
    CHASSIS_ACK_ERROR = 3, CHASSIS_MOTOR_ERROR = 4,
    CHASSIS_FEEDBACK_ERROR = 5, CHASSIS_MOVE_TIMEOUT = 6,
    CHASSIS_IMU_OFFLINE = 7, CHASSIS_HEADING_ERROR = 8,
    CHASSIS_WHEEL_STALLED = 9
} ChassisFault_t;
typedef struct {
    uint8_t address[4];
    int8_t forward_sign[4];
    float wheel_diameter_mm;
    float motor_turns_per_wheel_turn; /* 1.0 for direct drive. */
    uint16_t motor_steps_per_rev;    /* 200 for a 1.8-degree motor. */
    uint16_t microsteps;             /* MUST match motor MStep menu. */
    float forward_correction;        /* New = old * commanded / measured. */
    float left_correction;
} ChassisConfig_t;
typedef struct {
    uint32_t state, fault;
    uint8_t fault_address;
    uint8_t axis;                    /* 0 forward/back, 1 left/right. */
    uint8_t reached_mask;            /* bit0 FL, bit1 FR, bit2 RL, bit3 RR. */
    uint32_t completed_count;
    float requested_forward_mm, requested_left_mm;
    float pulses_per_mm;
    uint32_t target_pulses;          /* Magnitude for CURRENT axis. */
    int32_t expected_delta[4];       /* Raw feedback units: 65536/rev. */
    int32_t measured_delta[4];
    uint32_t elapsed_ms, timeout_ms;
    uint8_t control_mode;            /* 0 motor position, 1 distance + IMU velocity. */
} ChassisMotion_t;
extern volatile ChassisConfig_t chassis_config;
extern volatile ChassisMotion_t chassis_motion;
void Chassis_Init(void);             /* After CAN starts; sends stops. */
void Chassis_Task(void);             /* Call repeatedly in main loop. */
void Chassis_Stop(void);             /* Cancels the entire XY request. */
uint8_t Chassis_IsBusy(void);
/* Nonblocking start; +forward=front, +left=left. Negative values reverse.
 * XY executes FORWARD then LEFT, not a diagonal. BUSY requests are not queued.
 * Foreground only. Poll Chassis_Task until DONE, STOPPED or FAULT.
 */
ChassisResult_t Chassis_MoveXY(float forward_mm, float left_mm, uint16_t rpm, uint8_t acc);
ChassisResult_t Chassis_MoveForward(float mm, uint16_t rpm, uint8_t acc);
ChassisResult_t Chassis_MoveLeft(float mm, uint16_t rpm, uint8_t acc);
#endif
