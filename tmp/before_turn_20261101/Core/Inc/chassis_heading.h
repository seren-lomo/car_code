#ifndef CHASSIS_HEADING_H
#define CHASSIS_HEADING_H
#include "chassis_motion.h"

typedef struct {
    float distance_kp;           /* motor RPM / mm, default 0.20 */
    float yaw_kp;                /* differential motor RPM / degree, default 1.5 */
    float yaw_kd;                /* differential RPM / (degree/s), default 0 */
    float max_correction_rpm;    /* default 12 RPM */
    float ramp_rpm_per_s;        /* software acceleration limit, default 80 */
    float distance_tolerance_mm; /* default 3 mm of wheel odometry */
    float yaw_tolerance_deg;     /* default 1 degree */
    float max_yaw_error_deg;     /* default 20 degrees: fault stop */
    int8_t imu_yaw_sign;         /* +1: raw IMU yaw increases on chassis CCW turn */
} ChassisHeadingConfig_t;

typedef struct {
    uint32_t phase;              /* 0 inactive, 1 preparing, 2 moving, 3 done, 4 stopped/fault */
    uint32_t cycles, cycle_ms;   /* actual control period, not a claimed fixed rate */
    float target_mm, traveled_mm, remaining_mm;
    float forward_mm, left_mm;   /* wheel odometry projections, not ground truth */
    float target_yaw_deg, yaw_deg, yaw_error_deg;
    float translation_rpm, correction_rpm;
    int16_t wheel_rpm[4];        /* raw signed command: FL, FR, RL, RR */
    uint32_t settled_ms;
    uint8_t pending_address, pending_function;
} ChassisHeading_t;

extern volatile ChassisHeadingConfig_t chassis_heading_config;
extern volatile ChassisHeading_t chassis_heading;

/* RobotTask only. Nonblocking start; keep polling Chassis_Task().
 * Capture current raw IMU heading once per segment, hold it during translation.
 * +mm=forward/left, -mm=backward/right. The returned OK means accepted, not arrived.
 * Shared owner/state with the original position APIs; BUSY requests are rejected.
 * Motor F6 velocity mode, acc=0; acceleration/deceleration is generated here.
 */
ChassisResult_t Chassis_MoveForwardHold(float mm, uint16_t max_rpm);
ChassisResult_t Chassis_MoveLeftHold(float mm, uint16_t max_rpm);

/* Internal dispatch hooks: use Chassis_Task/Stop from business code. */
void Chassis_HeadingTask(void);
void Chassis_HeadingStop(void);
#endif
