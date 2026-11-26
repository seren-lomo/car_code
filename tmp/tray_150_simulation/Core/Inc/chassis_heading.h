#ifndef CHASSIS_HEADING_H
#define CHASSIS_HEADING_H
#include "chassis_motion.h"

#define CHASSIS_MAX_TURN_DEG 360.0f
#define CHASSIS_TURN_TIMEOUT_MS 60000U

typedef struct {
    float distance_kp;           /* motor RPM / mm, default 0.20 */
    float yaw_kp;                /* differential motor RPM / degree, default 1.5 */
    float yaw_kd;                /* differential RPM / (degree/s), default 0 */
    float max_correction_rpm;    /* default 12 RPM */
    float ramp_rpm_per_s;        /* software acceleration limit, current default 120 */
    float distance_tolerance_mm; /* default 3 mm of wheel odometry */
    float yaw_tolerance_deg;     /* default 1 degree */
    float max_yaw_error_deg;     /* default 20 degrees: fault stop */
    int8_t imu_yaw_sign;         /* Vehicle default -1; +1 if raw yaw grows on CCW turn. */
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
    uint8_t action;              /* 0 translate with heading hold, 1 rotate in place, 2 continuous path */
    float target_turn_deg;      /* Requested signed chassis turn: +CCW, -CW. */
    float turned_deg;           /* Raw IMU increments accumulated across +/-180. */
    float remaining_turn_deg;
} ChassisHeading_t;

/* Last heading-controller fault, retained after stop clears live commands.
 * Inspect only after the vehicle stops; never halt an active velocity move.
 * Arrays use FL, FR, RL, RR order, with CAN IDs recorded in address[].
 * command_rpm is the latest computed vector, possibly awaiting sync.
 * feedback_age_ms == 0xFFFFFFFF means no speed sample was received.
 * valid remains set until reset or the next fault overwrites this snapshot.
 */
typedef struct {
    uint32_t valid, tick, fault;
    uint8_t fault_address, action, pending_address, pending_function;
    float target_yaw_deg, yaw_deg, yaw_error_deg, correction_rpm;
    int16_t command_rpm[4], feedback_rpm[4];
    uint32_t feedback_age_ms[4];
    uint8_t address[4], motor_flags[4];
    int8_t forward_sign[4], imu_yaw_sign;
} ChassisHeadingFaultSnapshot_t;
extern volatile ChassisHeadingFaultSnapshot_t chassis_heading_fault_snapshot;

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
/* Relative in-place turn, +CCW/left, -CW/right, up to +/-360 degrees.
 * max_rpm is a wheel motor RPM limit, not a chassis angular velocity.
 * Uses signed continuous progress, so +180 and -180 keep their requested direction.
 * The 20-degree heading guard applies to unintended deviation, not the requested turn.
 */
ChassisResult_t Chassis_TurnRelative(float degrees, uint16_t max_rpm);
ChassisResult_t Chassis_TurnAround(uint16_t max_rpm); /* +180 degrees */

/* Internal dispatch hooks: use Chassis_Task/Stop from business code. */
void Chassis_HeadingTask(void);
void Chassis_HeadingStop(void);
#endif
