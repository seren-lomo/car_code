#ifndef CAR_ARM_CONFIG_H
#define CAR_ARM_CONFIG_H

/* 0: complete vehicle; 1: stationary UART4 vision/arm commissioning.
 * Rebuild and download after changing. Test mode blocks route START and
 * chassis Watch moves; selection keys only record the manual arm origins.
 */
#ifndef ARM_VISION_TEST_MODE
#define ARM_VISION_TEST_MODE      1
#endif
#if ARM_VISION_TEST_MODE != 0 && ARM_VISION_TEST_MODE != 1
#error ARM_VISION_TEST_MODE_must_be_0_or_1
#endif

/* PROVISIONAL: motor shaft coordinates, 0.01 degree. These are NOT mechanism
 * millimetres or output yaw degrees. Positive = EMM direction 0 (CW).
 * Measure direction, gearing, camera-to-claw offset and clearance before use.
 * Measured: +Z up, +yaw right/clockwise, +reach forward.
 * Origin: Z uppermost, reach foremost, arm pointing into the work area.
 * Reported travel: Z down 145 mm; reach back 120 mm.
 * Software targets do not provide physical end-stop detection.
 * No stepper motion occurs at boot; explicitly record the manually set pose.
 */
#define ARM_PULSES_PER_REV       3200U /* 200 steps * MStep 16 */
#define ARM_Z_RPM                90U
#define ARM_YAW_RPM              40U
#define ARM_REACH_RPM            50U
#define ARM_ACCELERATION         100U
#define ARM_TOLERANCE_CDEG       50U
#define ARM_Z_MIN_CDEG          (-130500L) /* 145 mm * 900 cdeg/mm */
#define ARM_Z_MAX_CDEG           0L /* origin = uppermost; +Z goes up */
#define ARM_YAW_MIN_CDEG        (-27000L)//30度行程，90度为27000
#define ARM_YAW_MAX_CDEG         27000L /* normal work yaw: +90 degrees at 3:1 */
#define ARM_REACH_MIN_CDEG      (-38196L) /* floor(120 mm * 36000 / 113.1) */
#define ARM_REACH_MAX_CDEG       0L /* origin = foremost; +reach goes forward */

/* Placeholder pick stroke: 90 MOTOR degrees. Set sign and stroke after
 * measuring the Z belt. It is an offset from the visually aligned Z pose.
 */
#define ARM_PICK_DROP_CDEG       77450L /* 100 mm down at 40 mm/motor turn */
#define ARM_Z_DOWN_SIGN         (-1L) /* measured: negative Z goes down */

/* PA6 / TIM3_CH1: 10 us per count with a 20 ms period. */
#define ARM_CLAW_OPEN_COMPARE    50U
#define ARM_CLAW_CLOSED_COMPARE  170U
#define ARM_CLAW_CLOSE_MS        500U

/* Origin is fully UP and fully FORWARD. Reach=0 is NOT retracted.
 * Yaw clearance: first lift to Z=0, then retract 60 mm.
 * Never rotate yaw below 105 mm down from the upper origin.
 * Transport stays 60 mm retracted.
 * It is not inferred to be a mechanical end stop. Calibrate these values.
 * Before a yaw adjustment: Z clearance -> reach clearance -> yaw -> reach -> Z.
 */
#define ARM_CLEAR_Z_CDEG         0L
#define ARM_CLEAR_REACH_CDEG    (-19098L) /* 60 mm * 36000 / 113.1, rounded *///转yaw前退的距离
#define ARM_YAW_MIN_Z_CDEG      (-94500L) /* maximum 105 mm down while yaw rotates */
/* A visual yaw correction up to 1 output degree (300 motor cdeg at 3:1)
 * can turn at the current reach when it starts in the front 60 mm and Z
 * is at a clear height. Reach and Z adjustments are planned independently.
 */
#define ARM_ALIGN_MICRO_YAW_STEP_CDEG   300L
#define ARM_MATERIAL_ALIGN_YAW_STEP_CDEG  13500L /* max 45 degrees per direct turn at Z clearance */
#define ARM_MATERIAL_ALIGN_YAW_LIMIT_CDEG 13500L /* direct-turn sector: +/-45 degrees from work origin */
#define ARM_ALIGN_MICRO_MIN_Z_CDEG      ARM_YAW_MIN_Z_CDEG
#define ARM_ALIGN_MICRO_MIN_REACH_CDEG  ARM_CLEAR_REACH_CDEG
#define ARM_ALIGN_REACH_TOLERANCE_CDEG 90U /* visual reach move completion only */
#define ARM_TRANSPORT_YAW_CDEG   0L

/* PA7 / TIM3_CH2. Index positions measured from the installed first tray. */
#ifndef ARM_TRAY_ENABLED
#define ARM_TRAY_ENABLED         ARM_VISION_TEST_MODE
#endif
#define ARM_TRAY_FIRST_COMPARE   50U
#define ARM_TRAY_SECOND_COMPARE  150U
#define ARM_TRAY_THIRD_COMPARE   250U
#define ARM_TRAY_SETTLE_MS       1000U /* allow the full two-index sweep */
#define ARM_CLAW_RELEASE_MS      500U
#define ARM_TRAY_REACH_CDEG      (-33421L) /* 110 mm back at 113.1 mm per turn *///转yaw后退的距离
#define ARM_TRAY_YAW_CDEG        43500L    /* +120 degrees at the arm, 3:1 reduction  1度300  44100*/
#define ARM_TRAY_PLACE_Z_CDEG    0L        /* release at the upper origin */
#define ARM_TRAY_PICK_Z_CDEG     0L        /* grip at the upper origin */
#if ARM_TRAY_ENABLED
/* Only the fixed tray approach uses this extended positive yaw range. */
#define ARM_YAW_TRAVEL_MAX_CDEG  45000L
#if ARM_TRAY_YAW_CDEG <= 0 || ARM_TRAY_YAW_CDEG > ARM_YAW_TRAVEL_MAX_CDEG || \
    ARM_TRAY_REACH_CDEG < ARM_REACH_MIN_CDEG || ARM_TRAY_REACH_CDEG > ARM_REACH_MAX_CDEG || \
    ARM_TRAY_PLACE_Z_CDEG < ARM_Z_MIN_CDEG || ARM_TRAY_PLACE_Z_CDEG > ARM_Z_MAX_CDEG || \
    ARM_TRAY_PICK_Z_CDEG < ARM_Z_MIN_CDEG || ARM_TRAY_PICK_Z_CDEG > ARM_Z_MAX_CDEG
#error ARM_TRAY_pose_outside_arm_travel_limits
#endif
#else
#define ARM_YAW_TRAVEL_MAX_CDEG  ARM_YAW_MAX_CDEG
#endif
#endif
