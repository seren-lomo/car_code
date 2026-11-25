#ifndef CAR_ARM_CONFIG_H
#define CAR_ARM_CONFIG_H

/* PROVISIONAL: motor shaft coordinates, 0.01 degree. These are NOT mechanism
 * millimetres or output yaw degrees. Positive = EMM direction 0 (CW).
 * Measure direction, gearing, camera-to-claw offset and clearance before use.
 * No stepper motion occurs at boot; explicitly record the manually set pose.
 */
#define ARM_PULSES_PER_REV       3200U /* 200 steps * MStep 16 */
#define ARM_Z_RPM                10U
#define ARM_YAW_RPM              10U
#define ARM_REACH_RPM            10U
#define ARM_ACCELERATION         100U
#define ARM_TOLERANCE_CDEG       50U
#define ARM_Z_MIN_CDEG          (-72000L)
#define ARM_Z_MAX_CDEG           72000L
#define ARM_YAW_MIN_CDEG        (-27000L)//30度行程，90度为27000
#define ARM_YAW_MAX_CDEG         27000L
#define ARM_REACH_MIN_CDEG      (-72000L)
#define ARM_REACH_MAX_CDEG       72000L

/* Placeholder pick stroke: 90 MOTOR degrees. Set sign and stroke after
 * measuring the Z belt. It is an offset from the visually aligned Z pose.
 */
#define ARM_PICK_DROP_CDEG       9000L//下降20mm  18000下降10
#define ARM_Z_DOWN_SIGN          1L//下降方向

/* The initial photo pose is the provisional clearance/transport pose.
 * It is not inferred to be a mechanical end stop. Calibrate these values.
 * Before a yaw adjustment: Z clearance -> reach clearance -> yaw -> reach -> Z.
 */
#define ARM_CLEAR_Z_CDEG         0L
#define ARM_CLEAR_REACH_CDEG     0L
#define ARM_TRANSPORT_YAW_CDEG   0L
#define ARM_PHOTO_Z_CDEG         0L
#define ARM_PHOTO_REACH_CDEG     0L
#define ARM_PHOTO_YAW_CDEG       0L
#endif
