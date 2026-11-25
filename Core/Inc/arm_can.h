#ifndef CAR_ARM_CAN_H
#define CAR_ARM_CAN_H
#include "main.h"

#define ARM_AXIS_COUNT 3U
typedef enum { ARM_Z=0, ARM_YAW=1, ARM_REACH=2 } ArmAxis_t;
typedef enum { ARM_MOTOR_IDLE=0, ARM_MOTOR_BUSY=1, ARM_MOTOR_DONE=2,
               ARM_MOTOR_FAULT=3, ARM_MOTOR_STOPPED=4 } ArmMotorState_t;
typedef enum { ARM_MOTOR_OK=0, ARM_MOTOR_CONFIG=1, ARM_MOTOR_TX=2,
               ARM_MOTOR_REPLY=3, ARM_MOTOR_TIMEOUT=4, ARM_MOTOR_FLAGS=5,
               ARM_MOTOR_CAN=6, ARM_MOTOR_RX_OVERFLOW=7 } ArmMotorFault_t;

/* Coordinates are signed MOTOR shaft centidegrees relative to an explicitly
 * recorded startup pose. Positive means protocol direction 0 (CW).
 * No gear ratio is guessed: Pi converts mechanism distances/angles to shafts.
 * All three origins must be recorded again following MCU/motor power loss.
 */
typedef struct {
    uint32_t pulses_per_rev; /* motor_steps * MStep; 0 blocks motion */
    int32_t min_cdeg, max_cdeg;
    uint16_t rpm, tolerance_cdeg;
    uint8_t acceleration;
} ArmMotorConfig_t;

typedef struct {
    uint32_t state, fault, tick, rx_frames, rx_errors, can_errors;
    uint32_t position_tick, speed_tick;
    int32_t position_counts, origin_counts, target_counts, position_cdeg;
    int16_t speed_rpm;
    uint8_t address, homed, flags, last_function, last_status;
} ArmMotorStatus_t;

/* Diagnostic counters are cumulative since reset. rx_raw holds the first
 * 16 CAN payload bytes received since the most recently attempted TX.
 * tx_raw retains the logical address/function/data/6B command before CAN splitting.
 * timeout_* freezes the last timeout before stop commands overwrite live state.
 * fault_code is set on an active operation failure and cleared at the next
 * operation. The remaining fault_* fields preserve the context before stopping.
 */
typedef struct {
    uint32_t tx_requests, tx_completed, rx_bytes, last_hal_status;
    uint32_t timeout_function, timeout_stage, timeout_tx_active, timeout_waiting;
    uint32_t tx_can_frames, rx_can_frames, last_rx_ext_id, last_rx_dlc, last_can_error;
    uint8_t tx_length, tx_raw[13], rx_length, rx_raw[16];
    uint8_t fault_code, fault_stage, fault_function, fault_flags, fault_status;
    int16_t fault_speed_rpm;
} ArmCANDiag_t;
extern volatile ArmCANDiag_t arm_can_diag[ARM_AXIS_COUNT];

extern volatile ArmMotorConfig_t arm_motor_config[ARM_AXIS_COUNT];
extern volatile ArmMotorStatus_t arm_motor_status[ARM_AXIS_COUNT];

/* Shared CAN1, chassis IDs 1..4; Z/yaw/reach IDs 5/6/7. Init resets local
 * state only; main owns CAN startup. Task/commands: RobotTask only. The shared
 * FIFO0 ISR calls RxFrame, which only enqueues complete single-frame replies.
 */
HAL_StatusTypeDef ArmCAN_Init(void);
void ArmCAN_Task(void);
uint8_t ArmCAN_Busy(void);
uint8_t ArmCAN_Probe(uint8_t axis); /* one 0x36 read; no enable/move/zero */
uint8_t ArmCAN_RecordOrigin(uint8_t axis); /* F3 enable, then verify stopped */
uint8_t ArmCAN_Move(uint8_t axis, int32_t motor_cdeg);
/* Override this move's completion tolerance only. Zero uses the axis default. */
uint8_t ArmCAN_MoveWithTolerance(uint8_t axis, int32_t motor_cdeg, uint16_t tolerance_cdeg);
/* Check position feedback against the axis default. */
uint8_t ArmCAN_PositionWithinTolerance(uint8_t axis, int32_t motor_cdeg);
void ArmCAN_StopAll(void); /* hold enable; never release a loaded Z axis */
void ArmCAN_RxFrame(uint32_t ext_id,const uint8_t *data,uint8_t dlc);

#endif
