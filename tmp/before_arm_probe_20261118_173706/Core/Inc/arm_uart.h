#ifndef CAR_ARM_UART_H
#define CAR_ARM_UART_H
#include "main.h"

#define ARM_AXIS_COUNT 3U
typedef enum { ARM_Z=0, ARM_YAW=1, ARM_REACH=2 } ArmAxis_t;
typedef enum { ARM_MOTOR_IDLE=0, ARM_MOTOR_BUSY=1, ARM_MOTOR_DONE=2,
               ARM_MOTOR_FAULT=3, ARM_MOTOR_STOPPED=4 } ArmMotorState_t;
typedef enum { ARM_MOTOR_OK=0, ARM_MOTOR_CONFIG=1, ARM_MOTOR_TX=2,
               ARM_MOTOR_REPLY=3, ARM_MOTOR_TIMEOUT=4, ARM_MOTOR_FLAGS=5,
               ARM_MOTOR_UART=6, ARM_MOTOR_RX_OVERFLOW=7 } ArmMotorFault_t;

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
    uint32_t state, fault, tick, rx_frames, rx_errors, uart_errors;
    uint32_t position_tick, speed_tick;
    int32_t position_counts, origin_counts, target_counts, position_cdeg;
    int16_t speed_rpm;
    uint8_t address, homed, flags, last_function, last_status;
} ArmMotorStatus_t;

extern volatile ArmMotorConfig_t arm_motor_config[ARM_AXIS_COUNT];
extern volatile ArmMotorStatus_t arm_motor_status[ARM_AXIS_COUNT];

/* Init only arms UART RX; it never enables or moves a motor. Task and command
 * functions have one owner: RobotTask. RX callbacks only enqueue bytes.
 */
HAL_StatusTypeDef ArmUART_Init(void);
void ArmUART_Task(void);
uint8_t ArmUART_Busy(void);
uint8_t ArmUART_RecordOrigin(uint8_t axis); /* F3 enable, then verify stopped */
uint8_t ArmUART_Move(uint8_t axis, int32_t motor_cdeg);
void ArmUART_StopAll(void); /* hold enable; never release a loaded Z axis */
void ArmUART_RxCpltCallback(UART_HandleTypeDef *uart);
void ArmUART_ErrorCallback(UART_HandleTypeDef *uart);
void ArmUART_TxCpltCallback(UART_HandleTypeDef *uart);

#endif
