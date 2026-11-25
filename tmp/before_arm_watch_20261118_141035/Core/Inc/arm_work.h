#ifndef CAR_ARM_WORK_H
#define CAR_ARM_WORK_H
#include "route.h"
#include "arm_uart.h"

typedef enum {
    ARM_WORK_NEED_ORIGIN=0, ARM_WORK_ZEROING=1, ARM_WORK_READY=2,
    ARM_WORK_PREPARING=3, ARM_WORK_WAIT_VISION=4, ARM_WORK_ALIGNING=5,
    ARM_WORK_ALIGNED=6, ARM_WORK_LOWERING=7, ARM_WORK_WAIT_GRIP=8,
    ARM_WORK_LIFTING=9, ARM_WORK_PICKED=10, ARM_WORK_PARKING=11,
    ARM_WORK_READY_LEAVE=12, ARM_WORK_STOPPED=13, ARM_WORK_FAULT=14
} ArmWorkState_t;
typedef enum {
    ARM_WORK_NO_FAULT=0, ARM_WORK_MOTOR=1, ARM_WORK_POSE=2,
    ARM_WORK_CONTEXT=3, ARM_WORK_CANCELLED=4
} ArmWorkFault_t;

typedef struct {
    uint32_t state, fault, route_id, command_id, aligned_command_id, tick;
    uint16_t point_index, work_id;
    uint8_t active_axis, opcode;
    int32_t target_cdeg[3]; /* Z, yaw, reach */
} ArmWorkStatus_t;
extern volatile ArmWorkStatus_t arm_work_status;

/* No PWM/claw actuation. WAIT_GRIP only advances on explicit Pi command 4.
 * GPIO/servo support and onboard tray placement intentionally remain absent.
 */
HAL_StatusTypeDef ArmWork_Init(void);
void ArmWork_Task(void); /* once per RobotTask tick, after Route_Task */

/* Watch setup only: set command=1 then increment trigger to record all three
 * manually positioned origins with the route stopped and chassis stationary.
 * command=0 + increment trigger requests Route_Stop(). No motion test shortcuts.
 */
typedef struct { uint32_t command, trigger, result; } ArmWorkDebug_t;
extern volatile ArmWorkDebug_t arm_work_debug;

#endif
