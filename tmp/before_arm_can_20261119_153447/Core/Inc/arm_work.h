#ifndef CAR_ARM_WORK_H
#define CAR_ARM_WORK_H
#include "route.h"
#include "arm_uart.h"

typedef enum {
    ARM_WORK_NEED_ORIGIN=0, ARM_WORK_ZEROING=1, ARM_WORK_READY=2,
    ARM_WORK_PREPARING=3, ARM_WORK_WAIT_VISION=4, ARM_WORK_ALIGNING=5,
    ARM_WORK_ALIGNED=6, ARM_WORK_LOWERING=7, ARM_WORK_WAIT_GRIP=8,
    ARM_WORK_LIFTING=9, ARM_WORK_PICKED=10, ARM_WORK_PARKING=11,
    ARM_WORK_READY_LEAVE=12, ARM_WORK_STOPPED=13, ARM_WORK_FAULT=14,
    ARM_WORK_DEBUG_MOVING=15, ARM_WORK_DEBUG_HOLD=16
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

/* RobotTask consumes one command per trigger change. Write parameters FIRST,
 * then increment trigger LAST. command: 0 stop, 1 record manual origins,
 * 2 move one axis to an ABSOLUTE motor centidegree target, 3 return all to zero.
 * 4 read position of selected axis only (works before origins / after fault).
 * Probe completion is motor state 2; it does not record origins or clear work fault.
 * Tests are rejected during routes/work. After a test, route START is blocked
 * until command 3 finishes or manual origins are explicitly recorded again.
 * result uses RouteResult_t: acceptance only; observe motor state for arrival.
 */
typedef struct {
    uint32_t command, trigger, result, axis;
    int32_t target_cdeg;
} ArmWorkDebug_t;
extern volatile ArmWorkDebug_t arm_work_debug;

#endif
