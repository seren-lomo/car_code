#ifndef CAR_ARM_WORK_H
#define CAR_ARM_WORK_H
#include "route.h"
#include "arm_can.h"

typedef enum {
    ARM_WORK_NEED_ORIGIN=0, ARM_WORK_ZEROING=1, ARM_WORK_READY=2,
    ARM_WORK_PREPARING=3, /* Reserved legacy wire value. */
    ARM_WORK_WAIT_VISION=4, ARM_WORK_ALIGNING=5,
    ARM_WORK_ALIGNED=6, ARM_WORK_LOWERING=7, ARM_WORK_WAIT_GRIP=8,
    ARM_WORK_LIFTING=9, ARM_WORK_PICKED=10, ARM_WORK_PARKING=11,
    ARM_WORK_READY_LEAVE=12, ARM_WORK_STOPPED=13, ARM_WORK_FAULT=14,
    ARM_WORK_DEBUG_MOVING=15, ARM_WORK_DEBUG_HOLD=16,
    ARM_WORK_TRAY_LOADING=17, ARM_WORK_TRAY_UNLOADING=18, ARM_WORK_PLACED=19
} ArmWorkState_t;
typedef enum {
    ARM_WORK_NO_FAULT=0, ARM_WORK_MOTOR=1, ARM_WORK_POSE=2,
    ARM_WORK_CONTEXT=3, ARM_WORK_CANCELLED=4, ARM_WORK_TRAY_PWM=5
} ArmWorkFault_t;

typedef struct {
    uint32_t state, fault, route_id, command_id, aligned_command_id, tick;
    uint16_t point_index, work_id;
    uint8_t active_axis, opcode;
    int32_t target_cdeg[3]; /* Z, yaw, reach */
} ArmWorkStatus_t;
extern volatile ArmWorkStatus_t arm_work_status;

/* Material work 102/105 closes the PA6 claw when Z reaches its pick height.
 * Command 4 requests the lift. The material claw stays closed during lifting.
 */
HAL_StatusTypeDef ArmWork_Init(void);
void ArmWork_Task(void); /* once per RobotTask tick, after Route_Task */

/* ARM_VISION_TEST_MODE=1: UART4 ARM_COMMAND opcode 6 opens a stationary
 * work-area session (route_id=test_id, point_index=0, work_id=102..107,
 * command_id=1). Then use existing ALIGN/PICK/GRIP_DONE/FINISH opcodes.
 * ARM_EVENT flags bit3 identifies this build; real route state is unchanged.
 * See README_arm_vision_test.md for the complete Pi interface.
 */

/* RobotTask consumes one command per trigger change. Write parameters FIRST,
 * then increment trigger LAST. command: 0 stop, 1 record manual origins,
 * 2 move selected axis to an ABSOLUTE motor centidegree target, 3 return all to zero.
 * Yaw tests first raise Z and retract reach to the configured clearance pose;
 * they leave Z/reach there. Z/reach tests move only the selected axis.
 * 4 read position of selected axis only (works before origins / after fault).
 * Probe completion is motor state 2; it does not record origins or clear work fault.
 * 5 acknowledge a manually emptied tray and claw after an interrupted transfer.
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
