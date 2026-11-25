#include "arm_work.h"
#include "arm_config.h"
#include "route_uart.h"
#include "chassis_motion.h"
#include "tim.h"

#define TYPE_ARM_COMMAND 0x31U
#define TYPE_ARM_STATUS  0x32U
#define TYPE_ARM_EVENT   0x83U
#define ARM_COMMAND_BYTES 32U
#define ARM_STATUS_BYTES 52U
#define CMD_ZERO 1U
#define CMD_ALIGN 2U
#define CMD_PICK 3U
#define CMD_GRIP_DONE 4U
#define CMD_FINISH 5U
#define CMD_TEST_BEGIN 6U
#define CMD_TRAY_PLACE 7U
#define CMD_TRAY_RELOAD_PICK 8U
#define CMD_GRIP_ABORT 9U

#if ARM_TRAY_ENABLED
#define TRAY_CAPACITY 3U
typedef enum {
    TRAY_PARK, TRAY_SELECT, TRAY_APPROACH, TRAY_GRIP, TRAY_MOVE_TO_SLOT,
    TRAY_RELEASE, TRAY_ADVANCE, TRAY_COMPLETE
} TrayAction_t;
static const TrayAction_t tray_load_actions[]={
    TRAY_PARK,TRAY_SELECT,TRAY_APPROACH,TRAY_RELEASE,TRAY_PARK,TRAY_ADVANCE,TRAY_COMPLETE
};
static const TrayAction_t tray_unload_actions[]={
    TRAY_PARK,TRAY_SELECT,TRAY_APPROACH,TRAY_GRIP,TRAY_MOVE_TO_SLOT,TRAY_RELEASE,
    TRAY_PARK,TRAY_ADVANCE,TRAY_COMPLETE
};
static const TrayAction_t *tray_actions;
static uint8_t tray_step, tray_count, tray_head, tray_index, tray_pwm_started;
static uint8_t tray_waiting, tray_uncertain, claw_holding, destination_slot, destination_mask;
static uint32_t tray_wait_at;
static int32_t destination_pose[3];
static uint8_t placed_slots[TRAY_CAPACITY], placed_count, reloaded_count;
#endif

volatile ArmWorkStatus_t arm_work_status;
volatile ArmWorkDebug_t arm_work_debug;
static uint8_t managed, plan_count, plan_index, moving, origin_index;
static uint8_t plan_axis[5], saved[ARM_COMMAND_BYTES], have_saved;
static uint32_t highest_command, debug_trigger;
#if ARM_VISION_TEST_MODE
static uint32_t last_test_session;
#endif
static int32_t plan_target[5], aligned_z;
static uint32_t plan_done_state, claw_closed_at;
enum {ALIGN_YAW_CLEARANCE, ALIGN_YAW_MICRO, ALIGN_YAW_MATERIAL};
static uint8_t plan_align_yaw;
static uint16_t event_sequence;

static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
static uint32_t u32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void put16(uint8_t *p,uint16_t n) {p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void put32(uint8_t *p,uint32_t n)
{p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);p[2]=(uint8_t)(n>>16);p[3]=(uint8_t)(n>>24);}
static uint8_t work_known(uint16_t w)
{ return w>=ROUTE_WORK_MATERIAL_1 && w<=ROUTE_WORK_STORAGE_2; }
static uint8_t material_work(void)
{
    return arm_work_status.work_id == ROUTE_WORK_MATERIAL_1 ||
        arm_work_status.work_id == ROUTE_WORK_MATERIAL_2;
}

#if ARM_TRAY_ENABLED
static uint8_t processing_work(void)
{
    return arm_work_status.work_id==ROUTE_WORK_PROCESSING_1 ||
        arm_work_status.work_id==ROUTE_WORK_PROCESSING_2;
}
static uint8_t tray_loading_work(void)
{
    return material_work() || (processing_work() && placed_count==TRAY_CAPACITY && reloaded_count<placed_count);
}
#endif

static void claw_open(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ARM_CLAW_OPEN_COMPARE);
#if ARM_TRAY_ENABLED
    claw_holding=0U;
#endif
}
static void claw_close(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ARM_CLAW_CLOSED_COMPARE);
#if ARM_TRAY_ENABLED
    claw_holding=1U;
#endif
}
static void cancel_claw(void)
{
#if ARM_TRAY_ENABLED
    if(tray_actions || claw_holding) tray_uncertain=1U;
    tray_actions=0;
    if(claw_holding) return;
#endif
    claw_open();
}

static uint8_t origins(void)
{
    uint8_t i;
    for(i=0;i<3U;i++) if(!arm_motor_status[i].homed || arm_motor_status[i].state==ARM_MOTOR_FAULT) return 0U;
    return 1U;
}
static uint8_t axis_target_ok(uint8_t axis,int32_t value)
{
    return value>=arm_motor_config[axis].min_cdeg && value<=arm_motor_config[axis].max_cdeg;
}
static uint8_t pose_ok(uint8_t axis,int32_t value)
{
    return axis_target_ok(axis,value) && (axis!=ARM_YAW || value<=ARM_YAW_MAX_CDEG);
}
static void report(void)
{
    uint8_t out[ARM_STATUS_BYTES+10U]={0xA5U,0x5AU,1U,TYPE_ARM_EVENT},i;
    uint16_t flags=0U;
    put16(out+4,++event_sequence); put16(out+6,ARM_STATUS_BYTES);
    put32(out+8,arm_work_status.route_id); put16(out+12,arm_work_status.point_index);
    put16(out+14,arm_work_status.work_id); put32(out+16,arm_work_status.command_id);
    put32(out+20,arm_work_status.aligned_command_id);
    out[24]=(uint8_t)arm_work_status.state; out[25]=(uint8_t)arm_work_status.fault;
    out[26]=arm_work_status.active_axis; out[27]=arm_work_status.opcode;
    if(origins()) flags|=1U;
    if(arm_work_status.state==ARM_WORK_READY_LEAVE || arm_work_status.state==ARM_WORK_READY) flags|=2U;
    /* Bit 2 marks the external GRIP_DONE handshake, including at material stations. */
    flags|=4U;
    if(ARM_VISION_TEST_MODE) flags|=8U;
#if ARM_TRAY_ENABLED
    flags|=16U;
    if(tray_uncertain) flags|=32U;
    out[30]=tray_count;
    out[31]=destination_slot;
#endif
    if(arm_work_status.state==ARM_WORK_FAULT && arm_work_status.fault==ARM_WORK_MOTOR &&
       arm_work_status.active_axis<ARM_AXIS_COUNT) {
        const volatile ArmCANDiag_t *diag=&arm_can_diag[arm_work_status.active_axis];
        if(diag->fault_code!=ARM_MOTOR_OK) {
            const uint16_t driver_flags=diag->fault_flags;
            /* Bits 6..8: motor fault. Bits 9..12: CAN stage.
             * Bits 13..15: captured driver status bits 0, 2 and 3.
             */
            flags|=(uint16_t)(((diag->fault_code&7U)<<6) | ((diag->fault_stage&15U)<<9) |
                ((driver_flags&1U)<<13) | ((driver_flags&0x0CU)<<12));
        }
    }
    put16(out+28,flags);
    for(i=0;i<3U;i++) {
        put32(out+32+4U*i,(uint32_t)arm_motor_status[i].position_cdeg);
        put32(out+44+4U*i,(uint32_t)arm_work_status.target_cdeg[i]);
    }
    put32(out+56,HAL_GetTick()); put16(out+60,Route_CRC16(out+2,58U));
    if(!RouteUART_Send(out,sizeof(out))) route_status.tx_drops++;
}
static void state(uint32_t value)
{
    arm_work_status.state=value; arm_work_status.tick=HAL_GetTick(); report();
}
static void fail(uint32_t fault,uint8_t axis)
{
    uint8_t was_managed=managed;
    uint32_t id=arm_work_status.route_id; uint16_t index=arm_work_status.point_index;
    arm_work_status.fault=fault; arm_work_status.active_axis=axis;
    plan_count=plan_index=moving=0U; managed=0U;
    ArmCAN_StopAll();
    cancel_claw();
    state(ARM_WORK_FAULT);
    if(!ARM_VISION_TEST_MODE && was_managed && route_status.state==ROUTE_WAIT_WORK)
        Route_CompleteWork(id,index,0U);
}
HAL_StatusTypeDef ArmWork_Init(void)
{
    uint8_t i;
    ArmWorkStatus_t empty={0}; ArmWorkDebug_t debug={0};
    arm_work_status=empty; arm_work_debug=debug;
#if ARM_TRAY_ENABLED
    tray_actions=0;
    tray_count=tray_head=tray_pwm_started=tray_uncertain=0U;
    tray_index=TRAY_CAPACITY; /* Unindexed until safely parked on the first transfer. */
    destination_slot=destination_mask=0U;
#endif
    managed=plan_count=plan_index=moving=origin_index=have_saved=0U;
    highest_command=debug_trigger=event_sequence=0U;
#if ARM_VISION_TEST_MODE
    last_test_session=0U;
#endif
    arm_work_status.active_axis=255U;
    for(i=0;i<3U;i++) {
        arm_motor_config[i].pulses_per_rev=ARM_PULSES_PER_REV;
        arm_motor_config[i].acceleration=ARM_ACCELERATION;
        arm_motor_config[i].tolerance_cdeg=ARM_TOLERANCE_CDEG;
    }
    arm_motor_config[ARM_Z].rpm=ARM_Z_RPM;
    arm_motor_config[ARM_Z].min_cdeg=ARM_Z_MIN_CDEG; arm_motor_config[ARM_Z].max_cdeg=ARM_Z_MAX_CDEG;
    arm_motor_config[ARM_YAW].rpm=ARM_YAW_RPM;
    arm_motor_config[ARM_YAW].min_cdeg=ARM_YAW_MIN_CDEG; arm_motor_config[ARM_YAW].max_cdeg=ARM_YAW_TRAVEL_MAX_CDEG;
    arm_motor_config[ARM_REACH].rpm=ARM_REACH_RPM;
    arm_motor_config[ARM_REACH].min_cdeg=ARM_REACH_MIN_CDEG; arm_motor_config[ARM_REACH].max_cdeg=ARM_REACH_MAX_CDEG;
    claw_open();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) return HAL_ERROR;
    return ArmCAN_Init();
}
static void plan(uint8_t count,const uint8_t *axes,const int32_t *targets,uint32_t after)
{
    uint8_t i;
    plan_count=count; plan_index=moving=0U; plan_align_yaw=ALIGN_YAW_CLEARANCE; plan_done_state=after;
    for(i=0;i<count;i++) {plan_axis[i]=axes[i];plan_target[i]=targets[i];}
}
/* Full-pose yaw moves use clearance; ALIGN micro moves have a separate gate. */
static uint8_t clearance_ok(void)
{
    return pose_ok(ARM_Z,ARM_CLEAR_Z_CDEG) &&
        ARM_CLEAR_Z_CDEG>=ARM_YAW_MIN_Z_CDEG &&
        pose_ok(ARM_REACH,ARM_CLEAR_REACH_CDEG);
}
static uint8_t yaw_clear_now(void)
{
    return clearance_ok() && arm_motor_status[ARM_Z].position_cdeg>=ARM_YAW_MIN_Z_CDEG &&
        ArmCAN_PositionWithinTolerance(ARM_REACH,ARM_CLEAR_REACH_CDEG);
}
static uint8_t full_pose_plan(const int32_t *pose,uint32_t after,uint8_t move_final_z)
{
    uint8_t axes[5]={ARM_Z,ARM_REACH,ARM_YAW,ARM_REACH,ARM_Z};
    uint8_t count=(move_final_z || pose[ARM_Z]!=ARM_CLEAR_Z_CDEG) ? 5U : 4U;
    int32_t targets[5]; uint8_t i;
    if(!clearance_ok()) return 0U;
    targets[0]=ARM_CLEAR_Z_CDEG;targets[1]=ARM_CLEAR_REACH_CDEG;
    targets[2]=pose[ARM_YAW];targets[3]=pose[ARM_REACH];targets[4]=pose[ARM_Z];
    for(i=0;i<count;i++) if(!axis_target_ok(axes[i],targets[i])) return 0U;
    /* Material alignment normally reuses clearance left by a tray operation.
     * An explicit return to the photo origin after an empty grip may use clearance. */
    if(after==ARM_WORK_ALIGNED) {
        if(ArmCAN_PositionWithinTolerance(ARM_REACH,ARM_CLEAR_REACH_CDEG)) {
            for(i=1U;i+1U<count;i++) {axes[i]=axes[i+1U];targets[i]=targets[i+1U];}
            count--;
        } else if(material_work() &&
                  (arm_work_status.state!=ARM_WORK_WAIT_VISION ||
                   arm_work_status.opcode!=CMD_GRIP_ABORT ||
                   pose[ARM_Z] || pose[ARM_YAW] || pose[ARM_REACH])) return 0U;
    }
    for(i=0;i<3U;i++) arm_work_status.target_cdeg[i]=pose[i];
    plan(count,axes,targets,after); return 1U;
}
static uint8_t pose_plan(const int32_t *pose,uint32_t after)
{
    return full_pose_plan(pose,after,1U);
}
static int64_t distance_cdeg(int32_t a,int32_t b)
{
    int64_t d=(int64_t)a-b;return d<0?-d:d;
}
static uint8_t direct_align_yaw_ok(int32_t target,uint8_t yaw_mode)
{
    const int32_t reach=arm_motor_status[ARM_REACH].position_cdeg;
    const int32_t reach_max=arm_motor_config[ARM_REACH].max_cdeg;
    const int32_t yaw=arm_motor_status[ARM_YAW].position_cdeg;
    const int64_t step=distance_cdeg(target,yaw);
    /* At the material camera's upper position, allow a wider correction
     * inside the work-origin sector. Keep the normal micro limit elsewhere.
     */
    if(step>ARM_ALIGN_MICRO_YAW_STEP_CDEG &&
       (yaw_mode!=ALIGN_YAW_MATERIAL || !material_work() || step>ARM_MATERIAL_ALIGN_YAW_STEP_CDEG ||
        distance_cdeg(yaw,0)>ARM_MATERIAL_ALIGN_YAW_LIMIT_CDEG ||
        distance_cdeg(target,0)>ARM_MATERIAL_ALIGN_YAW_LIMIT_CDEG ||
        !ArmCAN_PositionWithinTolerance(ARM_Z,ARM_CLEAR_Z_CDEG))) return 0U;
    /* Use the visual reach move's arrival tolerance at both direct yaw boundaries. */
    return pose_ok(ARM_YAW,target) &&
        arm_motor_status[ARM_Z].position_cdeg>=ARM_ALIGN_MICRO_MIN_Z_CDEG &&
        (reach>=ARM_ALIGN_MICRO_MIN_REACH_CDEG ||
         ArmCAN_PositionWithinCustomTolerance(ARM_REACH,ARM_ALIGN_MICRO_MIN_REACH_CDEG,
                                             ARM_ALIGN_REACH_TOLERANCE_CDEG)) &&
        (reach<=reach_max || ArmCAN_PositionWithinCustomTolerance(ARM_REACH,reach_max,
                                                                 ARM_ALIGN_REACH_TOLERANCE_CDEG));
}
static uint8_t align_plan(const int32_t *pose)
{
    uint8_t axes[3],i,n=0U,move[3];int32_t targets[3];
    const int32_t current_z=arm_motor_status[ARM_Z].position_cdeg;
    uint8_t lift_first,clear_low_reach,retract_first,yaw_mode;
    for(i=0U;i<3U;i++)
        move[i]=(uint8_t)(distance_cdeg(pose[i],arm_motor_status[i].position_cdeg)>arm_motor_config[i].tolerance_cdeg);
    yaw_mode=distance_cdeg(pose[ARM_YAW],arm_motor_status[ARM_YAW].position_cdeg)>ARM_ALIGN_MICRO_YAW_STEP_CDEG ?
        ALIGN_YAW_MATERIAL : ALIGN_YAW_MICRO;
    if(move[ARM_YAW] && !direct_align_yaw_ok(pose[ARM_YAW],yaw_mode)) return pose_plan(pose,ARM_WORK_ALIGNED);

    /* Lift before horizontal alignment and lower after it. When extending
     * from below the visual clearance height, first clear it without a reach detour.
     */
    clear_low_reach=(uint8_t)(move[ARM_REACH] && current_z<ARM_ALIGN_MICRO_MIN_Z_CDEG &&
        (!move[ARM_Z] || pose[ARM_Z]<ARM_ALIGN_MICRO_MIN_Z_CDEG));
    lift_first=(uint8_t)(move[ARM_Z] && pose[ARM_Z]>current_z);
    /* Use the vision-requested retraction to shorten the arm before a
     * wider turn when the requested reach is still in the direct yaw zone.
     */
    retract_first=(uint8_t)(move[ARM_REACH] && move[ARM_YAW] && yaw_mode==ALIGN_YAW_MATERIAL &&
        pose[ARM_REACH]<arm_motor_status[ARM_REACH].position_cdeg &&
        pose[ARM_REACH]>=ARM_ALIGN_MICRO_MIN_REACH_CDEG);
    if(clear_low_reach) {
        if(!pose_ok(ARM_Z,ARM_CLEAR_Z_CDEG)) return 0U;
        axes[n]=ARM_Z;targets[n++]=ARM_CLEAR_Z_CDEG;
    } else if(lift_first) {axes[n]=ARM_Z;targets[n++]=pose[ARM_Z];}
    if(retract_first) {axes[n]=ARM_REACH;targets[n++]=pose[ARM_REACH];}
    if(move[ARM_YAW]) {axes[n]=ARM_YAW;targets[n++]=pose[ARM_YAW];}
    if(move[ARM_REACH] && !retract_first) {axes[n]=ARM_REACH;targets[n++]=pose[ARM_REACH];}
    if(clear_low_reach || (move[ARM_Z] && !lift_first)) {axes[n]=ARM_Z;targets[n++]=pose[ARM_Z];}
    plan(n,axes,targets,ARM_WORK_ALIGNED);plan_align_yaw=yaw_mode;
    for(i=0U;i<3U;i++) arm_work_status.target_cdeg[i]=pose[i];
    if(!move[ARM_Z] && !clear_low_reach) arm_work_status.target_cdeg[ARM_Z]=current_z;
    return 1U;
}
static uint8_t transport_plan(uint32_t after)
{
    int32_t pose[3]={ARM_CLEAR_Z_CDEG,ARM_TRANSPORT_YAW_CDEG,ARM_CLEAR_REACH_CDEG};
    return pose_plan(pose,after);
}

#if ARM_TRAY_ENABLED
static uint8_t tray_slot_at_tail(void)
{
    return (uint8_t)((tray_head+tray_count)%TRAY_CAPACITY);
}
static uint8_t tray_pose_ok(int32_t z)
{
    return clearance_ok() && pose_ok(ARM_Z,z) && axis_target_ok(ARM_YAW,ARM_TRAY_YAW_CDEG) &&
        pose_ok(ARM_REACH,ARM_TRAY_REACH_CDEG) && pose_ok(ARM_YAW,ARM_TRANSPORT_YAW_CDEG);
}
static void tray_begin(const TrayAction_t *actions, ArmWorkState_t next_state)
{
    tray_actions=actions;
    tray_step=tray_waiting=0U;
    state(next_state);
}
static void tray_wait(void)
{
    tray_wait_at=HAL_GetTick();
    tray_waiting=1U;
}
static void tray_select(uint8_t index)
{
    const uint16_t compare[TRAY_CAPACITY]={ARM_TRAY_FIRST_COMPARE,ARM_TRAY_SECOND_COMPARE,ARM_TRAY_THIRD_COMPARE};
    if(tray_index==index) {tray_step++;return;}
    /* Rotate only after all three axes have reached the transport pose. */
    if(!ArmCAN_PositionWithinTolerance(ARM_Z,ARM_CLEAR_Z_CDEG) ||
       !ArmCAN_PositionWithinTolerance(ARM_YAW,ARM_TRANSPORT_YAW_CDEG) ||
       !ArmCAN_PositionWithinTolerance(ARM_REACH,ARM_CLEAR_REACH_CDEG)) {
        fail(ARM_WORK_POSE,255U);return;
    }
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,compare[index]);
    if(!tray_pwm_started && HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2)!=HAL_OK) {
        fail(ARM_WORK_TRAY_PWM,255U);return;
    }
    tray_pwm_started=1U;
    tray_index=index;
    tray_wait();
}
static void tray_task(void)
{
    TrayAction_t action=tray_actions[tray_step];
    uint8_t loading=(uint8_t)(arm_work_status.state==ARM_WORK_TRAY_LOADING);
    uint32_t wait_ms;
    if(tray_waiting) {
        wait_ms=action==TRAY_GRIP ? ARM_CLAW_CLOSE_MS :
            (action==TRAY_RELEASE ? ARM_CLAW_RELEASE_MS : ARM_TRAY_SETTLE_MS);
        if((uint32_t)(HAL_GetTick()-tray_wait_at)<wait_ms) return;
        if(action==TRAY_RELEASE) {
            if(loading) {
                tray_count++;
                if(processing_work()) reloaded_count++;
            } else {
                tray_count--;
                tray_head=tray_count ? (uint8_t)((tray_head+1U)%TRAY_CAPACITY) : 0U;
                destination_mask|=(uint8_t)(1U<<(destination_slot-1U));
                placed_slots[placed_count++]=destination_slot;
            }
        }
        tray_waiting=0U;tray_step++;return;
    }
    switch(action) {
    case TRAY_PARK:
        if(!transport_plan(arm_work_status.state)) {fail(ARM_WORK_POSE,255U);return;}
        break;
    case TRAY_APPROACH: {
        int32_t pose[3]={loading ? ARM_TRAY_PLACE_Z_CDEG : ARM_TRAY_PICK_Z_CDEG,
                        ARM_TRAY_YAW_CDEG,ARM_TRAY_REACH_CDEG};
        /* Stay at the upper clearance height during the final reach into the tray. */
        if(!full_pose_plan(pose,arm_work_status.state,0U)) {fail(ARM_WORK_POSE,255U);return;}
        break;
    }
    case TRAY_MOVE_TO_SLOT:
        if(!pose_plan(destination_pose,arm_work_status.state)) {fail(ARM_WORK_POSE,255U);return;}
        break;
    case TRAY_SELECT:
        tray_select(loading ? tray_slot_at_tail() : tray_head);return;
    case TRAY_ADVANCE:
        if(loading && tray_count<TRAY_CAPACITY) {tray_select(tray_slot_at_tail());return;}
        if(!loading && !tray_count) {tray_select(0U);return;}
        break;
    case TRAY_GRIP:
        claw_close();tray_wait();return;
    case TRAY_RELEASE:
        claw_open();tray_wait();return;
    case TRAY_COMPLETE:
        tray_actions=0;
        state(loading ? ARM_WORK_PICKED : ARM_WORK_PLACED);return;
    }
    tray_step++;
}
#endif

/* Strong application hooks. Route and arm functions all run in RobotTask. */
uint8_t Route_AuxIsBusy(void)
{ return managed || plan_count || arm_work_status.state==ARM_WORK_ZEROING || ArmCAN_Busy(); }
RouteResult_t Route_CheckStart(void)
{
    if(Route_AuxIsBusy()) return ROUTE_BUSY;
#if ARM_TRAY_ENABLED
    if(tray_uncertain) return ROUTE_BAD_STATE;
#endif
    if(!origins() || arm_work_status.state!=ARM_WORK_READY) return ROUTE_BAD_STATE;
    return ROUTE_OK;
}
uint8_t Route_WorkMayComplete(uint32_t id,uint16_t index,uint8_t success)
{
    if(!success || !work_known(route_status.work_id)) return 1U;
    return managed && id==arm_work_status.route_id && index==arm_work_status.point_index &&
        arm_work_status.state==ARM_WORK_READY_LEAVE && !ArmCAN_Busy();
}
static void begin_work(uint32_t id,uint16_t index,uint16_t work)
{
    uint8_t axis;
    if(!work_known(work)) return; /* QR and home keep the existing Pi work gate */
    managed=1U; highest_command=0U; have_saved=0U;
#if ARM_TRAY_ENABLED
    destination_slot=destination_mask=placed_count=reloaded_count=0U;
#endif
    arm_work_status.route_id=id;arm_work_status.point_index=index;arm_work_status.work_id=work;
    arm_work_status.command_id=arm_work_status.aligned_command_id=0U;
    arm_work_status.opcode=0U;arm_work_status.fault=ARM_WORK_NO_FAULT;
    if (!origins()) {
        fail(ARM_WORK_POSE, 255U);
        return;
    }

    /* Hold the current pose until vision sends a target. */
    arm_work_status.active_axis = 255U;
    for (axis = 0U; axis < ARM_AXIS_COUNT; axis++) {
        arm_work_status.target_cdeg[axis] = arm_motor_status[axis].position_cdeg;
    }
    state(ARM_WORK_WAIT_VISION);
}
void Route_OnWork(uint32_t id,uint16_t index,uint16_t work)
{
    if(!ARM_VISION_TEST_MODE) begin_work(id,index,work);
}
static uint8_t context_valid(uint32_t id,uint16_t index,uint16_t work)
{
    if(!managed || id!=arm_work_status.route_id || index!=arm_work_status.point_index ||
       work!=arm_work_status.work_id) return 0U;
#if ARM_VISION_TEST_MODE
    return !Route_IsActive() && !Chassis_IsBusy();
#else
    return route_status.state==ROUTE_WAIT_WORK && id==route_status.route_id &&
        index==route_status.point_index && work==route_status.work_id;
#endif
}
void Route_OnStop(void)
{
    if(!Route_AuxIsBusy() || arm_work_status.state==ARM_WORK_FAULT) return;
    managed=0U;plan_count=plan_index=moving=0U;
    ArmCAN_StopAll();
    cancel_claw();
    arm_work_status.fault=ARM_WORK_CANCELLED;
    state(ARM_WORK_STOPPED);
}

static uint8_t same_command(const uint8_t *p)
{uint8_t i;for(i=0;i<ARM_COMMAND_BYTES;i++)if(saved[i]!=p[i])return 0U;return 1U;}
static void remember(const uint8_t *p)
{
    uint8_t i;
    for(i=0;i<ARM_COMMAND_BYTES;i++)saved[i]=p[i];
    have_saved=1U;highest_command=u32(p+8);
    arm_work_status.command_id=highest_command;arm_work_status.opcode=p[12];
}
static RouteResult_t zero(void)
{
    if(Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy()) return ROUTE_BUSY;
    managed=0U;origin_index=0U;
    arm_work_status.route_id=0U;arm_work_status.point_index=arm_work_status.work_id=0U;
    arm_work_status.aligned_command_id=0U;arm_work_status.fault=0U;
    arm_work_status.active_axis=ARM_Z;
    if(!ArmCAN_RecordOrigin(ARM_Z)) return ROUTE_BUSY;
    state(ARM_WORK_ZEROING); return ROUTE_OK;
}
RouteResult_t Route_PrepareStartPose(void)
{
    uint8_t i;
    RouteResult_t result=zero();
    if(result!=ROUTE_OK) return result;
    /* A physical key confirms a NEW manual pose, even after debug tests.
     * Never reuse a stale homed bit or a previous Pi ZERO command ID. */
    for(i=0U;i<ARM_AXIS_COUNT;i++) arm_motor_status[i].homed=0U;
    have_saved=0U;highest_command=0U;
    arm_work_status.command_id=0U;arm_work_status.opcode=CMD_ZERO;
    return ROUTE_OK;
}
RouteResult_t Route_OnAuxCommand(uint8_t type,const uint8_t *p,uint16_t length)
{
    uint32_t id,command,reference; uint16_t index,work;
    uint8_t opcode,i; int32_t pose[3]; int64_t lower;
    if(type==TYPE_ARM_STATUS) {
        if(length) return ROUTE_BAD_LENGTH;
        report();return ROUTE_OK;
    }
    if(type!=TYPE_ARM_COMMAND) return ROUTE_BAD_TYPE;
    if(length!=ARM_COMMAND_BYTES) return ROUTE_BAD_LENGTH;
    id=u32(p);index=u16(p+4);work=u16(p+6);command=u32(p+8);opcode=p[12];reference=u32(p+28);
    if(!command || u16(p+14)) return ROUTE_BAD_VALUE;
    for(i=0;i<3U;i++)pose[i]=(int32_t)u32(p+16+4U*i);
    if(opcode<CMD_ZERO || opcode>CMD_GRIP_ABORT) return ROUTE_BAD_VALUE;
    if(opcode==CMD_TRAY_PLACE || opcode==CMD_TRAY_RELOAD_PICK) {
        if(p[13]<1U || p[13]>3U) return ROUTE_BAD_VALUE;
        if(opcode==CMD_TRAY_RELOAD_PICK && (pose[ARM_YAW] || pose[ARM_REACH])) return ROUTE_BAD_VALUE;
    } else if(p[13] || (opcode!=CMD_ALIGN && (pose[0] || pose[1] || pose[2]))) return ROUTE_BAD_VALUE;
    if(opcode!=CMD_PICK && opcode!=CMD_GRIP_DONE &&
       opcode!=CMD_TRAY_RELOAD_PICK && opcode!=CMD_GRIP_ABORT && reference) return ROUTE_BAD_VALUE;
    if(opcode==CMD_TEST_BEGIN) {
        /* New opcode is deliberately unavailable in the complete-vehicle build.
         * route_id is a monotonic nonzero test session, index=0, command=1. */
#if !ARM_VISION_TEST_MODE
        return ROUTE_BAD_STATE;
#else
        if(!id || index || command!=1U || !work_known(work)) return ROUTE_BAD_VALUE;
        if(id<=last_test_session) {
            if(id==last_test_session && context_valid(id,index,work)) {report();return ROUTE_OK;}
            return ROUTE_CONFLICT;
        }
        if(Route_IsActive() || Chassis_IsBusy() || ArmCAN_Busy() || plan_count)
            return ROUTE_BUSY;
#if ARM_TRAY_ENABLED
        if(tray_uncertain) return ROUTE_BAD_STATE;
#endif
        if(!origins() || (arm_work_status.state!=ARM_WORK_READY &&
           arm_work_status.state!=ARM_WORK_READY_LEAVE)) return ROUTE_BAD_STATE;
        last_test_session=id;
        begin_work(id,index,work);
        remember(p);report();return ROUTE_OK;
#endif
    }
    /* Validate identity before matching duplicate commands. A command from an
     * earlier station must never operate on the new station's object. */
    if(opcode==CMD_ZERO) {
        if(id || index || work) return ROUTE_BAD_ID;
        if(have_saved && saved[12]==CMD_ZERO && u32(saved+8)==command) {
            if(!same_command(p))return ROUTE_CONFLICT;
            report();return ROUTE_OK;
        }
    } else {
        if(!context_valid(id,index,work))
            return ROUTE_BAD_STATE;
        if(have_saved && command==highest_command) {
            if(!same_command(p))return ROUTE_CONFLICT;
            report();return ROUTE_OK;
        }
        if(command<=highest_command) return ROUTE_CONFLICT;
    }
    if(opcode==CMD_ZERO) {
        RouteResult_t result=zero();
        if(result!=ROUTE_OK)return result;
        remember(p);report();return ROUTE_OK;
    }
    if(!origins()) return ROUTE_BAD_STATE;
    if(opcode==CMD_TRAY_PLACE) {
#if ARM_TRAY_ENABLED
        uint8_t slot=p[13];
        if(material_work() || !tray_count || tray_uncertain ||
           (arm_work_status.state!=ARM_WORK_WAIT_VISION && arm_work_status.state!=ARM_WORK_PLACED)) return ROUTE_BAD_STATE;
        if(destination_mask & (1U<<(slot-1U))) return ROUTE_CONFLICT;
        if(!tray_pose_ok(ARM_TRAY_PICK_Z_CDEG)) return ROUTE_BAD_VALUE;
        for(i=0U;i<3U;i++) if(!pose_ok(i,pose[i])) return ROUTE_BAD_VALUE;
        for(i=0U;i<3U;i++) destination_pose[i]=pose[i];
        destination_slot=slot;
        remember(p);tray_begin(tray_unload_actions,ARM_WORK_TRAY_UNLOADING);
        return ROUTE_OK;
#else
        return ROUTE_BAD_STATE;
#endif
    }
#if ARM_TRAY_ENABLED
    if(opcode!=CMD_FINISH && !tray_loading_work()) return ROUTE_BAD_STATE;
#else
    if(opcode==CMD_TRAY_RELOAD_PICK) return ROUTE_BAD_STATE;
#endif
    if(opcode==CMD_ALIGN) {
        if(arm_work_status.state!=ARM_WORK_WAIT_VISION && arm_work_status.state!=ARM_WORK_ALIGNED &&
           arm_work_status.state!=ARM_WORK_PICKED && arm_work_status.state!=ARM_WORK_PLACED) return ROUTE_BAD_STATE;
        for(i=0;i<3U;i++) if(!pose_ok(i,pose[i]))return ROUTE_BAD_VALUE;
        if(!align_plan(pose))return ROUTE_BAD_VALUE;
#if !ARM_TRAY_ENABLED
        if (material_work() && arm_work_status.state == ARM_WORK_PICKED) claw_open();
#endif
        remember(p);arm_work_status.aligned_command_id=0U;state(ARM_WORK_ALIGNING);
        if(!plan_count) {
            arm_work_status.aligned_command_id=command;
            arm_work_status.active_axis=255U;state(ARM_WORK_ALIGNED);
        }
    } else if(opcode==CMD_PICK || opcode==CMD_TRAY_RELOAD_PICK) {
        int32_t lift_z;
        const int32_t z_min = arm_motor_config[ARM_Z].min_cdeg;
        const int32_t z_max = arm_motor_config[ARM_Z].max_cdeg;
        const uint16_t tolerance = arm_motor_config[ARM_Z].tolerance_cdeg;
        if(arm_work_status.state!=ARM_WORK_ALIGNED || !reference ||
           reference!=arm_work_status.aligned_command_id) return ROUTE_BAD_STATE;

#if ARM_TRAY_ENABLED
        if(tray_count>=TRAY_CAPACITY || tray_uncertain ||
           (opcode==CMD_TRAY_RELOAD_PICK)!=processing_work()) return ROUTE_BAD_STATE;
        if(opcode==CMD_TRAY_RELOAD_PICK && p[13]!=placed_slots[reloaded_count]) return ROUTE_CONFLICT;
        if(!tray_pose_ok(ARM_TRAY_PLACE_Z_CDEG)) return ROUTE_BAD_VALUE;
#endif
        /* Settled encoder feedback can lie just outside the legal target range. */
        lift_z = arm_motor_status[ARM_Z].position_cdeg;
        if (lift_z > z_max) {
            if (distance_cdeg(lift_z, z_max) > tolerance) return ROUTE_BAD_VALUE;
            lift_z = z_max;
        } else if (lift_z < z_min) {
            if (distance_cdeg(lift_z, z_min) > tolerance) return ROUTE_BAD_VALUE;
            lift_z = z_min;
        }

        lower = (int64_t)lift_z + ARM_Z_DOWN_SIGN * ARM_PICK_DROP_CDEG;
#if ARM_TRAY_ENABLED
        if(opcode==CMD_TRAY_RELOAD_PICK) {
            lower=pose[ARM_Z];
            if(lower>=lift_z) return ROUTE_BAD_VALUE;
        }
#endif
        if (lower < z_min || lower > z_max) return ROUTE_BAD_VALUE;
        aligned_z = lift_z;
        {uint8_t axis=ARM_Z;int32_t target=(int32_t)lower;plan(1U,&axis,&target,ARM_WORK_WAIT_GRIP);}
        remember(p);arm_work_status.target_cdeg[ARM_Z]=(int32_t)lower;state(ARM_WORK_LOWERING);
    } else if(opcode==CMD_GRIP_DONE || opcode==CMD_GRIP_ABORT) {
        uint8_t axis=ARM_Z;
        uint32_t after=ARM_WORK_PICKED;
        if(arm_work_status.state!=ARM_WORK_WAIT_GRIP || !reference ||
           reference!=arm_work_status.aligned_command_id)return ROUTE_BAD_STATE;
        if(opcode==CMD_GRIP_ABORT) {
            claw_open();
            arm_work_status.aligned_command_id=0U;
            /* At photo height, wait for vision to send the return-to-origin ALIGN. */
            after=ARM_WORK_WAIT_VISION;
        }
        plan(1U,&axis,&aligned_z,after);
        remember(p);arm_work_status.target_cdeg[ARM_Z]=aligned_z;state(ARM_WORK_LIFTING);
    } else {
        if(arm_work_status.state!=ARM_WORK_WAIT_VISION && arm_work_status.state!=ARM_WORK_ALIGNED &&
           arm_work_status.state!=ARM_WORK_PICKED && arm_work_status.state!=ARM_WORK_PLACED) return ROUTE_BAD_STATE;
#if ARM_TRAY_ENABLED
        if(material_work() ? tray_count!=TRAY_CAPACITY :
           (processing_work() ? reloaded_count!=TRAY_CAPACITY : placed_count!=TRAY_CAPACITY))
            return ROUTE_BAD_STATE;
#endif
        if(!transport_plan(ARM_WORK_READY_LEAVE))return ROUTE_BAD_VALUE;
        claw_open();
        remember(p);state(ARM_WORK_PARKING);
    }
    return ROUTE_OK;
}

static RouteResult_t debug_move(uint32_t selected,int32_t target,uint8_t return_zero)
{
    uint8_t axis,i;
    int32_t pose[3]={0,0,0};
    if(Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy()) return ROUTE_BUSY;
    if(!origins() || (arm_work_status.state!=ARM_WORK_READY &&
       arm_work_status.state!=ARM_WORK_DEBUG_HOLD)) return ROUTE_BAD_STATE;
    if(return_zero) {
        if(!pose_plan(pose,ARM_WORK_READY)) return ROUTE_BAD_VALUE;
        state(ARM_WORK_PARKING);
    } else {
        if(selected>=ARM_AXIS_COUNT) return ROUTE_BAD_VALUE;
        axis=(uint8_t)selected;
        if(!pose_ok(axis,target)) return ROUTE_BAD_VALUE;
        for(i=0;i<ARM_AXIS_COUNT;i++)
            arm_work_status.target_cdeg[i]=arm_motor_status[i].position_cdeg;
        arm_work_status.target_cdeg[axis]=target;
        if(axis==ARM_YAW) {
            const uint8_t axes[3]={ARM_Z,ARM_REACH,ARM_YAW};
            int32_t targets[3]={ARM_CLEAR_Z_CDEG,ARM_CLEAR_REACH_CDEG,target};
            if(!clearance_ok()) return ROUTE_BAD_VALUE;
            arm_work_status.target_cdeg[ARM_Z]=ARM_CLEAR_Z_CDEG;
            arm_work_status.target_cdeg[ARM_REACH]=ARM_CLEAR_REACH_CDEG;
            plan(3U,axes,targets,ARM_WORK_DEBUG_HOLD);
        } else plan(1U,&axis,&target,ARM_WORK_DEBUG_HOLD);
        state(ARM_WORK_DEBUG_MOVING);
    }
    return ROUTE_OK;
}

void ArmWork_Task(void)
{
    uint8_t axis;
    uint16_t tolerance=0U;
    ArmCAN_Task();
    if(arm_work_debug.trigger!=debug_trigger) {
        debug_trigger=arm_work_debug.trigger;
        if(arm_work_debug.command==0U) {Route_Stop();arm_work_debug.result=ROUTE_OK;}
        else if(arm_work_debug.command==1U) {
            arm_work_debug.result=zero();
            if(arm_work_debug.result==ROUTE_OK) {have_saved=0U;highest_command=0U;}
        } else if(arm_work_debug.command==2U || arm_work_debug.command==3U) {
            arm_work_debug.result=debug_move(arm_work_debug.axis,arm_work_debug.target_cdeg,
                (uint8_t)(arm_work_debug.command==3U));
        } else if(arm_work_debug.command==4U) {
            if(arm_work_debug.axis>=ARM_AXIS_COUNT) arm_work_debug.result=ROUTE_BAD_VALUE;
            else if(Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy())
                arm_work_debug.result=ROUTE_BUSY;
            else arm_work_debug.result=ArmCAN_Probe((uint8_t)arm_work_debug.axis)?ROUTE_OK:ROUTE_BUSY;
#if ARM_TRAY_ENABLED
        } else if(arm_work_debug.command==5U) {
            if(Route_IsActive() || Chassis_IsBusy() || Route_AuxIsBusy()) arm_work_debug.result=ROUTE_BUSY;
            else {
                tray_count=tray_head=tray_uncertain=destination_slot=0U;
                tray_index=TRAY_CAPACITY;
                claw_open();
                arm_work_debug.result=ROUTE_OK;
                report();
            }
#endif
        } else arm_work_debug.result=ROUTE_BAD_VALUE;
    }
    if(managed && !context_valid(arm_work_status.route_id,arm_work_status.point_index,
       arm_work_status.work_id)) {
        if(arm_work_status.state==ARM_WORK_READY_LEAVE && !ArmCAN_Busy()) {
            managed=0U;arm_work_status.active_axis=255U;state(ARM_WORK_READY);
        } else {fail(ARM_WORK_CONTEXT,255U);return;}
    }
    if((managed || plan_count) && !origins()) {
        for(axis=0U;axis<3U;axis++)
            if(!arm_motor_status[axis].homed || arm_motor_status[axis].state==ARM_MOTOR_FAULT) break;
        fail(ARM_WORK_MOTOR,axis);return;
    }
    if(arm_work_status.state==ARM_WORK_ZEROING) {
        axis=origin_index;
        if(arm_motor_status[axis].state==ARM_MOTOR_FAULT) {fail(ARM_WORK_MOTOR,axis);return;}
        if(ArmCAN_Busy())return;
        if(!arm_motor_status[axis].homed) {fail(ARM_WORK_MOTOR,axis);return;}
        origin_index++;
        if(origin_index<3U) {
            arm_work_status.active_axis=origin_index;
            if(!ArmCAN_RecordOrigin(origin_index))fail(ARM_WORK_MOTOR,origin_index);
        } else {arm_work_status.active_axis=255U;state(ARM_WORK_READY);}
        return;
    }
    if(!plan_count) {
#if ARM_TRAY_ENABLED
        if(tray_actions) tray_task();
#endif
        return;
    }
    if (arm_work_status.state == ARM_WORK_LIFTING &&
#if ARM_TRAY_ENABLED
        claw_holding &&
#else
        material_work() && plan_done_state==ARM_WORK_PICKED &&
#endif
        (uint32_t)(HAL_GetTick() - claw_closed_at) < ARM_CLAW_CLOSE_MS) return;
    axis=plan_axis[plan_index];arm_work_status.active_axis=axis;
    if(arm_motor_status[axis].state==ARM_MOTOR_FAULT) {fail(ARM_WORK_MOTOR,axis);return;}
    if(moving) {
        if(ArmCAN_Busy())return;
        if(arm_motor_status[axis].state!=ARM_MOTOR_DONE) {fail(ARM_WORK_MOTOR,axis);return;}
        moving=0U;plan_index++;
    }
    if(plan_index==plan_count) {
        plan_count=0U;arm_work_status.active_axis=255U;
#if ARM_TRAY_ENABLED
        if(tray_actions) return;
        if(plan_done_state==ARM_WORK_PICKED) {
            tray_begin(tray_load_actions,ARM_WORK_TRAY_LOADING);return;
        }
#endif
        if(plan_done_state==ARM_WORK_ALIGNED)arm_work_status.aligned_command_id=arm_work_status.command_id;
#if ARM_TRAY_ENABLED
        if (plan_done_state == ARM_WORK_WAIT_GRIP) {
#else
        if (plan_done_state == ARM_WORK_WAIT_GRIP && material_work()) {
#endif
            claw_close();
            claw_closed_at = HAL_GetTick();
        }
        state(plan_done_state);return;
    }
    axis=plan_axis[plan_index];arm_work_status.active_axis=axis;
    if(axis==ARM_YAW) {
        uint8_t allowed=plan_align_yaw==ALIGN_YAW_CLEARANCE ? yaw_clear_now() :
            direct_align_yaw_ok(plan_target[plan_index],plan_align_yaw);
#if ARM_TRAY_ENABLED
        if(tray_actions && !ArmCAN_PositionWithinTolerance(ARM_Z,ARM_CLEAR_Z_CDEG)) allowed=0U;
#endif
        if(!allowed) {fail(ARM_WORK_POSE,axis);return;}
    }
    if(axis==ARM_REACH && plan_done_state==ARM_WORK_ALIGNED &&
       plan_target[plan_index]!=ARM_CLEAR_REACH_CDEG)
        tolerance=ARM_ALIGN_REACH_TOLERANCE_CDEG;
    if(!ArmCAN_MoveWithTolerance(axis,plan_target[plan_index],tolerance)) {fail(ARM_WORK_POSE,axis);return;}
    moving=1U;
}
