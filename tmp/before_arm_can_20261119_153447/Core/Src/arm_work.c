#include "arm_work.h"
#include "arm_config.h"
#include "route_uart.h"
#include "chassis_motion.h"

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

volatile ArmWorkStatus_t arm_work_status;
volatile ArmWorkDebug_t arm_work_debug;
static uint8_t managed, plan_count, plan_index, moving, origin_index;
static uint8_t plan_axis[5], saved[ARM_COMMAND_BYTES], have_saved;
static uint32_t highest_command, debug_trigger;
static int32_t plan_target[5], aligned_z;
static uint32_t plan_done_state;
static uint16_t event_sequence;

static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
static uint32_t u32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void put16(uint8_t *p,uint16_t n) {p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void put32(uint8_t *p,uint32_t n)
{p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);p[2]=(uint8_t)(n>>16);p[3]=(uint8_t)(n>>24);}
static uint8_t work_known(uint16_t w)
{ return w>=ROUTE_WORK_MATERIAL_1 && w<=ROUTE_WORK_STORAGE_2; }
static uint8_t origins(void)
{
    uint8_t i;
    for(i=0;i<3U;i++) if(!arm_motor_status[i].homed || arm_motor_status[i].state==ARM_MOTOR_FAULT) return 0U;
    return 1U;
}
static uint8_t pose_ok(uint8_t axis,int32_t value)
{
    return value>=arm_motor_config[axis].min_cdeg && value<=arm_motor_config[axis].max_cdeg;
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
    /* bit2 permanently marks external claw handling; never claim grasp sensing. */
    flags|=4U; put16(out+28,flags);
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
    ArmUART_StopAll(); state(ARM_WORK_FAULT);
    if(was_managed && route_status.state==ROUTE_WAIT_WORK)
        Route_CompleteWork(id,index,0U);
}
HAL_StatusTypeDef ArmWork_Init(void)
{
    uint8_t i;
    ArmWorkStatus_t empty={0}; ArmWorkDebug_t debug={0};
    arm_work_status=empty; arm_work_debug=debug;
    managed=plan_count=plan_index=moving=origin_index=have_saved=0U;
    highest_command=debug_trigger=event_sequence=0U;
    arm_work_status.active_axis=255U;
    for(i=0;i<3U;i++) {
        arm_motor_config[i].pulses_per_rev=ARM_PULSES_PER_REV;
        arm_motor_config[i].acceleration=ARM_ACCELERATION;
        arm_motor_config[i].tolerance_cdeg=ARM_TOLERANCE_CDEG;
    }
    arm_motor_config[ARM_Z].rpm=ARM_Z_RPM;
    arm_motor_config[ARM_Z].min_cdeg=ARM_Z_MIN_CDEG; arm_motor_config[ARM_Z].max_cdeg=ARM_Z_MAX_CDEG;
    arm_motor_config[ARM_YAW].rpm=ARM_YAW_RPM;
    arm_motor_config[ARM_YAW].min_cdeg=ARM_YAW_MIN_CDEG; arm_motor_config[ARM_YAW].max_cdeg=ARM_YAW_MAX_CDEG;
    arm_motor_config[ARM_REACH].rpm=ARM_REACH_RPM;
    arm_motor_config[ARM_REACH].min_cdeg=ARM_REACH_MIN_CDEG; arm_motor_config[ARM_REACH].max_cdeg=ARM_REACH_MAX_CDEG;
    return ArmUART_Init();
}
static void plan(uint8_t count,const uint8_t *axes,const int32_t *targets,uint32_t after)
{
    uint8_t i;
    plan_count=count; plan_index=moving=0U; plan_done_state=after;
    for(i=0;i<count;i++) {plan_axis[i]=axes[i];plan_target[i]=targets[i];}
}
static uint8_t pose_plan(const int32_t *pose,uint32_t after)
{
    const uint8_t axes[5]={ARM_Z,ARM_REACH,ARM_YAW,ARM_REACH,ARM_Z};
    int32_t targets[5]; uint8_t i;
    targets[0]=ARM_CLEAR_Z_CDEG;targets[1]=ARM_CLEAR_REACH_CDEG;
    targets[2]=pose[ARM_YAW];targets[3]=pose[ARM_REACH];targets[4]=pose[ARM_Z];
    for(i=0;i<5U;i++) if(!pose_ok(axes[i],targets[i])) return 0U;
    for(i=0;i<3U;i++) arm_work_status.target_cdeg[i]=pose[i];
    plan(5U,axes,targets,after); return 1U;
}
static uint8_t transport_plan(uint32_t after)
{
    int32_t pose[3]={ARM_CLEAR_Z_CDEG,ARM_TRANSPORT_YAW_CDEG,ARM_CLEAR_REACH_CDEG};
    return pose_plan(pose,after);
}

/* Strong application hooks. Route and arm functions all run in RobotTask. */
uint8_t Route_AuxIsBusy(void)
{ return managed || plan_count || arm_work_status.state==ARM_WORK_ZEROING || ArmUART_Busy(); }
RouteResult_t Route_CheckStart(void)
{
    if(Route_AuxIsBusy()) return ROUTE_BUSY;
    if(!origins() || arm_work_status.state!=ARM_WORK_READY) return ROUTE_BAD_STATE;
    return ROUTE_OK;
}
uint8_t Route_WorkMayComplete(uint32_t id,uint16_t index,uint8_t success)
{
    if(!success || !work_known(route_status.work_id)) return 1U;
    return managed && id==arm_work_status.route_id && index==arm_work_status.point_index &&
        arm_work_status.state==ARM_WORK_READY_LEAVE && !ArmUART_Busy();
}
void Route_OnWork(uint32_t id,uint16_t index,uint16_t work)
{
    int32_t pose[3]={ARM_PHOTO_Z_CDEG,ARM_PHOTO_YAW_CDEG,ARM_PHOTO_REACH_CDEG};
    if(!work_known(work)) return; /* QR and home keep the existing Pi work gate */
    managed=1U; highest_command=0U; have_saved=0U;
    arm_work_status.route_id=id;arm_work_status.point_index=index;arm_work_status.work_id=work;
    arm_work_status.command_id=arm_work_status.aligned_command_id=0U;
    arm_work_status.opcode=0U;arm_work_status.fault=ARM_WORK_NO_FAULT;
    if(!origins() || !pose_plan(pose,ARM_WORK_WAIT_VISION)) {fail(ARM_WORK_POSE,255U);return;}
    state(ARM_WORK_PREPARING);
}
void Route_OnStop(void)
{
    if(!Route_AuxIsBusy() || arm_work_status.state==ARM_WORK_FAULT) return;
    managed=0U;plan_count=plan_index=moving=0U;
    ArmUART_StopAll();arm_work_status.fault=ARM_WORK_CANCELLED;
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
    if(!ArmUART_RecordOrigin(ARM_Z)) return ROUTE_BUSY;
    state(ARM_WORK_ZEROING); return ROUTE_OK;
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
    if(!command || p[13] || u16(p+14)) return ROUTE_BAD_VALUE;
    for(i=0;i<3U;i++)pose[i]=(int32_t)u32(p+16+4U*i);
    if(opcode<CMD_ZERO || opcode>CMD_FINISH) return ROUTE_BAD_VALUE;
    if(opcode!=CMD_ALIGN && (pose[0] || pose[1] || pose[2])) return ROUTE_BAD_VALUE;
    if(opcode!=CMD_PICK && opcode!=CMD_GRIP_DONE && reference) return ROUTE_BAD_VALUE;
    /* Validate identity before matching duplicate commands. A command from an
     * earlier station must never operate on the new station's object. */
    if(opcode==CMD_ZERO) {
        if(id || index || work) return ROUTE_BAD_ID;
        if(have_saved && saved[12]==CMD_ZERO && u32(saved+8)==command) {
            if(!same_command(p))return ROUTE_CONFLICT;
            report();return ROUTE_OK;
        }
    } else {
        if(!managed || route_status.state!=ROUTE_WAIT_WORK || id!=route_status.route_id ||
           id!=arm_work_status.route_id || index!=arm_work_status.point_index ||
           index!=route_status.point_index || work!=arm_work_status.work_id || work!=route_status.work_id)
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
    if(opcode==CMD_ALIGN) {
        if(arm_work_status.state!=ARM_WORK_WAIT_VISION && arm_work_status.state!=ARM_WORK_ALIGNED &&
           arm_work_status.state!=ARM_WORK_PICKED) return ROUTE_BAD_STATE;
        for(i=0;i<3U;i++) if(!pose_ok(i,pose[i]))return ROUTE_BAD_VALUE;
        if(!pose_plan(pose,ARM_WORK_ALIGNED))return ROUTE_BAD_VALUE;
        remember(p);arm_work_status.aligned_command_id=0U;state(ARM_WORK_ALIGNING);
    } else if(opcode==CMD_PICK) {
        if(arm_work_status.state!=ARM_WORK_ALIGNED || !reference ||
           reference!=arm_work_status.aligned_command_id) return ROUTE_BAD_STATE;
        aligned_z=arm_motor_status[ARM_Z].position_cdeg;
        lower=(int64_t)aligned_z+ARM_Z_DOWN_SIGN*ARM_PICK_DROP_CDEG;
        if(lower<-2147483647LL || lower>2147483647LL || !pose_ok(ARM_Z,(int32_t)lower))return ROUTE_BAD_VALUE;
        {uint8_t axis=ARM_Z;int32_t target=(int32_t)lower;plan(1U,&axis,&target,ARM_WORK_WAIT_GRIP);}
        remember(p);arm_work_status.target_cdeg[ARM_Z]=(int32_t)lower;state(ARM_WORK_LOWERING);
    } else if(opcode==CMD_GRIP_DONE) {
        /* Explicit external confirmation, not a servo command or grip sensor. */
        if(arm_work_status.state!=ARM_WORK_WAIT_GRIP || !reference ||
           reference!=arm_work_status.aligned_command_id)return ROUTE_BAD_STATE;
        {uint8_t axis=ARM_Z;plan(1U,&axis,&aligned_z,ARM_WORK_PICKED);}
        remember(p);arm_work_status.target_cdeg[ARM_Z]=aligned_z;state(ARM_WORK_LIFTING);
    } else {
        if(arm_work_status.state!=ARM_WORK_WAIT_VISION && arm_work_status.state!=ARM_WORK_ALIGNED &&
           arm_work_status.state!=ARM_WORK_PICKED) return ROUTE_BAD_STATE;
        if(!transport_plan(ARM_WORK_READY_LEAVE))return ROUTE_BAD_VALUE;
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
        plan(1U,&axis,&target,ARM_WORK_DEBUG_HOLD);
        state(ARM_WORK_DEBUG_MOVING);
    }
    return ROUTE_OK;
}

void ArmWork_Task(void)
{
    uint8_t axis;
    ArmUART_Task();
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
            else arm_work_debug.result=ArmUART_Probe((uint8_t)arm_work_debug.axis)?ROUTE_OK:ROUTE_BUSY;
        } else arm_work_debug.result=ROUTE_BAD_VALUE;
    }
    if(managed && (route_status.state!=ROUTE_WAIT_WORK ||
       route_status.route_id!=arm_work_status.route_id || route_status.point_index!=arm_work_status.point_index)) {
        if(arm_work_status.state==ARM_WORK_READY_LEAVE && !ArmUART_Busy()) {
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
        if(ArmUART_Busy())return;
        if(!arm_motor_status[axis].homed) {fail(ARM_WORK_MOTOR,axis);return;}
        origin_index++;
        if(origin_index<3U) {
            arm_work_status.active_axis=origin_index;
            if(!ArmUART_RecordOrigin(origin_index))fail(ARM_WORK_MOTOR,origin_index);
        } else {arm_work_status.active_axis=255U;state(ARM_WORK_READY);}
        return;
    }
    if(!plan_count)return;
    axis=plan_axis[plan_index];arm_work_status.active_axis=axis;
    if(arm_motor_status[axis].state==ARM_MOTOR_FAULT) {fail(ARM_WORK_MOTOR,axis);return;}
    if(moving) {
        if(ArmUART_Busy())return;
        if(arm_motor_status[axis].state!=ARM_MOTOR_DONE) {fail(ARM_WORK_MOTOR,axis);return;}
        moving=0U;plan_index++;
    }
    if(plan_index==plan_count) {
        plan_count=0U;arm_work_status.active_axis=255U;
        if(plan_done_state==ARM_WORK_ALIGNED)arm_work_status.aligned_command_id=arm_work_status.command_id;
        state(plan_done_state);return;
    }
    axis=plan_axis[plan_index];arm_work_status.active_axis=axis;
    if(!ArmUART_Move(axis,plan_target[plan_index])) {fail(ARM_WORK_POSE,axis);return;}
    moving=1U;
}
