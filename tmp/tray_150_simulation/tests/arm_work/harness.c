#include "arm_work.h"
#include "arm_config.h"
#include "route_keys.h"
#include "chassis_debug.h"
#include "tim.h"

TIM_HandleTypeDef htim3;
static uint32_t pwm_compare, pwm_close_count, pwm_bad_channel, tick;
#if ARM_TRAY_ENABLED
static uint32_t tray_pwm_log[16], tray_pwm_writes, tray_pwm_starts, tray_changed_at;
static uint32_t tray_timing_errors, tray_pick_index[6], tray_picks, tray_z_moves, z_moves_at_tray_yaw;
static int32_t placed_z[6], placed_yaw[6], placed_reach[6];
static uint32_t placements;
#endif
void mock_tim_set_compare(TIM_HandleTypeDef *timer,uint32_t channel,uint32_t value)
{
#if ARM_TRAY_ENABLED
    if(timer==&htim3 && channel==TIM_CHANNEL_2) {
        if(tray_pwm_writes<16) tray_pwm_log[tray_pwm_writes]=value;
        tray_pwm_writes++;
        tray_changed_at=tick;
        if(ArmCAN_Busy() || !ArmCAN_PositionWithinTolerance(ARM_Z,ARM_CLEAR_Z_CDEG) ||
           !ArmCAN_PositionWithinTolerance(ARM_YAW,ARM_TRANSPORT_YAW_CDEG) ||
           !ArmCAN_PositionWithinTolerance(ARM_REACH,ARM_CLEAR_REACH_CDEG)) tray_timing_errors++;
        return;
    }
    if(pwm_compare==ARM_CLAW_OPEN_COMPARE && value==ARM_CLAW_CLOSED_COMPARE &&
       arm_work_status.state==ARM_WORK_TRAY_UNLOADING && tray_picks<6U) {
        tray_pick_index[tray_picks++]=tray_pwm_log[tray_pwm_writes-1U];
        if(!ArmCAN_PositionWithinTolerance(ARM_Z,ARM_TRAY_PICK_Z_CDEG) ||
           !ArmCAN_PositionWithinTolerance(ARM_YAW,45000L) ||
           !ArmCAN_PositionWithinTolerance(ARM_REACH,-31830L) ||
           tray_z_moves!=z_moves_at_tray_yaw) tray_timing_errors++;
    }
    if(pwm_compare==ARM_CLAW_CLOSED_COMPARE && value==ARM_CLAW_OPEN_COMPARE) {
        if(arm_work_status.state==ARM_WORK_TRAY_LOADING &&
           (!ArmCAN_PositionWithinTolerance(ARM_Z,ARM_TRAY_PLACE_Z_CDEG) ||
            !ArmCAN_PositionWithinTolerance(ARM_YAW,45000L) ||
            !ArmCAN_PositionWithinTolerance(ARM_REACH,-31830L) ||
            tray_z_moves!=z_moves_at_tray_yaw)) tray_timing_errors++;
        if(arm_work_status.state==ARM_WORK_TRAY_UNLOADING && placements<6U) {
            placed_z[placements]=arm_motor_status[ARM_Z].position_cdeg;
            placed_yaw[placements]=arm_motor_status[ARM_YAW].position_cdeg;
            placed_reach[placements++]=arm_motor_status[ARM_REACH].position_cdeg;
        }
    }
#endif
    if (timer!=&htim3 || channel!=TIM_CHANNEL_1) pwm_bad_channel++;
    pwm_compare=value;
    if (value==ARM_CLAW_CLOSED_COMPARE) pwm_close_count++;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer,uint32_t channel)
{
#if ARM_TRAY_ENABLED
    if(timer==&htim3 && channel==TIM_CHANNEL_2) {
        tray_pwm_starts++;
        return tray_pwm_writes==1 && tray_pwm_log[0]==ARM_TRAY_FIRST_COMPARE ? HAL_OK : HAL_ERROR;
    }
#endif
    return timer==&htim3 && channel==TIM_CHANNEL_1 &&
        pwm_compare==ARM_CLAW_OPEN_COMPARE ? HAL_OK : HAL_ERROR;
}
CAN_HandleTypeDef hcan={0};
volatile HAL_StatusTypeDef can_last_tx_status=HAL_OK;
static uint32_t mask,tx_count,stop_count,move_count;
static uint8_t response[3][8],response_n[3];
static uint32_t response_at[3];
static int32_t pos[3],target[3];
static uint8_t moving[3],drop_function,reject_function,stop_reject,tx_fail;
static uint8_t flags=1;
volatile ChassisDebug_t chassis_debug;
volatile uint32_t key0_level,key0_press_count;
static uint8_t key_levels[2]={1,1};
static uint32_t area_signals,selected_area,motion_inhibit;
GPIO_PinState HAL_GPIO_ReadPin(void *port,uint16_t pin)
{(void)pin;return key_levels[port==GPIOC?0:1]?GPIO_PIN_SET:GPIO_PIN_RESET;}
void Route_SetMotionInhibit(uint8_t inhibit) {motion_inhibit=inhibit;}
RouteResult_t Route_RequestStartArea(uint8_t area)
{
    if(Route_AuxIsBusy())return ROUTE_BUSY;
    if(Route_CheckStart()!=ROUTE_OK)return ROUTE_BAD_STATE;
    selected_area=area;area_signals++;route_status.state=ROUTE_EMPTY;
    return ROUTE_OK;
}

static uint32_t yaw_clearance_violations;
static uint8_t reproduce_reach_clearance, drift_after_clearance;
static uint32_t axis_moves[3];
uint32_t HAL_GetTick(void) {return tick;}
uint32_t __get_PRIMASK(void) {return mask;}
void __disable_irq(void) {mask=1;}
void __set_PRIMASK(uint32_t m) {mask=m;}
void __DMB(void) {}
void *__aeabi_memclr4(void *out,uint32_t n)
{uint8_t *p=out;while(n--) *p++=0;return out;}
void *__aeabi_memclr(void *out,uint32_t n) {return __aeabi_memclr4(out,n);}
void *__aeabi_memcpy(void *out,const void *in,uint32_t n)
{uint8_t *p=out;const uint8_t *q=in;while(n--) *p++=*q++;return out;}
void *__aeabi_memcpy4(void *out,const void *in,uint32_t n)
{return __aeabi_memcpy(out,in,n);}
/* Python CPU harness implements the ARM EABI signed 64-bit division contract. */
__attribute__((naked)) void __aeabi_ldivmod(void) {__asm volatile("bx lr");}
__attribute__((naked)) void __aeabi_uldivmod(void) {__asm volatile("bx lr");}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *u) {return u->ErrorCode;}
void can_SendCmd(volatile uint8_t *f,uint8_t n)
{
    uint8_t i,*r,function=f[1];
    uint32_t v,pulses;
    tx_count++;
    can_last_tx_status=HAL_ERROR;
    if(tx_fail || f[0]<5 || f[0]>7 || f[n-1]!=0x6B)return;
    i=(uint8_t)(f[0]-5);r=response[i];
    r[0]=f[0];r[1]=function;r[2]=2;r[3]=0x6B;response_n[i]=4;
    if(function==0xFE) {stop_count++;moving[i]=0;target[i]=pos[i];if(stop_reject)r[2]=0xE2;}
    if(function==0xFD) {
        if(n!=13 || f[10]!=2 || f[11]!=0)return;
        move_count++;axis_moves[i]++;
#if ARM_TRAY_ENABLED
        if(tray_pwm_writes && (uint32_t)(tick-tray_changed_at)<ARM_TRAY_SETTLE_MS) tray_timing_errors++;
        if(i==ARM_Z) tray_z_moves++;
        if(i==ARM_YAW && arm_motor_status[i].target_counts-arm_motor_status[i].origin_counts==45000LL*65536/36000)
            z_moves_at_tray_yaw=tray_z_moves;
#endif
        if(i==ARM_YAW) {
            int32_t z=(int32_t)(((int64_t)pos[ARM_Z]-arm_motor_status[ARM_Z].origin_counts)*36000/65536);
            int32_t reach=pos[ARM_REACH]-arm_motor_status[ARM_REACH].origin_counts;
            int32_t error=reach-(int32_t)(ARM_CLEAR_REACH_CDEG*65536LL/36000);
            int32_t tolerance=(int32_t)(ARM_TOLERANCE_CDEG*65536U/36000U+
                32768U/ARM_PULSES_PER_REV+1U);
            if(error<0)error=-error;
            if(z<ARM_YAW_MIN_Z_CDEG || error>tolerance)yaw_clearance_violations++;
        }
        pulses=((uint32_t)f[6]<<24)|((uint32_t)f[7]<<16)|((uint32_t)f[8]<<8)|f[9];
        target[i]=pos[i]+(f[2]?-1:1)*(int32_t)(pulses*65536U/3200U);moving[i]=1;
        if(reproduce_reach_clearance && i==ARM_REACH &&
           arm_motor_status[i].target_counts-arm_motor_status[i].origin_counts==ARM_CLEAR_REACH_CDEG*65536LL/36000) {
            /* Recorded feedback: -34866 encoder counts relative to origin = -19152 cdeg. */
            target[i]=arm_motor_status[i].origin_counts-34866;
        }
    }
    if(function==0x36) {
        v=(uint32_t)(pos[i]<0?-pos[i]:pos[i]);r[2]=(uint8_t)(pos[i]<0);
        r[3]=(uint8_t)(v>>24);r[4]=(uint8_t)(v>>16);r[5]=(uint8_t)(v>>8);r[6]=(uint8_t)v;
        r[7]=0x6B;response_n[i]=8;
    } else if(function==0x35) {
        r[2]=(uint8_t)(target[i]<pos[i]);r[3]=0;r[4]=moving[i]?10:0;r[5]=0x6B;response_n[i]=6;
    } else if(function==0x3A)r[2]=flags;
    if(function==reject_function) {r[2]=0xE2;response_n[i]=4;}
    if(function==drop_function)response_n[i]=0;
    response_at[i]=tick+2;
    can_last_tx_status=HAL_OK;
}
static void step(uint32_t ms)
{
    const int32_t counts_per_tick=ARM_TRAY_ENABLED ? 40 : 10;
    uint8_t i,n;
    while(ms--) {
        tick++;
        for(i=0;i<3;i++) {
            if(moving[i]) {
                if(target[i]-pos[i]>counts_per_tick)pos[i]+=counts_per_tick;
                else if(pos[i]-target[i]>counts_per_tick)pos[i]-=counts_per_tick;
                else {pos[i]=target[i];moving[i]=0;}
            }
            if(response_n[i] && tick>=response_at[i]) {
                n=response_n[i];response_n[i]=0;
                ArmCAN_RxFrame((uint32_t)(5U+i)<<8,response[i]+1,(uint8_t)(n-1U));
            }
        }
        RouteKeys_Task();
        if(drift_after_clearance && arm_work_status.active_axis==ARM_REACH) {
            ArmCAN_Task();
            if(arm_motor_status[ARM_REACH].state==ARM_MOTOR_DONE) {
                /* Simulate fresh position feedback outside tolerance before enabling yaw. */
                pos[ARM_REACH]-=10;
                arm_motor_status[ARM_REACH].position_counts=pos[ARM_REACH];
                arm_motor_status[ARM_REACH].position_cdeg=(int32_t)(
                    (pos[ARM_REACH]-arm_motor_status[ARM_REACH].origin_counts)*36000LL/65536);
                drift_after_clearance=0;
            }
        }
        ArmWork_Task();
    }
}

volatile RouteStatus_t route_status;
static uint32_t report_count, work_failed;
static uint8_t report_bytes[62];
uint8_t Chassis_IsBusy(void) {return 0U;}
uint8_t Route_IsActive(void) {return route_status.state==ROUTE_RUNNING || route_status.state==ROUTE_WAIT_WORK;}
void Route_Stop(void) {Route_OnStop();route_status.state=ROUTE_STOPPED;}
RouteResult_t Route_CompleteWork(uint32_t id,uint16_t index,uint8_t success)
{
    if(!Route_WorkMayComplete(id,index,success))return ROUTE_BUSY;
    if(!success) {work_failed++;Route_OnStop();route_status.state=ROUTE_FAULT;}
    else {route_status.state=ROUTE_RUNNING;route_status.point_index++;}
    return ROUTE_OK;
}
uint16_t Route_CRC16(const uint8_t *p,uint16_t n)
{uint16_t crc=0xffff;uint8_t b;while(n--){crc^=(uint16_t)*p++<<8;for(b=0;b<8;b++)crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;}return crc;}
uint8_t RouteUART_Send(const uint8_t *p,uint16_t n)
{uint16_t i;if(n!=62)return 0;for(i=0;i<n;i++)report_bytes[i]=p[i];report_count++;return 1;}
static void put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void put32(uint8_t *p,uint32_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
static uint8_t packet[32];
static RouteResult_t command(uint32_t id,uint8_t op,int32_t z,int32_t yaw,int32_t reach,uint32_t ref)
{
    uint8_t i;for(i=0;i<32;i++)packet[i]=0;
    if(op!=1){put32(packet,route_status.route_id);put16(packet+4,route_status.point_index);put16(packet+6,route_status.work_id);}
    put32(packet+8,id);packet[12]=op;put32(packet+16,(uint32_t)z);put32(packet+20,(uint32_t)yaw);
    put32(packet+24,(uint32_t)reach);put32(packet+28,ref);
    return Route_OnAuxCommand(0x31,packet,32);
}
static uint8_t until(uint32_t state)
{uint32_t i;for(i=0;i<90000;i++){if(arm_work_status.state==state)return 1;if(arm_work_status.state==ARM_WORK_FAULT)return 0;step(1);}return 0;}
static uint8_t home(void)
{return command(1,1,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY);}
static uint8_t station(uint16_t work)
{
    uint32_t before=tx_count;
    route_status.route_id=99;route_status.point_index=2;route_status.work_id=work;route_status.state=ROUTE_WAIT_WORK;
    Route_OnWork(99,2,work);
    if (arm_work_status.state!=ARM_WORK_WAIT_VISION || report_bytes[24]!=ARM_WORK_WAIT_VISION) return 0;
    step(250);
    return arm_work_status.state==ARM_WORK_WAIT_VISION && tx_count==before;
}
static uint8_t align(void)
{return command(10,2,-1000,2000,-3000,0)==ROUTE_OK && until(ARM_WORK_ALIGNED);}
#define CHECK(c,n) do {if(!(c))return n;}while(0)
static RouteResult_t test_command(uint32_t session,uint16_t work,uint32_t cmd,uint8_t op,
                                 int32_t z,int32_t yaw,int32_t reach,uint32_t ref)
{
    uint8_t i;for(i=0;i<32;i++)packet[i]=0;
    put32(packet,session);put16(packet+4,0);put16(packet+6,work);
    put32(packet+8,cmd);packet[12]=op;put32(packet+16,(uint32_t)z);
    put32(packet+20,(uint32_t)yaw);put32(packet+24,(uint32_t)reach);put32(packet+28,ref);
    return Route_OnAuxCommand(0x31,packet,32);
}
static int test_standalone(uint32_t test)
{
    uint32_t before;uint16_t work;
    if(test==50 || test==51) {
        uint8_t key=test==50?1U:0U;
        key_levels[key]=0;step(100);
        CHECK(!tx_count && !area_signals,500);
        key_levels[key]=1;step(600);
        CHECK(arm_work_status.state==ARM_WORK_READY && !area_signals && !move_count,501);
        CHECK(route_keys_debug.arm_test_ready_count==1 && (report_bytes[28]&8),502);
        return 0;
    }
    if(test==52) {
        CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_BAD_STATE,520);
        CHECK(test_command(1,101,1,6,0,0,0,0)==ROUTE_BAD_VALUE,521);
        Route_OnWork(99,0,102);step(100);
        CHECK(!move_count && arm_work_status.state==ARM_WORK_NEED_ORIGIN,522);return 0;
    }
    CHECK(home(),530);
    if(test==59) {
        route_status.state=ROUTE_RUNNING;
        CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_BUSY,590);
        route_status.state=ROUTE_EMPTY;
    }
    before=tx_count;
    CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_OK,531);
    CHECK(arm_work_status.state==ARM_WORK_WAIT_VISION && report_bytes[24]==ARM_WORK_WAIT_VISION,5340);
    step(250);
    CHECK(tx_count==before,5341);
    if(test==54) {
        before=move_count;
        CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_OK && move_count==before,540);
    }
    CHECK(until(ARM_WORK_WAIT_VISION),532);
    CHECK(route_status.state==ROUTE_EMPTY && !area_signals,533);
    if(test==58) {
        CHECK(test_command(2,103,1,6,0,0,0,0)==ROUTE_BAD_STATE,580);
        CHECK(arm_work_status.route_id==1 && arm_work_status.work_id==102,581);return 0;
    }
    if(test==59) {
        CHECK(command(2,1,0,0,0,0)==ROUTE_BUSY,591);
        arm_work_debug.command=2;arm_work_debug.axis=0;arm_work_debug.target_cdeg=-900;
        arm_work_debug.trigger++;step(1);
        CHECK(arm_work_debug.result==ROUTE_BUSY,592);return 0;
    }
    if(test==55) {
        key_levels[0]=0;step(100);key_levels[0]=1;step(300);
        CHECK(arm_work_status.state==ARM_WORK_STOPPED && !work_failed,550);
        CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_CONFLICT,551);
        key_levels[1]=0;step(100);key_levels[1]=1;step(600);
        CHECK(arm_work_status.state==ARM_WORK_READY && !area_signals,552);
        CHECK(test_command(2,103,1,6,0,0,0,0)==ROUTE_OK,553);
        CHECK(until(ARM_WORK_WAIT_VISION),554);
        CHECK(test_command(1,102,2,2,0,0,0,0)==ROUTE_BAD_STATE,555);return 0;
    }
    if(test==57) {
        before=move_count;
        CHECK(test_command(1,102,2,2,ARM_Z_MIN_CDEG-1,0,0,0)==ROUTE_BAD_VALUE,570);
        CHECK(test_command(1,103,2,2,0,0,0,0)==ROUTE_BAD_STATE,571);
        put16(packet+4,1);
        CHECK(Route_OnAuxCommand(0x31,packet,32)==ROUTE_BAD_STATE,572);
        CHECK(move_count==before,573);return 0;
    }
    CHECK(test_command(1,102,2,2,-1000,2000,-3000,0)==ROUTE_OK,534);
    if(test==56) {
        drop_function=0x36;step(1000);
        CHECK(arm_work_status.state==ARM_WORK_FAULT && !work_failed,560);
        CHECK(route_status.state==ROUTE_EMPTY,561);return 0;
    }
    CHECK(until(ARM_WORK_ALIGNED),535);
    if(test==54) {
        before=move_count;
        CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_OK,541);step(100);
        CHECK(move_count==before && arm_work_status.state==ARM_WORK_ALIGNED,542);
        CHECK(test_command(1,102,2,2,-1000,2000,-3000,0)==ROUTE_OK,543);
        CHECK(test_command(1,102,2,2,-1001,2000,-3000,0)==ROUTE_CONFLICT,544);
        CHECK(test_command(1,102,3,3,0,0,0,1)==ROUTE_BAD_STATE,545);return 0;
    }
    CHECK(test_command(1,102,3,3,0,0,0,2)==ROUTE_OK && until(ARM_WORK_WAIT_GRIP),536);
    before=move_count;step(500);
    CHECK(move_count==before && arm_work_status.state==ARM_WORK_WAIT_GRIP,537);
    CHECK(test_command(1,102,4,4,0,0,0,2)==ROUTE_OK && until(ARM_WORK_PICKED),538);
    CHECK(test_command(1,102,5,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),539);
    for(work=103;work<=107;work++) {
        int32_t parked_reach=arm_motor_status[ARM_REACH].position_cdeg;
        before=tx_count;
        CHECK(test_command(work,work,1,6,0,0,0,0)==ROUTE_OK,546);
        CHECK(arm_work_status.state==ARM_WORK_WAIT_VISION,547);
        step(250);
        CHECK(tx_count==before && arm_work_status.target_cdeg[ARM_REACH]==parked_reach,5500);
        CHECK(arm_motor_status[ARM_REACH].position_cdeg==parked_reach,5501);
        CHECK(test_command(work,work,2,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),548);
    }
    CHECK(route_status.state==ROUTE_EMPTY && !work_failed,549);return 0;
}
static RouteResult_t micro_command(uint32_t id,int32_t z,int32_t yaw,int32_t reach)
{
#if ARM_VISION_TEST_MODE
    return test_command(1,102,id,2,z,yaw,reach,0);
#else
    return command(id,2,z,yaw,reach,0);
#endif
}
static RouteResult_t claw_command(uint16_t work,uint32_t id,uint8_t op,uint32_t ref)
{
#if ARM_VISION_TEST_MODE
    return test_command(1,work,id,op,0,0,0,ref);
#else
    (void)work;
    return command(id,op,0,0,0,ref);
#endif
}

static int claw_case(uint32_t test)
{
    const uint16_t work = (test==86) ? ROUTE_WORK_PROCESSING_1 :
        ((test==85) ? ROUTE_WORK_MATERIAL_2 : ROUTE_WORK_MATERIAL_1);
    uint32_t moves_before_lift, commands_before_pick;

    CHECK(pwm_compare==ARM_CLAW_OPEN_COMPARE && !pwm_bad_channel,830);
    CHECK(home(),831);
#if ARM_VISION_TEST_MODE
    CHECK(test_command(1,work,1,6,0,0,0,0)==ROUTE_OK,832);
#else
    CHECK(station(work),832);
#endif
    CHECK(claw_command(work,2,2,0)==ROUTE_OK && until(ARM_WORK_ALIGNED),833);
    if (test >= 88) {
        /* Inject the Z feedback returned just before PICK, at the upper limit. */
        int32_t feedback = arm_motor_config[ARM_Z].max_cdeg + 1;
        if (test == 89) feedback += arm_motor_config[ARM_Z].tolerance_cdeg;
        if (test == 90) feedback = arm_motor_config[ARM_Z].max_cdeg + arm_motor_config[ARM_Z].tolerance_cdeg;
        arm_motor_status[ARM_Z].position_cdeg = feedback;
    }
    if (test == 89) {
        commands_before_pick = tx_count;
        CHECK(claw_command(work,3,3,2)==ROUTE_BAD_VALUE,848);
        step(250);
        CHECK(tx_count==commands_before_pick && pwm_compare==ARM_CLAW_OPEN_COMPARE,849);
        CHECK(arm_work_status.state==ARM_WORK_ALIGNED && arm_work_status.command_id==2,850);
        return 0;
    }

    CHECK(claw_command(work,3,3,2)==ROUTE_OK,834);
    if (test >= 88) {
        CHECK(arm_work_status.target_cdeg[ARM_Z] == ARM_Z_MAX_CDEG +
            ARM_Z_DOWN_SIGN * ARM_PICK_DROP_CDEG,851);
    }
    CHECK(claw_command(work,4,4,2)==ROUTE_BAD_STATE,835);
    step(250);
    CHECK(pwm_compare==ARM_CLAW_OPEN_COMPARE && !pwm_close_count,836);
    CHECK(until(ARM_WORK_WAIT_GRIP),837);
    if (test==86) {
        CHECK(pwm_compare==ARM_CLAW_OPEN_COMPARE && !pwm_close_count,838);
        return 0;
    }
    CHECK(pwm_compare==ARM_CLAW_CLOSED_COMPARE && pwm_close_count==1,839);
    CHECK(claw_command(work,3,3,2)==ROUTE_OK && pwm_close_count==1,840);
    if (test==87) {
        Route_Stop();
        CHECK(pwm_compare==ARM_CLAW_OPEN_COMPARE && arm_work_status.state==ARM_WORK_STOPPED,841);
        return 0;
    }

    moves_before_lift=move_count;
    CHECK(claw_command(work,4,4,2)==ROUTE_OK,842);
    if (test >= 88) CHECK(arm_work_status.target_cdeg[ARM_Z]==ARM_Z_MAX_CDEG,852);
    step(ARM_CLAW_CLOSE_MS-1U);
    CHECK(move_count==moves_before_lift && pwm_compare==ARM_CLAW_CLOSED_COMPARE,843);
    CHECK(until(ARM_WORK_PICKED),844);
    CHECK(pwm_compare==ARM_CLAW_CLOSED_COMPARE && pwm_close_count==1,845);
    CHECK(claw_command(work,5,test==85?2:5,0)==ROUTE_OK,846);
    CHECK(pwm_compare==ARM_CLAW_OPEN_COMPARE && !pwm_bad_channel,847);
    return 0;
}

static int micro_case(uint32_t test)
{
    uint32_t before,z_before,reach_before,yaw_before;int32_t z,yaw,reach;
    CHECK(home(),700);
#if ARM_VISION_TEST_MODE
    CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_OK && until(ARM_WORK_WAIT_VISION),701);
#else
    CHECK(station(102),701);
#endif
    if(test==77 || test==78 || test==79) {
        int32_t initial_z=test==77?-95000:-1000;
        int32_t initial_reach=test==78?-25000:-3000;
        CHECK(micro_command(10,initial_z,2000,initial_reach)==ROUTE_OK && until(ARM_WORK_ALIGNED),702);
    }
    z=arm_motor_status[0].position_cdeg;yaw=arm_motor_status[1].position_cdeg;
    reach=arm_motor_status[2].position_cdeg;
    before=move_count;z_before=axis_moves[0];yaw_before=axis_moves[1];reach_before=axis_moves[2];
    if(test==70) { /* Both axes, at the output 1 degree / 4 mm limits. */
        CHECK(micro_command(11,z,yaw+300,reach-1273)==ROUTE_OK && until(ARM_WORK_ALIGNED),710);
        CHECK(move_count==before+2 && axis_moves[0]==z_before,711);
        CHECK(axis_moves[1]==yaw_before+1 && axis_moves[2]==reach_before+1,712);
        CHECK(arm_motor_status[2].position_cdeg>-1400,713);
    } else if(test==71 || test==72) {
        CHECK(micro_command(11,z,yaw+(test==71?200:0),reach-(test==72?600:0))==ROUTE_OK && until(ARM_WORK_ALIGNED),720);
        CHECK(move_count==before+1 && axis_moves[0]==z_before,721);
        CHECK(axis_moves[1]==yaw_before+(test==71) && axis_moves[2]==reach_before+(test==72),722);
    } else if(test==73) {
        CHECK(micro_command(11,z,yaw,reach)==ROUTE_OK && until(ARM_WORK_ALIGNED),730);
        CHECK(move_count==before && arm_work_status.aligned_command_id==11,731);
        CHECK(micro_command(11,z,yaw,reach)==ROUTE_OK && move_count==before,732);
    } else if(test==74 || test==75 || test==76 || test==77 || test==78) {
        CHECK(micro_command(11,z-(test==76?900:0),yaw+(test==74?301:200),reach-(test==75?1274:600))==ROUTE_OK && until(ARM_WORK_ALIGNED),740);
        CHECK(move_count==before+5 && axis_moves[0]==z_before+2,741);
    } else if(test==79) { /* Negative yaw / forward reach correction also direct. */
        CHECK(micro_command(11,z,yaw-200,reach+600)==ROUTE_OK && until(ARM_WORK_ALIGNED),790);
        CHECK(move_count==before+2 && axis_moves[0]==z_before,791);
    } else if(test==80 || test==81) { /* Execution-time validation is independent of full clearance. */
        CHECK(micro_command(11,z,yaw+200,reach-600)==ROUTE_OK,800);
        if(test==80)arm_motor_status[0].position_cdeg=ARM_ALIGN_MICRO_MIN_Z_CDEG-1;
        else arm_motor_status[1].position_cdeg=yaw-500;
        step(1);
        CHECK(arm_work_status.state==ARM_WORK_FAULT && arm_work_status.fault==ARM_WORK_POSE,801);
        CHECK(move_count==before,802);return 0;
    } else if(test==82) {
        CHECK(micro_command(11,z,yaw+200,reach-600)==ROUTE_OK && until(ARM_WORK_ALIGNED),820);
        before=move_count;
#if ARM_VISION_TEST_MODE
        CHECK(test_command(1,102,12,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),821);
#else
        CHECK(command(12,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),821);
#endif
        CHECK(move_count==before+5,822);
        CHECK(arm_motor_status[2].position_cdeg<ARM_CLEAR_REACH_CDEG+50,823);
    }
    CHECK(arm_work_status.fault==ARM_WORK_NO_FAULT,799);return 0;
}
static int clearance_after_pick(uint32_t test)
{
    const uint16_t work=ROUTE_WORK_MATERIAL_1;
    uint32_t yaw_moves, elapsed;

    CHECK(home(),910);
#if ARM_VISION_TEST_MODE
    CHECK(test_command(1,work,1,6,0,0,0,0)==ROUTE_OK,911);
#else
    CHECK(station(work),911);
#endif
    CHECK(micro_command(2,0,900,-3000)==ROUTE_OK && until(ARM_WORK_ALIGNED),912);
    CHECK(claw_command(work,3,3,2)==ROUTE_OK && until(ARM_WORK_WAIT_GRIP),913);
    CHECK(claw_command(work,4,4,2)==ROUTE_OK && until(ARM_WORK_PICKED),914);

    yaw_moves=axis_moves[ARM_YAW];
    reproduce_reach_clearance=1;
    drift_after_clearance=(uint8_t)(test==92);
    CHECK(claw_command(work,5,2,0)==ROUTE_OK,915);
    for(elapsed=0;elapsed<10000 && axis_moves[ARM_YAW]==yaw_moves &&
        arm_work_status.state!=ARM_WORK_FAULT;elapsed++) step(1);
    if(test==92) {
        CHECK(!drift_after_clearance && arm_work_status.state==ARM_WORK_FAULT,916);
        CHECK(arm_work_status.fault==ARM_WORK_POSE && arm_work_status.active_axis==ARM_YAW,917);
        CHECK(axis_moves[ARM_YAW]==yaw_moves,918);
    } else {
        CHECK(axis_moves[ARM_YAW]==yaw_moves+1 && !yaw_clearance_violations,919);
        CHECK(arm_motor_status[ARM_REACH].position_cdeg==-19152,920);
        CHECK(until(ARM_WORK_ALIGNED),921);
    }
    return 0;
}

#if ARM_TRAY_ENABLED
static uint8_t open_tray_station(uint32_t session,uint16_t work)
{
#if ARM_VISION_TEST_MODE
    return test_command(session,work,1,6,0,0,0,0)==ROUTE_OK;
#else
    (void)session;
    return station(work);
#endif
}
static RouteResult_t tray_command(uint32_t id,uint8_t opcode,uint8_t slot,int32_t z,int32_t yaw,int32_t reach,uint32_t ref)
{
    uint8_t i;
    for(i=0U;i<32U;i++)packet[i]=0U;
    put32(packet,arm_work_status.route_id);
    put16(packet+4,arm_work_status.point_index);
    put16(packet+6,arm_work_status.work_id);
    put32(packet+8,id);
    packet[12]=opcode;packet[13]=slot;
    put32(packet+16,(uint32_t)z);put32(packet+20,(uint32_t)yaw);put32(packet+24,(uint32_t)reach);put32(packet+28,ref);
    return Route_OnAuxCommand(0x31,packet,32);
}
static RouteResult_t tray_place_command(uint32_t id,uint8_t slot,int32_t z,int32_t yaw,int32_t reach)
{
    return tray_command(id,7U,slot,z,yaw,reach,0U);
}
static uint8_t close_tray_station(uint32_t id)
{
    if(tray_command(id,5,0,0,0,0,0)!=ROUTE_OK || !until(ARM_WORK_READY_LEAVE)) return 0U;
#if !ARM_VISION_TEST_MODE
    if(Route_CompleteWork(route_status.route_id,route_status.point_index,1U)!=ROUTE_OK) return 0U;
    step(1);
#endif
    return 1U;
}
static int tray_case(uint32_t test)
{
    const uint16_t destination=test==101 ? ROUTE_WORK_STORAGE_2 : ROUTE_WORK_PROCESSING_1;
    const uint8_t slots[]={3U,1U,2U};
    const uint32_t compares[]={ARM_TRAY_FIRST_COMPARE,ARM_TRAY_SECOND_COMPARE,ARM_TRAY_THIRD_COMPARE};
    uint8_t i;
    uint32_t id=2U;

    CHECK(!tray_pwm_writes && home() && open_tray_station(1,ROUTE_WORK_MATERIAL_1),1000);
    CHECK((report_bytes[28]&16) && !report_bytes[30] && !tray_pwm_writes,1001);
    CHECK(tray_place_command(id,1,0,0,0)==ROUTE_BAD_STATE,1002);
    CHECK(tray_command(id,2,0,0,45000,0,0)==ROUTE_BAD_VALUE,1044);
    for(i=0U;i<3U;i++,id+=3U) {
        CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id,2,0)==ROUTE_OK && until(ARM_WORK_ALIGNED),1003);
        if(test==102) {
            arm_motor_config[ARM_YAW].max_cdeg=ARM_TRAY_YAW_CDEG-1;
            CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id+1,3,id)==ROUTE_BAD_VALUE && !tray_pwm_writes,1004);
            arm_motor_config[ARM_YAW].max_cdeg=ARM_YAW_TRAVEL_MAX_CDEG;
        }
        CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id+1,3,id)==ROUTE_OK && until(ARM_WORK_WAIT_GRIP),1005);
        CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id+2,4,id)==ROUTE_OK,1006);
        if(test==102) {
            CHECK(until(ARM_WORK_TRAY_LOADING),1007);
            Route_Stop();step(100);
            CHECK(arm_work_status.state==ARM_WORK_STOPPED && (report_bytes[28]&32) && !report_bytes[30],1008);
            CHECK(!tray_pwm_writes && pwm_compare==ARM_CLAW_CLOSED_COMPARE,1009);
            arm_work_debug.command=5;arm_work_debug.trigger++;step(1);
            CHECK(arm_work_debug.result==ROUTE_OK && !(report_bytes[28]&32) && pwm_compare==ARM_CLAW_OPEN_COMPARE,1010);
            return 0;
        }
        CHECK(until(ARM_WORK_PICKED) && report_bytes[30]==i+1 && pwm_compare==ARM_CLAW_OPEN_COMPARE,1011);
        CHECK(tray_pwm_writes==(i==0 ? 2U : 3U) && tray_pwm_starts==1U,1012);
        CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id+2,4,id)==ROUTE_OK && report_bytes[30]==i+1,1013);
    }
    for(i=0U;i<3U;i++) CHECK(tray_pwm_log[i]==compares[i],1014);
    CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id,2,0)==ROUTE_OK && until(ARM_WORK_ALIGNED),1015);
    CHECK(claw_command(ROUTE_WORK_MATERIAL_1,id+1,3,id)==ROUTE_BAD_STATE,1016);
    CHECK(close_tray_station(id+1),1017);
    CHECK(open_tray_station(2,destination) && report_bytes[30]==3U,1019);
    CHECK(tray_place_command(2,0,0,0,0)==ROUTE_BAD_VALUE,1020);
    CHECK(tray_place_command(2,1,ARM_Z_MIN_CDEG-1,0,0)==ROUTE_BAD_VALUE,1021);
    CHECK(tray_place_command(2,1,0,45000,0)==ROUTE_BAD_VALUE,1045);
    for(i=0U;i<3U;i++) {
        int32_t z=-1000*(int32_t)slots[i], yaw=2000*(int32_t)slots[i], reach=-3000*(int32_t)slots[i];
        CHECK(tray_place_command(2U+i,slots[i],z,yaw,reach)==ROUTE_OK && until(ARM_WORK_PLACED),1022);
        CHECK(report_bytes[30]==2U-i && report_bytes[31]==slots[i],1023);
        CHECK(tray_pick_index[i]==compares[i] && placements==i+1U,1024);
        CHECK(placed_z[i]>=z-60 && placed_z[i]<=z+60 && placed_yaw[i]>=yaw-60 && placed_yaw[i]<=yaw+60 &&
              placed_reach[i]>=reach-60 && placed_reach[i]<=reach+60,1025);
        CHECK(tray_place_command(2U+i,slots[i],z,yaw,reach)==ROUTE_OK && placements==i+1U,1026);
        if(i<2U) CHECK(tray_place_command(3U+i,slots[i],z,yaw,reach)==ROUTE_CONFLICT,1027);
    }
    CHECK(tray_pwm_writes==7U && tray_pwm_log[6]==ARM_TRAY_FIRST_COMPARE,1028);
    CHECK(tray_place_command(5,1,0,0,0)==ROUTE_BAD_STATE,1029);
    if(test==100) {
        CHECK(tray_command(5U,5,0,0,0,0,0)==ROUTE_BAD_STATE,1031);
        for(i=0U,id=5U;i<3U;i++,id+=3U) {
            CHECK(tray_command(id,2,0,0,0,0,0)==ROUTE_OK && until(ARM_WORK_ALIGNED),1032);
            CHECK(tray_command(id+1,8,slots[(i+1U)%3U],-9000,0,0,id)==ROUTE_CONFLICT,1033);
            CHECK(tray_command(id+1,8,slots[i],0,0,0,id)==ROUTE_BAD_VALUE,1034);
            CHECK(tray_command(id+1,8,slots[i],-9000,0,0,id)==ROUTE_OK && until(ARM_WORK_WAIT_GRIP),1035);
            CHECK(pwm_compare==ARM_CLAW_CLOSED_COMPARE,1036);
            CHECK(tray_command(id+2,4,0,0,0,0,id)==ROUTE_OK && until(ARM_WORK_PICKED),1037);
            CHECK(report_bytes[30]==i+1,1038);
        }
        CHECK(close_tray_station(id) && open_tray_station(3,ROUTE_WORK_STORAGE_1),1039);
        for(i=0U;i<3U;i++) {
            CHECK(tray_place_command(2U+i,slots[i],-7000,3000,-5000)==ROUTE_OK && until(ARM_WORK_PLACED),1040);
            CHECK(tray_pick_index[3U+i]==compares[i] && report_bytes[30]==2U-i,1041);
        }
        CHECK(close_tray_station(5),1042);
    } else CHECK(close_tray_station(5),1043);
    CHECK(!tray_timing_errors && !pwm_bad_channel && !yaw_clearance_violations,1030);
    return 0;
}
#endif

int run_case(uint32_t test)
{
    uint8_t i;uint32_t before;
    CHECK(ArmWork_Init()==HAL_OK,1);RouteKeys_Init();
    for(i=0;i<3;i++){pos[i]=1000*(i+1);target[i]=pos[i];}
#if ARM_TRAY_ENABLED
    if(test>=100) return tray_case(test);
#endif
    if(test==91 || test==92) return clearance_after_pick(test);
    if(test>=84) return claw_case(test);
    if(test>=70) return micro_case(test);
    if(test>=50) return test_standalone(test);
    if(test==42) {CHECK(test_command(1,102,1,6,0,0,0,0)==ROUTE_BAD_STATE,420);return 0;}
    if(test>=36 && test<=41) {
        if(test==38) {
            CHECK(home(),380);
            arm_work_debug.command=2;arm_work_debug.axis=2;arm_work_debug.target_cdeg=-3183;
            arm_work_debug.trigger++;step(1);CHECK(until(ARM_WORK_DEBUG_HOLD),381);
            for(i=0;i<3;i++){pos[i]+=1234;target[i]=pos[i];}
        }
        if(test==39)drop_function=0x36;
        key_levels[test==37?0:1]=0;step(100);
        CHECK(!area_signals && !route_keys_debug.pose_preparing,360);
        if(test!=38)CHECK(!tx_count,361);
        key_levels[0]=key_levels[1]=1;step(35);
        if(test==40) {key_levels[0]=0;step(100);key_levels[0]=1;step(500);
            CHECK(!area_signals && !route_keys_debug.pending_area,400);
            CHECK(arm_work_status.state==ARM_WORK_STOPPED && !arm_motor_status[0].homed,401);return 0;}
        step(600);
        if(test==39) {CHECK(!area_signals && arm_work_status.state==ARM_WORK_FAULT,390);
            CHECK(route_keys_debug.pose_result==ROUTE_BAD_STATE && !route_keys_debug.pending_area,391);return 0;}
        CHECK(area_signals==1 && selected_area==(test==37?2U:1U),362);
        CHECK(arm_work_status.state==ARM_WORK_READY && !motion_inhibit,363);
        for(i=0;i<3;i++)CHECK(arm_motor_status[i].homed && arm_motor_status[i].origin_counts==pos[i],364);
        CHECK(!route_keys_debug.pending_area && !route_keys_debug.pose_preparing,365);
        if(test!=38)CHECK(!move_count,366);
        if(test==41) {
            for(i=0;i<3;i++){pos[i]+=567;target[i]=pos[i];}
            key_levels[1]=0;step(100);CHECK(area_signals==1,410);
            key_levels[1]=1;step(600);CHECK(area_signals==2,411);
            for(i=0;i<3;i++)CHECK(arm_motor_status[i].origin_counts==pos[i],412);
        }
        return 0;
    }
    if(test==1){step(500);CHECK(!tx_count && arm_work_status.state==ARM_WORK_NEED_ORIGIN,10);CHECK(Route_CheckStart()==ROUTE_BAD_STATE,11);return 0;}
    if(test==2){CHECK(home(),20);CHECK(!move_count && Route_CheckStart()==ROUTE_OK,21);CHECK(!arm_motor_status[0].position_cdeg && arm_motor_status[2].homed,22);return 0;}
    if(test==19) {
        arm_work_debug.command=2;arm_work_debug.axis=0;arm_work_debug.target_cdeg=900;arm_work_debug.trigger++;
        step(1);CHECK(arm_work_debug.result==ROUTE_BAD_STATE && !move_count,190);return 0;
    }
    if(test==28 || test==29 || test==30) {
        if(test==28) {drop_function=0xF3;CHECK(command(1,1,0,0,0,0)==ROUTE_OK,280);step(500);CHECK(arm_work_status.state==ARM_WORK_FAULT,281);drop_function=0;}
        if(test==29) route_status.state=ROUTE_RUNNING;
        arm_work_debug.command=4;arm_work_debug.axis=test==30?3:1;arm_work_debug.trigger++;
        before=tx_count;step(100);
        if(test==29 || test==30) {CHECK(arm_work_debug.result==(test==29?ROUTE_BUSY:ROUTE_BAD_VALUE) && tx_count==before,290);return 0;}
        CHECK(arm_work_debug.result==ROUTE_OK && arm_motor_status[1].state==ARM_MOTOR_DONE,282);
        CHECK(tx_count==before+1 && !move_count && !arm_motor_status[1].homed,283);
        CHECK(arm_work_status.state==ARM_WORK_FAULT && Route_CheckStart()==ROUTE_BAD_STATE,284);return 0;
    }
    CHECK(home(),2);
    if(test==33) {
        /* Start below yaw height and near the rear end: debug yaw must first
         * lift and advance to the 60 mm clearance, then turn, holding there. */
        pos[0]=arm_motor_status[0].origin_counts+(int32_t)(-108000LL*65536/36000);
        pos[2]=arm_motor_status[2].origin_counts+(int32_t)(-38000LL*65536/36000);
        target[0]=pos[0];target[2]=pos[2];
        arm_work_debug.command=2;arm_work_debug.axis=1;arm_work_debug.target_cdeg=1500;
        arm_work_debug.trigger++;step(1);CHECK(arm_work_debug.result==ROUTE_OK,330);
        CHECK(until(ARM_WORK_DEBUG_HOLD),331);
        CHECK(!yaw_clearance_violations && move_count==3,332);
        CHECK(arm_motor_status[0].position_cdeg>=-50 && arm_motor_status[0].position_cdeg<=0,333);
        CHECK(arm_motor_status[2].position_cdeg>ARM_CLEAR_REACH_CDEG-50 &&
            arm_motor_status[2].position_cdeg<ARM_CLEAR_REACH_CDEG+50,334);
        CHECK(arm_motor_status[1].position_cdeg>1450 && arm_motor_status[1].position_cdeg<1550,335);return 0;
    }
    if(test==34) {
        CHECK(station(102) && align(),340);CHECK(!yaw_clearance_violations,341);
        CHECK(command(11,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),342);
        CHECK(!yaw_clearance_violations && arm_motor_status[0].position_cdeg>=-50,343);
        CHECK(arm_motor_status[2].position_cdeg>ARM_CLEAR_REACH_CDEG-50 &&
            arm_motor_status[2].position_cdeg<ARM_CLEAR_REACH_CDEG+50,344);return 0;
    }
    if(test==35) {
        /* Force inconsistent feedback after the clearance steps. No yaw FD
         * may be sent; the workflow must report pose fault instead. */
        arm_work_debug.command=2;arm_work_debug.axis=1;arm_work_debug.target_cdeg=1500;
        arm_work_debug.trigger++;step(1);
        for(before=0;before<30000;before++) {
            if(move_count==2)break;
            step(1);
        }
        CHECK(before<30000,350);
        arm_motor_status[0].position_cdeg=ARM_YAW_MIN_Z_CDEG-1;step(10000);
        CHECK(arm_work_status.state==ARM_WORK_FAULT && arm_work_status.fault==ARM_WORK_POSE,351);
        CHECK(move_count==2 && !yaw_clearance_violations,352);return 0;
    }
    if(test==32) {
        arm_work_debug.command=2;arm_work_debug.axis=0;
        arm_work_debug.target_cdeg=ARM_Z_MIN_CDEG-1;arm_work_debug.trigger++;
        step(1);CHECK(arm_work_debug.result==ROUTE_BAD_VALUE && !move_count,320);
        arm_work_debug.axis=2;arm_work_debug.target_cdeg=ARM_REACH_MIN_CDEG-1;arm_work_debug.trigger++;
        step(1);CHECK(arm_work_debug.result==ROUTE_BAD_VALUE && !move_count,321);
        CHECK(station(102),322);before=move_count;
        CHECK(command(10,2,ARM_Z_MIN_CDEG-1,0,0,0)==ROUTE_BAD_VALUE,323);
        CHECK(command(10,2,0,0,ARM_REACH_MIN_CDEG-1,0)==ROUTE_BAD_VALUE,324);
        step(20);CHECK(move_count==before,325);return 0;
    }
    if(test==31) {
        arm_work_debug.command=2;arm_work_debug.axis=0;arm_work_debug.target_cdeg=900;arm_work_debug.trigger++;
        step(1);CHECK(arm_work_debug.result==ROUTE_BAD_VALUE && !move_count,310);
        arm_work_debug.axis=2;arm_work_debug.target_cdeg=318;arm_work_debug.trigger++;
        step(1);CHECK(arm_work_debug.result==ROUTE_BAD_VALUE && !move_count,311);
        CHECK(station(102),312);before=move_count;
        CHECK(command(10,2,900,0,0,0)==ROUTE_BAD_VALUE,313);
        CHECK(command(10,2,0,0,318,0)==ROUTE_BAD_VALUE,314);
        step(20);CHECK(move_count==before,315);return 0;
    }
    if(test>=20 && test<=27) {
        arm_work_debug.command=2;arm_work_debug.axis=2;arm_work_debug.target_cdeg=-3183;
        if(test==20)arm_work_debug.axis=3;
        if(test==21)arm_work_debug.target_cdeg=ARM_REACH_MAX_CDEG+1;
        if(test==22)route_status.state=ROUTE_RUNNING;
        arm_work_debug.trigger++;step(1);
        if(test==20 || test==21) {CHECK(arm_work_debug.result==ROUTE_BAD_VALUE && !move_count,200);return 0;}
        if(test==22) {CHECK(arm_work_debug.result==ROUTE_BUSY && !move_count,220);return 0;}
        CHECK(arm_work_debug.result==ROUTE_OK,201);
        if(test==23) {step(20);arm_work_debug.command=0;arm_work_debug.trigger++;step(300);CHECK(arm_work_status.state==ARM_WORK_STOPPED && !moving[2] && !arm_motor_status[2].homed,230);return 0;}
        if(test==24) {drop_function=0x36;step(500);CHECK(arm_work_status.state==ARM_WORK_FAULT && arm_work_status.fault==ARM_WORK_MOTOR,240);return 0;}
        if(test==25) {arm_work_debug.command=2;arm_work_debug.target_cdeg=-6366;arm_work_debug.trigger++;step(1);CHECK(arm_work_debug.result==ROUTE_BUSY,250);CHECK(until(ARM_WORK_DEBUG_HOLD),251);CHECK(arm_motor_status[2].position_cdeg< -3100,252);return 0;}
        CHECK(until(ARM_WORK_DEBUG_HOLD),202);
        CHECK(Route_CheckStart()==ROUTE_BAD_STATE && move_count==1,203);
        CHECK(arm_motor_status[2].position_cdeg< -3100 && arm_motor_status[2].position_cdeg> -3250,204);
        CHECK(!arm_motor_status[0].position_cdeg && !arm_motor_status[1].position_cdeg,205);
        before=move_count;step(300);CHECK(before==move_count,206);
        if(test==26) {
            arm_work_debug.target_cdeg=-6366;arm_work_debug.trigger++;step(1);CHECK(until(ARM_WORK_DEBUG_HOLD),260);
            CHECK(arm_motor_status[2].position_cdeg< -6300 && move_count==2,261);return 0;
        }
        arm_work_debug.command=3;arm_work_debug.trigger++;step(1);CHECK(until(ARM_WORK_READY),270);
        CHECK(arm_motor_status[2].position_cdeg>=-50 && arm_motor_status[2].position_cdeg<=50,271);
        CHECK(Route_CheckStart()==ROUTE_OK,272);return 0;
    }
    if(test==3){Route_OnWork(99,2,101);Route_OnWork(99,2,108);step(50);CHECK(!move_count && arm_work_status.state==ARM_WORK_READY,30);return 0;}
    if(test==4){CHECK(command(10,2,0,0,0,0)==ROUTE_BAD_STATE,40);CHECK(!move_count,41);return 0;}
    CHECK(station(102),3);
    if(test==5){CHECK(command(10,3,0,0,0,10)==ROUTE_BAD_STATE,50);CHECK(command(10,2,0,ARM_YAW_MAX_CDEG+1,0,0)==ROUTE_BAD_VALUE,51);return 0;}
    if(test==6){CHECK(command(10,2,-1000,2000,-3000,0)==ROUTE_OK,60);CHECK(command(10,2,-1000,2000,-3000,0)==ROUTE_OK,61);CHECK(until(ARM_WORK_ALIGNED),62);before=move_count;CHECK(command(10,2,-1000,2000,-3000,0)==ROUTE_OK,63);step(500);CHECK(move_count==before && arm_work_status.aligned_command_id==10,64);CHECK(arm_motor_status[2].position_cdeg< -2950,65);return 0;}
    CHECK(align(),4);
    if(test==7){CHECK(command(10,2,-1000,2001,-3000,0)==ROUTE_CONFLICT,70);CHECK(command(9,2,0,0,0,0)==ROUTE_CONFLICT,71);CHECK(command(11,3,0,0,0,9)==ROUTE_BAD_STATE,72);return 0;}
    if(test==8){CHECK(!Route_WorkMayComplete(99,2,1),80);CHECK(command(11,5,0,0,0,0)==ROUTE_OK,81);CHECK(!Route_WorkMayComplete(99,2,1),82);CHECK(until(ARM_WORK_READY_LEAVE),83);CHECK(Route_CompleteWork(99,2,1)==ROUTE_OK,84);step(1);CHECK(arm_work_status.state==ARM_WORK_READY && Route_CheckStart()==ROUTE_OK,85);return 0;}
    if(test==9){Route_Stop();step(300);CHECK(arm_work_status.state==ARM_WORK_STOPPED && !ArmCAN_Busy(),90);CHECK(Route_CheckStart()==ROUTE_BAD_STATE,91);CHECK(command(11,3,0,0,0,10)==ROUTE_BAD_STATE,92);return 0;}
    if(test==10){drop_function=0x36;CHECK(command(11,3,0,0,0,10)==ROUTE_OK,100);step(500);CHECK(arm_work_status.state==ARM_WORK_FAULT && arm_work_status.fault==ARM_WORK_MOTOR,101);CHECK(work_failed==1 && route_status.state==ROUTE_FAULT,102);return 0;}
    if(test==11){{uint8_t f[3]={0x3A,1,0x6B};for(i=0;i<128;i++) ArmCAN_RxFrame(0x700,f,3);}step(300);CHECK(arm_work_status.state==ARM_WORK_FAULT && work_failed==1,110);return 0;}
    if(test==12){CHECK(command(11,2,ARM_Z_MIN_CDEG+500,0,0,0)==ROUTE_OK && until(ARM_WORK_ALIGNED),120);CHECK(command(12,3,0,0,0,11)==ROUTE_BAD_VALUE,121);return 0;}
    CHECK(command(11,3,0,0,0,10)==ROUTE_OK,5);
    CHECK(until(ARM_WORK_WAIT_GRIP),6);
    before=move_count;
    CHECK(arm_motor_status[0].position_cdeg< -900-ARM_PICK_DROP_CDEG && arm_motor_status[1].position_cdeg>1950 && arm_motor_status[1].position_cdeg<2050,7);
    if(test==13){step(5000);CHECK(move_count==before && arm_work_status.state==ARM_WORK_WAIT_GRIP,130);CHECK(!Route_WorkMayComplete(99,2,1),131);CHECK(command(12,5,0,0,0,0)==ROUTE_BAD_STATE,132);return 0;}
    if(test==14){CHECK(command(11,3,0,0,0,10)==ROUTE_OK,140);step(500);CHECK(move_count==before,141);CHECK(command(12,4,0,0,0,9)==ROUTE_BAD_STATE,142);return 0;}
    CHECK(command(12,4,0,0,0,10)==ROUTE_OK && until(ARM_WORK_PICKED),8);
    CHECK(arm_motor_status[0].position_cdeg< -950 && arm_motor_status[0].position_cdeg> -1050,9);
    if(test==15){CHECK(command(11,3,0,0,0,10)==ROUTE_CONFLICT,150);CHECK(command(13,2,-1000,-1000,-1000,0)==ROUTE_OK && until(ARM_WORK_ALIGNED),151);CHECK(command(14,3,0,0,0,13)==ROUTE_OK && until(ARM_WORK_WAIT_GRIP),152);return 0;}
    if(test==16){CHECK(command(13,5,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY_LEAVE),160);CHECK(Route_CompleteWork(99,2,1)==ROUTE_OK,161);step(1);CHECK(station(103),162);put16(packet+4,2);CHECK(Route_OnAuxCommand(0x31,packet,32)==ROUTE_BAD_STATE,163);return 0;}
    if(test==17){Route_Stop();step(300);CHECK(arm_work_status.state==ARM_WORK_STOPPED,170);CHECK(home(),171);CHECK(Route_CheckStart()==ROUTE_OK,172);return 0;}
    if(test==18){CHECK(Route_OnAuxCommand(0x32,0,0)==ROUTE_OK && report_count>0,180);CHECK(report_bytes[3]==0x83 && report_bytes[6]==52,181);CHECK(Route_CRC16(report_bytes+2,58)==(uint16_t)(report_bytes[60]|report_bytes[61]<<8),182);CHECK(report_bytes[24]==ARM_WORK_PICKED && (report_bytes[28]&5)==5,183);return 0;}
    return 999;
}
