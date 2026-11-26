#include "chassis_heading.h"
#include "chassis_debug.h"
#include "imu.h"

int _fltused;
void *memset(void *out, int c, unsigned long long n)
{ unsigned char *p=out; while(n--) *p++=(unsigned char)c; return out; }
void *memcpy(void *out, const void *in, unsigned long long n)
{ unsigned char *p=out; const unsigned char *q=in; while(n--) *p++=*q++; return out; }

UART_HandleTypeDef huart5;
volatile MotorFeedback_t motor_feedback[5];
volatile HAL_StatusTypeDef can_last_tx_status;
static uint8_t *rx_destination;
static uint32_t tick, primask, key_level=1U;
static uint32_t stops, syncs, velocity_commands, position_commands, max_command;
static uint32_t invalid_batches, missing_function, missing_address, tx_fail_function;
static uint8_t enabled[5], queued_mask, clog_address, stall_address;
static uint8_t asymmetric, sensor_enabled=1U, negative_translation;
static float commanded[5], buffered[5], speed[5], pos[5];
static float yaw, true_forward, true_left, peak_error;
static int8_t sensor_sign=1;
static uint32_t requests[5];
static uint8_t request_function[5];
__declspec(dllexport) float result_distance, result_yaw, result_peak_error;
__declspec(dllexport) uint32_t result_ms, result_cycles;
__declspec(dllexport) float result_turned;

static float af(float x) { return x<0 ? -x : x; }
static float wrap(float v) { while(v>=180) v-=360; while(v< -180) v+=360; return v; }
static int32_t round_int(float v) { return (int32_t)(v<0 ? v-0.5f : v+0.5f); }
static int8_t sign(uint8_t a) { return a==1 || a==4 ? 1 : -1; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1U; }
void __set_PRIMASK(uint32_t p) { primask=p; }
void __DMB(void) {}
uint32_t HAL_GPIO_ReadPin(void *port, uint32_t pin)
{ (void)port; (void)pin; return key_level; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
    if (u!=&huart5 || n!=1) return HAL_ERROR;
    if (u->RxState!=HAL_UART_STATE_READY) return HAL_BUSY;
    rx_destination=p; u->RxState=HAL_UART_STATE_BUSY_RX; return HAL_OK;
}
static void sensor_frame(void)
{
    uint8_t f[11]={0x55,0x53,0,0,0,0,0,0,0,0,0}, i;
    int16_t raw=(int16_t)(wrap(yaw*sensor_sign)*(32768.0f/180.0f));
    f[6]=(uint8_t)raw; f[7]=(uint8_t)((uint16_t)raw>>8);
    for(i=0;i<10;i++) f[10]=(uint8_t)(f[10]+f[i]);
    for(i=0;i<11;i++) {
        *rx_destination=f[i]; huart5.RxState=HAL_UART_STATE_READY;
        IMU_UART_RxCpltCallback(&huart5);
    }
    IMU_Task();
}
static void ack(uint8_t a, uint8_t f)
{
    can_last_tx_status=tx_fail_function==f ? HAL_ERROR : HAL_OK;
    if(missing_address==a && missing_function==f) return;
    motor_feedback[a].ack_function=f;
    motor_feedback[a].ack_status=2U;
    motor_feedback[a].ack_count++;
}
void Emm_V5_Stop_Now(uint8_t a, bool sync)
{
    (void)sync; stops++;
    commanded[a]=buffered[a]=0;
    queued_mask &= (uint8_t)~(1U<<(a-1U));
    ack(a,0xFE);
}
void Emm_V5_En_Control(uint8_t a, bool on, bool sync)
{ (void)sync; enabled[a]=on; ack(a,0xF3); }
void Emm_V5_Vel_Control(uint8_t a,uint8_t d,uint16_t rpm,uint8_t acc,bool sync)
{
    velocity_commands++;
    if(!sync || acc!=0) invalid_batches++;
    if(rpm>max_command) max_command=rpm;
    buffered[a]=d?-(float)rpm:(float)rpm;
    queued_mask |= (uint8_t)(1U<<(a-1U));
    ack(a,0xF6);
}
void Emm_V5_Pos_Control(uint8_t a,uint8_t d,uint16_t rpm,uint8_t acc,
                        uint32_t pulses,uint8_t mode,bool sync)
{
    (void)d;(void)rpm;(void)acc;(void)pulses;(void)mode;(void)sync;
    position_commands++; queued_mask |= (uint8_t)(1U<<(a-1U)); ack(a,0xFD);
}
void Emm_V5_Synchronous_motion(uint8_t a)
{
    uint8_t i;
    if(a!=0 || queued_mask!=15U) invalid_batches++;
    can_last_tx_status=tx_fail_function==0xFF ? HAL_ERROR : HAL_OK;
    if(can_last_tx_status!=HAL_OK) return;
    syncs++;
    for(i=1;i<=4;i++) commanded[i]=buffered[i];
    if((commanded[1]-commanded[2]+commanded[4]-commanded[3])*0.25f < -1)
        negative_translation=1;
    queued_mask=0U;
}
static void respond(uint8_t a,uint8_t f)
{
    if(missing_address==a && missing_function==f) return;
    if(f==0x36) {
        motor_feedback[a].position=round_int(pos[a]);
        motor_feedback[a].position_tick=tick; motor_feedback[a].position_count++;
    } else if(f==0x35) {
        motor_feedback[a].speed_rpm=(int16_t)round_int(speed[a]);
        motor_feedback[a].speed_tick=tick; motor_feedback[a].speed_count++;
    } else if(f==0x3A) {
        motor_feedback[a].flags=clog_address==a ? 9U : (enabled[a]?1U:0U);
        motor_feedback[a].flags_tick=tick; motor_feedback[a].flags_count++;
    }
}
void Emm_V5_Read_Sys_Params(uint8_t a,SysParams_t param)
{
    uint8_t f=param==S_CPOS?0x36:(param==S_VEL?0x35:0x3A);
    can_last_tx_status=tx_fail_function==f?HAL_ERROR:HAL_OK;
    /* Keep real asynchronous response behavior in the new control path. */
    if(chassis_motion.control_mode==1U) {
        requests[a]=tick+2U; request_function[a]=f;
    } else respond(a,f);
}
static void advance(uint32_t ms,uint8_t debug)
{
    while(ms--) {
        float q[4], f,l,r;
        uint8_t a;
        tick++;
        for(a=1;a<=4;a++) {
            float gain=asymmetric ? ((a==1 || a==4)?0.96f:1.04f) : 1.0f;
            float desired=enabled[a] && stall_address!=a ? commanded[a]*gain : 0.0f;
            speed[a] += 0.1f*(desired-speed[a]);
            pos[a] += speed[a]*(65536.0f/60000.0f);
            if(request_function[a] && (int32_t)(tick-requests[a])>=0) {
                uint8_t fn=request_function[a]; request_function[a]=0;
                respond(a,fn);
            }
        }
        q[0]=speed[1]*sign(1); q[1]=speed[2]*sign(2);
        q[2]=speed[4]*sign(4); q[3]=speed[3]*sign(3);
        f=(q[0]+q[1]+q[2]+q[3])*0.25f;
        l=(-q[0]+q[1]+q[2]-q[3])*0.25f;
        r=(-q[0]+q[1]-q[2]+q[3])*0.25f;
        true_forward += f*(3.14159265358979323846f*76.0f/60000.0f);
        true_left += l*(3.14159265358979323846f*76.0f/60000.0f);
        yaw += r*0.0012f;
        if(sensor_enabled && tick%10U==0U) sensor_frame();
        if(tick%2U==0U) IMU_Task();
        if(debug) Chassis_DebugTask(); else Chassis_Task();
        if(af(chassis_heading.yaw_error_deg)>peak_error) peak_error=af(chassis_heading.yaw_error_deg);
    }
}
void HAL_Delay(uint32_t ms) { tick+=ms; }
static void init(float initial_yaw)
{
    yaw=initial_yaw;
    IMU_Init(); sensor_frame(); Chassis_DebugInit();
    chassis_debug.command=1; /* Application test uses Chassis_Task directly. */
}
static void finish(uint32_t limit)
{ while(Chassis_IsBusy() && limit--) advance(1,0); }
#define CHECK(x,n) do { if(!(x)) return n; } while(0)
__declspec(dllexport) int run_case(int test)
{
    uint32_t before, t;
    ChassisResult_t result;
    init(test==1 ? 179.0f : 0.0f);
    if(test>=1 && test<=4) {
        float mm=(test==2 || test==4)?-500.0f:500.0f;
        asymmetric=1;
        result=test<=2?Chassis_MoveForwardHold(mm,50):Chassis_MoveLeftHold(mm,50);
        CHECK(result==CHASSIS_OK,11);
        advance(700,0); yaw+=4.0f; /* External yaw disturbance including +180 wrap. */
        finish(25000);
        CHECK(chassis_motion.state==CHASSIS_DONE,12);
        CHECK(af(chassis_heading.traveled_mm-mm)<=3.1f,13);
        CHECK(af(chassis_heading.yaw_error_deg)<=1.01f,14);
        CHECK(peak_error>=3.0f && max_command<=50U && !invalid_batches,15);
        CHECK(!position_commands && syncs>20 && chassis_heading.settled_ms>=250,16);
    } else if(test==5) {
        chassis_config.forward_correction=1.1f;
        CHECK(Chassis_MoveForwardHold(100,20)==0,51); finish(20000);
        CHECK(chassis_motion.state==CHASSIS_DONE,52);
        CHECK(af(chassis_heading.traveled_mm-100)<=3.1f && true_forward>104,53);
    } else if(test==6) {
        CHECK(Chassis_MoveForwardHold(500,20)==0,61);
        CHECK(Chassis_MoveLeftHold(100,20)==CHASSIS_BUSY,62);
        CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_BUSY,63);
        advance(300,0); Chassis_Stop();
        CHECK(chassis_motion.state==CHASSIS_STOPPED && !queued_mask,64);
        CHECK(Chassis_MoveForward(100,20,100)==0 && chassis_motion.control_mode==0,65);
        CHECK(Chassis_MoveLeftHold(100,20)==CHASSIS_BUSY,66);
        Chassis_Stop();
    } else if(test==7) {
        IMU_Init();
        CHECK(Chassis_MoveForwardHold(500,20)==CHASSIS_IO_ERROR,71);
        CHECK(chassis_motion.fault==CHASSIS_IMU_OFFLINE && !velocity_commands,72);
    } else if(test==8) {
        CHECK(Chassis_MoveForwardHold(500,20)==0,81); advance(500,0);
        sensor_enabled=0; t=tick; advance(210,0);
        CHECK(chassis_motion.state==CHASSIS_FAULT && chassis_motion.fault==7,82);
        CHECK(!queued_mask && stops>=8 && tick-t==210,83);
    } else if(test==9 || test==10 || test==20) {
        if(test==20) { /* Stale preexisting counts must not satisfy a new read. */
            motor_feedback[1].position_count=123;
            missing_address=1; missing_function=0x36;
        }
        CHECK(Chassis_MoveForwardHold(500,20)==0,91);
        if(test!=20) {
            advance(500,0);
            missing_address=test==9?2:4;
            missing_function=test==9?0x36:0xF6;
        }
        advance(200,0);
        CHECK(chassis_motion.state==CHASSIS_FAULT,92);
        CHECK(chassis_motion.fault==(test==10?CHASSIS_ACK_ERROR:CHASSIS_FEEDBACK_ERROR),93);
        CHECK(!queued_mask && stops>=8,94);
    } else if(test==11) {
        CHECK(Chassis_MoveForwardHold(500,20)==0,111); tx_fail_function=0xFF;
        advance(200,0);
        CHECK(chassis_motion.fault==CHASSIS_TX_ERROR && !syncs && !queued_mask,112);
    } else if(test==12) {
        CHECK(Chassis_MoveForwardHold(500,20)==0,121);
        advance(300,0); clog_address=3; advance(100,0);
        CHECK(chassis_motion.fault==CHASSIS_MOTOR_ERROR && chassis_motion.fault_address==3,122);
    } else if(test==13) {
        stall_address=4; CHECK(Chassis_MoveForwardHold(500,20)==0,131);
        advance(1600,0);
        CHECK(chassis_motion.fault==CHASSIS_WHEEL_STALLED && chassis_motion.fault_address==4,132);
    } else if(test==14) {
        chassis_heading_config.imu_yaw_sign=-1;
        CHECK(Chassis_MoveForwardHold(500,50)==0,141);
        advance(500,0); yaw+=4; finish(15000);
        CHECK(chassis_motion.fault==CHASSIS_HEADING_ERROR && stops>=8,142);
    } else if(test==15) {
        union {uint32_t u; float f;} nan; nan.u=0x7fc00000U;
        CHECK(Chassis_MoveForwardHold(nan.f,20)==CHASSIS_INVALID,151);
        CHECK(Chassis_MoveForwardHold(500,0)==CHASSIS_INVALID,152);
        chassis_config.address[3]=1;
        CHECK(Chassis_MoveForwardHold(500,20)==CHASSIS_INVALID,153);
        chassis_config.address[3]=3; chassis_heading_config.yaw_kp=nan.f;
        CHECK(Chassis_MoveForwardHold(500,20)==CHASSIS_INVALID && !velocity_commands,154);
    } else if(test==16) {
        CHECK(Chassis_MoveForwardHold(3000,1)==CHASSIS_INVALID,161);
        CHECK(chassis_motion.fault==CHASSIS_CONFIG_ERROR && !velocity_commands,162);
    } else if(test==17) {
        chassis_debug.command=0;
        key_level=0; advance(40,1); key_level=1; advance(40,1);
        CHECK(Chassis_IsBusy() && chassis_debug.next_test==2 && chassis_debug.use_imu,171);
        advance(500,1); missing_address=1; missing_function=0x36;
        key_level=0; advance(40,1);
        CHECK(chassis_motion.state==CHASSIS_STOPPED && !queued_mask,172);
        before=syncs; key_level=1; advance(100,1);
        CHECK(syncs==before && !Chassis_IsBusy(),173);
        key_level=0; advance(850,1); key_level=1; advance(40,1);
        CHECK(chassis_debug.next_test==1 && !Chassis_IsBusy(),174);
    } else if(test==18) {
        CHECK(Chassis_MoveForwardHold(100,20)==0,181); advance(300,0);
        chassis_config.address[0]=2; chassis_config.forward_sign[0]=-1;
        chassis_heading_config.imu_yaw_sign=-1;
        yaw+=3; finish(20000);
        CHECK(chassis_motion.state==CHASSIS_DONE && !invalid_batches,182);
    } else if(test==19) {
        CHECK(Chassis_MoveForwardHold(500,10)==0,191); advance(1000,0); yaw+=15;
        advance(1000,0);
        CHECK(Chassis_IsBusy() && max_command<=10 && chassis_heading.correction_rpm<0,192);
        Chassis_Stop();
    } else if(test==21) {
        CHECK(Chassis_MoveForwardHold(100,20)==0,211); advance(500,0);
        /* Simulate being pushed ahead; reverse gently to recover the endpoint. */
        pos[1]+=50000; pos[2]-=50000; pos[4]+=50000; pos[3]-=50000;
        finish(20000);
        CHECK(chassis_motion.state==CHASSIS_DONE && negative_translation,212);
        CHECK(af(chassis_heading.traveled_mm-100)<=3.1f,213);
    } else if(test==22) {
        CHECK(Chassis_MoveForwardHold(0,20)==0,221); finish(1000);
        CHECK(chassis_motion.state==CHASSIS_DONE && max_command==0,222);
        before=syncs; advance(1000,0); CHECK(syncs==before,223);
    } else if(test==23) {
        CHECK(Chassis_MoveForwardHold(500,20)==0,231); advance(500,0);
        IMU_ZeroYaw(); advance(100,0);
        CHECK(Chassis_IsBusy() && af(chassis_heading.yaw_error_deg)<1,232);
        chassis_motion.timeout_ms=chassis_motion.elapsed_ms+20;
        advance(30,0); CHECK(chassis_motion.fault==CHASSIS_MOVE_TIMEOUT,233);
    } else if(test==24) {
        CHECK(Chassis_MoveForwardHold(500,20)==0,241);
        missing_address=1; missing_function=0xF3; advance(200,0);
        CHECK(chassis_motion.fault==CHASSIS_ACK_ERROR && !velocity_commands && !syncs,242);
    } else if(test==25) {
        chassis_debug.command=0;
        CHECK(Chassis_MoveForwardHold(100,20)==0,251);
        advance(500,1);
        CHECK(Chassis_IsBusy() && chassis_heading.traveled_mm>1,252);
        /* A command=0 trigger explicitly stops even if it was already zero. */
        chassis_debug.trigger++; advance(1,1);
        CHECK(chassis_motion.state==CHASSIS_STOPPED,253);
    } else if(test==26) {
        CHECK(Chassis_MoveForwardHold(100,20)==0,261);
        while(chassis_heading.cycles<1U && Chassis_IsBusy()) advance(1,0);
        chassis_debug.command=1; advance(1,1);
        chassis_debug.command=0; advance(1,1);
        CHECK(chassis_motion.state==CHASSIS_STOPPED,262);
        before=syncs; advance(500,1); CHECK(syncs==before,263);
    } else if(test==27) {
        /* Unsigned time arithmetic must work when HAL tick wraps in motion. */
        tick=0xFFFFFF00U; sensor_frame();
        CHECK(Chassis_MoveForwardHold(50,20)==0,271);
        finish(15000);
        CHECK(chassis_motion.state==CHASSIS_DONE,272);
    } else if(test==28) {
        CHECK(Chassis_MoveForwardHold(100,20)==0,281);
        finish(15000); CHECK(chassis_motion.state==CHASSIS_DONE,282);
        CHECK(Chassis_MoveForwardHold(-100,20)==0,283);
        finish(15000);
        CHECK(chassis_motion.state==CHASSIS_DONE && af(chassis_heading.traveled_mm+100)<=3.1f,284);
        CHECK(chassis_motion.completed_count==2 && af(true_forward)<1,285);
    } else if(test>=29 && test<=34) {
        float target=test==29?90.0f:(test==30?-90.0f:(test==31?180.0f:
                     (test==32?-180.0f:(test==33?360.0f:-360.0f))));
        yaw=test%2?170.0f:-170.0f; sensor_frame();
        CHECK(Chassis_TurnRelative(target,20)==0,291);
        advance(300,0);
        CHECK(chassis_heading.turned_deg*target>0,292);
        finish(50000);
        CHECK(chassis_motion.state==CHASSIS_DONE,293);
        CHECK(af(chassis_heading.turned_deg-target)<=1.01f,294);
        CHECK(max_command<=20 && !invalid_batches && !position_commands,295);
        CHECK(af(true_forward)<1 && af(true_left)<1 && chassis_heading.translation_rpm==0,296);
    } else if(test==35) {
        sensor_sign=-1; chassis_heading_config.imu_yaw_sign=-1;
        sensor_frame();
        CHECK(Chassis_TurnAround(20)==0,351); finish(30000);
        CHECK(chassis_motion.state==CHASSIS_DONE && chassis_heading.turned_deg>179 && yaw>179,352);
    } else if(test==36) {
        CHECK(Chassis_TurnRelative(-90,20)==0,361);
        advance(1000,0); before=syncs; Chassis_Stop(); advance(500,0);
        CHECK(chassis_motion.state==CHASSIS_STOPPED && syncs==before && !queued_mask,362);
    } else if(test==37) {
        CHECK(Chassis_TurnRelative(90,20)==0,371);
        advance(1000,0); sensor_enabled=0; advance(220,0);
        CHECK(chassis_motion.fault==CHASSIS_IMU_OFFLINE && stops>=8,372);
    } else if(test==38) {
        chassis_heading_config.imu_yaw_sign=-1;
        CHECK(Chassis_TurnRelative(90,20)==0,381); finish(5000);
        CHECK(chassis_motion.fault==CHASSIS_HEADING_ERROR && stops>=8,382);
    } else if(test==39) {
        CHECK(Chassis_TurnRelative(361,20)==CHASSIS_INVALID,391);
        CHECK(Chassis_TurnRelative(-361,20)==CHASSIS_INVALID,392);
        CHECK(Chassis_TurnRelative(90,0)==CHASSIS_INVALID,393);
        CHECK(Chassis_TurnRelative(0,20)==0,394); finish(2000);
        CHECK(chassis_motion.state==CHASSIS_DONE && max_command==0,395);
    } else if(test==40) {
        chassis_debug.command=0; chassis_debug.next_test=5;
        key_level=0; advance(40,1); key_level=1; advance(40,1);
        CHECK(Chassis_IsBusy() && chassis_heading.action==1 && chassis_heading.target_turn_deg==90,401);
        while(Chassis_IsBusy() && tick<20000) advance(1,1);
        CHECK(chassis_motion.state==CHASSIS_DONE && chassis_debug.next_test==1,402);
        chassis_debug.include_turn_tests=1;
        chassis_debug.command=6; chassis_debug.trigger++; advance(1,1);
        CHECK(chassis_heading.target_turn_deg==-90 && chassis_debug.next_test==7,403);
        Chassis_Stop(); chassis_debug.command=7; chassis_debug.trigger++; advance(1,1);
        CHECK(chassis_heading.target_turn_deg==180 && chassis_debug.next_test==1,404);
        Chassis_Stop();
    } else if(test==41) {
        /* Increased translation distance must remain supported. */
        CHECK(Chassis_MoveForwardHold(2000,50)==0,411); finish(40000);
        CHECK(chassis_motion.state==CHASSIS_DONE && af(chassis_heading.traveled_mm-2000)<=3.1f,412);
    } else if(test==42) {
        CHECK(Chassis_MoveForwardHold(100,20)==0,421); finish(15000);
        CHECK(chassis_motion.state==CHASSIS_DONE,422);
        CHECK(Chassis_TurnRelative(90,20)==0,423); finish(15000);
        CHECK(chassis_motion.state==CHASSIS_DONE && chassis_heading.turned_deg>89,424);
        CHECK(Chassis_MoveForwardHold(100,20)==0,425); finish(15000);
        CHECK(chassis_motion.state==CHASSIS_DONE && af(chassis_heading.yaw_error_deg)<=1,426);
        CHECK(chassis_motion.completed_count==3,427);
    } else return 999;
    result_distance=chassis_heading.traveled_mm;
    result_yaw=chassis_heading.yaw_error_deg;
    result_peak_error=peak_error;
    result_ms=tick;
    result_cycles=chassis_heading.cycles;
    result_turned=chassis_heading.turned_deg;
    return 0;
}
