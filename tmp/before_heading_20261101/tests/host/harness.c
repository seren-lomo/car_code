#include "chassis_debug.h"
int _fltused;
void *memset(void *out, int c, unsigned long long n)
{ unsigned char *p = out; while (n--) *p++ = (unsigned char)c; return out; }
void *memcpy(void *out, const void *in, unsigned long long n)
{ unsigned char *p=out; const unsigned char *q=in; while(n--) *p++=*q++; return out; }
volatile MotorFeedback_t motor_feedback[5];
volatile HAL_StatusTypeDef can_last_tx_status;
static uint32_t tick, key_level=1, stops, syncs, commands, bad_sync;
static uint8_t failed_fd, missing_read, missing_addr, tx_fail, clogged;
static uint8_t arrive, fast, dir[5], buffered[5];
static uint32_t count[5];
static int32_t actual[5], target[5];
uint32_t HAL_GetTick(void) { return tick++; }
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t HAL_GPIO_ReadPin(void *p, uint32_t pin)
{ (void)p; (void)pin; return key_level; }
static void Ack(uint8_t a, uint8_t f)
{
    can_last_tx_status = tx_fail ? HAL_ERROR : HAL_OK;
    if (a == failed_fd && f == 0xFD) return;
    motor_feedback[a].ack_function=f;
    motor_feedback[a].ack_status=0x02;
    ++motor_feedback[a].ack_count;
}
void Emm_V5_Stop_Now(uint8_t a, bool s)
{ (void)s; ++stops; target[a]=actual[a]; buffered[a]=0; Ack(a,0xFE); }
void Emm_V5_En_Control(uint8_t a, bool e, bool s)
{ (void)s; motor_feedback[a].flags=e?3:0; Ack(a,0xF3); }
void Emm_V5_Pos_Control(uint8_t a,uint8_t d,uint16_t rpm,uint8_t acc,
                        uint32_t p,uint8_t mode,bool s)
{
    (void)rpm; (void)acc;
    if(mode!=2 || !s) ++bad_sync;
    dir[a]=d; count[a]=p; buffered[a]=1; ++commands; Ack(a,0xFD);
}
void Emm_V5_Synchronous_motion(uint8_t a)
{
    uint8_t i;
    if(a!=0) ++bad_sync;
    for(i=1;i<=4;++i) if(!buffered[i]) ++bad_sync;
    ++syncs;
    for(i=1;i<=4;++i) {
        int32_t delta=(int32_t)((float)count[i]*65536.0f/3200.0f+0.5f);
        target[i]=actual[i]+(dir[i]?-delta:delta);
        buffered[i]=0;
    }
    can_last_tx_status=tx_fail?HAL_ERROR:HAL_OK;
}
void Emm_V5_Read_Sys_Params(uint8_t a, SysParams_t param)
{
    can_last_tx_status=HAL_OK;
    if(a==missing_addr && param==missing_read) return;
    if(param==S_FLAG) {
        motor_feedback[a].flags=clogged==a?9:3; /* deliberately stale reached=1 */
        motor_feedback[a].flags_tick=tick; ++motor_feedback[a].flags_count;
    } else if(param==S_CPOS) {
        if(arrive) actual[a]=target[a];
        motor_feedback[a].position=actual[a];
        motor_feedback[a].position_tick=tick; ++motor_feedback[a].position_count;
    } else if(param==S_VEL) {
        motor_feedback[a].speed_rpm=fast?20:0;
        motor_feedback[a].speed_tick=tick; ++motor_feedback[a].speed_count;
    }
}
static void Tasks(uint32_t n)
{ while(n--) { ++tick; Chassis_DebugTask(); } }
static void Polls(uint32_t n)
{ while(n--) { tick+=26; Chassis_Task(); } }
static void Press(void)
{ key_level=0; Tasks(40); key_level=1; Tasks(40); }
#define CHECK(x,n) do { if(!(x)) return n; } while(0)
__declspec(dllexport) int run_case(int test)
{
    uint32_t before, i;
    uint8_t order[4]={1,2,4,3};
    Chassis_DebugInit();
    if(test==1) {
        CHECK(stops==4 && !commands && chassis_motion.state==0,11);
        key_level=0; Tasks(10); key_level=1; Tasks(40);
        CHECK(!commands && !key0_press_count,12);
        Press(); CHECK(syncs==1 && commands==4 && chassis_debug.next_test==2,13);
        CHECK(chassis_motion.target_pulses==1340 && !bad_sync,14);
        key_level=0; Tasks(40);
        CHECK(chassis_motion.state==CHASSIS_STOPPED && stops>=12,15);
        before=commands; key_level=1; Tasks(40); CHECK(commands==before,16);
    } else if(test>=2 && test<=5) {
        uint8_t expected[4][4]={{0,1,0,1},{1,0,1,0},{1,1,0,0},{0,0,1,1}};
        chassis_debug.command=test-1; ++chassis_debug.trigger; Chassis_DebugTask();
        CHECK(syncs==1 && !bad_sync,21);
        for(i=0;i<4;++i) CHECK(dir[order[i]]==expected[test-2][i],22);
        arrive=1; Polls(8); CHECK(chassis_motion.state==CHASSIS_DONE,23);
        before=commands; Tasks(200); CHECK(commands==before,24);
    } else if(test==6) {
        failed_fd=3;
        CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_IO_ERROR,61);
        CHECK(!syncs && chassis_motion.fault==CHASSIS_ACK_ERROR,62);
        CHECK(chassis_motion.fault_address==3 && stops>=8,63);
    } else if(test==7 || test==8) {
        if(test==7) CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_OK,71);
        missing_addr=1; missing_read=test==7?S_FLAG:S_CPOS;
        if(test==7) Polls(1);
        else CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_IO_ERROR,72);
        CHECK(chassis_motion.state==CHASSIS_FAULT && chassis_motion.fault==5,73);
        if(test==8) CHECK(!syncs,74);
    } else if(test==9) {
        CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_OK,91);
        Polls(12); CHECK(chassis_motion.state==CHASSIS_RUNNING && !chassis_motion.reached_mask,92);
        tick+=chassis_motion.timeout_ms; Chassis_Task();
        CHECK(chassis_motion.state==CHASSIS_FAULT && chassis_motion.fault==6,93);
    } else if(test==10) {
        CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_OK,101);
        arrive=1; fast=1; Polls(8); CHECK(chassis_motion.state==CHASSIS_RUNNING,102);
        fast=0; Polls(7); CHECK(chassis_motion.state==CHASSIS_RUNNING,103);
        before=stops; Polls(1);
        CHECK(chassis_motion.state==CHASSIS_DONE && stops==before,104);
        CHECK(chassis_motion.reached_mask==15 && chassis_motion.completed_count==1,105);
    } else if(test==11) {
        CHECK(Chassis_MoveXY(100,100,20,100)==0,111);
        CHECK(Chassis_MoveForward(50,20,100)==CHASSIS_BUSY && commands==4,112);
        Chassis_Stop(); arrive=1; Polls(10);
        CHECK(syncs==1 && chassis_motion.state==CHASSIS_STOPPED,113);
    } else if(test==12) {
        CHECK(Chassis_MoveXY(100,-100,20,100)==0,121);
        chassis_config.wheel_diameter_mm=152; chassis_config.address[0]=2;
        arrive=1; Polls(8);
        CHECK(syncs==2 && chassis_motion.axis==1 && chassis_motion.target_pulses==1340,122);
        Polls(8); CHECK(chassis_motion.state==CHASSIS_DONE && !bad_sync,123);
    } else if(test==13) {
        CHECK(Chassis_MoveXY(0,0,20,100)==0 && !commands && !syncs,131);
        CHECK(chassis_motion.state==CHASSIS_DONE,132);
        CHECK(Chassis_MoveForward(3001,20,100)==CHASSIS_INVALID && !commands,133);
        CHECK(Chassis_MoveForward(10,0,100)==CHASSIS_INVALID && !commands,134);
        chassis_config.address[3]=1;
        CHECK(Chassis_MoveLeft(100,20,100)==CHASSIS_INVALID && !commands,135);
    } else if(test==14) {
        CHECK(Chassis_MoveForward(100,20,100)==0,141);
        actual[1]=1000; actual[2]=-1000; actual[4]=1000; actual[3]=-1000;
        Chassis_Stop(); CHECK(Chassis_MoveForward(50,20,100)==0,142);
        arrive=1; Polls(8);
        CHECK(chassis_motion.state==CHASSIS_DONE && syncs==2,143);
        CHECK(chassis_motion.measured_delta[0]==chassis_motion.expected_delta[0],144);
    } else if(test==15) {
        tx_fail=1; CHECK(Chassis_MoveForward(100,20,100)==CHASSIS_IO_ERROR,151);
        CHECK(chassis_motion.fault==2 && !syncs,152);
    } else if(test==16) {
        CHECK(Chassis_MoveForward(100,20,100)==0,161);
        clogged=1; Polls(1);
        CHECK(chassis_motion.fault==4 && chassis_motion.fault_address==1,162);
    } else if(test==17) {
        CHECK(Chassis_MoveForward(100,20,100)==0,171);
        chassis_debug.command=1; ++chassis_debug.trigger;
        /* Explicit command=0 is the debug stop request. */
        Chassis_DebugTask(); CHECK(commands==4,172);
        key_level=0; Tasks(850); key_level=1; Tasks(40);
        CHECK(chassis_debug.next_test==1 && commands==4 && !Chassis_IsBusy(),173);
    } else if(test==18) {
        chassis_config.forward_correction=1.1f;
        CHECK(Chassis_MoveForward(100,20,100)==0,181);
        CHECK(chassis_motion.target_pulses==1474,182);
    } else if(test==19) {
        union {uint32_t u; float f;} nan;
        nan.u=0x7fc00000;
        CHECK(Chassis_MoveForward(nan.f,20,100)==CHASSIS_INVALID && !commands,191);
        CHECK(Chassis_MoveForward(0.000001f,20,100)==CHASSIS_INVALID && !commands,192);
        CHECK(Chassis_MoveForward(3000,1,100)==CHASSIS_INVALID && !commands,193);
    } else return 999;
    return 0;
}
