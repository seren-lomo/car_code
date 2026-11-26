#include "arm_work.h"
#include "arm_config.h"
CAN_HandleTypeDef hcan={0};
volatile HAL_StatusTypeDef can_last_tx_status=HAL_OK;
static uint32_t tick,mask,tx_count,stop_count,move_count;
static uint8_t response[3][8],response_n[3];
static uint32_t response_at[3];
static int32_t pos[3],target[3];
static uint8_t moving[3],drop_function,reject_function,stop_reject,tx_fail;
static uint8_t flags=1;
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
        move_count++;
        pulses=((uint32_t)f[6]<<24)|((uint32_t)f[7]<<16)|((uint32_t)f[8]<<8)|f[9];
        target[i]=pos[i]+(f[2]?-1:1)*(int32_t)(pulses*65536U/3200U);moving[i]=1;
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
    uint8_t i,n;
    while(ms--) {
        tick++;
        for(i=0;i<3;i++) {
            if(moving[i]) {
                if(target[i]-pos[i]>10)pos[i]+=10;
                else if(pos[i]-target[i]>10)pos[i]-=10;
                else {pos[i]=target[i];moving[i]=0;}
            }
            if(response_n[i] && tick>=response_at[i]) {
                n=response_n[i];response_n[i]=0;
                ArmCAN_RxFrame((uint32_t)(5U+i)<<8,response[i]+1,(uint8_t)(n-1U));
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
{uint32_t i;for(i=0;i<30000;i++){if(arm_work_status.state==state)return 1;if(arm_work_status.state==ARM_WORK_FAULT)return 0;step(1);}return 0;}
static uint8_t home(void)
{return command(1,1,0,0,0,0)==ROUTE_OK && until(ARM_WORK_READY);}
static uint8_t station(uint16_t work)
{
    route_status.route_id=99;route_status.point_index=2;route_status.work_id=work;route_status.state=ROUTE_WAIT_WORK;
    Route_OnWork(99,2,work);return until(ARM_WORK_WAIT_VISION);
}
static uint8_t align(void)
{return command(10,2,-1000,2000,-3000,0)==ROUTE_OK && until(ARM_WORK_ALIGNED);}
#define CHECK(c,n) do {if(!(c))return n;}while(0)
int run_case(uint32_t test)
{
    uint8_t i;uint32_t before;
    CHECK(ArmWork_Init()==HAL_OK,1);
    for(i=0;i<3;i++){pos[i]=1000*(i+1);target[i]=pos[i];}
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
    CHECK(arm_motor_status[0].position_cdeg< -9900 && arm_motor_status[1].position_cdeg>1950 && arm_motor_status[1].position_cdeg<2050,7);
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
