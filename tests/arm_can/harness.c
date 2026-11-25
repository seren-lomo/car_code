#include "arm_can.h"
CAN_HandleTypeDef hcan={0};
volatile HAL_StatusTypeDef can_last_tx_status=HAL_OK;
static uint32_t tick,mask,tx_count,stop_count,move_count;
static uint8_t response[3][8],response_n[3];
static uint32_t response_at[3];
static int32_t pos[3],target[3];
static uint8_t moving[3],drop_function,reject_function,stop_reject,tx_fail,stop_tx_fail;
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
    if(tx_fail || (stop_tx_fail && function==0xFE) || f[0]<5 || f[0]>7 || f[n-1]!=0x6B)return;
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
        ArmCAN_Task();
    }
}
static uint8_t home(uint8_t i)
{if(!ArmCAN_RecordOrigin(i))return 0;step(100);return arm_motor_status[i].homed;}
#define CHECK(c,n) do {if(!(c))return n;}while(0)
int run_case(uint32_t test)
{
    uint8_t i;
    CHECK(ArmCAN_Init()==HAL_OK,1);
    for(i=0;i<3;i++) {
        arm_motor_config[i].pulses_per_rev=3200;
        arm_motor_config[i].min_cdeg=-72000;arm_motor_config[i].max_cdeg=72000;
        pos[i]=1000*(i+1);target[i]=pos[i];
    }
    if(test==1) {step(1000);CHECK(tx_count==0 && !ArmCAN_Busy(),10);CHECK(!ArmCAN_Move(0,1000),11);}
    else if(test==2) {for(i=0;i<3;i++){CHECK(home(i),20+i);CHECK(arm_motor_status[i].origin_counts==1000*(i+1),24+i);}CHECK(move_count==0,28);}
    else if(test==3 || test==4) {
        CHECK(home(1),30);CHECK(ArmCAN_Move(1,test==3?9000:-9000),31);step(4000);
        CHECK(arm_motor_status[1].state==ARM_MOTOR_DONE && move_count==1,32);
        CHECK(arm_motor_status[1].position_cdeg>(test==3?8900:-9100) &&
              arm_motor_status[1].position_cdeg<(test==3?9100:-8900),33);
    } else if(test==5) {CHECK(home(0),50);CHECK(!ArmCAN_Move(0,72001),51);CHECK(!move_count,52);}
    else if(test==6) {
        CHECK(home(0),60);reject_function=0xFD;CHECK(ArmCAN_Move(0,9000),61);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_REPLY && !arm_motor_status[0].homed,62);
        CHECK(stop_count>0 && !moving[0],63);
    } else if(test==7) {
        CHECK(home(0),70);drop_function=0x36;CHECK(ArmCAN_Move(0,9000),71);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TIMEOUT && !move_count,72);
    } else if(test==8) {
        CHECK(home(2),80);CHECK(ArmCAN_Move(2,36000),81);step(100);ArmCAN_StopAll();step(500);
        CHECK(stop_count>=3 && !moving[2] && !arm_motor_status[2].homed,82);
        CHECK(!ArmCAN_Busy(),83);
    } else if(test==9) {
        CHECK(home(1),90);flags=5;CHECK(ArmCAN_Move(1,9000),91);step(500);
        CHECK(arm_motor_status[1].fault==ARM_MOTOR_FLAGS && !move_count,92);
        CHECK(arm_can_diag[1].fault_code==ARM_MOTOR_FLAGS && arm_can_diag[1].fault_flags==5 &&
              arm_can_diag[1].fault_stage==2 && arm_can_diag[1].fault_function==0x3A,93);
    } else if(test==10) {
        CHECK(ArmCAN_RecordOrigin(0),100);CHECK(!ArmCAN_RecordOrigin(1),101);step(100);
        CHECK(arm_motor_status[0].homed,102);
    } else if(test==11) {
        CHECK(home(0),110);tx_fail=1;CHECK(ArmCAN_Move(0,9000),111);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TX && !ArmCAN_Busy(),112);
    } else if(test==12) {
        CHECK(home(0),120);stop_reject=1;ArmCAN_StopAll();step(1000);
        CHECK(stop_count==9 && !ArmCAN_Busy(),121);
    } else if(test==13) {
        CHECK(home(0),130);CHECK(ArmCAN_Move(0,9000),131);step(100);
        tx_fail=1;step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TX && !arm_motor_status[0].homed,132);
    } else if(test==14) {
        CHECK(home(0),140);CHECK(ArmCAN_Move(0,9000),141);step(100);
        {uint8_t f[3]={0x3A,1,0x6B};
        for(i=0;i<128;i++) ArmCAN_RxFrame(0x500,f,3);}step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_RX_OVERFLOW && !moving[0],142);
    } else if(test==15) {
        CHECK(ArmCAN_Probe(1),150);step(100);
        CHECK(arm_motor_status[1].state==ARM_MOTOR_DONE && !arm_motor_status[1].homed,151);
        CHECK(tx_count==1 && !stop_count && !move_count && arm_motor_status[1].position_counts==2000,152);
        CHECK(arm_can_diag[1].tx_completed==1 && arm_can_diag[1].rx_bytes==7,153);
        CHECK(arm_can_diag[1].tx_raw[1]==0x36 && arm_can_diag[1].rx_raw[6]==0x6B,154);
    } else if(test==16) {
        drop_function=0x36;CHECK(ArmCAN_Probe(0),160);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TIMEOUT && !ArmCAN_Busy(),161);
        CHECK(tx_count==1 && !stop_count && !move_count && !arm_can_diag[0].rx_bytes,162);
        CHECK(arm_can_diag[0].timeout_function==0x36 && !arm_can_diag[0].timeout_tx_active && arm_can_diag[0].timeout_waiting,163);
        drop_function=0;CHECK(ArmCAN_Probe(1),164);step(100);
        CHECK(arm_motor_status[1].state==ARM_MOTOR_DONE,165);
    } else if(test==17) {
        CHECK(!ArmCAN_Probe(3),170);CHECK(ArmCAN_Probe(0),171);
        CHECK(!ArmCAN_Probe(1),172);tx_fail=1;step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TX && !stop_count && !move_count && !ArmCAN_Busy(),173);
    } else if(test==18) {
        uint8_t f[7]={0x36,0,0,0,0,42,0x6B};
        CHECK(ArmCAN_Probe(0),180);drop_function=0x36;step(5);
        ArmCAN_RxFrame(0x100,f,7);ArmCAN_RxFrame(0x600,f,7);step(1);
        CHECK(arm_motor_status[0].rx_frames==0 && arm_motor_status[1].rx_frames==1,181);
        ArmCAN_RxFrame(0x500,f,7);step(1);
        CHECK(arm_motor_status[0].state==ARM_MOTOR_DONE && arm_motor_status[0].position_counts==42,182);
    } else if(test==19) {
        uint8_t f[7]={0x36,0,0,0,0,42,0x6B};
        drop_function=0x36;CHECK(ArmCAN_Probe(0),190);step(5);
        ArmCAN_RxFrame(0x501,f,7);ArmCAN_RxFrame(0x500,f,6);
        f[6]=0;ArmCAN_RxFrame(0x500,f,7);step(1);
        CHECK(arm_motor_status[0].rx_frames==0 && arm_motor_status[0].rx_errors==3,191);
        step(100);CHECK(arm_motor_status[0].fault==ARM_MOTOR_TIMEOUT,192);
    } else if(test==20) {
        arm_motor_status[0].address=1;CHECK(!ArmCAN_Probe(0),200);
        arm_motor_status[0].address=6;CHECK(!ArmCAN_Probe(0),201);
        CHECK(!tx_count,202);
    } else if(test==21) {
        CHECK(home(2),210);CHECK(ArmCAN_Move(2,9000),211);step(4000);
        CHECK(arm_motor_status[2].state==ARM_MOTOR_DONE,212);
        CHECK(arm_can_diag[2].tx_can_frames==arm_can_diag[2].tx_completed+1,213);
    } else if(test==22) {
        drop_function=0xF3;stop_tx_fail=1;
        CHECK(ArmCAN_RecordOrigin(2),220);step(200);
        CHECK(arm_motor_status[2].fault==ARM_MOTOR_TIMEOUT && !ArmCAN_Busy(),221);
        CHECK(arm_can_diag[2].fault_code==ARM_MOTOR_TIMEOUT && arm_can_diag[2].fault_stage==1 &&
              arm_can_diag[2].fault_function==0xF3 && !move_count,222);
        CHECK(arm_can_diag[2].last_hal_status!=HAL_OK,223);
    } else return 999;
    return 0;
}
