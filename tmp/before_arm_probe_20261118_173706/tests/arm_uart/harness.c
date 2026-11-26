#include "arm_uart.h"
UART_HandleTypeDef huart1={0},huart2={1},huart3={2};
static uint32_t tick,mask,tx_count,stop_count,move_count;
static uint8_t *rx[3],response[3][8],response_n[3];
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
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{if(n!=1)return HAL_ERROR;rx[u->index]=p;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{response_n[u->index]=0;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *f,uint16_t n)
{
    uint8_t i=(uint8_t)u->index,*r=response[i],function=f[1];
    uint32_t v,pulses;
    tx_count++;
    if(tx_fail)return HAL_ERROR;
    if(f[0]!=5+i || f[n-1]!=0x6B)return HAL_ERROR;
    r[0]=f[0];r[1]=function;r[2]=2;r[3]=0x6B;response_n[i]=4;
    if(function==0xFE) {stop_count++;moving[i]=0;target[i]=pos[i];if(stop_reject)r[2]=0xE2;}
    if(function==0xFD) {
        if(n!=13 || f[10]!=2 || f[11]!=0)return HAL_ERROR;
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
    ArmUART_TxCpltCallback(u);
    return HAL_OK;
}
static UART_HandleTypeDef *uart(uint8_t i) {return i==0?&huart1:(i==1?&huart2:&huart3);}
static void step(uint32_t ms)
{
    uint8_t i,j,n;
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
                for(j=0;j<n;j++) {*rx[i]=response[i][j];ArmUART_RxCpltCallback(uart(i));}
            }
        }
        ArmUART_Task();
    }
}
static uint8_t home(uint8_t i)
{if(!ArmUART_RecordOrigin(i))return 0;step(100);return arm_motor_status[i].homed;}
#define CHECK(c,n) do {if(!(c))return n;}while(0)
int run_case(uint32_t test)
{
    uint8_t i;
    CHECK(ArmUART_Init()==HAL_OK,1);
    for(i=0;i<3;i++) {
        arm_motor_config[i].pulses_per_rev=3200;
        arm_motor_config[i].min_cdeg=-72000;arm_motor_config[i].max_cdeg=72000;
        pos[i]=1000*(i+1);target[i]=pos[i];
    }
    if(test==1) {step(1000);CHECK(tx_count==0 && !ArmUART_Busy(),10);CHECK(!ArmUART_Move(0,1000),11);}
    else if(test==2) {for(i=0;i<3;i++){CHECK(home(i),20+i);CHECK(arm_motor_status[i].origin_counts==1000*(i+1),24+i);}CHECK(move_count==0,28);}
    else if(test==3 || test==4) {
        CHECK(home(1),30);CHECK(ArmUART_Move(1,test==3?9000:-9000),31);step(4000);
        CHECK(arm_motor_status[1].state==ARM_MOTOR_DONE && move_count==1,32);
        CHECK(arm_motor_status[1].position_cdeg>(test==3?8900:-9100) &&
              arm_motor_status[1].position_cdeg<(test==3?9100:-8900),33);
    } else if(test==5) {CHECK(home(0),50);CHECK(!ArmUART_Move(0,72001),51);CHECK(!move_count,52);}
    else if(test==6) {
        CHECK(home(0),60);reject_function=0xFD;CHECK(ArmUART_Move(0,9000),61);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_REPLY && !arm_motor_status[0].homed,62);
        CHECK(stop_count>0 && !moving[0],63);
    } else if(test==7) {
        CHECK(home(0),70);drop_function=0x36;CHECK(ArmUART_Move(0,9000),71);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TIMEOUT && !move_count,72);
    } else if(test==8) {
        CHECK(home(2),80);CHECK(ArmUART_Move(2,36000),81);step(100);ArmUART_StopAll();step(500);
        CHECK(stop_count>=3 && !moving[2] && !arm_motor_status[2].homed,82);
        CHECK(!ArmUART_Busy(),83);
    } else if(test==9) {
        CHECK(home(1),90);flags=5;CHECK(ArmUART_Move(1,9000),91);step(500);
        CHECK(arm_motor_status[1].fault==ARM_MOTOR_FLAGS && !move_count,92);
    } else if(test==10) {
        CHECK(ArmUART_RecordOrigin(0),100);CHECK(!ArmUART_RecordOrigin(1),101);step(100);
        CHECK(arm_motor_status[0].homed,102);
    } else if(test==11) {
        CHECK(home(0),110);tx_fail=1;CHECK(ArmUART_Move(0,9000),111);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_TX && !ArmUART_Busy(),112);
    } else if(test==12) {
        CHECK(home(0),120);stop_reject=1;ArmUART_StopAll();step(1000);
        CHECK(stop_count==9 && !ArmUART_Busy(),121);
    } else if(test==13) {
        CHECK(home(0),130);CHECK(ArmUART_Move(0,9000),131);step(100);
        ArmUART_ErrorCallback(&huart1);step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_UART && !moving[0] && !arm_motor_status[0].homed,132);
    } else if(test==14) {
        CHECK(home(0),140);CHECK(ArmUART_Move(0,9000),141);step(100);
        for(i=0;i<128;i++) {*rx[0]=0x55;ArmUART_RxCpltCallback(&huart1);}step(500);
        CHECK(arm_motor_status[0].fault==ARM_MOTOR_RX_OVERFLOW && !moving[0],142);
    } else return 999;
    return 0;
}
