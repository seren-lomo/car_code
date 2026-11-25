#include "can.h"
#include "arm_can.h"

static CAN_TypeDef peripheral;
static uint32_t tick, mode, tx_count, aborted, available;
static CAN_TxHeaderTypeDef sent[4];
static uint8_t bytes[4][8], incoming[8];
static CAN_RxHeaderTypeDef received;

uint32_t HAL_GetTick(void) { return tick++; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *handle) {return handle->ErrorCode;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *handle)
{ (void)handle;return mode==1U?0U:3U; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle,
    const CAN_TxHeaderTypeDef *header,const uint8_t data[],uint32_t *mailbox)
{
    uint32_t i;
    if(mode==2U)return HAL_ERROR;
    if(tx_count>=4U)return HAL_ERROR;
    sent[tx_count]=*header;
    for(i=0;i<header->DLC;i++)bytes[tx_count][i]=data[i];
    tx_count++;*mailbox=CAN_TX_MAILBOX0;
    handle->Instance->TSR=mode==4U?0U:CAN_TSR_TXOK0;
    return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle,uint32_t mailbox)
{(void)handle;(void)mailbox;return mode==3U;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle,uint32_t mailbox)
{(void)handle;(void)mailbox;aborted++;return HAL_OK;}
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *handle,uint32_t fifo)
{(void)handle;(void)fifo;return available;}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *handle,uint32_t fifo,
    CAN_RxHeaderTypeDef *header,uint8_t data[])
{
    uint32_t i;(void)handle;(void)fifo;*header=received;
    for(i=0;i<8;i++)data[i]=incoming[i];
    available=0;return HAL_OK;
}
void *__aeabi_memclr4(void *out,uint32_t n)
{uint8_t *p=out;while(n--)*p++=0;return out;}
void *__aeabi_memclr(void *out,uint32_t n) {return __aeabi_memclr4(out,n);}
void *__aeabi_memcpy(void *out,const void *in,uint32_t n)
{uint8_t *p=out;const uint8_t *q=in;while(n--)*p++=*q++;return out;}
void *__aeabi_memcpy4(void *out,const void *in,uint32_t n)
{return __aeabi_memcpy(out,in,n);}
__attribute__((naked)) void __aeabi_ldivmod(void) {__asm volatile("bx lr");}
__attribute__((naked)) void __aeabi_uldivmod(void) {__asm volatile("bx lr");}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
static void receive(uint32_t id,uint32_t ide,uint32_t rtr,uint32_t length)
{
    received.ExtId=id;received.IDE=ide;received.RTR=rtr;received.DLC=length;
    available=1;HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
}
#define CHECK(c,n) do {if(!(c))return n;}while(0)
int run_case(uint32_t test)
{
    uint32_t i;
    uint8_t query[3]={5,0x36,0x6B};
    uint8_t motion[13]={5,0xFD,0,0,10,100,0,0,1,0,2,0,0x6B};
    hcan.Instance=&peripheral;ArmCAN_Init();
    for(i=0;i<3;i++)arm_motor_config[i].pulses_per_rev=3200;
    if(test==1U) {
        can_SendCmd(motion,13);
        CHECK(can_last_tx_status==HAL_OK && tx_count==2,10);
        CHECK(sent[0].IDE==CAN_ID_EXT && sent[0].ExtId==0x500 && sent[0].DLC==8,11);
        CHECK(sent[1].ExtId==0x501 && sent[1].DLC==5,12);
        CHECK(bytes[0][0]==0xFD && bytes[1][0]==0xFD,13);
        for(i=0;i<7;i++)CHECK(bytes[0][i+1]==motion[i+2],14);
        for(i=0;i<4;i++)CHECK(bytes[1][i+1]==motion[i+9],15);
    } else if(test>=2U && test<=5U) {
        mode=test-1U;can_SendCmd(query,3);
        CHECK(can_last_tx_status==((mode==1U || mode==3U)?HAL_TIMEOUT:HAL_ERROR),20);
        CHECK(can_tx_error_count==1,21);
        CHECK(aborted==(mode==3U),22);
    } else if(test==6U) {
        CHECK(ArmCAN_Probe(0),60);ArmCAN_Task();
        CHECK(tx_count==1 && sent[0].ExtId==0x500 && sent[0].DLC==2,61);
        CHECK(bytes[0][0]==0x36 && bytes[0][1]==0x6B,62);
        incoming[0]=0x36;incoming[1]=0;incoming[5]=42;incoming[6]=0x6B;
        receive(0x100,CAN_ID_EXT,CAN_RTR_DATA,7);ArmCAN_Task();
        CHECK(motor_feedback[1].position==42 && arm_motor_status[0].rx_frames==0,63);
        CHECK(arm_motor_status[0].state==ARM_MOTOR_BUSY,64);
        receive(0x600,CAN_ID_EXT,CAN_RTR_DATA,7);ArmCAN_Task();
        CHECK(motor_feedback[6].position==42 && arm_motor_status[1].rx_frames==1,65);
        CHECK(arm_motor_status[0].state==ARM_MOTOR_BUSY,66);
        receive(0x500,CAN_ID_EXT,CAN_RTR_DATA,7);ArmCAN_Task();
        CHECK(arm_motor_status[0].state==ARM_MOTOR_DONE && !arm_motor_status[0].fault,67);
        CHECK(motor_feedback[5].position==42 && arm_motor_status[0].position_counts==42,68);
    } else if(test==7U) {
        incoming[0]=0x36;incoming[5]=42;incoming[6]=0x6B;
        receive(0x500,CAN_ID_STD,CAN_RTR_DATA,7);
        receive(0x500,CAN_ID_EXT,CAN_RTR_REMOTE,7);
        receive(0x500,CAN_ID_EXT,CAN_RTR_DATA,1);
        receive(0x500,CAN_ID_EXT,CAN_RTR_DATA,9);
        receive(0,CAN_ID_EXT,CAN_RTR_DATA,7);
        receive(0x800,CAN_ID_EXT,CAN_RTR_DATA,7);ArmCAN_Task();
        CHECK(!motor_feedback[5].position_count && !arm_can_diag[0].rx_can_frames,70);
        receive(0x501,CAN_ID_EXT,CAN_RTR_DATA,7);ArmCAN_Task();
        CHECK(!motor_feedback[5].position_count && arm_motor_status[0].rx_errors==1,71);
        incoming[6]=0;receive(0x500,CAN_ID_EXT,CAN_RTR_DATA,7);ArmCAN_Task();
        CHECK(!motor_feedback[5].position_count && arm_motor_status[0].rx_errors==2,72);
    } else if(test==8U) {
        ArmCAN_StopAll();ArmCAN_Task();
        CHECK(tx_count==3,80);
        for(i=0;i<3;i++) {
            CHECK(sent[i].ExtId==((5U+i)<<8) && sent[i].DLC==4,81);
            CHECK(bytes[i][0]==0xFE && bytes[i][1]==0x98 && bytes[i][2]==0 && bytes[i][3]==0x6B,82);
        }
    } else return 99;
    return 0;
}
