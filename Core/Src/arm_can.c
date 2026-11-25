#include "arm_can.h"
#include "can.h"

#define RX_SIZE 128U
#define REPLY_MS 80U
#define MOTION_MAX_MS 120000U
#define POLL_MS 20U
#define SETTLE_MS 100U

volatile ArmMotorConfig_t arm_motor_config[ARM_AXIS_COUNT] = {
    {0U,0,0,10U,50U,100U}, {0U,0,0,10U,50U,100U}, {0U,0,0,10U,50U,100U}
};
volatile ArmMotorStatus_t arm_motor_status[ARM_AXIS_COUNT];
volatile ArmCANDiag_t arm_can_diag[ARM_AXIS_COUNT];

typedef struct {
    uint8_t rx[RX_SIZE], frame[8], tx[13];
    volatile uint16_t head, tail;
    volatile uint8_t rx_fault;
    uint8_t used, waiting, expected, reply, tx_active;
    uint8_t operation, stage, stop_pending, stop_attempts, settling;
    uint32_t sent, started, timeout, polled, settle_tick, rx_tick;
    int32_t requested;
    ArmMotorConfig_t config;
} Port;
static Port ports[ARM_AXIS_COUNT];

static int64_t magnitude(int64_t v) { return v<0 ? -v : v; }
static void fail(uint8_t i, uint32_t fault)
{
    Port *p=&ports[i];
    uint8_t probe=(uint8_t)(p->operation==3U);
    arm_can_diag[i].fault_code=(uint8_t)fault;
    arm_can_diag[i].fault_stage=p->stage;
    arm_can_diag[i].fault_function=p->expected;
    arm_can_diag[i].fault_flags=arm_motor_status[i].flags;
    arm_can_diag[i].fault_status=arm_motor_status[i].last_status;
    arm_can_diag[i].fault_speed_rpm=arm_motor_status[i].speed_rpm;
    if(fault==ARM_MOTOR_TIMEOUT) {
        arm_can_diag[i].timeout_function=p->expected;
        arm_can_diag[i].timeout_stage=p->stage;
        arm_can_diag[i].timeout_tx_active=p->tx_active;
        arm_can_diag[i].timeout_waiting=p->waiting;
    }
    arm_motor_status[i].fault=fault;
    arm_motor_status[i].state=ARM_MOTOR_FAULT;
    arm_motor_status[i].homed=0U;
    p->operation=p->waiting=p->reply=0U;
    /* A read-only probe must not send an enable, stop, or motion command. */
    if(!probe) {p->stop_pending=1U; p->stop_attempts=0U;}
}
static void clear_rx(Port *p)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq(); p->tail=p->head; p->used=0U; __set_PRIMASK(key);
}
static uint8_t send(uint8_t i, uint8_t function, uint8_t n)
{
    Port *p=&ports[i];
    HAL_StatusTypeDef result;
    uint8_t j;
    uint32_t key;
    if(p->tx_active) return 0U;
    clear_rx(p);
    p->tx[0]=arm_motor_status[i].address; p->tx[1]=function; p->tx[n-1U]=0x6BU;
    p->expected=function; p->waiting=1U; p->reply=0U;
    p->sent=HAL_GetTick(); p->tx_active=1U;
    arm_can_diag[i].tx_requests++;
    arm_can_diag[i].tx_length=n;
    for(j=0U;j<n;j++) arm_can_diag[i].tx_raw[j]=p->tx[j];
    key=__get_PRIMASK(); __disable_irq();
    arm_can_diag[i].rx_length=0U;
    for(j=0U;j<16U;j++) arm_can_diag[i].rx_raw[j]=0U;
    __set_PRIMASK(key);
    /* Shared foreground transport waits for actual CAN transmission. All
     * callers (chassis and arm) belong to RobotTask; no ISR/CommTask sends here.
     * Position commands are split by can_SendCmd into consecutive packets.
     */
    can_SendCmd(p->tx,n);
    result=can_last_tx_status;
    arm_can_diag[i].last_hal_status=(uint32_t)result;
    arm_can_diag[i].last_can_error=HAL_CAN_GetError(&hcan);
    p->tx_active=0U;
    if(result==HAL_OK) {
        arm_can_diag[i].tx_completed++;
        arm_can_diag[i].tx_can_frames+=(uint32_t)((n-2U+6U)/7U);
    }
    if(result!=HAL_OK) {
        arm_motor_status[i].can_errors++;
        p->tx_active=p->waiting=0U;
        if(p->operation) fail(i,ARM_MOTOR_TX);
        else if(arm_motor_status[i].state!=ARM_MOTOR_FAULT) {
            /* A failed stop transmission must not replace the original fault. */
            arm_motor_status[i].fault=ARM_MOTOR_TX;
            arm_motor_status[i].state=ARM_MOTOR_FAULT;
            arm_motor_status[i].homed=0U;
        }
        return 0U;
    }
    return 1U;
}
static uint8_t query(uint8_t i, uint8_t f) { return send(i,f,3U); }

HAL_StatusTypeDef ArmCAN_Init(void)
{
    uint8_t i;
    for(i=0;i<ARM_AXIS_COUNT;i++) {
        Port empty={0}; ArmMotorStatus_t status={0}; ArmCANDiag_t diag={0};
        ports[i]=empty; arm_motor_status[i]=status; arm_can_diag[i]=diag;
        arm_motor_status[i].address=(uint8_t)(5U+i);
    }
    return HAL_OK; /* CAN/filter/FIFO IRQ already started by main. No motion. */
}

/* Called only by the shared CAN FIFO0 ISR. Current replies fit one CAN frame.
 * Preserve whole validated frame boundaries before reconstructing the common
 * address/function/data/6B representation consumed in foreground.
 */
void ArmCAN_RxFrame(uint32_t ext_id,const uint8_t *data,uint8_t dlc)
{
    uint8_t i,j,expected,n;
    uint16_t head,free_bytes;
    uint32_t address=ext_id>>8;
    Port *p;
    if(address<5U || address>7U) return;
    for(i=0U;i<ARM_AXIS_COUNT;i++) if(arm_motor_status[i].address==address) break;
    if(i==ARM_AXIS_COUNT) return;
    p=&ports[i];
    arm_can_diag[i].rx_can_frames++;
    arm_can_diag[i].last_rx_ext_id=ext_id;
    arm_can_diag[i].last_rx_dlc=dlc;
    if(!data || dlc<2U || dlc>8U) {arm_motor_status[i].rx_errors++;return;}
    arm_can_diag[i].rx_bytes+=dlc;
    for(j=0U;j<dlc && arm_can_diag[i].rx_length<16U;j++) {
        uint8_t slot=arm_can_diag[i].rx_length;
        arm_can_diag[i].rx_raw[slot]=data[j];
        arm_can_diag[i].rx_length=(uint8_t)(slot+1U);
    }
    if(data[0]==0x36U) expected=7U;
    else if(data[0]==0x35U) expected=5U;
    else if(data[0]==0x3AU || data[0]==0xF3U || data[0]==0xFDU || data[0]==0xFEU) expected=3U;
    else return; /* e.g. chassis broadcast-sync reply; not an arm operation. */
    if((ext_id&0xFFU)!=0U || dlc!=expected || data[dlc-1U]!=0x6BU) {
        arm_motor_status[i].rx_errors++;return;
    }
    n=(uint8_t)(dlc+1U);
    free_bytes=(uint16_t)((p->tail-p->head-1U)&(RX_SIZE-1U));
    if(free_bytes<n) {p->rx_fault=1U;return;}
    head=p->head;p->rx[head]=(uint8_t)address;head=(uint16_t)((head+1U)&(RX_SIZE-1U));
    for(j=0U;j<dlc;j++) {p->rx[head]=data[j];head=(uint16_t)((head+1U)&(RX_SIZE-1U));}
    __DMB();p->head=head;
}

static void accept(uint8_t i, uint8_t n)
{
    Port *p=&ports[i]; uint8_t *f=p->frame, function=f[1];
    uint32_t now=HAL_GetTick(), value;
    if(f[n-1U]!=0x6BU) { arm_motor_status[i].rx_errors++; return; }
    if((function==0x36U || function==0x35U) && f[2]>1U) {
        arm_motor_status[i].rx_errors++;return;
    }
    arm_motor_status[i].rx_frames++;
    /* Late replies may update telemetry but cannot satisfy another function. */
    if(function==0x36U && n==8U && f[2]<=1U) {
        value=((uint32_t)f[3]<<24)|((uint32_t)f[4]<<16)|((uint32_t)f[5]<<8)|f[6];
        if(value>2147483647U) return;
        arm_motor_status[i].position_counts=f[2]?-(int32_t)value:(int32_t)value;
        arm_motor_status[i].position_tick=now;
        if(arm_motor_status[i].homed) arm_motor_status[i].position_cdeg=(int32_t)(
            ((int64_t)arm_motor_status[i].position_counts-arm_motor_status[i].origin_counts)*36000/65536);
    } else if(function==0x35U && n==6U && f[2]<=1U) {
        value=((uint32_t)f[3]<<8)|f[4]; if(value>32767U) return;
        arm_motor_status[i].speed_rpm=f[2]?-(int16_t)value:(int16_t)value;
        arm_motor_status[i].speed_tick=now;
    } else if(function==0x3AU && n==4U) arm_motor_status[i].flags=f[2];
    else if(n==4U) {
        arm_motor_status[i].last_function=function; arm_motor_status[i].last_status=f[2];
        if(f[2]==0x9FU) return; /* reached notification is not a receive ACK */
        if(p->waiting && p->expected==function && f[2]!=0x02U) {
            if(p->operation) fail(i,ARM_MOTOR_REPLY);
            else p->waiting=0U; /* bounded stop retries; no recursive fault */
            return;
        }
    } else return;
    if(p->waiting && p->expected==function) { p->waiting=0U; p->reply=1U; }
}
static void parse(uint8_t i)
{
    Port *p=&ports[i]; uint8_t n,b,f;
    if(p->used && (uint32_t)(HAL_GetTick()-p->rx_tick)>REPLY_MS) p->used=0U;
    while(p->tail!=p->head) {
        b=p->rx[p->tail]; p->tail=(uint16_t)((p->tail+1U)&(RX_SIZE-1U));
        p->rx_tick=HAL_GetTick();
        if(!p->used && b!=arm_motor_status[i].address) continue;
        p->frame[p->used++]=b;
        if(p->used<2U) continue;
        f=p->frame[1];
        if(f==0x36U) n=8U;
        else if(f==0x35U) n=6U;
        else if(f==0x3AU || f==0xF3U || f==0xFDU || f==0xFEU) n=4U;
        else { p->used=(uint8_t)(b==arm_motor_status[i].address); p->frame[0]=b; continue; }
        if(p->used==n) { accept(i,n); p->used=0U; }
    }
}
uint8_t ArmCAN_Busy(void)
{
    uint8_t i;
    for(i=0;i<ARM_AXIS_COUNT;i++)
        if(ports[i].operation || ports[i].stop_pending || ports[i].tx_active || ports[i].waiting) return 1U;
    return 0U;
}
static uint8_t begin(uint8_t i, uint8_t operation)
{
    Port *p;
    if(i>=ARM_AXIS_COUNT || ArmCAN_Busy()) return 0U;
    p=&ports[i];
    /* On a shared bus these addresses must never overlap chassis 1..4. */
    if(arm_motor_status[i].address<5U || arm_motor_status[i].address>7U) return 0U;
    {uint8_t j;for(j=0U;j<ARM_AXIS_COUNT;j++)
        if(j!=i && arm_motor_status[j].address==arm_motor_status[i].address) return 0U;}
    if(p->rx_fault || p->tx_active) return 0U;
    p->config=arm_motor_config[i];
    p->operation=operation; p->stage=0U; p->settling=0U;
    p->started=HAL_GetTick(); p->timeout=3000U;
    arm_can_diag[i].fault_code=ARM_MOTOR_OK;
    arm_motor_status[i].state=ARM_MOTOR_BUSY; arm_motor_status[i].fault=ARM_MOTOR_OK;
    return 1U;
}
uint8_t ArmCAN_Probe(uint8_t i)
{ return begin(i,3U); }
uint8_t ArmCAN_RecordOrigin(uint8_t i)
{
    if(!begin(i,1U)) return 0U;
    arm_motor_status[i].homed=0U;
    return 1U;
}
static uint8_t position_config_valid(const ArmMotorConfig_t *config)
{
    return config->pulses_per_rev && config->pulses_per_rev<=256000U &&
        config->tolerance_cdeg && config->tolerance_cdeg<=1000U;
}
static int64_t position_target_counts(uint8_t i, int32_t target)
{
    return (int64_t)arm_motor_status[i].origin_counts+(int64_t)target*65536/36000;
}
static uint8_t position_within_tolerance(uint8_t i, int64_t target_counts, const ArmMotorConfig_t *config)
{
    int64_t tolerance=(int64_t)config->tolerance_cdeg*65536/36000;
    /* Account for <= half a configured motor input pulse. */
    tolerance+=32768/config->pulses_per_rev+1;
    return magnitude(target_counts-arm_motor_status[i].position_counts)<=tolerance;
}
uint8_t ArmCAN_PositionWithinTolerance(uint8_t i, int32_t target)
{
    ArmMotorConfig_t config;
    if(i>=ARM_AXIS_COUNT || !arm_motor_status[i].homed) return 0U;
    config=arm_motor_config[i];
    return position_config_valid(&config) &&
        position_within_tolerance(i,position_target_counts(i,target),&config);
}
uint8_t ArmCAN_PositionWithinCustomTolerance(uint8_t i, int32_t target, uint16_t tolerance_cdeg)
{
    ArmMotorConfig_t config;
    if(i>=ARM_AXIS_COUNT || !arm_motor_status[i].homed) return 0U;
    config=arm_motor_config[i];
    config.tolerance_cdeg=tolerance_cdeg;
    return position_config_valid(&config) &&
        position_within_tolerance(i,position_target_counts(i,target),&config);
}
uint8_t ArmCAN_MoveWithTolerance(uint8_t i, int32_t target, uint16_t tolerance_cdeg)
{
    ArmMotorConfig_t c;
    int64_t counts;
    if(i>=ARM_AXIS_COUNT || !arm_motor_status[i].homed) return 0U;
    c=arm_motor_config[i];
    if(tolerance_cdeg) c.tolerance_cdeg=tolerance_cdeg;
    if(!position_config_valid(&c) || !c.rpm || c.rpm>300U ||
       c.min_cdeg>0 || c.max_cdeg<0 || target<c.min_cdeg || target>c.max_cdeg) return 0U;
    counts=position_target_counts(i,target);
    if(counts < -2147483647LL || counts>2147483647LL || !begin(i,2U)) return 0U;
    ports[i].config.tolerance_cdeg=c.tolerance_cdeg;
    ports[i].requested=target; arm_motor_status[i].target_counts=(int32_t)counts;
    return 1U;
}
uint8_t ArmCAN_Move(uint8_t i, int32_t target)
{ return ArmCAN_MoveWithTolerance(i,target,0U); }
void ArmCAN_StopAll(void)
{
    uint8_t i;
    for(i=0;i<ARM_AXIS_COUNT;i++) {
        Port *p=&ports[i];
        /* CAN sends finish before returning; no UART/DMA owns this axis. */
        p->tx_active=p->waiting=p->reply=p->operation=0U;
        p->stop_pending=1U; p->stop_attempts=0U;
        /* An interrupted move may stop between pulses. A fresh origin is
         * required before resuming any automatic action. */
        arm_motor_status[i].homed=0U;
        if(arm_motor_status[i].state!=ARM_MOTOR_FAULT) arm_motor_status[i].state=ARM_MOTOR_STOPPED;
    }
}
static void advance(uint8_t i)
{
    Port *p=&ports[i]; uint32_t now=HAL_GetTick(), pulses;
    int64_t delta, ms;
    if(p->waiting || p->tx_active) return;
    if(!p->operation) return;
    if((uint32_t)(now-p->started)>p->timeout) { fail(i,ARM_MOTOR_TIMEOUT); return; }
    if(p->operation==3U) {
        if(p->stage==0U) {if(query(i,0x36U)) p->stage=1U;}
        else if(p->reply) {arm_motor_status[i].state=ARM_MOTOR_DONE; p->operation=0U;}
        return;
    }
    if(p->stage==0U) {
        p->tx[2]=0xABU; p->tx[3]=1U; p->tx[4]=0U;
        if(send(i,0xF3U,6U)) p->stage=1U;
    } else if(p->stage==1U && p->reply) {
        if(query(i,0x3AU)) p->stage=2U;
    } else if(p->stage==2U && p->reply) {
        if((arm_motor_status[i].flags & 0x0DU)!=1U) { fail(i,ARM_MOTOR_FLAGS); return; }
        if(query(i,0x36U)) p->stage=3U;
    } else if(p->stage==3U && p->reply) {
        if(query(i,0x35U)) p->stage=4U;
    } else if(p->stage==4U && p->reply) {
        if(magnitude(arm_motor_status[i].speed_rpm)>1) { fail(i,ARM_MOTOR_FLAGS); return; }
        if(p->operation==1U) {
            arm_motor_status[i].origin_counts=arm_motor_status[i].position_counts;
            arm_motor_status[i].position_cdeg=0;
            arm_motor_status[i].homed=1U; arm_motor_status[i].state=ARM_MOTOR_DONE;
            p->operation=0U; return;
        }
        delta=(int64_t)arm_motor_status[i].target_counts-arm_motor_status[i].position_counts;
        pulses=(uint32_t)((magnitude(delta)*p->config.pulses_per_rev+32768)/65536);
        ms=magnitude(delta)*60000/(65536LL*p->config.rpm)*3+3000;
        if(ms>MOTION_MAX_MS) { fail(i,ARM_MOTOR_CONFIG); return; }
        p->timeout=(uint32_t)ms; p->started=now;
        p->tx[2]=(uint8_t)(delta<0); p->tx[3]=(uint8_t)(p->config.rpm>>8);
        p->tx[4]=(uint8_t)p->config.rpm; p->tx[5]=p->config.acceleration;
        p->tx[6]=(uint8_t)(pulses>>24); p->tx[7]=(uint8_t)(pulses>>16);
        p->tx[8]=(uint8_t)(pulses>>8); p->tx[9]=(uint8_t)pulses;
        p->tx[10]=2U; p->tx[11]=0U; /* relative CURRENT; execute immediately */
        if(send(i,0xFDU,13U)) p->stage=5U;
    } else if(p->stage==5U && p->reply) { p->stage=6U; p->polled=now-POLL_MS; }
    else if(p->stage==6U && (uint32_t)(now-p->polled)>=POLL_MS) {
        if(query(i,0x3AU)) { p->stage=7U; p->polled=now; }
    } else if(p->stage==7U && p->reply) {
        if((arm_motor_status[i].flags & 0x0DU)!=1U) { fail(i,ARM_MOTOR_FLAGS); return; }
        if(query(i,0x36U)) p->stage=8U;
    } else if(p->stage==8U && p->reply) {
        if(query(i,0x35U)) p->stage=9U;
    } else if(p->stage==9U && p->reply) {
        if(position_within_tolerance(i,arm_motor_status[i].target_counts,&p->config) &&
           magnitude(arm_motor_status[i].speed_rpm)<=1) {
            if(!p->settling) { p->settling=1U; p->settle_tick=now; }
            if((uint32_t)(now-p->settle_tick)>=SETTLE_MS) {
                arm_motor_status[i].state=ARM_MOTOR_DONE; p->operation=0U; return;
            }
        } else p->settling=0U;
        p->stage=6U;
    }
}
void ArmCAN_Task(void)
{
    uint8_t i; uint32_t now=HAL_GetTick();
    for(i=0;i<ARM_AXIS_COUNT;i++) {
        Port *p=&ports[i];
        if(p->rx_fault) {
            uint8_t cause=p->rx_fault; p->rx_fault=0U; clear_rx(p);
            if(p->operation) fail(i,cause==1U?ARM_MOTOR_RX_OVERFLOW:ARM_MOTOR_CAN);
            arm_motor_status[i].homed=0U;
        }
        parse(i);
        if((p->waiting || p->tx_active) && (uint32_t)(now-p->sent)>=REPLY_MS) {
            if(p->operation) fail(i,ARM_MOTOR_TIMEOUT);
            /* The CAN transfer is already complete; only reply may time out. */
            p->tx_active=p->waiting=0U;
        }
        if(p->stop_pending && !p->waiting && !p->tx_active) {
            /* Stop is idempotent; retry only this command, never a motion. */
            if(p->reply && p->expected==0xFEU) p->stop_pending=0U;
            else if(p->stop_attempts<3U) {
                p->tx[2]=0x98U; p->tx[3]=0U; p->stop_attempts++;
                send(i,0xFEU,5U);
            } else p->stop_pending=0U;
        }
        advance(i);
        arm_motor_status[i].tick=now;
    }
}
