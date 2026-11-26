#include "route.h"
#include "route_uart.h"
#include "chassis_heading.h"
#include "imu.h"

#define FRAME_BYTES (ROUTE_MAX_PAYLOAD + 10U)
#define PARTIAL_TIMEOUT_MS 500U
#define TYPE_UPLOAD 0x10U
#define TYPE_START 0x11U
#define TYPE_STOP 0x12U
#define TYPE_WORK_DONE 0x13U
#define TYPE_STATUS 0x14U
#define TYPE_VISION 0x30U

volatile RouteStatus_t route_status;
volatile RouteConfig_t route_config = {6.0f, 1.5f, 0U};
volatile RouteVision_t route_vision;
static RouteConfig_t active_config;
static uint8_t rx_queue[ROUTE_RX_QUEUE_SIZE], frame[FRAME_BYTES];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_broken;
static uint16_t used, route_length, event_sequence;
static uint8_t saved_route[ROUTE_MAX_PAYLOAD];
static uint32_t highest_id, work_started, last_work_id;
static uint16_t last_work_index;
static uint8_t last_work_result, have_last_work;
static float anchor_raw_yaw, anchor_map_yaw, leg_yaw;
static int8_t imu_sign;
static uint8_t motion_inhibit;

static uint16_t u16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static void put16(uint8_t *p, uint16_t n) { p[0]=(uint8_t)n; p[1]=(uint8_t)(n>>8); }
static void put32(uint8_t *p, uint32_t n)
{ p[0]=(uint8_t)n; p[1]=(uint8_t)(n>>8); p[2]=(uint8_t)(n>>16); p[3]=(uint8_t)(n>>24); }
static float absf(float n) { return n<0.0f?-n:n; }
static float wrap(float a)
{ while(a>=180.0f) a-=360.0f; while(a< -180.0f) a+=360.0f; return a; }
static int32_t rounded(float x) { return (int32_t)(x<0.0f?x-0.5f:x+0.5f); }

/* Range-reduced degree sine, ninth-order polynomial. Max absolute error below
 * 4e-6 over a full circle; avoids large libm code/stack on Cortex-M3. */
static float sine(float deg)
{
    float x, x2;
    deg=wrap(deg);
    if(deg>90.0f) deg=180.0f-deg;
    else if(deg< -90.0f) deg=-180.0f-deg;
    x=deg*0.0174532925199433f; x2=x*x;
    return x*(1.0f+x2*(-1.0f/6.0f+x2*(1.0f/120.0f+
        x2*(-1.0f/5040.0f+x2*(1.0f/362880.0f)))));
}
static void body_to_map(float f, float l, float yaw, float *dx, float *dy)
{
    float s=sine(yaw), c=sine(yaw+90.0f);
    *dx=c*f-s*l; *dy=s*f+c*l;
}

uint16_t Route_CRC16(const uint8_t *data, uint16_t count)
{
    uint16_t crc=0xFFFFU;
    uint8_t bit;
    while(count--) {
        crc^=(uint16_t)*data++ << 8;
        for(bit=0U;bit<8U;bit++) crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
    }
    return crc;
}

static void report(uint8_t type, uint16_t sequence, uint8_t request, uint8_t code)
{
    uint8_t out[46]={0};
    out[0]=0xA5U; out[1]=0x5AU; out[2]=1U; out[3]=type;
    put16(out+4,sequence); put16(out+6,36U);
    out[8]=request; out[9]=code; out[10]=(uint8_t)route_status.state; out[11]=route_status.phase;
    put32(out+12,route_status.route_id);
    put16(out+16,route_status.point_index); put16(out+18,route_status.point_count);
    put16(out+20,route_status.work_id);
    out[22]=route_status.chassis_fault; out[23]=route_status.fault_address;
    put32(out+24,HAL_GetTick()); put32(out+28,route_status.fault);
    put32(out+32,(uint32_t)rounded(route_status.estimated_x_mm));
    put32(out+36,(uint32_t)rounded(route_status.estimated_y_mm));
    put16(out+40,(uint16_t)(int16_t)rounded(route_status.map_yaw_deg*100.0f));
    put16(out+44,Route_CRC16(out+2,42U));
    if(!RouteUART_Send(out,sizeof(out))) route_status.tx_drops++;
}
static void event(uint8_t kind) { report(0x81U,++event_sequence,0U,kind); }

void Route_Init(void)
{
    RouteStatus_t empty={0}; RouteVision_t empty_vision={0};
    route_status=empty; route_vision=empty_vision;
    rx_head=rx_tail=used=route_length=event_sequence=0U;
    rx_broken=have_last_work=motion_inhibit=0U; highest_id=0U;
}
uint8_t Route_IsActive(void)
{ return route_status.state==ROUTE_RUNNING || route_status.state==ROUTE_WAIT_WORK; }

void Route_ReceiveBytes(const uint8_t *data, uint16_t count)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq(); /* serialize DMA and UART IDLE producers */
    while(count--) {
        uint16_t next=(uint16_t)((rx_head+1U)&(ROUTE_RX_QUEUE_SIZE-1U));
        if(next==rx_tail) { route_status.rx_overflows++; rx_broken=1U; break; }
        rx_queue[rx_head]=*data++; __DMB(); rx_head=next; route_status.rx_bytes++;
    }
    route_status.last_rx_tick=HAL_GetTick();
    __set_PRIMASK(key);
}
void Route_ReceiveFault(void)
{ rx_broken=1U; route_status.rx_faults++; }

static void route_fail(uint32_t fault)
{
    route_status.chassis_fault=(uint8_t)chassis_motion.fault;
    if(fault==ROUTE_IMU_LOST) route_status.chassis_fault=CHASSIS_IMU_OFFLINE;
    route_status.fault_address=chassis_motion.fault_address;
    if(Chassis_IsBusy()) Chassis_Stop();
    route_status.fault=fault; route_status.state=ROUTE_FAULT;
    event(7U);
}
void Route_Stop(void)
{
    if(Chassis_IsBusy()) Chassis_Stop();
    if(!Route_IsActive() && route_status.state!=ROUTE_READY) return;
    route_status.state=ROUTE_STOPPED; route_status.fault=ROUTE_EXTERNAL_STOP;
    event(6U);
}
void Route_SetMotionInhibit(uint8_t inhibit) { motion_inhibit=inhibit; }

static uint8_t angle_ok(int32_t value) { return value>=-18000L && value<=18000L; }
static uint8_t xy_ok(int32_t value) { return value>=-ROUTE_MAP_LIMIT_MM && value<=ROUTE_MAP_LIMIT_MM; }
static void point_at(uint16_t index, RoutePoint_t *point)
{
    const uint8_t *p=saved_route+20U+index*ROUTE_POINT_BYTES;
    point->x_mm=(int32_t)u32(p); point->y_mm=(int32_t)u32(p+4); point->yaw_cdeg=(int32_t)u32(p+8);
    point->move_rpm=u16(p+12); point->turn_rpm=u16(p+14);
    point->work_id=u16(p+16); point->flags=u16(p+18);
}
static RouteResult_t load_route(const uint8_t *p, uint16_t length)
{
    uint16_t count, i;
    uint32_t id;
    if(length<20U) return ROUTE_BAD_LENGTH;
    id=u32(p); count=u16(p+4);
    if(count==0U || count>ROUTE_MAX_POINTS || length!=20U+count*ROUTE_POINT_BYTES) return ROUTE_BAD_LENGTH;
    if(id==0U) return ROUTE_BAD_ID;
    /* Exact duplicate upload never resets/restarts a running or completed route. */
    if(id==route_status.route_id) {
        if(route_length!=length) return ROUTE_CONFLICT;
        for(i=0U;i<length;i++) if(saved_route[i]!=p[i]) return ROUTE_CONFLICT;
        return ROUTE_OK;
    }
    if(Route_IsActive() || Chassis_IsBusy()) return ROUTE_BUSY;
    if(id<=highest_id) return ROUTE_BAD_ID;
    if(u16(p+6)!=0U || !xy_ok((int32_t)u32(p+8)) ||
       !xy_ok((int32_t)u32(p+12)) || !angle_ok((int32_t)u32(p+16))) return ROUTE_BAD_VALUE;
    for(i=0U;i<count;i++) {
        const uint8_t *q=p+20U+i*ROUTE_POINT_BYTES;
        uint16_t flags=u16(q+18), work=u16(q+16);
        if(!xy_ok((int32_t)u32(q)) || !xy_ok((int32_t)u32(q+4)) || !angle_ok((int32_t)u32(q+8)) ||
           u16(q+12)==0U || u16(q+12)>CHASSIS_MAX_RPM || u16(q+14)==0U || u16(q+14)>CHASSIS_MAX_RPM ||
           (flags&~ROUTE_POINT_WORK)!=0U || ((flags&ROUTE_POINT_WORK)!=0U ? work==0U : work!=0U))
            return ROUTE_BAD_VALUE;
    }
    /* Commit only after the entire frame and every waypoint passed validation. */
    for(i=0U;i<length;i++) saved_route[i]=p[i];
    route_length=length; highest_id=id; have_last_work=0U;
    route_status.route_id=id; route_status.point_count=count; route_status.point_index=0U;
    route_status.work_id=route_status.phase=route_status.chassis_fault=route_status.fault_address=0U;
    route_status.fault=ROUTE_NO_FAULT;
    route_status.estimated_x_mm=(float)(int32_t)u32(p+8);
    route_status.estimated_y_mm=(float)(int32_t)u32(p+12);
    route_status.map_yaw_deg=(float)(int32_t)u32(p+16)*0.01f;
    route_status.state=ROUTE_READY; route_status.received_routes++;
    route_vision.count=0U; event(1U);
    return ROUTE_OK;
}

RouteResult_t Route_Start(uint32_t route_id)
{
    IMU_Data_t imu;
    if(route_id!=route_status.route_id || route_id==0U) return ROUTE_BAD_ID;
    if(motion_inhibit) return ROUTE_BAD_STATE;
    if(Route_IsActive()) return ROUTE_OK; /* START retry is idempotent. */
    if(route_status.state!=ROUTE_READY) return ROUTE_BAD_STATE;
    if(Chassis_IsBusy()) return ROUTE_BUSY;
    active_config=route_config;
    if(!(active_config.position_tolerance_mm>=1.5f*chassis_heading_config.distance_tolerance_mm &&
         active_config.position_tolerance_mm<=100.0f) ||
       !(active_config.heading_tolerance_deg>=chassis_heading_config.yaw_tolerance_deg &&
         active_config.heading_tolerance_deg<=5.0f) ||
       active_config.work_timeout_ms>3600000U ||
       (chassis_heading_config.imu_yaw_sign!=1 && chassis_heading_config.imu_yaw_sign!=-1)) return ROUTE_BAD_VALUE;
    if(!IMU_GetData(&imu)) return ROUTE_BAD_STATE;
    imu_sign=chassis_heading_config.imu_yaw_sign;
    anchor_raw_yaw=imu.angle_deg[2]; anchor_map_yaw=route_status.map_yaw_deg;
    route_status.state=ROUTE_RUNNING; route_status.phase=0U;
    event(2U); return ROUTE_OK;
}

static void next_point(void)
{
    route_status.completed_points++; event(8U);
    route_status.point_index++; route_status.work_id=0U; route_status.phase=0U;
    if(route_status.point_index==route_status.point_count) {
        route_status.state=ROUTE_DONE; route_status.completed_routes++; event(5U);
    } else route_status.state=ROUTE_RUNNING;
}
RouteResult_t Route_CompleteWork(uint32_t id, uint16_t index, uint8_t success)
{
    if(id!=route_status.route_id || id==0U) return ROUTE_BAD_ID;
    if(success>1U) return ROUTE_BAD_VALUE;
    if(have_last_work && last_work_id==id && last_work_index==index)
        return last_work_result==success ? ROUTE_OK : ROUTE_CONFLICT;
    if(route_status.state!=ROUTE_WAIT_WORK || index!=route_status.point_index) return ROUTE_BAD_STATE;
    have_last_work=1U; last_work_id=id; last_work_index=index; last_work_result=success;
    if(!success) route_fail(ROUTE_WORK_FAILED);
    else next_point();
    return ROUTE_OK;
}

__weak void Route_OnWork(uint32_t id, uint16_t index, uint16_t work)
{ (void)id; (void)index; (void)work; }
__weak void Route_OnVision(const RouteVision_t *vision) { (void)vision; }

static RouteResult_t vision_message(const uint8_t *p, uint16_t length)
{
    RouteVision_t v;
    if(length!=20U) return ROUTE_BAD_LENGTH;
    if(u32(p)!=route_status.route_id) return ROUTE_BAD_ID;
    if(route_status.state!=ROUTE_WAIT_WORK || u16(p+4)!=route_status.point_index || u16(p+6)!=route_status.work_id)
        return ROUTE_BAD_STATE;
    if(u16(p+18)>1000U) return ROUTE_BAD_VALUE;
    v.route_id=u32(p); v.point_index=u16(p+4); v.work_id=u16(p+6);
    v.dx_mm=(int32_t)u32(p+8); v.dy_mm=(int32_t)u32(p+12);
    v.dyaw_cdeg=(int16_t)u16(p+16); v.confidence=u16(p+18);
    v.tick=HAL_GetTick(); v.count=route_vision.count+1U;
    route_vision=v; Route_OnVision(&v); return ROUTE_OK;
}

static void dispatch(void)
{
    uint8_t type=frame[3];
    uint16_t length=u16(frame+6), sequence=u16(frame+4);
    const uint8_t *p=frame+8;
    RouteResult_t code=ROUTE_BAD_TYPE;
    switch(type) {
    case TYPE_UPLOAD: code=load_route(p,length); break;
    case TYPE_START: code=length==4U ? Route_Start(u32(p)) : ROUTE_BAD_LENGTH; break;
    case TYPE_STOP:
        if(length!=4U) code=ROUTE_BAD_LENGTH;
        else if(u32(p)!=0U && u32(p)!=route_status.route_id) code=ROUTE_BAD_ID;
        else { Route_Stop(); code=ROUTE_OK; }
        break;
    case TYPE_WORK_DONE:
        if(length!=8U) code=ROUTE_BAD_LENGTH;
        else if(p[7]!=0U) code=ROUTE_BAD_VALUE;
        else code=Route_CompleteWork(u32(p),u16(p+4),p[6]);
        break;
    case TYPE_STATUS: code=length==0U?ROUTE_OK:ROUTE_BAD_LENGTH; break;
    case TYPE_VISION: code=vision_message(p,length); break;
    default: break;
    }
    if(code==ROUTE_BAD_VALUE) route_status.value_errors++;
    if(code==ROUTE_BAD_LENGTH) route_status.length_errors++;
    report(0x80U,sequence,type,(uint8_t)code);
}

static void drop(uint16_t count)
{
    uint16_t i;
    used=(uint16_t)(used-count);
    for(i=0U;i<used;i++) frame[i]=frame[i+count];
}
static void parse_byte(uint8_t b)
{
    uint16_t length, total;
    if(used==FRAME_BYTES) drop(1U);
    frame[used++]=b;
    while(used>=2U) {
        if(frame[0]!=0xA5U || frame[1]!=0x5AU) { drop(1U); continue; }
        if(used<8U) return;
        length=u16(frame+6);
        if(frame[2]!=1U || length>ROUTE_MAX_PAYLOAD) {
            route_status.length_errors++; drop(1U); continue;
        }
        total=(uint16_t)(length+10U);
        if(used<total) return;
        if(Route_CRC16(frame+2,(uint16_t)(length+6U))!=u16(frame+total-2U)) {
            route_status.crc_errors++; drop(1U); continue;
        }
        route_status.frames++; dispatch(); drop(total);
    }
}

static void update_leg_pose(void)
{
    float dx,dy,yaw=leg_yaw;
    if(route_status.phase==1U) yaw=wrap(yaw+chassis_heading.turned_deg*0.5f);
    body_to_map(chassis_heading.forward_mm,chassis_heading.left_mm,yaw,&dx,&dy);
    route_status.estimated_x_mm+=dx; route_status.estimated_y_mm+=dy;
}
static void execute(void)
{
    IMU_Data_t imu;
    RoutePoint_t goal;
    float dx,dy,f,l,yaw_error,s,c,amount;
    ChassisResult_t result;
    if(!Route_IsActive()) return;
    if(route_status.state==ROUTE_WAIT_WORK) {
        if(active_config.work_timeout_ms!=0U &&
           (uint32_t)(HAL_GetTick()-work_started)>=active_config.work_timeout_ms) route_fail(ROUTE_WORK_TIMEOUT);
        return;
    }
    if(!IMU_GetData(&imu)) {
        route_fail(ROUTE_IMU_LOST); return;
    }
    if(chassis_heading_config.imu_yaw_sign!=imu_sign) { route_fail(ROUTE_START_FAILED); return; }
    route_status.map_yaw_deg=wrap(anchor_map_yaw+wrap(imu.angle_deg[2]-anchor_raw_yaw)*imu_sign);
    if(route_status.phase!=0U) {
        Chassis_Task();
        if(chassis_motion.state==CHASSIS_FAULT) { route_fail(ROUTE_CHASSIS_FAULT); return; }
        if(chassis_motion.state==CHASSIS_STOPPED) { Route_Stop(); return; }
        if(chassis_motion.state!=CHASSIS_DONE) return;
        update_leg_pose(); route_status.phase=0U;
    }
    point_at(route_status.point_index,&goal);
    route_status.target_x_mm=(float)goal.x_mm; route_status.target_y_mm=(float)goal.y_mm;
    route_status.target_yaw_deg=(float)goal.yaw_cdeg*0.01f;
    yaw_error=wrap(route_status.target_yaw_deg-route_status.map_yaw_deg);
    if(yaw_error== -180.0f) yaw_error=180.0f;
    leg_yaw=route_status.map_yaw_deg;
    /* Orient to the waypoint's map heading first, then approach using its
     * current body frame. Re-plan from measured wheel progress after each leg. */
    if(absf(yaw_error)>active_config.heading_tolerance_deg) {
        route_status.phase=1U; result=Chassis_TurnRelative(yaw_error,goal.turn_rpm);
    } else {
        dx=goal.x_mm-route_status.estimated_x_mm; dy=goal.y_mm-route_status.estimated_y_mm;
        if(dx*dx+dy*dy<=active_config.position_tolerance_mm*active_config.position_tolerance_mm) {
            event(3U);
            if((goal.flags&ROUTE_POINT_WORK)!=0U) {
                route_status.work_id=goal.work_id; route_status.phase=4U;
                route_status.state=ROUTE_WAIT_WORK; work_started=HAL_GetTick();
                event(4U); Route_OnWork(route_status.route_id,route_status.point_index,goal.work_id);
            } else next_point();
            return;
        }
        s=sine(leg_yaw); c=sine(leg_yaw+90.0f);
        f=c*dx+s*dy; l=-s*dx+c*dy;
        if(absf(f)>=absf(l)) {
            amount=f; route_status.phase=2U;
        } else { amount=l; route_status.phase=3U; }
        if(amount>CHASSIS_MAX_DISTANCE_MM) amount=CHASSIS_MAX_DISTANCE_MM;
        else if(amount< -CHASSIS_MAX_DISTANCE_MM) amount=-CHASSIS_MAX_DISTANCE_MM;
        result=route_status.phase==2U?Chassis_MoveForwardHold(amount,goal.move_rpm):Chassis_MoveLeftHold(amount,goal.move_rpm);
    }
    if(result!=CHASSIS_OK) route_fail(ROUTE_START_FAILED);
}

void Route_Task(void)
{
    uint16_t budget;
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    if(rx_broken) { rx_tail=rx_head; rx_broken=0U; used=0U; }
    __set_PRIMASK(key);
    if(used!=0U && rx_head==rx_tail &&
       (uint32_t)(HAL_GetTick()-route_status.last_rx_tick)>PARTIAL_TIMEOUT_MS) {
        used=0U; route_status.partial_timeouts++;
    }
    for(budget=0U;budget<512U;budget++) {
        uint8_t b;
        key=__get_PRIMASK(); __disable_irq();
        if(rx_broken) { rx_tail=rx_head; rx_broken=0U; used=0U; }
        if(rx_tail==rx_head) { __set_PRIMASK(key); break; }
        b=rx_queue[rx_tail]; rx_tail=(uint16_t)((rx_tail+1U)&(ROUTE_RX_QUEUE_SIZE-1U));
        __set_PRIMASK(key); parse_byte(b);
    }
    execute();
}
