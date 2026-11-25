#ifndef CAR_ROUTE_H
#define CAR_ROUTE_H
#include "main.h"

#define ROUTE_MAX_POINTS 64U
#define ROUTE_POINT_BYTES 20U
#define ROUTE_MAX_PAYLOAD (20U + ROUTE_MAX_POINTS * ROUTE_POINT_BYTES)
#define ROUTE_RX_QUEUE_SIZE 2048U
#define ROUTE_MAP_LIMIT_MM 100000L
#define ROUTE_POINT_WORK 1U

typedef enum {
    ROUTE_EMPTY=0, ROUTE_READY=1, ROUTE_RUNNING=2, ROUTE_WAIT_WORK=3,
    ROUTE_DONE=4, ROUTE_STOPPED=5, ROUTE_FAULT=6
} RouteState_t;
typedef enum {
    ROUTE_OK=0, ROUTE_BAD_TYPE=1, ROUTE_BAD_LENGTH=2, ROUTE_BAD_CRC=3,
    ROUTE_BAD_VALUE=4, ROUTE_BUSY=5, ROUTE_BAD_ID=6, ROUTE_BAD_STATE=7,
    ROUTE_CONFLICT=8
} RouteResult_t;
typedef enum {
    ROUTE_NO_FAULT=0, ROUTE_CHASSIS_FAULT=1, ROUTE_START_FAILED=2,
    ROUTE_WORK_FAILED=3, ROUTE_WORK_TIMEOUT=4, ROUTE_EXTERNAL_STOP=5,
    ROUTE_IMU_LOST=6
} RouteFault_t;

/* Map: +X/+Y in mm, +yaw is CCW from map +X. At yaw=0 chassis front is +X,
 * chassis left is +Y. Wire heading is signed 0.01 degree, [-18000,18000]. */
typedef struct {
    int32_t x_mm, y_mm, yaw_cdeg;
    uint16_t move_rpm, turn_rpm, work_id, flags;
} RoutePoint_t;

typedef struct {
    float position_tolerance_mm; /* estimated XY radius, default 6 mm */
    float heading_tolerance_deg; /* default 1.5 degrees */
    uint32_t work_timeout_ms;    /* default 0: explicitly wait until completed */
} RouteConfig_t;

typedef struct {
    uint32_t state, fault, route_id;
    uint16_t point_index, point_count, work_id; /* index is zero-based */
    uint8_t phase;              /* 0 plan, 1 turn, 2 forward, 3 left, 4 work */
    uint8_t chassis_fault, fault_address;
    uint32_t received_routes, completed_routes, completed_points;
    uint32_t rx_bytes, frames, crc_errors, length_errors, value_errors;
    uint32_t rx_overflows, rx_faults, partial_timeouts, tx_drops;
    uint32_t last_rx_tick;
    float estimated_x_mm, estimated_y_mm, map_yaw_deg;
    float target_x_mm, target_y_mm, target_yaw_deg;
} RouteStatus_t;

typedef struct {
    uint32_t route_id, tick, count;
    uint16_t point_index, work_id;
    int32_t dx_mm, dy_mm;
    int16_t dyaw_cdeg;
    uint16_t confidence;        /* 0..1000; 0 means no reliable detection */
} RouteVision_t;

extern volatile RouteStatus_t route_status;
extern volatile RouteConfig_t route_config;
extern volatile RouteVision_t route_vision;

/* Init once before starting the scheduler. RobotTask is the only route owner.
 * Route_Task consumes serial input and advances the sequence. It also calls
 * Chassis_Task while a movement is active; no second caller is required then.
 */
void Route_Init(void);
void Route_Task(void);
uint8_t Route_IsActive(void);
void Route_Stop(void); /* Cancel the entire route; no automatic resume. */
void Route_SetMotionInhibit(uint8_t inhibit); /* RobotTask: held KEY0 blocks START. */
RouteResult_t Route_Start(uint32_t route_id);
RouteResult_t Route_CompleteWork(uint32_t route_id, uint16_t point_index, uint8_t success);

/* ISR producer (or serialized test input) -> RobotTask consumer. */
void Route_ReceiveBytes(const uint8_t *data, uint16_t count);
void Route_ReceiveFault(void);
uint16_t Route_CRC16(const uint8_t *data, uint16_t count);

/* Weak, nonblocking hook, called ONCE per work point in RobotTask. Override in
 * application code to initiate visual alignment/grab/place. Later call
 * Route_CompleteWork() in RobotTask when the operation actually finishes.
 * Default implementation waits; it never pretends the operation is complete.
 */
void Route_OnWork(uint32_t route_id, uint16_t point_index, uint16_t work_id);
void Route_OnVision(const RouteVision_t *vision); /* Same protocol/UART, RobotTask. */

#endif
