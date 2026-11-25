#ifndef CAR_ROUTE_KEYS_H
#define CAR_ROUTE_KEYS_H
#include "main.h"
typedef struct {
    uint32_t key1_level, key1_press_count, area_requests, ignored_presses;
    uint32_t pending_area, pose_preparing, pose_result; /* auto-origin before area signal */
    uint8_t both_pressed;
} RouteKeysDebug_t;
extern volatile RouteKeysDebug_t route_keys_debug;
void RouteKeys_Init(void); /* Once after Route_Init, before scheduler. */
void RouteKeys_Task(void); /* RobotTask only, before Route_Task. */
#endif
