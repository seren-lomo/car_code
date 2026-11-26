#ifndef CHASSIS_PATH_H
#define CHASSIS_PATH_H
#include "chassis_heading.h"

#define CHASSIS_PATH_MAX_NODES 64U
typedef struct {
    float pass_tolerance_mm; /* Ordinary waypoint radius; default 3 mm. */
    float corner_rpm;        /* Translation budget at corners; default 5 RPM. */
} ChassisPathConfig_t;
typedef struct {
    uint16_t node_index, node_count, route_index;
    uint8_t active;
    float remaining_mm, forward_rpm, left_rpm, junction_rpm;
    uint16_t speed_limit_rpm;
} ChassisPathStatus_t;
extern volatile ChassisPathConfig_t chassis_path_config;
extern volatile ChassisPathStatus_t chassis_path;

/* RobotTask only. Body coordinates relative to the beginning of the batch.
 * Heading stays fixed; the last node stops. Busy requests never replace motion. */
uint8_t Chassis_PathBuild(void);
uint8_t Chassis_PathAppend(float forward_mm, float left_mm, uint16_t rpm, uint16_t route_index);
ChassisResult_t Chassis_PathStart(void);

/* Internal hooks for chassis_heading.c. */
void Chassis_PathCancel(void);
uint8_t Chassis_PathCompute(float forward_mm, float left_mm, uint32_t dt_ms, float *f_rpm, float *l_rpm);
#endif
