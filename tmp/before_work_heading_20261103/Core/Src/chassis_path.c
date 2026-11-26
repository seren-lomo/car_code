#include "chassis_path.h"

typedef struct { float f, l, exit_rpm; uint16_t rpm, route_index; } PathNode;
static PathNode nodes[CHASSIS_PATH_MAX_NODES];
static uint16_t count, index;
static float vf, vl, kp, acceleration, pass_radius, corner_rpm, stop_radius;
static float mm_per_rpm, forward_scale, left_scale;
volatile ChassisPathConfig_t chassis_path_config = {3.0f, 5.0f};
volatile ChassisPathStatus_t chassis_path;

static float af(float x) { return x < 0 ? -x : x; }
static float minf(float a, float b) { return a < b ? a : b; }
static float root(float x)
{
    float r;
    uint8_t i;
    if(x <= 0) return 0;
    r = x > 1 ? x : 1;
    for(i = 0; i < 20; i++) r = 0.5f * (r + x / r);
    return r;
}
static float length(float f, float l) { return root(f*f + l*l); }

uint8_t Chassis_PathBuild(void)
{
    if(Chassis_IsBusy()) return 0;
    count = 0;
    return 1;
}
uint8_t Chassis_PathAppend(float f, float l, uint16_t rpm, uint16_t route_index)
{
    if(Chassis_IsBusy() || count == CHASSIS_PATH_MAX_NODES || rpm == 0 || rpm > CHASSIS_MAX_RPM ||
       !(af(f) <= CHASSIS_MAX_DISTANCE_MM && af(l) <= CHASSIS_MAX_DISTANCE_MM)) return 0;
    nodes[count].f = f; nodes[count].l = l;
    nodes[count].rpm = rpm; nodes[count].route_index = route_index;
    count++;
    return 1;
}
void Chassis_PathCancel(void)
{
    chassis_path.active = 0;
    chassis_path.forward_rpm = chassis_path.left_rpm = vf = vl = 0;
}
static float junction(uint16_t i)
{
    float a, b, c, d, la, lb, dot;
    if(i + 1U == count) return 0;
    a = nodes[i].f - (i ? nodes[i-1].f : 0);
    b = nodes[i].l - (i ? nodes[i-1].l : 0);
    c = nodes[i+1].f - nodes[i].f; d = nodes[i+1].l - nodes[i].l;
    la = length(a,b); lb = length(c,d);
    if(la < 0.1f || lb < 0.1f) return minf(nodes[i].rpm, nodes[i+1].rpm);
    dot = (a*c+b*d)/(la*lb);
    if(dot < -0.9f) return 0; /* Reversal must physically pass through zero. */
    if(dot > 0.999f) return minf(nodes[i].rpm, nodes[i+1].rpm);
    return minf(corner_rpm, minf(nodes[i].rpm, nodes[i+1].rpm));
}
ChassisResult_t Chassis_PathStart(void)
{
    ChassisResult_t result;
    uint16_t i;
    float seconds = 0, pf = 0, pl = 0, timeout, travel, bound;
    if(Chassis_IsBusy()) return CHASSIS_BUSY;
    if(!count || !(chassis_path_config.pass_tolerance_mm >= 1 && chassis_path_config.pass_tolerance_mm <= 10) ||
       !(chassis_path_config.corner_rpm >= 1 && chassis_path_config.corner_rpm <= 15) ||
       !(chassis_config.wheel_diameter_mm >= 20 && chassis_config.wheel_diameter_mm <= 500) ||
       !(chassis_config.motor_turns_per_wheel_turn >= 0.1f && chassis_config.motor_turns_per_wheel_turn <= 100) ||
       !(chassis_config.forward_correction >= 0.1f && chassis_config.forward_correction <= 10) ||
       !(chassis_config.left_correction >= 0.1f && chassis_config.left_correction <= 10)) return CHASSIS_INVALID;
    forward_scale = chassis_config.forward_correction; left_scale = chassis_config.left_correction;
    mm_per_rpm = 3.14159265359f * chassis_config.wheel_diameter_mm /
        (60.0f * chassis_config.motor_turns_per_wheel_turn);
    for(i = 0; i < count; i++) {
        seconds += (af(nodes[i].f-pf)*forward_scale + af(nodes[i].l-pl)*left_scale) /
            (mm_per_rpm*nodes[i].rpm);
        pf = nodes[i].f; pl = nodes[i].l;
    }
    timeout = seconds*3000.0f + 6000.0f + count*1500.0f;
    if(!(timeout <= CHASSIS_MAX_TIMEOUT_MS)) return CHASSIS_INVALID;
    /* Reuse the CAN preparation, feedback freshness, stall, IMU and stop guards. */
    result = Chassis_MoveForwardHold(0, nodes[0].rpm);
    if(result != CHASSIS_OK) return result;
    kp = chassis_heading_config.distance_kp; acceleration = chassis_heading_config.ramp_rpm_per_s;
    pass_radius = chassis_path_config.pass_tolerance_mm; corner_rpm = chassis_path_config.corner_rpm;
    stop_radius = chassis_heading_config.distance_tolerance_mm;
    /* Use a conservative upper bound on physical mm/s for braking distance. */
    mm_per_rpm /= minf(forward_scale, left_scale);
    for(i = 0; i < count; i++) nodes[i].exit_rpm = junction(i);
    /* Look through short segments so a later corner or slower segment is not
     * discovered too late. Executed once at start, never in the control loop. */
    for(i = count-1U; i > 0U; i--) {
        travel = length(nodes[i].f-nodes[i-1].f, nodes[i].l-nodes[i-1].l) - 2*pass_radius;
        if(travel < 0) travel = 0;
        bound = root(nodes[i].exit_rpm*nodes[i].exit_rpm + 2*acceleration*travel/mm_per_rpm);
        nodes[i-1].exit_rpm = minf(nodes[i-1].exit_rpm, bound);
    }
    index = 0; vf = vl = 0;
    chassis_path.node_index = 0; chassis_path.node_count = count;
    chassis_path.route_index = nodes[0].route_index; chassis_path.speed_limit_rpm = nodes[0].rpm;
    chassis_path.active = 1;
    chassis_heading.action = 2; chassis_motion.axis = 3;
    chassis_motion.timeout_ms = (uint32_t)timeout;
    return CHASSIS_OK;
}
uint8_t Chassis_PathCompute(float f, float l, uint32_t dt, float *out_f, float *out_l)
{
    float df, dl, dist, exit_speed, sf, sl, seg, uf = 0, ul = 0, along = 0, cross = 0;
    float distance_to_brake, limit, desired_f, desired_l, scale, change, correction_f, correction_l;
    uint8_t done = 0;
    for(;;) {
        df = nodes[index].f-f; dl = nodes[index].l-l; dist = length(df,dl);
        exit_speed = nodes[index].exit_rpm;
        sf = nodes[index].f - (index ? nodes[index-1].f : 0);
        sl = nodes[index].l - (index ? nodes[index-1].l : 0); seg = length(sf,sl);
        if(seg > 0.1f) {
            uf = sf/seg; ul = sl/seg; along = df*uf+dl*ul; cross = -df*ul+dl*uf;
        }
        if(index+1U == count) { done = (uint8_t)(dist <= stop_radius); break; }
        if(dist > pass_radius && !(seg > 0.1f && along <= 0 && af(cross) <= pass_radius &&
           dist <= pass_radius + mm_per_rpm*(af(vf)+af(vl))*dt*0.001f)) break;
        if(exit_speed == 0 && (af(vf)+af(vl) > 0.5f || dist > stop_radius)) break;
        index++;
    }
    distance_to_brake = dist;
    if(exit_speed > 0 && index+1U < count) distance_to_brake -= pass_radius;
    if(distance_to_brake < 0) distance_to_brake = 0;
    limit = minf(nodes[index].rpm, exit_speed + kp*distance_to_brake);
    limit = minf(limit, root(exit_speed*exit_speed + 2*acceleration*distance_to_brake/mm_per_rpm));
    if(done) desired_f = desired_l = 0;
    else {
        if(seg > 0.1f) {
            if(along < 0) limit = -limit;
            scale = af(uf)*forward_scale + af(ul)*left_scale;
            correction_f = -ul*kp*cross*forward_scale;
            correction_l = uf*kp*cross*left_scale;
            desired_f = uf*forward_scale*limit/scale + correction_f;
            desired_l = ul*left_scale*limit/scale + correction_l;
        } else {
            scale = af(df)*forward_scale + af(dl)*left_scale;
            desired_f = scale > 0 ? limit*df*forward_scale/scale : 0;
            desired_l = scale > 0 ? limit*dl*left_scale/scale : 0;
        }
        scale = af(desired_f)+af(desired_l);
        if(scale > nodes[index].rpm) { desired_f *= nodes[index].rpm/scale; desired_l *= nodes[index].rpm/scale; }
    }
    change = acceleration*(dt > 50U ? 50U : dt)*0.001f;
    df = desired_f-vf; dl = desired_l-vl; scale = af(df)+af(dl);
    if(scale > change) { df *= change/scale; dl *= change/scale; }
    vf += df; vl += dl;
    if(af(vf)+af(vl) >= 0.5f) done = 0;
    chassis_path.node_index = index; chassis_path.route_index = nodes[index].route_index;
    chassis_path.remaining_mm = dist; chassis_path.forward_rpm = vf; chassis_path.left_rpm = vl;
    chassis_path.junction_rpm = exit_speed; chassis_path.speed_limit_rpm = nodes[index].rpm;
    chassis_heading.target_mm = 0; chassis_heading.traveled_mm = 0; chassis_heading.remaining_mm = dist;
    *out_f = vf; *out_l = vl;
    return done;
}
