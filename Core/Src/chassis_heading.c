#include "chassis_heading.h"
#include "imu.h"
#include "chassis_path.h"
#include "can.h"
#include "Emm_V5.h"

#define PI_F 3.14159265358979323846f
#define PERIOD_MS 20U
#define RESPONSE_MS 60U
#define PREPARE_MS 2000U
#define SETTLE_MS 250U
#define STALL_MS 1000U

volatile ChassisHeadingConfig_t chassis_heading_config = {
    0.20f, 1.5f, 0.0f, 12.0f, 120.0f, 3.0f, 1.0f, 20.0f, -1
};
volatile ChassisHeading_t chassis_heading;
volatile ChassisHeadingFaultSnapshot_t chassis_heading_fault_snapshot;

static ChassisConfig_t wheels;
static ChassisHeadingConfig_t tuning;
static uint8_t have_config, preparing, step, waiting, pending_function;
static uint16_t rpm_limit;
static uint32_t previous_counter, request_tick, started, last_cycle, cycle_started, settle_tick;
static uint8_t settling;
static int32_t origin[4], position[4], previous_position[4];
static int16_t measured_speed[4], next_rpm[4];
static uint32_t stalled_ms[4];
static float mm_per_count, correction_factor;
static float translation, rotation, previous_yaw, yaw_rate;
static uint32_t previous_angle_tick, previous_angle_count;
static IMU_Data_t sample;
static float turn_previous_yaw;
static uint32_t turn_previous_count, turn_previous_tick;

static float absf(float v) { return v < 0.0f ? -v : v; }
static float clamp(float v, float lo, float hi)//限幅函数
{ return v < lo ? lo : (v > hi ? hi : v); }
static float wrap(float v)//角度过零处理函数
{
    while (v >= 180.0f) v -= 360.0f;
    while (v < -180.0f) v += 360.0f;
    return v;
}
static float approach(float current, float target, float limit)//斜坡函数
{ return current + clamp(target - current, -limit, limit); }

static void capture_fault(uint32_t fault, uint8_t address)
{
    uint8_t i, addr;
    uint32_t now = HAL_GetTick();
    chassis_heading_fault_snapshot.valid = 0U;
    chassis_heading_fault_snapshot.tick = now;
    chassis_heading_fault_snapshot.fault = fault;
    chassis_heading_fault_snapshot.fault_address = address;
    chassis_heading_fault_snapshot.action = chassis_heading.action;
    chassis_heading_fault_snapshot.pending_address = chassis_heading.pending_address;
    chassis_heading_fault_snapshot.pending_function = chassis_heading.pending_function;
    chassis_heading_fault_snapshot.target_yaw_deg = chassis_heading.target_yaw_deg;
    chassis_heading_fault_snapshot.yaw_deg = chassis_heading.yaw_deg;
    chassis_heading_fault_snapshot.yaw_error_deg = chassis_heading.yaw_error_deg;
    chassis_heading_fault_snapshot.correction_rpm = chassis_heading.correction_rpm;
    chassis_heading_fault_snapshot.imu_yaw_sign = tuning.imu_yaw_sign;
    for (i = 0U; i < 4U; i++) {
        addr = wheels.address[i];
        chassis_heading_fault_snapshot.address[i] = addr;
        chassis_heading_fault_snapshot.forward_sign[i] = wheels.forward_sign[i];
        chassis_heading_fault_snapshot.command_rpm[i] = chassis_heading.wheel_rpm[i];
        chassis_heading_fault_snapshot.feedback_rpm[i] = 0;
        chassis_heading_fault_snapshot.feedback_age_ms[i] = 0xFFFFFFFFU;
        chassis_heading_fault_snapshot.motor_flags[i] = 0U;
        if (addr >= 1U && addr <= 4U) {
            chassis_heading_fault_snapshot.feedback_rpm[i] = motor_feedback[addr].speed_rpm;
            chassis_heading_fault_snapshot.motor_flags[i] = motor_feedback[addr].flags;
            if (motor_feedback[addr].speed_count != 0U)
                chassis_heading_fault_snapshot.feedback_age_ms[i] = now - motor_feedback[addr].speed_tick;
        }
    }
    __DMB();
    chassis_heading_fault_snapshot.valid = 1U;
}

static void stop_wheels(void)//底层急停函数
{
    uint8_t i;
    if (!have_config) return;
    /* Immediate individual stops also cancel any queued synchronous movement.
     * Do not send a sync trigger after aborting a partially queued batch. */
    for (i = 0U; i < 4U; i++) Emm_V5_Stop_Now(wheels.address[i], false);
    translation = rotation = 0.0f;
    Chassis_PathCancel();
    chassis_heading.translation_rpm = chassis_heading.correction_rpm = 0.0f;
    for (i = 0U; i < 4U; i++) chassis_heading.wheel_rpm[i] = 0;
    waiting = 0U;
    chassis_heading.pending_address = chassis_heading.pending_function = 0U;
}
static void fail(uint32_t fault, uint8_t address)//故障处理
{
    capture_fault(fault, address);
    chassis_motion.fault = fault;
    chassis_motion.fault_address = address;
    chassis_motion.state = CHASSIS_FAULT;
    chassis_heading.phase = 4U;
    stop_wheels();
}
void Chassis_HeadingStop(void)//正常停车
{
    stop_wheels();
    chassis_heading.phase = 4U;
    if (chassis_motion.state != CHASSIS_FAULT) chassis_motion.state = CHASSIS_STOPPED;
}

static uint8_t valid_config(float mm, uint16_t rpm)//参数安全校验
{
    uint8_t i, j;
    if (!(mm >= -CHASSIS_MAX_DISTANCE_MM && mm <= CHASSIS_MAX_DISTANCE_MM) ||
        rpm == 0U || rpm > CHASSIS_MAX_RPM ||
        !(wheels.wheel_diameter_mm >= 20.0f && wheels.wheel_diameter_mm <= 500.0f) ||
        !(wheels.motor_turns_per_wheel_turn >= 0.1f && wheels.motor_turns_per_wheel_turn <= 100.0f) ||
        !(wheels.forward_correction >= 0.1f && wheels.forward_correction <= 10.0f) ||
        !(wheels.left_correction >= 0.1f && wheels.left_correction <= 10.0f) ||
        !(tuning.distance_kp >= 0.01f && tuning.distance_kp <= 2.0f) ||
        !(tuning.yaw_kp > 0.0f && tuning.yaw_kp <= 10.0f) ||
        !(tuning.yaw_kd >= 0.0f && tuning.yaw_kd <= 2.0f) ||
        !(tuning.max_correction_rpm >= 1.0f && tuning.max_correction_rpm <= 30.0f) ||
        !(tuning.ramp_rpm_per_s >= 10.0f && tuning.ramp_rpm_per_s <= 300.0f) ||
        !(tuning.distance_tolerance_mm >= 0.5f && tuning.distance_tolerance_mm <= 20.0f) ||
        !(tuning.yaw_tolerance_deg >= 0.2f && tuning.yaw_tolerance_deg <= 5.0f) ||
        !(tuning.max_yaw_error_deg >= 5.0f && tuning.max_yaw_error_deg <= 45.0f) ||
        tuning.yaw_tolerance_deg >= tuning.max_yaw_error_deg ||
        (tuning.imu_yaw_sign != 1 && tuning.imu_yaw_sign != -1)) return 0U;
    for (i = 0U; i < 4U; i++) {
        if (wheels.address[i] < 1U || wheels.address[i] > 4U ||
            (wheels.forward_sign[i] != 1 && wheels.forward_sign[i] != -1)) return 0U;
        for (j = 0U; j < i; j++) if (wheels.address[i] == wheels.address[j]) return 0U;
    }
    return 1U;
}

static ChassisResult_t start_move(float mm, uint16_t rpm, uint8_t axis)
{
    float nominal_ms;
    ChassisHeading_t empty = {0};
    if (Chassis_IsBusy()) return CHASSIS_BUSY;
    Chassis_PathCancel();
    if (axis == 2U && !(mm >= -CHASSIS_MAX_TURN_DEG && mm <= CHASSIS_MAX_TURN_DEG)) {
        chassis_motion.fault = CHASSIS_CONFIG_ERROR;
        chassis_motion.fault_address = 0U;
        chassis_motion.state = CHASSIS_FAULT;
        return CHASSIS_INVALID;
    }
    /* Validate using temporary snapshots; failed requests do not change the
     * addresses used to stop an earlier accepted move. */
    {
        ChassisConfig_t old_wheels = wheels;
        wheels = chassis_config;
        tuning = chassis_heading_config;
        if (!valid_config(mm, rpm)) {
            wheels = old_wheels;
            chassis_motion.fault = CHASSIS_CONFIG_ERROR;
            chassis_motion.fault_address = 0U;
            chassis_motion.state = CHASSIS_FAULT;
            return CHASSIS_INVALID;
        }
    }
    have_config = 1U;
    chassis_motion.control_mode = 1U;
    chassis_motion.fault = CHASSIS_NO_FAULT;
    chassis_motion.fault_address = 0U;
    chassis_motion.axis = axis;
    chassis_motion.reached_mask = 0U;
    chassis_motion.elapsed_ms = 0U;
    chassis_motion.requested_forward_mm = axis == 0U ? mm : 0.0f;
    chassis_motion.requested_left_mm = axis == 1U ? mm : 0.0f;
    chassis_motion.target_pulses = 0U; /* Not a motor position command. */
    chassis_motion.pulses_per_mm = 0.0f;
    chassis_heading = empty;
    chassis_heading.action = axis == 2U ? 1U : 0U;
    chassis_heading.target_mm = axis == 2U ? 0.0f : mm;
    chassis_heading.target_turn_deg = axis == 2U ? mm : 0.0f;
    chassis_heading.remaining_turn_deg = chassis_heading.target_turn_deg;
    rpm_limit = rpm;
    correction_factor = axis == 0U ? wheels.forward_correction : wheels.left_correction;
    mm_per_count = PI_F * wheels.wheel_diameter_mm /
        (65536.0f * wheels.motor_turns_per_wheel_turn);
    nominal_ms = absf(mm) * correction_factor /
        (PI_F * wheels.wheel_diameter_mm * rpm / (60.0f * wheels.motor_turns_per_wheel_turn)) * 3000.0f + 6000.0f;
    if (axis == 2U) nominal_ms = (float)CHASSIS_TURN_TIMEOUT_MS;
    if (!(nominal_ms <= (float)CHASSIS_MAX_TIMEOUT_MS)) {
        fail(CHASSIS_CONFIG_ERROR, 0U);
        return CHASSIS_INVALID;
    }
    chassis_motion.timeout_ms = (uint32_t)nominal_ms;
    if (!IMU_GetData(&sample)) {
        fail(CHASSIS_IMU_OFFLINE, 0U);
        return CHASSIS_IO_ERROR;
    }
    /* Raw angle makes a software IMU_ZeroYaw() during a move harmless. */
    chassis_heading.target_yaw_deg = sample.angle_deg[2];
    turn_previous_yaw = sample.angle_deg[2];
    turn_previous_count = sample.angle_count;
    turn_previous_tick = sample.angle_tick;
    if (axis == 2U) chassis_heading.target_yaw_deg =
        wrap(sample.angle_deg[2] + mm * tuning.imu_yaw_sign);
    previous_yaw = sample.angle_deg[2];
    previous_angle_tick = sample.angle_tick;
    previous_angle_count = sample.angle_count;
    yaw_rate = translation = rotation = 0.0f;
    settling = 0U;
    step = waiting = 0U;
    preparing = 1U;
    started = HAL_GetTick();
    last_cycle = started;
    cycle_started = started;
    chassis_heading.phase = 1U;
    chassis_motion.state = CHASSIS_RUNNING;
    return CHASSIS_OK;
}
ChassisResult_t Chassis_MoveForwardHold(float mm, uint16_t max_rpm)
{ return start_move(mm, max_rpm, 0U); }
ChassisResult_t Chassis_MoveLeftHold(float mm, uint16_t max_rpm)
{ return start_move(mm, max_rpm, 1U); }
ChassisResult_t Chassis_TurnRelative(float degrees, uint16_t max_rpm)
{ return start_move(degrees, max_rpm, 2U); }
ChassisResult_t Chassis_TurnAround(uint16_t max_rpm)
{ return Chassis_TurnRelative(180.0f, max_rpm); }

/* No spin-wait for motor ACKs: one outstanding operation, checked on the next
 * RobotTask tick. Polling the key and IMU therefore continues during a timeout. */
static volatile uint32_t *counter(uint8_t addr, uint8_t function)
{
    if (function == 0x36U) return &motor_feedback[addr].position_count;
    if (function == 0x35U) return &motor_feedback[addr].speed_count;
    if (function == 0x3AU) return &motor_feedback[addr].flags_count;
    return &motor_feedback[addr].ack_count;
}
static uint8_t reply_received(uint8_t addr, uint8_t function, uint8_t i)//接收到的位置/速度更新到数组里
{
    if (*counter(addr, function) != previous_counter) {
        if (function == 0x36U) position[i] = motor_feedback[addr].position;
        else if (function == 0x35U) measured_speed[i] = motor_feedback[addr].speed_rpm;
        else if (function == 0x3AU) {
            if ((motor_feedback[addr].flags & 0x0DU) != 0x01U) {
                fail(CHASSIS_MOTOR_ERROR, addr); return 0U;
            }
        } else {
            if (motor_feedback[addr].ack_function != function) return 0U;
            if (motor_feedback[addr].ack_status != 0x02U) {
                fail(CHASSIS_ACK_ERROR, addr); return 0U;
            }
        }
        return 1U;
    }
    return 0U;
}
static void issue(uint8_t i, uint8_t function)//CAN 指令并开始计时
{
    uint8_t addr = wheels.address[i];
    int16_t rpm = next_rpm[i];
    pending_function = function;
    chassis_heading.pending_address = addr;
    chassis_heading.pending_function = function;
    previous_counter = *counter(addr, function);
    request_tick = HAL_GetTick();
    waiting = 1U;
    switch (function) {
    case 0xFEU: Emm_V5_Stop_Now(addr, false); break;
    case 0xF3U: Emm_V5_En_Control(addr, true, false); break;
    case 0x36U: Emm_V5_Read_Sys_Params(addr, S_CPOS); break;
    case 0x35U: Emm_V5_Read_Sys_Params(addr, S_VEL); break;
    case 0x3AU: Emm_V5_Read_Sys_Params(addr, S_FLAG); break;
    default:
        Emm_V5_Vel_Control(addr, rpm < 0 ? 1U : 0U,
            (uint16_t)(rpm < 0 ? -rpm : rpm), 0U, true);
        break;
    }
    if (can_last_tx_status != HAL_OK) fail(CHASSIS_TX_ERROR, addr);
}

static uint8_t compute(uint32_t now)
{
    float q[4], forward, left, error, desired, rotate, max_change;
    float f, l, r, wheel[4], largest = 0.0f;
    uint8_t i, stopped = 1U, path_done = 0U;
    float path_f = 0.0f, path_l = 0.0f;
    uint16_t wheel_limit;
    uint32_t dt = now - last_cycle;
    if (dt == 0U) dt = 1U;
    chassis_heading.cycle_ms = dt;
    last_cycle = now;
    for (i = 0U; i < 4U; i++) {
        int64_t delta = (int64_t)position[i] - origin[i];
        int64_t change = (int64_t)position[i] - previous_position[i];
        if (delta < -2147483647LL || delta > 2147483647LL) {
            fail(CHASSIS_FEEDBACK_ERROR, wheels.address[i]); return 0U;
        }
        chassis_motion.measured_delta[i] = (int32_t)delta;
        q[i] = (float)delta * wheels.forward_sign[i] * mm_per_count;
        if (measured_speed[i] < -1 || measured_speed[i] > 1) stopped = 0U;
        if ((next_rpm[i] >= 5 || next_rpm[i] <= -5) &&
            measured_speed[i] >= -1 && measured_speed[i] <= 1 &&
            change > -20 && change < 20) stalled_ms[i] += dt;
        else stalled_ms[i] = 0U;
        previous_position[i] = position[i];
        if (stalled_ms[i] >= STALL_MS) {
            fail(CHASSIS_WHEEL_STALLED, wheels.address[i]); return 0U;
        }
    }
    /* Rotation cancels in these X-mecanum translation projections. */
    forward = (q[0] + q[1] + q[2] + q[3]) * 0.25f / wheels.forward_correction;
    left = (-q[0] + q[1] + q[2] - q[3]) * 0.25f / wheels.left_correction;
    chassis_heading.forward_mm = forward;
    chassis_heading.left_mm = left;
    chassis_heading.traveled_mm = chassis_motion.axis == 0U ? forward :
        (chassis_motion.axis == 1U ? left : 0.0f);
    error = chassis_heading.target_mm - chassis_heading.traveled_mm;
    chassis_heading.remaining_mm = error;

    /* Use angle-derived rate only on a fresh IMU sample; HWT101 need not send
     * a 0x52 gyro frame. Default kd=0 until mounting/direction is verified. */
    if (sample.angle_count != previous_angle_count) {
        uint32_t elapsed = sample.angle_tick - previous_angle_tick;
        if (elapsed != 0U && elapsed <= IMU_TIMEOUT_MS) {
            float measured = wrap(sample.angle_deg[2] - previous_yaw) *
                tuning.imu_yaw_sign * (1000.0f / elapsed);
            yaw_rate += 0.25f * (measured - yaw_rate);
        }
        previous_yaw = sample.angle_deg[2];
        previous_angle_tick = sample.angle_tick;
        previous_angle_count = sample.angle_count;
    }
    desired = absf(error) <= tuning.distance_tolerance_mm ? 0.0f ://目标平移速度
        clamp(tuning.distance_kp * error * correction_factor, -(float)rpm_limit, (float)rpm_limit);
    rotate = absf(chassis_heading.yaw_error_deg) <= tuning.yaw_tolerance_deg ? 0.0f :
        clamp(tuning.yaw_kp * chassis_heading.yaw_error_deg - tuning.yaw_kd * yaw_rate,
              -tuning.max_correction_rpm, tuning.max_correction_rpm);
    if (chassis_heading.action == 1U) {
        desired = 0.0f;
        rotate = absf(chassis_heading.remaining_turn_deg) <= tuning.yaw_tolerance_deg ? 0.0f :
            clamp(tuning.yaw_kp * chassis_heading.remaining_turn_deg - tuning.yaw_kd * yaw_rate,
                  -(float)rpm_limit, (float)rpm_limit);
    }
    if (chassis_heading.action == 2U) {
        path_done = Chassis_PathCompute(forward, left, dt, &path_f, &path_l);
        error = path_done ? 0.0f : tuning.distance_tolerance_mm + 1.0f;
    }
    /* Limit catch-up after scheduler delays; never create a large velocity jump. */
    max_change = tuning.ramp_rpm_per_s * (dt > 50U ? 50U : dt) * 0.001f;
    translation = chassis_heading.action == 2U ? absf(path_f) + absf(path_l) :
        approach(translation, desired, max_change);
    rotation = approach(rotation, rotate, max_change);

    if (absf(error) <= tuning.distance_tolerance_mm &&
        absf(chassis_heading.yaw_error_deg) <= tuning.yaw_tolerance_deg &&
        absf(translation) < 0.5f && absf(rotation) < 0.5f && stopped) {
        if (!settling) { settling = 1U; settle_tick = now; }
        chassis_heading.settled_ms = now - settle_tick;
        if (chassis_heading.settled_ms >= SETTLE_MS) {
            stop_wheels();
            chassis_heading.phase = 3U;
            chassis_motion.state = CHASSIS_DONE;
            chassis_motion.reached_mask = 0x0FU;
            chassis_motion.completed_count++;
            return 0U;
        }
    } else {
        settling = 0U;
        chassis_heading.settled_ms = 0U;
    }

    f = chassis_motion.axis == 0U ? translation : 0.0f;
    l = chassis_motion.axis == 1U ? translation : 0.0f;
    if (chassis_heading.action == 2U) { f = path_f; l = path_l; }
    r = rotation;
    wheel[0] = f - l - r; wheel[1] = f + l + r;
    wheel[2] = f + l - r; wheel[3] = f - l + r;
    wheel_limit = chassis_heading.action == 2U ? chassis_path.speed_limit_rpm : rpm_limit;
    /* Scale the whole vector together so saturation preserves the mixture. */
    for (i = 0U; i < 4U; i++) if (absf(wheel[i]) > largest) largest = absf(wheel[i]);
    for (i = 0U; i < 4U; i++) {
        float command = wheel[i];
        if (largest > wheel_limit) command *= wheel_limit / largest;
        command *= wheels.forward_sign[i];
        next_rpm[i] = (int16_t)(command >= 0.0f ? command + 0.5f : command - 0.5f);
        chassis_heading.wheel_rpm[i] = next_rpm[i];
    }
    chassis_heading.translation_rpm = translation;
    chassis_heading.correction_rpm = rotation;
    chassis_heading.cycles++;
    return 1U;
}

void Chassis_HeadingTask(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t i, function;
    if (chassis_motion.state != CHASSIS_RUNNING) return;
    chassis_motion.elapsed_ms = now - started;
    if (!IMU_GetData(&sample)) { fail(CHASSIS_IMU_OFFLINE, 0U); return; }
    chassis_heading.yaw_deg = sample.angle_deg[2];
    if (chassis_heading.action == 1U) {
        if (sample.angle_count != turn_previous_count) {
            if ((uint32_t)(sample.angle_tick - turn_previous_tick) > IMU_TIMEOUT_MS) {
                fail(CHASSIS_IMU_OFFLINE, 0U); return;
            }
            chassis_heading.turned_deg += wrap(sample.angle_deg[2] - turn_previous_yaw) * tuning.imu_yaw_sign;
            turn_previous_yaw = sample.angle_deg[2];
            turn_previous_count = sample.angle_count;
            turn_previous_tick = sample.angle_tick;
        }
        chassis_heading.remaining_turn_deg = chassis_heading.target_turn_deg - chassis_heading.turned_deg;
        chassis_heading.yaw_error_deg = chassis_heading.remaining_turn_deg;
        /* Planned 90/180-degree motion is valid. Abort substantial wrong-way
         * travel or excessive overshoot instead of applying the translation guard. */
        if ((chassis_heading.target_turn_deg > 0.0f && chassis_heading.turned_deg < -tuning.max_yaw_error_deg) ||
            (chassis_heading.target_turn_deg < 0.0f && chassis_heading.turned_deg > tuning.max_yaw_error_deg) ||
            absf(chassis_heading.turned_deg) > absf(chassis_heading.target_turn_deg) + tuning.max_yaw_error_deg) {
            fail(CHASSIS_HEADING_ERROR, 0U); return;
        }
    } else {
        chassis_heading.yaw_error_deg = wrap(chassis_heading.target_yaw_deg - sample.angle_deg[2]) * tuning.imu_yaw_sign;
        if (absf(chassis_heading.yaw_error_deg) > tuning.max_yaw_error_deg) {
            fail(CHASSIS_HEADING_ERROR, 0U); return;
        }
    }
    if (chassis_motion.elapsed_ms >= chassis_motion.timeout_ms ||
        (preparing && chassis_motion.elapsed_ms >= PREPARE_MS)) {
        fail(CHASSIS_MOVE_TIMEOUT, 0U); return;
    }
    if (waiting) {
        i = (uint8_t)(step % 4U);
        if (!reply_received(wheels.address[i], pending_function, i)) {
            if (chassis_motion.state != CHASSIS_RUNNING) return;
            if ((uint32_t)(now - request_tick) >= RESPONSE_MS)
                fail(pending_function == 0xF6U || pending_function == 0xFEU || pending_function == 0xF3U ?
                     CHASSIS_ACK_ERROR : CHASSIS_FEEDBACK_ERROR, wheels.address[i]);
            return;
        }
        waiting = 0U;
        step++;
        chassis_heading.pending_address = chassis_heading.pending_function = 0U;
    }
    if (preparing && step == 16U) {
        for (i = 0U; i < 4U; i++) {
            origin[i] = previous_position[i] = position[i];
            next_rpm[i] = measured_speed[i] = 0;
            stalled_ms[i] = 0U;
            chassis_motion.measured_delta[i] = chassis_motion.expected_delta[i] = 0;
        }
        preparing = 0U;
        chassis_heading.phase = 2U;
        last_cycle = now;
        cycle_started = now;
        step = 0U;
    }
    if (preparing) {
        function = step < 4U ? 0xFEU : (step < 8U ? 0xF3U : (step < 12U ? 0x3AU : 0x36U));
        issue((uint8_t)(step % 4U), function);
        return;
    }
    if (step == 0U) {
        if ((uint32_t)(now - cycle_started) < PERIOD_MS) return;
        cycle_started = now;
    }
    if (step == 12U && !compute(now)) return;
    if (step == 16U) {
        /* All four F6 commands were accepted before this broadcast trigger.
         * Broadcast ACK behavior differs by motor configuration: monitor fresh
         * wheel position/speed on the next cycle rather than assume an ACK. */
        Emm_V5_Synchronous_motion(0U);
        if (can_last_tx_status != HAL_OK) { fail(CHASSIS_TX_ERROR, 0U); return; }
        step = 0U;
        return;
    }
    function = step < 4U ? 0x36U : (step < 8U ? 0x35U : (step < 12U ? 0x3AU : 0xF6U));
    issue((uint8_t)(step % 4U), function);
}
