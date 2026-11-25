#include "chassis_motion.h"
#include "can.h"
#include "Emm_V5.h"

#define PI_F 3.14159265358979323846f
#define RESPONSE_TIMEOUT_MS 100U
#define POLL_INTERVAL_MS 25U
#define POSITION_TOLERANCE 200 /* About 1.10 motor degrees, not ground mm. */
#define STOP_SPEED_RPM 1

volatile ChassisConfig_t chassis_config = {
    {1, 2, 4, 3}, {1, -1, 1, -1}, 76.0f, 1.0f, 200, 16, 1.0f, 1.0f
};
volatile ChassisMotion_t chassis_motion;
static ChassisConfig_t active_config;
static uint8_t have_active_config;
static float requested[2];
static uint32_t pulses[2], axis_timeout[2];
static uint16_t active_rpm;
static uint8_t active_acc, poll_wheel, settled[4];
static int32_t start_position[4];
static uint32_t axis_started, last_poll;

static float AbsFloat(float x) { return x < 0.0f ? -x : x; }
static void SetFault(uint32_t fault, uint8_t addr)
{
    chassis_motion.fault = fault;
    chassis_motion.fault_address = addr;
}
static uint8_t TxOK(uint8_t addr)
{
    if (can_last_tx_status == HAL_OK) return 1;
    SetFault(CHASSIS_TX_ERROR, addr);
    return 0;
}
static uint8_t WaitAck(uint8_t addr, uint8_t function, uint32_t previous)
{
    uint32_t started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < RESPONSE_TIMEOUT_MS) {
        if (motor_feedback[addr].ack_count != previous &&
            motor_feedback[addr].ack_function == function) {
            if (motor_feedback[addr].ack_status == 0x02U) {
                HAL_Delay(3);
                return 1;
            }
            break;
        }
    }
    SetFault(CHASSIS_ACK_ERROR, addr);
    return 0;
}
static uint8_t ReadFeedback(uint8_t addr, SysParams_t param, volatile uint32_t *counter)
{
    uint32_t before = *counter;
    uint32_t started;
    Emm_V5_Read_Sys_Params(addr, param);
    if (!TxOK(addr)) return 0;
    started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < RESPONSE_TIMEOUT_MS) {
        if (*counter != before) { HAL_Delay(3); return 1; }
    }
    SetFault(CHASSIS_FEEDBACK_ERROR, addr);
    return 0;
}
static uint8_t ReadFlags(uint8_t addr)
{
    if (!ReadFeedback(addr, S_FLAG, &motor_feedback[addr].flags_count)) return 0;
    if ((motor_feedback[addr].flags & 0x0DU) == 0x01U) return 1;
    SetFault(CHASSIS_MOTOR_ERROR, addr);
    return 0;
}
static void StopAll(void)
{
    uint8_t i;
    if (!have_active_config) return;
    for (i = 0; i < 4U; ++i) {
        Emm_V5_Stop_Now(active_config.address[i], false);
        HAL_Delay(3);
    }
}
static void Fail(void) { StopAll(); chassis_motion.state = CHASSIS_FAULT; }

/* Validate before any command and snapshot configuration for the whole move. */
static uint8_t Prepare(float forward, float left, uint16_t rpm, uint8_t acc)
{
    uint8_t i, j;
    float per_rev, scale, count, milliseconds;
    active_config = chassis_config;//
    if (!(forward >= -CHASSIS_MAX_DISTANCE_MM && forward <= CHASSIS_MAX_DISTANCE_MM) ||
        !(left >= -CHASSIS_MAX_DISTANCE_MM && left <= CHASSIS_MAX_DISTANCE_MM) ||
        rpm == 0U || rpm > CHASSIS_MAX_RPM ||
        !(active_config.wheel_diameter_mm >= 20.0f && active_config.wheel_diameter_mm <= 500.0f) ||
        !(active_config.motor_turns_per_wheel_turn >= 0.1f && active_config.motor_turns_per_wheel_turn <= 100.0f) ||
        active_config.motor_steps_per_rev == 0U || active_config.motor_steps_per_rev > 1000U ||
        active_config.microsteps == 0U || active_config.microsteps > 256U ||
        !(active_config.forward_correction >= 0.1f && active_config.forward_correction <= 10.0f) ||
        !(active_config.left_correction >= 0.1f && active_config.left_correction <= 10.0f)) return 0;
    for (i = 0; i < 4U; ++i) {
        if (active_config.address[i] == 0U || active_config.address[i] > 4U ||
            (active_config.forward_sign[i] != 1 && active_config.forward_sign[i] != -1)) return 0;
        for (j = 0; j < i; ++j)
            if (active_config.address[i] == active_config.address[j]) return 0;
    }
		// 计算电机转一圈需要的总脉冲数（基本步数 × 细分数，比如 200 × 16 = 3200）
    per_rev = (float)active_config.motor_steps_per_rev * active_config.microsteps;
    requested[0] = forward;// 暂存 X 轴（前后）请求距离
    requested[1] = left;// 暂存 Y 轴（左右）请求距离
    for (i = 0; i < 2U; ++i) {
			// 核心公式：脉冲比例 = (电机一圈脉冲数 × 减速比) / (PI × 轮径)
        // 算出来的是“底盘每移动 1 毫米，电机需要多少个脉冲”
        scale = per_rev * active_config.motor_turns_per_wheel_turn /
                (PI_F * active_config.wheel_diameter_mm);
        scale *= i == 0U ? active_config.forward_correction : active_config.left_correction;
        // 总脉冲数 = 绝对距离(mm) × 脉冲比例
			  count = AbsFloat(requested[i]) * scale;
        if (!(count <= 10000000.0f)) return 0;
        if (!(count / per_rev * 65536.0f <= 1000000000.0f)) return 0;
       // 四舍五入转成整数，存入 pulses 数组
			pulses[i] = (uint32_t)(count + 0.5f);
        if (requested[i] != 0.0f && pulses[i] == 0U) return 0;
			
			// 估算运动时间 = (总脉冲 / 每圈脉冲) / 转速RPM × 60000(转毫秒) + 2000毫秒(余量)
        milliseconds = count / per_rev * 60000.0f / rpm + 2000.0f;
        if (acc != 0U) milliseconds += (256U - acc) * 0.1f * rpm;
        if (!(milliseconds <= (float)CHASSIS_MAX_TIMEOUT_MS)) return 0;
        axis_timeout[i] = (uint32_t)milliseconds;
        if (axis_timeout[i] < 3000U) axis_timeout[i] = 3000U;
    }
    return 1;
}
static uint8_t StartAxis(uint8_t axis)
{
    uint8_t i, addr, direction;
    int32_t delta;
    int8_t sign;
    uint32_t previous;
    float per_rev = (float)active_config.motor_steps_per_rev * active_config.microsteps;
    chassis_motion.axis = axis;
    chassis_motion.target_pulses = pulses[axis];
    chassis_motion.pulses_per_mm = pulses[axis] / AbsFloat(requested[axis]);
    chassis_motion.timeout_ms = axis_timeout[axis];
    chassis_motion.elapsed_ms = 0;
    chassis_motion.reached_mask = 0;
    for (i = 0; i < 4U; ++i) {
        addr = active_config.address[i];
			// 1. 发送紧急停止，清除电机内部的旧状态
        previous = motor_feedback[addr].ack_count;
        Emm_V5_Stop_Now(addr, false);
        if (!TxOK(addr) || !WaitAck(addr, 0xFEU, previous)) return 0;
       // 2. 重新使能电机闭环控制
 			previous = motor_feedback[addr].ack_count;
        Emm_V5_En_Control(addr, true, false);
        if (!TxOK(addr) || !WaitAck(addr, 0xF3U, previous) || !ReadFlags(addr)) return 0;
        // 3. 读取当前电机的绝对编码器位置，存入 start_position，作为误差计算的基准点
			if (!ReadFeedback(addr, S_CPOS, &motor_feedback[addr].position_count)) return 0;
        start_position[i] = motor_feedback[addr].position;
        settled[i] = 0;
        chassis_motion.measured_delta[i] = 0;
    }
		// 将脉冲数换算为 Emm_V5 协议内部的位置单位（乘以 65536）
    delta = (int32_t)((float)pulses[axis] * 65536.0f / per_rev + 0.5f);
    for (i = 0; i < 4U; ++i) {
			// 判断基本符号：请求大于 0 就是正转(1)，小于 0 就是反转(-1)
        sign = requested[axis] < 0.0f ? -1 : 1;
			// ★ 核心运动学解算 (麦克纳姆轮/全向轮横移特性) ★
        // 如果当前是横移 (axis == 1)，0号和3号轮子的转向必须反转。
        // （麦轮横移时，对角线的轮子转向相同，同侧的轮子转向相反）
        if (axis == 1U && (i == 0U || i == 3U)) sign = -sign;
			// 乘上每个电机的物理安装正方向（比如左边两个电机和右边两个电机如果是镜像安装的，需要反向）
        sign *= active_config.forward_sign[i];
			// 转换为协议需要的 0 或 1 格式
        direction = sign < 0 ? 1U : 0U;
			// 记录预期编码器应该变化的值，留给后面检查是否走到了
        chassis_motion.expected_delta[i] = sign * delta;
        addr = active_config.address[i];
        previous = motor_feedback[addr].ack_count;
			// ★ 下发位置控制指令 ★ 
        // 注意最后一个参数 true (snF=true)，这代表“多机同步”：
        // 电机收到这条指令后会存入缓冲区，但绝对不会转动。
        /* raF=2: relative CURRENT position, including after an interrupted move.
         * snF=true: buffer all wheels before the synchronous trigger. */
        Emm_V5_Pos_Control(addr, direction, active_rpm, active_acc, pulses[axis], 2U, true);
        if (!TxOK(addr) || !WaitAck(addr, 0xFDU, previous)) return 0;
    }
    /* Chassis-only bus: other devices must not have pending sync commands. */
		// 向 CAN 总线的地址 0（广播地址）发送同步启动指令。
    // 这时，刚才挂在缓冲区里的 4 个电机，会在同一微秒内同时启动，保证底盘不会跑偏。
    Emm_V5_Synchronous_motion(0U);
    if (!TxOK(0U)) return 0;
    HAL_Delay(3);
    axis_started = HAL_GetTick();
    last_poll = axis_started;
    poll_wheel = 0;
    chassis_motion.state = CHASSIS_RUNNING;
    return 1;
}
void Chassis_Init(void)
{
    uint8_t i;
    active_config = chassis_config;
    for (i = 0; i < 4U; ++i)
        if (active_config.address[i] == 0U || active_config.address[i] > 4U) return;
    have_active_config = 1;
    StopAll();
    chassis_motion.state = CHASSIS_IDLE;
}
uint8_t Chassis_IsBusy(void) { return chassis_motion.state == CHASSIS_RUNNING; }
void Chassis_Stop(void)
{
    StopAll();
    if (chassis_motion.state != CHASSIS_FAULT) chassis_motion.state = CHASSIS_STOPPED;
}
ChassisResult_t Chassis_MoveXY(float forward, float left, uint16_t rpm, uint8_t acc)
{
    ChassisConfig_t previous_config;
    if (Chassis_IsBusy()) return CHASSIS_BUSY;
    previous_config = active_config;
    if (!Prepare(forward, left, rpm, acc)) {
        active_config = previous_config;
        SetFault(CHASSIS_CONFIG_ERROR, 0);
        chassis_motion.state = CHASSIS_FAULT;
        return CHASSIS_INVALID;
    }
    have_active_config = 1;
    active_rpm = rpm;
    active_acc = acc;
    chassis_motion.fault = 0;
    chassis_motion.fault_address = 0;
    chassis_motion.requested_forward_mm = forward;
    chassis_motion.requested_left_mm = left;
    chassis_motion.reached_mask = 0;
    if (pulses[0] == 0U && pulses[1] == 0U) {
        uint8_t i;
        for (i = 0; i < 4U; ++i) {
            chassis_motion.expected_delta[i] = 0;
            chassis_motion.measured_delta[i] = 0;
        }
        chassis_motion.target_pulses = 0;
        chassis_motion.pulses_per_mm = 0;
        chassis_motion.elapsed_ms = 0;
        chassis_motion.timeout_ms = 0;
        chassis_motion.state = CHASSIS_DONE;
        ++chassis_motion.completed_count;
        return CHASSIS_OK;
    }
    if (!StartAxis(pulses[0] ? 0U : 1U)) { Fail(); return CHASSIS_IO_ERROR; }
    return CHASSIS_OK;
}
ChassisResult_t Chassis_MoveForward(float mm, uint16_t rpm, uint8_t acc)
{ return Chassis_MoveXY(mm, 0.0f, rpm, acc); }
ChassisResult_t Chassis_MoveLeft(float mm, uint16_t rpm, uint8_t acc)
{ return Chassis_MoveXY(0.0f, mm, rpm, acc); }
void Chassis_Task(void)
{
    uint8_t i, addr;
    uint32_t now;
    int64_t delta, error;
    if (!Chassis_IsBusy()) return;
    now = HAL_GetTick();
    chassis_motion.elapsed_ms = now - axis_started;
    if (chassis_motion.elapsed_ms >= chassis_motion.timeout_ms) {
        SetFault(CHASSIS_MOVE_TIMEOUT, 0); Fail(); return;
    }
    if ((uint32_t)(now - last_poll) < POLL_INTERVAL_MS) return;
    last_poll = now;
    i = poll_wheel;
    addr = active_config.address[i];
    if (!ReadFlags(addr) ||
        !ReadFeedback(addr, S_CPOS, &motor_feedback[addr].position_count) ||
        !ReadFeedback(addr, S_VEL, &motor_feedback[addr].speed_count)) { Fail(); return; }
    delta = (int64_t)motor_feedback[addr].position - start_position[i];
    if (delta < -2147483647LL || delta > 2147483647LL) {
        SetFault(CHASSIS_FEEDBACK_ERROR, addr); Fail(); return;
    }
    chassis_motion.measured_delta[i] = (int32_t)delta;
    error = delta - chassis_motion.expected_delta[i];
    if ((motor_feedback[addr].flags & 0x02U) != 0U &&
        error >= -POSITION_TOLERANCE && error <= POSITION_TOLERANCE &&
        motor_feedback[addr].speed_rpm >= -STOP_SPEED_RPM &&
        motor_feedback[addr].speed_rpm <= STOP_SPEED_RPM) {
        if (settled[i] < 2U) ++settled[i];
    } else settled[i] = 0;
    if (settled[i] >= 2U) chassis_motion.reached_mask |= (uint8_t)(1U << i);
    else chassis_motion.reached_mask &= (uint8_t)~(1U << i);
    poll_wheel = (uint8_t)((i + 1U) % 4U);
    if (chassis_motion.reached_mask == 0x0FU) {
        if (chassis_motion.axis == 0U && pulses[1] != 0U) {
            if (!StartAxis(1U)) Fail();
        } else {
            chassis_motion.state = CHASSIS_DONE; /* Keep position hold. */
            ++chassis_motion.completed_count;
        }
    }
}
