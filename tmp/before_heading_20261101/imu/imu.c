#include "imu.h"
#include "usart.h"

#define RX_SIZE 512U
#define RX_MASK (RX_SIZE - 1U)
#define FRAME_SIZE 11U

volatile IMU_Data_t imu_data;
volatile IMU_Debug_t imu_debug;
volatile uint8_t imu_zero_request;

static uint8_t rx_byte;
static uint8_t rx_queue[RX_SIZE];
static uint32_t rx_ticks[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault;
static uint8_t initialized;
static uint8_t frame[FRAME_SIZE], frame_used;
static uint8_t have_previous;
static float previous_yaw, yaw_zero;

static float wrap_angle(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int16_t frame_i16(uint8_t offset)
{
    return (int16_t)((uint16_t)frame[offset] |
                     ((uint16_t)frame[offset + 1U] << 8));
}

static void arm_rx(void)
{
    if (HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) != HAL_OK) {
        imu_debug.rearm_errors++;
        rx_fault = 1U;
    }
}

HAL_StatusTypeDef IMU_Init(void)
{
    /* No baud scan, EEPROM writes, calibration, or timer initialization. */
    IMU_Data_t empty_data = {0};
    IMU_Debug_t empty_debug = {0};
    imu_data = empty_data;
    imu_debug = empty_debug;
    rx_head = rx_tail = 0U;
    rx_fault = frame_used = have_previous = 0U;
    yaw_zero = previous_yaw = 0.0f;
    imu_zero_request = 0U;
    initialized = 1U;
    imu_debug.init_status = HAL_UART_Receive_IT(&huart5, &rx_byte, 1U);
    if (imu_debug.init_status != HAL_OK) imu_debug.rearm_errors++;
    return imu_debug.init_status;
}

void IMU_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint8_t byte;
    uint16_t head, next;
    if (uart != &huart5 || !initialized) return;
    byte = rx_byte;
    arm_rx();
    imu_debug.rx_bytes++;
    head = rx_head;
    next = (uint16_t)((head + 1U) & RX_MASK);
    if (next == rx_tail) {
        imu_debug.overflows++;
        rx_fault = 1U;
        return;
    }
    rx_queue[head] = byte;
    rx_ticks[head] = HAL_GetTick();
    __DMB();
    rx_head = next;
}

void IMU_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart5 || !initialized) return;
    imu_debug.uart_errors++;
    imu_debug.last_uart_error = uart->ErrorCode;
    rx_fault = 1U;
    /* HAL ends reception on ORE; for FE/NE/PE it may still be receiving. */
    if (uart->RxState == HAL_UART_STATE_READY) arm_rx();
}

static void accept_frame(uint32_t tick)
{
    uint8_t axis;
    imu_debug.valid_frames++;
    if (frame[1] == 0x51U) {
        for (axis = 0U; axis < 3U; axis++)
            imu_data.acc_g[axis] = frame_i16((uint8_t)(2U + axis * 2U)) * (16.0f / 32768.0f);
        imu_data.acc_tick = tick;
        imu_data.valid_mask |= IMU_VALID_ACC;
    } else if (frame[1] == 0x52U) {
        for (axis = 0U; axis < 3U; axis++)
            imu_data.gyro_dps[axis] = frame_i16((uint8_t)(2U + axis * 2U)) * (2000.0f / 32768.0f);
        imu_data.gyro_tick = tick;
        imu_data.valid_mask |= IMU_VALID_GYRO;
    } else if (frame[1] == 0x53U) {
        float yaw, relative;
        uint8_t continuous = have_previous &&
            ((uint32_t)(tick - imu_data.angle_tick) <= IMU_TIMEOUT_MS);
        for (axis = 0U; axis < 3U; axis++)
            imu_data.angle_deg[axis] = frame_i16((uint8_t)(2U + axis * 2U)) * (180.0f / 32768.0f);
        yaw = imu_data.angle_deg[2];
        if (imu_zero_request) {
            yaw_zero = yaw;
            imu_zero_request = 0U;
            imu_data.yaw_total_deg = 0.0f;
        } else if (continuous) {
            imu_data.yaw_total_deg += wrap_angle(yaw - previous_yaw);
        } else {
            if (imu_data.angle_count != 0U) imu_debug.continuity_resets++;
            imu_data.yaw_total_deg = wrap_angle(yaw - yaw_zero);
        }
        relative = wrap_angle(yaw - yaw_zero);
        imu_data.yaw_deg = relative;
        previous_yaw = yaw;
        have_previous = 1U;
        imu_data.angle_tick = tick;
        imu_data.angle_count++;
        imu_data.valid_mask |= IMU_VALID_ANGLE;
    }
}

static void parse_byte(uint8_t byte, uint32_t tick)
{
    uint8_t sum = 0U, i, skip;
    if (frame_used == 0U && byte != 0x55U) return;
    frame[frame_used++] = byte;
    if (frame_used < FRAME_SIZE) return;
    for (i = 0U; i < FRAME_SIZE - 1U; i++) sum = (uint8_t)(sum + frame[i]);
    if (sum == frame[10]) {
        accept_frame(tick);
        frame_used = 0U;
    } else {
        imu_debug.checksum_errors++;
        /* Keep a possible next header, including when frames were truncated. */
        for (skip = 1U; skip < FRAME_SIZE; skip++)
            if (frame[skip] == 0x55U) break;
        frame_used = (uint8_t)(FRAME_SIZE - skip);
        for (i = 0U; i < frame_used; i++) frame[i] = frame[i + skip];
    }
}

void IMU_Task(void)
{
    uint16_t processed;
    uint32_t key;
    if (!initialized) return;
    /* Bound each call so this task also has time for other communication. */
    for (processed = 0U; processed < RX_SIZE; processed++) {
        uint8_t byte, fault;
        uint32_t tick;
        uint16_t tail;
        key = __get_PRIMASK();
        __disable_irq();
        fault = rx_fault;
        if (fault) {
            rx_tail = rx_head;
            rx_fault = 0U;
        }
        tail = rx_tail;
        if (tail == rx_head) {
            __set_PRIMASK(key);
            if (fault) {
                frame_used = have_previous = 0U;
                imu_data.valid_mask = 0U;
            }
            break;
        }
        byte = rx_queue[tail];
        tick = rx_ticks[tail];
        rx_tail = (uint16_t)((tail + 1U) & RX_MASK);
        __set_PRIMASK(key);
        parse_byte(byte, tick);
    }
    key = __get_PRIMASK();
    __disable_irq();
    if (huart5.RxState == HAL_UART_STATE_READY) arm_rx();
    __set_PRIMASK(key);
    imu_data.online = (!rx_fault && (imu_data.valid_mask & IMU_VALID_ANGLE) != 0U &&
        (uint32_t)(HAL_GetTick() - imu_data.angle_tick) <= IMU_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t IMU_GetData(IMU_Data_t *out)
{
    uint32_t key;
    if (out == 0) return 0U;
    key = __get_PRIMASK();
    __disable_irq();
    *out = imu_data;
    out->online = !rx_fault && ((out->valid_mask & IMU_VALID_ANGLE) != 0U) &&
        ((uint32_t)(HAL_GetTick() - out->angle_tick) <= IMU_TIMEOUT_MS);
    __set_PRIMASK(key);
    return out->online;
}

void IMU_ZeroYaw(void)
{
    imu_zero_request = 1U;
}

void imu_init(void)
{
    (void)IMU_Init();
}

void imu_scan(float *acc, float *gyro, float *angle)
{
    IMU_Data_t data;
    uint8_t axis;
    (void)IMU_GetData(&data);
    for (axis = 0U; axis < 3U; axis++) {
        if (acc) acc[axis] = data.acc_g[axis];
        if (gyro) gyro[axis] = data.gyro_dps[axis];
        if (angle) angle[axis] = data.angle_deg[axis];
    }
}
