#ifndef CAR_IMU_H
#define CAR_IMU_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HWT101: UART5 PC12/TX, PD2/RX, 115200 8N1, normal 0x55 protocol.
 * The sensor must send angle (0x53) frames at >= 10 Hz.
 */
#define IMU_TIMEOUT_MS 200U
#define IMU_VALID_ACC   0x01U
#define IMU_VALID_GYRO  0x02U
#define IMU_VALID_ANGLE 0x04U

typedef struct {
    float angle_deg[3];       /* Raw roll, pitch, yaw; HWT101 mainly uses yaw. */
    float gyro_dps[3];        /* Only valid if a 0x52 frame was received. */
    float acc_g[3];           /* Only valid if a 0x51 frame was received. */
    float yaw_deg;            /* Software-relative yaw, [-180, 180). */
    float yaw_total_deg;      /* Continuous yaw; resets continuity after a gap. */
    uint32_t angle_count;
    uint32_t angle_tick;
    uint32_t gyro_tick;
    uint32_t acc_tick;
    uint8_t valid_mask;
    uint8_t online;           /* Fresh angle frame, not just UART traffic. */
} IMU_Data_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t overflows;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t rearm_errors;
    uint32_t continuity_resets;
    HAL_StatusTypeDef init_status;
} IMU_Debug_t;

extern volatile IMU_Data_t imu_data;
extern volatile IMU_Debug_t imu_debug;
extern volatile uint8_t imu_zero_request; /* Set to 1 in Watch to zero next angle. */

HAL_StatusTypeDef IMU_Init(void);  /* Once, after MX_UART5_Init(). */
void IMU_Task(void);              /* One consumer only: CommTask, every 2 ms. */
uint8_t IMU_GetData(IMU_Data_t *out); /* Snapshot; returns 1 only for fresh yaw. */
void IMU_ZeroYaw(void);           /* Software zero on next valid angle frame. */
void IMU_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void IMU_UART_ErrorCallback(UART_HandleTypeDef *uart);

/* Compatibility with the imported interface. Units: g, degrees/s, degrees.
 * New code should use IMU_GetData() and check its return value.
 */
void imu_init(void);
void imu_scan(float *acc, float *gyro, float *angle);

#ifdef __cplusplus
}
#endif
#endif
