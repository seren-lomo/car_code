# HWT101 接入说明（STM32F103RCT6 / car_code1）

## 接线与传感器设置

本驱动按工程现有配置使用 **UART5、115200、8N1、无流控、接收中断、无 DMA**。

| HWT101 信号 | 主控 |
| --- | --- |
| TX | PD2 / UART5_RX |
| RX | PC12 / UART5_TX（只接收角度时可不接） |
| GND | GND，共地 |
| VCC | 按实际 HWT101 模块规格供电 |

串口信号须为主控兼容的 TTL 电平；若拿到的是 RS232/RS485 版本，需对应的电平转换。
传感器使用维特标准二进制协议：11 字节帧、帧头 0x55、角度类型 0x53，
末字节为前 10 字节和的低 8 位。设置传感器输出角度，建议 50～100 Hz。
驱动默认 200 ms 无有效角度判定掉线，因此至少使用 10 Hz 输出。
传感器实际波特率必须与 UART5 一致；若仍是 9600，用上位机设置或同步修改 CubeMX UART5。
驱动不会自动改波特率、写 EEPROM、校准或恢复出厂。

## 工程已接入的位置

- `imu/imu.c`、`imu/imu.h`：替换原 MDK-ARM 下移植来的实现，Keil 的 imu 分组已更新路径。
- `main.c`：完成外设和底盘初始化后调用一次 `IMU_Init()`，开启 UART5 单字节接收中断。
- `usart.c`：HAL 接收完成/错误回调转交驱动，仅处理 huart5。其他串口驱动以后在同一回调中分发，不要重复定义 HAL 回调。
- `stm32f1xx_it.c`：沿用 CubeMX 的 UART5_IRQHandler → HAL_UART_IRQHandler，不新增重复 IRQ。
- `freertos.c`：CommTask 每 2 ms 调用 `IMU_Task()`，在任务中完成校验、解析和浮点换算。
  RobotTask 执行原来的 KEY0 底盘测试；调度器启动后的 main while 不再承担业务。

USART2 不再被 IMU 占用，TIM3 不再被 IMU 初始化。原维特 SDK 文件保留，
新的只读数据接收驱动不依赖其自动扫描、命令处理或串口初始化。

FreeRTOS 堆改为 20480 字节。CMSIS-RTOS v1 本工程任务栈单位为 32 位 word：

| 任务 | 优先级 | 栈 word / 字节 | 当前工作 |
| --- | --- | --- | --- |
| CommTask | High | 768 / 3072 | IMU 解析；优先于底盘任务，避免 CAN 等待阻塞解析 |
| RobotTask | AboveNormal | 1024 / 4096 | 现有 KEY0 底盘位置测试 |
| MonitorTask | BelowNormal | 256 / 1024 | 预留 |
| DisplayTask | Low | 384 / 1536 | 预留 |

任务创建失败进入 Error_Handler。任务栈和堆同步写入 .ioc；
后续增加视觉、屏幕等业务后，应再测任务栈余量和堆余量。
UART5 IRQ 优先级保留 6，中断内不调用 FreeRTOS API、不做浮点运算。

当前 CubeMX 所带的 FreeRTOS `portable/RVDS/ARM_CM3` 需要 ARM Compiler 5，
Keil 已选择本机安装的 **V5.06 update 7 (build 960)**。
若 Keil 已经打开旧工程，关闭并重新打开 `MDK-ARM/car_code1.uvprojx`，再 Rebuild，
不要用旧窗口缓存的 ARM Compiler 6 配置构建。CubeMX 重新生成后也应核对编译器和 imu 分组/包含路径。

## 上电查看 Watch

烧录后 Run，旋转 IMU，查看以下变量：

| 变量 | 含义 / 正常表现 |
| --- | --- |
| `imu_debug.init_status` | 0=HAL_OK，只代表成功启动接收，不代表已收到传感器 |
| `imu_debug.rx_bytes` | UART5 接收的字节数，持续增加 |
| `imu_debug.valid_frames` | 校验成功的所有标准帧数（包含角度以外的帧） |
| `imu_data.angle_count` | 校验成功的 0x53 角度帧数，持续增加 |
| `imu_data.online` | 1=最近 200 ms 有有效角度；0=无角度、掉线或接收故障 |
| `imu_data.angle_deg[2]` | 原始 Z 轴航向角，单位度；量程约 -180～180 |
| `imu_data.yaw_deg` | 软件归零后的航向角，[-180,180) |
| `imu_data.yaw_total_deg` | 连续累计航向，可跨越 ±180；断流后连续性重新建立 |
| `imu_debug.checksum_errors` | 错帧候选数，持续增加时检查波特率、接线和干扰 |
| `imu_debug.uart_errors` / `last_uart_error` | UART 错误次数 / 最近 HAL 错误位掩码 |
| `imu_debug.overflows` | 接收队列满次数，正常为 0 |
| `imu_debug.rearm_errors` | 启动/重启接收失败次数，正常为 0 |
| `imu_debug.continuity_resets` | 断流或接收故障后，累计角连续性重建次数 |

只收到 0x52 陀螺仪帧或其他帧不会将角度置为在线。`gyro_dps`、`acc_g`
仅在收到对应 0x52/0x51 时有效；HWT101 若仅输出角度，这两组值不更新。
`valid_mask` 的 1/2/4 位分别表示自最近接收故障以来收到过加速度/角速度/角度。
这是“收到过”的标志，角速度/加速度是否新鲜还需检查各自 `gyro_tick`/`acc_tick`。

掉线后保留最后数据供排查，使用数据前必须检查在线状态。接收队列保存每个字节的接收时刻，
旧积压数据不会因刚被解析就被视为新鲜。
累计角假设连续两帧之间旋转小于 180°；断流时无法恢复丢失的整圈，
重连后 `yaw_total_deg` 从当前相对角重新累计。

## 业务调用

```c
#include "imu.h"

IMU_Data_t sample;
if (IMU_GetData(&sample)) {
    float yaw = sample.yaw_deg;
    /* 使用 yaw；车辆安装方向/正负号还需实车确认。 */
} else {
    /* 航向反馈不可用，由上层决定等待、停止或其他故障处理。 */
}

/* 车体摆正后调用，下一帧有效角度作为零点。 */
IMU_ZeroYaw();
```

也可在 Watch 把 `imu_zero_request` 改为 1，下一帧处理后自动回到 0。
这是软件相对零点，不会修改传感器内部校准。归零仅执行一次，不要放在循环中反复调用。

`IMU_Task()` 只能由 CommTask 调用；其他任务用 `IMU_GetData()` 获取一致快照。
`imu_init()`、`imu_scan()` 仅保留移植接口兼容，main 已初始化后不要再次调用初始化。
新代码推荐上面的接口，以便识别掉线。

**本次接入只读取航向，尚未将 IMU 加入底盘航向 PID，也未增加自动转向。**
HWT101 提供航向，不能单独解决麦轮平移打滑造成的定位误差；后续仍需视觉对位等定位信息。

## 验证

`tests/imu/build_and_test.ps1` 用 HAL mock 调用实际驱动，覆盖有符号角度换算、
分段接收、噪声/校验/残帧恢复、±180° 连续累计、软件归零、掉线与计时回绕、
积压数据时效、队列溢出、串口错误恢复、接收重启失败重试和其他串口隔离。
这不能替代实际传感器波特率、接线、安装方向和数据更新测试。

原文件备份：`tmp/before_imu_20261101_164542`。
