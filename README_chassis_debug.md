# F103 麦轮底盘：按键位置测试

> 最新默认按键测试已启用 IMU 实时纠偏，请先看 [README_chassis_imu.md](README_chassis_imu.md)。
> 本文说明保留的原位置模式；在空闲时将 `chassis_debug.use_imu=0` 可使用它。

工程：`E:/Inte_handing/car_code1/MDK-ARM/car_code1.uvprojx`。
板子 STM32F103RCT6；KEY0=PC5，低电平按下；四轮 X 排列。
本版替换旧单轮/定时速度测试，只做四轮底盘定距离平移。

## 已设置的实物参数

- 轮径 76 mm，电机直接带轮，传动比 1。
- 轮序 FL/FR/RL/RR = 左前/右前/左后/右后，地址 `{1,2,4,3}`。
- 沿用已经验证的前进方向符号 `{+1,-1,+1,-1}`。
- 按 **1.8°、16 细分**计算，每圈 3200 个命令脉冲；请确认四台电机的 MStep 菜单均为 16。若是其他细分，修改 `chassis_config.microsteps` 的初值；若是 0.9°电机，将 `motor_steps_per_rev` 改为 400。
- 电机菜单保持 `FW_Emm`、`CR_VFOC`、`CAN1_MAP`、500000、校验 0x6B、`S_Vel_IS=Disable`。
- **Response 必须为 Receive 或 Both**：要确认缓存的位置命令已接收，才能发同步启动。只设 Reached/Other/None 不适用于本程序。

接线保持之前验证正常的接法。四台电机共用 CANH/CANL 和一个收发器，TXD 接 PA12、RXD 接 PA11，共地，主干两个物理末端各一个 120Ω 终端。

## 烧录和按键

使用新的 AXF/HEX，重新下载并复位；进入 Keil 调试后按 F5 运行。不要在运动中暂停 MCU。位置命令下发后电机会自行执行，暂停 MCU 也会暂停按键和故障处理。

上电约 1 秒后发送停车命令，等待按键，不自动移动。短按并松开 KEY0，每次只启动一项，完成后保持使能/位置保持，不自动开始下一项。

| 下一项 next_test | 动作 | 默认距离 |
|---|---|---|
| 1 | 前进 | 100 mm |
| 2 | 后退 | 100 mm |
| 3 | 左横移 | 100 mm |
| 4 | 右横移 | 100 mm，然后下一项回到 1 |

默认最高转速 20 RPM，加速度档位 100。静止时可在 Watch 修改 `chassis_debug.distance_mm`（正数）、`rpm`、`acc`、`next_test`。距离只在下一次启动时采样。

- 运动中按 KEY0：消抖后取消整段动作并停车，松开不会再次启动。停止也取消尚未执行的第二个轴。
- 长按至少 800 ms：停车，并把下一项重置为 1，松开不运动。
- 不再有单电机测试、方向解锁长按或按时间自动停车。
- 按键是软件停车，依赖主控和 CAN 链路正常工作，不是独立硬件急停。响应中有短暂的通信等待；单次应答等待上限 100 ms。

## 代码封装与调用

底层运动封装在 `Core/Inc/chassis_motion.h`、`Core/Src/chassis_motion.c`；按键封装在 `chassis_debug.c/.h`。应用程序不需要自己拼 CAN 指令。

以下是**各自独立的调用例子，不要把它们连续执行当成排队**：

```c
#include "chassis_motion.h"

Chassis_MoveForward(200.0f, 20, 100);   // 前进 200 mm
Chassis_MoveForward(-200.0f, 20, 100);  // 后退 200 mm
Chassis_MoveLeft(150.0f, 20, 100);      // 左移 150 mm
Chassis_MoveLeft(-150.0f, 20, 100);     // 右移 150 mm
Chassis_MoveXY(200.0f, -100.0f, 20, 100); // 先前进 200，再右移 100 mm
Chassis_Stop();                       // 取消整段动作
```

参数为距离 mm、最高电机转速 RPM、EMM 加速度档位 acc。`acc=0` 是直接启动，非零时档位越大加速越快，acc 不是 mm/s²。XY 是分轴折线路径，不是同时合成斜向移动。

这些是**非阻塞启动接口**：返回 `CHASSIS_OK` 表示接收了请求，尚不表示已经走完；正在运动时新请求返回 `CHASSIS_BUSY`，不会排队。其余返回值 `CHASSIS_INVALID` 参数错误、`CHASSIS_IO_ERROR` 通信/电机检查失败。

当前 main 已调用 `Chassis_DebugInit()` 和循环 `Chassis_DebugTask()`，按键版无需修改 main。

后续改为自主任务时，用 `Chassis_Init()` 初始化，并在主循环调用 `Chassis_Task()`；替换当前按键测试的 `Chassis_DebugTask()`，不要两个任务同时控制同一底盘。示例：

```c
/* CAN、滤波器和接收中断启动完成后执行一次 */
Chassis_Init();
ChassisResult_t result = Chassis_MoveForward(200.0f, 20, 100);
if (result == CHASSIS_OK) {
    while (Chassis_IsBusy()) {
        Chassis_Task();
        // 此处也可处理你的按键/急停请求，调用 Chassis_Stop()。
        HAL_Delay(1);
    }
    if (chassis_motion.state == CHASSIS_DONE) {
        // 本段完成后才启动下一段。
    }
}
```

后续比赛任务建议用状态机替代上述阻塞等待示例。无需每毫秒重复下发 Move 指令，只需持续执行 Task。接口不支持中断调用。

## 距离换算与标定

```text
电机每圈命令脉冲 = motor_steps_per_rev × microsteps
每毫米脉冲 = 每圈脉冲 × 传动比 / (π × 轮径) × 方向校正系数
目标脉冲 = round(abs(距离mm) × 每毫米脉冲)
```

当前初始值约 13.40252 脉冲/mm，因此 100 mm 对应 1340 脉冲。它是理想运动学换算；麦轮打滑、轮子受压、载重会影响地面距离。

在 `chassis_motion.c` 的 `chassis_config` 初值中，前后和左右分别有 `forward_correction`、`left_correction`，初始均为 1.0。静止时 Watch 修改只在 RAM 生效；确定标定值后应写回源码。

例如命令前进 500 mm，实测只走 480 mm：

```text
新的 forward_correction = 原系数 × 500 / 480
                       = 1.0416667（原系数为 1 时）
```

左右横移单独测量和标定，建议从 100 mm 开始，逐步验证 300、500 mm，重复测量。位置模式保证电机轴转动，不能单独修正整车偏航或打滑；后续仍可加入 IMU/视觉。

## 四轮同步和到位判定

1. 停车、使能四轮，确认健康状态，读取各轮当前实际位置。
2. 四轮各发 `Emm_V5_Pos_Control(..., raF=2, snF=true)`，逐台等待成功 ACK。
3. 全部成功后发 `Emm_V5_Synchronous_motion(0)` 广播同步启动。
4. 每约 25 ms 轮询一个电机的状态、位置和转速。正常情况下每轮约 100 ms 更新一次；通信耗时会使周期延长。
5. 每轮连续两次满足：到位标志为 1、相对启动位置的实际增量接近期望增量（±200 编码单位，约电机轴 ±1.10°）、速度绝对值不大于 1 RPM，四轮均满足才完成。
6. 正常完成保持位置；反馈缺失、失能、堵转或超时，发送四轮停车并记录故障。

`raF=2` 表示相对**当前实际位置**。途中停车后，下一次从实际停点计算，不追赶上一段未完成目标。它与开源项目中的相对上次目标位置 `raF=0` 不同。

命令脉冲（当前 3200/圈）与位置反馈（EMM 65536/圈）是两种单位，封装已换算，不能直接比较。

广播同步会触发同总线上其他设备已缓存的同步命令，因此此总线按当前四轮底盘专用使用；后续其他机构不要同时留下待触发同步命令。故障过程中不发同步启动；下一次移动会重新给四轮写入命令。

单轴距离允许 ±3000 mm，速度 1～120 RPM，并要求计算出的单轴超时不超过 120 秒。超时按距离、速度、加减速时间加余量自动估算，只用于异常停车，不决定正常终点。XY 每个轴独立计时。

## Watch 参数

| 变量 | 含义 |
|---|---|
| `chassis_debug.distance_mm` | 四项按键测试的距离，默认 100 |
| `chassis_debug.rpm / acc` | 最高转速与加速度档位，默认 20 / 100 |
| `chassis_debug.next_test` | 下一次按键测试项 1～4 |
| `chassis_motion.state` | 0空闲、1运动、2完成、3手动停止、4故障 |
| `chassis_motion.fault / fault_address` | 故障原因与电机地址 |
| `chassis_motion.axis` | 当前轴，0前后、1左右 |
| `chassis_motion.target_pulses` | 当前轴每轮命令脉冲的绝对值 |
| `chassis_motion.expected_delta[4]` | 预计电机位置变化，原始有符号反馈单位 |
| `chassis_motion.measured_delta[4]` | 已测位置变化，轮序 FL/FR/RL/RR |
| `chassis_motion.reached_mask` | bit0～3对应四轮，全到位为15/0x0F |
| `chassis_motion.completed_count` | 完成的整个请求次数，XY完成两轴才加1 |
| `chassis_motion.elapsed_ms / timeout_ms` | 当前轴已运行时间/超时上限 |
| `motor_feedback[1..4]` | 按实际 CAN 地址索引的反馈，与轮序不同 |
| `key0_level / key0_press_count` | 松开1按下0 / 消抖后的按下次数 |
| `chassis_task_count / chassis_task_tick` | 按键任务运行次数/毫秒tick |
| `app_boot_stage / app_error` | 正常最终为9 / 0 |

fault：0无故障；1参数错误；2 CAN发送失败；3命令未确认或被拒绝；4电机失能/堵转；5状态、位置或速度反馈缺失；6运动超时。fault_address=0通常表示全局问题。修复原因后可重新短按发起新请求，运行中的故障不会自动重试。

Watch 手动触发：静止时先设置 `command=1..4`，最后递增 `trigger`。`command=0` 请求停车。运行中递增 trigger 不会排队、不会替换当前动作。

## 验证与备份

已完成 Keil ARM Compiler 6.23 编译（0 errors / 0 warnings）和 19 项模拟 HAL 行为测试。覆盖四方向映射、毫米换算、同步前四轮确认、反馈判定、按键消抖/停车、故障停车、XY分轴、配置快照、中断动作后重启及无效参数。测试针对实际 `chassis_motion.c` 和 `chassis_debug.c`；不能替代实物地面测距。

```powershell
powershell -ExecutionPolicy Bypass -File tests/host/build_and_test.ps1
```

旧版工作代码、测试及 HEX/AXF 备份：`tmp/before_position_20261031_190723/`。
旧 F407 工程 `E:/Inte_handing/car_code` 未修改。
