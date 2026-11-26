# 麦轮底盘：指定距离行驶 + IMU 实时航向纠偏

## 当前实现

适用当前 `car_code1` 的 STM32F103RCT6、X 型麦轮、76 mm 直驱轮：

| 轮子顺序 | 左前 FL | 右前 FR | 左后 RL | 右后 RR |
| --- | --- | --- | --- | --- |
| CAN 地址 | 1 | 2 | 4 | 3 |
| 车辆前进对应的协议有符号方向 | + | - | + | - |

配置沿用 `chassis_config`，启动一段运动时复制配置，运行中更改 Watch 不会改变该段映射。
IMU 接线及数据解析见 `README_imu.md`：UART5，HWT101 TX→PD2，115200、8N1，标准角度帧。
建议 IMU 输出 50～100 Hz。IMU 应与车体刚性固定，Z 轴垂直地面。

电机保持 `FW_Emm`、`CR_VFOC`、`CAN1_MAP`、500000、校验 0x6B；
`Response` 需为 `Receive` 或 `Both`，控制器依赖 FE/F3/F6 命令回复再推进；
保留已验证的 `S_Vel_IS=Disable`。无需把电机菜单改为开环。

本功能使用 **电机速度模式 F6**，电机原有 CR_VFOC 闭环配置保留。
主控负责距离外环和航向外环，不再一次下发整段 FD 位置指令：

1. 停止四轮、使能、检查状态、读取初始轮子位置。准备过程也检查 IMU。
2. 四轮当前位置减去初始位置，按 65536 单位/电机圈换算位移。
3. 对轮子位移做麦轮正解，提取前后/左右平移量，旋转分量在组合中抵消。
4. 剩余距离 P 控制得到平移 RPM，接近终点自动减速；若超过终点可低速反向修正。
5. 一段动作开始时记录 IMU 原始航向，运动中用环绕后的航向误差得到旋转修正 RPM。
6. 叠加平移与旋转，对四轮整体同比例限幅，四个 F6 均收到成功回复后发送广播同步触发。
7. 距离、航向同时处于容差内，命令趋于零，且四轮实测转速均不超过 1 RPM，持续 250 ms 后停止并返回 DONE。

控制周期目标下限 20 ms，实际还包括 CAN 应答等待；可用 `chassis_heading.cycle_ms` 查看实际周期。
请求/应答使用非阻塞状态机，每次 RobotTask 调用推进，不会等待整个应答超时才扫描按键。
底层 CAN 单次发送自身仍有最长 20 ms 的传输等待。
CommTask 优先级高于 RobotTask，继续每 2 ms 解析 IMU。

## 先做一次 500 mm 按键测试

1. 关闭并重新打开 `MDK-ARM/car_code1.uvprojx`，Rebuild，再烧录、Run。
   如果 Keil 弹出外部文件修改提示，应重新加载；工程已加入 `chassis_heading.c`。
2. 确认 `imu_data.online=1`，角度帧数持续增加，四轮地址/方向仍是上述映射。
3. 保留当前默认 `chassis_debug.distance_mm=500`、`rpm=50`、`use_imu=1`。
   首次确认纠偏方向时，可先在 Watch 把 `rpm` 降到 **20**；后续再恢复 50。
4. 平放车体，记下车头朝向和轮子起点，短按并松开 KEY0（PC5，低有效）。
   首次向前走 500 mm，保持该段起始车头方向。上电不会自行运动。
5. 等 `chassis_debug.state=2`，测量地面实际距离。下次短按依次后退、左移、右移，各 500 mm。
6. 运动中再次按下 KEY0 会停车，该次松开不会启动下一项。长按 800 ms 将下一测试项复位为前进。

`use_imu=0` 可在空闲时切回原电机位置模式对比；改变它不会切换正在执行的动作。
`chassis_debug.acc` 只用于原位置测试。IMU 模式发送 motor acc=0，使用主控软件斜坡，
调加减速应改 `chassis_heading_config.ramp_rpm_per_s`。

**先确认纠偏方向：** 此处 `imu_yaw_sign=+1` 表示俯视车体逆时针旋转时，IMU 原始 yaw 增加。
可手动缓慢转动车体，在 `imu_data.angle_deg[2]` 中核对，注意跨越 ±180° 时的数值环绕。
若逆时针时原始 yaw 减少，应在下一次运动前设置 `chassis_heading_config.imu_yaw_sign=-1`。
设置错误会形成正反馈；默认偏航超过 20° 停车。

## 业务封装调用

只在 RobotTask 中发起和轮询运动，不从多个任务同时发送底盘命令。

```c
#include "chassis_heading.h"

/* 一次调用，正值前进，负值后退；50 是所有轮子的最高命令 RPM。 */
ChassisResult_t result = Chassis_MoveForwardHold(500.0f, 50);

/* 其他动作示例：每段结束后再启动下一段。 */
Chassis_MoveForwardHold(-300.0f, 30);  // 后退 300 mm
Chassis_MoveLeftHold(200.0f, 30);      // 左移 200 mm
Chassis_MoveLeftHold(-200.0f, 30);     // 右移 200 mm
```

上面四条是独立示例，不是可连续无等待执行的路径。
返回 `CHASSIS_OK` 只代表动作已接受；正在运动时再次启动返回 `CHASSIS_BUSY`，不会排队。
持续调用 `Chassis_Task()`，直到 `chassis_motion.state` 为 DONE、STOPPED 或 FAULT。
当前工程 `Chassis_DebugTask()` 已经调用它，不需要再重复轮询。

需要把两段动作串起来，可在 RobotTask 里用状态机，例如：

```c
/* 示意：由业务启动条件把 route 从 0 改为 1，一段结束再发下一段。
 * 不要在 KEY0 测试同时运行这段路径状态机。
 */
static uint8_t route;
switch (route) {
case 1:
    if (Chassis_MoveForwardHold(500.0f, 30) == CHASSIS_OK) route = 2;
    else route = 0;  // 检查返回值并由业务决定是否重试
    break;
case 2:
    if (chassis_motion.state == CHASSIS_DONE) route = 3;
    else if (chassis_motion.state == CHASSIS_FAULT ||
             chassis_motion.state == CHASSIS_STOPPED) route = 0;
    break;
case 3:
    if (Chassis_MoveLeftHold(200.0f, 30) == CHASSIS_OK) route = 4;
    else route = 0;
    break;
case 4:
    if (chassis_motion.state != CHASSIS_RUNNING) route = 0;
    break;
default:
    break;
}
```

当前调试面板空闲的 `command=0` 不会取消业务 API 启动的运动。
若要从 Watch 显式停止，把非零 command 改为 0；若它已经为 0，则再递增 trigger。
KEY0 的运动中停车始终有效。
`Chassis_Stop()` 同样适用于两种模式。原 `Chassis_MoveForward/Left/XY` 保留为电机位置控制，
与新模式共享 BUSY 状态，不会同时启动两个后端。

## Watch 变量

| 变量 | 含义 |
| --- | --- |
| `chassis_debug.distance_mm` / `rpm` | 按键测试距离/四轮最高命令转速，下一段生效 |
| `chassis_debug.use_imu` | 1=实时纠偏，0=原位置模式 |
| `chassis_debug.state` | 0 空闲、1 运行、2 完成、3 已取消、4 故障 |
| `chassis_debug.fault` / `fault_address` | 故障码 / 电机地址；0 地址表示整体故障 |
| `chassis_motion.control_mode` | 0=位置、1=距离+IMU速度控制 |
| `chassis_heading.phase` | 0 未启动、1 初始化、2 行驶/稳定、3 完成、4 停车/故障 |
| `chassis_heading.target_mm` | 本段有符号目标距离 |
| `chassis_heading.traveled_mm` | 本段沿运动轴的有符号轮式里程 |
| `chassis_heading.remaining_mm` | 目标减去当前里程，允许负值（已超过目标） |
| `chassis_heading.forward_mm` / `left_mm` | 轮式里程的前后/左右投影 |
| `chassis_heading.target_yaw_deg` / `yaw_deg` | 起始航向/当前原始 IMU 航向 |
| `chassis_heading.yaw_error_deg` | 最短角误差，正值要求车体逆时针修正 |
| `chassis_heading.translation_rpm` / `correction_rpm` | 软件斜坡后的平移/旋转分量，最终轮子还会整体限幅 |
| `chassis_heading.wheel_rpm[0..3]` | 左前、右前、左后、右后的协议有符号 RPM |
| `chassis_heading.cycles` / `cycle_ms` | 已计算的周期数 / 最近实际周期时长 |
| `chassis_heading.settled_ms` | 距离/航向/速度同时满足条件的持续时间 |
| `chassis_heading.pending_address` / `pending_function` | 当前等待的电机/协议功能码 |

IMU 纠偏模式中 `chassis_motion.target_pulses`、`expected_delta` 不用于判定到位，
请看 `chassis_heading`；`measured_delta` 仍记录各轮原始反馈增量。
航向使用原始角度，不受运行中软件 `IMU_ZeroYaw()` 改变相对零点影响。

故障码：1 配置不合法；2 CAN 发送失败；3 电机命令未成功回复；4 电机未使能/堵转保护等状态异常；
5 位置/速度/状态反馈超时；6 整段或准备超时；7 IMU 角度掉线（200 ms）；
8 偏航超过配置限制；9 某轮有至少 5 RPM 命令但连续约 1 秒没有有效转动。
发生故障立即尝试停止四轮，不会自动恢复行驶。排除问题后重新触发一段动作。
CAN 已断开时，主控无法保证停止指令送达电机；测试时应保留可直接切断动力的方式。

## 调参和精度边界

`chassis_heading_config` 在每段开始时复制，Watch 修改在下一段生效：

| 参数 | 默认值 | 调整依据 |
| --- | --- | --- |
| `distance_kp` | 0.20 RPM/mm | 太大可能过冲，太小则靠近终点较慢 |
| `yaw_kp` | 1.5 差动RPM/度 | 确认正负方向后，纠偏慢可逐步增加；摇摆则减小 |
| `yaw_kd` | 0 | 默认仅 P；需要阻尼时再小幅增加，角速度由连续角度帧差分并滤波 |
| `max_correction_rpm` | 12 | 最大旋转分量，轮子最终仍不超过调用的 max_rpm |
| `ramp_rpm_per_s` | 80 | 平移和旋转分量的软件变化率 |
| `distance_tolerance_mm` | 3 | 轮式里程到位容差，不能当作地面绝对定位精度 |
| `yaw_tolerance_deg` | 1 | 航向到位容差 |
| `max_yaw_error_deg` | 20 | 超出立即故障停车 |
| `imu_yaw_sign` | +1 | 逆时针时原始 yaw 增加为 +1，否则 -1 |

轮距/轴距无需先猜测：当前航向增益直接输出差动轮速，通过实车调节。
此功能限制单轴平移，不执行任意曲线或斜向运动。
里程来自电机编码器和车体麦轮正解，保持航向时近似本段前后/左右距离；
没有融合外部地面位置，也没有积分 IMU 加速度。轮子空转、侧滑以及已经发生的地面横向偏移无法靠航向反馈消除，
抓取放置前仍需视觉等绝对对位。

地面距离标定可分别改 `chassis_config.forward_correction` 和 `left_correction`：
`新系数 = 旧系数 × 指令距离 / 实测距离`，重复多次取平均。
当前所有调参均只改主控 RAM/源码，不修改电机或 IMU EEPROM。

## 软件验证

`tests/heading/build_and_test.ps1` 编译实际底盘控制、按键、IMU 驱动，
用异步 CAN 回复和四轮动态模型测试，不连接实际硬件。
覆盖 500 mm 正反向及横移、±180° 环绕、轮速不一致、4° 外部偏航扰动、距离标定、
模式互斥/切换、IMU 掉线、缺失/旧反馈、命令 ACK 缺失、CAN 失败、堵转、
纠偏方向错误、非法参数、按键停车/长按、配置快照、轮速限幅、超调回退、零距离、
软件归零、总超时、业务 API 不受空闲调试面板干扰、计时回绕和连续分段。

四向 500 mm 模型测试中，终点轮式里程约 ±497.2 mm、航向误差约 0.93～0.98°，符合设定容差；
这些是软件模型结果，不能作为实车精度承诺。
原位置模式的 19 项测试在 `tests/host`，IMU 协议测试在 `tests/imu`。

本次修改前备份：`tmp/before_heading_20261101`。
