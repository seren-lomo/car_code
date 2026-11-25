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
   转向测试详见本文末尾，默认不改变这四项循环。
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
平移接口限制单轴；新增原地转向接口见下文。尚不执行边前进边绕弧的轨迹或斜向运动。
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

## 2026-11-01 截图结果解读

用户提供的 Watch 截图显示：

| 字段 | 截图值 | 解读 |
| --- | --- | --- |
| `app_boot_stage` / `app_error` | 9 / 0 | 应用初始化走到任务就绪，没有进入全局 Error_Handler |
| `key0_press_count` | 2 | 已识别两次有效按键按下；结合目标值可知本段是后退测试 |
| `phase` | 3 | heading 后端已经完成；注意这不是 `chassis_debug.state=3`，后者表示被取消 |
| `target_mm` | -500 | 后退 500 mm |
| `traveled_mm` / `forward_mm` | -497.164429 | 编码器推算后退约 497.16 mm |
| `remaining_mm` | -2.835571 | 目标减已走，仍需后退约 2.84 mm；负数不是过冲标志 |
| `left_mm` | -0.449936 | 轮式里程推算右移约 0.45 mm，不等于地面实测侧偏 |
| `target_yaw_deg` / `yaw_deg` | -1.483154 / -0.703125 | 段起始航向 / 最近一次控制读取的航向；不用先归零 |
| `yaw_error_deg` | -0.780029 | 按配置方向换算后的误差，绝对值约 0.78° |
| `cycles` | 0x16E = 366 | 完成了 366 次控制计算 |
| `cycle_ms` | 0x14 = 20 | 最近一次计算周期为 20 ms |
| `settled_ms` | 0x104 = 260 | 位置、方向和停转条件连续满足 260 ms |
| `translation_rpm` / `correction_rpm` / `wheel_rpm` | 全 0 | 完成后的停车命令状态 |
| `pending_address` / `pending_function` | 全 0 | 没有等待中的电机请求 |

当前默认距离容差 3 mm、航向容差 1°、稳定时间 250 ms，所以这些数值满足 DONE 条件。
这是一次终点状态截图，说明当前终点达标，不能证明全程没有偏航，也不能据此断言实车地面误差为 2.84 mm。
若要评估全程，记录 `yaw_error_deg` 的时序或用逻辑分析/串口输出；若要评估距离精度，实际测量起终点。
完成后 heading 数据保留该段最后一次计算结果，`yaw_deg` 不再跟随转动车体实时更新；实时传感器读数看 `imu_data`。

## 程序调用过程

启动流程：

```text
main
 ├─ HAL_Init / SystemClock_Config / MX_* 外设初始化
 ├─ USER_CAN1_Filter_Init / HAL_CAN_Start / 开启接收通知
 ├─ Chassis_DebugInit → Chassis_Init → 四轮停止
 ├─ IMU_Init → HAL_UART_Receive_IT(huart5)
 ├─ MX_FREERTOS_Init
 └─ osKernelStart
```

并行运行的逻辑：

```text
UART5_IRQHandler → HAL_UART_IRQHandler
  → HAL_UART_RxCpltCallback → IMU_UART_RxCpltCallback → 接收队列
CommTask（每 2 ms）→ IMU_Task → parse_byte / accept_frame → imu_data

RobotTask（每轮 osDelay(1)）→ Chassis_DebugTask
  ├─ PollKey：30 ms 消抖，短按松开生成 command/trigger，运动中按下停车
  ├─ 新命令 → Chassis_MoveForwardHold / Chassis_MoveLeftHold / Chassis_TurnRelative
  │           → start_move：检查参数/忙状态/IMU，复制配置和起始角度，置 RUNNING
  └─ Chassis_Task → 按 control_mode 分发
       ├─ 0：原电机位置模式
       └─ 1：Chassis_HeadingTask
            ├─ 检查 IMU、偏航/转向进度、总超时
            ├─ 准备：issue → 四轮 FE停止/F3使能/3A状态/36位置
            ├─ 运行：issue → 四轮36位置/35转速/3A状态
            ├─ reply_received：确认本次请求的新回复，超时则 fail
            ├─ compute：里程/剩余距离/角度误差 → P/可选D → 斜坡 → 麦轮混合/限幅
            ├─ issue → 四轮 Emm_V5_Vel_Control(F6, sync=true)，逐个确认成功
            └─ Emm_V5_Synchronous_motion(0)：广播同步执行

CAN 接收中断 → HAL_CAN_RxFifo0MsgPendingCallback → motor_feedback
```

每次任务调用只推进当前步骤，不会在业务任务里等待整个动作结束。
`compute()` 内同时满足容差和停转条件超过 250 ms 后，`stop_wheels()` 设置停车指令，置 DONE。
发生错误则 `fail()` 记录故障码/电机地址、停止四轮。

当前平移控制核心关系：

```text
distance_error = target_mm - traveled_mm
translation_rpm = 剩余距离 × distance_kp（限幅、软件斜坡）
yaw_error = wrap(target_yaw - current_yaw) × imu_yaw_sign
rotation_rpm = yaw_kp × yaw_error - yaw_kd × yaw_rate（限幅、软件斜坡）

车轮物理前进方向为正：
FL = forward - left - rotation
FR = forward + left + rotation
RL = forward + left - rotation
RR = forward - left + rotation

再乘各轮 forward_sign，得到实际 EMM 协议方向和 RPM。
```

## 走更远的距离

- 按键：空闲时改 `chassis_debug.distance_mm=1000` 或 `2000`，下一次按键生效；用 `next_test=1` 指定下一次向前。
- 固定默认值：改 `Core/Src/chassis_debug.c` 初始化中的 `.distance_mm`，重新编译烧录。
- 业务调用：`Chassis_MoveForwardHold(2000.0f, 50)`，前进 2 m；负距离代表后退。
- 左右移动：`Chassis_MoveLeftHold(1000.0f, 30)`，左移 1 m；负距离代表右移。

单段上限仍为 `CHASSIS_MAX_DISTANCE_MM=3000`（见 `Core/Inc/chassis_motion.h`）。
超过 3 m 推荐分成多段，每段 DONE 后再发下一段；分段会重新捕获各段起始航向，并不是全路线统一航向基准。
总超时按距离、轮径、减速比和最高 RPM 自动估算；极低速度/过长动作可能因超过 120 s 上限被拒绝。
先增加距离，保持已验证的转速和增益，不需要因为走得远而直接提高增益。
不能在运动中修改 `chassis_heading.target_mm` 等状态变量来追加距离；这些是诊断数据，目标应通过调用/调试参数设置。

## 原地转向、掉头（新增）

```c
Chassis_TurnRelative(90.0f, 20);    // 原地左转90°，俯视逆时针
Chassis_TurnRelative(-90.0f, 20);   // 原地右转90°，俯视顺时针
Chassis_TurnAround(20);            // 原地左转180°掉头
Chassis_TurnRelative(-180.0f, 20);  // 原地右转180°掉头
```

以上仍是独立示例，每次只启动一段，等 DONE 后再发下一段。
第二个参数是电机最高 RPM，不是度/秒。转向允许 ±360°；默认每段超时 60 s，
轮速与车体角速度的关系取决于轴距/轮距，初始用 20 RPM 验证。
这组函数执行“停下 → 原地转向 → 再走”，没有实现一边前进一边绕固定半径的弧线；
弧线控制还需要车体几何尺寸和速度规划。

转向使用连续原始 yaw 增量累计，`+180` 明确向左、`-180` 明确向右；
跨越传感器 ±180° 时不会把目标方向倒过来。进度从本段的 0 开始，IMU 软件归零不影响它。
主动转向不会套用平移时的“偏航超过20°就停车”；仍保留逆向转动超过20°、超过目标太多、
角度断流、CAN/电机错误、堵转、总超时和按键停车检查。
算法假设相邻有效角度之间小于180°；建议50～100 Hz输出，转动期间不要暂停主控调试器。

转向 Watch 字段：

| 字段 | 含义 |
| --- | --- |
| `chassis_heading.action` | 0 平移纠偏、1 原地转向 |
| `target_turn_deg` | 有符号目标角：+左/-右 |
| `turned_deg` | 实际累计转角 |
| `remaining_turn_deg` | 目标减实际累计转角 |
| `yaw_error_deg` | 转向模式下等于剩余转角，允许大于20° |
| `target_yaw_deg` | 目标原始航向，归一化到±180°供观察；控制用累计角而非单个目标值 |

转向时 `target_mm/traveled_mm/remaining_mm` 为0，前后/左右轮式投影仍保留供观察，
不主动控制底盘中心在地面的平移漂移。转向最大旋转分量按调用的 RPM 限制，
平移纠偏的 `max_correction_rpm` 不限制主动转向速度。

单独使用按键选择转向：

1. 空闲时设置 `chassis_debug.use_imu=1`。
2. 设置 `chassis_debug.turn_deg=90`、`turn_rpm=20`（默认已如此）。
3. 设置 `chassis_debug.next_test=5`，下一次短按 KEY0 左转；6为右转；7为左转180°掉头。
4. 或设置 `command=5/6/7`，最后递增 `trigger`，直接从 Watch 启动一次。
5. `include_turn_tests=1` 可将 KEY0 循环扩展为1～7：前进、后退、左移、右移、左转、右转、掉头。
   默认仍为0，保留原1～4循环。动作中按下停车，长按复位下一项为1。

turn_deg 只控制5/6命令，7始终转180°，三项转向均使用turn_rpm。

新增软件验证包含左右90°、左右180°、左右360°、±180环绕、反向安装IMU、转向停止/掉线、
方向配置错误、零角度/非法输入、按键选择、2m平移及“走100mm→左转90°→再走100mm”分段。
完整42项运动测试和19项原位置回归测试通过；转向尚需实车验证。
转向修改前备份：`tmp/before_turn_20261101`，保留用户添加的源文件注释。
