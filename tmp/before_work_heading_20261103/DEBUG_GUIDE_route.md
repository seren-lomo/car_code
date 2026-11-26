# 路线执行调用链与联调参数

对应工程：`E:\Inte_handing\car_code1`，STM32F103RCT6。本文整理当前实现和调试方法；2026-11-03默认启用连续路径，详见 [README_continuous.md](README_continuous.md)。二进制协议、接线和树莓派命令见 `README_route.md`。

按键选择启停区的扩展流程见 `README_start_area.md`：空闲时 KEY0→区2、KEY1→区1；运行/等工作时 KEY0 停车、KEY1 忽略。

## 1. 各文件负责什么

| 文件 | 作用 | 后续通常改什么 |
|---|---|---|
| `Core/Src/main.c` | 外设、CAN、底盘、IMU、路线串口初始化，启动 RTOS | 初始化及上电配置 |
| `Core/Src/freertos.c` | RobotTask / CommTask 调度 | 接入机械臂任务轮询；保持底盘单一调用者 |
| `Core/Src/route_uart.c` | UART4 DMA 接收、回复发送、串口错误恢复 | 一般不调；通信异常时排查 |
| `Core/Src/route_keys.c` | PC5/PA15 消抖、选区上报、运行时 KEY0 停车 | 看按键电平/计数；30 ms 消抖一般无需调 |
| `Core/Src/route.c` | 解析整条路线、地图坐标换算、按点执行、等待工作 | 路线容差、工作超时、工作/视觉接口 |
| `Core/Src/chassis_path.c` | 连续折线、普通点切换、拐角/制动前瞻 | `pass_tolerance_mm=3`、`corner_rpm=5` |
| `Core/Src/chassis_heading.c` | 编码器计程、IMU 纠偏/转向、四轮速度生成 | 距离比例增益、航向增益、加减速、底盘容差 |
| `Core/Src/chassis_motion.c` | 底盘公共入口、模式分发、原有位置控制 | `chassis_config` 中的距离校准、轮径、地址/方向 |
| `Core/Src/chassis_debug.c` | KEY0、Watch 状态镜像、保留的单项测试 | 路线模式已关闭单项测试，不在这里改路线速度/距离 |
| `Core/Src/can.c` | CAN 收发、解析各电机反馈 | 排查某个电机应答、反馈更新 |
| `can_zdt/Emm_V5.c` | 张大头指令封包 | 一般无需改协议指令 |
| `imu/imu.c` | UART5 IMU 帧解析、在线判断、角度快照 | 确认角度帧率、方向与数据新鲜度 |
| `tools/route_client.py` | 树莓派协议客户端 | 与雷达、视觉、工作流程对接 |
| `tools/route_demo.json` | 路线示例 | 当前起点、航点、每点速度、工作编号 |

`route.c` 内 `static` 函数是模块内部实现。业务代码使用头文件公开接口，不需要从其他文件调用 `execute()`、`compute()` 等内部函数。

## 2. 上电与两个主要任务

```text
main()
 ├─ HAL / 时钟 / GPIO / DMA / CAN / 各串口等初始化
 ├─ USER_CAN1_Filter_Init() → HAL_CAN_Start() → 开 CAN 接收通知
 ├─ Chassis_DebugInit() → Chassis_Init()，初始化并向电机发送停止
 ├─ IMU_Init()，启动 UART5 单字节中断接收
 ├─ allow_manual_tests=0，external_owner=1
 ├─ Route_Init()，路线状态 EMPTY → RouteKeys_Init()，初始化两个按键
 ├─ RouteUART_Init(&huart4)，启动 UART4 循环 DMA + 空闲接收
 └─ MX_FREERTOS_Init() → osKernelStart()

RobotTask 循环（osDelay(1)）
 ├─ RouteKeys_Task()：按键选区通知；运行中 KEY0 停车；任意键按住禁止 START
 ├─ Chassis_DebugTask()：更新 Watch 镜像，路线模式不再重复处理按键
 └─ Route_Task()：解析串口、执行路线、推进当前底盘动作

CommTask 循环（osDelay(2)）
 ├─ IMU_Task()：解析 UART5 缓存，更新 imu_data
 └─ RouteUART_Task()：发送 UART4 回复，出错后重启接收
```

CommTask 优先级高于 RobotTask，保证底盘工作期间仍能更新 IMU 和收发通信。保持此分工，不要在其他任务同时启动底盘动作。

`osDelay(1)` 是任务让出 CPU 的延时，不是四轮控制周期。底盘设计最小周期 `PERIOD_MS=20`，每轮还需要完成四个电机的反馈查询和应答，实际周期看 `chassis_heading.cycle_ms`。

## 3. 树莓派报文如何触发动作

```text
树莓派 UART → UART4 RX DMA / IDLE / HT / TC
 → HAL_UARTEx_RxEventCallback()
 → RouteUART_RxEventCallback()
 → Route_ReceiveBytes()：只把新字节放入队列

RobotTask → Route_Task()
 → parse_byte()：找帧头、检查长度、CRC
 → dispatch()：按消息类型分发
     UPLOAD    → load_route()       → 全部航点合格后保存，READY
     START     → Route_Start()      → 校验状态/IMU，建立地图航向关系，RUNNING
     STOP      → Route_Stop()       → 停车，整条路线取消
     WORK_DONE → Route_CompleteWork() → 成功推进下一点；失败 FAULT
     VISION    → vision_message()   → 更新 route_vision，调用 Route_OnVision()
     STATUS    → 回复当前状态
     AREA_ACK  → 确认当前选区请求，停止定时重发
     UPLOAD_FOR_AREA → 匹配本次选区请求号后校验并保存路线
 → execute()：按当前状态推进路线

空闲时按键 → Route_RequestStartArea() → START_AREA(0x82)
 → 树莓派 wait_start_area() 确认 → 扫描规划 → 带本请求号上传

ACK / EVENT → report() → RouteUART_Send()：加入发送队列
 → CommTask 的 RouteUART_Task()
 → HAL_UART_Transmit_IT() → HAL_UART_TxCpltCallback()
```

串口中断只搬运字节和记录错误，控制动作在 RobotTask 执行。上传成功不会自动运动，必须再收到 START。

START 时用上传的初始地图航向和此时 IMU 原始角建立对应关系。大致为：

```text
地图航向 = 初始地图航向 + IMU原始角变化 × imu_yaw_sign
```

角度会处理 ±180° 环绕。请提供真实的起点地图位姿；调用 `IMU_ZeroYaw()` 不会自动把地图起点设成 0°。

## 4. 一个航点具体怎样执行

`Route_Task()` 每次调用 `execute()`，按以下顺序处理：

1. WAIT_WORK时等待真实工作完成，检查可选超时，不启动下一批。
2. RUNNING时检查IMU，更新地图航向；有底盘动作时轮询 `Chassis_Task()`。
3. 连续批次运行时，根据本批起点和累计编码器距离持续更新地图位置估计。普通点通过后发ARRIVED/POINT_DONE并推进点号，同一批不停车、不重新取编码器起点。
4. 批次DONE后重新核对当前点的地图位置/航向；工作点进入WAIT_WORK并发送WORK_REQUIRED。
5. 准备新动作时，如航向误差超过容差，先 `Chassis_TurnRelative()` 原地转向。
6. 航向合格且未到目标时，`start_path()`收集同一目标航向下至首个工作点的折线，调用 `Chassis_PathBuild/Append/Start()`，设 `phase=5`。
7. 路径控制器提前为拐角/限速/末点减速，普通点连续衔接，末点停稳250 ms。工作点不被合并跨过。
8. 连续模式关闭，或批次超出范围/时限等条件时，保留原来的较大分量优先分轴移动回退；此时phase为2/3，仍会逐动作停车。

地图到车体的换算如下，其中 θ 是当前地图航向：

```text
dx = 目标地图X - 当前估计地图X
dy = 目标地图Y - 当前估计地图Y
前进分量 =  cosθ × dx + sinθ × dy
左移分量 = -sinθ × dx + cosθ × dy
```

例：起点(0,0,0°)，目标(500,300,90°)。先原地左转约90°，连续模式将位移换为车体前进+300 mm、左移−500 mm，沿两地图坐标间直线移动；需要绕障时，必须显式给出中间航点。关闭连续模式后才按较大的右移500 mm分量、再前进300 mm执行。

拐角连续通过会有小幅圆滑过渡，车体航向仍由IMU保持。雷达规划应按新的折线及实车扫掠误差留出空间。先平移后旋转可用保持原航向的移动点，再接同坐标的新航向点。

### 启动函数不是阻塞运动函数

```c
Chassis_TurnRelative(90.0f, 20);
```

返回 `CHASSIS_OK` 表示接受动作，实际运动靠之后反复执行 `Chassis_Task()` 推进。上面的 90°是**相对当前方向**；路线文件中的 `yaw_deg=90` 是**地图绝对航向**，两者由 `execute()` 换算。

当前路线程序已负责启动和轮询，不要在 RobotTask 的每次循环另加单独的 `Chassis_TurnRelative()`、`Chassis_MoveForwardHold()`。那样会与路线争用底盘。单独底盘测试应在明确切换到独立测试模式后进行。

## 5. 底盘如何计算速度并发送给电机

```text
execute()
 → start_path() → Chassis_PathBuild/Append/Start()（默认平移）
 或 Chassis_MoveForwardHold() / Chassis_MoveLeftHold()（回退）/ Chassis_TurnRelative()
 → start_move()：快照配置、记录目标、清除本段进度，设置 RUNNING

之后反复：Route_Task() → execute() → Chassis_Task()
 → 根据 control_mode=1 分发到 Chassis_HeadingTask()
```

`Chassis_HeadingTask()` 每个批次的准备阶段按四轮依次执行（普通点不重复准备）：停止 FE → 使能 F3 → 读状态 3A → 读位置 36 并记录本段编码器起点。

运行阶段每轮执行：

```text
读四轮位置 36 → 读四轮速度 35 → 读四轮状态 3A
 → compute()
     编码器增量 → forward_mm / left_mm
     连续模式：Chassis_PathCompute() → forward_rpm / left_rpm
     旧单轴模式：目标距离 - 已走距离 → translation_rpm
     IMU航向误差 / 剩余转角 → correction_rpm
     加减速限制 → 麦轮四轮速度混合 → 整体限幅
 → 分别下发四轮 F6 速度命令（同步标志开启）并检查应答
 → Emm_V5_Synchronous_motion(0)，广播同步执行
```

CAN 接收中断 `HAL_CAN_RxFifo0MsgPendingCallback()` 更新 `motor_feedback[地址]`；下一次底盘任务依据反馈计数判断是否收到新应答。

接近目标时速度逐渐降低。距离/角度达到容差、控制输出足够小、反馈轮速停止，并持续满足 `SETTLE_MS=250` 后，底盘子动作进入 DONE。连续模式在运行中持续更新位置；只有批次末点使用该停稳判定，普通点直接切换下一目标。

这里有两级闭环：电机内部 EMM FOC 闭环；主控用轮反馈控制行驶距离、用 IMU 控制车体航向。当前路线使用 **F6 速度模式 + 主控距离/角度闭环**，没有调用原有 FD 位置指令。

## 6. 工作点与视觉怎样接

`Route_OnWork(route_id, point_index, work_id)` 每个工作点调用一次，默认空实现；这是启动抓取/放置状态机的入口。回调要快速返回，真实动作完成后再由 RobotTask 调用：

```c
Route_CompleteWork(route_id, point_index, 1); /* 真实完成 */
/* 失败时 success=0，路线进入 FAULT。 */
```

或者树莓派收到 WORK_REQUIRED / STATUS=WAIT_WORK 后启动工作流程，真正完成再发送 WORK_DONE。两端约定一个完成通知责任方。不要仅仅“收到工作指令”就立即通知完成。

机构动作过程需要你后续新增非阻塞任务轮询，可在 RobotTask 主循环每次调用一次自己的 `Work_Task()`。当前仓库尚未实现该机构状态机。

VISION 报文只在路线号、点索引、工作号匹配当前 WAIT_WORK 时接收。`Route_OnVision()` 当前只提供扩展接口，**未实现视觉自动调整底盘或机械臂**。等待态不会调用底盘运动轮询，不要直接在此回调启动底盘动作；视觉底盘微调需要扩展工作阶段状态机。

## 7. 联调先后顺序

| 顺序 | 测试 | 验收重点 |
|---:|---|---|
| 1 | 只发 STATUS、UPLOAD，暂不 START | 收到 ACK，EMPTY→READY，电机不运动，CRC/溢出计数不增长 |
| 2 | 确认地图原点、轴、单位和起点；测试正负 90° | +X/+Y、逆时针正角与雷达一致，机器人中心与雷达坐标外参明确 |
| 3 | 固定航向前后各走约 1000 mm | 测量实际距离，校准前进比例；每次使用实际起点 |
| 4 | 固定航向左右各走约 1000 mm | 单独校准横移比例，观察是否明显打滑 |
| 5 | 原地 ±90°、180°，再做带角度的绝对航点 | 检查航向误差、角度环绕、地图/车体换算 |
| 6 | 一个通行点 + 一个工作点 | 到工作点后停住；正确完成通知后才继续；错误点号被拒绝 |
| 7 | 同一串口穿插 VISION、STATUS | 数据类型不混淆，当前工作点测量更新 |
| 8 | 联调完整路线及抓放，逐步提高速度 | 实测重复到点误差、任务耗时、打滑、通信错误、任务栈余量 |

不要同时改距离校准、速度、航向增益和容差。先固定速度和载荷，每次只改一类，记录实测值；之后换比赛载荷再复测。

## 8. 第一批要调整的参数

### A. 树莓派路线参数：每条路线/每个航点都要正确

| 参数 | 当前示例 | 如何设置 |
|---|---:|---|
| `initial_pose.x_mm / y_mm / yaw_deg` | 0/0/0 | 必须是启动时真实地图位姿，不能每次盲填 0 |
| 航点 `x_mm / y_mm / yaw_deg` | 示例航点 | 绝对地图位置和最终航向；mm、度，ROS 单位需转换 |
| `move_rpm` | 40 | 平移速度上限；先用 30～40，稳定后提高。机械打滑时先降低 |
| `turn_rpm` | 20 | 原地转向轮速上限；先保留 20。不是“每秒多少度” |
| `work_id` | 0/1/2 | 0 是通行点，非 0 是工作点；与抓放动作表统一 |
| `route_id` | 1 | 同次上电新路线必须递增；停止/完成后重跑应换 id 和真实起点 |

76 mm 直驱轮理论上 40 RPM 约为 159 mm/s；横移、加减速及轮速混合会影响实际车速。先确认轨迹空间符合分轴移动方式。

JSON 的 `yaw_deg=90` 由客户端转换成线上的 `yaw_cdeg=9000`，不要在 JSON 中直接填 9000。

### B. 距离比例：`chassis_config`，在 chassis_motion.c

| 参数 | 当前值 | 调整依据 |
|---|---:|---|
| `wheel_diameter_mm` | 76 | 实际轮径；确定后固定，别同时用它和比例系数反复补同一误差 |
| `motor_turns_per_wheel_turn` | 1 | 直驱保持 1，有减速器才按真实减速比修改 |
| `forward_correction` | 1.0 | 前后走实际距离与反馈计程的比例校准 |
| `left_correction` | 1.0 | 左右横移独立校准，麦轮横移通常要单独测 |

本代码将编码器距离除以 correction。因此车实际走得偏短时，需增大对应 correction，让同一目标需要更多轮子转动。

```text
新 correction = 旧 correction × 本段反馈计程 / 实测地面位移
```

例：目标 1000 mm，停下后 `chassis_heading.traveled_mm=997`，卷尺实测 950 mm，原系数 1.0，则新系数约为 `997/950=1.0495`。粗调也可用目标距离代替反馈计程，但会带入停止容差。

用较长直线、多次正反向测试取稳定结果。随机打滑、地面变化和机械间隙不能靠一个固定系数消除。若后续用转弯+多段路线测系数，不能直接拿最后一段 `traveled_mm` 当整条路线距离。

地址 `{1,2,4,3}`、方向 `{+1,-1,+1,-1}` 已验证，不作为速度或精度调参项。`microsteps` 要符合电机菜单，但当前 F6 反馈计程用 65536/rev，不通过修改细分来校准转角。

### C. 运动响应：`chassis_heading_config`，在 chassis_heading.c

| 参数 | 当前值 | 作用 / 有依据时怎样调 |
|---|---:|---|
| `distance_kp` | 0.20 | 每毫米剩余误差生成多少 RPM；末段过慢可小幅增大，过冲/来回找点则减小并检查加减速 |
| `yaw_kp` | 1.5 | 每度航向误差生成多少纠偏 RPM；回正慢可小幅增大，左右摆动则减小，先排除方向和反馈延迟 |
| `yaw_kd` | 0 | 航向角速度阻尼；初期保持 0，确认噪声/周期后再考虑引入 |
| `max_correction_rpm` | 12 | 平移时的纠偏速度上限；长期顶到上限且回正慢，再考虑增加，轮速仍受航点上限限制 |
| `ramp_rpm_per_s` | 120 | 速度变化斜率；起停冲击/打滑可降低，例如先试 60，动作太缓慢再提高 |
| `distance_tolerance_mm` | 3 | 底盘单段距离允许误差；更小可能增加末段调整时间，不代表地面位置更准 |
| `yaw_tolerance_deg` | 1 | 底盘单段角度允许误差；低于实际角度噪声可能一直无法完成 |
| `max_yaw_error_deg` | 20 | 故障保护阈值；用于平移偏航或转向反向/超调保护，不是最大允许转角 |
| `imu_yaw_sign` | -1 | 已按实车确认的方向，保持 -1，不用来调转角大小 |

增益通常每次只改约 10%～20%，同样条件重复测试。纯转向时的速度限幅来自 `turn_rpm`，不是 `max_correction_rpm`。平移接近最高轮速时纠偏也会压缩可用平移速度。

### D. 到点判定：`route_config`，在 route.c

| 参数 | 当前值 | 约束与用途 |
|---|---:|---|
| `position_tolerance_mm` | 6 | 地图估计位置到目标的圆形半径；必须 ≥ `1.5 × distance_tolerance_mm`，且 ≤100 |
| `heading_tolerance_deg` | 1.5 | 路线航向容差；必须 ≥ `yaw_tolerance_deg`，且 ≤5 |
| `work_timeout_ms` | 0 | 0 表示无限等真实工作完成；确认抓放最长耗时后可设置超时，最大 3600000 |

先保留 6 mm / 1.5°，测量真实误差再决定是否缩小。地面打滑造成 30 mm 偏差时，缩小此处 6 mm 没有位置反馈来源去识别它。

`route_config` 与 `route_continuous_enabled` 在 START 时快照；连续通过/拐角参数在每批启动时快照；`chassis_config / chassis_heading_config` 在每个底盘子动作启动时快照。Watch 中改配置不会立即改变正在执行的子动作。建议停车、统一改好参数，再用新 id 和真实起点测试；最终把确认参数写回初始化值后编译。

### E. 通信和保护：先观察，出现证据再改

| 项目 | 当前值 | 调试判断 |
|---|---:|---|
| UART4 | 115200，8N1 | 双方一致即可，不要用改波特率掩盖 CRC/协议错误 |
| IMU 角度帧新鲜度 | 200 ms | HWT101 必须发送 0x53 角度帧；建议先配置稳定 50 Hz（若模块支持），10 Hz 是当前要求的最低频率 |
| CAN 单次应答等待 `RESPONSE_MS` | 60 ms | 故障 3/5 先查具体电机应答、线路和任务延迟，不先加大等待 |
| 底盘准备超时 `PREPARE_MS` | 2000 ms | 停止/使能/状态/初始位置准备是否卡住 |
| 稳定判定 `SETTLE_MS` | 250 ms | 工作点/批次末点完成前稳定等待；普通途经点不执行此等待 |
| 堵转判定 `STALL_MS` | 1000 ms | 有足够速度命令却无实际转动时停机，先查电机使能/负载 |
| 转向超时 | 60000 ms | 不是转角校准系数；长时间转不到时先看方向、角度变化、速度 |

原有位置模式的 `POSITION_TOLERANCE`、`chassis_debug.distance_mm / rpm / turn_deg` 不决定当前路线运行参数；路线距离/转角来自航点，速度来自航点 move_rpm/turn_rpm。

## 9. Watch 最实用的观察项

| 观察组 | Watch 变量 | 正常意义 |
|---|---|---|
| 总路线 | `route_status.state / point_index / point_count / work_id` | 状态、当前点、总点数、等待哪项工作；点索引从 0 开始 |
| 当前动作 | `route_status.phase` | 0 规划、1 转向、2 前后、3 左右、4 等工作、5 连续路径 |
| 地图目标 | `route_status.target_x_mm / target_y_mm / target_yaw_deg` | 当前点应去哪里 |
| 地图估计 | `route_status.estimated_x_mm / estimated_y_mm / map_yaw_deg` | X/Y 在子动作完成后更新；RUNNING 时航向跟随 IMU |
| 单段距离 | `chassis_heading.target_mm / traveled_mm / remaining_mm` | 仅当前底盘子动作，不能视作整条路线距离 |
| 单段转角 | `chassis_heading.target_turn_deg / turned_deg / remaining_turn_deg` | 相对转角目标、实际累计进度、剩余量 |
| 航向纠偏 | `chassis_heading.yaw_error_deg / correction_rpm` | 偏航大小与控制输出；纠偏方向要让误差减小 |
| 执行轮速 | `chassis_heading.wheel_rpm[0..3]` | 顺序左前/右前/左后/右后，含电机安装方向符号 |
| 实际反馈 | `motor_feedback[1..4].speed_rpm / position / flags_count` | 轮速/编码器/状态是否持续有新反馈；数组下标为电机地址 |
| 控制周期 | `chassis_heading.cycle_ms / cycles` | 实际控制更新时间和次数，不能用任务延时推断 |
| IMU | `imu_data.online / angle_count / angle_tick / angle_deg[2]` | 在线、帧数和时间持续更新；此 yaw 是传感器原始角 |
| 故障 | `route_status.fault / chassis_fault / fault_address` 和 `chassis_motion.fault` | 先分清路线故障与底盘故障；地址定位到哪台电机 |
| 串口 | `route_uart_debug.rx_events / uart_errors / tx_errors` | 有无 DMA 数据、错误与发射失败 |
| 选区 | `route_start_area.area / request_id / acknowledged / pending` | 哪个区域、哪次请求、树莓派是否确认、是否还在等路线 |
| 协议 | `route_status.rx_bytes / frames / crc_errors / rx_overflows / tx_drops` | 字节和有效帧增长，错误/溢出不应持续增加 |
| 工作视觉 | `route_vision.count / work_id / dx_mm / dy_mm / confidence` | 收到当前工作点视觉测量；不等同于已经应用纠偏 |

不要在调试暂停时判断电机故障和实时刷新是否正常。暂停会让主控控制停止，电机与传感器仍可能继续运行；需要看运行时变量时优先用 Live Watch/串口 STATUS，关键断点放在尚未启动运动或已经停车的阶段。

### 状态值不要混用

`route_status.state`：0 EMPTY、1 READY、2 RUNNING、3 WAIT_WORK、4 DONE、5 STOPPED、6 FAULT。

`chassis_motion.state`：0 IDLE、1 RUNNING、2 DONE、3 STOPPED、4 FAULT。

所以路线 state=3 表示正常等待工作，底盘 state=3 则是动作被停止。

底盘 fault：1 配置、2 CAN发送、3 应答、4 电机状态、5 反馈、6 超时、7 IMU、8 航向方向/偏差、9 堵转。路线 fault 的另一套含义见 `README_route.md`。

## 10. 按症状找问题

| 现象 | 先看什么 | 再处理什么 |
|---|---|---|
| 上传后不运动 | 是否 READY，是否已 START，START ACK 是否成功 | READY 本来就等待 START；排查 KEY0 按住、IMU 未就绪 |
| 某工作点一直不走 | state 是否 WAIT_WORK，work_id/index 是否匹配 | 检查真实抓放流程和 WORK_DONE，默认不会自动继续 |
| 前后准、横移偏 | left_correction 与实际横移距离 | 分别校准，不先动 yaw_kp；观察滚子/地面打滑 |
| 一转向后地图运动方向错 | 初始航向、正负方向、ROS 单位/雷达外参 | 先修坐标约定，再考虑控制增益 |
| 行走左右摆动 | yaw_error、correction_rpm、cycle_ms | 先确认方向和反馈稳定，降低速度，再小幅降低 yaw_kp |
| 接近终点很慢 | remaining_mm、translation_rpm、轮速反馈 | 确认容差是否合理，再小幅调 distance_kp |
| 转角少很多并停住 | route/chassis fault、turned_deg、IMU 是否在线 | 排查故障早停和方向，不把目标 90 改成 180 补偿 |
| 地图估计到点、卷尺位置偏 | 实际起点、距离系数、打滑、雷达坐标外参 | 固定偏差校准；随机偏差需要视觉/雷达闭环校正 |
| rx_bytes 增长但 frames 不增长 | crc_errors、length_errors、partial_timeouts | 检查小端、CRC、帧长、版本、是否混入未封包视觉文本 |
| 反馈故障只指向一个地址 | fault_address、pending_address/function、该地址反馈计数 | 查该电机供电、使能、ID、CAN 接线/应答配置 |

实际全程定位精度目前来自“起点正确 + 标定后的轮计程 + IMU航向”。要让每次抓放都贴近同一真实位置，后续还需要把视觉/持续雷达定位误差接入末端校正，不能仅依赖将到点容差调小。
