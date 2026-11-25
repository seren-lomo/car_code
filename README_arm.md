# 三轴云台与视觉抓取接口

当前工程：`E:/Inte_handing/car_code1`，STM32F103RCT6。

模式开关 `ARM_VISION_TEST_MODE` 在 `Core/Inc/arm_config.h`。0 为整车，1 为静止视觉和云台联调。切换后重新编译下载，独立模式入口见 [README_arm_vision_test.md](README_arm_vision_test.md)。

开启 `ARM_TRAY_ENABLED` 后，两种模式都走 [三位转盘流程](README_arm_tray.md)，其中包括原料入盘、粗加工放置与取回、暂存放置，以及扩展报文。下文的夹爪流程和报文说明以关闭转盘为例。

电控管理三轴 CAN 位置控制、视觉调姿、抓取下降和抬升。原料区 102/105 使用 PA6 夹爪，下降到位后自动闭合。`PICKED` 表示 Z 已抬升到位。

## 接线与电机菜单

| 机构 | 数组下标 | CAN ID | 总线 |
|---|---:|---:|---|
| 上下 Z | 0 | 5 | 与底盘共用 CAN1 |
| 云台 yaw | 1 | 6 | 与底盘共用 CAN1 |
| 前后 reach | 2 | 7 | 与底盘共用 CAN1 |

七个电机 CANH 共接 H、CANL 共接 L，主控继续使用已有 TJA1050，PA12为CAN_TX、PA11为CAN_RX；共同信号地，两个物理末端各120Ω。菜单统一 `P_Serial=CAN1_MAP`、`CAN_Baud=500000`、`Checksum=0x6B`、`Response=Receive` 或 `Both`、`S_Vel_IS=Disable`、`Mstep=16`、`Ctrl Mode=CR_VFOC`、EMM固件；ID1～7不重复。

机构不再接 USART1/2/3。UART4 仍由树莓派统一收发雷达、视觉、路线和云台协议，UART5仍为IMU。七节点负载评估、完整接线和首次测试见 [README_CAN7.md](README_CAN7.md)。

## 最先标定这些参数

编辑 `Core/Inc/arm_config.h`。全部目标角度单位是 **电机轴的 0.01°**，不是机构输出角度，不是毫米，也不是地图航向。`9000` 表示电机轴90°，一圈36000。正值发送协议方向0/CW，负值方向1/CCW。本机已实测：正Z上升、正yaw向右转、正前后轴前伸。软件原点为Z最高、前后最前、臂朝向工作区。故Z和前后轴目标应为零或负值；yaw右转为正、左转为负。yaw的零位是机构姿态，与底盘地图航向不是同一数值。

| 宏 | 当前占位值 | 用途 |
|---|---:|---|
| ARM_PULSES_PER_REV | 3200 | 200步×16细分，一圈输入脉冲数 |
| ARM_Z_RPM / ARM_YAW_RPM / ARM_REACH_RPM | 10 | 三轴电机轴速度，rpm |
| ARM_ACCELERATION | 100 | Emm加速度档位0~255，非零值越大加速越快 |
| ARM_TOLERANCE_CDEG | 50 | 电机轴到位窗口0.5°；另要求转速≤1rpm并稳定100ms |
| ARM_Z_MIN/MAX_CDEG | -130500 / 0 | 对应从最高零位向下0～145mm |
| ARM_YAW_MIN/MAX_CDEG | -27000 / 27000 | 当前电机轴范围±270°，1:3传动对应机构±90° |
| ARM_REACH_MIN/MAX_CDEG | -38196 / 0 | 对应从最前零位向后约0～120mm，向内取整 |
| ARM_PICK_DROP_CDEG | 90000 | 相对最后对齐高度下降的电机轴角度，当前100mm |
| ARM_Z_DOWN_SIGN | -1 | 已实测正值上升，抓取下降使用负向 |
| ARM_CLEAR_Z_CDEG / ARM_CLEAR_REACH_CDEG | 0 / -19098 | 转yaw前先升到最高，再后退约60mm；行驶收回姿态也使用此位置 |
| ARM_YAW_MIN_Z_CDEG | -94500 | 转yaw时Z最多下降105mm；实际自动避让先升至最高 |
| ARM_TRANSPORT_YAW_CDEG | 0 | 云台收回后yaw电机角度 |

向下145mm、向后120mm是用户提供的最大行程；下降量与避让位置仍待联调，软件范围**没有被认定为机械限位或实际安全行程**。先空载确认方向和可用行程，再填写。皮带/齿轮传动比测好后由视觉把机构位移换算为电机轴角度；主控不猜齿数。若Z电机每转一圈机构下降L毫米，下降d毫米应设置 `ARM_PICK_DROP_CDEG=round(d/L*36000)`，符号放在 `ARM_Z_DOWN_SIGN`。

相机与夹爪的安装偏移也要标定。图像中心对准物料，不一定意味着夹爪竖直投影已对准物料；应把该偏移补到视觉目标角度/图像准心，再确认对齐。

## 按键启动：手动摆正即确认初始姿态

日常跑路线不需要先操作 Watch 或让树莓派单独发送 ZERO：手动把三个轴摆到最高、最前、yaw朝工作区的初始姿态，然后短按并松开 KEY1（启停区1）或 KEY0（启停区2）即可。按键表示你确认姿态已经摆好；主控在松键后依次使能并读取三轴编码器，把当前位置记录为新原点，完成后才向树莓派发送原来的 START_AREA。记录过程不发位置运动，不寻找机械零点。

每次新的选区按键都会重新记录，先前调试留下的 DEBUG_HOLD 或旧原点不需要手动清除。无需在每个工作区重复记录。三轴反馈失败时不会发送选区信号，故障可看 `arm_work_status` 和 `arm_motor_status[]`。

## 独立调试：手动记录原点

上电 `ArmWork_Init()` 只初始化机构驱动状态，不发使能或运动命令。只测云台、不想触发树莓派规划时，把三个轴手动摆回照片中的初始姿态，再选择以下一种方式记录：

1. Keil Watch 设置 `arm_work_debug.command=1`，然后把 `arm_work_debug.trigger` 加1。
2. 同一个树莓派 Client 调 `client.arm_zero(command_id)`，再 `client.wait_arm('READY', command_id=command_id, timeout=10)`。

调试命令也可运行 `python tools/route_client.py --port COMx arm-zero 1`，Linux用实际端口。命令会依次使能5、6、7号电机，读取位置/速度，确认静止后记录软件原点，不会执行运动或机械回零。`arm_work_status.state=2` 才准备完毕。此后 START 才接受路线，未记录原点会返回 `BAD_STATE`。

主控或电机断电、手动挪动、急停中断云台动作后，都要重新确认初始姿态并记录。不要在当前位置随意清零后把它当作照片初始姿态。重新记录使用新的 command_id。正常完成一个工作区后无需重新记录；所有绝对角度始终相对本次上电记录的同一原点。

## 运行顺序与调用链

```text
main → Route_Init / RouteUART_Init / ArmWork_Init
RobotTask（1ms）
  RouteKeys_Task → Chassis_DebugTask → Route_Task → ArmWork_Task
CommTask（2ms）
  IMU_Task → RouteUART_Task
CAN FIFO0中断 → ArmCAN_RxFrame → 各轴接收队列
ArmWork_Task → ArmCAN_Task → 解析反馈 / 位置命令 / 实测到位判断
```

`Route_Task` 完成连续路径行驶和原有工作区底盘转向后，置 `WAIT_WORK`，调用 `Route_OnWork()`。原料102/105、粗加工103/106、暂存104/107启用云台流程；二维码101、启停区108仍按原来的视觉作业完成逻辑，不自动动云台。

```text
底盘WAIT_WORK → 云台保持当前姿态并报告WAIT_VISION
视觉发送ALIGN绝对角度 → ALIGNING → ALIGNED
视觉重新拍照；未对齐可再次ALIGN，确认对齐后才发PICK
PICK → LOWERING（只动Z） → WAIT_GRIP（原料区已开始闭爪）
GRIP_DONE 请求抬升 → LIFTING（只动Z） → PICKED
视觉决定下一件：再次ALIGN…；全部完成：FINISH
FINISH → PARKING → READY_LEAVE → 树莓派发送原0x13 WORK_DONE
底盘开始下一段，云台回READY
```

原料区夹爪 PWM 使用 TIM3_CH1（PA6），周期 20 ms，张开比较值 50，闭合比较值 170。上电、待机和视觉对齐时张开。闭爪后至少等待 500 ms 才执行抬升，并保持闭合直至收到下一条有效 ALIGN、FINISH，或工作停止、故障。收到 ALIGN 或 FINISH 会原地释放物料，再执行对应的姿态动作。粗加工区和暂存区由外部确认抓取后发送 GRIP_DONE。

大幅调姿和收回使用串行动作：避让Z → 收回前后轴 → yaw → 前后到目标 → Z到目标。每一步等实际位置和速度满足条件才继续。下降与抬升保持yaw/前后不变。暂不支持自动放到车上料盘；携带物料时收回是否会碰撞、夹爪是否已正确处理，要由后续转运方案确认。

`Route_WorkMayComplete()` 拦截过早的成功 `WORK_DONE`：受管工作点只有 `READY_LEAVE` 才放行，否则返回 `BUSY=5`。失败 `WORK_DONE(success=0)` 随时可中止作业。新流程中 Pi 原来的“到点等2秒→完成”必须停用。

KEY0或串口STOP取消正在执行的云台动作，对三个轴发停止命令，保持使能以托住Z，不自动续跑；云台原点失效。电机故障会停止三轴并通过路线 `fault=3(WORK_FAILED)` 上报，具体轴故障看独立云台状态和 `arm_motor_status[]`。此时不能只依据 `route_status.chassis_fault` 诊断云台。未接机械限位，没有自动回零。

## 树莓派调用示例

复用 `tools/route_client.py` 中同一个 Client，新增封装在 `tools/arm_sequence.py`。下方示例中的外部夹爪步骤用于粗加工区和暂存区。原料区 102/105 由电控在 WAIT_GRIP 入口闭爪，继续使用 GRIP_DONE 请求抬升。

```python
from arm_sequence import ArmStation, MotorPose

def external_gripper_action():
    raise NotImplementedError('请接入真实夹爪动作和完成确认')

# 通过选区按键启动时，主控已自动记录原点，不再发arm_zero。
# 仅绕过按键独立调试时，手动摆好后调用：
# client.arm_zero(1)
# client.wait_arm('READY', command_id=1, timeout=10)

# 路线运行到state=WAIT_WORK且work_id=102..107，传入本次到点status：
station = ArmStation(client, status)
station.ready()   # 等主控进入WAIT_VISION

try:
    # 视觉根据实际物料决定循环次数、顺序，以下只展示一件：
    station.align(MotorPose(z_deg=0, yaw_deg=5, reach_deg=20))
    # 重新采图；未对齐就继续发修正后的绝对姿态：
    # station.align(MotorPose(z_deg=0, yaw_deg=4.2, reach_deg=18.6))
    # 此处由真实视觉确认中心及夹爪投影已对齐，然后：
    station.lower_after_visual_confirmation()
    # 这是外部夹爪工作区的调用示例。此处需确认外部夹爪动作完成，
    # 实机调试应先停在WAIT_GRIP，不能用固定延时冒充夹取成功。
    if external_gripper_action() is not True:
        raise RuntimeError('夹爪尚未确认完成')
    station.lift_after_external_grip_completed()
    # 多件可继续align→确认→lower→真实夹爪处理→lift。
    # 如涉及料盘存取，需先完成后续转运实现，不能认为PICKED已放到料盘。
    station.park()
    client.work_done(*station.context[:2], success=True)
except BaseException:
    client.stop()  # 通信完全断开时，串口STOP也可能无法送达
    raise
```

`align()` 返回只表示电机到位；仍须重拍图像判断是否对齐。`PICK` 是视觉给出的最终确认，不会由主控自动判断图像。云台姿态采用电机角度，与底盘地图航向分开。Pi重连先查询ARM_STATUS和STATUS核对真实阶段，不能重开整个抓取循环；超时是动作结果未知，应查状态，不能换新动作编号盲目再抓一次。

## 二进制协议（与路线共用UART4）

外框仍是 `A5 5A 01 type seq:u16 payload_len:u16 payload CRC16:u16`，多字节小端；CRC16/CCITT-FALSE从版本到payload末尾，初始FFFF、多项式1021。外框seq只用于ACK匹配；command_id负责动作去重。

### 0x31 ARM_COMMAND，payload固定32字节

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | route_id，ZERO时0 |
| 4 | u16 | point_index，零起始；ZERO时0 |
| 6 | u16 | work_id，ZERO时0 |
| 8 | u32 | command_id，必须>0，同一工作点单调增加 |
| 12 | u8 | opcode：1记录原点、2ALIGN、3PICK、4GRIP_DONE、5FINISH、6TEST_BEGIN（仅独立模式） |
| 13 | u8 | flags=0 |
| 14 | u16 | reserved=0 |
| 16 | i32 | Z电机绝对角度cdeg，仅ALIGN使用 |
| 20 | i32 | yaw电机绝对角度cdeg，仅ALIGN使用 |
| 24 | i32 | 前后电机绝对角度cdeg，仅ALIGN使用 |
| 28 | u32 | reference_id，PICK/GRIP_DONE填最近ALIGNED的command_id，其余0 |

除ALIGN外三个角度字段必须全0。PICK通过宏计算下降量。重复最后一条已接受的相同command_id及完整内容，只ACK不重复动作；同ID不同内容或回放更早的ID返回CONFLICT。换工作点后旧路线/点号/工作ID不再有效。ACK沿用0x80原36字节route状态，request=0x31、code=0仅表示接受，必须等云台状态表示完成。

### 0x32 ARM_STATUS，payload为空

主控先返回0x83云台状态，再返回0x80 ACK（request=0x32）。`client.arm_status()` 已封装。不应把0x83用旧36字节route解码器解析。

### 0x83 ARM_EVENT/STATUS，payload固定52字节

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 / 4 / 6 | u32 / u16 / u16 | route_id / point_index / work_id |
| 8 / 12 | u32 / u32 | 最近接受command_id / 最近完成对齐aligned_command_id |
| 16 / 17 | u8 / u8 | 云台state / 云台fault |
| 18 / 19 | u8 / u8 | active_axis（0上下/1yaw/2前后/255无） / 最近opcode |
| 20 / 22 | u16 / u16 | flags / reserved=0 |
| 24 / 28 / 32 | i32 | 当前Z/yaw/前后电机角度cdeg |
| 36 / 40 / 44 | i32 | 本动作最终Z/yaw/前后目标cdeg |
| 48 | u32 | 主控tick毫秒 |

flags：bit0三轴原点均有效；bit1处于READY或READY_LEAVE；bit2需要GRIP_DONE抬升指令；bit3为云台独立测试固件。停止或故障后位置可能是失效前最后反馈，应结合bit0和state，不能把它当作可靠坐标。无运动时不持续轮询编码器，手动移动和电机单独重上电后必须重设原点。状态改变主动上报，Pi也可查询补回丢失事件。

### 云台state（与route_status.state是两套枚举）

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | NEED_ORIGIN | 等待手动初始姿态记录 |
| 1 | ZEROING | 依次使能并读取软件原点 |
| 2 | READY | 可启动路线 |
| 3 | PREPARING | 旧版状态值，保留编号 |
| 4 | WAIT_VISION | 等待视觉发送绝对姿态 |
| 5 | ALIGNING | 按视觉目标调姿 |
| 6 | ALIGNED | 电机调姿到位，等待视觉确认或再次修正 |
| 7 | LOWERING | Z下降 |
| 8 | WAIT_GRIP | 等 GRIP_DONE，原料区已开始闭爪 |
| 9 | LIFTING | Z抬回本次对齐高度 |
| 10 | PICKED | Z 已抬回，原料区仍夹持 |
| 11 | PARKING | 收回云台 |
| 12 | READY_LEAVE | 已收回，允许WORK_DONE放行 |
| 13 | STOPPED | 已取消，需要重新记录原点 |
| 14 | FAULT | 云台故障 |
| 15 | DEBUG_MOVING | Watch单轴测试中 |
| 16 | DEBUG_HOLD | 测试到位，等待继续测试/回收；禁止启动路线 |

云台fault：0正常、1电机错误、2目标/配置错误、3工作点上下文改变、4取消。`arm_motor_status[i].fault`：0正常、1配置、2发送、3电机拒绝/回复错误、4超时、5电机状态异常、6CAN错误（预留）、7接收溢出。

## Watch与验证记录

看 `arm_work_status`（总流程）、`arm_motor_status[0..2]`（地址/原点/实际位置/目标/反馈计数/故障）、`arm_motor_config[0..2]`（实际配置）。ZERO后应三轴homed=1；ALIGN时当前轴state=BUSY；ALIGNED后再拍照；PICK后应停state=8等确认，不能自己抬升。

代码使用CAN反馈状态机，共用发送函数逐帧等待发送完成，FreeRTOS沿用原RobotTask和CommTask，未新增任务/增加栈配置。发送运动命令后不重试运动；丢回复转故障，停止命令可有限重试。电机协议固定6B校验、不带事务编号，不能消除所有丢包或旧同功能回复歧义。

已用实际C代码做ARM CPU仿真覆盖三轴驱动和工作流程，并完成Keil编译；没有烧录、实机动作或视觉标定验证。共享CAN集成测试另见 `tests/can_shared/run_tests.py`。测试入口：`tests/arm_can/run_tests.py`、`tests/arm_work/run_tests.py`、`tests/arm_work/test_client.py`。ARM仿真依赖放在项目tmp/arm_test_runtime；测试时无需连接硬件。


## 单轴方向与行程测试

已新增Watch command=2单轴绝对运动、command=3三轴回初始0。详见 [DEBUG_GUIDE_arm.md](DEBUG_GUIDE_arm.md)。上下40mm/电机圈、前后113.1mm/电机圈、yaw减速3:1；目标字段仍为电机轴0.01°。测试状态15/16是新增枚举，Pi状态名称同步加入tools/route_client.py。

## 本机零位与视觉换算（2026-11-19实测）

手动摆好：Z在最高、前后在最前，yaw让臂朝工作区、垂直于工作区边缘。command=1仅记录这一姿态，不会自动顶到机械挡块。每次上电应采用同一机械参考；手动摆放误差不会被软件零点消除。

若视觉用下降量d(mm)、后退量r(mm)、从臂零位向右转角a(机构度)，全部相对同一次初始姿态，则发送电机cdeg：Z=-round(d*900)，前后=-round(r*36000/113.1)，yaw=round(a*300)。例如下降10mm、后退10mm、向右转5°，电机目标依次为 -9000、1500、约-3183（协议顺序Z/yaw/reach）。`tools/arm_sequence.py` 的MotorPose和client.arm_align使用的是电机度，需再把这些cdeg除以100。

重新对齐时仍发送相对原点的绝对目标，不能把每帧修正量直接当绝对目标。底盘车头转向与机构yaw零位分别管理，机构yaw=0代表朝向当前车身定义的工作侧。

抓取从实际对齐Z再下降100mm，抬升回本次对齐高度。例如对齐Z=-4500（下降5mm），PICK目标约-94500（总下降105mm）。实际量受电机位置量化影响。正Z/前后目标现在返回BAD_VALUE；负向限幅已按下降145mm、后退120mm设置；这些是目标限幅，不是硬件限位。

当前CLEAR_Z=0、CLEAR_REACH=-19098：先抬至最高，再退约60mm，随后转yaw。FINISH结束保持最高、后退60mm、yaw=0，允许离站；到下个工作区保持当前姿态并等待视觉指令。Watch command=3仍返回三轴软件零位，因此最后会前伸到最前，不能把它当作FINISH行驶收回动作。

## 行程补充

Z目标范围[-130500,0]电机cdeg，前后目标范围[-38196,0]电机cdeg；PICK追加下降100mm后的目标也会检查范围，越界拒绝动作。请不要把最大机械行程作为日常动作目标，应根据实际止挡、齿隙与原点重复误差留余量。

用户确认转 yaw 时，Z 从最高零位下降不得超过 105 mm。后退 60 mm 可以转动，后退至 120 mm 端点禁止转动。需要完整避让时先到最高、后退 60 mm。yaw 发起前检查 Z 和前后反馈，条件不符报云台 `fault=2`。

## yaw测试的避让动作

Watch command=2、axis=1 不再只动yaw：依次Z到0 → 前后到-19098 → yaw到target_cdeg，完成后state=16并保持最高/后退60mm。测试目标1500仍代表yaw机构右转5°。axis=0或2仍是单轴测试；它们可以到达总行程范围内的其他位置，之后yaw测试会先重新避让。

需要避让的非原料区 `ALIGN`、`FINISH` 和 Watch `command=3` 返回零位会经过这个位置再转 yaw。最终视觉 Z 可以下降到 145 mm 软件限位，yaw 发起前仍检查转动高度。


## ALIGN 视觉对齐

只调整 Z 或前后轴时，`ALIGN` 直接走视觉目标。需要抬高时先抬高，需要下降时放在水平动作之后。Z 低于视觉转动高度且要横移时，会先抬升到安全高度。

小幅 yaw 调整可直接执行。条件是机构转角不超过 1°（电机 `300 cdeg`），且起始 Z 和前后位置处于可转动范围。前后目标距离与 Z 目标改变量分别处理，yaw 发起前复查反馈。

原料区 `102/105` 在 Z 最高点、前后轴处于前 60 mm 可转动区间时，直接 yaw 对齐覆盖工作原点左 45° 到右 45°。单次最多转动 45°（电机 `13500 cdeg`）。如果视觉给了同一区间内更靠后的前后轴目标，机械臂先到这个目标，再转 yaw。

原料区 `102/105` 的 `ALIGN` 不会主动去 60 mm 避让位。若目标 yaw 需要避让，只有前后轴已在避让位时才会接受；其余情况回 `BAD_VALUE`，等待视觉发送可直接执行的目标。像 `300 cdeg` 这样的步幅是相对当前 yaw 的变化量，报文仍写绝对目标。抓取后入盘和 `FINISH` 收回使用固定避让位。
