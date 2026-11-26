# 三轴云台与视觉抓取接口

当前工程：`E:/Inte_handing/car_code1`，STM32F103RCT6。

本次实现三轴串口位置控制、视觉调姿、上下轴下降/抬升、等待夹爪完成确认和云台收回。夹爪 PWM、车上料盘放置、物料转运和完整比赛抓放顺序按你的要求留待后续实现。`PICKED` 只代表外部确认后 Z 已抬回，不代表主控检测到了物料。

## 接线与电机菜单

| 机构 | 数组下标 | ID | 串口 | 主控 TX → 电机 RX | 主控 RX ← 电机 TX |
|---|---:|---:|---|---|---|
| 上下 Z | 0 | 5 | USART1 | PA9 | PA10 |
| 云台 yaw | 1 | 6 | USART2 | PA2 | PA3 |
| 前后 reach | 2 | 7 | USART3 | PB10 | PB11 |

三个电机各接各的 TTL 串口，信号地与主控共地；电机动力电源独立供电。这里不经过 CAN 收发器。主控五个串口都是 115200、8N1。UART4 仍由一个树莓派进程统一收发雷达、视觉、路线和云台协议，UART5 仍为 IMU。

电机菜单：`P_Serial=UART_FUN`、`ID_Addr=5/6/7`、串口波特率115200、`Checksum=0x6B`、`Response=Receive` 或 `Both`、`S_Vel_IS=Disable`、`Mstep=16`、`Ctrl Mode=CR_VFOC`。本驱动发送 Emm 的 FD 位置命令。USART1/2/3 使用逐字节 RX 中断与 TX 中断，已有 DMA 配置保留但此驱动不启动其 RX DMA；UART4 的循环 DMA 保持原方案。

## 最先标定这些参数

编辑 `Core/Inc/arm_config.h`。全部角度单位是 **电机轴的 0.01°**，不是机构输出角度，不是毫米，也不是地图航向。`9000` 表示电机轴90°，一圈36000。正值发送协议方向0/CW，负值方向1/CCW。不能把“正值”直接当成上升/前伸/向左：这三个实际方向要你逐轴确认。

| 宏 | 当前占位值 | 用途 |
|---|---:|---|
| ARM_PULSES_PER_REV | 3200 | 200步×16细分，一圈输入脉冲数 |
| ARM_Z_RPM / ARM_YAW_RPM / ARM_REACH_RPM | 10 | 三轴电机轴速度，rpm |
| ARM_ACCELERATION | 100 | Emm加速度档位0~255，非零值越大加速越快 |
| ARM_TOLERANCE_CDEG | 50 | 电机轴到位窗口0.5°；另要求转速≤1rpm并稳定100ms |
| ARM_Z_MIN/MAX_CDEG | -72000 / 72000 | Z的电机轴范围，暂定±2圈 |
| ARM_YAW_MIN/MAX_CDEG | -9000 / 9000 | yaw电机轴范围，暂定±90° |
| ARM_REACH_MIN/MAX_CDEG | -72000 / 72000 | 前后电机轴范围，暂定±2圈 |
| ARM_PICK_DROP_CDEG | 9000 | 相对最后对齐高度下降的电机轴角度，暂定90° |
| ARM_Z_DOWN_SIGN | +1 | 下降方向，若实测相反改为-1 |
| ARM_CLEAR_Z_CDEG / ARM_CLEAR_REACH_CDEG | 0 / 0 | 转yaw前的避让高度/前后位置 |
| ARM_TRANSPORT_YAW_CDEG | 0 | 云台收回后yaw电机角度 |
| ARM_PHOTO_Z/YAW/REACH_CDEG | 0 / 0 / 0 | 每到工作区先去的拍照初始姿态 |

这些范围、下降量和避让位置都是占位值，**没有被认定为机械限位或实际安全行程**。先空载确认方向和可用行程，再填写。皮带/齿轮传动比测好后由视觉把机构位移换算为电机轴角度；主控不猜齿数。若Z电机每转一圈机构下降L毫米，下降d毫米应设置 `ARM_PICK_DROP_CDEG=round(d/L*36000)`，符号放在 `ARM_Z_DOWN_SIGN`。

相机与夹爪的安装偏移也要标定。图像中心对准物料，不一定意味着夹爪竖直投影已对准物料；应把该偏移补到视觉目标角度/图像准心，再确认对齐。

## 上电：手动摆正后记录原点

上电 `ArmWork_Init()` 只启动接收，不发使能或运动命令。把三个轴手动摆回照片中的初始姿态，再选择以下一种方式记录：

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
USART中断 → ArmUART_*Callback → 接收队列
ArmWork_Task → ArmUART_Task → 解析反馈 / 位置命令 / 实测到位判断
```

`Route_Task` 完成连续路径行驶和原有工作区底盘转向后，置 `WAIT_WORK`，调用 `Route_OnWork()`。原料102/105、粗加工103/106、暂存104/107启用云台流程；二维码101、启停区108仍按原来的视觉作业完成逻辑，不自动动云台。

```text
底盘WAIT_WORK → PREPARING（去拍照姿态） → WAIT_VISION
视觉发送ALIGN绝对角度 → ALIGNING → ALIGNED
视觉重新拍照；未对齐可再次ALIGN，确认对齐后才发PICK
PICK → LOWERING（只动Z） → WAIT_GRIP（无限等待外部夹爪完成）
真实夹爪动作完成后发GRIP_DONE → LIFTING（只动Z） → PICKED
视觉决定下一件：再次ALIGN…；全部完成：FINISH
FINISH → PARKING → READY_LEAVE → 树莓派发送原0x13 WORK_DONE
底盘开始下一段，云台回READY
```

每次调姿/收回使用串行动作：避让Z → 收回前后轴 → yaw → 前后到目标 → Z到目标。每一步等实际位置和速度满足条件才继续。下降与抬升保持yaw/前后不变。暂不支持自动放到车上料盘；携带物料时收回是否会碰撞、夹爪是否已正确处理，要由后续转运方案确认。

`Route_WorkMayComplete()` 拦截过早的成功 `WORK_DONE`：受管工作点只有 `READY_LEAVE` 才放行，否则返回 `BUSY=5`。失败 `WORK_DONE(success=0)` 随时可中止作业。新流程中 Pi 原来的“到点等2秒→完成”必须停用。

KEY0或串口STOP取消正在执行的云台动作，对三个轴发停止命令，保持使能以托住Z，不自动续跑；云台原点失效。电机故障会停止三轴并通过路线 `fault=3(WORK_FAILED)` 上报，具体轴故障看独立云台状态和 `arm_motor_status[]`。此时不能只依据 `route_status.chassis_fault` 诊断云台。未接机械限位，没有自动回零。

## 树莓派调用示例

复用 `tools/route_client.py` 中同一个 Client，新增封装在 `tools/arm_sequence.py`。以下是接口顺序示例，角度和物料数须由视觉实时计算；不是可直接完成比赛的整套抓放程序。

```python
from arm_sequence import ArmStation, MotorPose

def external_gripper_action():
    raise NotImplementedError('请接入真实夹爪动作和完成确认')

# 上电后手动摆好，只做一次，command_id需为正整数：
client.arm_zero(1)
client.wait_arm('READY', command_id=1, timeout=10)

# 路线运行到state=WAIT_WORK且work_id=102..107，传入本次到点status：
station = ArmStation(client, status)
station.ready()   # 等云台拍照姿态到位；非二维码/回家工作点

try:
    # 视觉根据实际物料决定循环次数、顺序，以下只展示一件：
    station.align(MotorPose(z_deg=0, yaw_deg=5, reach_deg=20))
    # 重新采图；未对齐就继续发修正后的绝对姿态：
    # station.align(MotorPose(z_deg=0, yaw_deg=4.2, reach_deg=18.6))
    # 此处由真实视觉确认中心及夹爪投影已对齐，然后：
    station.lower_after_visual_confirmation()
    # 此处需执行真实夹爪动作，并确认完成。当前固件未实现夹爪，
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
| 12 | u8 | opcode：1记录原点、2ALIGN、3PICK、4GRIP_DONE、5FINISH |
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

flags：bit0三轴原点均有效；bit1处于READY或READY_LEAVE；bit2外部夹爪处理（当前恒为1）。停止或故障后位置可能是失效前最后反馈，应结合bit0和state，不能把它当作可靠坐标。无运动时不持续轮询编码器，手动移动和电机单独重上电后必须重设原点。状态改变主动上报，Pi也可查询补回丢失事件。

### 云台state（与route_status.state是两套枚举）

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | NEED_ORIGIN | 等待手动初始姿态记录 |
| 1 | ZEROING | 依次使能并读取软件原点 |
| 2 | READY | 可启动路线 |
| 3 | PREPARING | 到站后去拍照姿态 |
| 4 | WAIT_VISION | 等待视觉发送绝对姿态 |
| 5 | ALIGNING | 按视觉目标调姿 |
| 6 | ALIGNED | 电机调姿到位，等待视觉确认或再次修正 |
| 7 | LOWERING | Z下降 |
| 8 | WAIT_GRIP | 等真实夹爪完成确认 |
| 9 | LIFTING | Z抬回本次对齐高度 |
| 10 | PICKED | 外部确认后Z已抬回 |
| 11 | PARKING | 收回云台 |
| 12 | READY_LEAVE | 已收回，允许WORK_DONE放行 |
| 13 | STOPPED | 已取消，需要重新记录原点 |
| 14 | FAULT | 云台故障 |

云台fault：0正常、1电机错误、2目标/配置错误、3工作点上下文改变、4取消。`arm_motor_status[i].fault`：0正常、1配置、2发送、3电机拒绝/回复错误、4超时、5电机状态异常、6UART错误、7接收溢出。

## Watch与验证记录

看 `arm_work_status`（总流程）、`arm_motor_status[0..2]`（地址/原点/实际位置/目标/反馈计数/故障）、`arm_motor_config[0..2]`（实际配置）。ZERO后应三轴homed=1；ALIGN时当前轴state=BUSY；ALIGNED后再拍照；PICK后应停state=8等确认，不能自己抬升。

代码使用串口非阻塞状态机，FreeRTOS沿用原RobotTask和CommTask，未新增任务/增加栈配置。发送运动命令后不重试运动；丢回复转故障，停止命令可有限重试。电机协议固定6B校验、不带事务编号，不能消除所有串口噪声或旧同功能回复歧义。

已用实际C代码做ARM CPU仿真覆盖三轴驱动和工作流程，并完成Keil编译；没有烧录、实机动作或视觉标定验证。测试入口：`tests/arm_uart/run_tests.py`、`tests/arm_work/run_tests.py`、`tests/arm_work/test_client.py`。ARM仿真依赖放在项目tmp/arm_test_runtime；测试时无需连接硬件。
