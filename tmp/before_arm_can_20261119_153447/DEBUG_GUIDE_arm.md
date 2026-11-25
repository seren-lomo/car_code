# 云台三轴 Watch 方向与行程测试

工程：E:/Inte_handing/car_code1。Watch测试入口在 `Core/Src/arm_work.c`，结构定义在 `Core/Inc/arm_work.h`。使用新增 command=4 和 arm_uart_diag 前，必须编译、下载本次固件，并重新进入调试。

## 串口无回复：先独立读取，不执行三轴原点记录

原点记录 command=1 按 USART1→USART2→USART3 顺序执行，第一轴失败后其他轴会停止。因此其他轴 state=4、fault=0 不是通信成功证据。交换两台电机及其ID后，仍然卡第一轴，也不能单凭这个结果认定USART1硬件损坏。

新增 command=4：只发送一次 `地址 36 6B` 读取位置，不发送使能、运动、清零指令，不要求 homed=1。查询超时也不自动发送停止指令。只允许在底盘、路线、机构空闲时执行；可在初始未记录原点或故障停止后执行。参数必须先写，trigger最后加1。等待上次停止重试结束（约1秒）再查询。

```text
arm_work_debug.command = 4
arm_work_debug.axis = 0       # 0:USART1，1:USART2，2:USART3
arm_work_debug.trigger = 1    # 示例：原来为0；每次在当前值上加1
```

保持F5运行，约1秒后查看对应下标。接着改axis=1，trigger再加1，独立检查USART2；axis=2同理。command=4没有新增串口协议，树莓派不需要更改。

| Watch | 意义 |
|---|---|
| arm_work_debug.result | 0仅为接受查询；5忙；4轴编号错误 |
| arm_motor_status[i].state/fault | 查询完成：2/0；超时：3/4 |
| arm_motor_status[i].position_tick | 成功读取位置后更新 |
| arm_motor_status[i].rx_frames | 匹配地址、功能和帧尾的回复数量 |
| arm_uart_diag[i].tx_requests | 累计尝试调用发送的次数 |
| arm_uart_diag[i].tx_completed | 累计进入HAL发送完成回调次数；不证明电机收到 |
| arm_uart_diag[i].rx_bytes | 累计接收中断收到的原始字节数，包括被协议丢弃的字节 |
| arm_uart_diag[i].last_hal_status | 最近发送调用结果：0成功，1错误，2忙，3超时 |
| arm_uart_diag[i].tx_length/tx_raw | 最近尝试发送的长度和字节，数组用Hex查看 |
| arm_uart_diag[i].rx_length/rx_raw | 最近一次发送后收到的前16字节，每次发送前清空；数组用Hex查看 |
| arm_uart_diag[i].timeout_function | 最近故障超时所等功能码，54=0x36位置查询，243=0xF3使能 |
| arm_uart_diag[i].timeout_tx_active | 最近故障超时当时是否仍未处理发送完成，1为是 |
| arm_uart_diag[i].timeout_waiting | 最近故障超时当时是否还在等回复，1为是 |
| arm_uart_diag[i].timeout_stage | 最近故障超时的驱动步骤；command=4发送位置查询后为1 |

计数累计到复位，测试前后比较增量。timeout_*只在操作故障超时更新，不会随成功查询清零。
成功查询不代表已记录初始位置，也不清除 arm_work_status 原来的故障。修复链路后仍需手动摆好，再command=1完成三轴初始位置记录。

判读：

- tx_requests增加、last_hal_status=0、tx_completed增加、rx_bytes不变：HAL发送完成，但没有原始接收字节。检查实际引脚、连接线、公共地、电机UART菜单、波特率和返回线路。发送完成不能证明外部引脚波形正确。
- tx_requests增加、last_hal_status=0、tx_completed不变，timeout_tx_active=1：检查USART中断、发送完成回调和CPU是否运行。
- rx_bytes增加但rx_frames不增加：查看rx_raw原始字节，区分回显、错误地址、错误格式或波特率问题。
- rx_frames增加且state=2、fault=0、position_tick更新：本次位置查询成功。若随后command=1仍卡F3，重点核对电机Response是否为Receive/Both。

若USART2独立查询正常，做同一电机、同一线束交叉验证：保持该电机ID不变，把其TX/RX/GND接到另一USART，临时修改对应 `arm_motor_status[i].address` 为实际ID，再command=4查询。仅查询阶段这样改，完成后恢复默认地址5/6/7及机械轴接线，不能带着交换后的机械轴映射执行抓取。
USART1是PA9(TX)→电机RX、PA10(RX)←电机TX；USART2是PA2(TX)→电机RX、PA3(RX)←电机TX。板载串口跳线保持拔下，电机通信线上不并接DAP的TX。

USART1当前配置为APB2=72MHz、115200，运行时 `huart1.Instance->BRR` 应为625(0x271)。此值是配置核对，不替代示波器测实际波特率。此前PA9/PA10回环有数据，只说明发送接收有活动，不足以证明绝对波特率或电机外部线路正确。

## 1. Watch添加

```text
arm_work_debug
arm_work_status
arm_motor_status[0]
arm_motor_status[1]
arm_motor_status[2]
arm_motor_config[0]
arm_motor_config[1]
arm_motor_config[2]
```

数组下标0=上下Z(ID5,USART1)，1=yaw(ID6,USART2)，2=前后(ID7,USART3)。数组下标与电机地址不同。程序F5保持运行，可开启Keil的Periodic Window Update；运动时不要靠暂停/断点当急停，主控暂停不等于电机停止。

测试期间不要按KEY1/让树莓派启动路线。单轴测试只动所选轴，不会自动做完整避让动作，所以先确认该轴的小行程有空间。初始姿态若在一侧行程端点，只先测试可运动的一侧。

## 2. 手动初始姿态记录

先把三轴手动摆到定义的初始姿态，释放按键，主控及电机供电、串口正常。每次测试命令必须先写command和参数，最后写trigger；不要让trigger自动持续递增。

```text
arm_work_debug.command = 1
arm_work_debug.trigger = 1   # 假设当前为0；以后每发一次命令加1
```

等 `arm_work_status.state=2(READY)`、三个 `arm_motor_status[i].homed=1`、fault=0，位置position_cdeg约为0。记录过程逐个使能并读取位置，不会自动寻找机械零点。失败看单轴fault/last_status/rx_frames。不要在测试后的任意位置重新记录原点以代替回初始位置。

## 3. 小行程逐轴测试

建议先把所测轴 `arm_motor_config[i].rpm` 改为5，单位电机轴rpm，此修改仅当前运行有效，修改后会影响该轴后续动作，复位恢复arm_config.h中的值。

例如上下轴从原点移动1mm：

```text
arm_work_debug.command = 2
arm_work_debug.axis = 0
arm_work_debug.target_cdeg = 900
arm_work_debug.trigger = 2   # 在当前trigger上加1
```

`result=0`表示接受，`arm_work_status.state=15`运动，等其变成16(DEBUG_HOLD)，且 `arm_motor_status[0].state=2(DONE)`、fault=0表示到位。记下正数究竟对应上升还是下降，再实测距离。正数=协议方向0/CW，不保证是你定义的上/前/逆时针。

每测完一个轴可仍用command=2，把该轴target_cdeg写0再trigger加1，返回该轴初始位置。即使目标0到位，仍处于DEBUG_HOLD；最后command=3才能收回三轴并解除路线启动限制。

| axis | 机构 | 首次目标target_cdeg | 理论机构变化幅度 | 放大行程示例 |
|---:|---|---:|---|---|
| 0 | 上下 | 900 | 1mm | 9000=10mm，18000=20mm |
| 1 | yaw | 900 | 3° | 9000=30°，27000=90° |
| 2 | 前后 | 318 | 约1mm | 3183约10mm，15915约50mm |

数值都是相对软件原点的电机轴绝对角度，单位0.01°。先+900再+1800表示从初始位置累计到+1800，中间只多走+900。重复+900不会每次再增加900；要反向验证可以先回0，再在机构允许的范围发负值。

以本次参数：上下40mm/电机圈，前后113.1mm/电机圈（模数2、18齿），yaw电机3圈/机构1圈。因此Watch中可额外添加下列表达式观察相对原点的位置：

```text
arm_motor_status[0].position_cdeg / 900.0
arm_motor_status[1].position_cdeg / 300.0
arm_motor_status[2].position_cdeg * 113.1 / 36000.0
```

三行分别为上下毫米、yaw机构角度、前后毫米；符号是电机坐标符号，物理正向需要实测。位置只是电机编码器反馈经传动比例换算，皮带打滑、齿隙或安装松动不会被单独检测，必须同时测实际机构位移。

## 4. 收回、停止与错误码

测试结束，保持初始姿态原点有效，发：

```text
arm_work_debug.command = 3
arm_work_debug.trigger += 1
```

程序按已配置避让顺序返回三轴电机角度0，state先11(PARKING)，完成变2(READY)，才重新允许底盘START。这个动作是回软件初始坐标，不是限位回零。单轴测试结束后直接启动路线会被拒绝。

运动时需要停止：command=0，然后trigger加1；或按KEY0。主控会发三轴停止、保持使能，并使软件原点失效；重新手动摆好并记录后才能再测试。停止不等于断电释放Z。

| 参数 | 主要取值 |
|---|---|
| arm_work_debug.command | 0停止，1记录初始原点，2单轴绝对运动，3三轴回初始0 |
| arm_work_debug.axis | 0上下，1yaw，2前后，仅command=2使用 |
| arm_work_debug.target_cdeg | 电机轴绝对目标0.01°，仅command=2使用 |
| arm_work_debug.result | 0接受，4参数/目标越界，5忙或路线运行，7没原点/状态不允许 |
| arm_work_status.state | 0需原点，1记录中，2准备好，11回收中，13停止，14故障，15测试运动，16测试到位待回收 |
| arm_motor_status[i].state | 0空闲，1执行中，2到位/原点记录完成，3故障，4停止 |
| arm_motor_status[i].fault | 0正常，1配置，2发送失败，3电机拒绝/回复错误，4超时，5电机状态异常，6UART错误，7接收溢出 |
| arm_motor_status[i].position_cdeg | 实际角度，相对已记录的原点，0.01° |
| arm_motor_status[i].speed_rpm | 实际电机轴转速rpm，正负对应反馈方向 |
| arm_motor_status[i].homed | 1软件原点有效，0未记录/已失效 |
| arm_motor_status[i].rx_frames | 收到有效格式反馈的累计帧数 |
| arm_work_status.target_cdeg[i] | 本次计划最终目标角度0.01° |

`result=0`不表示已经到位，必须看状态和实际位置。只修改position_cdeg、target_counts或arm_work_status不会正确启动电机；这些是观测值，不是命令入口。现场若不动，提供debug、work_status及对应motor_status整组截图。
