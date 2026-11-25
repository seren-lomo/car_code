# 云台三轴 CAN Watch 方向与行程测试

工程：E:/Inte_handing/car_code1。七台电机共用 CAN1，设置与接线见 [README_CAN7.md](README_CAN7.md)。使用本版前需重新编译下载，再进入调试；旧 arm_uart_diag 已换为 arm_can_diag。

## CAN无回复：先独立查询

三轴原点记录按ID5→6→7执行，第一轴失败就停止其余轴。其他轴state=4、fault=0不能证明通信成功。先逐台独立查询：

```text
arm_work_debug.command = 4
arm_work_debug.axis = 0       # 0:上下ID5，1:yaw ID6，2:前后ID7
arm_work_debug.trigger = 1    # 假设当前为0；每次在当前值加1
```

保持F5运行，约1秒后看对应轴state=2、fault=0、rx_frames增加、position_tick更新。然后axis=1、2分别再触发一次。command=4只读取位置，不使能、不运动、不记录原点，允许未记录原点或故障停止后测试；底盘/路线/机构须空闲，等待此前停止重试结束再发。

ID5位置查询逻辑字节 `05 36 6B`；实际CAN扩展ID0x500、载荷 `36 6B`；回复同ID、载荷 `36 sign pos[4] 6B`。ID6/7使用0x600/0x700。CAN载荷中的地址已移入扩展ID。

| Watch | 意义 |
|---|---|
| arm_work_debug.result | 0仅为接受，5忙，4轴编号错误 |
| arm_motor_status[i].rx_frames | 已解析有效格式回复，需与state/fault结合判断 |
| arm_motor_status[i].can_errors | CAN发送失败累计次数 |
| arm_can_diag[i].tx_requests / tx_completed | 尝试/完整发送成功的逻辑命令数量 |
| arm_can_diag[i].tx_can_frames | 完整成功命令的CAN分包数量，部分失败不计入 |
| arm_can_diag[i].rx_can_frames / rx_bytes | 对应地址收到的CAN帧/载荷字节数，含后续被丢弃的格式 |
| arm_can_diag[i].tx_length / tx_raw | 最近发送的逻辑命令，含地址，数组用Hex |
| arm_can_diag[i].rx_length / rx_raw | 最近发送后收到的前16个CAN载荷字节，不含地址 |
| arm_can_diag[i].last_rx_ext_id / last_rx_dlc | 最近接收扩展ID和载荷长度 |
| arm_can_diag[i].last_hal_status | 最近发送结果：0成功、1错误、2忙、3超时 |
| arm_can_diag[i].last_can_error | 最近发送后采样的HAL CAN错误位组合 |
| arm_can_diag[i].timeout_function | 最近超时所等功能码，54=0x36，243=0xF3 |
| arm_can_diag[i].timeout_stage / timeout_waiting | 超时时驱动步骤/是否等回复；独立查询stage=1 |
| can_last_tx_status / can_tx_error_count | 整条共享总线最近发送结果/累计发送错误 |

发送成功说明CAN分包发出，不保证指定电机执行；其他节点也可提供总线ACK。tx_completed增加而rx_can_frames不变，查电机地址、CAN映射、Response、支线与波特率。rx_can_frames增加而rx_frames不增，检查DLC、帧尾6B、功能码及载荷。发送失败先查总线接线、终端和HAL错误。

计数累计至复位，timeout_*保留最近故障快照。独立查询成功不清除总流程旧故障，也不代表原点有效；修复后仍需手动摆好并用command=1记录初始姿态。

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

数组下标0=上下Z(ID5,CAN)，1=yaw(ID6,CAN)，2=前后(ID7,CAN)。数组下标与电机地址不同。程序F5保持运行，可开启Keil的Periodic Window Update；运动时不要靠暂停/断点当急停，主控暂停不等于电机停止。

测试期间不要按KEY1/让树莓派启动路线。单轴测试只动所选轴，不会自动做完整避让动作，所以先确认该轴的小行程有空间。初始姿态若在一侧行程端点，只先测试可运动的一侧。

## 2. 手动初始姿态记录

先把三轴手动摆到定义的初始姿态，释放按键，主控及电机供电、CAN通信正常。每次测试命令必须先写command和参数，最后写trigger；不要让trigger自动持续递增。

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
| arm_work_debug.command | 0停止，1记录初始原点，2单轴绝对运动，3三轴回初始0，4独立读位置 |
| arm_work_debug.axis | 0上下，1yaw，2前后，command=2/4使用 |
| arm_work_debug.target_cdeg | 电机轴绝对目标0.01°，仅command=2使用 |
| arm_work_debug.result | 0接受，4参数/目标越界，5忙或路线运行，7没原点/状态不允许 |
| arm_work_status.state | 0需原点，1记录中，2准备好，11回收中，13停止，14故障，15测试运动，16测试到位待回收 |
| arm_motor_status[i].state | 0空闲，1执行中，2到位/原点记录完成，3故障，4停止 |
| arm_motor_status[i].fault | 0正常，1配置，2发送失败，3电机拒绝/回复错误，4超时，5电机状态异常，6CAN错误（预留），7接收溢出 |
| arm_motor_status[i].position_cdeg | 实际角度，相对已记录的原点，0.01° |
| arm_motor_status[i].speed_rpm | 实际电机轴转速rpm，正负对应反馈方向 |
| arm_motor_status[i].homed | 1软件原点有效，0未记录/已失效 |
| arm_motor_status[i].rx_frames | 收到有效格式反馈的累计帧数 |
| arm_work_status.target_cdeg[i] | 本次计划最终目标角度0.01° |

`result=0`不表示已经到位，必须看状态和实际位置。只修改position_cdeg、target_counts或arm_work_status不会正确启动电机；这些是观测值，不是命令入口。现场若不动，提供debug、work_status及对应motor_status整组截图。
