# 普通航点连续通过、工作点停车（2026-11-03）

工程：`E:\Inte_handing\car_code1`，STM32F103RCT6。修改前备份在 `tmp/before_continuous_20261103/`，包括源码、测试、Keil工程和原说明。此次保留用户的 `ramp_rpm_per_s=120`、轮径76 mm、电机地址/方向和IMU符号配置。

**后续工作区朝向功能：** 现在默认对工作编号101～108执行到站转向并保持朝向行驶，详见 [README_work_heading.md](README_work_heading.md)。以下8次纯平移、全部yaw90°的结果是新增工作朝向前的基准；启用工作朝向后仍8次工作等待，另有7次转向，phase新增6。

## 本次日志结论

`区1_优化路线_完整日志` 的路线1793701103，起点(150,150,90°)，共16点；普通点8个，工作点8个。巡航100 rpm，部分进出区段20 rpm，全部目标车头航向90°。普通点的工作标记正确，无须重新标记。

原程序在每个航点先减速到零，停稳250 ms，再启动下一段的停止/使能/编码器起点准备。因此树莓派删除直线中间点后，保留下来的拐点仍会停。

本次将同一目标车头航向下的点组成连续折线，保留每个规划拐点，到工作点才结束该批运动并停稳。没有把多个拐点连成跨越障碍物的直线。

| 零起始点索引 | work_id | 工作站 |
|---|---|---|
| 1 | 101 | 二维码 |
| 3 | 102 | 原料第一次 |
| 4 | 103 | 粗加工第一次 |
| 6 | 104 | 暂存第一次 |
| 8 | 105 | 原料第二次 |
| 9 | 106 | 粗加工第二次 |
| 11 | 107 | 暂存第二次 |
| 15 | 108 | 启停区 |

工作点进入 `state=3 / phase=4`，发送 WORK_REQUIRED，保持停车，直到收到正确路线号、点索引的 `0x13 WORK_DONE`。日志中的“停2秒后模拟完成”是树莓派测试行为；主控没有新增自动放行定时器。

## 函数调用与运动方式

```text
RobotTask → Route_Task() → execute()
  ├─ 如需改变车头航向：Chassis_TurnRelative()，完成后继续
  └─ start_path()：收集当前点至第一个工作点，或目标航向变化前的点
       → Chassis_PathBuild() → Chassis_PathAppend() → Chassis_PathStart()

之后每次轮询：
Route_Task() → Chassis_Task() → Chassis_HeadingTask()
  → 读取四轮位置/速度/状态
  → compute() → Chassis_PathCompute()
     ├─ 普通点：切换下一个折线目标，保持同一批运动
     ├─ 直线：连续行驶，前瞻下一个限速/拐点的制动距离
     ├─ 拐角：提前减速，平滑改变前/左平移速度
     └─ 批次末点：减速到零
  → IMU航向纠偏 → 四轮速度混合/整体限幅
  → 四轮F6速度命令 → 广播同步执行

批次末点：位置/航向/轮速合格且稳定250 ms → DONE
  → 工作点：WAIT_WORK → WORK_DONE → 下一批
```

连续运动仍为 **F6速度模式＋主控编码器距离闭环＋IMU航向闭环**。电机自身FOC闭环也保持运行。底盘任务、CAN应答超时、堵转检查和故障取消机制沿用原实现，无需新增任务或中断。

新连续模式按相邻地图航点的直线段跟踪，支持前/左速度同时输出；斜向航点之间不再默认按较大分量分轴走。规划器应显式给出需要绕行的所有拐点。日志中的轴对齐折线保持原路径。

路线层持续更新本批次的地图位置估计；每个普通点仍发 ARRIVED(事件3)、POINT_DONE(事件8)，这些事件现在表示途经，**不表示车已停稳**。树莓派仅应在 WAIT_WORK / WORK_REQUIRED(事件4) 时启动工作流程。

## 参数在哪里改

`Core/Src/chassis_path.c`：

```c
volatile ChassisPathConfig_t chassis_path_config = {3.0f, 5.0f};
/*                                                  ↑     ↑
                                    普通点通过半径mm   拐角速度预算rpm */
```

| 参数 | 默认值 | 用法 |
|---|---|---|
| `chassis_path_config.pass_tolerance_mm` | 3 mm | 普通点切换半径，允许1～10；不是实车定位精度 |
| `chassis_path_config.corner_rpm` | 5 rpm | 拐角衔接速度预算，允许1～15；实际过点速度还受加减速和控制周期影响 |
| 每点 `move_rpm` | 本日志100或20 | 树莓派下发的该段四轮转速上限，保留原协议 |
| `chassis_heading_config.ramp_rpm_per_s` | 120 rpm/s | 原有加减速斜率，本次保留用户值 |
| `route_continuous_enabled` | 1 | `Core/Src/route.c` 初始化；START前设0可对照旧的逐点停车逻辑 |

配置请停车后修改；连续开关在START时快照，连续路径参数在每批启动时快照。加减速、限速提前量会自动参与规划；工作点无需保留额外的120 mm低速航点，也会减速停车。如需较长的20 rpm精确接近段，仍可在该段起点附近设置相应限速的普通航点。

拐角不是严格无圆角的几何折线，有限加速度会产生毫米级过渡。默认参数在理想轮子仿真中的最大折线偏差为5.63 mm，不能作为实车误差上界；打滑、轮胎和采样延迟会改变误差。本次日志的 `footprint.margin_mm=2`，不足以从此仿真推出拐角必然有净空，规划余量需按车体扫掠区域和实测误差重新核对。

若希望贴近拐点，可先适当减小通过半径、降低拐角速度，再实测车体与障碍物的最小间距。巡航建议先降到40～60 rpm验证，再恢复100 rpm。

## Watch与联调判据

| Watch | 意义/预期 |
|---|---|
| `route_status.state` | 正常经过拐点保持2；八个工作点变3；全程结束4 |
| `route_status.phase` | **5=连续折线**，4=等待工作；1=原地转向；2/3为旧分轴动作 |
| `route_status.point_index / work_id` | 当前路线点/工作编号；索引从0开始 |
| `chassis_heading.action` | **2=连续路径**；0=旧平移；1=原地转向 |
| `chassis_motion.axis` | **3=连续路径**；0/1/2为旧前后/左右/转向 |
| `chassis_path.node_index / node_count / route_index` | 本批路径节点进度及其对应路线点 |
| `chassis_path.forward_rpm / left_rpm` | 本批当前车体前/左平移速度命令，带符号 |
| `chassis_path.remaining_mm` | 到本批当前目标节点的距离 |
| `chassis_path.junction_rpm` | 当前节点的前瞻出口速度预算；工作末点为0 |
| `chassis_path.speed_limit_rpm` | 当前节点四轮限速 |
| `chassis_heading.forward_mm / left_mm` | **从本批起点**累计的编码器距离；普通点不清零 |
| `chassis_heading.target_mm / traveled_mm` | 连续模式不用这两个旧单轴量，均为0 |
| `chassis_heading.yaw_error_deg / wheel_rpm` | IMU纠偏误差和四轮真实下发命令 |
| `chassis_motion.completed_count` | 本次16点路线正常增加8次，而非每个普通点都增加 |
| `route_status.fault / chassis_motion.fault` | 正常均为0 |

原始数据类型/帧长度/CRC/选区握手/WORK_DONE格式保持不变；状态回复中的 `phase` 新增5。树莓派如果限定了phase取值，需要允许5并显示“连续行驶”。

“地图移动方向拐弯”与“车头旋转”是两件事。这份路线全部yaw90°，因此车头保持90°，以麦轮平移改变运动方向，不会在二维码区自动旋转90°。这是纯连续平移基准。后续的工作区朝向模式已在主控按work_id自动到站转向，无需修改原报文yaw；关闭该模式后，才完全依照报文目标航向。

## 适用边界与验证

本次16点路线已验证为8个连续批次、8次工作等待。改变目标车头航向时会结束当前批次并用原地转向接口执行；反向折返物理上必须经过零速度；路线结束即使不是工作点也会停车。每批最多64节点、相对起点前/左坐标各不超过4000 mm，估算超时不超过120 s；越界/过长批次或连续配置不可用时保留旧分轴执行回退，可能恢复部分途经停车。这些边界没有在本次日志中触发。

仿真测试使用真实 `route.c / chassis_path.c / chassis_heading.c`，模拟异步CAN反馈、四轮动态和IMU串口数据；日志夹具保留实际0x15/0x16/0x11帧。通过25项串口/路线回归、46项底盘回归、15项连续路径仿真（含原始16点路线、连续短段、拐角、工作点边界、航向变化、停止、IMU/反馈丢失和距离校准）。仿真时长不作为实车用时承诺。

Keil ARMCC5全量构建0错误、7项原有文件末尾换行警告；Code36308、RO544、RW596、ZI36844 bytes。RAM静态合计37440 bytes（包含原RTOS heap），低于48 KB。构建日志：`tmp/continuous_20261103_build.log`。

重新打开 `MDK-ARM/car_code1.uvprojx`，确认工程树有新增 `chassis_path.c`，再编译/烧录。已生成 `MDK-ARM/car_code1/car_code1.hex`，尚未实车烧录验证。
