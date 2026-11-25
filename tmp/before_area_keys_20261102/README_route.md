# 树莓派整条路线下发与主控执行

本次修改的是 **`E:\Inte_handing\car_code1`（STM32F103RCT6）**。

主流程：树莓派发送整条路线 → STM32 校验并保存 → 树莓派发送 START → 逐点执行 → 工作点停车等待 → 收到真实工作完成通知后继续。没有路线或没有 START 时，上电不会自动转动。KEY0 现在用于取消路线，不再循环执行底盘测试。

函数调用链、Watch 变量、参数含义和联调顺序见 [DEBUG_GUIDE_route.md](DEBUG_GUIDE_route.md)。

## 接线、CubeMX 与任务

| 项目 | 当前配置 |
|---|---|
| UART4 TX | PC10 → 树莓派 RX |
| UART4 RX | PC11 ← 树莓派 TX |
| 电平 | 3.3 V TTL，双方 GND 相连 |
| 串口 | 115200，8 数据位，无校验，1 停止位，无流控 |
| 接收 | DMA2 Channel3，Byte/Byte、Memory Increment、Circular；保留 HT/TC 中断 |
| 中断 | UART4 全局中断优先级 6，DMA2 Channel3 优先级 5，已配置 |
| 任务 | RobotTask 每 1 ms 调度路线；CommTask 每 2 ms 处理 IMU 与串口发送/重启 |

同一台树莓派的一对 TX/RX 同时承载雷达路线和视觉报文，靠消息类型区分。树莓派上由一个通信进程/对象统一读写此 UART；雷达和视觉进程通过队列交给它，不能分别打开串口抢读。关闭 Raspberry Pi 此串口的系统登录控制台。UART5 仍专供 IMU。

FreeRTOS 沿用当前配置：RobotTask 1024 words、CommTask 768 words、总 heap 20480 bytes，无需新增任务。本次 Keil 构建 Code 32076、RO 540、RW 532、ZI 35732 bytes；RAM 静态合计 36264 bytes（包含 RTOS heap），低于此芯片 48 KB RAM。操作抓取机构后应再观察各任务栈余量。

Keil 工程中已加入 `route.c`、`route_uart.c`。如果工程一直开着，请重新打开 `MDK-ARM/car_code1.uvprojx`，让工程树读入新文件，再编译、烧录。

## 地图坐标与实际轨迹约定

- 距离全部使用 **mm**，角度使用 **度**。地图 +X 为航向 0°，+Y 为该方向的左侧，逆时针为正。航向范围 −180°～+180°。
- 上传时必须给出机器人**当前实际起点**的地图 X/Y/航向。STM32 的 IMU 软件零点不等于地图 0°；START 时会建立二者的角度对应关系。起点不能随意填零。
- ROS 常见单位为米、弧度，树莓派下发前分别乘 1000、乘 180/π。地图原点、轴方向与机器人中心必须一致。
- 单点执行顺序为：**原地转到该点的目标航向 → 将地图位移投影到车体前/左方向 → 先走绝对值较大的分量 → 再走剩余分量**。每段后按轮子反馈重新计算剩余距离，超过 4000 mm 自动拆段。
- 航点之间**不是任意方向直线或曲线跟踪**。雷达规划器必须为这种先转向、再分轴移动的扫掠区域预留空间；有障碍物时增加中间通行点。想先走再转，可用同坐标的两个连续航点表达：前一个保持原航向，后一个改航向。
- `work_id=0` 为通行点，到达自动继续；`work_id>0` 为工作点，到达后等待。工作编号的抓取/放置含义由你们约定，例如 1=抓取、2=放置。
- 初始参数保留你已验证的电机地址、方向、76 mm 轮径及 `imu_yaw_sign=-1`。平移继续用现有“轮编码器计程 + IMU 航向纠偏”的 F6 速度控制，转弯用 IMU 角度闭环。

**绝对坐标是目标的表达方式，当前没有持续雷达定位反馈闭环。** 主控估计位置来自轮子反馈，不能识别地面打滑；估计位置默认到 6 mm、航向到 1.5°即判定到点，这些不是实测定位精度保证。抓取/放置精度仍需视觉或雷达定位作最终校正。当前视觉报文只更新测量并调用接口，不会自动纠偏或驱动抓取机构。

## 树莓派端快速试验

安装 `pyserial`，示例文件为 `tools/route_demo.json`。先检查地图起点与周边空间，再执行：

```bash
python3 -m pip install pyserial
python3 tools/route_client.py --port /dev/serial0 status
python3 tools/route_client.py --port /dev/serial0 run tools/route_demo.json --interactive
```

示例：从 (0,0,0°) 前进到 (500,0,0°)，等待 work_id=1；确认真实动作结束后在终端输入 `done`；移动到 (500,300,0°)，原地转到 90°并等待 work_id=2；再次输入 `done` 后走到 (0,300,90°)。输入其他文字视作工作失败，路线进入 FAULT。Ctrl+C 发送 STOP。

移除 `--interactive` 后会停在工作点一直等待；测试程序不会伪造抓取完成。`run` 已占用串口时不能再开第二个 CLI 发完成通知，正式软件请使用同一个 `Client` 实例完成视觉和工作通知。

```python
from route_client import Client
import serial, json

def perform_work(client, status):
    # 在这里启动你们的真实视觉/机械臂流程，等待它实际完成。
    # 可以用 client.vision(...) 经同一串口发送视觉测量。
    # 返回 True 才会发送 WORK_DONE(success=1)；失败返回 False。
    return robot.perform_and_wait(status['work_id'])  # robot 为你们自己的机构控制对象

with serial.Serial('/dev/serial0', 115200, timeout=0.05, write_timeout=1) as port:
    client = Client(port)
    route = json.load(open('tools/route_demo.json', encoding='utf-8'))
    client.run(route, on_work=perform_work)
```

也可分步 `upload 文件.json`、`start 路线号`、`status`、`stop`；分步测试在程序退出释放串口后可运行 `work-done 路线号 航点索引`，索引从 0 开始。加入 `--failed` 表示工作失败。

## 主控抓取/放置接口

`route.c` 中的弱函数 `Route_OnWork(route_id, point_index, work_id)` 在每个工作点只调用一次，默认什么也不做，并保持停车等待。可在自己的应用 `.c` 文件实现同名函数来启动机械臂状态机。

该回调运行在 RobotTask，必须快速返回。机械臂动作完成后，在 **RobotTask** 调用 `Route_CompleteWork(route_id, point_index, 1)`；失败传 0。也可以让树莓派统一通过 WORK_DONE 报文通知，二者选定一个完成通知责任方。不能在串口中断/其他任务直接调用底盘/路线操作接口。

`Route_OnVision(const RouteVision_t *)` 在匹配当前工作点的视觉报文到达时调用，可保存测量用于机构操作。偏移约定 +dx 向车头、+dy 向左、+dyaw 逆时针，需树莓派完成相机坐标转换。视觉微调底盘需要后续扩展 WAIT_WORK 的控制状态机；当前不要在此回调直接启动底盘运动，因为等待态不会调度底盘动作。

## 串口协议 V1

所有多字节整数为**小端**，结构按下述字段逐项打包，不直接发送 C 结构体。

| 帧字段 | 字节数 | 含义 |
|---|---:|---|
| 帧头 | 2 | `A5 5A` |
| version | 1 | `01` |
| type | 1 | 见下表 |
| sequence | 2 | 请求序号；ACK 原样返回 |
| payload_length | 2 | 0～1300 |
| payload | 可变 | 见消息定义 |
| CRC16 | 2 | CCITT-FALSE，poly=0x1021，init=0xFFFF，无反射，无 xorout；从 version 到 payload 尾；CRC 也为小端 |

CRC 标准校验值：ASCII `123456789` → `0x29B1`。超长/错误版本/CRC 错误帧丢弃，不 ACK；半包间隔超过 500 ms 清除，环形缓冲溢出或 UART 错误时丢弃未解析数据重新同步。只要硬件无长时间屏蔽中断，64 点最大帧 1310 bytes 可连续发送，无需自行插入延时。

| type | 消息 | payload |
|---:|---|---|
| 0x10 | UPLOAD 整条路线 | 路线头 20 bytes + N×航点 20 bytes |
| 0x11 | START | route_id:u32 |
| 0x12 | STOP | route_id:u32；0 表示停止当前路线 |
| 0x13 | WORK_DONE | route_id:u32, point_index:u16, success:u8, reserved:u8=0 |
| 0x14 | STATUS | 空 |
| 0x30 | VISION | 下述 20 bytes |
| 0x80 | ACK | 下述 36 bytes，sequence 与请求一致 |
| 0x81 | EVENT | 同样 36 bytes，sequence 为独立事件计数 |

UPLOAD 路线头（20 bytes）：

| 偏移 | 字段 | 类型 / 限制 |
|---:|---|---|
| 0 | route_id | u32，非 0；同一次上电中，新路线必须递增 |
| 4 | count | u16，1～64 |
| 6 | reserved | u16，0 |
| 8 | initial_map_x | i32，mm，±100000 |
| 12 | initial_map_y | i32，mm，±100000 |
| 16 | initial_map_heading | i32，**0.01°**，−18000～18000 |

每个航点（20 bytes）：

| 偏移 | 字段 | 类型 / 限制 |
|---:|---|---|
| 0 / 4 | x / y | i32，绝对地图 mm，±100000 |
| 8 | heading | i32，绝对地图 0.01°，−18000～18000 |
| 12 | move_rpm | u16，1～120，建议先 40 |
| 14 | turn_rpm | u16，1～120，建议先 20；为轮子转速上限 |
| 16 | work_id | u16，通行点必须 0，工作点必须非 0 |
| 18 | flags | u16，0=通行点，1=工作点，其他位必须 0 |

VISION（20 bytes）：`route_id:u32, point_index:u16, work_id:u16, dx_mm:i32, dy_mm:i32, dyaw_cdeg:i16, confidence:u16`。confidence=0～1000；0 表示无可靠检测。仅当前 WAIT_WORK、路线号/索引/工作号全匹配时接收。接收视觉测量不等同于完成工作。

ACK/EVENT 公共 payload（36 bytes）：

| 偏移 | 字段 | 类型 |
|---:|---|---|
| 0 | request_type | u8，EVENT 为 0 |
| 1 | result / event_code | u8 |
| 2 / 3 | state / phase | u8 / u8 |
| 4 | route_id | u32 |
| 8 / 10 / 12 | point_index / point_count / work_id | u16 / u16 / u16 |
| 14 / 15 | chassis_fault / fault_address | u8 / u8 |
| 16 | tick_ms | u32 |
| 20 | route_fault | u32 |
| 24 / 28 | estimated_map_x / y | i32 mm，在每个底盘子动作完成后更新 |
| 32 | map_yaw | i16，0.01°；RUNNING 时跟随 IMU 更新 |
| 34 | reserved | u16，0 |

状态 state：0 EMPTY、1 READY、2 RUNNING、3 WAIT_WORK、4 DONE、5 STOPPED、6 FAULT。

phase：0 规划、1 转向、2 前后移动、3 左右移动、4 等待工作。

ACK result：0 OK、1 BAD_TYPE、2 BAD_LENGTH、3 BAD_CRC（保留，CRC 错误实际静默丢弃）、4 BAD_VALUE、5 BUSY、6 BAD_ID、7 BAD_STATE、8 CONFLICT。

EVENT code：1 READY、2 STARTED、3 ARRIVED、4 WORK_REQUIRED、5 DONE、6 STOPPED、7 FAULT、8 POINT_DONE。使用 WORK_REQUIRED 触发工作；ARRIVED 是进入等待之前的到达通知。DONE 时 point_index 等于 point_count。

route_fault：0 无、1 底盘故障、2 动作启动/配置失败、3 工作失败、4 工作等待超时、5 外部停止、6 IMU 掉线。底盘故障码仍见 `chassis_motion.h`，7 表示 IMU 掉线。

## 重发、恢复与停车

- 全包 CRC 与所有点参数都通过，才提交新路线；错误的后续航点不会造成前面的点先执行。
- 相同 route_id、完全相同内容的 UPLOAD 重发只确认，不复位进度；同号内容不同报 CONFLICT。RUNNING/WAIT_WORK 时拒收另一条路线。
- START 在 RUNNING/WAIT_WORK 时重发不重复启动；DONE/STOPPED/FAULT 后不能用同号重新 START。要重跑必须上传新 id，并填**新的实际起点**。
- WORK_DONE 必须对应当前等待的点。最近一次成功处理的工作完成报文允许重复确认；长期延迟到后面多个点才重发的旧通知会被拒绝。
- ACK 与 EVENT 可能先后交错。客户端按 sequence **和 request_type** 匹配 ACK。不要把下一帧默认当作 ACK。
- 异步事件不保证送达；通信繁忙或发射异常可丢回复，主控保留状态。树莓派定期 STATUS 查询，尤其用 WAIT_WORK/index/work_id 恢复漏掉的工作通知。
- KEY0 按下取消 READY/RUNNING/WAIT_WORK 整条路线；按住时禁止 START。STOP 不是暂停，取消后不会继续执行。
- 默认 `route_config.work_timeout_ms=0`，工作点无限等待。需要超时时，在 START **之前**设定。配置在 START 时快照。
- 路线缓存和 id 记录不掉电保存。主控重启后 EMPTY，不自动恢复，不自动运动；树莓派应重新定位、上传和显式启动。串口断连不会自动中止正在执行的路线，会继续当前路线并在工作点等待；需要断连停车时再设计双方心跳与超时约定。

## 调试观察与验证

Watch 优先看 `route_status.state / point_index / work_id / phase / fault`，地图目标与估计值，以及 `chassis_motion.fault / fault_address`、`imu_data.online`。

串口问题看 `route_uart_debug.rx_events / uart_errors / restart_errors / tx_errors`，以及 `route_status.rx_bytes / crc_errors / rx_overflows / partial_timeouts / tx_drops`。RX DMA 有事件但没有有效帧时检查协议、CRC、单位与树莓派串口控制台占用。

已验证：Keil ARMCC5 构建 0 errors，7 个原有第三方文件尾换行 warnings；新增 15 项协议/路线/UART 主机测试通过，原有 46 项距离/航向/转向仿真回归通过。这些验证不含真实硬件运行。

```powershell
& tests/route/build_and_test.ps1
& tests/heading/build_and_test.ps1
```

修改前快照：`tmp/before_route_20261102/`。
