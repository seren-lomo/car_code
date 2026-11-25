# KEY0 / KEY1 选择启停区并通知树莓派

对应 `E:\Inte_handing\car_code1` 的 STM32F103RCT6 工程。主控通过现有 UART4（PC10 TX / PC11 RX，115200 8N1）通知树莓派。通知使用已有二进制协议的扩展消息，不是裸发 ASCII `1` / `2`。

以下选区流程用于 `ARM_VISION_TEST_MODE=0`。设为1时按键只记录云台原点，不发送选区通知；独立云台联调见 [README_arm_vision_test.md](README_arm_vision_test.md)。

## 按键行为

| 状态 | KEY0：PC5，低电平 | KEY1：PA15，低电平 |
|---|---|---|
| 空闲、尚未启动、路线完成或停止后 | 上报“启停区2”，等待该请求的规划结果 | 上报“启停区1”，等待该请求的规划结果 |
| 行驶中 / 工作点 WAIT_WORK | 立即取消路线并停车；本次按住不触发扫描 | 忽略，不打断当前路线 |
| 已有 READY 路线，但尚未 START | 作废旧路线，重新选择启停区2 | 作废旧路线，重新选择启停区1 |

30 ms 消抖，稳定按下登记一次选择，松开后执行；长按不会产生新的请求号。两键一起按时不选择区域，全部松开后才能重新选择；正在运行时 KEY0 仍能停车。上电时已经按住的键必须先松开再按，防止上电直接触发。

按键同时代表你已经把云台手动摆到初始姿态：上下最高、前后最前、yaw朝工作区。松键后主控自动读取5/6/7号电机编码器并记录三轴新原点，不发位置运动；完成后才发送选区通知。无需先在 Watch 写 command=1，也无需树莓派发送 arm_zero。记录失败时不发送选区通知；记录中再按 KEY0 会取消本次操作。

任意键按住时禁止底盘 START。树莓派测试客户端在按键规划流程中最多等待 10 秒，允许用户松开按键、IMU 就绪；超过后返回错误，用户应查询状态再处理。

PA15 配为 GPIO Input / Pull-up。工程已使用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`，禁用 JTAG、保留 SWD，所以 PA15 可用且仍可通过 PA13/PA14 下载调试。`.ioc` 已同步此配置，无需开按键 EXTI 中断。

## 通信流程

```text
先运行树莓派接收程序，保持 UART4 通道打开
 → 人工将车放入实际启停区，并将云台手动摆到初始姿态
 → KEY1 / KEY0 稳定按下后松开
 → STM32 自动记录三轴编码器原点（不执行机械回零）
 → STM32 发送 START_AREA(area=1 / 2, request_id)
 → 树莓派发送 AREA_ACK
 → STM32 回复普通 ACK，停止该请求的定时重发
 → 树莓派开始雷达扫描和路线规划
 → 树莓派发送 UPLOAD_FOR_AREA(request_id + 整条绝对坐标路线)
 → STM32 校验请求号、整包数据和所有航点，回复 READY
 → 树莓派发送 START(route_id)
 → 按原路线逻辑执行，并在工作点等待真实完成通知
```

未收到 AREA_ACK 时，STM32 每 500 ms 重发相同 request_id 的通知。重发是同一个任务请求，树莓派不能每收到一帧就重开一次扫描。

重新按下选择另一启停区，会产生新 request_id 并清除旧缓存路线。旧扫描结果即使随后才回来，也会因请求号不匹配被拒绝。STOP 会取消正在等待的扫描请求和后续路线执行；雷达侧如正在耗时计算，应自行取消计算或丢弃其结果。

START_AREA 的 request_id 和路线 route_id 用途不同：前者由 STM32 每次按键生成，后者由树莓派每条新路线递增生成。request_id 在同次主控上电中递增，不掉电保存；主控重启后重新建立树莓派 Client/任务状态。

## 树莓派：先只验证按键通信

```bash
python3 -m pip install pyserial
python3 tools/route_client.py --port /dev/serial0 wait-area
```

运行后再按键，例如 KEY0 输出：

```json
{"request_id": 1, "tick": 12345, "area": 2, "key": 0}
```

此命令已经自动确认按键通知，但只打印、不启动底盘、不调用真实雷达。request_id/tick 以实际返回为准。

## 树莓派：接入你们的雷达规划函数

```python
import serial
from route_client import Client

def scan_and_plan(signal):
    area = signal['area']             # 1=启停区1，2=启停区2
    # 下面 radar 是你们自己的雷达/规划模块，不是本仓库现成的驱动。
    # 只有收到按键并完成确认后，才会进入此函数。
    radar.start_scan(area)
    route = radar.plan_absolute_route(area)
    # route 结构与 route_demo.json 一致：
    # route_id、真实 initial_pose，以及绝对地图 points 列表。
    return route

def on_work(client, status):
    # robot 为你们自己的机构控制对象；真实完成后才返回 True。
    return robot.perform_and_wait(status['work_id'])

with serial.Serial('/dev/serial0', 115200, timeout=0.05, write_timeout=1) as port:
    client = Client(port)
    client.run_after_key(scan_and_plan, on_work=on_work)
```

`run_after_key()` 已封装：等按键、确认、调用扫描规划函数、带对应请求号上传、START、处理工作等待。一次调用处理一次任务；下一次任务再次调用，或在你们的主程序任务循环中调度。所有雷达、视觉和工作报文复用这一个 Client，不能再开另一个进程抢读此串口。

如果现有程序已有自己的状态机，可分开调用：

```python
signal = client.wait_start_area()
route = scan_and_plan(signal)
client.run(route, on_work=on_work, start_request_id=signal['request_id'])
```

已选择启停区后，必须用对应 token 上传。原普通 UPLOAD(0x10) 会被拒绝，防止绕过本次按键请求；在未按任何选择键的上电会话中，原直接上传/运行功能仍可用于独立通信调试。

树莓派程序本身必须把“开始扫描/规划”的调用放在该回调内。主控发通知无法阻止树莓派其他代码提前扫描。

## 不接雷达，先跑预设路线测试

`tools/route_area1_demo.json` / `route_area2_demo.json` 提供两份不同起点和航向的示例，**坐标只是示范，不是你的比赛地图坐标**。先修改成实际起点和目标，并确认前方约 500 mm 空间合适，再运行：

```bash
python3 tools/route_client.py --port /dev/serial0 wait-run \
  --area1 tools/route_area1_demo.json \
  --area2 tools/route_area2_demo.json \
  --interactive
```

KEY1 选择 area1 文件，KEY0 选择 area2 文件。先运行程序，再短按并松开按键。示例在一个工作点停下，输入 `done` 表示真实工作完成；Ctrl+C 发 STOP。

同次主控上电中新路线 route_id 必须递增。再次测试时修改 JSON 的 route_id，同时给出本次实际起点。不能反复使用已完成的同一个 route_id。

分步 CLI 示例：先 `wait-area` 取得并确认 request_id=1，程序释放串口后运行 `upload 文件.json --start-request-id 1`，再 `start 路线号`。生产软件优先用一个常驻 Client，不拆成多个进程。

## 扩展报文定义

帧头、版本、sequence、小端编码、CRC16 与 `README_route.md` 一致。新增最大 payload 为 1304 bytes，完整帧最大 1314 bytes；普通 UPLOAD 的路线内容仍最大 1300 bytes。

### STM32 → Pi：START_AREA，type=0x82，payload=12 bytes

| payload偏移 | 字段 | 类型 / 含义 |
|---:|---|---|
| 0 | request_id | u32，非0，本次按键请求号 |
| 4 | tick | u32，原点记录完成并生成选区请求时的 HAL tick，重发保持不变 |
| 8 | area | u8，1 或 2 |
| 9 | key | u8，0=KEY0，1=KEY1 |
| 10 | reserved | u16，0 |

type=0x82 的 payload 与 0x80/0x81 的 36-byte 状态不同，树莓派先按 type 分流再解析。

### Pi → STM32：AREA_ACK，type=0x15，payload=8 bytes

`request_id:u32, area:u8, reserved:u8[3]=0`。请求号/区域必须对应当前选择。STM32 用原 0x80 ACK 回复结果；已确认的相同请求可重复确认，但不会恢复已取消的路线；过期请求或取消后才首次确认的请求会被拒绝。

### Pi → STM32：UPLOAD_FOR_AREA，type=0x16

`request_id:u32 + 原UPLOAD完整payload`，即 4-byte 请求号 + 20-byte 路线头 + N×20-byte 航点。先 AREA_ACK 成功，再上传；上限 64 点。不是在每个点前加请求号。

上传完成仍需原 START(0x11)，其中只携带 route_id。后续 VISION、WORK_DONE、STATUS、STOP 格式保持不变。

## 调试变量和调用关系

```text
main: Route_Init() → RouteKeys_Init() → RouteUART_Init()
RobotTask: RouteKeys_Task() → Chassis_DebugTask() → Route_Task()
RouteKeys_Task(): 消抖 / 松键 → Route_PrepareStartPose() → 等 Route_CheckStart() 成功
  → Route_RequestStartArea()
ArmWork_Task(): 自动依次记录三轴原点 → READY
Route_RequestStartArea(): 清除旧路线 → send_start_area() → RouteUART_Send()
Route_Task(): 处理 AREA_ACK / UPLOAD_FOR_AREA；未确认则定时重发
```

Watch：

- `key0_level`、`route_keys_debug.key1_level`：松开 1，按下 0。
- `key0_press_count`、`route_keys_debug.key1_press_count`：消抖后的按下次数。
- `route_keys_debug.area_requests`：实际接受了多少次启停区选择。
- `route_keys_debug.pending_area`：待处理的区域，0无、1区1、2区2。
- `route_keys_debug.pose_preparing`：1正在自动记录原点；结束后回0。
- `route_keys_debug.pose_result`：0成功、5仍忙/记录中、7原点记录失败或机构状态不满足。
- `route_start_area.area / request_id`：当前区域和请求号。
- `route_start_area.tx_count`：本请求成功入发送队列次数，未确认时会增长。
- `route_start_area.acknowledged`：收到正确 AREA_ACK 后为 1。
- `route_start_area.pending`：选择后到整条路线接收成功前为 1；取消后也清0。
- `route_status.state`：选区后 EMPTY；正确路线接收后 READY；START 后 RUNNING。

验证包含按键消抖、长按重发、双键/上电按住、活动中 KEY0 停车、活动中 KEY1 忽略、旧请求拒绝、确认/路线上传、64 点 DMA 接收以及树莓派确认后才调用规划。主机测试不等于实车验证，仍需你按上述流程联调。
