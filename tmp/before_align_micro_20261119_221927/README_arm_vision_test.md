# 静止测试各工作区云台与视觉接口

## 切换模式

在 `Core/Inc/arm_config.h` 修改，重新编译并下载：

```c
#define ARM_VISION_TEST_MODE      1
```

- `0`：整车模式，选区键记录原点后通知树莓派规划路线，沿用到工作点后的云台流程。
- `1`：云台独立联调模式。选区键只记录云台原点，不发送 START_AREA，不启动雷达路线。串口路线 START 返回 BAD_STATE，底盘 Watch 手动运动也禁用。无需底盘到点、无需 IMU 航向就绪，视觉直接选择云台测试工作区。

本次默认设为 **1**。测试结束改回 **0** 并重新编译下载。按键仍为 KEY0=PC5、KEY1=PA15，低电平；动作中 KEY0 停止云台。

## 主控侧操作

1. 手动摆好三轴初始姿态：上下最高、前后最前、yaw朝待测试工作区。把车固定放在合适的拍照/抓取距离；此模式不会自动移动或旋转底盘。
2. 短按并松开 KEY0 或 KEY1，主控自动记录三轴编码器原点，不发送位置运动。独立模式下两个选区键的原点记录作用相同。
3. 树莓派通过 UART4 的 `ARM_STATUS(0x32)` 等待 `state=2 READY`、`flags & 1 != 0`。这是软件自动等待，不需要人工检查 Watch。`flags & 8 != 0` 表示当前固件为独立模式。
4. 视觉发送 TEST_BEGIN 选择工作区；主控通过原避让顺序去拍照姿态，完成后报告 WAIT_VISION。
5. 视觉定位、发送三轴绝对目标角度；对齐后发送 PICK，主控只下降上下轴；真实外部夹爪处理完成后再发送 GRIP_DONE，主控抬升。FINISH 收回后，本轮测试结束。

选区键代表你确认初始姿态已经摆好，不会判断机构是否确实在最高/最前。连续切换测试工作区可直接 FINISH 后发新的 TEST_BEGIN，不用重新摆位或清零；始终使用最初记录的原点。若确实手动挪动了轴，先结束/停止当前测试，再摆回初始姿态、短按松键记录新原点。

## 工作区编号

| work_id | 区域 |
|---:|---|
| 102 | 第一次原料区 |
| 103 | 第一次粗加工区 |
| 104 | 第一次暂存区 |
| 105 | 第二次原料区 |
| 106 | 第二次粗加工区 |
| 107 | 第二次暂存区 |

二维码101、启停区108不启用云台抓取测试。各工作区使用相同的抓取状态机；具体目标由视觉发送，不会根据区域自动猜测物料位置或旋转底盘。

## 树莓派调用示例

复制本次更新的 `tools/route_client.py`、`tools/arm_sequence.py` 到树莓派。复用一个 UART4 Client，暂时停用原来的“等选区→雷达规划→上传→START”任务，不要有两个进程同时读取串口。

```python
import serial
from route_client import Client
from arm_sequence import ArmStation, MotorPose

with serial.Serial('/dev/serial0', 115200, timeout=0.05, write_timeout=1) as port:
    client = Client(port)
    # 人工摆好后按键。程序轮询等待记录完成，无需手动写Watch。
    client.wait_arm('READY', timeout=120)

    # 每轮测试使用更大的test_id；主控重启后计数可重新从1开始。
    station = ArmStation.for_test(client, test_id=1, work_id=102)

    # 此处用真实视觉算法计算的绝对“电机轴角度”替换下面的小行程示例。
    # 下面相当于下降1mm、机构yaw右转3°、后退约1mm。
    station.align(MotorPose(z_deg=-9.0, yaw_deg=9.0, reach_deg=-3.18))

    # 重新拍照确认中心与物料/夹爪投影对齐后才执行：
    station.lower_after_visual_confirmation()
    # 现在停在WAIT_GRIP。当前固件未写夹爪舵机控制。
    # 实际夹爪动作完成后，再由视觉业务代码调用：
    # station.lift_after_external_grip_completed()
    # station.park()
```

需要多次修正时重复 `station.align(新的绝对目标)`，不累加上次角度。抓完一件抬升后可继续下一件 ALIGN→PICK→外部夹爪确认→GRIP_DONE。全部结束用 `station.park()` 等 READY_LEAVE，再使用更大的 test_id 创建新工作区测试。独立测试 **不发送路线 WORK_DONE(0x13)**。

只想测试通信和拍照姿态，可运行：

```bash
python3 tools/route_client.py --port /dev/serial0 arm-status
python3 tools/route_client.py --port /dev/serial0 arm-test-begin 1 102
```

此命令仅开启测试并等到拍照姿态，不自动下降抓取。后续动作由同一个常驻视觉程序调用 SDK；重新连接先检查 arm-status，不能用新命令号盲目重复下降。

## 视觉自写协议时的定义

保留 A5 5A 帧、version=1、sequence、小端字段和 CRC16。复用 `type=0x31` 的32-byte payload，新增 opcode=6（TEST_BEGIN）：

| 偏移 | 类型 | TEST_BEGIN取值 |
|---:|---|---|
| 0 | u32 route_id | 非零 test_id，每轮新测试严格递增 |
| 4 | u16 point_index | 0 |
| 6 | u16 work_id | 102～107 |
| 8 | u32 command_id | 1 |
| 12 | u8 opcode | 6 |
| 13 | u8 reserved | 0 |
| 14 | u16 reserved | 0 |
| 16 | i32 z_cdeg | 0 |
| 20 | i32 yaw_cdeg | 0 |
| 24 | i32 reach_cdeg | 0 |
| 28 | u32 reference_id | 0 |

ACK=0只表示接受。等 `type=0x83` 云台报告的 `state=4 WAIT_VISION`，并核对 `(route_id,point_index,work_id)=(test_id,0,work_id)` 后，再发 ALIGN。TYPE_ARM_EVENT 的格式不变，flags 新增 bit3=独立测试固件。路线 STATUS 仍反映真实路线状态（可能 EMPTY），不能要求其为 WAIT_WORK 才允许云台测试。

后续 opcode 沿用：2 ALIGN、3 PICK、4 GRIP_DONE、5 FINISH。三个身份字段保持一致，command_id从2开始递增，PICK/GRIP_DONE的reference_id必须对应最近成功ALIGN的command_id。相同命令重发不得改变内容；TEST_BEGIN重复不会重启已进行中的测试。上轮未FINISH或未重新摆好记录原点，不接受下一轮测试。

STOP(0x12，route_id=0)或动作中KEY0停止云台，原点失效；手动摆回、再次按键记录后，用更大的test_id重试。故障与通信超时继续保留诊断，不用虚构的到位或自动夹爪完成代替真实反馈。

## 调试参数

`Core/Inc/arm_config.h` 中保留原参数：ARM_PHOTO_Z/YAW/REACH_CDEG拍照姿态；ARM_PICK_DROP_CDEG下降量（当前9000电机cdeg=10mm）；三轴RPM和行程范围。所有目标单位是电机轴0.01°，不是毫米或机构角度。

- Z向下d毫米：`z_cdeg=-d*900`。
- 前后向后r毫米：`reach_cdeg=-r*36000/113.1`。
- 机构yaw向右a度：`yaw_cdeg=a*300`，向左为负。

调姿仍按抬至最高→后退60mm→转yaw→前后到目标→Z到目标执行。PICK只动Z；夹爪PWM、料盘转运仍未实现。

Watch可看 `arm_work_status`、`arm_motor_status[0..2]`；`route_keys_debug.arm_test_ready_count` 每次按键成功记录原点加1。独立模式下 `area_requests` 不增加是预期行为。

## 验证

实际C代码CPU仿真：整车模式42项、独立模式10项；Python协议/SDK12项。测试入口：

```bash
python tests/arm_work/run_tests.py --mode 0
python tests/arm_work/run_tests.py --mode 1
python tests/arm_work/test_client.py
```

仿真覆盖完整下降/等待夹爪/抬升/收回、六个work_id、重发、身份错误、行程限幅、按键记录、取消和反馈故障；实车效果需要下载后与视觉联调。
