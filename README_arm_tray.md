# 三位转盘与视觉联调

转盘舵机接 **PA7 / TIM3_CH2**，夹爪接 **PA6 / TIM3_CH1**。两个通道共用 20 ms 周期。第一件在初始档 `50` 放下，第二档为 `150`，第三档为 `250`。两次换盘都增加 100 个比较计数，按实测比例估算约 120°。每件入盘后机械臂先回到行驶收回姿态，转盘再逆时针换到空盘。第三件放好后保持在第三盘。出盘时选中最早入盘的物料，最后一件取完并放妥后回到 `50`。

## 实车标定

设置 `ARM_VISION_TEST_MODE=1` 时，独立云台测试默认启用转盘。整车模式也可以在 [arm_config.h](Core/Inc/arm_config.h) 将 `ARM_TRAY_ENABLED` 设为 `1`，使用同一套转盘动作。

| 参数 | 标定内容 |
|---|---|
| `ARM_TRAY_YAW_CDEG` | 机械臂从原点向右的固定盘位。按 3:1 传动比，机构角度每 1° 对应电机轴 `300 cdeg`。扩展角度用于固定转盘动作，视觉目标和手动调试仍限在 ±90°。 |
| `ARM_TRAY_REACH_CDEG` | 后退 115 mm 对应 `-36605 cdeg`。 |
| `ARM_TRAY_PLACE_Z_CDEG` / `ARM_TRAY_PICK_Z_CDEG` | 均为 `0`。机械臂到盘位后保持最高点，直接松爪或夹取。 |
| `ARM_TRAY_SETTLE_MS` | 初值 1000 ms。转盘没有回读，带负载时实测从 `250` 到 `50` 的动作时间再设定。 |

首次运行前清空三个小盘。摆好云台初始姿态，按键记录三轴原点。第一次取放时，主控到达行驶收回姿态后才启动 CH2。实际机构需确认 Z 到最高且前后轴回到避让位后，yaw 转动和小盘换位均无干涉。

## 视觉指令

颜色和槽号由视觉按任务码决定。电控按实际入盘顺序出盘，每条放置命令接收一个槽号和该槽**松爪时的绝对电机轴坐标**。粗加工区取回也按刚才放下的槽序执行。第二批暂存码垛时，视觉用颜色匹配第一批所占槽，并发送堆叠高度。

| 工作区 | 每件物料的报文顺序 |
|---|---|
| 原料 102 / 105 | `ALIGN` → 确认视觉对齐 → `PICK` → 等 `WAIT_GRIP` → `GRIP_DONE` → 等 `PICKED`。开启转盘功能时，`PICKED` 表示物料已放进小盘、机械臂已退出、需要换盘时已等待换盘完成。 |
| 粗加工 103 / 106 | 发送三次 `TRAY_PLACE`，每次等 `PLACED`。放齐后，按原槽序逐件发送 `ALIGN` → `TRAY_RELOAD_PICK` → 等 `WAIT_GRIP` → `GRIP_DONE` → 等 `PICKED`。 |
| 暂存 104 / 107 | 发送三次 `TRAY_PLACE`，每次等 `PLACED`。槽号和高度允许由视觉分别给出。 |

每个工作区做完三件后发送 `FINISH` 并等 `READY_LEAVE`。粗加工区包含放下三件和重新入盘三件。整车随后发 `WORK_DONE`。独立联调切换工作区用递增的 `TEST_BEGIN`，三盘状态会跨工作区保留。

复用树莓派现有 UART4 Client 和 [ArmStation](tools/arm_sequence.py)。放置用 `station.place_next_from_tray(slot, release_pose)`。粗加工区取回时，先 `station.align(approach_pose)`，再 `station.lower_for_tray_reload(slot, pick_z_deg)`，然后沿用 `station.lift_after_external_grip_completed()`。Python 的姿态单位为电机轴度数，串口报文为电机轴百分之一度。

### 0x31 命令扩展

| opcode | 字节 13 | Z/yaw/前后字段（偏移 16/20/24） | reference_id（偏移 28） |
|---|---|---|---|
| 7 `TRAY_PLACE` | 目标槽号 1～3 | 三个绝对松爪坐标 | 0 |
| 8 `TRAY_RELOAD_PICK` | 粗加工取回的槽号 1～3 | 绝对夹取 Z、0、0 | 最近一次 `ALIGNED` 的 command_id |

长度仍为 32 字节。ACK 表示命令已接受。重试使用同一个 command_id 和同一份报文。同一工作区重复占用已放槽位会返回 `CONFLICT`。

### 0x83 状态扩展

- `state=17 TRAY_LOADING`，`state=18 TRAY_UNLOADING`，`state=19 PLACED`。
- `flags` 的 bit4 表示转盘使能，bit5 表示中断后盘内数量待核对。
- payload 偏移 22 为盘内件数，偏移 23 为最近一次接受放置命令的槽号。两者均为 u8。

夹持时中断会保持夹爪闭合。若 bit5 置位，将机构停稳后托住并清空夹爪及小盘，用 Keil 写 `arm_work_debug.command=5` 后递增 `trigger`，再重新记录云台原点。转盘库存保存在主控 RAM，重新上电前应清空。
