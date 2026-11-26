# 七电机共用 CAN1

工程：`E:/Inte_handing/car_code1`，STM32F103RCT6，CAN1，500 kbit/s。此次将三轴机构从 USART1/2/3 改为现有 CAN，总共七个独立地址。电机仍用 EMM FD 位置控制，所有目标仍相对手动记录的初始姿态。

## 地址与接线

| 机构 | CAN 地址 | 软件观测 |
|---|---:|---|
| 左前轮 | 1 | motor_feedback[1] |
| 右前轮 | 2 | motor_feedback[2] |
| 右后轮 | 3 | motor_feedback[3] |
| 左后轮 | 4 | motor_feedback[4] |
| 上下 Z | 5 | arm_motor_status[0] / arm_can_diag[0] |
| 云台 yaw | 6 | arm_motor_status[1] / arm_can_diag[1] |
| 前后 reach | 7 | arm_motor_status[2] / arm_can_diag[2] |

七个电机 CANH 接同一根 H，CANL 接同一根 L；主控继续用已有 TJA1050：PA12(CAN_TX)→TXD、PA11(CAN_RX)←RXD。CANH/L 不直接接 PA 引脚。各节点信号地连接到共同参考地，动力电源按电机要求供电。

沿车布线为一条主干，各电机短支线引出，避免长星形分叉。**整个总线只在两个物理末端各接一个 120Ω 终端**，其他节点关闭内置终端；不是每台电机都接 120Ω。全断电后测 CANH 与 CANL，两个 120Ω 并联应约 60Ω（测量时考虑模块内部电路影响）。原 USART1/2/3 与电机 T/R 的线撤下，七电机使用各自实际 CAN 接口。

## 电机菜单

全部电机：`P_Serial=CAN1_MAP`、`CAN_Baud=500000`、`Checksum=0x6B`、`Response=Receive` 或 `Both`、`S_Vel_IS=Disable`、`Ctrl Mode=CR_VFOC`、EMM 固件。ID 分别为 1～7，不能重复。三轴 `Mstep=16`，配合代码 `ARM_PULSES_PER_REV=3200`；底盘继续使用此前验证的细分设置。

`CAN1_MAP` 是驱动器接口映射名称；波特率和菜单需与此前能正常工作的底盘电机一致。保存菜单并按电机要求重新上电使配置生效。

## 可行性与负载

七个节点可以共用当前 CAN。闭环电流/位置控制在各驱动器内部运行，主控只发目标、查询反馈，不需要将闭环控制每个周期都放到总线上。

按扩展帧每帧最多约 160 bit（包括填充等开销）的保守工程预算：底盘 20ms 更新一轮，四轮各查询位置/速度/状态，往返 24 帧；四轮速度指令及回执 8 帧，加同步触发约 1～2 帧，总计约 33～34 帧，约占 500k 带宽的 53～55%。云台一次只动一轴，20ms 三项查询往返 6 帧，约占 10%。实际帧长与任务等待会降低平均占用。这是估算，尚未用 CAN 分析仪测量；当前流程通常是底盘停稳后再动作云台，不会同时达到两者峰值。

没有为了七个电机新开七个任务。所有 CAN 发送仍在 RobotTask 顺序执行，接收由共享 FIFO0 中断分发。后续若新增高频遥测或底盘与多轴同时运动，应重新测占用和响应延时。

## 代码变化

- `Core/Src/arm_can.c`、`Core/Inc/arm_can.h`：三轴 CAN 驱动，ID5/6/7，位置指令复用 `can_SendCmd()` 分包。
- `Core/Src/can.c`、`Core/Inc/can.h`：共享反馈数组扩至地址1～7，FIFO0 分发云台反馈，保留底盘处理。
- `Core/Src/arm_work.c`、`Core/Inc/arm_work.h`：流程调用换为 `ArmCAN_*`；视觉调姿、下降、等待夹爪、抬升、收回规则沿用原流程。
- `Core/Src/usart.c`：取消三个电机的串口回调。USART1/2/3 的 CubeMX 初始化保留，但机构驱动不再使用它们；UART4 树莓派、UART5 IMU 沿用现有配置。
- `MDK-ARM/car_code1.uvprojx`：编译 `arm_can.c`。旧 `arm_uart.c` 为停用说明文件，`arm_uart.h` 仅为包含兼容壳，不再提供旧 API。
- Watch 将 `arm_uart_diag` 换为 `arm_can_diag`，`arm_motor_status[i].uart_errors` 换为 `can_errors`。Pi 的 UART4 二进制协议和电机轴角度单位不变。

云台 FD 指令同步位=0，立即执行；停止逐个发给5/6/7。底盘原有地址0同步触发保留；云台不排队等待同步，不会被底盘触发为新动作。

## 首次测试

先编译下载，保持主控运行，让树莓派暂不启动路线。Watch 添加 `arm_work_debug`、`arm_work_status`、`arm_motor_status[0..2]`、`arm_can_diag[0..2]`、`can_last_tx_status`、`can_tx_error_count`。所有命令均先填参数，最后把 trigger 在当前值上加1。

先只读上下轴：

```text
arm_work_debug.command = 4
arm_work_debug.axis = 0
arm_work_debug.trigger = 1  // 假设当前为0
```

约1秒后应见 `arm_motor_status[0].state=2`、`fault=0`、`rx_frames` 增加。依次改 axis=1、2，每次 trigger 再加1，检查yaw与前后。此操作不记录原点，不发运动。

上下轴读位置：逻辑 `tx_raw=05 36 6B`；总线实际发扩展ID `0x500`、载荷 `36 6B`。回复扩展ID `0x500`、7字节载荷 `36 sign pos[4] 6B`。实际 CAN 载荷没有开头的05；地址在扩展ID里。yaw/前后对应0x600/0x700。

三轴都读通后，手动摆到初始姿态，command=1、trigger加1，等 `arm_work_status.state=2` 且三轴 `homed=1`。再用 command=2，axis=0/1/2，`target_cdeg=-900/900/-318` 分别测试向下约1mm、向右3°机构yaw、后退约1mm，速度可暂改 `arm_motor_config[i].rpm=5`。测试的是相对原点的绝对目标；重复同一目标不会继续累加。完成后command=3、trigger加1收回并恢复READY。详细说明见 `DEBUG_GUIDE_arm.md`。

本机实测正Z向上、正yaw向右、正reach向前；零位为最高/最前/臂朝工作区，Z/reach最大角度现设0，下降方向改-1。负向总行程、避让位置仍待测。配置在 `Core/Inc/arm_config.h`；保留现有用户设定：yaw范围±27000电机cdeg，抓取下降9000电机cdeg。

## 通信诊断

- `tx_completed` 增加：逻辑命令各个 CAN 分包均发送成功，仍须检查电机应用层回复；其他节点也可能提供 CAN ACK。
- `tx_can_frames`：完整发送成功命令所使用的 CAN 帧数量，失败的部分分包不计入。
- `rx_can_frames` / `rx_bytes`：对应电机地址收到的 CAN 帧/载荷字节，包括之后被协议校验丢弃的帧。
- `rx_raw`：最近发送后收到的前16个 CAN 载荷字节；`last_rx_ext_id`、`last_rx_dlc` 辅助核对。
- `last_hal_status`：0成功、1错误、2忙、3超时；`last_can_error` 为 HAL CAN 错误位组合。
- 单轴fault=2：发送失败，先查接线、波特率、终端、总线错误；fault=4：等待回复或整个操作超时，不表示单片机必然没发；fault=0表示当前无记录故障，需结合state/rx_frames判断动作结果。
- 发送成功但无回包：检查该电机CAN菜单/ID/Response及支线。`rx_can_frames` 增加但 `rx_frames` 不增：检查载荷、功能码、DLC、6B帧尾与反馈格式。

三轴原点记录顺序为5→6→7，第一轴失败就停止其余轴；其他轴fault=0不等于已经通信成功。command=4可逐台独立验证。

## 验证范围

Keil重编译通过，0错误、7条原有文件末尾换行警告。实际C代码CPU仿真：21项电机驱动、31项抓取工作流、8项共享CAN集成；Python客户端另有8项协议测试。测试入口为 `tests/arm_can/run_tests.py`、`tests/arm_work/run_tests.py`、`tests/can_shared/run_tests.py`、`tests/arm_work/test_client.py`。未自动烧录，也未完成七台电机实车验证。

2026-11-19方向更新：抓取下降9000电机cdeg的幅度保留，实际目标为对齐Z减9000。CLEAR_REACH当前0是最前，自动yaw和行驶时的缩臂距离待测后设置为负值。

修改前备份：`tmp/before_arm_can_20261119_153447/`。
