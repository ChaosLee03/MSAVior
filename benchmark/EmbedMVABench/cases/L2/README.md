# L2 级案例 — 跨函数/间接访问

L2 级缺陷特征：共享变量的冲突读写涉及**跨函数调用**或**间接访问**。主任务侧通过一层函数调用间接访问共享变量，或 ISR 侧通过回调/子函数写入。调用链深度为 2。

共 10 个正例。

---

## L2_001 — 四路K型热电偶PID温控系统

- **领域**: 工业温控 | **处理器**: 8051 | **代码行数**: 1039
- **系统概述**: 基于8051单片机的四路独立加热区PID闭环温度控制系统，通过K型热电偶采集温度并驱动继电器式加热器，实现多通道精密恒温控制。
- **功能模块**:
  - `main()`: 200ms周期主循环，依次执行温度采集、模式状态机、PID运算和EEPROM持久化
  - `RunHeatModeSM()`: 温控模式状态机（INIT→PREHEAT→PID→HOLD→FAULT）
  - `CollectTemperature()`: 采集各通道AD590温度，移位平均滤波
  - `PDKongWen()`: PD控温核心算法，遍历4通道计算偏差与微分项，更新HeatOpenTime
  - `Timer0_INT()`: ⚡ 定时器0中断（1ms），根据HeatOpenTime控制继电器通断，实现时间比例式PWM
  - `LoadPIDParams()` / `SavePIDParams()`: EEPROM参数加载/保存（含CRC校验）
  - `DiagOutput()`: 串口诊断数据帧输出
- **硬件接口**: AD590温度传感器（4通道12位ADC, 0x4000），继电器加热器（4路, 0x8000），EEPROM (0x0400)，LCD (0x0800)，看门狗 (0x7FFE)，UART
- **模块关系**: 主循环采集温度→状态机→PD计算→更新HeatOpenTime; Timer0_INT每1ms比较HeatOpenTime与计数器决定继电器通断。
- **缺陷概要**: `HeatOpenTime[4]`（int16数组）— `PDKongWen`(主循环)逐元素读-改-写 vs `Timer0_INT`(1ms中断)读取同一元素，8051对16位操作非原子导致撕裂

---

## L2_002 — 星载工程参数存储备份系统

- **领域**: 嵌入式存储管理 | **处理器**: 8051 | **代码行数**: 1179
- **系统概述**: 基于8051的星载嵌入式系统，负责工程遥测参数的周期性备份存储、CRC校验、Flash/EEPROM持久化以及RS422遥测下传。
- **功能模块**:
  - `main()`: 50ms主循环，喂狗、系统维护、时间检查、存储状态机
  - `system_maintance()`: 系统维护，周期触发备份任务，监测母线电压和轨道周期
  - `important_backup()`: 将时间等关键字段逐字节写入备份RAM并计算校验和
  - `TriggerBackupTask()`: 备份调度，最多重试3次
  - `RunStorageSM()`: 存储状态机（空闲→写入→校验→故障→恢复）
  - `timer2_int()`: ⚡ 定时器2中断（10ms），递增System_time毫秒字段，溢出进位秒字段
  - `CheckSystemTime()`: 读取时间快照做完整性检查
- **硬件接口**: EEPROM (0xA000, 8扇区), 备份RAM (0xC000), RTC (0x8000), Flash (0x5000), RS422 (0x2000), 看门狗 (0x7E00), 电压监测 (0x3000)
- **模块关系**: 主循环周期触发`important_backup`分步读取System_time[0/1/2]写入备份区; `timer2_int`每10ms分步递增这三个字段。
- **缺陷概要**: `System_time[0/1/2]`（三元组）— `important_backup`(主循环)分步读取 vs `timer2_int`(10ms中断)分步递增并进位，备份区可能记录不一致时间

---

## L2_003 — 航天载荷相机RS422通信管理系统

- **领域**: 航天载荷相机通信 | **处理器**: 8051 | **代码行数**: 1107
- **系统概述**: 基于8051的航天载荷管理系统，通过RS422串行总线与星载相机（A/B双机备份）进行帧通信，实现相机指令收发、帧校验、快门/曝光/增益参数解析、错误统计和超时恢复。
- **功能模块**:
  - `main()`: 10ms主循环，喂狗、RS422状态机、载荷管理、超时检查
  - `RunRS422StateMachine()`: RS422五态通信状态机
  - `RS422_int1()`: ⚡ 外部中断1，从RS422 FIFO读取完整17字节帧到CAMERA_A_REC_BUF
  - `payload_management()`: 载荷管理，调用工程参数采集和RS422帧处理
  - `payload_RS422()`: 拷贝命令字段→帧校验→解析命令
  - `ParseCameraCmd()`: 解析相机命令（快门/曝光/增益/状态查询）
  - `CheckRS422Timeout()`: 5秒无响应超时检测和接口复位
- **硬件接口**: 相机A RS422 (RX FIFO 0xA000, 状态 0xA002, TX 0xA004), 相机B RS422 (0xB000), 状态寄存器 (0x4000), EEPROM (0xD000), 看门狗 (0x6000/P1.5), 外部中断1
- **模块关系**: `RS422_int1`中断将17字节帧写入CAMERA_A_REC_BUF并置标志; 主循环`payload_RS422`从缓冲区拷贝命令字段并校验帧。
- **缺陷概要**: `CAMERA_A_REC_BUF[17]`/`Camera_A_RS422`/`Camera_A_Err_Cnt` — `payload_RS422`(主循环)先拷贝命令再校验帧 vs `RS422_int1`(中断)覆写整个缓冲区

---

## L2_004 — 卫星1553B广播遥测参数打包系统

- **领域**: 卫星1553B广播遥测 | **处理器**: 8051 | **代码行数**: 1002
- **系统概述**: 基于8051和65170 RT芯片的卫星1553B总线遥测系统，负责接收广播参数、打包遥测数据包到RT RAM的TX发送缓冲区。
- **功能模块**:
  - `main()`: 50ms主循环，RT状态机和任务调度
  - `RTStateMachine()`: RT状态机（RESET→READY→ACTIVE→FAULT）
  - `WriteYCReturnBagTask()`: 遥测数据包写入任务，从FPGA拷贝192字节
  - `fnBrcstParaSet()`: 广播参数打包核心，将BRCST_buf参数写入RT RAM
  - `ExtractBroadcastParam()`: 从BRCST_buf[6]/[7]提取位置时间参数
  - `ace_int0()`: ⚡ 外部中断0（1553B事务完成），RX时批量拷贝到BRCST_buf
  - `CheckBroadcastSeq()`: 广播序列号连续性检测
- **硬件接口**: 65170 RT芯片 (RT RAM 0x8000, TX子地址8 0x8100, RX子地址8 0x8200), FPGA (0x7000), BST缓冲 (0x9000), 看门狗 (0x6000), MIL-STD-1553B总线
- **模块关系**: `ace_int0`中断RX时批量拷贝到BRCST_buf; 主循环`ExtractBroadcastParam`分步读取BRCST_buf[6]/[7]写入RT RAM。
- **缺陷概要**: `BRCST_buf[6]`/`[7]` — `ExtractBroadcastParam`(主循环)分步读取 vs `ace_int0`(中断)批量覆写，位置-时间参数不一致

---

## L2_005 — 卫星1553B遥控向量字写入系统

- **领域**: 卫星1553B遥控写入 | **处理器**: 8051 | **代码行数**: 1061
- **系统概述**: 基于8051的卫星1553B总线RT终端遥控子系统，负责接收地面遥控指令、执行指令鉴权，将遥测数据包写入TX子地址，并管理向量字读写以触发RT服务请求。
- **功能模块**:
  - `main()`: 50ms主循环，任务调度
  - `RunTCStateMachine()`: 遥控指令状态机（IDLE→PARSING→EXEC→ACK→ERROR）
  - `WriteYCReturnBagTask()`: 遥测包写入并触发服务请求
  - `SetYCServiceRequest()`: 设置向量字为0x8000触发RT服务请求
  - `WriteRT()` / `ReadRT()`: RT RAM双字节读写（先低后高，含nop时序间隔）
  - `Ext0Int()`: ⚡ 外部中断0（1553B事务完成），TX时清向量字; RX子地址15时读取遥控指令
  - `AuthorizeTC()`: 遥控指令鉴权
- **硬件接口**: 1553B RT芯片 (RT RAM 0x8000, 向量字0x82D0, 配置寄存器), FPGA (0x7000), 看门狗 (0xBFE0), MIL-STD-1553B总线
- **模块关系**: `Ext0Int`中断TX时清向量字为0x0000; 主循环`WriteRT`写向量字为0x8000; 两个上下文并发写入。
- **缺陷概要**: `VECTOR_WORD_ADDR[0]`/`[1]` — `WriteRT`(主循环)先写低字节再写高字节 vs `Ext0Int`(中断)同时清零两字节，交叉导致向量字非预期值

---

## L2_006 — 卫星OBDH遥测时间回报系统

- **领域**: 卫星遥测时间计算 | **处理器**: 通用 | **代码行数**: 914
- **系统概述**: 卫星星载计算机通过OBDH总线接收SMU基准时间，计算本地时间与基准的偏差并通过遥测帧下发，同时监控时钟漂移趋势。
- **功能模块**:
  - `MainLoop()`: 主循环，周期执行时间同步状态机、漂移检测、遥测帧发送
  - `RunTimeSyncSM()`: 四态时间同步状态机（FREE→SYNC_PENDING→SYNCED↔DRIFT）
  - `MonitorTimeDrift()`: 计算偏差写入环形缓冲
  - `IntOBDH()`: ⚡ OBDH中断，读取数据按子地址分发
  - `SnapshotSystemTime()`: 读取系统时间快照（Time.MS, Time.S）
  - `ProcessTimeSendBack()`: 计算本地与SMU基准的差值写入T25寄存器
  - `ClockISR()`: ⚡ 1ms时钟中断，递增Time.MS/Time.S
  - `PackTelemetryFrame()`: 遥测帧打包
- **硬件接口**: T25寄存器组, OBDH总线接口, 时钟中断控制器, UART, 看门狗, 遥测帧输出地址
- **模块关系**: `ClockISR`每1ms更新Time; `IntOBDH`中断调用`ProcessTimeSendBack→SnapshotSystemTime`读取Time; 两个中断上下文交叉。
- **缺陷概要**: `Time.MS`/`Time.S` — `SnapshotSystemTime`(OBDH中断上下文)先读MS后读S vs `ClockISR`(时钟中断)更新MS并进位S

---

## L2_007 — 卫星NLS载荷UART指令收发系统

- **领域**: 卫星NLS单路UART指令收发 | **处理器**: 通用 | **代码行数**: 915
- **系统概述**: 载荷计算机通过单路UART向NLS模块发送控制指令，同时接收1553B总线上的CMU遥控指令并完成帧解析、应答构造和链路质量评估。
- **功能模块**:
  - `main()`: 主循环，调用AppMain和RunNLSSM
  - `AppMain()`: 处理查询指令，填写snd_sig[2~5]并发送
  - `RunNLSSM()`: NLS协议五态状态机
  - `TrackLinkQuality()`: 滑动窗口链路帧质量评估
  - `SYS_ISRExt6()`: ⚡ 外部中断6（1553B接收完成），调用CmdUnpack
  - `CmdUnpack()`: 在中断上下文解析CMU帧，调用DispatchSignal写snd_sig[2~5]
  - `DispatchSignal()`: 构造NLS应答帧并发送
  - `SendSingleData()`: UART FIFO逐字节发送
- **硬件接口**: 单路UART (NLS通信), 1553B总线接口, 外部中断6, 定时器0, 看门狗
- **模块关系**: 主循环`AppMain`写snd_sig[2~5]后发送; 中断`CmdUnpack→DispatchSignal`也写snd_sig[2~5]后发送; 两个上下文交叉写同一缓冲区。
- **缺陷概要**: `snd_sig[2~5]` — `AppMain`(主循环)写入后发送 vs `DispatchSignal`(中断上下文)写入后发送，帧数据混杂

---

## L2_008 — 卫星SMU遥测采集与电池管理系统

- **领域**: 卫星电源管理遥测采集 | **处理器**: 8051 | **代码行数**: 980
- **系统概述**: 基于8051+外部12位ADC的星载电源管理单元，负责7路电池单体电压和充放电电流的遥测采集、电池状态机管理、安时计量、过压/过放保护及遥测帧下行。
- **功能模块**:
  - `main()`: 主循环，驱动采集、状态机、健康监测、帧打包
  - `TM_Collect()`: 遥测采集，对9路通道ADC采样写入uiAnSmp
  - `Self_Manage()`: 自管理保护，读uiAnSmp做ODP/OCP判断（中断上下文调用）
  - `Timer0_ISR()`: ⚡ 定时器0中断（40ms），更新时间，每秒调用Self_Manage
  - `RunBatSM()`: 电池五态状态机（INIT/CHARGE/DISCH/BALANCE/PROTECT）
  - `AhMeter_Calc()`: 安时计量，充放电电流积分
  - `PackSmuFrame()`: 32字节SMU遥测帧打包
- **硬件接口**: 外部12位ADC (9通道), 定时器0, UART, 看门狗, GPIO
- **模块关系**: 主循环`TM_Collect`逐通道写uiAnSmp; `Timer0_ISR`每秒调用`Self_Manage`读uiAnSmp; 中断在采样过程中读到新旧混合数据。
- **缺陷概要**: `uiAnSmp[9]`/`Time.MS`/`Time.S` — `TM_Collect`(主循环)逐通道写 vs `Self_Manage`(40ms中断)读取，新旧采样混合导致误判

---

## L2_009 — 卫星CAN总线双缓冲遥测系统

- **领域**: 卫星CAN总线遥测双缓冲 | **处理器**: 通用 | **代码行数**: 1001
- **系统概述**: 基于CAN总线的星载遥测数据采集与分发系统，采用双缓冲机制实现快速(7通道)和慢速(70通道)遥测数据的无缝切换与CAN帧应答。
- **功能模块**:
  - `MainLoop()`: 主循环，周期采集、更新双缓冲、CAN状态机
  - `CollectQuickRmt()` / `CollectSlowRmt()`: 快速/慢速遥测ADC采集
  - `RmtData_Rdy()`: 将实时数据复制到双缓冲A区（77个元素）
  - `CanA_RvIntr()`: ⚡ CAN-A接收中断，根据查询类型从A/B区读取数据发送
  - `RunCanBusSM()`: CAN总线五态状态机
  - `PackQuickFrame()` / `PackSlowFrame()`: 遥测数据打包为CAN帧
  - `MonitorCanBus()`: 总线健康检查
- **硬件接口**: CAN-A总线 (收发缓冲/控制/状态/中断), ADC (快速7通道+慢速70通道), 定时器0, UART, 看门狗
- **模块关系**: 主循环采集→`RmtData_Rdy`逐元素复制到A区; CAN中断`CanA_RvIntr`直接从A区读取发送; 中断在复制过程中读到部分更新的数据。
- **缺陷概要**: `Can_QuickRmtA[7]`/`Can_SlowRmtA[70]` — `RmtData_Rdy`(主循环)逐元素复制 vs `CanA_RvIntr`(CAN中断)读取A区，部分更新不一致

---

## L2_010 — 航天载荷相机RS422通信系统

- **领域**: 航天载荷相机RS422帧校验 | **处理器**: 8051 | **代码行数**: 1143
- **系统概述**: 基于8051的航天载荷管理系统，通过RS422接口与相机模块通信，完成指令帧接收、同步字/校验和验证、命令解析应答、工程参数管理及EEPROM备份。
- **功能模块**:
  - `main()`: 主循环，周期调用payload_management和DiagDump
  - `payload_RS422()`: RS422帧处理，验证同步字和校验和，解析命令
  - `RunRS422SM()`: RS422五态通信状态机
  - `RS422_int1()`: ⚡ 外部中断1，从RS422 FIFO读取17字节帧到CAMERA_A_REC_BUF
  - `ParseCameraReply()`: 解析相机应答命令（8种命令码）
  - `BackupEngPara()`: 工程参数备份到EEPROM
  - `MonitorRS422Health()`: 通信健康监测，错误累计超阈值复位接口
- **硬件接口**: RS422 (接收/发送FIFO, 状态寄存器), UART, 定时器1/2, 外部中断1, GPIO, 看门狗, EEPROM
- **模块关系**: `RS422_int1`中断读17字节帧到CAMERA_A_REC_BUF; 主循环`payload_RS422`遍历缓冲区计算校验和并逐字节读取; 中断可能在遍历过程中覆写整个缓冲区。
- **缺陷概要**: `CAMERA_A_REC_BUF[17]` — `payload_RS422`(主循环)遍历计算校验和 vs `RS422_int1`(外部中断1)覆写整个缓冲区，两帧混合撕裂
