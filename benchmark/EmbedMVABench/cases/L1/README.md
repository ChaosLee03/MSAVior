# L1 级案例 — 单函数直接访问

L1 级缺陷特征：共享变量的冲突读写发生在**单个函数内**，主任务函数直接读写共享变量，与 ISR 的读写产生冲突。调用链深度为 1（直接访问）。

共 8 个正例。

---

## L1_001 — ACE 1553B总线内部指令处理系统

- **领域**: 航空电子 | **处理器**: 通用 | **代码行数**: 735
- **系统概述**: 基于DDC ACE 65170芯片的MIL-STD-1553B RT模式总线通信系统，负责接收BC下发的内部指令、遥测帧和状态帧，并通过RS422接口向地面上报总线健康状态。
- **功能模块**:
  - `main()`: 系统主循环，驱动ACE三态状态机（RESET/READY/FAULT），周期喂狗
  - `AceInit()` / `AceHwReset()` / `AceSwConfig()`: ACE芯片初始化、硬件复位、软件配置
  - `fnInternalHandling()`: 主任务读取BUF_INTERNAL双缓冲做一致性比较
  - `fnTelemetryHandling()`: 读取子地址9遥测帧，校验同步字和CRC
  - `fnStatusHandling()`: 读ACE状态寄存器更新总线状态位
  - `fnBusHealthMonitor()`: 周期性CRC错误率计算，更新健康等级（优/良/差/故障）
  - `ace_int0()`: ⚡ 1553B消息接收完成中断，按消息码分发写BUF_INTERNAL
  - `AceSendStatusToGround()`: 经RS422下行状态摘要
  - `CalcFrameCRC()`: 16位异或帧校验
- **硬件接口**: DDC ACE 65170 (1553B协议引擎, 基地址0xD0000000), RS422串行接口 (0xB0000000), 外部看门狗IC (0xBFF0)
- **模块关系**: 主循环在READY态依次调用四个处理函数; `ace_int0` ISR由1553B消息接收完成触发，写入`BUF_INTERNAL`双缓冲; 健康监控通过RS422周期性向地面下行状态摘要。
- **缺陷概要**: `BUF_INTERNAL[0]`/`[1]` — `fnInternalHandling`(主任务)非原子读两字 vs `ace_int0`(ISR)分两步写入

---

## L1_002 — 航天载荷相机供电管理系统

- **领域**: 航天载荷控制 | **处理器**: 8051 | **代码行数**: 840
- **系统概述**: 基于8051单片机的星载相机载荷供电管理系统，负责相机上电时序控制、RS422帧通信、ADC遥测采集、遥测包组装发送及载荷健康监控。
- **功能模块**:
  - `main()`: 主循环，驱动载荷供电状态机（POWEROFF/POWERING/POWERED/ERROR），调度10ms/100ms/1s周期任务
  - `payload_powerup()` / `payload_powerup_down()`: 载荷上电序列和上电完成后使能
  - `timer2_int()`: ⚡ Timer2中断（20ms周期），累加5秒上电计时器并切换相机状态
  - `PeriodicTask10ms/100ms/1s()`: 多频率周期任务
  - `Rs422SendFrame()` / `Rs422ProcRxFrame()`: RS422帧协议收发
  - `TmPktBuild()` / `TmPktSend()`: 遥测包组装和发送
- **硬件接口**: 8051 SFR, 中断掩码寄存器 (0x4010), 相机A控制寄存器组 (0x5000~0x5013), RS422 (0x6000), ADC (0x7000), 看门狗 (0x8000), 遥测存储 (0x9000)
- **模块关系**: Timer2 ISR每20ms累加计时器，5秒到达后切换相机状态; 主任务读取状态判断上电是否完成。
- **缺陷概要**: `FLAG_CAMERA_A_STATE`/`TIME_FIVE_SECOND_CAMERA_A` — `payload_powerup_down`(主任务)读写状态 vs `timer2_int`(ISR)累加计时并修改状态

---

## L1_003 — VIDS高精度时钟自适应频率校正系统

- **领域**: 卫星时钟频率校正 | **处理器**: SPARC | **代码行数**: 877
- **系统概述**: 基于LEON3FT GR712RC处理器的VIDS高精度时钟自适应频率校正系统，通过TC指令设置平均调整周期，时钟中断据此执行精细频率修正，并集成GPS秒脉冲监测和频率漂移告警。
- **功能模块**:
  - `main()`: 主循环，处理TC指令、GPS脉冲丢失检测、频率漂移监控、时钟状态上报
  - `HandelTcBlkAvgAdjust()`: 处理TC设置平均调整周期命令，分两步写入高低字节
  - `Ext2ClockIrq()`: ⚡ 时钟中断ISR（1ms），读高低字节组装16位周期值触发频率修正
  - `UartRxIrq()`: ⚡ UART接收中断，逐字节接收TC指令
  - `GpsPpsIrq()`: ⚡ GPS秒脉冲中断，记录PPS时间戳
  - `FreqDriftMonitor()`: 滑动窗口频率漂移告警
  - `ClkFlagUpdate()`: 时钟状态标志机（IDLE/CALIBRATING/LOCKED/ERROR）
- **硬件接口**: LEON3 AMBA总线, 时钟控制寄存器 (0xA0000000), GPTIMER (0xA0010000), UART (0xA0020000), GPS接收机 (0xA0040000), 看门狗 (0x80000500), PLL 50MHz
- **模块关系**: TC指令更新调整周期高低字节; `Ext2ClockIrq` ISR每1ms读取这两个字节组装16位值决定修正时机。
- **缺陷概要**: `VIDS_AVG_ADJ_PERH`/`PERL` — `HandelTcBlkAvgAdjust`(主任务)分两步写高低字节 vs `Ext2ClockIrq`(ISR)分两步读取并组装

---

## L1_004 — AHB总线错误中断控制与内存洗刷系统

- **领域**: 实时操作系统AHB总线错误中断控制 | **处理器**: SPARC | **代码行数**: 811
- **系统概述**: 基于LEON3/SPARC V8的RTOS级AHB总线单粒子翻转错误处理系统，实现陷阱级错误捕获、错误日志环形缓冲、内存ECC洗刷、中断恢复状态机和系统健康监控。
- **功能模块**:
  - `main()`: 主循环，执行系统任务、内存洗刷、定期状态上报
  - `sysTask()`: 检测`BusErrIntClose`标志后重新使能AHB中断，驱动状态机
  - `AhbStateMachine()`: AHB错误五态状态机（INIT/IDLE/ACTIVE/TRAP_PEND/ERROR）
  - `Trap_Single_Error()`: ⚡ AHB单粒子翻转陷阱处理，读MEC故障信息、记日志、置标志并关中断
  - `ScrubMemTask()` / `ScrubOneBlock()`: 内存ECC洗刷周期任务
  - `AhbStatusReport()`: UART输出错误统计
- **硬件接口**: LEON3 MEC寄存器 (0x80000000+0x90~0x9C), 内存洗刷控制器 (0x80000040), 中断控制器 (0x80000200), 看门狗 (0x80000300), UART (0xA0100000), SRAM洗刷区 (0x40000000)
- **模块关系**: `Trap_Single_Error`置标志并关中断; `sysTask`检测标志后使能中断并清标志; 两者交替操作实现错误处理-恢复循环。
- **缺陷概要**: `BusErrIntClose`/`mec[MEC_AHB_INTCTRL_REG/4]` — `sysTask`(主任务)先使能中断再清标志 vs `Trap_Single_Error`(陷阱)先置标志再关中断，顺序窗口导致新陷阱标志被覆盖

---

## L1_005 — BU-61580 1553B总线共享内存读写系统

- **领域**: 1553B总线共享内存读写 | **处理器**: 8051 | **代码行数**: 757
- **系统概述**: 基于8051单片机和BU-61580 1553B协议芯片的RT模式总线通信系统，负责从4K共享RAM中读写16位数据字，实现命令接收、状态上报和总线健康管理。
- **功能模块**:
  - `main()`: 主循环，驱动总线四态状态机，处理接收数据，周期维护
  - `buRtRdRam()`: 从BU-61580 4K RAM读16位字，两次XBYTE读msb和lsb
  - `buRtWrRam()`: 向BU-61580写16位字
  - `ProcessRxData()`: 从命令栈取数据并通过buRtRdRam读取
  - `extrenINIT0()`: ⚡ 外部中断0 ISR，BC下发新数据触发，分两步写msb和lsb
  - `BuRtCmdDecode()`: 1553B命令字解码
- **硬件接口**: 8051 SFR, BU-61580 1553B芯片 (寄存器区0x0000, 4K RAM 0x0200), 外部中断控制寄存器
- **模块关系**: `extrenINIT0` ISR写msb/lsb并压命令栈; 主任务`ProcessRxData`从栈取命令，`buRtRdRam`读msb/lsb。
- **缺陷概要**: `msb`/`lsb` — `buRtRdRam`(主任务)分两步读 vs `extrenINIT0`(ISR)分两步写，交叉导致16位撕裂

---

## L1_006 — 1553B广播帧接收与遥控处理系统

- **领域**: 1553B广播数据接收与遥控处理 | **处理器**: 通用 | **代码行数**: 723
- **系统概述**: 基于MIL-STD-1553B RT模式的广播帧接收与遥控指令处理系统，负责接收BC广播的遥控数据、校验帧合法性、执行遥控指令并通过RS422上报状态。
- **功能模块**:
  - `main()`: 主循环，驱动遥控接收四态状态机（IDLE/RECEIVING/VALIDATING/EXECUTING）
  - `cgYC()`: 广播数据采集和遥控帧处理，从环形缓冲取数据到guangboGet
  - `rt_1553_isr()`: ⚡ 1553B RT中断ISR，广播帧收到后分两步写guangboGet[0]/[1]
  - `ValidateGuangboFrame()`: 广播帧校验（值域/奇偶/重复帧检测）
  - `HandleRcError()`: 遥控错误分类处理，经RS422上报
- **硬件接口**: 1553B RT控制器 (0xC0000000), TC遥控输出接口 (0xC0020000), RS422 (0xB0000000), 看门狗 (0xBFF0)
- **模块关系**: `rt_1553_isr` ISR分两步写guangboGet并入环形缓冲; 主循环`cgYC`从缓冲取数据到guangboGet。
- **缺陷概要**: `guangboGet[0]`/`[1]` — `cgYC`(主任务)读取 vs `rt_1553_isr`(ISR)分两步写入，中断可能在读取间隔覆写

---

## L1_007 — 伺服电机RDC角度测量与闭环控制系统

- **领域**: 伺服电机RDC角度测量 | **处理器**: 通用 | **代码行数**: 889
- **系统概述**: 基于旋变数字转换器(RDC)的伺服电机双轴角度测量与闭环控制系统，负责RDC采样、角度计算与校准、滑动窗口滤波、电机PWM控制输出、UART指令解码和角度遥测上报。
- **功能模块**:
  - `main()`: 主循环，驱动RDC状态机，计算控制输出
  - `ReadFactAngle()`: 读RDC数据，计算双轴角度并应用三冗余校准，写入FactX/FactY
  - `Par3Choice2()`: 三冗余中值选择校准算法
  - `FilterAngle()`: 8点滑动窗口均值滤波
  - `MotorControlOutput()`: 角度误差到PWM的转换和限幅输出
  - `UartAIsr()`: ⚡ UART接收中断ISR，帧完整后调用DecodeFunction
  - `DecodeFunction()`: UART指令解码（RESET命令直接写FactX=0/FactY=0）
- **硬件接口**: RDC寄存器 (0xE0000000), 电机定时器/PWM (0xE0010000), UART-A (0xE0020000), 中断控制器 (0xE0030000), 看门狗 (0xE0040000)
- **模块关系**: 主循环`ReadFactAngle`计算写入FactX/FactY; `UartAIsr` ISR的RESET命令直接写FactX=0/FactY=0; 两个写入上下文交叉。
- **缺陷概要**: `FactX`/`FactY` — `ReadFactAngle`(主任务)分两步写 vs `UartAIsr→DecodeFunction`(ISR)分两步清零，交叉导致不一致组合

---

## L1_008 — 光学传感器零位偏差标定系统

- **领域**: 光学传感器零位偏差标定 | **处理器**: 通用 | **代码行数**: 880
- **系统概述**: DSP平台上的光学传感器XY零位偏差标定系统，通过UART中断接收传感器数据帧，进行三冗余SRAM存储、统计方差校验、多测站分发，并向地面站上报标定结果。
- **功能模块**:
  - `main()`: 主循环，驱动标定状态机，轮转测站切换
  - `SRAMDataCalc()`: XY偏差标定计算核心，从XYOffset提取浮点偏差值
  - `DealFrame()`: 接收帧处理，提取XY偏差写入三冗余XYOffset
  - `CalStateMachine()`: 标定五态状态机（IDLE/COLLECT/COMPUTE/DONE/ERROR）
  - `c_int01()`: ⚡ UART接收/定时器中断ISR，帧完整后调用DealFrame
  - `StationDataDeal()`: 标定结果分发到3个测站SRAM
  - `U32DataToSRAM()`: 三冗余数据写入
- **硬件接口**: DSP UART (0x01C80000), SRAM控制器 (0x60000000), 三个测站SRAM (0x60010000/20000/30000), 地面站接口 (0x70000000), 定时器 (0x01C40000), 看门狗 (0x01C50000)
- **模块关系**: `c_int01` ISR接收帧后调用`DealFrame`写入XYOffset数组; 主循环`SRAMDataCalc`读取XYOffset计算浮点偏差。
- **缺陷概要**: `XYOffset[0]`/`[2]` — `SRAMDataCalc`(主任务)分两步读X/Y偏差 vs `c_int01→DealFrame`(ISR)批量更新，导致X/Y来自不同帧
