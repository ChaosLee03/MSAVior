# L3 级案例 — 深层调用链/多字段访问

L3 级缺陷特征：共享变量的冲突读写涉及**深层调用链**（3 层以上）或**多字段（≥3）一致性访问**。缺陷隐藏在复杂的函数调用关系中，代码审查难度显著增加。

共 6 个正例。

---

## L3_001 — 卫星GNC时间管理系统

- **领域**: 卫星GNC时间管理 | **处理器**: 通用 | **代码行数**: 1823
- **系统概述**: 卫星导航制导控制（GNC）子系统的星载时间管理软件，负责GPS时间同步、轨道时间计算、Kalman滤波导航和地面站指令交互。
- **功能模块**:
  - `main()`: 主循环，喂看门狗、调度任务、执行导航更新
  - `ScheduleTask()`: 多频率任务调度器
  - `GNC_TimeTask()`: GNC时间任务，调用GetGNCCTime和CalcOrbitTime
  - `GetGNCCTime()`: 批量读取GNC_Time的S/MS/GPT_Old/deltaMS四字段
  - `NavUpdate()`: 导航更新，Kalman滤波+RK4轨道传播
  - `KalmanPredict()` / `KalmanUpdate()`: Kalman一步预测和量测更新
  - `PropagateOrbit()`: RK4四阶龙格库塔轨道传播（二体引力模型）
  - `ClockISR()` / `UpdateTimeISR()`: ⚡ 时钟中断，分步更新GNC_Time四字段
  - `SendTimeISR()`: ⚡ UART中断触发遥测包发送
  - `ProcessGscFrame()`: 地面站指令帧解析
- **硬件接口**: GPT定时器 (0x80000200), 中断控制器 (0x80000100), 看门狗 (0x80000300), UART
- **模块关系**: 主循环通过ScheduleTask调度时间、GPS、Kalman、诊断等任务; `UpdateTimeISR`在1ms中断中分步写GNC_Time四字段; `GetGNCCTime`在主循环无保护批量读取四字段。
- **缺陷概要**: `GNC_Time.S`/`.MS`/`.GPT_Old`/`.deltaMS` — `GetGNCCTime`(主循环)无保护批量读取 vs `UpdateTimeISR`(时钟中断)分步写入，四字段不一致

---

## L3_002 — 星上绝对时间控制系统

- **领域**: 星上绝对时间控制 | **处理器**: 通用 | **代码行数**: 1952
- **系统概述**: 卫星星上绝对时间（ABS Time）管理软件，通过PPS秒脉冲中断和UART遥控指令实现高精度时间同步，并提供漂移监测和诊断遥测。
- **功能模块**:
  - `main()`: 主循环，递增系统节拍、喂看门狗、执行增强时间任务
  - `AppTask_TimeEnhanced()`: 增强时间任务，ABS时间监控、诊断遥测、TC接收
  - `Func_SyncABSTime()`: 将绝对时间参数分步写入abs_time_ctrl结构体
  - `ISR_pps()`: ⚡ PPS秒脉冲中断，读取48位定时器、处理abs_time_ctrl
  - `RunTcReceiver()`: UART遥控帧接收状态机
  - `DriftFilt_Update()`: PPS漂移滑动窗口滤波
  - `RunAbsTimeMonitor()`: 绝对时间监控（PPS有效性、漂移补偿、同步失败告警）
  - `SetAbsTimeHw()`: 三重冗余写入设置绝对时间硬件
- **硬件接口**: PPS定时器 (48位, 0x20F00000), 三重冗余SYSTRI32 (0x20E00000), UART (0x80000300), 中断控制器 (0x80000200), 看门狗 (0x40005C00)
- **模块关系**: 主循环`Func_SyncABSTime`分步写abs_time_ctrl的abshi/abspps/available三字段; `ISR_pps`中断读取并判断available后使用abshi和abspps。
- **缺陷概要**: `abs_time_ctrl[0].abshi`/`.abspps`/`.available` — `Func_SyncABSTime`(主循环)分步写 vs `ISR_pps`(PPS中断)读取，三字段写入未完成时中断读到不一致组合

---

## L3_003 — 工业继电器控制系统

- **领域**: 工业继电器控制 | **处理器**: 8051 | **代码行数**: 1599
- **系统概述**: 基于8051的工业继电器控制软件，通过UART遥控帧接收指令，驱动4路继电器执行高/低模式合闸分闸序列，并提供健康监控和状态遥测。
- **功能模块**:
  - `main()`: 主循环，执行控制任务、监控任务和遥控指令处理
  - `ControlTask()`: 控制主任务，设置继电器参数、接收遥控帧、驱动状态机
  - `GJ_SetZSF_Function()`: 设置继电器操作参数，分四步写入GJ_Struct的Address/Length/Status/Command_is_done
  - `Int_4ms()` / `ZSF_update()`: ⚡ 4ms定时中断，读取GJ_Struct字段驱动继电器动作
  - `PEU_RIGHT()`: ⚡ 功率执行单元脉冲式驱动继电器
  - `RunSysStateMachine()`: 系统四态状态机（IDLE/RUNNING/FAULT/RECOVERY）
  - `RunUartReceiver()`: UART接收状态机
  - `ParseTcFrame()`: 遥控帧解析分发
- **硬件接口**: 继电器控制寄存器A/B (0x8000/0x8001), 状态寄存器 (0x8002), Timer0 (0x8008), 中断使能 (0x8010), UART (0x8020)
- **模块关系**: 主循环`GJ_SetZSF_Function`分四步写入Address/Length/Status/Command_is_done; `Int_4ms/ZSF_update`每4ms中断读取这些字段驱动继电器。
- **缺陷概要**: `GJ_Struct.GJ_Command_is_done`/`.GJ_ZSF_Address`/`.GJ_ZSF_Length`/`.GJ_ZSF_Status` — `GJ_SetZSF_Function`(主循环)分四步写 vs `ZSF_update`(4ms中断)读取，读到不一致字段组合

---

## L3_004 — 卫星星时同步系统

- **领域**: 卫星星时同步 | **处理器**: SPARC | **代码行数**: 1477
- **系统概述**: 基于SPARC/LEON3的卫星星时（恒星时）同步管理软件，通过GPTIMER计数器差值累积计算星时，1553B总线中断接收CMU校准指令进行时间校正，并提供漂移诊断和遥测输出。
- **功能模块**:
  - `main()`: 双任务协作调度主循环（Task1每拍、Task2每5拍、CMU响应每10拍）
  - `LoopTask1()`: 高优先级星时同步任务，调用SynStarTime
  - `LoopTask2()`: 低优先级遥测打包任务
  - `SynStarTime()`: 星时同步核心，读取GPTIMER计算增量并累加StarTime
  - `Pack_PK_Data()`: 生成遥测时间字段
  - `CMU_cmd_respond()`: CMU校准指令响应，直接覆写StarTime
  - `SYS_ISRExt6()` / `CmdUnpack()`: ⚡ 外部中断6（1553B接收完成），解析CMU帧写入timeupdata_int
  - `RunStarTimeSyncSM()`: 星时同步状态机
  - `ComputeStarTimeDrift()`: 漂移计算和诊断
- **硬件接口**: GPTIMER (0x80000300), 中断控制器 (0x80000200), APBUART (0x80000100), 1553B共享RAM (0x40000000), 看门狗 (0x80000500)
- **模块关系**: Task1调用`SynStarTime`读写StarTime/timeupdata_new等; 中断`CmdUnpack`写入timeupdata_int和cmdtimeflag; 多字段读写缺乏原子保护。
- **缺陷概要**: `StarState.StarTime`/`.timeupdata_new`/`.timeupdata_new_NLS` — `SynStarTime`(主循环)读写多字段 vs `CmdUnpack`(Ext6中断)写入timeupdata_int，多字段竞争

---

## L3_005 — 星敏感器姿态四元数处理系统

- **领域**: 星敏感器姿态四元数处理 | **处理器**: 通用 | **代码行数**: 1992
- **系统概述**: 星敏感器（Star Tracker）姿态确定软件，通过UART接收星敏感器帧数据提取方向向量，采用全天球/局部/窗口三种识别模式计算卫星姿态四元数和欧拉角。
- **功能模块**:
  - `main()`: 主循环，喂看门狗、调度模式管理和模式判决
  - `ModeScheduler()` → `StarSensorProcess()` → `ModeInvoke()`: 三层模式调度
  - `WinRecognise()`: 窗口识别模式，四元数→方向余弦矩阵
  - `LocalRecognise()` → `CONSGSTP()` → `SeekIP()` → `JDSIGN()`: 局部识别四层调用链
  - `JDSIGN()`: 星对几何判决，读取mRoughVector.v3[0]/[1]
  - `INT_UART_ISR()` / `ParseStarFrame()`: ⚡ UART中断接收并解析星敏帧，写入mRoughVector.v3[0/1/2]
  - `ModeJudge()`: 模式判决，根据粗捕获向量决定模式切换
  - `Sqrtx()` / `QuatNormalize()` / `Vec3Cross()`: 数学工具函数
- **硬件接口**: UART (0x40001000, 含FIFO), 中断控制器 (0x40002000), 看门狗 (0x50008400)
- **模块关系**: 主循环通过**8层调用链** `main→ModeScheduler→StarSensorProcess→ModeInvoke→LocalRecognise→CONSGSTP→SeekIP→JDSIGN` 读取mRoughVector; UART中断`ParseStarFrame`分步写入v3[0/1/2]。
- **缺陷概要**: `mRoughVector.v3[0]`/`[1]`/`[2]` — `JDSIGN`(主循环, 8层调用链)读v3[0]/[1] vs `ParseStarFrame`(UART中断)分步写v3[0/1/2]，三分量不一致

---

## L3_006 — 卫星姿态计算系统

- **领域**: 卫星姿态计算 | **处理器**: SPARC | **代码行数**: 1429
- **系统概述**: 基于SPARC/LEON3的卫星姿态确定与控制软件，通过外部中断接收星敏感器测量帧更新姿态四元数，主任务读取四元数和角速度进行坐标变换、欧拉角计算和姿态控制输出。
- **功能模块**:
  - `main()`: 15ms周期主循环，模式管理、姿态状态机
  - `ModeManager()` → `ModeJudge_1()` → `CoordTransform()`: 三层调用链
  - `CoordTransform()`: 坐标变换核心，读取Auxi_Result_1的8个成员，归一化四元数、更新旋转矩阵、计算欧拉角、计算控制力矩
  - `ExtInt3_coordinate()`: ⚡ 外部中断3，Shepperd方法旋转矩阵→四元数，写入8个字段
  - `C2QF()`: ⚡ Shepperd方法旋转矩阵到四元数转换
  - `CalcEulerFromQuat()`: 四元数→欧拉角转换
  - `CalcAttCtrl()`: 比例控制律计算控制力矩
  - `RunAttCalcSM()`: 姿态计算三相状态机
  - `OmegaFiltUpdate()`: 角速度滑动窗口低通滤波
- **硬件接口**: SPARC中断控制器 (0x80000200), GPTIMER (0x80000300), 看门狗 (0x80000520), 遥测存储器 (0x60000000), UART (0x80001000)
- **模块关系**: 主循环三层调用链`ModeManager→ModeJudge_1→CoordTransform`分步读取Auxi_Result_1的Q.q[0..3]/omega[0..2]/Startime共8个成员; `ExtInt3_coordinate`在外部中断3中分步写入这8个成员。
- **缺陷概要**: `Auxi_Result_1.Q.q[0..3]`/`.omega[0..2]`/`.Startime`（共8字段）— `CoordTransform`(主循环, 三层调用链)分步读取 vs `ExtInt3_coordinate`(外部中断3)分步写入，四元数/角速度/时间戳来自不同测量帧
