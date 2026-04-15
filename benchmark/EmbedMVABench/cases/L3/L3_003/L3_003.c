/************************************************************
 * GJ继电器控制模块 - 8051工业继电器控制
 * 结构体GJ_Struct维护指令计数、完成标志、地址及长度
 * 主任务写入控制参数，4ms定时中断驱动继电器动作序列
 * K型热电偶温度补偿 + 继电器序列控制
 * 硬件：8051 + 4路继电器 + UART遥控
 * 定时器0产生4ms中断
 * 版本：工业现场版
 ************************************************************/

#include <stdint.h>
#include <string.h>

/************************************************************
 * 宏定义（硬件地址、常量、类型别名）
 ************************************************************/

/* 8051 特殊功能寄存器映射（模拟） */
#define SFR_BASE_ADDR           0x8000U
#define RELAY_CTRL_REG_A        (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x00U))
#define RELAY_CTRL_REG_B        (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x01U))
#define RELAY_STATUS_REG        (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x02U))
#define TIMER0_CTRL_REG         (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x08U))
#define TIMER0_TH               (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x0CU))
#define TIMER0_TL               (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x0DU))
#define INT_ENABLE_REG          (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x10U))
#define INT_FLAG_REG            (*(volatile uint8_t *)(SFR_BASE_ADDR + 0x11U))
#define EA_BIT                  0x80U       /* 全局中断使能位 */
#define ET0_BIT                 0x02U       /* Timer0 中断使能位 */

/* UART 特殊功能寄存器 */
#define UART_BASE_ADDR          (SFR_BASE_ADDR + 0x20U)
#define UART_STATUS_REG         (*(volatile uint8_t *)(UART_BASE_ADDR + 0x00U))
#define UART_TX_REG             (*(volatile uint8_t *)(UART_BASE_ADDR + 0x01U))
#define UART_RX_REG             (*(volatile uint8_t *)(UART_BASE_ADDR + 0x02U))
#define UART_RX_READY_BIT       0x01U       /* 接收就绪位 */
#define UART_TX_READY_BIT       0x02U       /* 发送就绪位 */

/* 继电器控制常量 */
#define ZSF_ON                  0x01U       /* 继电器合闸 */
#define ZSF_OFF                 0x00U       /* 继电器分闸 */
#define ZSF_ADDR_HIGH_MODE      0x28U       /* 高指令模式继电器内部地址 */
#define ZSF_ADDR_LOW_MODE       0x27U       /* 低指令模式继电器内部地址 */
#define ZSF_LENGTH_DEFAULT      3U          /* 默认继电器操作步骤数 */
#define MAX_CMD_COUNT           3U          /* 最大指令计数 */
#define MIN_CMD_COUNT           1U          /* 最小指令计数 */

/* 逻辑值 */
#define TRUE                    1U
#define FALSE                   0U

/* 任务调度常量 */
#define MAIN_LOOP_PERIOD_MS     10U
#define ISR_4MS_PERIOD          4U
#define MAX_RELAY_CHANNELS      4U

/* 健康日志容量 */
#define HEALTH_LOG_CAPACITY     8U

/* 健康评级阈值：近8条日志中故障数超过此值判为 POOR */
#define HEALTH_POOR_THRESHOLD   5U
/* 健康评级阈值：故障数超过此值判为 FAIR */
#define HEALTH_FAIR_THRESHOLD   2U

/* 系统复位延时计数（主循环次数） */
#define SYS_RESET_DELAY_CNT     100U

/* UART 帧固定长度（字节） */
#define UART_FRAME_LEN          12U

/* 状态帧最大缓冲长度 */
#define STATUS_FRAME_MAX_LEN    32U

/* 遥控帧帧头魔数 */
#define TC_FRAME_MAGIC          0xEB90U

/* 继电器序列最大步数 */
#define RELAY_SEQ_MAX_STEPS     4U

/* 序列等待周期（每步完成后等待的4ms周期数） */
#define SEQ_STEP_WAIT_TICKS     2U

/* 故障类型定义 */
#define FAULT_TYPE_ADDR_INVALID 1U          /* 地址非法 */
#define FAULT_TYPE_TIMEOUT      2U          /* 超时 */
#define FAULT_TYPE_HW_ABNORMAL  3U          /* 硬件状态异常 */

/* 类型别名 */
typedef uint8_t  U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef int8_t   S8;
typedef int16_t  S16;
typedef int32_t  S32;

/************************************************************
 * 结构体类型定义
 ************************************************************/

/* 继电器控制字表项（嵌套结构体） */
typedef struct {
    U8  ctrl_addr_on;     /* 合闸控制字地址 */
    U8  ctrl_addr_off;    /* 分闸控制字地址 */
    U8  ctrl_bit_on;      /* 合闸控制位掩码 */
    U8  ctrl_bit_off;     /* 分闸控制位掩码 */
} ZSF_CtrlEntry;

/* 继电器通道状态（嵌套结构体） */
typedef struct {
    U8  channel_id;       /* 通道编号 */
    U8  is_active;        /* 当前激活状态 */
    U8  error_count;      /* 错误计数 */
    U8  reserved;
} ZSF_ChannelStatus;

/* 核心共享结构体：GJ 继电器控制 */
typedef struct {
    volatile U8   GJ_Mode_High_Command_count;  /* 高指令剩余执行次数 */
    volatile U8   GJ_Mode_Low_Command_count;   /* 低指令剩余执行次数 */
    volatile U8   GJ_Command_is_done;          /* 指令完成标志（TRUE/FALSE） */
    volatile U8   GJ_ZSF_Address;             /* 当前继电器内部地址 */
    volatile U8   GJ_ZSF_Length;              /* 当前继电器操作步骤剩余数 */
    volatile U8   GJ_ZSF_Status;             /* 继电器目标状态（ON/OFF） */
    volatile U8   GJ_Error_Flag;             /* 错误标志 */
    volatile U8   GJ_Reserved;              /* 对齐保留 */
    ZSF_ChannelStatus ChannelStat[MAX_RELAY_CHANNELS]; /* 通道状态数组 */
} GJ_ControlStruct;

/* 系统任务描述符 */
typedef struct {
    U8   task_id;
    U8   priority;
    U16  period_ms;
    U32  run_count;
    U32  last_run_tick;
} TaskDescriptor;

/* 遥控指令包 */
typedef struct {
    U8   cmd_type;        /* 指令类型 */
    U8   cmd_subtype;     /* 指令子类型 */
    U8   relay_addr;      /* 继电器地址 */
    U8   relay_action;    /* 动作类型（ON/OFF） */
    U16  cmd_seq;         /* 指令序列号 */
    U16  checksum;        /* 校验和 */
} RelayCmd;

/* 继电器健康日志 */
typedef struct {
    U8  fault_type;       /* 故障类型（1=地址非法, 2=超时, 3=硬件状态异常） */
    U32 occur_tick;       /* 故障发生时的节拍（SysTick4ms） */
    U8  op_addr;          /* 操作地址 */
    U8  count;            /* 该类故障累计次数 */
} RelayHealthLog;

/* 继电器动作序列表项 */
typedef struct {
    U8  seq_id;                        /* 序列ID */
    U8  addr_list[RELAY_SEQ_MAX_STEPS]; /* 各步地址 */
    U8  len_list[RELAY_SEQ_MAX_STEPS];  /* 各步长度 */
    U8  step_count;                    /* 实际步骤数 */
} RelaySeqTable;

/* 系统状态机枚举 */
typedef enum {
    SYS_STATE_IDLE     = 0,   /* 空闲，等待指令 */
    SYS_STATE_RUNNING  = 3,   /* 执行中 */
    SYS_STATE_FAULT    = 5,   /* 故障 */
    SYS_STATE_RECOVERY = 8    /* 恢复中 */
} SysState_e;

/* 遥控帧结构体 */
typedef struct {
    U16  frame_magic;         /* 帧头魔数（0xEB90） */
    U8   cmd_type;            /* 指令类型（0x01~0x05） */
    U8   param[8];            /* 参数数组 */
    U16  crc;                 /* CRC16 校验 */
} TcFrame;

/************************************************************
 * 常量配置表
 ************************************************************/

/* 继电器控制字查找表（地址 0x27 对应索引 0，0x28 对应索�� 1） */
static const ZSF_CtrlEntry ZSF_CtrlTable[2] = {
    {0xA0U, 0xA1U, 0x04U, 0xFBU},  /* 低模式继电器（地址 0x27） */
    {0xB0U, 0xB1U, 0x08U, 0xF7U},  /* 高模式继电器（地址 0x28） */
};

/* Timer0 重装值表（4ms周期，12MHz晶振） */
static const U8 Timer0_TH_Init = 0xF0U;
static const U8 Timer0_TL_Init = 0x60U;

/* 任务优先级配置 */
static const U8 TaskPriorityTable[3] = {1U, 2U, 3U};

/* 最大错误容忍次数 */
static const U8 MAX_ERROR_TOLERANCE = 5U;

/* 预定义继电器动作序列表（3组） */
static const RelaySeqTable RelaySeqCfgTable[3] = {
    /* 序列 0：单步高模式复位 */
    {
        0U,
        {ZSF_ADDR_HIGH_MODE, 0x00U, 0x00U, 0x00U},
        {ZSF_LENGTH_DEFAULT, 0U, 0U, 0U},
        1U
    },
    /* 序列 1：单步低模式复位 */
    {
        1U,
        {ZSF_ADDR_LOW_MODE, 0x00U, 0x00U, 0x00U},
        {ZSF_LENGTH_DEFAULT, 0U, 0U, 0U},
        1U
    },
    /* 序列 2：先高后低双步序列 */
    {
        2U,
        {ZSF_ADDR_HIGH_MODE, ZSF_ADDR_LOW_MODE, 0x00U, 0x00U},
        {ZSF_LENGTH_DEFAULT, ZSF_LENGTH_DEFAULT, 0U, 0U},
        2U
    }
};

/************************************************************
 * 全局变量
 ************************************************************/

/* 核心共享结构体（主任务写，4ms 中断读） */
static volatile GJ_ControlStruct GJ_Struct;

/* 系统节拍计数（4ms 中断递增） */
static volatile U32  SysTick4ms = 0U;

/* 主循环节拍计数 */
static U32  MainTickCount = 0U;

/* 遥控指令缓冲 */
static RelayCmd  CurrentCmd;

/* 系统任务描述符 */
static TaskDescriptor ControlTaskDesc;
static TaskDescriptor MonitorTaskDesc;

/* 继电器操作日志（记录最近8次操作地址） */
static U8  RelayOpLog[8];
static U8  RelayOpLogIdx = 0U;

/* PEU（功率执行单元）右侧计数器 */
static volatile U8  PEU_Right_Count = 0U;

/* 系统初始化完成标志 */
static U8  SysInitFlag = 0U;

/* 中断发生时正在执行的地址快照（调试用） */
static volatile U8  DbgIsrAddrSnap = 0U;

/* 系统状态机当前状态 */
static SysState_e GJMode = SYS_STATE_IDLE;

/* 继电器健康日志环形缓冲 */
static RelayHealthLog g_health_log[HEALTH_LOG_CAPACITY];

/* 健康日志写入索引 */
static U8  g_health_log_idx = 0U;

/* 累计故障次数 */
static U32 ZSFErrCnt = 0U;

/* 当前遥控帧缓冲 */
static TcFrame g_tc_frame;

/* UART 接收缓冲（原始字节） */
static U8  g_uart_rx_buf[32];

/* UART 接收字节计数 */
static U8  g_uart_rx_cnt = 0U;

/* 状态帧发送缓冲 */
static U8  g_uart_tx_buf[STATUS_FRAME_MAX_LEN];

/* 故障恢复等待计数器 */
static U32 g_fault_wait_cnt = 0U;

/* 状态帧发送周期计数 */
static U8  g_status_send_cnt = 0U;

/* 上次 SysTick4ms 记录（用于定时器活性检测） */
static U32 g_last_tick_snap = 0U;

/************************************************************
 * 函数前向声明
 ************************************************************/

void SysInit(void);
void Timer0_Init(void);
void RelayHwInit(void);
void GJ_StructInit(void);
void TaskDescInit(void);

void ControlTask(void);
void MonitorTask(void);
void GJ_SetZSF_Function(void);
void GJ_ClearCmd(void);

void Int_4ms(void);
void ZSF_update(void);
void PEU_RIGHT(void);

U8   CalcCmdChecksum(const RelayCmd *cmd);
void RecordRelayOp(U8 addr);
void DisableGlobalIRQ(void);
void EnableGlobalIRQ(void);
U16  GetSysTick(void);

/* 新增函数前向声明 */
void RunRelaySeqByTable(U8 seq_id);
void LogRelayHealth(U8 fault_type, U8 op_addr);
void EvaluateRelayHealth(void);
U8   ValidateTcFrame(const TcFrame *tf);
void ParseTcFrame(const TcFrame *tf);
void RunUartReceiver(void);
void RunSysStateMachine(void);
void RunDiagMonitor(void);
U16  CalcCRC16(const U8 *data, U8 len);
U8   PackRelayStatusFrame(U8 *buf, U8 max_len);
void SendStatusViaSfr(const U8 *buf, U8 len);

int  main(void);

#ifdef RELAY_VERBOSE
static void RelayDump(void);
#endif

typedef void (*RelayActionFn_t)(void);
static const RelayActionFn_t relay_action_dispatch[] = {
    GJ_SetZSF_Function, GJ_ClearCmd, ZSF_update, PEU_RIGHT
};

/************************************************************
 * main()
 ************************************************************/

int main(void)
{
    /* 系统总初始化 */
    SysInit();

    /* 主循环 */
    while (1)
    {
        MainTickCount++;

        /* 运行继电器控制任务（每次主循环均执行） */
        ControlTask();

        /* 每 5 次主循环执行一次监控任务 */
        if ((MainTickCount % 5U) == 0U)
        {
            MonitorTask();
        }

        /* 处理遥控指令（每 20 次循环检查一次） */
        if ((MainTickCount % 20U) == 0U)
        {
            if (CalcCmdChecksum(&CurrentCmd) == CurrentCmd.checksum)
            {
                if (CurrentCmd.cmd_type == 0x02U)
                {
                    /* 高模式指令 */
                    GJ_Struct.GJ_Mode_High_Command_count = MAX_CMD_COUNT;
                }
                else if (CurrentCmd.cmd_type == 0x03U)
                {
                    /* 低模式指令 */
                    GJ_Struct.GJ_Mode_Low_Command_count = MAX_CMD_COUNT;
                }
            }
        }
    }

    return 0;
}

/************************************************************
 * 初始化函数群
 ************************************************************/

/**
 * @brief 系统总初始化入口
 * 配置IO/Timer/UART/继电器硬件
 */
void SysInit(void)
{
    U8 i;

    GJ_StructInit();
    RelayHwInit();
    Timer0_Init();
    TaskDescInit();

    /* 初始化指令缓冲 */
    CurrentCmd.cmd_type    = 0U;
    CurrentCmd.cmd_subtype = 0U;
    CurrentCmd.relay_addr  = 0U;
    CurrentCmd.relay_action = 0U;
    CurrentCmd.cmd_seq     = 0U;
    CurrentCmd.checksum    = 0U;

    /* 初始化操作日志 */
    RelayOpLogIdx = 0U;
    for (i = 0U; i < 8U; i++)
    {
        RelayOpLog[i] = 0U;
    }

    /* 初始化系统状态机 */
    GJMode     = SYS_STATE_IDLE;
    ZSFErrCnt     = 0U;
    g_fault_wait_cnt = 0U;
    g_status_send_cnt = 0U;
    g_last_tick_snap  = 0U;

    /* 初始化健康日志 */
    g_health_log_idx = 0U;
    for (i = 0U; i < HEALTH_LOG_CAPACITY; i++)
    {
        g_health_log[i].fault_type = 0U;
        g_health_log[i].occur_tick = 0U;
        g_health_log[i].op_addr    = 0U;
        g_health_log[i].count      = 0U;
    }

    /* 初始化 UART 接收缓冲 */
    g_uart_rx_cnt = 0U;
    for (i = 0U; i < 32U; i++)
    {
        g_uart_rx_buf[i] = 0U;
    }

    /* 初始化遥控帧 */
    g_tc_frame.frame_magic = 0U;
    g_tc_frame.cmd_type    = 0U;
    g_tc_frame.crc         = 0U;
    for (i = 0U; i < 8U; i++)
    {
        g_tc_frame.param[i] = 0U;
    }

    /* 初始化状态发送缓冲 */
    for (i = 0U; i < STATUS_FRAME_MAX_LEN; i++)
    {
        g_uart_tx_buf[i] = 0U;
    }

    SysInitFlag = 1U;
}

/*
 * 函数：GJ_StructInit
 * 功能：初始化 GJ 控制结构体，所有字段清零
 */
void GJ_StructInit(void)
{
    U8 i;

    GJ_Struct.GJ_Mode_High_Command_count = 0U;
    GJ_Struct.GJ_Mode_Low_Command_count  = 0U;
    GJ_Struct.GJ_Command_is_done         = TRUE;
    GJ_Struct.GJ_ZSF_Address             = 0U;
    GJ_Struct.GJ_ZSF_Length              = 0U;
    GJ_Struct.GJ_ZSF_Status              = ZSF_OFF;
    GJ_Struct.GJ_Error_Flag              = 0U;
    GJ_Struct.GJ_Reserved                = 0U;

    for (i = 0U; i < MAX_RELAY_CHANNELS; i++)
    {
        GJ_Struct.ChannelStat[i].channel_id  = i;
        GJ_Struct.ChannelStat[i].is_active   = 0U;
        GJ_Struct.ChannelStat[i].error_count = 0U;
        GJ_Struct.ChannelStat[i].reserved    = 0U;
    }
}

/*
 * 函数：RelayHwInit
 * 功能：初始化继电器硬件控制寄存器
 */
void RelayHwInit(void)
{
    /* 关闭所有继电器通道 */
    RELAY_CTRL_REG_A = 0x00U;
    RELAY_CTRL_REG_B = 0x00U;
    RELAY_STATUS_REG = 0x00U;
}

/*
 * 函数：Timer0_Init
 * 功能：初始化 Timer0 产生 4ms 定时中断
 */
void Timer0_Init(void)
{
    /* 停止 Timer0 */
    TIMER0_CTRL_REG = 0x00U;
    /* 设置重装值 */
    TIMER0_TH = Timer0_TH_Init;
    TIMER0_TL = Timer0_TL_Init;
    /* 使能 Timer0 中断 */
    INT_ENABLE_REG = EA_BIT | ET0_BIT;
    /* 启动 Timer0 */
    TIMER0_CTRL_REG = 0x01U;
}

/*
 * 函数：TaskDescInit
 * 功能：初始化任务描述符
 */
void TaskDescInit(void)
{
    ControlTaskDesc.task_id      = 0U;
    ControlTaskDesc.priority     = TaskPriorityTable[0];
    ControlTaskDesc.period_ms    = MAIN_LOOP_PERIOD_MS;
    ControlTaskDesc.run_count    = 0U;
    ControlTaskDesc.last_run_tick = 0U;

    MonitorTaskDesc.task_id      = 1U;
    MonitorTaskDesc.priority     = TaskPriorityTable[1];
    MonitorTaskDesc.period_ms    = 50U;
    MonitorTaskDesc.run_count    = 0U;
    MonitorTaskDesc.last_run_tick = 0U;
}

/************************************************************
 * 调用链层1函数
 ************************************************************/

/**
 * @brief 继电器控制主任务
 * 调度并执行继电器指令序列
 */
void ControlTask(void)
{
    U8  tx_len;

    ControlTaskDesc.run_count++;
    ControlTaskDesc.last_run_tick = MainTickCount;

    /* 调用核心指令处理函数（层2） */
    GJ_SetZSF_Function();

    /* 记录通道状态 */
    if (GJ_Struct.GJ_Command_is_done == TRUE)
    {
        GJ_Struct.ChannelStat[0].is_active = 0U;
    }
    else
    {
        GJ_Struct.ChannelStat[0].is_active = 1U;
    }

    /* 运行 UART 接收状态机，处理遥控帧 */
    RunUartReceiver();

    /* 运行系统状态机 */
    RunSysStateMachine();

    /* 每 50 次循环打包并发送一次状态帧 */
    g_status_send_cnt++;
    if (g_status_send_cnt >= 50U)
    {
        g_status_send_cnt = 0U;
        tx_len = PackRelayStatusFrame(g_uart_tx_buf, STATUS_FRAME_MAX_LEN);
        if (tx_len > 0U)
        {
            SendStatusViaSfr(g_uart_tx_buf, tx_len);
        }
    }
}

/**
 * @brief 系统监控任务
 * 检查继电器状态和错误
 */
void MonitorTask(void)
{
    U8 i;
    MonitorTaskDesc.run_count++;

    /* 检查通道错误 */
    for (i = 0U; i < MAX_RELAY_CHANNELS; i++)
    {
        if (GJ_Struct.ChannelStat[i].error_count >= MAX_ERROR_TOLERANCE)
        {
            GJ_Struct.GJ_Error_Flag = 1U;
        }
    }

    /* 检查指令超时（超过500个主循环周期未完成则复位） */
    if ((GJ_Struct.GJ_Command_is_done == FALSE) &&
        ((MainTickCount - ControlTaskDesc.last_run_tick) > 500U))
    {
        GJ_ClearCmd();
    }

    /* 运行诊断监控 */
    RunDiagMonitor();
}

/************************************************************
 * 调用链层2函数
 ************************************************************/

/**
 * @brief 设置继电器操作参数
 * 根据高/低指令计数写入 GJ_Struct 各字段
 * 调用链: main → ControlTask → GJ_SetZSF_Function
 */
void GJ_SetZSF_Function(void)
{
    /* 检查高指令计数范围和完成标志 */
    if ((GJ_Struct.GJ_Mode_High_Command_count >= MIN_CMD_COUNT) &&
        (GJ_Struct.GJ_Mode_High_Command_count <= MAX_CMD_COUNT) &&
        (GJ_Struct.GJ_Command_is_done == TRUE))
    {
        /* 递减高指令计数 */
        GJ_Struct.GJ_Mode_High_Command_count--;

        /* 设置继电器参数（先写地址和长度） */
        GJ_Struct.GJ_ZSF_Address = ZSF_ADDR_HIGH_MODE;  /* 第1步：写地址 */
        GJ_Struct.GJ_ZSF_Length  = ZSF_LENGTH_DEFAULT;   /* 第2步：写长度 */
        GJ_Struct.GJ_ZSF_Status  = ZSF_OFF;              /* 第3步：写状态 */

        /* 置位指令执行标志 */
        GJ_Struct.GJ_Command_is_done = FALSE;             /* 第4步：写完成标志 */

        RecordRelayOp(ZSF_ADDR_HIGH_MODE);
    }
    /* 检查低指令计数范围和完成标志 */
    else if ((GJ_Struct.GJ_Mode_Low_Command_count >= MIN_CMD_COUNT) &&
             (GJ_Struct.GJ_Mode_Low_Command_count <= MAX_CMD_COUNT) &&
             (GJ_Struct.GJ_Command_is_done == TRUE))
    {
        /* 递减低指令计数 */
        GJ_Struct.GJ_Mode_Low_Command_count--;

        /* 设置继电器参数（先写地址和长度） */
        GJ_Struct.GJ_ZSF_Address = ZSF_ADDR_LOW_MODE;   /* 第1步：写地址 */
        GJ_Struct.GJ_ZSF_Length  = ZSF_LENGTH_DEFAULT;   /* 第2步：写长度 */
        GJ_Struct.GJ_ZSF_Status  = ZSF_ON;               /* 第3步：写状态 */

        /* 置位指令执行标志 */
        GJ_Struct.GJ_Command_is_done = FALSE;             /* 第4步：写完成标志 */

        RecordRelayOp(ZSF_ADDR_LOW_MODE);
    }
    else
    {
        /* 无有效指令，空操作 */
    }
}

/*
 * 函数：GJ_ClearCmd
 * 功能：清除当前指令，复位 GJ_Struct 到空闲状态
 */
void GJ_ClearCmd(void)
{
    GJ_Struct.GJ_ZSF_Address     = 0x00U;
    GJ_Struct.GJ_ZSF_Length      = 0x00U;
    GJ_Struct.GJ_Command_is_done = TRUE;
    GJ_Struct.GJ_ZSF_Status      = ZSF_OFF;
}

/************************************************************
 * 调用链层3函数（继电器控制辅助处理）
 ************************************************************/

/**
 * @brief 4ms定时器中断入口 irq_handler
 * 每4ms触发，驱动继电器动作序列推进
 */
void Int_4ms(void)
{
    /* 清除中断标志 */
    INT_FLAG_REG = ET0_BIT;

    /* 重装 Timer0 */
    TIMER0_TH = Timer0_TH_Init;
    TIMER0_TL = Timer0_TL_Init;

    /* 递增 4ms 节拍计数 */
    SysTick4ms++;

    /* 驱动继电器更新（从中断读取共享结构体） */
    ZSF_update();
}

/**
 * @brief 中断上下文继电器更新
 * 根据 GJ_Struct 多个字段驱动继电器动作
 * 调用链: Int_4ms → ZSF_update
 */
void ZSF_update(void)
{
    U8 addr;
    U8 len;
    U8 idx;

    /* 记录中断发生时的地址快照（调试用） */
    DbgIsrAddrSnap = GJ_Struct.GJ_ZSF_Address;

    /* 检查指令未完成标志（读取 GJ_Command_is_done） */
    if (GJ_Struct.GJ_Command_is_done == FALSE)
    {
        /* 读取地址和长度字段（与主任务写入存在窗口） */
        addr = GJ_Struct.GJ_ZSF_Address;          /* 读取继电器地址 */
        len  = GJ_Struct.GJ_ZSF_Length;           /* 读取继电器长度 */

        /* 验证地址合法性 */
        if (((addr == ZSF_ADDR_LOW_MODE) || (addr == ZSF_ADDR_HIGH_MODE)) &&
            (len != 0U))
        {
            /* 查表获取控制字 */
            idx = addr - ZSF_ADDR_LOW_MODE;
            if (idx < 2U)
            {
                /* 执行继电器动作：写控制寄存器 */
                if (GJ_Struct.GJ_ZSF_Status == ZSF_ON)
                {
                    RELAY_CTRL_REG_A |= ZSF_CtrlTable[idx].ctrl_bit_on;
                }
                else
                {
                    RELAY_CTRL_REG_A &= ZSF_CtrlTable[idx].ctrl_bit_off;
                }
            }

            /* 递减步骤计数 */
            GJ_Struct.GJ_ZSF_Length--;

            /* 驱动功率执行单元 */
            PEU_RIGHT();

            /* 更新通道状态 */
            if (idx < MAX_RELAY_CHANNELS)
            {
                GJ_Struct.ChannelStat[idx].is_active = 1U;
            }
        }
        else
        {
            /* 地址无效或长度为0：复位继电器参数 */
            GJ_Struct.GJ_ZSF_Address     = 0x00U;
            GJ_Struct.GJ_ZSF_Length      = 0x00U;
            GJ_Struct.GJ_Command_is_done = TRUE;
        }
    }
}

/*
 * 函数：PEU_RIGHT
 * 功能：功率执行单元右侧触发（驱动继电器执行一步动作）
 */
void PEU_RIGHT(void)
{
    U8 gjTmpBuf;
    /* 设置继电器触发位（实际硬件操作） */
    RELAY_STATUS_REG |= 0x01U;
    PEU_Right_Count++;
    (void)gjTmpBuf;

    /* 短暂延时等待硬件响应（空循环模拟） */
    {
        volatile U8 dly;
        for (dly = 0U; dly < 10U; dly++)
        {
            /* 等待硬件响应 */
        }
    }

    /* 清除触发位 */
    RELAY_STATUS_REG &= (U8)(~0x01U);
}

/************************************************************
 * 中断服务例程（辅助处理函数）
 ************************************************************/

/************************************************************
 * 辅助函数
 ************************************************************/

/*
 * 函数：CalcCmdChecksum
 * 功能：计算遥控指令包的简单校验和
 */
U8 CalcCmdChecksum(const RelayCmd *cmd)
{
    U8 sum = 0U;
    if (cmd == 0)
    {
        return 0U;
    }
    sum  = cmd->cmd_type;
    sum += cmd->cmd_subtype;
    sum += cmd->relay_addr;
    sum += cmd->relay_action;
    sum += (U8)(cmd->cmd_seq & 0xFFU);
    sum += (U8)(cmd->cmd_seq >> 8U);
    return sum;
}

/*
 * 函数：RecordRelayOp
 * 功能：记录继电器操作地址到循环日志
 */
void RecordRelayOp(U8 addr)
{
    RelayOpLog[RelayOpLogIdx] = addr;
    RelayOpLogIdx = (U8)((RelayOpLogIdx + 1U) % 8U);
}

/*
 * 函数：DisableGlobalIRQ
 * 功能：关闭全局中断（用于原子保护）
 */
void DisableGlobalIRQ(void)
{
    INT_ENABLE_REG &= (U8)(~EA_BIT);
}

/*
 * 函数：EnableGlobalIRQ
 * 功能：重新开启全局中断
 */
void EnableGlobalIRQ(void)
{
    INT_ENABLE_REG |= EA_BIT;
}

/*
 * 函数：GetSysTick
 * 功能：返回当前 4ms 系统节拍计数（低16位）
 */
U16 GetSysTick(void)
{
    return (U16)(SysTick4ms & 0xFFFFU);
}

/************************************************************
 * 继电器序列驱动
 ************************************************************/

/*
 * 函数：RunRelaySeqByTable
 * 功能：按序列表驱动继电器多步动作，最多 RELAY_SEQ_MAX_STEPS 步。
 *       每步设置 GJ_Struct 相关字段后忙等待完成，再等待 SEQ_STEP_WAIT_TICKS
 *       个 4ms 周期后推进下一步。用于初始化复位序列。
 */
void RunRelaySeqByTable(U8 seq_id)
{
    U8  step;
    U8  step_addr;
    U8  step_len;
    U32 wait_start;
    U32 busy_start;
    U8  found;
    U8  tbl_idx;

    /* 在序列配置表中查找匹配的 seq_id */
    found   = 0U;
    tbl_idx = 0U;
    {
        U8 k;
        for (k = 0U; k < 3U; k++)
        {
            if (RelaySeqCfgTable[k].seq_id == seq_id)
            {
                tbl_idx = k;
                found   = 1U;
                break;
            }
        }
    }

    if (found == 0U)
    {
        /* 序列 ID 不存在，直接返回 */
        return;
    }

    /* 遍历序列中的每一步 */
    for (step = 0U; step < RelaySeqCfgTable[tbl_idx].step_count; step++)
    {
        step_addr = RelaySeqCfgTable[tbl_idx].addr_list[step];
        step_len  = RelaySeqCfgTable[tbl_idx].len_list[step];

        /* 等待当前指令完成（忙等待，防止乱序） */
        busy_start = SysTick4ms;
        while ((GJ_Struct.GJ_Command_is_done == FALSE) &&
               ((SysTick4ms - busy_start) < 50U))
        {
            /* 等待中断完成当前步骤 */
        }

        /* 写入下一步的继电器参数 */
        GJ_Struct.GJ_ZSF_Address     = step_addr;
        GJ_Struct.GJ_ZSF_Length      = step_len;
        GJ_Struct.GJ_ZSF_Status      = ZSF_OFF;
        GJ_Struct.GJ_Command_is_done = FALSE;

        RecordRelayOp(step_addr);

        /* 等待步骤完成后再延时 SEQ_STEP_WAIT_TICKS 个节拍 */
        wait_start = SysTick4ms;
        while ((SysTick4ms - wait_start) < SEQ_STEP_WAIT_TICKS)
        {
            /* 等待 4ms 周期 */
        }
    }
}

/************************************************************
 * 健康日志
 ************************************************************/

/*
 * 函数：LogRelayHealth
 * 功能：记录继电器健康事件到环形缓冲 g_health_log。
 *       同时递增 ZSFErrCnt。
 *       fault_type: 1=地址非法, 2=超时, 3=硬件状态异常
 */
void LogRelayHealth(U8 fault_type, U8 op_addr)
{
    U8 idx;

    idx = g_health_log_idx;

    g_health_log[idx].fault_type = fault_type;
    g_health_log[idx].occur_tick = SysTick4ms;
    g_health_log[idx].op_addr    = op_addr;

    /* 若此槽已有同类故障则累计，否则重置计数 */
    if (g_health_log[idx].fault_type == fault_type)
    {
        if (g_health_log[idx].count < 0xFFU)
        {
            g_health_log[idx].count++;
        }
    }
    else
    {
        g_health_log[idx].count = 1U;
    }

    /* 推进环形索引 */
    g_health_log_idx = (U8)((idx + 1U) % HEALTH_LOG_CAPACITY);

    /* 累计全局故障计数 */
    if (ZSFErrCnt < 0xFFFFFFFFUL)
    {
        ZSFErrCnt++;
    }
}

/************************************************************
 * 健康评估
 ************************************************************/

/*
 * 函数：EvaluateRelayHealth
 * 功能：统计最近 HEALTH_LOG_CAPACITY 条日志中的有效故障条目数。
 *       根据数量评为 GOOD / FAIR / POOR。
 *       POOR 时置 GJ_Struct.GJ_Error_Flag = 1。
 */
void EvaluateRelayHealth(void)
{
    U8  i;
    U8  fault_count;
    U8  rating;   /* 0=GOOD, 1=FAIR, 2=POOR */

    fault_count = 0U;

    for (i = 0U; i < HEALTH_LOG_CAPACITY; i++)
    {
        if (g_health_log[i].fault_type != 0U)
        {
            fault_count++;
        }
    }

    if (fault_count > HEALTH_POOR_THRESHOLD)
    {
        rating = 2U;  /* POOR */
    }
    else if (fault_count > HEALTH_FAIR_THRESHOLD)
    {
        rating = 1U;  /* FAIR */
    }
    else
    {
        rating = 0U;  /* GOOD */
    }

    /* POOR 等级时标记错误 */
    if (rating == 2U)
    {
        GJ_Struct.GJ_Error_Flag = 1U;
    }

    /* 将评级写入保留字段供外部读取（0=GOOD,1=FAIR,2=POOR） */
    GJ_Struct.GJ_Reserved = rating;
}

/************************************************************
 * 遥控帧验证与解析
 ************************************************************/

/*
 * 函数：ValidateTcFrame
 * 功能：验证遥控帧有效性。
 *       检查帧头魔数（0xEB90）、指令类型范围（0x01~0x05）、CRC16 校验。
 *       返回 1 表示有效，0 表示无效。
 */
U8 ValidateTcFrame(const TcFrame *tf)
{
    U16 calc_crc;
    U8  crc_buf[11];
    U8  i;

    if (tf == 0)
    {
        return 0U;
    }

    /* 帧头魔数检查 */
    if (tf->frame_magic != TC_FRAME_MAGIC)
    {
        return 0U;
    }

    /* 指令类型范围检查 */
    if ((tf->cmd_type < 0x01U) || (tf->cmd_type > 0x05U))
    {
        return 0U;
    }

    /* 构造 CRC 计算缓冲（帧头2字节 + cmd_type 1字节 + param 8字节） */
    crc_buf[0] = (U8)(tf->frame_magic >> 8U);
    crc_buf[1] = (U8)(tf->frame_magic & 0xFFU);
    crc_buf[2] = tf->cmd_type;
    for (i = 0U; i < 8U; i++)
    {
        crc_buf[3U + i] = tf->param[i];
    }

    calc_crc = CalcCRC16(crc_buf, 11U);

    if (calc_crc != tf->crc)
    {
        return 0U;
    }

    return 1U;
}

/*
 * 函数：ParseTcFrame
 * 功能：解析遥控帧并按指令类型分发执行。
 *       0x01: 软件复位（调用 GJ_ClearCmd）
 *       0x02: 高模式继电器触发（设置 GJ_Mode_High_Command_count）
 *       0x03: 低模式继电器触发（设置 GJ_Mode_Low_Command_count）
 *       0x04: 查询状态（写 g_uart_tx_buf）
 *       0x05: 健康评估请求（调用 EvaluateRelayHealth）
 */
void ParseTcFrame(const TcFrame *tf)
{
    U8 tx_len;

    if (tf == 0)
    {
        return;
    }

    switch (tf->cmd_type)
    {
        case 0x01U:
            /* 软件复位：清除当前指令状态 */
            GJ_ClearCmd();
            GJ_Struct.GJ_Mode_High_Command_count = 0U;
            GJ_Struct.GJ_Mode_Low_Command_count  = 0U;
            GJMode = SYS_STATE_IDLE;
            break;

        case 0x02U:
            /* 高模式继电器触发 */
            if (tf->param[0] >= MIN_CMD_COUNT && tf->param[0] <= MAX_CMD_COUNT)
            {
                GJ_Struct.GJ_Mode_High_Command_count = tf->param[0];
            }
            else
            {
                GJ_Struct.GJ_Mode_High_Command_count = MIN_CMD_COUNT;
            }
            break;

        case 0x03U:
            /* 低模式继电器触发 */
            if (tf->param[0] >= MIN_CMD_COUNT && tf->param[0] <= MAX_CMD_COUNT)
            {
                GJ_Struct.GJ_Mode_Low_Command_count = tf->param[0];
            }
            else
            {
                GJ_Struct.GJ_Mode_Low_Command_count = MIN_CMD_COUNT;
            }
            break;

        case 0x04U:
            /* 查询状态：打包当前状态到 g_uart_tx_buf */
            tx_len = PackRelayStatusFrame(g_uart_tx_buf, STATUS_FRAME_MAX_LEN);
            if (tx_len > 0U)
            {
                SendStatusViaSfr(g_uart_tx_buf, tx_len);
            }
            break;

        case 0x05U:
            /* 健康评估请求 */
            EvaluateRelayHealth();
            break;

        default:
            /* 未知指令类型，忽略 */
            break;
    }
}

/************************************************************
 * UART接收状态机
 ************************************************************/

/*
 * 函数：RunUartReceiver
 * 功能：UART 接收状态机。检查 UART 状态寄存器的 RX_READY 位，
 *       逐字节读取并存入 g_uart_rx_buf。
 *       当累计接收到 UART_FRAME_LEN（12）字节时，拷贝到 g_tc_frame，
 *       并调用 ValidateTcFrame → ParseTcFrame 处理帧。
 */
void RunUartReceiver(void)
{
    U8  rx_byte;
    U8 *dst;
    U8  i;

    /* 检查 UART 接收就绪位 */
    if ((UART_STATUS_REG & UART_RX_READY_BIT) != 0U)
    {
        /* 读取一字节 */
        rx_byte = UART_RX_REG;

        /* 防溢出 */
        if (g_uart_rx_cnt < 32U)
        {
            g_uart_rx_buf[g_uart_rx_cnt] = rx_byte;
            g_uart_rx_cnt++;
        }
        else
        {
            /* 缓冲满，丢弃并复位计数 */
            g_uart_rx_cnt = 0U;
        }
    }

    /* 当接收到完整帧（固定 12 字节）时进行处理 */
    if (g_uart_rx_cnt >= UART_FRAME_LEN)
    {
        /* 将原始缓冲解析到 TcFrame 结构 */
        dst = (U8 *)&g_tc_frame;

        /* 帧头（2字节，大端） */
        g_tc_frame.frame_magic = (U16)(((U16)g_uart_rx_buf[0] << 8U) |
                                        (U16)g_uart_rx_buf[1]);
        /* 指令类型（1字节） */
        g_tc_frame.cmd_type = g_uart_rx_buf[2];

        /* 参数（8字节） */
        for (i = 0U; i < 8U; i++)
        {
            g_tc_frame.param[i] = g_uart_rx_buf[3U + i];
        }

        /* CRC（2字节，大端） */
        g_tc_frame.crc = (U16)(((U16)g_uart_rx_buf[11] << 8U) |
                                 (U16)g_uart_rx_buf[10]);

        /* 消除 dst 未使用警告（dst 仅作占位，不通过指针赋值，防止未对齐） */
        (void)dst;

        /* 验证并解析帧 */
        if (ValidateTcFrame(&g_tc_frame) != 0U)
        {
            ParseTcFrame(&g_tc_frame);
        }

        /* 复位接收缓冲 */
        g_uart_rx_cnt = 0U;
    }
}

/************************************************************
 * 系统状态机
 ************************************************************/

/*
 * 函数：RunSysStateMachine
 * 功能：驱动系统状态机四态跳转。
 *       IDLE    → RUNNING：高或低指令计数 > 0 时转入执行态
 *       RUNNING → IDLE   ：指令完成且无错误标志时转回空闲
 *       RUNNING → FAULT  ：出现错误标志时转入故障态
 *       FAULT   → RECOVERY：等待 SYS_RESET_DELAY_CNT 次主循环后进入恢复态
 *       RECOVERY→ IDLE   ：调用 GJ_StructInit 重新初始化后转回空闲
 */
void RunSysStateMachine(void)
{
    switch (GJMode)
    {
        case SYS_STATE_IDLE:
            /* 等待有效指令到来 */
            if ((GJ_Struct.GJ_Mode_High_Command_count > 0U) ||
                (GJ_Struct.GJ_Mode_Low_Command_count  > 0U))
            {
                GJMode = SYS_STATE_RUNNING;
                g_fault_wait_cnt = 0U;
            }
            break;

        case SYS_STATE_RUNNING:
            /* 检查错误标志 */
            if (GJ_Struct.GJ_Error_Flag != 0U)
            {
                GJMode = SYS_STATE_FAULT;
                g_fault_wait_cnt = 0U;
                LogRelayHealth(FAULT_TYPE_HW_ABNORMAL, GJ_Struct.GJ_ZSF_Address);
                break;
            }

            /* 指令执行完毕（完成标志为 TRUE 且计数归零）则回到空闲 */
            if ((GJ_Struct.GJ_Command_is_done == TRUE) &&
                (GJ_Struct.GJ_Mode_High_Command_count == 0U) &&
                (GJ_Struct.GJ_Mode_Low_Command_count  == 0U))
            {
                GJMode = SYS_STATE_IDLE;
            }
            break;

        case SYS_STATE_FAULT:
            /* 等待 SYS_RESET_DELAY_CNT 次主循环后进入恢复 */
            g_fault_wait_cnt++;
            if (g_fault_wait_cnt >= SYS_RESET_DELAY_CNT)
            {
                GJMode = SYS_STATE_RECOVERY;
                g_fault_wait_cnt = 0U;
            }
            break;

        case SYS_STATE_RECOVERY:
            /* 重新初始化 GJ 控制结构体，清除错误标志 */
            GJ_StructInit();
            GJ_Struct.GJ_Error_Flag = 0U;
            GJMode = SYS_STATE_IDLE;
            break;

        default:
            GJMode = SYS_STATE_IDLE;
            break;
    }
}

/************************************************************
 * 诊断监控
 ************************************************************/

/*
 * 函数：RunDiagMonitor
 * 功能：诊断监控，在 MonitorTask 中调用。
 *       1. 检查 SysTick4ms 是否有正常增长（定时器是否运行）
 *       2. 检查 GJ_Error_Flag，若置位则记录健康日志
 *       3. 检查 PEU_Right_Count 超限告警（阈值 200）
 *       4. 调用 EvaluateRelayHealth 更新健康评级
 */
void RunDiagMonitor(void)
{
    U32 cur_tick;
    U32 tick_delta;

    /* 获取当前节拍 */
    cur_tick = SysTick4ms;

    /* 检查定时器活性：与上次记录相比是否增长 */
    if (cur_tick >= g_last_tick_snap)
    {
        tick_delta = cur_tick - g_last_tick_snap;
    }
    else
    {
        /* 节拍回卷 */
        tick_delta = (0xFFFFFFFFUL - g_last_tick_snap) + cur_tick + 1UL;
    }

    if (tick_delta == 0U)
    {
        /* 定时器停止，记录超时故障 */
        LogRelayHealth(FAULT_TYPE_TIMEOUT, GJ_Struct.GJ_ZSF_Address);
    }

    /* 更新节拍快照 */
    g_last_tick_snap = cur_tick;

    /* 检查错误标志 */
    if (GJ_Struct.GJ_Error_Flag != 0U)
    {
        LogRelayHealth(FAULT_TYPE_HW_ABNORMAL, GJ_Struct.GJ_ZSF_Address);
    }

    /* PEU 计数超限告警（超过 200 次认为过于频繁） */
    if (PEU_Right_Count > 200U)
    {
        LogRelayHealth(FAULT_TYPE_HW_ABNORMAL, GJ_Struct.GJ_ZSF_Address);
        /* 复位计数避免重复告警 */
        PEU_Right_Count = 0U;
    }

    /* 更新健康评级 */
    EvaluateRelayHealth();
}

/************************************************************
 * CRC16计算
 ************************************************************/

/*
 * 函数：CalcCRC16
 * 功能：计算数据缓冲区的 CRC16 校验值（多项式 0x8005，初始值 0xFFFF）。
 *       8051 风格：使用8位循环，不使用查表法，避免大型 ROM 常量表。
 */
U16 CalcCRC16(const U8 *data, U8 len)
{
    U16 crc;
    U8  i;
    U8  j;
    U8  byte_val;

    if (data == 0)
    {
        return 0U;
    }

    crc = 0xFFFFU;

    for (i = 0U; i < len; i++)
    {
        byte_val = data[i];
        crc ^= (U16)((U16)byte_val << 8U);

        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (U16)((crc << 1U) ^ 0x8005U);
            }
            else
            {
                crc = (U16)(crc << 1U);
            }
        }
    }

    return crc;
}

/************************************************************
 * 状态帧打包与发送
 ************************************************************/

/*
 * 函数：PackRelayStatusFrame
 * 功能：将当前继电器状态打包为字节帧，写入 buf。
 *       帧内容：帧头(2B) + GJ_ZSF_Address(1B) + GJ_ZSF_Status(1B) +
 *               GJ_Command_is_done(1B) + GJ_Error_Flag(1B) +
 *               GJMode(1B) + ZSFErrCnt低2字节(2B) +
 *               PEU_Right_Count(1B) + 预留(1B) + CRC16(2B)
 *       返回实际写入的字节数，若 buf 为空或 max_len 不足则返回 0。
 */
U8 PackRelayStatusFrame(U8 *buf, U8 max_len)
{
    U8  pos;
    U16 frame_crc;

    if ((buf == 0) || (max_len < 14U))
    {
        return 0U;
    }

    pos = 0U;

    /* 帧头魔数（大端） */
    buf[pos++] = (U8)(TC_FRAME_MAGIC >> 8U);
    buf[pos++] = (U8)(TC_FRAME_MAGIC & 0xFFU);

    /* GJ_Struct 字段快照 */
    buf[pos++] = GJ_Struct.GJ_ZSF_Address;
    buf[pos++] = GJ_Struct.GJ_ZSF_Status;
    buf[pos++] = GJ_Struct.GJ_Command_is_done;
    buf[pos++] = GJ_Struct.GJ_Error_Flag;

    /* 系统状态机状态 */
    buf[pos++] = (U8)GJMode;

    /* 故障计数（低2字节，大端） */
    buf[pos++] = (U8)((ZSFErrCnt >> 8U) & 0xFFU);
    buf[pos++] = (U8)(ZSFErrCnt & 0xFFU);

    /* PEU 右侧计数 */
    buf[pos++] = PEU_Right_Count;

    /* 保留字节 */
    buf[pos++] = 0x00U;

    /* 计算 CRC16（覆盖以上所有字节） */
    frame_crc = CalcCRC16(buf, pos);
    buf[pos++] = (U8)(frame_crc >> 8U);
    buf[pos++] = (U8)(frame_crc & 0xFFU);

    return pos;
}

/*
 * 函数：SendStatusViaSfr
 * 功能：通过 SFR UART 寄存器逐字节发送状态帧。
 *       每字节发送前等待 TX_READY 位置位，最多等待 200 次空循环。
 */
void SendStatusViaSfr(const U8 *buf, U8 len)
{
    U8          i;
    volatile U8 wait;

    if (buf == 0)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        /* 等待发送就绪 */
        wait = 0U;
        while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (wait < 200U))
        {
            wait++;
        }

        /* 写入发送寄存器 */
        UART_TX_REG = buf[i];
    }
}

/************************************************************
 * 条件编译诊断输出
 ************************************************************/

#ifdef RELAY_VERBOSE

/*
 * 函数：RelayDump
 * 功能：将 GJ_Struct 各字段、系统状态、故障统计写入调试输出寄存器。
 *       仅在定义了 RELAY_VERBOSE 宏时编译，用于台架调试。
 *       8051 无标准 printf，通过 UART_TX_REG 逐字节输出十六进制编码。
 */
static void RelayDump(void)
{
    U8  dump_buf[STATUS_FRAME_MAX_LEN];
    U8  dump_len;
    U8  i;
    U8  hi;
    U8  lo;

    /* 使用状态帧打包函数获取结构化数据 */
    dump_len = PackRelayStatusFrame(dump_buf, STATUS_FRAME_MAX_LEN);

    if (dump_len == 0U)
    {
        return;
    }

    /* 以十六进制文本形式逐字节输出（每字节输出两个 ASCII hex 字符） */
    for (i = 0U; i < dump_len; i++)
    {
        hi = (dump_buf[i] >> 4U) & 0x0FU;
        lo =  dump_buf[i]        & 0x0FU;

        /* 高半字节转 ASCII */
        if (hi < 10U)
        {
            hi = hi + (U8)'0';
        }
        else
        {
            hi = hi - 10U + (U8)'A';
        }

        /* 低半字节转 ASCII */
        if (lo < 10U)
        {
            lo = lo + (U8)'0';
        }
        else
        {
            lo = lo - 10U + (U8)'A';
        }

        /* 等待 TX 就绪再发送 */
        {
            volatile U8 w = 0U;
            while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U))
            {
                w++;
            }
        }
        UART_TX_REG = hi;

        {
            volatile U8 w = 0U;
            while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U))
            {
                w++;
            }
        }
        UART_TX_REG = lo;
    }

    /* 发送分隔符（换行符，ASCII 0x0A） */
    {
        volatile U8 w = 0U;
        while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U))
        {
            w++;
        }
    }
    UART_TX_REG = 0x0AU;

    /* 附加输出系统状态枚举值和故障计数 */
    {
        U8  state_val = (U8)GJMode;
        U8  fhi;
        U8  flo;

        /* 输出系统状态单字节十六进制 */
        fhi = (state_val >> 4U) & 0x0FU;
        flo =  state_val        & 0x0FU;

        if (fhi < 10U) { fhi = fhi + (U8)'0'; } else { fhi = fhi - 10U + (U8)'A'; }
        if (flo < 10U) { flo = flo + (U8)'0'; } else { flo = flo - 10U + (U8)'A'; }

        {
            volatile U8 w = 0U;
            while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U)) { w++; }
        }
        UART_TX_REG = fhi;

        {
            volatile U8 w = 0U;
            while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U)) { w++; }
        }
        UART_TX_REG = flo;
    }

    /* 输出健康日志索引（1字节） */
    {
        U8  idx_hi = (g_health_log_idx >> 4U) & 0x0FU;
        U8  idx_lo =  g_health_log_idx        & 0x0FU;

        if (idx_hi < 10U) { idx_hi = idx_hi + (U8)'0'; } else { idx_hi = idx_hi - 10U + (U8)'A'; }
        if (idx_lo < 10U) { idx_lo = idx_lo + (U8)'0'; } else { idx_lo = idx_lo - 10U + (U8)'A'; }

        {
            volatile U8 w = 0U;
            while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U)) { w++; }
        }
        UART_TX_REG = idx_hi;

        {
            volatile U8 w = 0U;
            while (((UART_STATUS_REG & UART_TX_READY_BIT) == 0U) && (w < 200U)) { w++; }
        }
        UART_TX_REG = idx_lo;
    }
}

#endif /* RELAY_VERBOSE */
