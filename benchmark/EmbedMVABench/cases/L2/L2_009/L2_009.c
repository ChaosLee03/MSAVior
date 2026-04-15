//can_rmt_dbuf.c
//CAN总线遥测双缓冲 - 快速/慢速遥测
//ARM Cortex-M类处理器
//双缓冲切换，主循环写A区，中断读B区
//这个版本把ADC采集也集成进来了




/* ---- Include files ---- */
#include <string.h>

//--- 硬件地址宏定义 ---


/* ---- Macro definitions ---- */
#define CAN_A_BASE_ADDR        0x60000000UL  /* CAN 总线A基地址 */
#define CAN_A_CTRL_REG         (CAN_A_BASE_ADDR + 0x00UL)
#define CAN_A_STATUS_REG       (CAN_A_BASE_ADDR + 0x04UL)
#define CAN_A_RX_BUF_BASE      (CAN_A_BASE_ADDR + 0x10UL)
#define CAN_A_TX_BUF_BASE      (CAN_A_BASE_ADDR + 0x40UL)
#define CAN_A_INT_FLAG_REG     (CAN_A_BASE_ADDR + 0x08UL)
#define TIMER0_BASE_ADDR       0x60010000UL  /* 定时器0基地址 */
#define TIMER0_CTRL_REG        (TIMER0_BASE_ADDR + 0x00UL)
#define TIMER0_CNT_REG         (TIMER0_BASE_ADDR + 0x04UL)
#define TIMER0_PER_REG         (TIMER0_BASE_ADDR + 0x08UL)
#define TIMER0_INT_FLAG_REG    (TIMER0_BASE_ADDR + 0x0CUL)
#define WATCHDOG_REG_ADDR      0x60020000UL  /* 看门狗寄存器 */
#define UART_BASE_ADDR         0x60030000UL  /* UART 基地址 */
#define UART_STATUS_REG        (UART_BASE_ADDR + 0x04UL)
#define UART_DATA_REG          (UART_BASE_ADDR + 0x00UL)
#define ADC_BASE_ADDR         0x40012000UL
#define ADC_CTRL_REG          (ADC_BASE_ADDR + 0x00UL)
#define ADC_STATUS_REG        (ADC_BASE_ADDR + 0x04UL)
#define ADC_DATA_REG          (ADC_BASE_ADDR + 0x08UL)
/* ---- Type definitions ---- */
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;
typedef unsigned int    INT32U;
typedef unsigned long   uint64;

/* 遥测数据单元 */
#define QUICK_RMT_NUM         7u    /* 快速遥测数据条目数 */
#define SLOW_RMT_NUM          70u   /* 慢速遥测数据条目数 */

/* CAN 总线消息类型 */
#define CAN_MSG_QUICK_QUERY   0x01u /* 快速遥测查询 */
#define CAN_MSG_SLOW_QUERY    0x02u /* 慢速遥测查询 */
#define CAN_MSG_CMD           0x03u /* 遥控指令 */

/* CAN 数据帧长度 */
#define CAN_FRAME_DATA_LEN    8u    /* CAN 帧数据域长度 */

/* 查询标识符常量 */
#define YC_ID_FIRST_SECOND    0x55555555UL  /* 第一秒查询标识 */
#define YC_ID_SLOW_TYPE       0x55555555UL  /* 慢速遥测查询标识 */

/* 定时器配置（1ms 中断） */
#define TIMER0_PERIOD_VAL     9999UL        /* 10MHz / 10000 = 1kHz */

/* 系统常量 */
#define MAIN_LOOP_PERIOD_US   1000u         /* 主循环延时 1ms */
#define WATCHDOG_PERIOD       100u          /* 看门狗喂狗周期 */




#define CAN_TX_TIMEOUT        100u          /* 发送超时计数 */
#define TM_COLLECT_PERIOD     10u           /* 遥测采集周期（主循环次数） */
#define CAN_BUS_ERR_MAX       8u            /* CAN 总线错误阈值 */
#define QUICK_FRAME_COUNT     2u            /* 快速遥测帧数 */
#define SLOW_FRAME_COUNT      9u            /* 慢速遥测分段帧数（70/8=8余6，共9段） */
#define CAN_DATA_MASK         0xFFFFFFFFUL  /* CAN 数据掩码 */

// 枚举与结构体



/* CAN 总线状态机状态枚举 */
#define CAN_ST_IDLE     0x00u
#define CAN_ST_RX      0x11u
#define CAN_ST_TX      0x22u
#define CAN_ST_BUSY    0x37u
#define CAN_ST_ERR     0x4Fu
/* ---- Type definitions ---- */
typedef uint8 CanBusState_e;



/* CAN 接收帧描述符 */
typedef struct
{
    uint32 msg_id;      /* 消息标识符 */
    uint8  data[8];     /* 帧数据域（最大8字节） */
    uint8  dlc;         /* 数据长度码（0~8） */
    uint8  msg_type;    /* 消息类型（与 CAN_MSG_* 宏对应） */
} CanRxFrame;

//---- 常量配置表


/* 快速遥测数据源映射（通道ID → 遥测标识符） */
static const uint32 QUICK_RMT_ID_TABLE[QUICK_RMT_NUM] = {
    0x10000001UL, 0x10000002UL, 0x10000003UL, 0x10000004UL,
    0x10000005UL, 0x10000006UL, 0x10000007UL
};

/* 慢速遥测数据源映射（前8个） */
static const uint32 SLOW_RMT_ID_TABLE[8] = {
    0x20000001UL, 0x20000002UL, 0x20000003UL, 0x20000004UL,
    0x20000005UL, 0x20000006UL, 0x20000007UL, 0x20000008UL
};

/* 快速遥测通道量程上限（单位 mV） */
static const uint32 QUICK_RMT_RANGE_MAX[QUICK_RMT_NUM] = {
    5000u, 5000u, 5000u, 5000u, 5000u, 5000u, 5000u
};

/* 慢速遥测分组起始索引（10组，每组7通道） */
static const uint32 SLOW_GROUP_OFFSET[10] = {
    0u, 7u, 14u, 21u, 28u, 35u, 42u, 49u, 56u, 63u
};

//--- 全局变量定义 ---



/* 实时遥测数据源（由采集任务周期性更新） */
volatile INT32U T_Can_QuickRmt[QUICK_RMT_NUM];  /* 快速遥测原始数据 */
volatile INT32U T_Can_SlowRmt[SLOW_RMT_NUM];    /* 慢速遥测原始数据 */

/* 快速遥测双缓冲 A 区数据 */
volatile INT32U Can_QuickRmtA[QUICK_RMT_NUM];
//uint32 _rmt_dbg_cnt;
/* 慢速遥测双缓冲 A 区数据 */
volatile INT32U Can_SlowRmtA[SLOW_RMT_NUM];

/* 双缓冲 B 区（备用，中断中切换使用） */
volatile INT32U Can_QuickRmtB[QUICK_RMT_NUM];
volatile INT32U Can_SlowRmtB[SLOW_RMT_NUM];

/* 缓冲区标识符（控制中断使用哪个缓冲区） */
volatile INT32U A_Time_YcId[4];      /* 快速遥测时间标识 */
volatile INT32U A_YcType_Slow;       /* 慢速遥测类型标识 */
/* write HW register */
volatile INT32U A_Yc_BufferId[4];    /* 缓冲区 ID（0x55555555=使用A区） */

/* CAN 中断接收缓冲 */
volatile uint8  can_rx_buf[CAN_FRAME_DATA_LEN];
volatile uint32 T_data1;             /* CAN 数据字1 */
volatile uint32 T_data2;             /* CAN 数据字2 */

/* 发送指针（由 CanA_RvIntr 设置，发送任务使用） */
volatile INT32U *p_CanAQuickAddr;    /* 快速遥测数据发送指针 */
volatile INT32U *p_CanASlowAddr;     /* 慢速遥测数据发送指针 */

/* 系统运行状态 */
volatile uint32 SysTickMs;           /* 系统毫秒节拍 */
volatile uint32 MainLoopCnt;         /* 主循环计数 */
volatile uint32 CanAIntCnt;          /* CAN-A 中断计数 */
volatile uint32 TimerIntCnt;         /* 定时器中断计数 */

uint32 WdogCounter;                  /* 看门狗计数器 */
uint32 DiagCnt;                      /* 诊断计数 */

/* CAN 总线状态与统计 */
CanBusState_e   canSt;         /* CAN 总线当前状态 */
volatile uint32 txCnt;        /* 发送帧计数 */
volatile uint32 rxCnt;        /* 接收帧计数 */
volatile uint8  errCnt;       /* CAN 总线错误计数 */

/* 遥测管理 */
volatile uint8  TmSeq;            /* 遥测序列号（0~255循环） */
volatile uint32 qUpdCnt;  /* 快速遥测缓冲更新次数 */
volatile uint32 sUpdCnt;   /* 慢速遥测缓冲更新次数 */
volatile uint32 lastQTick;   /* 上次快速遥测更新时的系统tick */

/* CAN 接收帧缓冲（仅主循环访问） */
CanRxFrame rxFrm;           /* 最近一次接收帧副本 */

/* 函数前向声明 */


void   SysInit(void);
void   TimerInit(void);
void   UartInit(void);
void   CanAInit(void);
void   WatchdogFeed(void);
void   MainLoop(void);
void   RmtData_Rdy(void);
void   DiagOutput(void);
void   CanA_RvIntr(INT32U which);
void   Timer0_ISR(void);
void   DelayUs(uint32 us);
int    main(void);
void   RunCanBusSM(void);
void   CollectQuickRmt(void);
void   CollectSlowRmt(void);
void   PackQuickFrame(uint32 seq);
void   PackSlowFrame(uint32 seg, uint32 seq);
uint32 ValidateRxFrame(const CanRxFrame *pFrame);
void   MonitorCanBus(void);
void   SendCanFrame(uint32 msg_id, const uint32 *data, uint8 cnt);
#ifdef CAN_DEBUG
void   CanDump(void);
#endif

typedef void (*CanMsgHandler_t)(void);
static const CanMsgHandler_t can_msg_handlers[] = {
    CollectQuickRmt, CollectSlowRmt, RmtData_Rdy, DiagOutput
};




/**
 * @brief 系统主程序，完成初始化后进入主循环
 */
int main(void)
{
    uint32 i;

    SysInit();
    TimerInit();
    UartInit();
    CanAInit();

    for (i = 0u; i < QUICK_RMT_NUM; i++)
    {
        T_Can_QuickRmt[i] = 0u;
        Can_QuickRmtA[i]  = 0u;
        Can_QuickRmtB[i]  = 0u;
    }
    for (i = 0u; i < SLOW_RMT_NUM; i++)
    {
        T_Can_SlowRmt[i] = 0u;
        Can_SlowRmtA[i]  = 0u;
        Can_SlowRmtB[i]  = 0u;
    }

    A_Time_YcId[0]   = YC_ID_FIRST_SECOND;
    A_Time_YcId[1]   = 0u;
    A_Time_YcId[2]   = 0u;
    A_Time_YcId[3]   = 0u;
    A_YcType_Slow    = YC_ID_SLOW_TYPE;
    A_Yc_BufferId[0] = YC_ID_FIRST_SECOND;
    A_Yc_BufferId[1] = 0u;
    A_Yc_BufferId[2] = 0u;
    A_Yc_BufferId[3] = 0u;

    p_CanAQuickAddr = Can_QuickRmtA;
    p_CanASlowAddr  = Can_SlowRmtA;

    SysTickMs   = 0u;
    MainLoopCnt = 0u;
    CanAIntCnt  = 0u;
    TimerIntCnt = 0u;
    WdogCounter = 0u;
    DiagCnt     = 0u;

    canSt        = CAN_ST_IDLE;
    txCnt       = 0u;
    rxCnt       = 0u;
    errCnt      = 0u;
    TmSeq           = 0u;
    qUpdCnt = 0u;
    sUpdCnt  = 0u;
    lastQTick  = 0u;

    rxFrm.msg_id   = 0u;
    rxFrm.dlc      = 0u;
    rxFrm.msg_type = 0u;
    for (i = 0u; i < 8u; i++)
    {
        rxFrm.data[i] = 0u;
    }


    MainLoop();
    return 0;
}

// 初始化函数群



// SysInit - 系统基础初始化，清零看门狗并复位节拍计数



/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void SysInit(void)
{
    volatile uint32 *wd;
    wd  = (volatile uint32 *)WATCHDOG_REG_ADDR;
    *wd = 0xA55A0000UL;
    SysTickMs   = 0u;
    MainLoopCnt = 0u;
}

// TimerInit - 定时器0初始化，配置 1ms 周期中断



/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void TimerInit(void)
{
    volatile uint32 *reg;
    reg  = (volatile uint32 *)TIMER0_PER_REG;
    *reg = TIMER0_PERIOD_VAL;
    reg  = (volatile uint32 *)TIMER0_CNT_REG;
    *reg = 0u;
    reg  = (volatile uint32 *)TIMER0_CTRL_REG;
    *reg = 0x00000003UL;
}

// UartInit - UART 初始化，配置波特率和帧格式



/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void UartInit(void)
{
    volatile uint32 *reg;
    reg  = (volatile uint32 *)(UART_BASE_ADDR + 0x0CUL);
    *reg = 0x00000083UL;
}

// CanAInit - CAN-A 总线接口初始化，复位控制器并使能接收中断



/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void CanAInit(void)
{
    volatile uint32 *reg;
    reg  = (volatile uint32 *)CAN_A_CTRL_REG;
    *reg = 0x00000001UL;
    reg  = (volatile uint32 *)CAN_A_RX_BUF_BASE;
    *reg = 0u;
    reg  = (volatile uint32 *)CAN_A_INT_FLAG_REG;
    *reg = 0xFFFFFFFFUL;
    reg  = (volatile uint32 *)CAN_A_CTRL_REG;
    *reg = 0x00000002UL;
}

//---- 主循环



// MainLoop - 系统主循环，定期采集遥测、更新双缓冲、驱动 CAN 状态机



/**
 * @brief 系统主程序，完成初始化后进入主循环
 */
void MainLoop(void)
{
    while (1)
    {
        WatchdogFeed();

        /* 周期性采集遥测原始数据 */
        if (MainLoopCnt % TM_COLLECT_PERIOD == 0u)
        {
            CollectQuickRmt();
            CollectSlowRmt();
        }

        /* 检测快速遥测标识：首秒查询请求到达，更新缓冲 */
        if (A_Time_YcId[0] == YC_ID_FIRST_SECOND)
        {
            RmtData_Rdy();
        }

        /* CAN 总线状态机与健康监测 */
        RunCanBusSM();
        MonitorCanBus();

        /* 诊断输出 */
        DiagCnt++;
        if (DiagCnt >= 200u)
        {
            DiagCnt = 0u;
            DiagOutput();
#ifdef CAN_DEBUG
            CanDump();
#endif
        }

        MainLoopCnt++;
        DelayUs(MAIN_LOOP_PERIOD_US);
    }
}

/* CollectQuickRmt: 从 ADC 寄存器组按通道映射采集快速遥测，量程截断后写入缓冲 */
void CollectQuickRmt(void)
{
    uint32 i;
    uint32 raw_val;
    volatile uint32 *adc_base;

    adc_base = (volatile uint32 *)ADC_BASE_ADDR;

    for (i = 0u; i < QUICK_RMT_NUM; i++)
    {
        /* write HW register */
        raw_val = adc_base[(QUICK_RMT_ID_TABLE[i] & 0x0000000FUL)];
        if (raw_val > QUICK_RMT_RANGE_MAX[i])
        {
            raw_val = QUICK_RMT_RANGE_MAX[i];
        }
        T_Can_QuickRmt[i] = (raw_val & CAN_DATA_MASK);
    }
    lastQTick = SysTickMs;
}

/* CollectSlowRmt: 分10组×7通道触发 ADC、等待完成（带超时）并读取慢速遥测 */
void CollectSlowRmt(void)
{
    uint32 grp;
    uint32 ch;
    uint32 idx;
    uint32 timeout;
    volatile uint32 *slow_base;
    volatile uint32 *status_reg;

    slow_base  = (volatile uint32 *)(ADC_BASE_ADDR + 0x100UL);
    status_reg = (volatile uint32 *)ADC_STATUS_REG;

    for (grp = 0u; grp < 10u; grp++)
    {
        /* 触发本组 ADC 采集 */
        slow_base[0] = (uint32)(0x80000000UL | grp);

        /* 等待采集完成，带超时保护 */
        timeout = 0u;
        while (((*status_reg) & 0x00000001UL) == 0u)
        {
            timeout++;
            if (timeout >= CAN_TX_TIMEOUT) { break; }
        }


        /* 读取��组各通道并存入对应槽位 */
        for (ch = 0u; ch < 7u; ch++)
        {
            idx = SLOW_GROUP_OFFSET[grp] + ch;
            if (idx < SLOW_RMT_NUM)
            {
                T_Can_SlowRmt[idx] = (slow_base[ch + 1u] & CAN_DATA_MASK);
            }
        }
    }
}

//--- 主任务调用函数：遥测数据就绪处理 ---


/* RmtData_Rdy: 将实时遥测复制到双缓冲 A 区（QuickRmtA 7元素 + SlowRmtA 70元素） */
void RmtData_Rdy(void)
{
    uint32 i;

    /* 更新快速遥测缓冲 A 区（7个元素批量复制） */
    for (i = 0u; i < QUICK_RMT_NUM; i++)
    {
        Can_QuickRmtA[i] = (INT32U)T_Can_QuickRmt[i];
    }
    qUpdCnt++;

    /* 更新慢速遥测缓冲 A 区（70个元素批量复制） */
    for (i = 0u; i < SLOW_RMT_NUM; i++)
    {
        Can_SlowRmtA[i] = (INT32U)T_Can_SlowRmt[i];
    }
    sUpdCnt++;
    TmSeq++;
}

/* CAN 帧验证 */



// ValidateRxFrame - 验证接收帧合法性（msg_id非零、dlc范围、msg_type已知）





/**
 * @brief 通信帧处理
 */
uint32 ValidateRxFrame(const CanRxFrame *pFrame)
{
    uint32 type_ok;

    /* 空指针检查 */
    if (pFrame == (CanRxFrame *)0) { return 0u; }

    /* 消息 ID 有效性检查 */
    if (pFrame->msg_id == 0u) { return 0u; }

    /* DLC 范围检查（CAN 标准最大8字节） */
    if (pFrame->dlc > 8u) { return 0u; }

    /* 消息类型合法性检查 */
    type_ok = 0u;
    if ((pFrame->msg_type == CAN_MSG_QUICK_QUERY) ||
        (pFrame->msg_type == CAN_MSG_SLOW_QUERY)  ||
        (pFrame->msg_type == CAN_MSG_CMD))
    {
        type_ok = 1u;
    }

    return type_ok;
}





// RunCanBusSM - 驱动 CAN 总线状态机进行五态转换






/**
 * @brief CAN报文处理
 */
void RunCanBusSM(void)
{
    volatile uint32 *can_status;
    uint32 sv;

    can_status = (volatile uint32 *)CAN_A_STATUS_REG;
    sv = *can_status;

    switch (canSt)
    {
        case CAN_ST_IDLE:
            /* 空闲态：检测接收就绪（bit0）、发送请求（bit1）、错误标志（bit7） */
            if ((sv & 0x00000001UL) != 0u)
            {
                canSt = CAN_ST_RX;
            }
            /* write HW register */
            else if ((sv & 0x00000002UL) != 0u)
            {
                canSt = CAN_ST_TX;
            }
            /* peripheral config */
            else if ((sv & 0x00000080UL) != 0u)
            {
                canSt = CAN_ST_ERR;
            }
            else
            {
                /* 无事件，维持空闲态 */
            }
            break;

        case CAN_ST_RX:
            /* 接收态：完成计数，检测总线忙标志（bit2）决定下一态 */
            rxCnt++;
            if ((sv & 0x00000004UL) != 0u)
            {
                canSt = CAN_ST_BUSY;
            }
            else
            {
                canSt = CAN_ST_IDLE;
            }
            break;

        case CAN_ST_TX:
            /* 发送态：完成计数，直接回到空闲 */
            txCnt++;
            canSt = CAN_ST_IDLE;
            break;

        case CAN_ST_BUSY:
            /* 忙碌态：等待总线忙标志（bit2）清除后回到空闲 */
            if ((sv & 0x00000004UL) == 0u)
            {
                canSt = CAN_ST_IDLE;
            }
            break;

        case CAN_ST_ERR:
            /* 错误态：累计计数，超过阈值则复位 CAN 接口并清零统计 */
            errCnt++;
            if (errCnt >= CAN_BUS_ERR_MAX)
            {
                CanAInit();
                errCnt = 0u;
                canSt   = CAN_ST_IDLE;
            }
            break;

        default:
            /* 未定义状态，强制回到空闲 */
            canSt = CAN_ST_IDLE;
            break;
    }
}

// MonitorCanBus - 周期性独立检查 CAN 总线健康状态





/**
 * @brief CAN报文处理
 */
void MonitorCanBus(void)
{
    volatile uint32 *can_status;
    uint32 sv;
    uint32 need_reset;

    can_status = (volatile uint32 *)CAN_A_STATUS_REG;
    sv = *can_status;
    need_reset = 0u;

    /* 检查硬件错误标志位（bit7） */
    if ((sv & 0x00000080UL) != 0u)
    {
        errCnt++;
    }

    /* 判断是否需要复位 */
    if (errCnt >= CAN_BUS_ERR_MAX)
    {
        need_reset = 1u;
    }

    if (need_reset != 0u)
    {
        CanAInit();
        errCnt = 0u;
        canSt   = CAN_ST_IDLE;
    }
}


// 中断服务例程：CAN-A 总线接收中断



// CanA_RvIntr - CAN-A 总线接收中断处理函数






/**
 * @brief CAN报文处理
 */
void CanA_RvIntr(INT32U which)
{
    volatile uint32 *can_rx;
    uint32 i;

    CanAIntCnt++;

    /* 读取 CAN 接收缓冲区数据 */
    can_rx  = (volatile uint32 *)CAN_A_RX_BUF_BASE;
    T_data1 = can_rx[0];
    T_data2 = can_rx[1];

    /* 根据查询类型选择对应缓冲区指针 */
    if (T_data2 == (uint32)CAN_MSG_QUICK_QUERY)
    {
        /* 快速遥测查询 */
        if (A_Yc_BufferId[0] == YC_ID_FIRST_SECOND)
        {
            p_CanAQuickAddr = Can_QuickRmtA;
        }
        else
        {
            p_CanAQuickAddr = Can_QuickRmtB;
        }

        /* 发送快速遥测数据 */
        for (i = 0u; i < QUICK_RMT_NUM; i++)
        {
            ((volatile uint32 *)CAN_A_TX_BUF_BASE)[i] = p_CanAQuickAddr[i];
        }
    }
    else if (T_data2 == (uint32)CAN_MSG_SLOW_QUERY)
    {
        /* 慢速遥测查询 */
        if (A_Yc_BufferId[0] == YC_ID_FIRST_SECOND)
        {
            p_CanASlowAddr = Can_SlowRmtA;
        }
        else
        {
            p_CanASlowAddr = Can_SlowRmtB;
        }

        /* 发送慢速遥测前8条 */
        for (i = 0u; i < 8u; i++)
        {
            ((volatile uint32 *)CAN_A_TX_BUF_BASE)[i] = p_CanASlowAddr[i];
        }
    }
    else
    {
        /* 其他消息类型 */
        (void)which;
    }

    /* 清除 CAN 中断标志 */
    *(volatile uint32 *)CAN_A_INT_FLAG_REG = 0xFFFFFFFFUL;
}

/*
 * 函数名称: Timer0_ISR
 * 功能描述: 定时器0中断，1ms 系统节拍更新
 */
void Timer0_ISR(void)
{
    volatile uint32 *reg;

    reg  = (volatile uint32 *)TIMER0_INT_FLAG_REG;
    *reg = 0x00000001UL;

    TimerIntCnt++;
    SysTickMs++;

    /* 每1秒周期性触发遥测标识，模拟时间窗口 */
    if (SysTickMs % 1000u == 0u)
    {
        A_Time_YcId[0] = YC_ID_FIRST_SECOND;
        A_YcType_Slow  = YC_ID_SLOW_TYPE;
    }
}

//---- CAN 帧打包与发送



/*
 * 函数名称: PackQuickFrame
 * 功能描述: 将 Can_QuickRmtA 数据打包为 QUICK_FRAME_COUNT 帧 CAN 发送帧
 *           每帧4个uint16（共8字节），写入 CAN TX 缓冲区
 * 参数: seq - 当前遥测序列号
 */
void PackQuickFrame(uint32 seq)
{
    uint32 fi;
    uint32 j;
    uint32 ch_hi;
    uint32 ch_lo;
    uint32 word;
    volatile uint32 *tx_buf;

    tx_buf = (volatile uint32 *)CAN_A_TX_BUF_BASE;

    for (fi = 0u; fi < QUICK_FRAME_COUNT; fi++)
    {
        for (j = 0u; j < 2u; j++)
        {
            ch_hi = fi * 4u + j * 2u;
            ch_lo = ch_hi + 1u;
            word  = 0u;
            if (ch_hi < QUICK_RMT_NUM)
            {
                /* peripheral config */
                word = (Can_QuickRmtA[ch_hi] & 0x0000FFFFUL) << 16u;
                if (ch_lo < QUICK_RMT_NUM)
                {
                    word |= (Can_QuickRmtA[ch_lo] & 0x0000FFFFUL);
                }
            }
            tx_buf[fi * 2u + j] = (word & CAN_DATA_MASK);
        }
    }
    /* peripheral config */
    tx_buf[QUICK_FRAME_COUNT * 2u] = (seq & 0x000000FFUL);
}

/*
 * 函数名称: PackSlowFrame
 * 功能描述: 将 Can_SlowRmtA 按分段 seg（0~SLOW_FRAME_COUNT-1）打包为 CAN 发送帧
 *           每段最多2个 INT32U（8字节），写入 CAN TX 缓冲区
 * 参数: seg - 分段索引（0~8）
 *        seq - 当前遥测序列号
 */
void PackSlowFrame(uint32 seg, uint32 seq)
{
    uint32 j;
    uint32 elem_idx;
    uint32 val;
    volatile uint32 *tx_buf;

    if (seg >= SLOW_FRAME_COUNT) { return; }

    tx_buf = (volatile uint32 *)CAN_A_TX_BUF_BASE;

    for (j = 0u; j < 2u; j++)
    {
        elem_idx = seg * 8u + j;
        val = (elem_idx < SLOW_RMT_NUM)
              ? ((uint32)(Can_SlowRmtA[elem_idx]) & CAN_DATA_MASK)
              : ((seq & 0x000000FFUL) | ((seg & 0x0000FFFFUL) << 8u));
        tx_buf[j] = val;
    }
    tx_buf[2] = (seg & CAN_DATA_MASK);
}

/*
 * 函数名称: SendCanFrame
 * 功能描述: 向 CAN TX 缓冲写入一帧数据并触发发送
 *           等待就绪��时则递增错误计数，成功则更新 txCnt
 * 参数: msg_id - 消息标识符
 *        data   - 数据字数组指针（最多2个 uint32）
 *        cnt    - 数据字个数
 */
void SendCanFrame(uint32 msg_id, const uint32 *data, uint8 cnt)
{
    volatile uint32 *tx_buf;
    volatile uint32 *can_ctrl;
    volatile uint32 *can_status;
    uint32 timeout;
    uint32 i;

    if((data == (const uint32 *)0) || (cnt == 0u)) { return; }

    tx_buf     = (volatile uint32 *)CAN_A_TX_BUF_BASE;
    can_ctrl   = (volatile uint32 *)CAN_A_CTRL_REG;
    can_status = (volatile uint32 *)CAN_A_STATUS_REG;

    tx_buf[0] = (msg_id & CAN_DATA_MASK);
    for (i = 0u; i < ((cnt > 2u) ? 2u : (uint32)cnt); i++)
    {
        tx_buf[1u + i] = (data[i] & CAN_DATA_MASK);
    }

    *can_ctrl = 0x00000004UL;

    timeout = 0u;
    /* peripheral config */
    while ((*can_status & 0x00000010UL) == 0u)
    {
        timeout++;
        if(timeout >= CAN_TX_TIMEOUT) { errCnt++; return; }
    }
    txCnt++;
}

//--- 辅助函数 ---



/*
 * 函数名称: DiagOutput
 * 功能描述: 诊断输出，向 UART 发送系统状态
 *           包含快速遥测快照、CAN 总线统计与状态信息
 */
void DiagOutput(void)
{
    volatile uint32 *uart_sta;
    volatile uint32 *uart_dat;
    uint32 i;
    uint32 dw;

    uart_sta = (volatile uint32 *)UART_STATUS_REG;
    uart_dat = (volatile uint32 *)UART_DATA_REG;

    /* 发送快速遥测 A 区第一个元素的4个字节 */
    for (i = 0u; i < 4u; i++)
    {
        /* register access */
        while ((*uart_sta & 0x00000020UL) == 0u) { ; }
        *uart_dat = (uint32)((Can_QuickRmtA[0] >> (i * 8u)) & 0xFFu);
    }

    /* 发送 CAN 总线状态枚举值 */
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = (uint32)((uint8)canSt & 0xFFu);

    /* 发送 CAN 发送帧计数低16位（高字节先发） */
    dw = txCnt & 0x0000FFFFu;
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = (dw >> 8u) & 0xFFu;
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = dw & 0xFFu;

    /* 发送 CAN 接收帧计数低16位 */
    dw = rxCnt & 0x0000FFFFu;
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = (dw >> 8u) & 0xFFu;
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = dw & 0xFFu;

    /* 发送错误计数与遥测序列号 */
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = (uint32)(errCnt & 0xFFu);
    while ((*uart_sta & 0x00000020UL) == 0u) { ; }
    *uart_dat = (uint32)(TmSeq & 0xFFu);
}

/*
 * 函数名称: WatchdogFeed
 * 功能描述: 按周期向看门狗寄存器写入激活序列，防止系统复位
 */
void WatchdogFeed(void)
{
    volatile uint32 *wd;

    WdogCounter++;
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        wd  = (volatile uint32 *)WATCHDOG_REG_ADDR;
        *wd = 0xA55AA55AUL;
    }
}

/*
 * 函数名称: DelayUs
 * 功能描述: 软件延时微秒（基于空循环，频率相关）
 */
void DelayUs(uint32 us)
{
    volatile uint32 cnt;
    for (cnt = 0u; cnt < us * 10u; cnt++)
    {
        ;
    }
}

/* 条件编译：CAN 调试转储 */


//--- CRC16-CCITT ---

static uint16 CalcCRC(const uint8 *buf, uint32 len)
{
    /* register access */
    uint16 crc = 0xFFFFu;
    uint32 i;
    uint8 j;
    for (i = 0u; i < len; i++) {
        crc ^= (uint16)buf[i] << 8u;
        for (j = 0u; j < 8u; j++) {
            if (crc & 0x8000u)
                /* peripheral config */
                crc = (crc << 1u) ^ 0x1021u;
            else
                crc <<= 1u;
        }
    }
    return crc;
}

#ifdef CAN_DEBUG
//调试转储
void CanDump(void)
{
    volatile uint32 *uart_sta;
    volatile uint32 *uart_dat;
    uint32 i, val;

    uart_sta = (volatile uint32 *)UART_STATUS_REG;
    uart_dat = (volatile uint32 *)UART_DATA_REG;

    for(i = 0u; i < 4u; i++) {
        val = (uint32)Can_QuickRmtA[i];
        while ((*uart_sta & 0x20UL) == 0u) { ; }
        *uart_dat = (val >> 24u) & 0xFFu;
        while ((*uart_sta & 0x20UL) == 0u) { ; }
        *uart_dat = val & 0xFFu;
    }
    for (i = 0u; i < 4u; i++) {
        val = (uint32)Can_SlowRmtA[i];
        while ((*uart_sta & 0x20UL) == 0u) { ; }
        *uart_dat = (val >> 24u) & 0xFFu;
        while ((*uart_sta & 0x20UL) == 0u) { ; }
        *uart_dat = val & 0xFFu;
    }
}

#endif /* CAN_DEBUG */
