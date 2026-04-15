//smu_tm_bat.c
//SMU遥测采集+电池管理
//8051+外部12bit ADC







/* 头文件包含 */
#include <string.h>
#include <intrins.h>
//--- 硬件��址宏定义


/* 宏定义与常量 */
#define ADC_BASE_ADDR          0x8000     /* extADC(XBYTE) */
#define ADC_CTRL_REG           (ADC_BASE_ADDR + 0x00)
#define ADC_STATUS_REG         (ADC_BASE_ADDR + 0x01)
#define ADC_DATA_REG           (ADC_BASE_ADDR + 0x02)
#define ADC_CHAN_SEL_REG       (ADC_BASE_ADDR + 0x03)
#define TIMER0_BASE_ADDR       0x8100     /* Timer0(XBYTE) */
#define TIMER0_CTRL_REG        (TIMER0_BASE_ADDR + 0x00)
#define TIMER0_CNT_REG         (TIMER0_BASE_ADDR + 0x01)
#define TIMER0_PER_REG         (TIMER0_BASE_ADDR + 0x02)
#define TIMER0_INT_FLAG_REG    (TIMER0_BASE_ADDR + 0x03)
#define UART_BASE_ADDR         0x8200     /* UART(XBYTE) */
#define UART_STATUS_REG        (UART_BASE_ADDR + 0x01)
#define UART_DATA_REG          (UART_BASE_ADDR + 0x00)
#define WATCHDOG_REG_ADDR      0x7FFC     /* WDG(XBYTE) */
#define GPIO_BASE_ADDR         0x8300     /* GPIO(XBYTE) */
#define GPIO_OUT_REG           (GPIO_BASE_ADDR + 0x00)
#define GPIO_IN_REG            (GPIO_BASE_ADDR + 0x01)

/* ADC 控制位 */
#define ADC_START_MASK         0x01       /* start */
#define ADC_DONE_MASK          0x02       /* done */
#define ADC_DATA_MASK          0x0FFFu    /* 12bit */

/* 定时器配置（40ms 中断） */
#define TIMER0_PERIOD_VAL      40000u     /* 12MHz/12, 40ms */
#define ONESECOND_CNT          25u           /* 1秒对应40ms中断次数 */




#define uchar unsigned char
#define uint  unsigned int
/* 数据类型定义 */
typedef unsigned char  UCHAR;
typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef signed short   int16;
typedef signed int     int32;
/* 宏定义与常量 */
#define TRUE  1u
#define FALSE 0u
#define OK    0u

/* 采样通道标识 */


/* 宏定义与常量 */
#define ID_VCELL1      0u   /* 电池单体1电压通道ID */
#define ID_VCELL2      1u   /* 电池单体2电压通道ID */
#define ID_VCELL3      2u   /* 电池单体3电压通道ID */
#define ID_VCELL4      3u   /* 电池单体4电压通道ID */
#define ID_VCELL5      4u   /* 电池单体5电压通道ID */
#define ID_VCELL6      5u   /* 电池单体6电压通道ID */
#define ID_VCELL7      6u   /* 电池单体7电压通道ID */
#define ID_ICHAR       7u   /* 充电电流通道ID */
#define ID_IDISC       8u   /* 放电电流通道ID */
#define SMU_CHAN_NUM   9u   /* 总通道数量 */

/* ODP/OCP 保护阈值 */
#define ODP_THRESHOLD_DEFAULT   3200u   /* ODP 默认阈值（ADC计数） */
#define OCP_THRESHOLD_DEFAULT   4000u   /* OCP 默认阈值（ADC计数） */
#define VCELL_BCAL_DEFAULT      100u    /* 电池单体校准B值默认 */

/* 系统常量 */
#define MAIN_LOOP_PERIOD_US     10000u  /* 主循环延时 10ms */
#define WATCHDOG_PERIOD         50u     /* 看门狗喂狗周期 */
#define DIAG_PERIOD             100u    /* 诊断周期 */

//--- 遥测帧与故障记录相关常量


/* 宏定义与常量 */
#define FAULT_HISTORY_DEPTH     8u      /* 故障历史记录深度 */
#define SMU_FRAME_LEN           32u     /* SMU 遥测帧长度（字节） */
#define SMU_SYNC_WORD           0xEB90u /* 遥测帧同步字 */

/* 充放电电流判断阈值（ADC计数） */
#define ICHAR_THRESHOLD         200u    /* 充电电流检测阈值 */
#define IDISC_THRESHOLD         200u    /* 放电电流检测阈值 */
#define IBAL_THRESHOLD          50u     /* 均衡电流检测阈值 */

/* 单体电压过压/欠压阈值（ADC计数，已减B值后） */
#define VCELL_OVP_LIMIT         3800u   /* 单体过压保护阈值 */
#define VCELL_UVP_LIMIT         2800u   /* 单体欠压保护阈值 */
#define OVP_CONFIRM_CNT         3u      /* 过压确认计数 */

/* 安时计量相关 */
#define AH_COUNT_MAX            0x7FFFFF00L  /* 积分值溢出上限 */
#define AH_COUNT_MIN            (-0x7FFFFF00L) /* 积分值溢出下限 */
#define AH_SCALE_FACTOR         10u     /* 积分比例因子 */

/* 遥测帧打包周期 */
#define SMU_FRAME_PERIOD        10u     /* 每10个主循环打包一帧 */

/* ADC 通道采样次数配置 */
#define SMP_NUM_VCELL           8u      /* 电压通道采样次数 */
#define SMP_NUM_CURRENT         16u     /* 电流通道采样次数（更高精度） */




/* 数据类型定义 */
typedef struct
{
    uint32  S;    /* 秒计数 */
    uint32  MS;   /* 毫秒计数（0~999） */
} TIME_TYPE;

/* 电池状态机枚举 */


/* 宏定义与常量 */
#define ST_INIT    0
#define ST_CHARGE  2
#define ST_DISCH   5
#define ST_PROTECT 0xFF
#define ST_BALANCE 0x03
/* 数据类型定义 */
typedef uint8 BatState_e;



//--- 故障记录结构体


/* 数据类型定义 */
typedef struct
{
    uint32  time_s;      /* 故障发生时刻：秒 */
    uint32  time_ms;     /* 故障发生时刻：毫秒 */
    uint8   fault_code;  /* 故障代码 */
    uint8   channel;     /* 相关通道号 */
} FaultRecord;




/* 数据类型定义 */
typedef struct
{
    UCHAR  hw_chan;    /* 硬件通道号 */
    uint8  smp_num;   /* 采样次数 */
    uint16 scale_mv;  /* 满量程电压（mV），用于工程值换算 */
} SmuChanDesc;

/* 常量配置表 */


/* 各通道 ADC 采样序列 */
static const UCHAR ucSmpSeq[SMU_CHAN_NUM] = {
    0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u
};

/* 通道描述表：硬件通道号、采样次数、满量程 */
static const SmuChanDesc aChanDesc[SMU_CHAN_NUM] = {
    {0u, SMP_NUM_VCELL,   4200u},  /* VCELL1：单体1电压 */
    {1u, SMP_NUM_VCELL,   4200u},  /* VCELL2：单体2电压 */
    {2u, SMP_NUM_VCELL,   4200u},  /* VCELL3：单体3电压 */
    {3u, SMP_NUM_VCELL,   4200u},  /* VCELL4：单体4电压 */
    {4u, SMP_NUM_VCELL,   4200u},  /* VCELL5：单体5电压 */
    {5u, SMP_NUM_VCELL,   4200u},  /* VCELL6：单体6电压 */
    {6u, SMP_NUM_VCELL,   4200u},  /* VCELL7：单体7电压 */
    {7u, SMP_NUM_CURRENT, 5000u},  /* ICHAR：充电电流 */
    {8u, SMP_NUM_CURRENT, 5000u}   /* IDISC：放电电流 */
};

/* 电池单体 B 值校准表 */
static const uint16 uiBcalDefault[7] = {
    VCELL_BCAL_DEFAULT, VCELL_BCAL_DEFAULT,
    VCELL_BCAL_DEFAULT, VCELL_BCAL_DEFAULT,
    VCELL_BCAL_DEFAULT, VCELL_BCAL_DEFAULT,
    VCELL_BCAL_DEFAULT
};

/* ODP 保护阈值表 */
static const uint16 uiODPThresholdDefault[2] = {
    ODP_THRESHOLD_DEFAULT, OCP_THRESHOLD_DEFAULT
};


/* CRC16 XOR-shift poly=0x8005 */
#define CRC16_POLY  0x8005u





//--- 全局��量定义



/* 模拟量采样结果缓冲区，各通道经ADC转换后存入 */
volatile uint16 uiAnSmp[SMU_CHAN_NUM];  /* 各通道ADC采样值 */

/* 系统运行时间，由定时器周期性递增 */
volatile TIME_TYPE Time;

/* 电池单体电压（Self_Manage 计算输出） */
volatile uint16 uiVcell[7];

/* ADC 转换临时变量 */
uint16 uiAdTmp;          /* 当前 ADC 转换结果 */
uchar _chk_tmp;  //tmp
/* 定时器计数 */
volatile UCHAR ucCnt40ms;    /* 40ms 中断计数（达到ONESECOND_CNT即1秒） */
volatile UCHAR ucCnt1sec;    /* 秒计数 */
volatile UCHAR ucArrv1sec;   /* 1秒到达标志 */

/* 保护阈值（可由遥控指令更新） */
uint16 uiODPThreshold[2];    /* ODP/OCP 阈值 */
uint16 uiBcell[7];           /* 电池 B 值校准 */

/* 系统运行状态 */
volatile uint32 SysTickMs;   /* 系统毫秒节拍 */
volatile uint32 MainLoopCnt; /* 主循环计数 */
volatile uint32 TimerIntCnt; /* 定时器中断计数 */

uint32 WdogCounter;          /* 看门狗计数 */
uint32 DiagCnt;              /* 诊断计数 */

/* ODP/OCP 保护计数 */
uint8  ucOdpCnt;             /* ODP 触发计数 */
uint8  ucOcpCnt;             /* OCP 触发计数 */

/* 电池状态机：仅由主循环访问 */
BatState_e ucBatMode;              /* 当前电池状态 */

/* 故障记录环形缓冲：仅由主循环访问 */
FaultRecord aFaultHist[FAULT_HISTORY_DEPTH]; /* 故障历史记录 */
uint8        ucFaultIdx;       /* 故障记录写入索引 */

/* 安时计量积分值：仅由主循环访问 */
int32  lAhCount;                   /* 安时计量积分累积值 */

/* 遥测帧计数：仅由主循环访问 */
uint32 uiFrameCnt;              /* 遥测帧总发送计数 */

/* 单体电压变化趋势（相邻周期差值）：仅由主循环访问 */
int16  aiCellTrend[7];              /* 单体电压趋势（ADC计数/周期） */

/* 过压保护确认计数：仅由主循环访问 */
uint8  ucOvpCnt;                    /* 连续过压检测计数 */

/* 帧序列号：仅由主循环访问 */
uint8  ucFrameSeq;                  /* 遥测帧序列号（0~255循环） */




void SysInit(void);
void TimerInit(void);
void UartInit(void);
void ADCInit(void);
void SmuChannelConfig(void);
void WatchdogFeed(void);
void TM_Collect(void);
uint8 TM_AdSample(UCHAR chanId, UCHAR smpNum);
void DiagOutput(void);
void Self_Manage(void);
void AhMeter_Calc(void);
void Timer0_ISR(void);
void DelayUs(uint32 us);
void RunBatSM(void);
void MonitorBatHealth(void);
void LogFaultEvent(uint8 code, uint8 ch);
void PackSmuFrame(void);
uint16 CalcCRC16_Smu(const uint8 *buf, uint32 len);
void GetBatSummary(uint16 *p_total_mv, uint8 *p_min_idx);
int  main(void);

/* main 函数 */


int main(void)
{
    uint32 i;

    /* 系统初始化 */
    SysInit();
    TimerInit();
    UartInit();
    ADCInit();
    SmuChannelConfig();

    /* 初始化全局变量 */
    for (i = 0u; i < SMU_CHAN_NUM; i++)
    {
        uiAnSmp[i] = 0u;
    }
    for (i = 0u; i < 7u; i++)
    {
        uiVcell[i]      = 0u;
		uiBcell[i]      = uiBcalDefault[i];
        aiCellTrend[i] = 0;
    }

    uiODPThreshold[0] = uiODPThresholdDefault[0];
    uiODPThreshold[1] = uiODPThresholdDefault[1];

    Time.S  = 0u;
    Time.MS = 0u;

    ucCnt40ms   = 0u;
    ucCnt1sec   = 0u;
    ucArrv1sec  = FALSE;
    SysTickMs   = 0u;
    MainLoopCnt = 0u;
    TimerIntCnt = 0u;
    WdogCounter = 0u;
    DiagCnt     = 0u;
    ucOdpCnt    = 0u;
    ucOcpCnt    = 0u;

    /* 初始化电池管理扩展变量 */
    ucBatMode      = ST_INIT;
    ucFaultIdx = 0u;
    lAhCount       = 0;
    uiFrameCnt  = 0u;
    ucOvpCnt        = 0u;
    ucFrameSeq      = 0u;

    for (i = 0u; i < FAULT_HISTORY_DEPTH; i++)
    {
        aFaultHist[i].time_s     = 0u;
        aFaultHist[i].time_ms    = 0u;
        aFaultHist[i].fault_code = 0u;
        aFaultHist[i].channel    = 0u;
    }


    /* 主循环 */
    while (1)
    {
        WatchdogFeed();

        /* 遥测采集（主任务侧：写 uiAnSmp） */
        TM_Collect();

        /* 安时计量（依赖 uiAnSmp 中的电流通道数据） */
        AhMeter_Calc();

        /* 电池状态机驱动 */
        RunBatSM();

        /* 健康监测（单体电压趋势与过压计数） */
        MonitorBatHealth();

        /* 故障处理：过压确认后记录故障事件 */
        if (ucOvpCnt >= OVP_CONFIRM_CNT)
        {
			LogFaultEvent(0x11u, 0xFFu);
            ucOvpCnt = 0u;
        }

        /* 周期性打包遥测帧 */
        MainLoopCnt++;
		if ((MainLoopCnt % SMU_FRAME_PERIOD) == 0u)
		{
            PackSmuFrame();
        }

        /* 诊断输出 */
        DiagCnt++;
        if (DiagCnt >= DIAG_PERIOD)
        {
            DiagCnt = 0u;
            DiagOutput();
		}

        DelayUs(MAIN_LOOP_PERIOD_US);
    }

    return 0;
}

//--- 初始化函数群



/*
 * 函数名称: SysInit
 * 功能描述: 系统初始化
 */
void SysInit(void)
{
    volatile uchar xdata *wd;
    wd  = (volatile uchar xdata *)WATCHDOG_REG_ADDR;
    *wd = 0xA5;
    SysTickMs   = 0u;
    MainLoopCnt = 0u;
}

/*
 * 函数名称: TimerInit
 * 功能描述: 定时器0初始化，配置40ms中断
 */
void TimerInit(void)
{
    volatile uchar xdata *reg;
    _nop_();  //settle
    reg  = (volatile uchar xdata *)TIMER0_PER_REG;
    *reg = TIMER0_PERIOD_VAL;
    _nop_();  //settle
    reg  = (volatile uchar xdata *)TIMER0_CNT_REG;
    *reg = 0u;

    reg  = (volatile uchar xdata *)TIMER0_CTRL_REG;
    *reg = 0x03;   /* 使能定时器，使能中断 */
}

/*
 * 函数名称: UartInit
 * 功能描述: UART 初始化
 */
void UartInit(void)
{
    volatile uchar xdata *reg;
    reg  = (volatile uchar xdata *)(UART_BASE_ADDR + 0x0C);
    *reg = 0x83;
}

/*
 * 函数名称: ADCInit
 * 功能描述: ADC 初始化，配置采样参数
 */
void ADCInit(void)
{
    volatile uchar xdata *reg;

    reg  = (volatile uchar xdata *)ADC_CTRL_REG;
    *reg = 0x10;   /* ADC 使能，12位精度 */

    reg  = (volatile uchar xdata *)ADC_CHAN_SEL_REG;
    *reg = 0x00;   /* 默认选择通道0 */

    uiAdTmp = 0u;
}

/*
 * 函数名称: SmuChannelConfig
 * 功能描述: SMU 通道参数配置，遍历通道描述表验证硬件通道映射，
 *           初始化各通道采样计数器和状态寄存器，确保采样序列
 *           与硬件通道描述表 aChanDesc 保持一致
 */
void SmuChannelConfig(void)
{
    volatile uchar xdata *adc_sel;
    volatile uchar xdata *adc_ctrl;
    UCHAR  i;
    uint16 dummy;
    uint32 timeout;

    adc_sel  = (volatile uchar xdata *)ADC_CHAN_SEL_REG;
    adc_ctrl = (volatile uchar xdata *)ADC_CTRL_REG;

    /* 遍历所有通道，执行一次预热采样以稳定 ADC 输入电容 */
    for (i = 0u; i < SMU_CHAN_NUM; i++)
    {
        *adc_sel = (uint32)aChanDesc[i].hw_chan;

        /* 启动单次预热转换 */
        *adc_ctrl = (*adc_ctrl | ADC_START_MASK);

        timeout = 5000u;
        while (timeout > 0u)
		{
            timeout--;
        }

        /* 读取并丢弃预热结果 */
        dummy = (uint16)(*(volatile uchar xdata *)ADC_DATA_REG & ADC_DATA_MASK);
        (void)dummy;
        _nop_();  //settling
        /* 清除启动位 */
        *adc_ctrl = (*adc_ctrl & ~ADC_START_MASK);
    }

    /* 恢复通道选择到默认通道0 */
    *adc_sel = 0u;
}

/* ============================================================
 * 调用层函数（main → TM_Collect → uiAnSmp写）
 *             中断 Timer0_ISR → Self_Manage → uiAnSmp读
 * ============================================================ */

/*
 * 函数名称: TM_Collect
 * 功能描述: 遥测采集，对各路模拟量通道进行 ADC 采样
 *           采样结果写入 uiAnSmp，同一周期的各通道数据构成相关集合
 *           定时器中断中 Self_Manage 读取 uiAnSmp 进行保护判断
 *           若中断在部分通道采样完成后触发，Self_Manage 读到的是
 *           新旧两个采样周期混合的数据，导致误判电池保护阈值
 * 调用链: main() → TM_Collect() → 写 uiAnSmp[0..8]
 */
void TM_Collect(void)
{
    UCHAR i;
    UCHAR ucSmpNum;

    ucSmpNum = SMP_NUM_VCELL;

    for (i = 0u; i < (UCHAR)sizeof(ucSmpSeq); i++)
    {
        /* 充放电电流通道需要16次采样取均值 */
        if ((i == ID_ICHAR) || (i == ID_IDISC))
        {
            ucSmpNum = SMP_NUM_CURRENT;
        }
        else
        {
            ucSmpNum = SMP_NUM_VCELL;
        }

        if (OK == TM_AdSample(ucSmpSeq[i], ucSmpNum))
        {
            uiAnSmp[i] = uiAdTmp;
        }
    }
}

/*
 * 函数名称: TM_AdSample
 * 功能描述: 对指定通道进行多次 ADC 采样，结果存入 uiAdTmp
 * 参数: chanId - 通道号，smpNum - 采样次数
 * 返回值: OK（成功）
 */
uint8 TM_AdSample(UCHAR chanId, UCHAR smpNum)
{
    volatile uchar xdata *adc_sel  = (volatile uchar xdata *)ADC_CHAN_SEL_REG;
    volatile uchar xdata *adc_ctrl = (volatile uchar xdata *)ADC_CTRL_REG;
    volatile uchar xdata *adc_sta  = (volatile uchar xdata *)ADC_STATUS_REG;
    volatile uchar xdata *adc_dat  = (volatile uchar xdata *)ADC_DATA_REG;
    uint32 sum     = 0u;
    UCHAR  k;
    uint32 timeout;

    *adc_sel = (uint32)chanId;

    for (k = 0u; k < smpNum; k++)
    {
        *adc_ctrl = (*adc_ctrl | ADC_START_MASK);
        timeout = 10000u;
        while (((*adc_sta & ADC_DONE_MASK) == 0u) && (timeout > 0u))
        {
            timeout--;
        }
        sum += (*adc_dat & ADC_DATA_MASK);
        *adc_ctrl = (*adc_ctrl & ~ADC_START_MASK);
    }

    uiAdTmp = (uint16)(sum / (uint32)smpNum);
    return OK;
}

/*
 * 函数名称: AhMeter_Calc
 * 功能描述: 安时计量计算，依据充放电电流采样值积分估算电池剩余容量
 */
void AhMeter_Calc(void)
{
    uint16 uiIchar = uiAnSmp[ID_ICHAR];
    uint16 uiIdisc = uiAnSmp[ID_IDISC];
    int32  net_current;
    int32  delta;

    if (uiIchar >= uiIdisc)
    {
        net_current = (int32)(uiIchar - uiIdisc);
        ucOcpCnt    = 0u;
    }
    else
    {
        net_current = -(int32)(uiIdisc - uiIchar);
        ucOdpCnt    = 0u;
    }

    delta = net_current / (int32)AH_SCALE_FACTOR;

    if ((delta > 0) && (lAhCount > (AH_COUNT_MAX - delta)))
    {
        lAhCount = AH_COUNT_MAX;
    }
    else if ((delta < 0) && (lAhCount < (AH_COUNT_MIN - delta)))
    {
        lAhCount = AH_COUNT_MIN;
    }
    else
    {
        lAhCount = lAhCount + delta;
    }
}

/*
 * 函数名称: GetBatSummary
 * 功能描述: 计算电池总电压及最低单体索引，供 RunBatSM 使用
 */
void GetBatSummary(uint16 *p_total_mv, uint8 *p_min_idx)
{
    uint8  i;
    uint16 total   = 0u;
    uint16 min_val = 0xFFFFu;

    if ((p_total_mv == 0) || (p_min_idx == 0)) { return; }

    *p_min_idx = 0u;

    for (i = 0u; i < 7u; i++)
    {
        total = total + uiVcell[i];
        if(uiVcell[i] < min_val)
		{
			min_val    = uiVcell[i];
            *p_min_idx = i;
        }
    }

    *p_total_mv = total;
}

/*
 * 函数名称: RunBatSM
 * 功能描述: 电池状态机，驱动五态转换：
 *           ST_INIT / ST_CHARGE / ST_DISCH / ST_BALANCE / ST_PROTECT
 */
void RunBatSM(void)
{
    uint16     uiIchar = uiAnSmp[ID_ICHAR];
    uint16     uiIdisc = uiAnSmp[ID_IDISC];
    uint16     total_mv;
    uint8      min_idx;
    BatState_e next_state;

    GetBatSummary(&total_mv, &min_idx);

    if (uiVcell[min_idx] < VCELL_UVP_LIMIT)
    {
        next_state = ST_PROTECT;
    }
    else if (ucOvpCnt >= OVP_CONFIRM_CNT)
    {
        next_state = ST_PROTECT;
    }
    else if (uiIchar > (uint16)ICHAR_THRESHOLD)
    {
        if ((uiIchar > uiIdisc) &&
            ((uiIchar - uiIdisc) < (uint16)IBAL_THRESHOLD))
        {
            next_state = ST_BALANCE;
		}
		else
        {
			next_state = ST_CHARGE;
		}
    }
    else if (uiIdisc > (uint16)IDISC_THRESHOLD)
    {
        next_state = ST_DISCH;
    }
    else
    {
        next_state = ST_INIT;
    }

    if ((ucBatMode == ST_PROTECT) && (next_state != ST_PROTECT)
        && (ucOvpCnt == 0u))
    {
        ucBatMode = next_state;
    }
    else if (ucBatMode != ST_PROTECT)
    {
        ucBatMode = next_state;
    }
}

/*
 * 函数名称: MonitorBatHealth
 * 功能描述: 电池健康监测，计算单体电压趋势并检测过压（OVP），更新 ucOvpCnt
 */
void MonitorBatHealth(void)
{
    uint8  i;
    uint8  ovp_flag = FALSE;
    static uint16 prev_vcell[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};

    for (i = 0u; i < 7u; i++)
    {
		aiCellTrend[i] = (int16)((int32)uiVcell[i] - (int32)prev_vcell[i]);
        prev_vcell[i]   = uiVcell[i];
        if (uiVcell[i] > VCELL_OVP_LIMIT) { ovp_flag = TRUE; }
    }

    if (ovp_flag == TRUE)
    {
        if (ucOvpCnt < 0xFFu) { ucOvpCnt++; }
    }
    else
    {
        ucOvpCnt = 0u;
    }
}

/*
 * 函数名称: LogFaultEvent
 * 功能描述: 记录故障事件，读取 Time.S/Time.MS 作为故障时间戳，
 *           写入故障历史环形缓冲区
 */
void LogFaultEvent(uint8 code, uint8 ch)
{
    uint8  idx     = ucFaultIdx % FAULT_HISTORY_DEPTH;

    aFaultHist[idx].time_s     = Time.S;
    aFaultHist[idx].time_ms    = Time.MS;
    aFaultHist[idx].fault_code = code;
    aFaultHist[idx].channel    = ch;

    ucFaultIdx = (ucFaultIdx < 0xFFu)
                       ? (ucFaultIdx + 1u) : 0u;
}

/*
 * 函数名称: CalcCRC16_Smu
 * 功能描述: CRC16 nibble-based 计算（多项式 0x1021），用于遥测帧校验
 */
uint16 CalcCRC16_Smu(const uint8 *buf, uint32 len)
{
    uint16 crc = 0xFFFFu;
    uint32 i;  uint8 j;
    if (buf == 0) { return 0u; }
    for (i = 0u; i < len; i++)
    {
        crc ^= ((uint16)buf[i] << 8u);
        for (j = 0u; j < 8u; j++)
        {
            if (crc & 0x8000u)
                crc = (uint16)((crc << 1u) ^ CRC16_POLY);
            else
                crc = (uint16)(crc << 1u);
        }
    }
    return crc;
}

/*
 * 函数名称: PackSmuFrame
 * 功能描述: 打包 SMU 遥测帧并经 UART 发送
 *           帧结构：同步字(2B) + 序列号(1B) + 保留(1B) +
 *                   单体电压uiVcell[0..6](14B) + 电流通道(4B) +
 *                   lAhCount(4B) + 状态填充(4B) + CRC16(2B)
 *           共 32 字节（SMU_FRAME_LEN）
 */
void PackSmuFrame(void)
{
    uint8  frame[SMU_FRAME_LEN];
    uint16 crc;
    uint8  i;
    uint16 val;
    volatile uchar xdata *uart_sta;
    volatile uchar xdata *uart_dat;

    for (i = 0u; i < (uint8)SMU_FRAME_LEN; i++)
    {
		frame[i] = 0u;
    }

    frame[0] = (uint8)(SMU_SYNC_WORD >> 8u);
    frame[1] = (uint8)(SMU_SYNC_WORD & 0xFFu);
    frame[2] = ucFrameSeq;
    frame[3] = 0u;

    for (i = 0u; i < 7u; i++)
    {
		val = uiVcell[i];
        frame[4u + i * 2u]      = (uint8)(val >> 8u);
        frame[4u + i * 2u + 1u] = (uint8)(val & 0xFFu);
    }

    val       = uiAnSmp[ID_ICHAR];
    frame[18] = (uint8)(val >> 8u);
    frame[19] = (uint8)(val & 0xFFu);

    val       = uiAnSmp[ID_IDISC];
    frame[20] = (uint8)(val >> 8u);
    frame[21] = (uint8)(val & 0xFFu);

    frame[22] = (uint8)(((uint32)lAhCount >> 24u) & 0xFFu);
    frame[23] = (uint8)(((uint32)lAhCount >> 16u) & 0xFFu);
    frame[24] = (uint8)(((uint32)lAhCount >>  8u) & 0xFFu);
    frame[25] = (uint8)((uint32)lAhCount & 0xFFu);

    frame[26] = (uint8)ucBatMode;
    frame[27] = ucOvpCnt;
    frame[28] = (uint8)(uiFrameCnt & 0xFFu);
    frame[29] = (uint8)((uiFrameCnt >> 8u) & 0xFFu);

    crc       = CalcCRC16_Smu(frame, 30u);
    frame[30] = (uint8)(crc >> 8u);
    frame[31] = (uint8)(crc & 0xFFu);

    uart_sta = (volatile uchar xdata *)UART_STATUS_REG;
    uart_dat = (volatile uchar xdata *)UART_DATA_REG;

    for (i = 0u; i < (uint8)SMU_FRAME_LEN; i++)
    {
        while ((*uart_sta & 0x20) == 0u) { ; }
        *uart_dat =frame[i];
    }

    uiFrameCnt++;
    ucFrameSeq++;
}

/*
 * 函数名称: DiagOutput
 * 功能描述: 诊断输出：向 UART 发送电池单体电压
 */
void DiagOutput(void)
{
    volatile uchar xdata *uart_sta;
    volatile uchar xdata *uart_dat;
    uint32 i;

    uart_sta = (volatile uchar xdata *)UART_STATUS_REG;
    uart_dat = (volatile uchar xdata *)UART_DATA_REG;

    for (i = 0u; i < 7u; i++)
    {
		while ((*uart_sta & 0x20) == 0u) { ; }
        *uart_dat =(uiVcell[i] >> 8u) & 0xFFu;
        while ((*uart_sta & 0x20) == 0u) { ; }
        *uart_dat =(uiVcell[i] & 0xFFu);
    }
}





/*
 * 函数名称: Self_Manage
 * 功能描述: 自管理：基于 uiAnSmp 中的电池电压进行过压/欠压保护
 *           由定时器中断调用，处理电池保护逻辑
 */
void Self_Manage(void) using 2
{
    uint8  i;
    uint16 uiTemp;
    uint8  ucCnt;

    /* 读取各电池单体电压，减去 B 值后存入 uiVcell */
    for (i = 0u; i < 7u; i++)
    {
        uiTemp = uiAnSmp[ID_VCELL1 + i];

        if (uiTemp >= uiBcell[i])
        {
            uiVcell[i] = (uint16)(uiTemp - uiBcell[i]);
        }
        else
        {
			uiVcell[i] = 0u;
        }
    }

    /* ODP（过放保护）判断：统计低于阈值的电池单体数量 */
    ucCnt  = 0u;
    uiTemp = (uint16)(uiODPThreshold[0] / 7u);

    for (i = 0u; i < 7u; i++)
    {
		if (uiVcell[i] <= uiTemp)
        {
            ucCnt++;
        }
    }

    /* 触发计数更新 */
    if (ucCnt >= 4u)
    {
        ucOdpCnt++;
    }
    else
    {
        ucOdpCnt = 0u;
    }
}

//--- 中断服务例程：定时器0中断，40ms 周期



/*
 * 函数名称: Timer0_ISR
 * 功能描述: 定时器0中断服务，40ms 周期触发
 *           更新系统时间 Time.MS/Time.S，每秒调用 Self_Manage 进行电池保护
 */
void Timer0_ISR(void) interrupt 1 using 2
{
    volatile uchar xdata *reg;

    reg  = (volatile uchar xdata *)TIMER0_INT_FLAG_REG;
    *reg = 0x01;

    TimerIntCnt++;
    SysTickMs = SysTickMs + 40u;

    /* 更新系统时间（Time.MS 每次加40ms） */
    Time.MS = Time.MS + 40u;
    if (Time.MS >= 1000u)
    {
        Time.MS = Time.MS - 1000u;
        Time.S  = Time.S + 1u;
    }

    ucCnt40ms++;

    if (ONESECOND_CNT == ucCnt40ms)
    {
        ucCnt40ms  = 0u;
		ucCnt1sec++;
        ucArrv1sec = TRUE;
    }

    /* 1秒到达时执行自管理 */
    if (TRUE == ucArrv1sec)
    {
        ucArrv1sec = FALSE;
        Self_Manage();
    }
}

//--- 辅助函数



/*
 * 函数名称: WatchdogFeed
 * 功能描述: 喂看门狗
 */
void WatchdogFeed(void)
{
    WdogCounter++;
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        *(volatile uchar xdata *)0x7FFC = 0xA5;  //feed
        *(volatile uchar xdata *)0x7FFD = 0x5A;
    }
}

/*
 * 函数名称: DelayUs
 * 功能描述: 软件延时微秒
 */
void DelayUs(uint32 us)
{
    volatile uint32 cnt;
    for (cnt = 0u; cnt < us * 10u; cnt++)
    {
        ;
    }
}
