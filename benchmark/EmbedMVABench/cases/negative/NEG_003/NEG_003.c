/************************************************************
 * PDKongWen - PID温控模块（影子变量保护版）
 * 8051 + K型热电偶
 * 四路独立加热区PID温度闭环控制
 * 主循环通过影子变量和临界区保护整体更新加热开启时间
 * 定时器0中断每1ms读取控制输出
 * 主循环每200ms更新PID计算结果
 ************************************************************/

#include <reg51.h>
#include <intrins.h>
/* --- */

/************************************************************
 * 硬件寄存器及地址宏定义
 ************************************************************/
#define ADDR_Heater         0x8000u
#define ADDR_AD590_BASE     0x4000u
#define ADDR_STATUS_REG     0x2000u
#define ADDR_WATCHDOG       0x7FFEu
#define ADDR_LCD_CTRL       0x0800u
#define ADDR_EEPROM_BASE    0x0400u
/* --- */

/* 温度传感器参数 */
#define AD590_SAMPLES       5u
#define HEAT_CHANNELS       4u

/* PID 参数默认值 */
#define PID_P_DEFAULT       12u
#define PID_D_DEFAULT       8u
#define KW_FZ_DEFAULT       5u
/* --- */

/* 加热时间上下限 */
#define HEAT_TIME_MAX       800
#define HEAT_TIME_MIN       0
#define HEAT_CYCLE          1000u
/* --- */

/* 定时器0重载值（1ms 中断） */
#define TIMER0_RELOAD_H     0xFCu
#define TIMER0_RELOAD_L     0x18u

/* 状态标志常量 */
#define FLAG_ACTIVE         0x55u
#define FLAG_INACTIVE       0xAAu
/* --- */

/* 系统常量 */
#define MAIN_LOOP_PERIOD    200u
#define WATCHDOG_PERIOD     50u
#define UART_BAUD_RELOAD    0xFDu
/* --- */

/* AD 采样有效范围 */
#define AD_SAMPLE_MIN       200u
#define AD_SAMPLE_MAX       4000u

/* EEPROM 布局常量 */
#define EEPROM_PID_P_BASE   0x00u
#define EEPROM_PID_D_BASE   0x04u
#define EEPROM_TARGET_BASE  0x08u
#define EEPROM_CRC_OFFSET   0x10u
#define EEPROM_SAVE_PERIOD  50u
/* --- */

/* K型热电偶热电动势补偿系数 */
#define TC_COMP_COEF_A      39u
#define TC_COMP_COEF_B      17u
#define TC_COMP_ZERO_OFFSET 273u
/* --- */

/* PWM 占空比限幅 */
#define PWM_DUTY_MAX        950u
#define PWM_DUTY_MIN        50u
#define PWM_RAMP_STEP       25u
/* --- */

/************************************************************
 * 类型别名
 ************************************************************/
#define uchar unsigned char
#define uint  unsigned int
/*
 * 数据类型定义
 */
typedef unsigned long  uint32;
typedef signed int     int16;
typedef signed long    int32;
/* --- */

/************************************************************
 * 温控模式枚举
 ************************************************************/
typedef enum {
    HEAT_MODE_INIT    = 0u,
    HEAT_MODE_PREHEAT = 2u,
    HEAT_MODE_PID     = 5u,
    HEAT_MODE_HOLD    = 7u,
    HEAT_MODE_FAULT   = 0xFFu
} HeatMode_e;
/* --- */

/************************************************************
 * 控制返回码枚举
 ************************************************************/
typedef enum {
    CTRL_OK          = 0u,
    CTRL_ERR_SENSOR  = 1u,
    CTRL_ERR_RANGE   = 2u,
    CTRL_ERR_HW      = 3u
} CtrlErr_e;
/* --- */

/************************************************************
 * 常量配置表
 ************************************************************/
static const uint code CHANNEL_TARGET_TEMP[HEAT_CHANNELS] = {
    3500u, 3500u, 3600u, 3400u
};
/* --- */

static const uint code CHANNEL_PID_P[HEAT_CHANNELS] = {
    12u, 14u, 12u, 10u
};

static const uint code CHANNEL_PID_D[HEAT_CHANNELS] = {
    8u, 8u, 6u, 10u
};

static const uint code CHANNEL_KW_FZ[HEAT_CHANNELS] = {
    /* 以下进行数据处理和参数校验 */
    5u, 5u, 4u, 6u
};

/* K型热电偶线性化查表（每50度一个点，单位0.1mV） */
static const uchar code HEATER_CHANNEL_MASK[HEAT_CHANNELS] = {
    0xFEu, 0xFDu, 0xFBu, 0xF7u
};
/* --- */

/* PWM占空比限幅表（百分比*10） */
static const uint code HEATER_PWM_LIMIT[HEAT_CHANNELS] = {
    950u, 900u, 850u, 800u
};
/* --- */

/* 各通道热惯性时间常数（秒*10） */
static const uint code CHANNEL_THERMAL_TAU[HEAT_CHANNELS] = {
    120u, 135u, 110u, 145u
};
/* --- */

/************************************************************
 * 全局变量定义
 ************************************************************/

volatile int16 xdata HeatOpenTime[HEAT_CHANNELS];
/* --- */

int16 xdata HeatOpenTime_shadow[HEAT_CHANNELS];

uint xdata AD590Collect_Buf[HEAT_CHANNELS][AD590_SAMPLES + 1];

int16 xdata PKWtmp1[HEAT_CHANNELS];
int16 xdata DKWtmp1[HEAT_CHANNELS];
int16 xdata CJErr[HEAT_CHANNELS];
/* --- */

uchar data Ctlbyte;
uchar data Number_Heater;
uchar data HeatState;
uchar data HeatState_CJ;
/* --- */

uchar xdata PIDStart_Flag[HEAT_CHANNELS];
uchar xdata First_KW_Flag[HEAT_CHANNELS];
uchar xdata HeatOpen_Flag[HEAT_CHANNELS];

uint xdata Count_HeatOpen[HEAT_CHANNELS];
/* --- */

uint xdata MB_Temp;
uint xdata KW_fz;
uchar  xdata PID_P;
uchar  xdata PID_D;

/* 执行业务逻辑 */
uchar data PKtmp;
uchar data fgtmp;
uchar data HeatState_tmp;

volatile uchar xdata SysTickCount;
uchar  xdata WdogCounter;
uchar  xdata DiagCounter;
/* --- */

/* 温控模式状态机变量 */
static uchar xdata KWMode[HEAT_CHANNELS];
static uchar xdata GZCode[HEAT_CHANNELS];
static uint xdata LXErrCnt[HEAT_CHANNELS];
/* --- */

/* 控制错误计数 */
static uint xdata CtlErrCnt[HEAT_CHANNELS];

/* 执行统计变量 */
static volatile uint xdata PIDCnt = 0u;
static volatile uchar  xdata g_last_stable_tick = 0u;
/* --- */

/* EEPROM 持久化计数 */
static uchar xdata g_eeprom_save_ctr = 0u;

/* PWM斜率限制计数 */
static int16 xdata g_pwm_last_out[HEAT_CHANNELS];
/* --- */

/************************************************************
 * 函数前向声明
 ************************************************************/
void SysInit(void);
void TimerInit(void);
void UartInit(void);
void HeaterInit(void);
void ADInit(void);
/* WDT服务 */
void WatchdogFeed(void);
void CollectTemperature(void);
void PDKongWen(void);
void UpdateHeatControl(void);
void DiagOutput(void);
uint ReadAD590(uchar channel);
void DelayMs(uint ms);
void Timer0_INT(void);
/* --- */

static CtrlErr_e ReadChannelTemp(uchar ch, int16 *pTemp);
static void RunHeatModeSM(uchar ch);
static void LoadPIDParams(void);
static void SavePIDParams(void);
static int16 ClampPWMRamp(uchar ch, int16 newVal);
/* --- */

#if 0
static void InjectTestTemp(uchar ch, uint val);
static void VerifyHeatOutput(void);
#endif
/* --- */

/************************************************************
 * main 函数
 ************************************************************/
void main(void)
{
    uchar i;

    /* 执行处理 */
    SysInit();
    TimerInit();
    UartInit();
    HeaterInit();
    /* 功能调用 */
    ADInit();

    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        HeatOpenTime[i]        = (int16)(HEAT_CYCLE / 2u);
        HeatOpenTime_shadow[i] = (int16)(HEAT_CYCLE / 2u);
        PIDStart_Flag[i]       = FLAG_INACTIVE;
    /* 通信数据处理部分 */
        First_KW_Flag[i]       = FLAG_ACTIVE;
        HeatOpen_Flag[i]       = FLAG_INACTIVE;
        Count_HeatOpen[i]      = 0u;
        KWMode[i]              = (uchar)HEAT_MODE_INIT;
        GZCode[i]              = 0u;
        LXErrCnt[i]            = 0u;
        CtlErrCnt[i]           = 0u;
        g_pwm_last_out[i]      = 0;
    }

    Ctlbyte        = 0x00u;
    /* 更新工作状态 */
    HeatState      = 0x00u;
    HeatState_CJ   = FLAG_INACTIVE;
    WdogCounter    = 0u;
    DiagCounter    = 0u;
    SysTickCount   = 0u;
    PIDCnt         = 0u;
    g_last_stable_tick = 0u;
    g_eeprom_save_ctr  = 0u;

    /* 执行处理 */
    LoadPIDParams();

    EA  = 1;
    ET0 = 1;
    /*
     * 此处完成核心计算
     */
    TR0 = 1;

    while (1)
    {
        /* 喂狗 */
        WatchdogFeed();

        CollectTemperature();

        /* 循环处理 */
        for (i = 0u; i < HEAT_CHANNELS; i++)
        {
            RunHeatModeSM(i);
        }

        /* 调用子函数 */
        PDKongWen();

        g_eeprom_save_ctr++;
        if (g_eeprom_save_ctr >= EEPROM_SAVE_PERIOD)
        {
            g_eeprom_save_ctr = 0u;
            /* 调用子函数 */
            SavePIDParams();
        }

        DiagCounter++;
        if (DiagCounter >= 10u)
        {
            DiagCounter = 0u;
            DiagOutput();
        }

        /* 调用子函数 */
        DelayMs(MAIN_LOOP_PERIOD);
    }
}

/************************************************************
 * 初始化函数群
 ************************************************************/

void SysInit(void)
{
    TCON = 0x00u;
    TMOD = 0x00u;
    EA = 0;
    /* --- */

    P0 = 0xFFu;
    P1 = 0x07u;
    P2 = 0xFBu;
    P3 = 0x30u;
    /* --- */

    Number_Heater = 0x0Fu;
    SysTickCount  = 0u;

    /* 喂狗 */
    XBYTE[ADDR_WATCHDOG] = 0xAAu;
}

// TimerInit
// 模块初始化，配置寄存器和外设参数
void TimerInit(void)
{
    /* 位操作 */
    TMOD &= 0xF0u;
    TMOD |= 0x01u;
    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;

    TMOD &= 0x0Fu;
    /* 位字段更新 */
    TMOD |= 0x20u;
    TH1 = UART_BAUD_RELOAD;
    TL1 = UART_BAUD_RELOAD;

    TF0 = 0;
    TF1 = 0;
    /* --- */

    TR1 = 1;
    TR0 = 0;
    ET0 = 0;
}
/* --- */

// UartInit
// 模块初始化，配置寄存器和外设参数
void UartInit(void)
{
    SCON = 0x50u;
    TI   = 0;
    RI   = 0;
}
/* --- */

// HeaterInit
// 模块初始化，配置寄存器和外设参数
void HeaterInit(void)
{
    uchar i;

    Ctlbyte = 0x00u;
    XBYTE[ADDR_Heater] = Ctlbyte;
    /* --- */

    MB_Temp = CHANNEL_TARGET_TEMP[0];
    KW_fz   = KW_FZ_DEFAULT;
    PID_P   = (uchar)PID_P_DEFAULT;
    PID_D   = (uchar)PID_D_DEFAULT;

    /* 遍历处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        Count_HeatOpen[i] = 0u;
    }

    /* 更新工作状态 */
    HeatState     = 0x00u;
    HeatState_tmp = 0x00u;
    HeatState_CJ  = FLAG_INACTIVE;
}
/* --- */

// ADInit
// 模块初始化，配置寄存器和外设参数
void ADInit(void)
{
    uchar i, j;

    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        /* 迭代计算 */
        for (j = 0u; j < (AD590_SAMPLES + 1u); j++)
        {
            AD590Collect_Buf[i][j] = 0u;
        }
        PKWtmp1[i] = 0;
        DKWtmp1[i] = 0;
        CJErr[i]   = 0;
    }
}
/* --- */

/************************************************************
 * 温控模式状态机
 ************************************************************/

static void RunHeatModeSM(uchar ch)
{
    int16  curTemp;
    CtrlErr_e ret;

    ret = ReadChannelTemp(ch, &curTemp);

    /* 根据类型分支处理 */
    switch ((HeatMode_e)KWMode[ch])
    {
        case HEAT_MODE_INIT:
            if (ret == CTRL_OK)
            {
                KWMode[ch]        = (uchar)HEAT_MODE_PREHEAT;
                LXErrCnt[ch]      = 0u;
            }
            else
            {
                CtlErrCnt[ch]++;
                GZCode[ch] = (uchar)ret;
            }
            break;
    /* --- */

        case HEAT_MODE_PREHEAT:
            if (ret != CTRL_OK)
            {
                LXErrCnt[ch]++;
                CtlErrCnt[ch]++;
                if (LXErrCnt[ch] >= 5u)
                {
                    KWMode[ch]   = (uchar)HEAT_MODE_FAULT;
                    GZCode[ch]   = (uchar)ret;
                }
            }
            else
            {
                LXErrCnt[ch] = 0u;
                /* 检查条件 */
                if (curTemp >= ((int16)MB_Temp - (int16)KW_fz))
                {
                    KWMode[ch] = (uchar)HEAT_MODE_PID;
                }
            }
            break;
    /* --- */

        case HEAT_MODE_PID:
            if (ret != CTRL_OK)
            {
                LXErrCnt[ch]++;
                CtlErrCnt[ch]++;
                if (LXErrCnt[ch] >= 10u)
                {
                    KWMode[ch]   = (uchar)HEAT_MODE_FAULT;
                    GZCode[ch]   = (uchar)ret;
                }
            }
            else
            {
                LXErrCnt[ch] = 0u;
    /* 硬件接口操作 */
                if ((curTemp >= ((int16)MB_Temp - 1))
                 && (curTemp <= ((int16)MB_Temp + 1)))
                {
                    g_last_stable_tick = SysTickCount;
                    KWMode[ch]         = (uchar)HEAT_MODE_HOLD;
                }
            }
            break;

        case HEAT_MODE_HOLD:
            if (ret != CTRL_OK)
            {
    /* 异常检测与恢复 */
                LXErrCnt[ch]++;
                CtlErrCnt[ch]++;
                if (LXErrCnt[ch] >= 8u)
                {
                    KWMode[ch]   = (uchar)HEAT_MODE_FAULT;
                    GZCode[ch]   = (uchar)ret;
                }
            }
            else
            {
                LXErrCnt[ch] = 0u;
                /* 参数检查 */
                if ((curTemp < ((int16)MB_Temp - (int16)KW_fz))
                 || (curTemp > ((int16)MB_Temp + (int16)KW_fz)))
                {
                    KWMode[ch] = (uchar)HEAT_MODE_PID;
                }
            }
            break;
    /* --- */

        case HEAT_MODE_FAULT:
            CtlErrCnt[ch]++;
            if (LXErrCnt[ch] > 0u)
            {
                LXErrCnt[ch]--;
            }
            if (LXErrCnt[ch] == 0u)
            {
                GZCode[ch] = 0u;
                KWMode[ch] = (uchar)HEAT_MODE_INIT;
            }
            break;
    /* --- */

        default:
            KWMode[ch] = (uchar)HEAT_MODE_INIT;
            break;
    }
}
/* --- */

/************************************************************
 * 温度读取中间层
 ************************************************************/

static CtrlErr_e ReadChannelTemp(uchar ch, int16 *pTemp)
{
    uint raw;
    /* --- */

    raw = AD590Collect_Buf[ch][AD590_SAMPLES];

    if ((raw < AD_SAMPLE_MIN) || (raw > AD_SAMPLE_MAX))
    {
        CtlErrCnt[ch]++;
        /* 输出处理结果 */
        return CTRL_ERR_SENSOR;
    }

    *pTemp = (int16)raw;
    return CTRL_OK;
}
/* --- */

/* PWM输出变化率限幅 */
static int16 ClampPWMRamp(uchar ch, int16 newVal)
{
    int16 delta;
    (void)HEATER_PWM_LIMIT;
    /* --- */

    delta = newVal - g_pwm_last_out[ch];

    if (delta > (int16)PWM_RAMP_STEP)
    {
        newVal = g_pwm_last_out[ch] + (int16)PWM_RAMP_STEP;
    }
    else if (delta < -((int16)PWM_RAMP_STEP))
    {
    /* 系统状态更新 */
        newVal = g_pwm_last_out[ch] - (int16)PWM_RAMP_STEP;
    }

    g_pwm_last_out[ch] = newVal;
    return newVal;
}
/* --- */

/************************************************************
 * 调用层函数（主循环调用层）
 ************************************************************/

void CollectTemperature(void)
{
    uchar  i, j;
    uint adVal;

    /* 迭代计算 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        for (j = 0u; j < AD590_SAMPLES; j++)
        {
            AD590Collect_Buf[i][j] = AD590Collect_Buf[i][j + 1u];
        }
        adVal = ReadAD590(i);
        AD590Collect_Buf[i][AD590_SAMPLES] = adVal;
    }
    /* --- */

    PIDCnt++;
}

// UpdateHeatControl
// 数据更新
void UpdateHeatControl(void)
{
    XBYTE[ADDR_Heater]    = Ctlbyte;
    XBYTE[ADDR_STATUS_REG] = HeatState;
}
/* --- */

// DiagOutput
// 功能处理
void DiagOutput(void)
{
    uchar i;

    /* 遍历处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        while (!TI) { ; }
        TI = 0;
        SBUF = (uchar)((uint)HeatOpenTime[i] >> 8u);
        /* 循环处理 */
        while (!TI) { ; }
        TI = 0;
        SBUF = (uchar)((uint)HeatOpenTime[i] & 0xFFu);
    }

    /* 循环处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        while (!TI) { ; }
        TI = 0;
        SBUF = KWMode[i];
    }

    /* 遍历处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        while (!TI) { ; }
        TI = 0;
    /*
     * 定时任务处理
     */
        SBUF = GZCode[i];
    }

    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        /* 遍历处理 */
        while (!TI) { ; }
        TI = 0;
        SBUF = (uchar)(LXErrCnt[i] >> 8u);
        while (!TI) { ; }
        TI = 0;
        SBUF = (uchar)(LXErrCnt[i] & 0xFFu);
    }

    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        /* 遍历处理 */
        while (!TI) { ; }
        TI = 0;
        SBUF = (uchar)(CtlErrCnt[i] >> 8u);
        while (!TI) { ; }
        TI = 0;
        SBUF = (uchar)(CtlErrCnt[i] & 0xFFu);
    }

    /* 迭代计算 */
    while (!TI) { ; }
    TI = 0;
    SBUF = (uchar)(PIDCnt >> 8u);
    while (!TI) { ; }
    TI = 0;
    SBUF = (uchar)(PIDCnt & 0xFFu);
}
/* --- */

/**
 * @brief PD控温计算（影子变量保护版）
 * @param 无（内部遍历全部通道）
 * PID计算结果先写入影子变量 HeatOpenTime_shadow[4]，
 * 所有通道计算完成后，在定时器0中断屏蔽的临界区内
 * 将影子变量整体复制到 HeatOpenTime[4]，
 * 保证定时器0中断读取到的 HeatOpenTime 始终处于完整一致状态。
 * 调用链: main() -> PDKongWen()
 */
void PDKongWen(void)
{
    uchar    i;
    int16    sampledTemp;
    int16    newTime;
    CtrlErr_e readRet;
    (void)CHANNEL_THERMAL_TAU;

    /* 遍历处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        readRet = ReadChannelTemp(i, &sampledTemp);
        if (readRet != CTRL_OK)
        {
            CtlErrCnt[i]++;
            continue;
        }

        PKWtmp1[i] = sampledTemp;

        DKWtmp1[i] = (int16)(AD590Collect_Buf[i][AD590_SAMPLES]
    /* 数据打包发送 */
                           - AD590Collect_Buf[i][0]);

        CJErr[i] = (int16)MB_Temp - PKWtmp1[i];

        /* 条件判断 */
        if ((PKWtmp1[i] >= ((int16)MB_Temp - (int16)KW_fz))
         && (PKWtmp1[i] <= ((int16)MB_Temp + (int16)KW_fz)))
        {
            PIDStart_Flag[i] = FLAG_ACTIVE;
    /* --- */

            newTime = HeatOpenTime_shadow[i]
                    - (int16)((int16)CJErr[i]   * (int16)PID_P)
                    + (int16)((int16)DKWtmp1[i] * (int16)PID_D);

            if (newTime > (int16)HEAT_TIME_MAX)
            {
                newTime = (int16)HEAT_TIME_MAX;
            }
            if (newTime < (int16)HEAT_TIME_MIN)
            {
                newTime = (int16)HEAT_TIME_MIN;
            }

            HeatOpenTime_shadow[i] = newTime;

    /* 接收缓冲区解析 */
            if (First_KW_Flag[i] == FLAG_ACTIVE)
            {
                First_KW_Flag[i] = FLAG_INACTIVE;
            }
        }
        else if (PKWtmp1[i] > ((int16)MB_Temp + (int16)KW_fz))
        {
            PKtmp = (uchar)(0x01u << i);
            PKtmp = (uchar)(~PKtmp);
            Ctlbyte = (uchar)(Ctlbyte & PKtmp);
    /* --- */

            PIDStart_Flag[i] = FLAG_INACTIVE;

            PKtmp = (uchar)(0x01u << (i + 4u));
            PKtmp = (uchar)(~PKtmp);
            /* 更新工作状态 */
            HeatState    = (uchar)(HeatState & PKtmp);
            HeatState_CJ = FLAG_ACTIVE;
        }
        else
        {
            PKtmp = (uchar)(0x01u << i);
            Ctlbyte = (uchar)(Ctlbyte | PKtmp);
            Ctlbyte = (uchar)(Ctlbyte & Number_Heater);

            PIDStart_Flag[i] = FLAG_INACTIVE;
            /* 状态机转移 */
            HeatState        = (uchar)((HeatState & 0x0Fu) | (Ctlbyte << 4u));
            First_KW_Flag[i] = FLAG_ACTIVE;
            HeatState_CJ     = FLAG_ACTIVE;
        }
    }

    ET0 = 0;
    /* 数据填充 */
    HeatOpenTime[0] = HeatOpenTime_shadow[0];
    HeatOpenTime[1] = HeatOpenTime_shadow[1];
    HeatOpenTime[2] = HeatOpenTime_shadow[2];
    HeatOpenTime[3] = HeatOpenTime_shadow[3];
    ET0 = 1;

    /* 执行处理 */
    UpdateHeatControl();
}

/**
 * @brief 定时器0中断服务 1ms周期
 * 读取各通道加热开启时间，控制继电器输出
 * 中断号: 1，使用寄存器组2
 */
void Timer0_INT(void) interrupt 1 using 2
{
    uchar i;
    /* --- */

    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;

    SysTickCount++;
    _nop_();

    /* 迭代计算 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        Count_HeatOpen[i]++;
        if (Count_HeatOpen[i] >= HEAT_CYCLE)
        {
            Count_HeatOpen[i] = 0u;
            HeatOpen_Flag[i]  = FLAG_INACTIVE;
        }
    }

    /* 循环处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        if (PIDStart_Flag[i] == FLAG_ACTIVE)
        {
            /* 条件判断 */
            if (Count_HeatOpen[i] > (uint)HeatOpenTime[i])
            {
                fgtmp   = (uchar)(0x01u << i);
                Ctlbyte = (uchar)(Ctlbyte | fgtmp);
                Ctlbyte = (uchar)(Ctlbyte & Number_Heater);
                XBYTE[ADDR_Heater] = Ctlbyte;
                HeatState_tmp = (uchar)((HeatState_tmp & 0x0Fu)
                                      | (Ctlbyte << 4u));
                HeatOpen_Flag[i] = FLAG_ACTIVE;
            }
            else
            {
                fgtmp   = (uchar)(0x01u << i);
                fgtmp   = (uchar)(~fgtmp);
    /* 参数范围限制 */
                Ctlbyte = (uchar)(Ctlbyte & fgtmp);
                Ctlbyte = (uchar)(Ctlbyte & Number_Heater);
                XBYTE[ADDR_Heater] = Ctlbyte;
                HeatOpen_Flag[i]   = FLAG_INACTIVE;
            }
        }
    }

    /* 更新工作状态 */
    HeatState = (uchar)((HeatState & 0x0Fu) | (HeatState_tmp & 0xF0u));
}

/************************************************************
 * EEPROM 参数持久化
 ************************************************************/

static void LoadPIDParams(void)
{
    uchar i;
    uchar crc_calc;
    uchar crc_stored;
    uchar param_p[HEAT_CHANNELS];
    uchar param_d[HEAT_CHANNELS];
    uint target_temp;

    crc_calc = 0u;

    /* 迭代计算 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        param_p[i] = XBYTE[ADDR_EEPROM_BASE + EEPROM_PID_P_BASE + i];
        crc_calc   = (uchar)(crc_calc + param_p[i]);
    }

    /* 迭代计算 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        param_d[i] = XBYTE[ADDR_EEPROM_BASE + EEPROM_PID_D_BASE + i];
        crc_calc   = (uchar)(crc_calc + param_d[i]);
    }
    /* --- */

    {
        uchar hi = XBYTE[ADDR_EEPROM_BASE + EEPROM_TARGET_BASE];
        uchar lo = XBYTE[ADDR_EEPROM_BASE + EEPROM_TARGET_BASE + 1u];
        target_temp = (uint)((uint)hi << 8u) | (uint)lo;
        crc_calc    = (uchar)(crc_calc + hi + lo);
    }
    /* --- */

    crc_stored = XBYTE[ADDR_EEPROM_BASE + EEPROM_CRC_OFFSET];

    if (crc_calc != crc_stored)
    {
        /* 遍历处理 */
        for (i = 0u; i < HEAT_CHANNELS; i++)
        {
            param_p[i] = (uchar)CHANNEL_PID_P[i];
            param_d[i] = (uchar)CHANNEL_PID_D[i];
        }
        target_temp = CHANNEL_TARGET_TEMP[0];
    }

    /* 循环处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        if (param_p[i] == 0u) { param_p[i] = (uchar)CHANNEL_PID_P[i]; }
        if (param_d[i] == 0u) { param_d[i] = (uchar)CHANNEL_PID_D[i]; }
    }
    /* --- */

    PID_P   = param_p[0];
    PID_D   = param_d[0];
    MB_Temp = target_temp;
}

// SavePIDParams
// PID控制计算
static void SavePIDParams(void)
{
    uchar  i;
    uchar  crc_calc;
    /* 控制量计算输出 */
    uchar  hi, lo;

    crc_calc = 0u;

    /* 迭代计算 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        XBYTE[ADDR_EEPROM_BASE + EEPROM_PID_P_BASE + i] = PID_P;
        crc_calc = (uchar)(crc_calc + PID_P);
    }

    /* 循环处理 */
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        XBYTE[ADDR_EEPROM_BASE + EEPROM_PID_D_BASE + i] = PID_D;
        crc_calc = (uchar)(crc_calc + PID_D);
    }
    /* --- */

    hi = (uchar)(MB_Temp >> 8u);
    lo = (uchar)(MB_Temp & 0xFFu);
    XBYTE[ADDR_EEPROM_BASE + EEPROM_TARGET_BASE]      = hi;
    XBYTE[ADDR_EEPROM_BASE + EEPROM_TARGET_BASE + 1u] = lo;
    crc_calc = (uchar)(crc_calc + hi + lo);
    /* --- */

    XBYTE[ADDR_EEPROM_BASE + EEPROM_CRC_OFFSET] = crc_calc;

    {
        uchar status_cur;
        status_cur  = XBYTE[ADDR_STATUS_REG];
        status_cur  = (uchar)((status_cur & 0x0Fu) | 0x80u);
        XBYTE[ADDR_STATUS_REG] = status_cur;
    }

    /* 执行处理 */
    DelayMs(10u);
}

/************************************************************
 * 辅助函数
 ************************************************************/

/**
 * @brief 读取AD590温度传感器
 * @param channel 通道号（0~3）
 * @return 12位ADC原始值
 */
uint ReadAD590(uchar channel)
{
    uint addrOffset;
    uint rawVal;
    uchar  hi, lo;
    /* --- */

    addrOffset = (uint)channel * 2u;
    hi         = XBYTE[ADDR_AD590_BASE + addrOffset];
    lo         = XBYTE[ADDR_AD590_BASE + addrOffset + 1u];
    rawVal     = (uint)((uint)hi << 4u) | ((uint)lo >> 4u);
    return rawVal;
}
/* --- */

// WatchdogFeed
// 看门狗喂狗
void WatchdogFeed(void)
{
    WdogCounter++;
    /* 喂狗 */
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        XBYTE[ADDR_WATCHDOG] = 0xAAu;
        /* 喂狗 */
        XBYTE[ADDR_WATCHDOG] = 0x55u;
    }
}

// DelayMs
// 软件延时
void DelayMs(uint ms)
{
    uint i, j;
    /* 迭代计算 */
    for (i = 0u; i < ms; i++)
    {
        for (j = 0u; j < 108u; j++)
        {
            _nop_();
            /* 功能调用 */
            _nop_();
            _nop_();
            _nop_();
        }
    }
}
/* --- */

#if 0
//PDKongWen debug
static void InjectTestTemp(uchar ch, uint val)
{
    uchar j;
    /* 遍历处理 */
    for (j = 0u; j <= AD590_SAMPLES; j++)
    {
        AD590Collect_Buf[ch][j] = val;
    }
}
/* --- */

// VerifyHeatOutput
// 数据校验
static void VerifyHeatOutput(void)
{
    uchar i;
    _nop_();
    for (i = 0u; i < HEAT_CHANNELS; i++)
    {
        /* 检查条件 */
        if (HeatOpenTime[i] < (int16)HEAT_TIME_MIN)
        {
            HeatOpenTime[i] = (int16)HEAT_TIME_MIN;
        }
        /* 检查条件 */
        if (HeatOpenTime[i] > (int16)HEAT_TIME_MAX)
        {
            HeatOpenTime[i] = (int16)HEAT_TIME_MAX;
        }
    }
}
#endif
