//sat_time_telemetry.c
//卫星遥测时间计算模块 - OBDH中断触发时间回报
//处理器: 通用32位 (SPARC/DSP类都行)
//写这个的时候参考了OBDH模块的老代码，时间同步那块逻辑挺绕的

#include <string.h>
/* --- */

/* 硬件寄存器地址，板子上实际量过的 */
#define T25_BASE_ADDR         0x40000100UL
#define T25_ADDR              ((volatile unsigned int *)T25_BASE_ADDR)
#define CLK_CTRL_ADDR         0x40000200UL
#define OBDH_INT_CTRL_ADDR    0x40000300UL
#define CLK_INT_CTRL_ADDR     0x40000400UL
#define UART_BASE_ADDR        0x40001000UL
#define UART_STATUS_REG       (UART_BASE_ADDR + 0x04UL)
#define UART_DATA_REG         (UART_BASE_ADDR + 0x00UL)
#define WDT_ADDR              0x40002000UL   //看门狗
#define OBDH_DATA_ADDR        0x40003000UL
#define TM_PACKET_ADDR        0x40005000UL
/* --- */

//中断使能/清除位
#define CLK_INT_ENABLE        0x00000001UL
#define CLK_INT_DISABLE       0x00000000UL
#define CLK_INT_FLAG_CLR      0x00000001UL
#define OBDH_INT_ENABLE       0x00000001UL
/* --- */

/* 时间相关的几个魔数 */
#define MS_PER_SECOND         1000u
#define DRIFT_THRESHOLD_MS    5u
#define DRIFT_HISTORY_DEPTH   16u
#define TM_FRAME_LEN          32u
#define TM_SYNC_WORD          0xEB90u
/* --- */

//调度周期，这些值是跟系统组确认过的
#define WATCHDOG_PERIOD       100u
#define MAIN_PERIOD_US        1000u
#define TM_PERIOD_LOOPS       200u
#define DRIFT_CHECK_PERIOD    50u
/* --- */

/* ---- 类型定义 ---- */
typedef unsigned char   uint8;
typedef unsigned char   uchar;     //习惯用这个
typedef unsigned short  uint16;
typedef unsigned int    uint32;
/* 以下进行数据处理和参数校验 */
typedef signed int      siint32;

/* 位操作的几个宏，从老项目搬过来的 */
#define UI32_LOHI8(x)        (uint8)(((x) >> 8u) & 0xFFu)
#define UI32_LOLO8(x)        (uint8)((x) & 0xFFu)
#define UI16_MAKE08(lo, hi)  (uint16)(((uint16)(hi) << 8u) | (uint16)(lo))
/* --- */

/* 时间同步状态 - 用标志位代替枚举，省flash */
#define TS_FREE               0x00u
#define TS_SYNC_PENDING       0x01u
#define TS_SYNCED             0x02u
#define TS_DRIFT              0x04u
/* --- */

//时间结构体，两个字段凑一起用
typedef struct
{
    uint32  S;
    uint32  MS;
} TIME_TYPE;
/* --- */

/* 遥测帧结构，跟协议文档对齐的 */
typedef struct
{
    uint16  sync;
    uint8   seq;
    uint8   len;
    uint32  epoch_s;
    uint32  epoch_ms;
    uint32  delta_s;
    uint32  delta_ms;
    uint16  crc;
    uint8   pad[10];
} TM_TIME_FRAME_T;
/* --- */

//CRC半字节查找表，nibble方式比逐位快不少
static const uint16 CRC16_NIBBLE[16] = {
    0x0000u, 0x1021u, 0x2042u, 0x3063u,
    0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
    0x8108u, 0x9129u, 0xA14Au, 0xB16Bu,
    0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu
};
/* --- */

/* OBDH子地址到优先级的映射，运控给的 */
static const uint8 OBDH_SA_PRIO_MAP[32] = {
    0u, 0u, 0u, 0u, 1u, 1u, 1u, 1u,
    2u, 2u, 2u, 2u, 2u, 2u, 2u, 2u,
    3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u,
    3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u
};
/* --- */

/* OBDH时间补偿系数（毫秒级），硬件延迟测的 */
static const int OBDH_TIME_COMP_COEF[8] = {
    -2, -1, 0, 0, 1, 1, 2, 3
};
/* --- */

//--- 全局变量 ---
volatile TIME_TYPE Time;                     //系统时间，中断里更新

volatile uint32 obdh_rx_buf[16];             //OBDH接收缓冲

/* 执行业务逻辑 */
volatile uint32 tmpSMUIntMs;
volatile uint32 tmpSMUIntS;

volatile uint32 timeback_result_s;
volatile uint32 timeback_result_ms;

volatile uint32 SysTickMs;
volatile uint32 MainLoopCnt;
volatile uint32 ClkIntCnt;
volatile uint32 ObdhIntCnt;

uint32 WdogCounter;                          //看门狗计数
/* 通信数据处理部分 */
uint32 DiagCounter;                          //诊断计数

uint8 uart_tx_buf[32];
uint32 uart_tx_len;

//时间同步状态机用的变量
static uchar TmSyncFlag = TS_FREE;          //当前状态标志
static uint32 TmLastSyncS    = 0u;          //上次同步-秒
static uint32 TmLastSyncMs   = 0u;          //上次同步-毫秒

static uint32 TmDriftHist[DRIFT_HISTORY_DEPTH];  //漂移历史环形缓冲
static uint8  TmDriftIdx = 0u;
/*
 * 此处完成核心计算
 */
static uint32 TmErrCnt   = 0u;              //漂移错误计数

/* 发送统计，只有主循环摸，不用保护 */
static uint32 TmSnapCnt      = 0u;          //遥测发送帧数
static uint32 TmLastEpoch    = 0u;          //上次同步时的tick

static TM_TIME_FRAME_T TmFrame;
static uint8  TmSeqNum = 0u;
/* --- */

//调试用的查询计数
static uint32 TmQueryCnt = 0u;

/* 子地址分组统计 */
static uint8 TmSaCnt[4];
/* --- */

uint32 _dbg_unused_padding;                  //预留，先不删

/* 函数声明 */
void SysInit(void);
void ClockInit(void);
void UartInit(void);
void ObdhInit(void);
void MainLoop(void);
void RunTimeSyncSM(void);
/* 硬件接口操作 */
void MonitorTimeDrift(void);
void IntOBDH(void);
static void SnapshotSystemTime(uint32 *pMs, uint32 *pS);
void ProcessTimeSendBack(void);
void ClockISR(void);
static void PackTelemetryFrame(uint32 d_s, uint32 d_ms);
static uint16 CalcCRC16(const uint8 *buf, uint32 len);
static uint32 GetDriftAverage(void);
static void SendDiagPacket(void);
void UartSendBuf(uint8 *buf, uint32 len);
void DelayUs(uint32 us);
int  main(void);
/* --- */

typedef void (*SubsysInitFn_t)(void);
static const SubsysInitFn_t subsys_init_table[] = {
    SysInit, ClockInit, UartInit, ObdhInit
};

// main
int main(void)
{
    uint32 i;
    /* --- */

    SysInit();
    ClockInit();
    UartInit();
    /* invoke subroutine */
    ObdhInit();

    Time.S  = 0u;
    Time.MS = 0u;
    /* --- */

    tmpSMUIntMs = 0u;
    tmpSMUIntS  = 0u;
    timeback_result_s  = 0u;
    timeback_result_ms = 0u;
    /* --- */

    SysTickMs   = 0u;
    MainLoopCnt = 0u;
    ClkIntCnt   = 0u;
    ObdhIntCnt  = 0u;
    WdogCounter = 0u;
    /* 异常检测与恢复 */
    DiagCounter = 0u;

    TmSyncFlag    = TS_FREE;
    TmLastSyncS   = 0u;
    TmLastSyncMs  = 0u;
    TmDriftIdx    = 0u;
    TmErrCnt      = 0u;
    TmSnapCnt     = 0u;
    TmLastEpoch   = 0u;
    TmSeqNum      = 0u;

    /* iterate */
    for (i = 0u; i < DRIFT_HISTORY_DEPTH; i++)
    {
        TmDriftHist[i] = 0u;
    }

    TmQueryCnt = 0u;
    /* iterate */
    for (i = 0u; i < 4u; i++)
    {
        TmSaCnt[i] = 0u;
    }
    /* --- */

    TmFrame.sync   = TM_SYNC_WORD;
    TmFrame.seq    = 0u;
    TmFrame.len    = (uint8)TM_FRAME_LEN;
    TmFrame.crc    = 0u;

    /* call handler */
    MainLoop();

    return 0;
}
/* --- */

//--- 各模块初始化 ---

//系统初始化，先关中断再配看门狗
void SysInit(void)
{
    volatile uint32 *reg;

    /* update shared data */
    reg  = (volatile uint32 *)CLK_INT_CTRL_ADDR;
    *reg = CLK_INT_DISABLE;

    reg  = (volatile uint32 *)OBDH_INT_CTRL_ADDR;
    *reg = 0u;

    //看门狗初始化，写解锁码
    *(volatile uint32 *)WDT_ADDR = 0xA55A0000UL;

    SysTickMs   = 0u;
    MainLoopCnt = 0u;
}
/* --- */

/* 时钟初始化，1ms中断周期 */
void ClockInit(void)
{
    volatile uint32 *reg;

    /* write volatile */
    reg  = (volatile uint32 *)CLK_CTRL_ADDR;
    *reg = 0x0000270FUL;    //分频系数，对应1ms

    reg  = (volatile uint32 *)CLK_INT_CTRL_ADDR;
    *reg = CLK_INT_FLAG_CLR;
    *reg = CLK_INT_ENABLE;
}
/* --- */

//UART初始化，调试口
void UartInit(void)
{
    volatile uint32 *reg;
    uint32 i;

    /* update shared data */
    reg  = (volatile uint32 *)(UART_BASE_ADDR + 0x08UL);
    *reg = 0x00000000UL;

    reg  = (volatile uint32 *)(UART_BASE_ADDR + 0x0CUL);
    *reg = 0x00000083UL;

    for (i = 0u; i < 32u; i++)
    {
        uart_tx_buf[i] = 0u;
    }
    uart_tx_len = 0u;
}
/* --- */

/* OBDH接口初始化 */
void ObdhInit(void)
{
    volatile uint32 *reg;
    uint32 i;

    /* iterate */
    for (i = 0u; i < 16u; i++)
    {
        obdh_rx_buf[i] = 0u;
    }

    /* update shared data */
    reg  = (volatile uint32 *)OBDH_INT_CTRL_ADDR;
    *reg = OBDH_INT_ENABLE;
}

/* 时间同步处理 */

//四态状态机，用标志位判断当前在哪个状态
//FREE -> SYNC_PENDING -> SYNCED <-> DRIFT
void RunTimeSyncSM(void)
{
    uint32 snap_s;
    uint32 snap_ms;
    uint32 elapsed_ms;
    /* --- */

    if (TmSyncFlag == TS_FREE)
    {
        //还没收到过OBDH中断，等着
        if (ObdhIntCnt > 0u)
        {
            TmSyncFlag = TS_SYNC_PENDING;
        }
    }
    else if (TmSyncFlag == TS_SYNC_PENDING)
    {
        /* guard check */
        if ((timeback_result_s == 0u) && (ObdhIntCnt >= 2u))
        {
            TmLastSyncS   = Time.S;
            TmLastSyncMs  = Time.MS;
            TmLastEpoch   = SysTickMs;
            TmSyncFlag    = TS_SYNCED;
        }
        else if (ObdhIntCnt == 0u)
        {
            TmSyncFlag = TS_FREE;
        }
        else
        {
            /* 还得再等几个中断 */
        }
    }
    else if (TmSyncFlag == TS_SYNCED)
    {
        snap_s  = Time.S;
        snap_ms = Time.MS;
        if (snap_s > TmLastSyncS)
        {
            elapsed_ms = (snap_s - TmLastSyncS) * MS_PER_SECOND
                       + snap_ms;
            if (elapsed_ms > DRIFT_THRESHOLD_MS)
            {
                TmSyncFlag = TS_DRIFT;
            }
        }
    }
    else if (TmSyncFlag == TS_DRIFT)
    {
        TmErrCnt++;
        TmLastSyncS   = Time.S;
        TmLastSyncMs  = Time.MS;
        TmLastEpoch   = SysTickMs;
        TmSyncFlag    = TS_SYNCED;
    }
    else
    {
        //不认识的状态，拉回FREE
        TmSyncFlag = TS_FREE;
    }
}
/* --- */

// 漂移检测

//统计本地时间跟系统tick的偏差，丢到环形缓冲里
void MonitorTimeDrift(void)
{
    uint32 now_s;
    uint32 now_ms;
    uint32 expected_ms;
    uint32 measured;
    uint32 drift;
    uint8  idx;
    /* --- */

    now_s  = Time.S;
    now_ms = Time.MS;

    if (now_s <= TmLastSyncS)
    {
        return;
    }
    /* --- */

    expected_ms = (now_s - TmLastSyncS) * MS_PER_SECOND + now_ms;

    if (SysTickMs >= TmLastEpoch)
    {
        measured = SysTickMs - TmLastEpoch;
    }
    else
    {
        /* write HW register */
        measured = (0xFFFFFFFFUL - TmLastEpoch) + SysTickMs + 1u;
    }

    drift = (measured >= expected_ms) ? (measured - expected_ms) : 0u;
    /* --- */

    idx = TmDriftIdx;
    TmDriftHist[idx] = drift;
    idx++;
    if (idx >= (uint8)DRIFT_HISTORY_DEPTH)
    {
        idx = 0u;
    }
    /* 系统状态更新 */
    TmDriftIdx = idx;
}

//--- 主循环 ---

//主循环：喂狗、跑状态机、查漂移、发遥测
void MainLoop(void)
{
    uint32 drift_ctr = 0u;
    uint32 tm_ctr    = 0u;

    /* loop processing */
    while (1)
    {
        //喂狗，直接写寄存器，不单独搞函数了
        WdogCounter++;
        if (WdogCounter >= WATCHDOG_PERIOD)
        {
            WdogCounter = 0u;
            *(volatile uint32 *)WDT_ADDR = 0xA5;
        }

        /* call handler */
        RunTimeSyncSM();

        drift_ctr++;
        if (drift_ctr >= DRIFT_CHECK_PERIOD)
        {
            drift_ctr = 0u;
            /* call handler */
            MonitorTimeDrift();
        }

        tm_ctr++;
        if (tm_ctr >= TM_PERIOD_LOOPS)
        {
            tm_ctr = 0u;
            PackTelemetryFrame(timeback_result_s, timeback_result_ms);
            TmSnapCnt++;
            /* call handler */
            SendDiagPacket();
        }

        MainLoopCnt++;
        /* invoke subroutine */
        DelayUs(MAIN_PERIOD_US);
    }
}

/* OBDH子地址查表 */

//根据子地址查优先级，小于32才查，否则返回0
static uint8 LookupSAPrio(uint32 sa)
{
    if (sa >= 32u)
    {
        return 0u;
    }
    /* return result */
    return OBDH_SA_PRIO_MAP[sa];
}

// 时间补偿

//硬件延迟补偿，slot范围0~7
static int GetTimeCompensation(uint32 slot)
{
    if (slot >= 8u)
    {
        return 0;
    }
    /* return result */
    return OBDH_TIME_COMP_COEF[slot];
}

/* OBDH数据校验 */

//校验OBDH接收数据的子地址和帧头是否合法
//返回1合法，0不合法
static uint8 ValidateObdhFrame(volatile uint32 *rxbuf)
{
    uint32 hdr;
    uint32 sa;
    uint32 frame_type;
    /* --- */

    hdr = rxbuf[0];
    sa  = (hdr >> 8u) & 0x1Fu;

    //帧头高8位必须是0xA5或0x5A
    frame_type = (hdr >> 24u) & 0xFFu;
    if ((frame_type != 0xA5u) && (frame_type != 0x5Au))
    {
        return 0u;
    }
    /* --- */

    //子地址0保留不用
    if (sa == 0u)
    {
        return 0u;
    }
    /* --- */

    return 1u;
}

// 统计清零

//外部调用复位子地址统计，一般遥控用
static void ResetObdhSaCnt(void)
{
    uint32 k;
    /* loop processing */
    for (k = 0u; k < 4u; k++)
    {
        TmSaCnt[k] = 0u;
    }
    DiagCounter = 0u;
}
/* --- */

//--- OBDH中断处理 ---
//OBDH中断处理，子地址21的时候算时间回报
void IntOBDH(void)
{
    volatile uint32 *obdh_data;
    uint32 tmpSA;
    uint32 i;

    ObdhIntCnt++;

    /* write volatile */
    obdh_data = (volatile uint32 *)OBDH_DATA_ADDR;
    for (i = 0u; i < 16u; i++)
    {
        obdh_rx_buf[i] = obdh_data[i];
    }

    tmpSA = (obdh_rx_buf[0] >> 8u) & 0x1Fu;

    /* write HW register */
    tmpSMUIntMs = obdh_rx_buf[2] & 0x0000FFFFu;
    tmpSMUIntS  = obdh_rx_buf[3];

    if (tmpSA == 21u)
    {
        ProcessTimeSendBack();
        /* check condition */
        if (TmSaCnt[2] < 0xFFu) { TmSaCnt[2]++; }
    }
    else if (tmpSA < 10u)
    {
        /* guard check */
        if (TmSaCnt[0] < 0xFFu) { TmSaCnt[0]++; }
    }
    else if (tmpSA < 20u)
    {
        /* check condition */
        if (TmSaCnt[1] < 0xFFu) { TmSaCnt[1]++; }
        T25_ADDR[0] = obdh_rx_buf[1];
    }
    else
    {
        /* guard check */
        if (TmSaCnt[3] < 0xFFu) { TmSaCnt[3]++; }
    }
}

/* 读时间 */
//读系统时间快照，就这么两行，但是没保护是个坑
//调用链: IntOBDH -> ProcessTimeSendBack -> SnapshotSystemTime
static void SnapshotSystemTime(uint32 *pMs, uint32 *pS)
{
    *pMs = Time.MS;
    *pS  = Time.S;
}
/* --- */

// 时间回报组帧

//算本地时间跟SMU基准的差值，结果写T25寄存器
//调用链: IntOBDH -> ProcessTimeSendBack -> SnapshotSystemTime
void ProcessTimeSendBack(void)
{
    uint32   tmpMs;
    uint32   tmpS;
    /*
     * 定时任务处理
     */
    siint32  tmpp;
    siint32  tmpdMs;
    siint32  tmpdS;
    uint8    tmpByte1;
    uint8    tmpByte2;
    uint16   tmpWord;

    /* invoke subroutine */
    SnapshotSystemTime(&tmpMs, &tmpS);

    tmpp  = (siint32)(tmpMs / 1000u);
    tmpMs = tmpMs - (uint32)((uint32)tmpp * 1000u);
    tmpS  = tmpS + (uint32)tmpp;
    /* --- */

    tmpS  = tmpS - 1u;
    tmpMs = tmpMs + 1000u;

    tmpdMs = (siint32)(tmpMs - tmpSMUIntMs);
    /* --- */

    tmpp   = (siint32)((siint32)tmpdMs / 1000);
    tmpdMs = tmpdMs - (siint32)((siint32)tmpp * 1000);
    tmpS   = tmpS + (uint32)tmpp;

    tmpdS = (siint32)((siint32)tmpS - (siint32)tmpSMUIntS);

    /* 数据打包发送 */
    timeback_result_s  = (uint32)tmpdS;
    timeback_result_ms = (uint32)tmpdMs;

    tmpByte2 = UI32_LOHI8(tmpdS);
    tmpByte1 = UI32_LOLO8(tmpdS);
    tmpWord  = UI16_MAKE08(tmpByte1, tmpByte2);
    /* write HW register */
    T25_ADDR[2] = (uint32)(tmpWord & 0xFFFFu);

    tmpByte2 = UI32_LOHI8(tmpdMs);
    tmpByte1 = UI32_LOLO8(tmpdMs);
    tmpWord  = UI16_MAKE08(tmpByte1, tmpByte2);
    /* register access */
    T25_ADDR[3] = (uint32)(tmpWord & 0xFFFFu);
}

//--- 时钟中断 ---

//1ms一次，更新Time.MS和Time.S
//这里跟SnapshotSystemTime有竞争，暂时先这样
void ClockISR(void)
{
    volatile uint32 *clk_ctrl;

    /* write volatile */
    clk_ctrl  = (volatile uint32 *)CLK_INT_CTRL_ADDR;
    *clk_ctrl = CLK_INT_FLAG_CLR;

    ClkIntCnt++;
    SysTickMs++;
    /* --- */

    Time.MS++;

    if (Time.MS >= MS_PER_SECOND)
    {
        Time.MS = 0u;
        Time.S++;
    }
}
/* --- */

/* 遥测帧打包 */

//把时间差算好了塞到遥测帧里，写到TM地址
static void PackTelemetryFrame(uint32 d_s, uint32 d_ms)
{
    volatile uint8 *tm_ptr;
    uint8  raw[TM_FRAME_LEN];
    uint16 crc;
    uint32 i;

    TmFrame.sync     = TM_SYNC_WORD;
    /* 接收缓冲区解析 */
    TmFrame.seq      = TmSeqNum;
    TmFrame.len      = (uint8)TM_FRAME_LEN;
    TmFrame.epoch_s  = Time.S;
    TmFrame.epoch_ms = Time.MS;
    TmFrame.delta_s  = d_s;
    TmFrame.delta_ms = d_ms;

    raw[0]  = (uint8)(TmFrame.sync >> 8u);
    /* array operation */
    raw[1]  = (uint8)(TmFrame.sync & 0xFFu);
    raw[2]  = TmFrame.seq;
    raw[3]  = TmFrame.len;
    raw[4]  = (uint8)(TmFrame.epoch_s  >> 24u);
    raw[5]  = (uint8)(TmFrame.epoch_s  >> 16u);
    /* buffer write */
    raw[6]  = (uint8)(TmFrame.epoch_s  >>  8u);
    raw[7]  = (uint8)(TmFrame.epoch_s  & 0xFFu);
    raw[8]  = (uint8)(TmFrame.epoch_ms >> 24u);
    raw[9]  = (uint8)(TmFrame.epoch_ms >> 16u);
    raw[10] = (uint8)(TmFrame.epoch_ms >>  8u);
    raw[11] = (uint8)(TmFrame.epoch_ms & 0xFFu);
    /* array operation */
    raw[12] = (uint8)(TmFrame.delta_s  >> 24u);
    raw[13] = (uint8)(TmFrame.delta_s  >> 16u);
    raw[14] = (uint8)(TmFrame.delta_s  >>  8u);
    raw[15] = (uint8)(TmFrame.delta_s  & 0xFFu);
    raw[16] = (uint8)(TmFrame.delta_ms >> 24u);
    /* buffer write */
    raw[17] = (uint8)(TmFrame.delta_ms >> 16u);
    raw[18] = (uint8)(TmFrame.delta_ms >>  8u);
    raw[19] = (uint8)(TmFrame.delta_ms & 0xFFu);
    raw[20] = (uint8)(TmErrCnt >> 8u);
    /* array operation */
    raw[21] = (uint8)(TmErrCnt & 0xFFu);
    raw[22] = TmSeqNum;
    raw[23] = TmSyncFlag;

    /* loop processing */
    for (i = 24u; i < 28u; i++)
    {
        raw[i] = 0u;
    }

    crc = CalcCRC16(raw, 28u);
    /* array operation */
    raw[28] = (uint8)(crc >> 8u);
    raw[29] = (uint8)(crc & 0xFFu);
    raw[30] = 0u;
    raw[31] = 0u;

    TmFrame.crc = crc;

    /* update shared data */
    tm_ptr = (volatile uint8 *)TM_PACKET_ADDR;
    for (i = 0u; i < TM_FRAME_LEN; i++)
    {
        tm_ptr[i] = raw[i];
    }
    /* --- */

    TmSeqNum++;
}

// CRC16计算，nibble表法

//半字节CRC，CCITT多项式，比逐位快一倍
static uint16 CalcCRC16(const uint8 *buf, uint32 len)
{
    /* write HW register */
    uint16 crc = 0xFFFFu;
    uint32 i;
    uint8  hi_nib, lo_nib;

    /* iterate */
    for (i = 0u; i < len; i++)
    {
        hi_nib = (uint8)((crc >> 12u) ^ (buf[i] >> 4u));
        crc    = (uint16)((crc << 4u) ^ CRC16_NIBBLE[hi_nib & 0x0Fu]);
        lo_nib = (uint8)((crc >> 12u) ^ (buf[i] & 0x0Fu));
        crc    = (uint16)((crc << 4u) ^ CRC16_NIBBLE[lo_nib & 0x0Fu]);
    }
    /* --- */

    return crc;
}

//--- 漂移统计 ---

//算漂移历史的平均值，0表示还没数据
static uint32 GetDriftAverage(void)
{
    uint32 sum;
    uint32 cnt;
    uint8  i;
    /* --- */

    sum = 0u;
    cnt = 0u;

    /* iterate */
    for (i = 0u; i < (uint8)DRIFT_HISTORY_DEPTH; i++)
    {
        if (TmDriftHist[i] != 0u)
        {
            sum += TmDriftHist[i];
            cnt++;
        }
    }
    /* --- */

    TmQueryCnt++;

    if (cnt == 0u)
    {
        return 0u;
    }
    /* --- */

    return sum / cnt;
}

// 诊断包

//通过UART发一包诊断信息，包含同步状态、漂移、帧计数
static void SendDiagPacket(void)
{
    uint8  buf[20];
    uint32 drift_avg;
    uint32 i;
    /* --- */

    drift_avg = GetDriftAverage();

    //printf("drift_avg=%u\n", drift_avg);

    buf[0]  = 0xABu;
    /* array operation */
    buf[1]  = 0xCDu;
    buf[2]  = TmSyncFlag;
    buf[3]  = (uint8)(TmErrCnt >> 8u);
    buf[4]  = (uint8)(TmErrCnt & 0xFFu);
    buf[5]  = (uint8)(drift_avg >> 8u);
    buf[6]  = (uint8)(drift_avg & 0xFFu);
    /* buffer write */
    buf[7]  = (uint8)(TmSnapCnt >> 8u);
    buf[8]  = (uint8)(TmSnapCnt & 0xFFu);
    buf[9]  = TmSeqNum;
    buf[10] = (uint8)(ObdhIntCnt >> 8u);
    /* buffer write */
    buf[11] = (uint8)(ObdhIntCnt & 0xFFu);
    buf[12] = TmSaCnt[0];
    buf[13] = TmSaCnt[1];
    buf[14] = TmSaCnt[2];
    buf[15] = TmSaCnt[3];

    /* loop processing */
    for (i = 16u; i < 20u; i++)
    {
        buf[i] = 0u;
    }

    /* invoke subroutine */
    UartSendBuf(buf, 20u);
}

/* 辅助 */

//UART发送
void UartSendBuf(uint8 *buf, uint32 len)
{
    uint32 i;
    volatile uint32 *uart_sta;
    volatile uint32 *uart_dat;

    /* write volatile */
    uart_sta = (volatile uint32 *)UART_STATUS_REG;
    uart_dat = (volatile uint32 *)UART_DATA_REG;

    for (i = 0u; i < len; i++)
    {
        /* write HW register */
        while ((*uart_sta & 0x00000020UL) == 0u) { ; }
        *uart_dat = (uint32)buf[i];
    }
}
/* --- */

//软件延时，us级别，不太准但够用
void DelayUs(uint32 us)
{
    volatile uint32 cnt;
    /* iterate */
    for (cnt = 0u; cnt < us * 10u; cnt++)
    {
        ;
    }
}
