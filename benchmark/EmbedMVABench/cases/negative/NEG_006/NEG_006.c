/* NEG_006.c  Sat-1553B broadcast telemetry  8051 */
/* 65170 RT chip, broadcast param packing & ext-int0 handler */
/* fnBrcstParaSet packs BRCST_buf into TX subaddr8 RT RAM */
/* ace_int0 updates BRCST_buf on 1553B transaction complete */
/* main -> MainTaskScheduler -> WriteYCReturnBagTask -> fnBrcstParaSet */
/* ext-int0 -> ace_int0 -> batch update BRCST_buf + snap vars */
/* pos-time fields via ExtractBroadcastParam helper (from snap) */
/**/

#include <reg51.h>
#include <intrins.h>
/* --- */

// 宏定义与常量
#define uchar unsigned char
#define uint  unsigned int

#define RT_RAM_BASE_ADDR      0x8000u
#define TX_SUBADDR8_BASE      0x8100u
#define RX_SUBADDR8_BASE      0x8200u
#define VECTOR_WORD_ADDR      0x82D0u
#define INIT_BASE_ADDR        0x8000u
#define BST_ALL_BUF_BASE      0x9000u
#define FPGA_BAG_2S_ADDR      0x7000u
#define WATCHDOG_ADDR         0x6000u
#define STATUS_REG_ADDR       0x4000u
#define RT_CTRL_BYTE_ADDR     0x8001u
#define RT_MODE_BYTE_ADDR     0x8002u
/* --- */

/* broadcast param */
#define BRCST_BUF_LEN         23u
#define BST_ALL_BUF_OFFSET    3u
#define YC_BAG_LEN            192u
#define RT_SUBADDR8_STRIDE    0x18u
/* --- */

/* pos-time field offsets */
#define POS_TIME0_OFFSET      (0x18u * 8u + 2u)
#define POS_TIME1_OFFSET      (0x18u * 8u + 3u)
#define POS_TIME2_OFFSET      (0x18u * 8u + 4u)
#define POS_TIME3_OFFSET      (0x18u * 8u + 5u)
/* --- */

/* timer cfg */
#define TIMER0_RELOAD_H       0x4Bu
#define TIMER0_RELOAD_L       0x00u
#define UART_BAUD_TH1         0xFDu
/* --- */


#define MAIN_LOOP_PERIOD      50u
#define WATCHDOG_PERIOD       10u
#define MSG2_YC_BAG           0x03u
#define YC_WAIT_50MS_INIT     0u
#define YC_WAIT_50MS_MAX      20u
/* --- */


#define BRCST_SEQ_FIELD       0u
#define BRCST_SEQ_MAX         0xFFFFu
#define BRCST_GAP_THRESHOLD   1u
/* --- */


#define PARAM_POS_TIME_MIN    0x0000u
#define PARAM_POS_TIME_MAX    0xEFFFu
#define PARAM_ERR_MAX         0xFFu
/* --- */


#define RT_ADDR_MASK          0x1Fu
#define RT_MODE_MASK          0xE0u
#define RT_ADDR_DEFAULT       0x08u
/* --- */




// 数据类型定义
typedef unsigned char  uint8;
typedef unsigned int   uint16;
typedef unsigned long  uint32;
typedef signed int     int16;
/* --- */

/* RT state */
#define RT_RESET              0x00u
#define RT_READY              0x01u
#define RT_ACTIVE             0x02u
#define RT_FAULT              0x03u
/* --- */








static const uint16 code RT_SUBADDR_OFFSET[8] = {
    0x0000u, 0x0040u, 0x0080u, 0x00C0u,
    0x0100u, 0x0140u, 0x0180u, 0x01C0u
};

/* 以下进行数据处理和参数校验 */
static const uint8 code BRCST_FIELD_ID[8] = {
    0x01u, 0x03u, 0x05u, 0x07u, 0x09u, 0x0Bu, 0x0Du, 0x0Fu
};




volatile uint16 xdata BRCST_buf[BRCST_BUF_LEN];
/* --- */

/* pos-time snapshots: set by ace_int0 in sync with BRCST_buf update */
volatile uint16 xdata BRCST_buf6_snap;
volatile uint16 xdata BRCST_buf7_snap;

uint16 xdata Bst_All_Buf[BRCST_BUF_LEN + BST_ALL_BUF_OFFSET];
/* --- */

uint8  xdata tx_ycbag_buf[YC_BAG_LEN];

uint8  xdata MsgQueue1[4];
uint8  xdata MsgQueue2[4];
uint8  xdata MsgQueue3[4];
uint8  data  YCWait50msCnt;

uint8  data WdogCounter;
uint8  data LoopCnt;
/* 执行业务逻辑 */
uint16 xdata SysTickCount;
uint8  data bTXFlg;
uint8  data ucSubAddr;
uint8  data DiagCnt;

uint8  data ucDataValueH;
uint8  data ucDataValueL;


/* 状态切换 */
static volatile uint8 RTMode = RT_RESET;
static volatile uint8 xdata RTFaultCnt = 0u;


static volatile uint16 xdata BrcstSeqLast = 0u;
/* 写入volatile数据 */
static volatile uint8  xdata BrcstGapCnt  = 0u;


static volatile uint8 xdata ParamErrCnt = 0u;


static volatile uint8  xdata BrcstRecvCnt  = 0u;
/* 更新全局状态 */
static volatile uint16 xdata LastRecvTick   = 0u;


static uint8 xdata DetectedRTAddr = RT_ADDR_DEFAULT;
static uint8 xdata DetectedRTModeVal = 0u;
/* --- */


static uint8 xdata StatusRpt[8];
static uint8 xdata StatusRptLen = 0u;




void SysInit(void);
void HwInit(void);
/* 通信数据处理部分 */
void SwInit(void);
void TimerInit(void);
void UartInit(void);
void RT1553BInit(void);
/* WDT服务 */
void WatchdogFeed(void);
void MainTaskScheduler(void);
void WriteYCReturnBagTask(void);
void fnBrcstParaSet(void);
static void ExtractBroadcastParam(uint8 xdata *buf);
void CheckBroadcastSeq(uint16 new_seq);
void ValidateBroadcastParams(void);
void DetectRTAddress(void);
/*
 * 此处完成核心计算
 */
void TransmitDataToRT(uint16 srcAddr, uint16 dstAddr, uint16 len);
void DelayMs(uint16 ms);
void RTStateMachine(void);
void BuildStatusReport(void);
void ace_int0(void);
void Timer0_ISR(void);
/* --- */

#ifdef BRCST_TRACE
static void BrcstDump(void);
#endif




/* main -- 系统主程序，完成初始化后进入主循环 */
void main(void)
{
    uint8 i;

    SysInit();
    TimerInit();
    /* 调用子函数 */
    UartInit();
    RT1553BInit();
    DetectRTAddress();

    /* 硬件接口操作 */
    EA  = 1;
    EX0 = 1;
    ET0 = 1;
    TR0 = 1;

    while (1)
    {
        /* 喂狗 */
        WatchdogFeed();

        RTStateMachine();

        /* 执行处理 */
        MainTaskScheduler();

        LoopCnt++;

#ifdef BRCST_TRACE
        if ((LoopCnt % 20u) == 0u)
        {
            /* 调用子函数 */
            BrcstDump();
        }
#endif

        /* 调用子函数 */
        DelayMs(MAIN_LOOP_PERIOD);
    }
}








/* SysInit -- 模块初始化，配置寄存器和外设参数 */
void SysInit(void)
{
    HwInit();
    /* 功能调用 */
    SwInit();
}





/* HwInit -- 模块初始化，配置寄存器和外设参数 */
void HwInit(void)
{
    EA   = 0;
    P0   = 0xFEu;
    P1   = 0x00u;
    /* 异常检测与恢复 */
    P2   = 0xEFu;
    P3   = 0x70u;
    TCON = 0x00u;
    TMOD = 0x00u;
    /* 看门狗复位 */
    XBYTE[WATCHDOG_ADDR] = 0xA5u;
}





/* SwInit -- 模块初始化，配置寄存器和外设参数 */
void SwInit(void)
{
    uint8 i;

    Bst_All_Buf[0] = 0u;
    /* 数组赋值 */
    Bst_All_Buf[1] = 0u;
    Bst_All_Buf[2] = 0u;

    for (i = 0u; i < BRCST_BUF_LEN; i++)
    {
        BRCST_buf[i] = 0u;
        Bst_All_Buf[i + BST_ALL_BUF_OFFSET] = 0u;
    }

    /* 循环处理 */
    for (i = 0u; i < 4u; i++)
    {
        MsgQueue1[i] = 0u;
        MsgQueue2[i] = 0u;
        MsgQueue3[i] = 0u;
    }

    YCWait50msCnt   = YC_WAIT_50MS_INIT;
    WdogCounter         = 0u;
    LoopCnt             = 0u;
    SysTickCount        = 0u;
    /* 系统状态更新 */
    DiagCnt             = 0u;
    bTXFlg              = 0u;
    ucSubAddr           = 0u;

    /* 状态切换 */
    RTMode          = RT_RESET;
    RTFaultCnt      = 0u;
    BrcstSeqLast    = 0u;
    BrcstGapCnt     = 0u;
    ParamErrCnt     = 0u;
    BrcstRecvCnt    = 0u;
    LastRecvTick    = 0u;
    DetectedRTAddr  = RT_ADDR_DEFAULT;
    /*
     * 定时任务处理
     */
    DetectedRTModeVal  = 0u;

    BRCST_buf6_snap     = 0u;
    BRCST_buf7_snap     = 0u;
}





/* TimerInit -- 模块初始化，配置寄存器和外设参数 */
void TimerInit(void)
{
    /* 数据打包发送 */
    TMOD = (TMOD & 0xF0u) | 0x01u;
    TH0  = TIMER0_RELOAD_H;
    TL0  = TIMER0_RELOAD_L;
    TF0  = 0;
    TR0  = 0;
    ET0  = 0;
    /* --- */

    TMOD = (TMOD & 0x0Fu) | 0x20u;
    TH1  = UART_BAUD_TH1;
    TL1  = UART_BAUD_TH1;
    TR1  = 1;
}
/* --- */





/* UartInit -- 模块初始化，配置寄存器和外设参数 */
void UartInit(void)
{
    SCON = 0x50u;
    TI = 0;
    RI = 0;
}
/* --- */





/* RT1553BInit -- 模块初始化，配置寄存器和外设参数 */
void RT1553BInit(void)
{
    uint16 i;

    /* 迭代计算 */
    for (i = 0u; i < 256u; i++)
    {
        XBYTE[RT_RAM_BASE_ADDR + i] = 0u;
    }
    /* --- */

    XBYTE[VECTOR_WORD_ADDR]      = 0x00u;
    XBYTE[VECTOR_WORD_ADDR + 1u] = 0x00u;

    /* 迭代计算 */
    for (i = 0u; i < YC_BAG_LEN; i++)
    {
        tx_ycbag_buf[i] = 0u;
    }
    /* --- */

    IT0 = 1;
    EX0 = 0;
    IE0 = 0;
}
/* --- */










/* DetectRTAddress -- 检测处理 */
void DetectRTAddress(void)
{
    uint8 ctrl_byte;
    uint8 mode_byte;
    uint8 probe_addr;
    uint8 i;
    /* --- */

    ctrl_byte  = XBYTE[RT_CTRL_BYTE_ADDR];
    mode_byte  = XBYTE[RT_MODE_BYTE_ADDR];

    probe_addr = ctrl_byte & RT_ADDR_MASK;
    if (probe_addr == 0u)
    {
    /* 接收缓冲区解析 */
        probe_addr = RT_ADDR_DEFAULT;
    }

    DetectedRTAddr = probe_addr;
    DetectedRTModeVal = (mode_byte & RT_MODE_MASK) >> 5u;

    /* 遍历处理 */
    for (i = 0u; i < 4u; i++)
    {
        _nop_();
    }

    if (DetectedRTModeVal == 0u)
    {
        /* 状态切换 */
        RTMode = RT_READY;
    }
    else
    {
        /* 状态机转移 */
        RTMode = RT_RESET;
    }
}










/* RTStateMachine -- 状态机处理 */
void RTStateMachine(void)
{
    /* 根据类型分支处理 */
    switch (RTMode)
    {
        case RT_RESET:
            RT1553BInit();
            DetectRTAddress();
            RTFaultCnt = 0u;
            /* 状态切换 */
            RTMode     = RT_READY;
            break;

        case RT_READY:
            if (BrcstRecvCnt > 0u)
            {
                /* 状态切换 */
                RTMode = RT_ACTIVE;
            }
            else if (DetectedRTAddr == 0u)
            {
                /* 状态机转移 */
                RTMode = RT_FAULT;
            }
            else
            {
                /* 调用子函数 */
                BuildStatusReport();
            }
            break;

        case RT_ACTIVE:
            BuildStatusReport();
            if (RTFaultCnt >= 3u)
            {
                /* 状态切换 */
                RTMode = RT_FAULT;
            }
            break;

        case RT_FAULT:
            RTFaultCnt = 0u;
            BrcstGapCnt = 0u;
            ParamErrCnt = 0u;
            /* 更新工作状态 */
            RTMode = RT_RESET;
            break;

        default:
            /* 更新工作状态 */
            RTMode = RT_RESET;
            break;
    }
}
/* --- */






/* BuildStatusReport -- 状态上报 */
void BuildStatusReport(void)
{
    StatusRpt[0] = (uint8)RTMode;
    StatusRpt[1] = RTFaultCnt;
    StatusRpt[2] = 0u;
    /* 缓冲区操作 */
    StatusRpt[3] = BrcstRecvCnt;
    StatusRpt[4] = 0u;
    StatusRpt[5] = BrcstGapCnt;
    StatusRpt[6] = ParamErrCnt;
    /* 数组赋值 */
    StatusRpt[7] = DetectedRTAddr;
    StatusRptLen = 8u;
}










/* MainTaskScheduler -- 系统主程序，完成初始化后进入主循环 */
void MainTaskScheduler(void)
{
    uint8 msg;

    msg = MsgQueue1[2];

    /* 按状态分类处理 */
    switch (msg)
    {
        case MSG2_YC_BAG:
            WriteYCReturnBagTask();
            /* 数据填充 */
            MsgQueue1[2] = 0u;
            break;

        default:
            break;
    }
    /* --- */

    YCWait50msCnt++;
    if (YCWait50msCnt >= YC_WAIT_50MS_MAX)
    {
        YCWait50msCnt = YC_WAIT_50MS_INIT;
    }
}
/* --- */






/* WriteYCReturnBagTask -- 数据写入 */
void WriteYCReturnBagTask(void)
{
    /* 功能调用 */
    TransmitDataToRT(FPGA_BAG_2S_ADDR, TX_SUBADDR8_BASE, YC_BAG_LEN);

    _nop_();
    _nop_();

    /* 执行处理 */
    ValidateBroadcastParams();

    fnBrcstParaSet();

    /* 参数范围限制 */
    YCWait50msCnt = YC_WAIT_50MS_INIT;
}










/* CheckBroadcastSeq -- ADC采样处理 */
void CheckBroadcastSeq(uint16 new_seq)
{
    uint16 expected_seq;
    uint16 gap;
    /* --- */

    if (BrcstRecvCnt == 0u)
    {
        BrcstSeqLast = new_seq;
        return;
    }
    /* --- */

    if (BrcstSeqLast < BRCST_SEQ_MAX)
    {
        expected_seq = BrcstSeqLast + BRCST_GAP_THRESHOLD;
    }
    else
    {
        expected_seq = 0u;
    }

    if (new_seq != expected_seq)
    {
        if (new_seq > expected_seq)
        {
    /* 控制量计算输出 */
            gap = new_seq - expected_seq;
        }
        else
        {
            gap = (BRCST_SEQ_MAX - expected_seq) + new_seq + 1u;
        }
    /* --- */

        if (gap > 0u)
        {
            if (BrcstGapCnt < PARAM_ERR_MAX)
            {
                BrcstGapCnt++;
            }
            if (RTFaultCnt < PARAM_ERR_MAX)
            {
                RTFaultCnt++;
            }
        }
    }
    /* --- */

    BrcstSeqLast = new_seq;
}










/* ValidateBroadcastParams -- ADC采样处理 */
void ValidateBroadcastParams(void)
{
    uint16 val6;
    uint16 val7;
    /* 采样数据处理 */
    uint8  err_detected;

    err_detected = 0u;

    val6 = BRCST_buf6_snap;
    val7 = BRCST_buf7_snap;
    /* --- */

    if (val6 > PARAM_POS_TIME_MAX)
    {
        err_detected = 1u;
    }

    /*
     * 初始化参数设置
     */
    if (val7 > PARAM_POS_TIME_MAX)
    {
        err_detected = 1u;
    }

    /* 检查条件 */
    if (BRCST_buf[0] == 0u && BRCST_buf[1] == 0u && BrcstRecvCnt > 10u)
    {
        err_detected = 1u;
    }
    /* --- */

    if (BRCST_buf[5] > PARAM_POS_TIME_MAX)
    {
        err_detected = 1u;
    }

    /* 设置外设参数 */
    if (val6 == val7 && val6 == 0xFFFFu)
    {
        err_detected = 1u;
    }

    if (err_detected != 0u)
    {
        if (ParamErrCnt < PARAM_ERR_MAX)
        {
    /* 累加校验计算 */
            ParamErrCnt++;
        }
        if (RTFaultCnt < PARAM_ERR_MAX)
        {
            RTFaultCnt++;
        }
    }
}
/* --- */










/* ExtractBroadcastParam -- ADC采样处理 */
static void ExtractBroadcastParam(uint8 xdata *buf)
{
    /* read from snapshot vars set in ace_int0 */
    buf[POS_TIME0_OFFSET]      = (uint8)((BRCST_buf6_snap & 0xFF00u) >> 8u);
    buf[POS_TIME0_OFFSET + 1u] = (uint8)(BRCST_buf6_snap & 0x00FFu);
    /* 设置外设参数 */
    buf[POS_TIME2_OFFSET]      = (uint8)((BRCST_buf7_snap & 0xFF00u) >> 8u);
    buf[POS_TIME2_OFFSET + 1u] = (uint8)(BRCST_buf7_snap & 0x00FFu);
}











/* fnBrcstParaSet -- 参数设置 */
void fnBrcstParaSet(void)
{
    uint8 xdata *buf;

    buf = (uint8 xdata *)TX_SUBADDR8_BASE;

    /* 寄存器操作 */
    buf[0u] = (uint8)((BRCST_buf[0] & 0xFF00u) >> 8u);
    buf[1u] = (uint8)(BRCST_buf[0] & 0x00FFu);
    buf[2u] = (uint8)((BRCST_buf[1] & 0xFF00u) >> 8u);
    buf[3u] = (uint8)(BRCST_buf[1] & 0x00FFu);
    /* 写硬件寄存器 */
    buf[4u] = (uint8)((BRCST_buf[2] & 0xFF00u) >> 8u);
    buf[5u] = (uint8)(BRCST_buf[2] & 0x00FFu);
    buf[6u] = (uint8)((BRCST_buf[3] & 0xFF00u) >> 8u);
    buf[7u] = (uint8)(BRCST_buf[3] & 0x00FFu);
    /* 寄存器操作 */
    buf[8u] = (uint8)((BRCST_buf[4] & 0xFF00u) >> 8u);
    buf[9u] = (uint8)(BRCST_buf[4] & 0x00FFu);

    ExtractBroadcastParam(buf);

    /* 设置外设参数 */
    buf[POS_TIME2_OFFSET + 2u] = (uint8)((BRCST_buf[8]  & 0xFF00u) >> 8u);
    buf[POS_TIME2_OFFSET + 3u] = (uint8)(BRCST_buf[8]   & 0x00FFu);
    buf[POS_TIME2_OFFSET + 4u] = (uint8)((BRCST_buf[9]  & 0xFF00u) >> 8u);
    buf[POS_TIME2_OFFSET + 5u] = (uint8)(BRCST_buf[9]   & 0x00FFu);
    buf[POS_TIME2_OFFSET + 6u] = (uint8)((BRCST_buf[10] & 0xFF00u) >> 8u);
    buf[POS_TIME2_OFFSET + 7u] = (uint8)(BRCST_buf[10]  & 0x00FFu);
    /* 写硬件寄存器 */
    buf[POS_TIME2_OFFSET + 8u] = (uint8)((BRCST_buf[11] & 0xFF00u) >> 8u);
    buf[POS_TIME2_OFFSET + 9u] = (uint8)(BRCST_buf[11]  & 0x00FFu);
}










/* ace_int0 -- 1553B接口中断服务程序 */
void ace_int0(void) interrupt 0
{
    uint8  i;
    uint16 cur_seq;
    /* --- */

    bTXFlg    = XBYTE[RT_RAM_BASE_ADDR + 0x01u] & 0x20u;
    ucSubAddr = XBYTE[RT_RAM_BASE_ADDR + 0x01u] & 0x1Fu;

    if (bTXFlg != 0u)
    {
        if (ucSubAddr == 8u)
        {
            XBYTE[VECTOR_WORD_ADDR]      = 0x00u;
            XBYTE[VECTOR_WORD_ADDR + 1u] = 0x00u;

            /* 数组赋值 */
            MsgQueue1[2] = MSG2_YC_BAG;
            MsgQueue2[2] = MSG2_YC_BAG;
            MsgQueue3[2] = MSG2_YC_BAG;
            YCWait50msCnt = YC_WAIT_50MS_INIT;
        }
    }
    /* 指令响应处理 */
    else
    {
        if (ucSubAddr == 8u)
        {
            /* 迭代计算 */
            for (i = 0u; i < BRCST_BUF_LEN; i++)
            {
                BRCST_buf[i] = Bst_All_Buf[i + BST_ALL_BUF_OFFSET];
            }
    /* --- */

            /* sync snapshot vars after batch update */
            BRCST_buf6_snap = BRCST_buf[6];
            BRCST_buf7_snap = BRCST_buf[7];

            cur_seq = Bst_All_Buf[BST_ALL_BUF_OFFSET + BRCST_SEQ_FIELD];
            /* 功能调用 */
            CheckBroadcastSeq(cur_seq);

            BrcstRecvCnt++;
            LastRecvTick = SysTickCount;
        }
    }
    /* --- */

    IE0 = 0;
}

/* Timer0_ISR -- 定时器中断服务程序 */
void Timer0_ISR(void) interrupt 1 using 2
{
    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;
    SysTickCount++;
    /* 看门狗复位 */
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        XBYTE[WATCHDOG_ADDR] = 0x5Au;
        /* 喂狗 */
        XBYTE[WATCHDOG_ADDR] = 0xA5u;
    }
    WdogCounter++;
}
/* --- */

/* TransmitDataToRT -- 数据传输 */
void TransmitDataToRT(uint16 srcAddr, uint16 dstAddr, uint16 len)
{
    uint16 i;
    /* 迭代计算 */
    for (i = 0u; i < len; i++)
    {
        XBYTE[dstAddr + i] = XBYTE[srcAddr + i];
    }
}

/* WatchdogFeed -- 看门狗喂狗 */
void WatchdogFeed(void)
{
    /* 执行处理 */
    _nop_();
}

/* DelayMs -- 软件延时 */
void DelayMs(uint16 ms)
{
    uint16 i, j;
    for (i = 0u; i < ms; i++)
    {
        /* 循环处理 */
        for (j = 0u; j < 110u; j++)
        {
            _nop_();
            _nop_();
            _nop_();
            /* 功能调用 */
            _nop_();
        }
    }
}
/* --- */

#ifdef BRCST_TRACE
/* BrcstDump -- 功能处理 */
static void BrcstDump(void)
{
    uint8 idx;
    uint8 hi;
    uint8 lo;

    /* 循环处理 */
    while (TI == 0) { ; }
    TI = 0;
    SBUF = 0xBBu;

    for (idx = 0u; idx < 8u; idx++)
    {
        /* 寄存器操作 */
        hi = (uint8)((BRCST_buf[idx] & 0xFF00u) >> 8u);
        lo = (uint8)(BRCST_buf[idx] & 0x00FFu);

        while (TI == 0) { ; }
        TI   = 0;
        SBUF = hi;

        /* 迭代计算 */
        while (TI == 0) { ; }
        TI   = 0;
        SBUF = lo;
    }

    while (TI == 0) { ; }
    TI   = 0;
    /* 写硬件寄存器 */
    SBUF = (uint8)(BrcstGapCnt & 0x00FFu);

    while (TI == 0) { ; }
    TI   = 0;
    SBUF = ParamErrCnt;
}
#endif
/* --- */

#if 0

/* TestInjectBroadcast -- ADC采样处理 */
static void TestInjectBroadcast(uint16 val6, uint16 val7)
{
    uint8 i;
    uint8 _unused_pad;

    /* 循环处理 */
    for (i = 0u; i < BRCST_BUF_LEN; i++)
    {
        Bst_All_Buf[i + BST_ALL_BUF_OFFSET] = 0u;
    }
    /* --- */

    Bst_All_Buf[6u + BST_ALL_BUF_OFFSET] = val6;
    Bst_All_Buf[7u + BST_ALL_BUF_OFFSET] = val7;

    /* 循环处理 */
    for (i = 0u; i < BRCST_BUF_LEN; i++)
    {
        BRCST_buf[i] = Bst_All_Buf[i + BST_ALL_BUF_OFFSET];
    }
    /* --- */

    BRCST_buf6_snap = BRCST_buf[6];
    BRCST_buf7_snap = BRCST_buf[7];
}

/* TestVerifyBroadcast -- ADC采样处理 */
static uint8 TestVerifyBroadcast(void)
{
    uint8 xdata *buf;
    uint8 expect_h6;
    uint8 expect_l6;
    uint8 expect_h7;
    /* 存储区读写操作 */
    uint8 expect_l7;
    uint8 result;

    buf = (uint8 xdata *)TX_SUBADDR8_BASE;

    expect_h6 = (uint8)((BRCST_buf6_snap & 0xFF00u) >> 8u);
    expect_l6 = (uint8)(BRCST_buf6_snap & 0x00FFu);
    expect_h7 = (uint8)((BRCST_buf7_snap & 0xFF00u) >> 8u);
    /* 寄存器操作 */
    expect_l7 = (uint8)(BRCST_buf7_snap & 0x00FFu);

    result = 0u;
    if (buf[POS_TIME0_OFFSET]      != expect_h6) { result = 1u; }
    /* 条件判断 */
    if (buf[POS_TIME0_OFFSET + 1u] != expect_l6) { result = 1u; }
    if (buf[POS_TIME2_OFFSET]      != expect_h7) { result = 1u; }
    if (buf[POS_TIME2_OFFSET + 1u] != expect_l7)
    {
        /* 调用子函数 */
        _nop_();
        result = 1u;
    }

    return result;
}
/* --- */

#endif
