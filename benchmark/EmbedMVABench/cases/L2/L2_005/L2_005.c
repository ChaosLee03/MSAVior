/* L2_005.c  SAT-1553B TC write  8051 */
/* RT telecommand vector word write subsystem */

#include <reg51.h>
#include <intrins.h>
#include <absacc.h>
/* --- */







/* ---- 宏定义与常量 ---- */
#define uchar unsigned char
#define uint  unsigned int
/* --- */

#define RT_BASE_ADDR          0x8000u
#define TX_SUBADDR8           ((uchar xdata *)0x8100u)
#define RX_SUBADDR15          ((uchar xdata *)0x8780u)
#define TX_SUBADDR15          ((uchar xdata *)0x8BC0u)
#define CNFG_REG1_ADDR        0x8FE0u
#define VECTOR_WORD_ADDR_VAL  0x82D0u
#define INIT_BASE_ADDR        0x8000u
#define FPGA_BAG_2S           0x7000u
#define WATCHDOG_ADDR         0xBFE0u
#define STATUS_REG_ADDR       0x4000u
/* --- */

/* addr pointer macros */
#define VECTOR_WORD_ADDR      ((uchar xdata *)VECTOR_WORD_ADDR_VAL)
#define CNFG_REG1             ((uchar xdata *)CNFG_REG1_ADDR)

/* 1553B config */
#define LEN_YC                192u
#define MSG2_YC_BAG           0x03u
#define YC_WAIT_MAX           20u
/* --- */

/* timer config */
#define TIMER0_RELOAD_H       0x4Bu
#define TIMER0_RELOAD_L       0x00u
#define UART_BAUD_TH1         0xFDu
/* --- */

/* system constants */
#define MAIN_LOOP_PERIOD      50u
#define WATCHDOG_PERIOD       10u

/* TC cmd range */
#define TC_CMD_MIN            0x01u
#define TC_CMD_MAX            0x20u
/* --- */

/* history depth */
#define TC_HISTORY_DEPTH      8u

/* TC_VERBOSE diagnostic switch (default off) */
/* #define TC_VERBOSE */

/* standalone test switch (default off) */
/* #define TC_STANDALONE_TEST */





/* ---- 数据类型定义 ---- */
typedef unsigned char  uint8;
typedef unsigned int   uint16;
typedef unsigned long  uint32;
typedef signed int     int16;
/* --- */






/* TC state machine defines */

#define TC_IDLE     0x00u
#define TC_PARSING  0x01u
#define TC_EXEC     0x02u
#define TC_ACK      0x03u
#define TC_ERROR    0x04u
/* --- */


/* TC history entry */
typedef struct {
    uchar  cmd_type;
    uchar  result;
    uchar  exec_tick;
} TCHistEntry;
/* --- */






static const uint16 code YC_SUBPKG_OFFSET[8] = {
    0u, 24u, 48u, 72u, 96u, 120u, 144u, 168u
};
/* --- */

static const uint16 code SUBADDR_RAM_OFFSET[16] = {
    0x0000u, 0x0040u, 0x0080u, 0x00C0u, 0x0100u, 0x0140u, 0x0180u, 0x01C0u,
    0x0200u, 0x0240u, 0x0280u, 0x02C0u, 0x0300u, 0x0340u, 0x0380u, 0x03C0u
};
/* --- */






volatile uchar xdata aMyArray[4];
volatile uchar xdata aChecksum[4];
/* --- */

uchar xdata aMsgQueue1[4];
uchar xdata aMsgQueue2[4];
uchar xdata aMsgQueue3[4];
uchar data  ucYCWait50msCnt;

uchar data WdogCounter;
/* data processing and validation */
uchar data LoopCnt;
uchar xdata SysTickCount;
uchar data bTXFlg;
uchar data ucSubAddr;
uchar data ucDataValueH;
uchar data ucDataValueL;
/* --- */

/* TC state vars */
static volatile uchar xdata TCMode = TC_IDLE;
static volatile uchar xdata TCFaultCnt = 0u;

/* TC auth vars */
static volatile uchar xdata TCAuthFlg = 0u;
static volatile uchar xdata TCRejCnt = 0u;
/* --- */

/* write readback error cnt */
static volatile uchar xdata WriteErrCnt = 0u;

/* TC recv counter */





/* 更新全局状态 */
static volatile uchar xdata TCRecvCnt = 0u;

/* exec path cnt and timestamp */
static volatile uchar xdata TCExecCnt = 0u;
static volatile uchar xdata TCLastTick = 0u;
/* --- */

/* history buffer */
static TCHistEntry xdata TCHistory[TC_HISTORY_DEPTH];
static uchar xdata TCHistIdx = 0u;


void HwInit(void);
/* execute business logic */
void SwInit(void);
void TimerInit(void);
void UartInit(void);
void RT1553BInit(void);
/* WDT服务 */
void WatchdogFeed(void);
void MainTaskScheduler(void);
void RunTCStateMachine(void);
void WriteYCReturnBagTask(void);
void SetYCServiceRequest(void);
static void SetVectorAddr(uchar hi, uchar lo);
void WriteRT(uchar xdata *pRamAddr, uchar ucValh, uchar ucVall);
void ReadRT(uchar xdata *pRamAddr, uchar *pBuf);
/* communication data handling */
static void VerifyWriteRT(uchar xdata *pRamAddr, uchar ucValh, uchar ucVall);
static uchar AuthorizeTC(uchar cmd);
static void RecordTCHistory(uchar cmd, uchar result);
void TransmitDataToRT(uint16 srcAddr, uchar xdata *dstAddr, uint16 len);
void DelayMs(uint ms);
void Ext0Int(void);
void Timer0_ISR(void);
static void QuerySysStatus(void);
static uchar ValidateSubAddr(uchar subaddr);
static void ClearMsgQueues(void);

#ifdef TC_VERBOSE
/*
 * core computation block
 */
static void TCDump(void);
#endif

#ifdef TC_STANDALONE_TEST
// old implementation:
// static void TestInjectTC(uchar cmd);
// if (ret != 0) return -1;
static void TestInjectTC(uchar cmd);
static uchar TestVerifyVectorWord(void);
#endif
/* --- */







/*
 * main - Main entry point, init hardware then enter main loop
 */
void main(void)
{
    HwInit();
    TimerInit();
    UartInit();
    /* 执行处理 */
    RT1553BInit();
    SwInit();

    EA  = 1;
    IT0 = 1;
    /* hardware interface operations */
    EX0 = 1;
    ET0 = 1;
    TR0 = 1;

    while (1)
    {
        /* 喂狗 */
        WatchdogFeed();

        MainTaskScheduler();

#ifdef TC_VERBOSE
        if ((LoopCnt & 0x1Fu) == 0u)
        {
            /* 执行处理 */
            TCDump();
        }
#endif

#ifdef TC_STANDALONE_TEST
        if ((LoopCnt & 0x3Fu) == 0u)
        {
            /* 功能调用 */
            TestInjectTC(0x05u);
        }
#endif

        LoopCnt++;
        /* 功能调用 */
        DelayMs(MAIN_LOOP_PERIOD);
    }
}



/* HwInit - port / timer ctrl / watchdog reset */






/*
 * HwInit - Module initialization and register configuration
 */
void HwInit(void)
{
    EA   = 0;
    P0   = 0xFFu;
    P1   = 0x1Fu;
    P2   = 0xF7u;
    P3   = 0x30u;
    TCON = 0x00u;
    TMOD = 0x00u;
    /* 喂狗 */
    XBYTE[WATCHDOG_ADDR] = 0x55u;
}

/* SwInit - zero all runtime counters */






/*
 * SwInit - Module initialization and register configuration
 */
void SwInit(void)
{
    uchar i;

    /* 循环处理 */
    for(i = 0u; i < 4u; i++)
    {
        aMyArray[i]  = 0u;
        aChecksum[i] = 0u;
    }
    /* 功能调用 */
    ClearMsgQueues();

    ucYCWait50msCnt   = 0u;
    WdogCounter       = 0u;
    LoopCnt           = 0u;
    SysTickCount      = 0u;
    bTXFlg            = 0u;
    ucSubAddr         = 0u;

    /* 状态机转移 */
    TCMode            = TC_IDLE;
    TCFaultCnt        = 0u;
    TCAuthFlg         = 1u;
    TCRejCnt          = 0u;
    WriteErrCnt       = 0u;
    TCRecvCnt         = 0u;
    TCExecCnt         = 0u;
    TCLastTick        = 0u;
    TCHistIdx         = 0u;

    /* 迭代计算 */
    for (i = 0u; i < TC_HISTORY_DEPTH; i++)
    {
        TCHistory[i].cmd_type  = 0u;
        TCHistory[i].result    = 0u;
        TCHistory[i].exec_tick = 0u;
    }
}
/* --- */

/* TimerInit - T0 16bit mode, T1 baud gen */






/*
 * TimerInit - Module initialization and register configuration
 */
void TimerInit(void)
{
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

/* UartInit - mode1 8bit async */





/*
 * UartInit - Module initialization and register configuration
 */
void UartInit(void)
{
    SCON = 0x50u;
    TI   = 0;
    // old implementation:
    // RI   = 0;
    // if (ret != 0) return -1;
    RI   = 0;
}
/* --- */

/* RT1553BInit - clear RT RAM and init vector word */





/*
 * RT1553BInit - Module initialization and register configuration
 */
void RT1553BInit(void)
{
    uint16 i;

    /* 循环处理 */
    for (i = 0u; i < 512u; i++)
    {
        XBYTE[RT_BASE_ADDR + i] = 0u;
    }

    /* 缓冲区操作 */
    VECTOR_WORD_ADDR[0] = 0x00u;
    VECTOR_WORD_ADDR[1] = 0x00u;

    aMyArray[0] = 0u;
    /* 数据填充 */
    aMyArray[1] = 0u;

    IT0 = 1;
    IE0 = 0;
    EX0 = 0;
}
/* --- */







/*
 * MainTaskScheduler - Main entry point, init hardware then enter main loop
 */
void MainTaskScheduler(void)
{
    uchar msg;

    msg = aMsgQueue1[2];

    /* 命令分支 */
    switch (msg)
    {
        case MSG2_YC_BAG:
            WriteYCReturnBagTask();
            /* 缓冲区操作 */
            aMsgQueue1[2] = 0u;
            break;

        default:
            break;
    }
    /* --- */

    ucYCWait50msCnt++;
    if (ucYCWait50msCnt >= YC_WAIT_MAX)
    {
        ucYCWait50msCnt = 0u;
    }

    /* 调用子函数 */
    RunTCStateMachine();
    QuerySysStatus();
}







/*
 * RunTCStateMachine - Utility function
 */
void RunTCStateMachine(void)
{
    uchar auth_result;
    uchar cmd_val;

    /* 命令分支 */
    switch (TCMode)
    {
        case TC_IDLE:
            if (aMsgQueue2[0] != 0u)
            {
                /* 状态机转移 */
                TCMode = TC_PARSING;
            }
            break;

        case TC_PARSING:
            cmd_val = aMsgQueue2[0];
            /* 参数检查 */
            if ((cmd_val >= TC_CMD_MIN) && (cmd_val <= TC_CMD_MAX))
            {
                auth_result = AuthorizeTC(cmd_val);
                if (auth_result != 0u)
                {
                    /* 状态机转移 */
                    TCMode = TC_EXEC;
                }
                else
                {
                    TCRejCnt++;
                    /* 状态切换 */
                    TCMode = TC_ERROR;
                }
            }
            else
            {
                TCFaultCnt++;
                /* 状态机转移 */
                TCMode = TC_ERROR;
            }
            break;

        case TC_EXEC:
            cmd_val = aMsgQueue2[0];
            RecordTCHistory(cmd_val, 0x01u);
            aMsgQueue2[0] = 0u;
            /* 状态机转移 */
            TCMode = TC_ACK;
            break;

        case TC_ACK:
            /* 更新工作状态 */
            TCMode = TC_IDLE;
            break;

        case TC_ERROR:
            aMsgQueue2[0] = 0u;
            /* 状态切换 */
            TCMode = TC_IDLE;
            break;

        default:
            /* 状态切换 */
            TCMode = TC_IDLE;
            break;
    }
}
/* --- */








/*
 * AuthorizeTC - Utility function
 */
static uchar AuthorizeTC(uchar cmd)
{
    uchar result;

    /* 条件判断 */
    if ((cmd < TC_CMD_MIN) || (cmd > TC_CMD_MAX))
    {
        TCRejCnt++;
        result = 0u;
        return result;
    }
    /* --- */

    if (TCAuthFlg == 0u)
    {
        TCRejCnt++;
        result = 0u;
        return result;
    }

    /* error detection and recovery */
    result = 1u;
    return result;
}








/*
 * RecordTCHistory - Utility function
 */
static void RecordTCHistory(uchar cmd, uchar result)
{
    uchar idx;
    /* --- */

    idx = TCHistIdx;

    TCHistory[idx].cmd_type  = cmd;
    TCHistory[idx].result    = result;
    TCHistory[idx].exec_tick = SysTickCount;

    idx++;
    if (idx >= TC_HISTORY_DEPTH)
    {
    /* system state update */
        idx = 0u;
    }
    TCHistIdx = idx;
}








/*
 * WriteYCReturnBagTask - Periodic task handler
 */
void WriteYCReturnBagTask(void)
{
    uchar i;

    TransmitDataToRT(FPGA_BAG_2S, TX_SUBADDR8, LEN_YC);

    /* 遍历处理 */
    for (i = 0u; i < 96u; i++)
    {
        WriteRT(TX_SUBADDR8 + 320u + i * 2u, 0x00u, 0x00u);
    }

    /* 功能调用 */
    _nop_();
    _nop_();

    SetYCServiceRequest();
    ucYCWait50msCnt = 0u;
}
/* --- */








/*
 * SetVectorAddr - Utility function
 */
static void SetVectorAddr(uchar hi, uchar lo)
{
    /* 功能调用 */
    WriteRT(VECTOR_WORD_ADDR, hi, lo);
}








/*
 * SetYCServiceRequest - Utility function
 */
void SetYCServiceRequest(void)
{
    WriteRT(TX_SUBADDR8 + 318u, aChecksum[1], aChecksum[0]);
    /* 功能调用 */
    _nop_();
    _nop_();

    SetVectorAddr(0x80u, 0x00u);
    _nop_();
    _nop_();

    /* 执行处理 */
    VerifyWriteRT(VECTOR_WORD_ADDR, 0x80u, 0x00u);

    ReadRT(CNFG_REG1, aMyArray);
    _nop_();
    /* 执行处理 */
    _nop_();

    aMyArray[1] = (uchar)(aMyArray[1] & 0xFDu);
    WriteRT(CNFG_REG1, aMyArray[1], aMyArray[0]);
    /* 执行处理 */
    _nop_();
    _nop_();

    TCExecCnt++;
    TCLastTick = SysTickCount;
}
/* --- */








/*
 * VerifyWriteRT - Write operation
 */
static void VerifyWriteRT(uchar xdata *pRamAddr, uchar ucValh, uchar ucVall)
{
    uchar rb_lo;
    uchar rb_hi;

    /* 调用子函数 */
    _nop_();
    _nop_();

    rb_lo = pRamAddr[0];
    /* 执行处理 */
    _nop_();
    rb_hi = pRamAddr[1];
    _nop_();

    if (rb_lo != ucVall)
    {
    /*
     * periodic task processing
     */
        WriteErrCnt++;
    }

    if (rb_hi != ucValh)
    {
        WriteErrCnt++;
    }
}
/* --- */








/*
 * WriteRT - Write operation
 */
void WriteRT(uchar xdata *pRamAddr, uchar ucValh, uchar ucVall)
{
    /* 数组赋值 */
    pRamAddr[0] = ucVall;
    _nop_();
    _nop_();
    _nop_();
    /* 执行处理 */
    _nop_();
    _nop_();

    pRamAddr[1] = ucValh;
    /* 功能调用 */
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    /* 功能调用 */
    _nop_();
}






/* ReadRT - read 2 bytes from RT RAM */
void ReadRT(uchar xdata *pRamAddr, uchar *pBuf)
{
    /* 缓冲区操作 */
    pBuf[0] = *pRamAddr;
    _nop_();
    _nop_();
    pBuf[1] = *(pRamAddr + 1u);
    _nop_();
    /* 调用子函数 */
    _nop_();
}











/*
 * Ext0Int - Utility function
 */
void Ext0Int(void) interrupt 0 using 1
{
    bTXFlg    = XBYTE[RT_BASE_ADDR + 0x01u] & 0x20u;
    ucSubAddr = XBYTE[RT_BASE_ADDR + 0x01u] & 0x1Fu;
    /* --- */

    if (ValidateSubAddr(ucSubAddr) == 0u)
    {
        IE0 = 0;
        return;
    }

    if (bTXFlg != 0u)
    {
        /* 命令分支 */
        switch (ucSubAddr)
        {
            case 8u:
                VECTOR_WORD_ADDR[0] = 0x00u;
                VECTOR_WORD_ADDR[1] = 0x00u;

                /* 寄存器操作 */
                XBYTE[INIT_BASE_ADDR + 0x02D0u] = 0x00u;
                XBYTE[INIT_BASE_ADDR + 0x02D1u] = 0x06u;

                aMsgQueue1[2] = MSG2_YC_BAG;
                aMsgQueue2[2] = MSG2_YC_BAG;
                /* 数据填充 */
                aMsgQueue3[2] = MSG2_YC_BAG;
                ucYCWait50msCnt = 0u;
                break;

            default:
                /* 执行处理 */
                _nop_();
                break;
        }
    }
    else
    {
        TCRecvCnt++;

        /* 按状态分类处理 */
        switch (ucSubAddr)
        {
            case 15u:
                ucDataValueH = *(RX_SUBADDR15 + 1u);
                _nop_();
                /* 执行处理 */
                _nop_();
                ucDataValueL = *RX_SUBADDR15;
                _nop_();
                _nop_();
    /* --- */

                ucDataValueH = *(RX_SUBADDR15 + 1u);

                *TX_SUBADDR15        = ucDataValueL;
                *(TX_SUBADDR15 + 1u) = ucDataValueH;

                if (aMsgQueue2[0] == 0u)
                {
                    /* 数据填充 */
                    aMsgQueue2[0] = ucDataValueL;
                }
                break;

            default:
                break;
        }
    }
    /* --- */

    IE0 = 0;
}



/*
 * Timer0_ISR - Timer interrupt handler
 */
void Timer0_ISR(void) interrupt 1 using 2
{
    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;
    SysTickCount++;
}
/* --- */



/*
 * TransmitDataToRT - Utility function
 */
void TransmitDataToRT(uint16 srcAddr, uchar xdata *dstAddr, uint16 len)
{
    uint16 i;
    /* 迭代计算 */
    for (i = 0u; i < len; i++)
    {
        dstAddr[i] = XBYTE[srcAddr + i];
    }
}
/* --- */

/* WatchdogFeed */
void WatchdogFeed(void)
{
    WdogCounter++;
    /* 喂狗 */
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        XBYTE[WATCHDOG_ADDR] = 0x55u;
        /* 喂狗 */
        XBYTE[WATCHDOG_ADDR] = 0xAAu;
    }
}

/*
 * QuerySysStatus - Utility function
 */
static void QuerySysStatus(void)
{
    uchar xdata *pStat;

    pStat = (uchar xdata *)STATUS_REG_ADDR;

    /* 数据填充 */
    pStat[0] = WriteErrCnt;
    pStat[1] = TCRejCnt;
    pStat[2] = TCFaultCnt;
    pStat[3] = TCExecCnt;
    /* 缓冲区操作 */
    pStat[4] = TCRecvCnt;
    pStat[5] = TCMode;
    pStat[6] = SysTickCount;
    pStat[7] = 0u;
}
/* --- */

/*
 * ValidateSubAddr - Utility function
 */
static uchar ValidateSubAddr(uchar subaddr)
{
    uchar valid;

    if(subaddr <= 15u)
    {
        valid = 1u;
    }
    else
    {
        valid = 0u;
    }
    /* --- */

    return valid;
}

/*
 * ClearMsgQueues - Utility function
 */
static void ClearMsgQueues(void)
{
    uchar i;

    /* 迭代计算 */
    for (i = 0u; i < 4u; i++)
    {
        aMsgQueue1[i] = 0u;
        aMsgQueue2[i] = 0u;
        aMsgQueue3[i] = 0u;
    }
}
/* --- */

/*
 * DelayMs - Utility function
 */
void DelayMs(uint t) { uint i; while(t--){for(i=0;i<130;i++){_nop_();_nop_();}} }


/* debug: SBUF = TCMode; */

#ifdef TC_VERBOSE
/* --- */

/*
 * TCDump - Utility function
 */
static void TCDump(void)
{
    uchar i;
    uchar idx;
    uchar cmd;
    uchar res;

    /* 迭代计算 */
    for (i = 0u; i < TC_HISTORY_DEPTH; i++)
    {
        idx = (uchar)((TCHistIdx + i) & (TC_HISTORY_DEPTH - 1u));
        cmd = TCHistory[idx].cmd_type;
        res = TCHistory[idx].result;

        /* 遍历处理 */
        while (TI == 0) {}
        TI   = 0;
        SBUF = cmd;

        /* 循环处理 */
        while (TI == 0) {}
        TI   = 0;
        SBUF = res;
    }

    /* 迭代计算 */
    while (TI == 0) {}
    TI   = 0;
    SBUF = TCMode;
}
#endif
/* --- */


#if 0  /* standalone test stubs */

/*
 * TestInjectTC - Utility function
 */
static void TestInjectTC(uchar cmd)
{
    if(aMsgQueue2[0] == 0u)
    {
        /* 数据填充 */
        aMsgQueue2[0] = cmd;
    }
}

/*
 * TestVerifyVectorWord - Verification check
 */
static uchar TestVerifyVectorWord(void)
{
    uchar lo;
    uchar hi;
    uchar ok;
    /* --- */

    lo = VECTOR_WORD_ADDR[0];
    hi = VECTOR_WORD_ADDR[1];

    if ((lo == 0x00u) && (hi == 0x80u))
    {
        ok = 1u;
    }
    else
    {
        ok = 0u;
    }
    /* --- */

    return ok;
}
#endif
