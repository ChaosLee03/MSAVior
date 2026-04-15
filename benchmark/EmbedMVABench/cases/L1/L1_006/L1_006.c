//guangbo_tc_handler.c
//1553B 广播帧接收与遥控量采集
//主任务cgYC()读guangboGet[]做遥控判断
//rt_1553_isr()中断分两步写guangboGet[0]和guangboGet[1]
//处理器: 通用
//
//
//

#include <stdio.h>
#include <string.h>

/* ---- 硬件基地址和寄存器 ---- */
//
//
#define MIL1553_BASE_ADDR       0xC0000000u
#define MIL1553_STATUS_REG      (MIL1553_BASE_ADDR + 0x00u)
#define MIL1553_INT_STATUS      (MIL1553_BASE_ADDR + 0x04u)
#define MIL1553_INT_MASK        (MIL1553_BASE_ADDR + 0x08u)
#define MIL1553_CMD_REG         (MIL1553_BASE_ADDR + 0x0Cu)
#define MIL1553_CONFIG_REG      (MIL1553_BASE_ADDR + 0x10u)
#define MIL1553_RT_ADDR_REG     (MIL1553_BASE_ADDR + 0x14u)
#define MIL1553_MEM_BASE        0xC0010000u

//遥控数据接口寄存器
#define TC_CTRL_BASE            0xC0020000u
#define TC_STATUS_REG           (TC_CTRL_BASE + 0x00u)
#define TC_DATA_OUT_REG         (TC_CTRL_BASE + 0x04u)
#define TC_ACK_REG              (TC_CTRL_BASE + 0x08u)
#define WDT_KICK_REG            TC_ACK_REG   //看门狗也用这个寄存器

/* RS422状态上报接口 */
#define RS422_TX_BASE_ADDR      0xB0000000u
#define RS422_TX_DATA_REG       (RS422_TX_BASE_ADDR + 0x00u)
#define RS422_TX_LEN_REG        (RS422_TX_BASE_ADDR + 0x04u)
#define RS422_TX_CTRL_REG       (RS422_TX_BASE_ADDR + 0x08u)
#define RS422_TX_STATUS_REG     (RS422_TX_BASE_ADDR + 0x0Cu)

/* ---- 寄存器访问宏 ---- */
//
//
#define REG32(addr)             (*((volatile unsigned int *)(addr)))
#define REG16(addr)             (*((volatile unsigned short *)(addr)))
#define REG8(addr)              (*((volatile unsigned char *)(addr)))

/* ---- 系统常量 ---- */
//
//
#define GUANGBO_SUBADDR         31u
#define GUANGBO_WORD_CNT        2u
#define CGYC_GUANGBO_DATA       0u

#define TC_FRAME_LEN            512u
#define TC_CHECKSUM_OFFSET      510u

#define RT_OWN_ADDRESS          3u
#define RT_INT_ENABLE_MASK      0x0007u

#define IRQ_1553B_RT            5u

#define SYS_OK                  0u
#define SYS_ERR                 1u

/* ---- 类型定义 ---- */
//
//
typedef unsigned char  U8;
typedef unsigned short U16;
typedef unsigned int   U32;

/* ---- 环形缓冲区结构 ---- */
//
//
#define RING_BUF_SIZE           8u
#define RING_CELL_SIZE          2u

typedef struct {
    U8   pCell[RING_BUF_SIZE * RING_CELL_SIZE];
    U32  pr;
    U32  pw;
    U32  bufSize;
    U32  cellSize;
} ringBuf;

#define RING_OK     0
#define RING_EMPTY  (-1)
#define RING_FULL   (-2)

/* ---- 遥控接收状态机 ---- */
//
//
#define RC_STATE_IDLE       0u
#define RC_STATE_RECEIVING  1u
#define RC_STATE_VALIDATING 2u
#define RC_STATE_EXECUTING  3u
//
//

static volatile U8  RcStateCur  = RC_STATE_IDLE;
static volatile U32 BcstErrCnt  = 0u;
static volatile U32 RcSeqCnt    = 0u;

/* ---- 遥控执行历史环形缓冲 ---- */
//
//
#define TC_HIST_SIZE   8u

typedef struct {
    U8  word0;
    U8  word1;
    U32 timestamp;
    U8  result;
} TcHistEntry_t;

static TcHistEntry_t TcHist[TC_HIST_SIZE];
static U8            TcHistIdx = 0u;

/* ---- TC/YC状态结构体 ---- */
//
//
typedef struct {
    U32 guangbo_cnt;
    U32 tc_exec_cnt;
    U32 checksum_err;
    U8  last_guangbo0;
    U8  last_guangbo1;
    U8  tc_state;
} TcYcStat_t;

/* ---- 错误计数 ---- */
//
//
static volatile U32 FmtErrCnt   = 0u;
static volatile U32 RangeErrCnt = 0u;
static volatile U32 TimeoutCnt  = 0u;

/* ---- 运行时变量 ---- */
//
//
static volatile U32 LoopMetric   = 0u;
static volatile U32 LastIsrTick  = 0u;

/* ---- 全局变量 ---- */
//
//

/*
 * 1553B广播帧双字缓冲
 * guangboGet[0]: 广播数据高字节（遥控指令码高位）
 * guangboGet[1]: 广播数据低字节（遥控指令码低位）
 * rt_1553_isr()分两步写入，cgYC()主任务读
 */
volatile U8 guangboGet[2] = {0u, 0u};

/* 广播数据环形缓冲 */
volatile ringBuf guangbodata;

//遥控帧缓冲
static U8 TcFrame[TC_FRAME_LEN];

/* TC/YC统计 */
volatile TcYcStat_t YCRecvCnt = {0u, 0u, 0u, 0u, 0u, 0u};

//最近一次1553B中断消息数据字
static volatile U32 MsgData0 = 0u;

//系统运行标志
static volatile U8  SysRun   = 0u;

//主循环调度计数
static volatile U32 LoopCnt  = 0u;

//看门狗计数
static volatile U32 WdtCnt   = 0u;

/* ---- 函数声明 ---- */
//
//
void HwRegInit(void);
void SwVarInit(void);
void Mil1553Init(void);
void TcInterfaceInit(void);
void WatchdogFeed(void);
void DelayMs(U32 ms);
void cgYC(void);
void StatusUpload(void);
void HandleRcError(U8 err_type);
static U8   ValidateGuangboFrame(U8 word0, U8 word1);
static void RecordTcHistory(U8 word0, U8 word1, U32 ts, U8 result);
static U32  GetTotalErrCount(void);
int  ringGetCount(volatile ringBuf *rBuf);
int  ringGetOne(volatile ringBuf *rBuf, char *cell, unsigned int count);
void ringPutOne(volatile ringBuf *rBuf, char *cell);
void memCopy(U8 *sorc, U8 *tart, U32 length);
void rt_1553_isr(void);

//
//
//

/* ---- 主函数 ---- */
//
//
int main(void)
{
    HwRegInit();
    SwVarInit();
    Mil1553Init();
    TcInterfaceInit();

    WatchdogFeed();

    SysRun = 1u;

    while (1)
    {
        LoopMetric++;

        /* 遥控接收状态机 */
        switch (RcStateCur)
        {
            case RC_STATE_IDLE:
                if (ringGetCount(&guangbodata) > 0)
                {
                    RcStateCur = RC_STATE_RECEIVING;
                }
                break;

            case RC_STATE_RECEIVING:
                cgYC();
                RcStateCur = RC_STATE_VALIDATING;
                break;

            case RC_STATE_VALIDATING:
                if (ValidateGuangboFrame(guangboGet[0], guangboGet[1]) != 0u)
                {
                    RcStateCur = RC_STATE_EXECUTING;
                }
                else
                {
                    BcstErrCnt++;
                    HandleRcError(0u);
                    RcStateCur = RC_STATE_IDLE;
                }
                break;

            case RC_STATE_EXECUTING:
                REG32(TC_DATA_OUT_REG) = (U32)(((U32)guangboGet[0] << 8u) |
                                               (U32)guangboGet[1]);
                REG32(TC_ACK_REG)      = 0x00000001u;
                YCRecvCnt.tc_exec_cnt++;
                RcSeqCnt++;
                RecordTcHistory(guangboGet[0], guangboGet[1], LoopMetric, 1u);
                RcStateCur = RC_STATE_IDLE;
                break;

            default:
                RcStateCur = RC_STATE_IDLE;
                break;
        }

        //定期上报状态和喂狗
        LoopCnt++;
        if (LoopCnt >= 200u)
        {
            LoopCnt = 0u;
            StatusUpload();
            WatchdogFeed();
        }

        //
        //
        //
        //
        //
        //
        //
        //
        //
        //

        DelayMs(5u);
    }

    return 0;
}

/* ---- 硬件寄存器初始化 ---- */
//
//
void HwRegInit(void)
{
    REG32(MIL1553_CMD_REG)    = 0x00000001u;
    REG32(MIL1553_INT_STATUS) = 0xFFFFFFFFu;
    REG32(MIL1553_INT_MASK)   = 0x00000000u;
    REG32(TC_ACK_REG)         = 0x00000000u;
    REG32(TC_STATUS_REG)      = 0x00000000u;
}

/* ---- 软件变量初始化 ---- */
//
//
void SwVarInit(void)
{
    U8 i;

    guangboGet[0] = 0u;
    guangboGet[1] = 0u;

    guangbodata.pr       = 0u;
    guangbodata.pw       = 0u;
    guangbodata.bufSize  = RING_BUF_SIZE;
    guangbodata.cellSize = RING_CELL_SIZE;
    memset((void *)guangbodata.pCell, 0, sizeof(guangbodata.pCell));

    memset(TcFrame, 0, sizeof(TcFrame));

    YCRecvCnt.guangbo_cnt   = 0u;
    YCRecvCnt.tc_exec_cnt   = 0u;
    YCRecvCnt.checksum_err  = 0u;
    YCRecvCnt.last_guangbo0 = 0u;
    YCRecvCnt.last_guangbo1 = 0u;
    YCRecvCnt.tc_state      = 0u;

    RcStateCur  = RC_STATE_IDLE;
    BcstErrCnt  = 0u;
    RcSeqCnt    = 0u;

    for (i = 0u; i < TC_HIST_SIZE; i++)
    {
        TcHist[i].word0     = 0u;
        TcHist[i].word1     = 0u;
        TcHist[i].timestamp = 0u;
        TcHist[i].result    = 0u;
    }
    TcHistIdx = 0u;

    FmtErrCnt   = 0u;
    RangeErrCnt = 0u;
    TimeoutCnt  = 0u;

    LoopMetric   = 0u;
    LastIsrTick  = 0u;

    MsgData0 = 0u;
    SysRun   = 0u;
    LoopCnt  = 0u;
    WdtCnt   = 0u;
}

/* ---- 1553B RT模式初始化 ---- */
//
//
void Mil1553Init(void)
{
    REG32(MIL1553_CMD_REG) = 0x00000001u;
    DelayMs(5u);

    REG32(MIL1553_RT_ADDR_REG) = (U32)RT_OWN_ADDRESS;
    REG32(MIL1553_CONFIG_REG)  = 0x00000003u;
    REG32(MIL1553_INT_STATUS)  = 0xFFFFFFFFu;
    REG32(MIL1553_INT_MASK)    = (U32)RT_INT_ENABLE_MASK;
}

/* ---- 遥控接口初始化 ---- */
//
//
void TcInterfaceInit(void)
{
    REG32(TC_ACK_REG)    = 0x00000000u;
    REG32(TC_STATUS_REG) = 0x00000000u;
}

/* ---- 看门狗喂狗 ---- */
//外部WDT IC，翻转WDI引脚
static volatile U8 wdi_toggle = 0x00u;
void WatchdogFeed(void)
{
    volatile U8 *wdi_port = (volatile U8*)0xBFF0;
    *wdi_port = wdi_toggle;
    wdi_toggle ^= 0xFF;
    WdtCnt = 0u;
}

/* ---- 毫秒延时 ---- */
//
//
void DelayMs(U32 ms)
{
    volatile U32 i, j;
    for (i = 0u; i < ms; i++)
    {
        for (j = 0u; j < 10000u; j++) { }
    }
}

/* ---- 内存拷贝 ---- */
//
//
void memCopy(U8 *sorc, U8 *tart, U32 length)
{
    U32 i;
    for (i = 0u; i < length; i++)
    {
        *tart = *sorc;
        tart++;
        sorc++;
    }
}

/* ---- 广播帧校验 ---- */
//查值域、帧一致性、奇偶、重复帧
//
static U8 ValidateGuangboFrame(U8 word0, U8 word1)
{
    U8 parity;
    U8 bits;

    if ((word0 == 0xFFu) && (word1 == 0xFFu))
    {
        return 0u;
    }
    if ((word0 == 0x00u) && (word1 == 0x00u))
    {
        return 0u;
    }
    if (word0 > 0xF0u)
    {
        RangeErrCnt++;
        return 0u;
    }
    if (word1 > 0xF0u)
    {
        RangeErrCnt++;
        return 0u;
    }
    if (((U16)word0 + (U16)word1) > 0x1E0u)
    {
        RangeErrCnt++;
        return 0u;
    }

    //简单奇偶校验：数word0 XOR word1的置位数
    bits   = (U8)(word0 ^ word1);
    parity = 0u;
    while (bits != 0u)
    {
        parity = (U8)(parity + (bits & 1u));
        bits   = (U8)(bits >> 1u);
    }
    if ((parity & 1u) == 0u)
    {
        return 0u;
    }

    /* 重复帧检测：连续相同帧拒绝 */
    if ((YCRecvCnt.last_guangbo0 == word0) &&
        (YCRecvCnt.last_guangbo1 == word1) &&
        (RcSeqCnt > 0u))
    {
        return 0u;
    }

    return 1u;
}

/* ---- 遥控执行历史记录 ---- */
//
//
static void RecordTcHistory(U8 word0, U8 word1, U32 ts, U8 result)
{
    TcHist[TcHistIdx].word0     = word0;
    TcHist[TcHistIdx].word1     = word1;
    TcHist[TcHistIdx].timestamp = ts;
    TcHist[TcHistIdx].result    = result;
    TcHistIdx = (U8)((TcHistIdx + 1u) % TC_HIST_SIZE);
}

/* ---- 遥控错误处理 ---- */
//分类帧格式、值域、超时错误
//
void HandleRcError(U8 err_type)
{
    switch (err_type)
    {
        case 0u:
            FmtErrCnt++;
            YCRecvCnt.checksum_err++;
            REG32(RS422_TX_DATA_REG) = 0xE0000001u;
            REG32(RS422_TX_LEN_REG)  = 4u;
            REG32(RS422_TX_CTRL_REG) = 0x00000001u;
            break;

        case 1u:
            RangeErrCnt++;
            REG32(RS422_TX_DATA_REG) = 0xE0000002u;
            REG32(RS422_TX_LEN_REG)  = 4u;
            REG32(RS422_TX_CTRL_REG) = 0x00000001u;
            break;

        case 2u:
            TimeoutCnt++;
            YCRecvCnt.tc_state |= 0x40u;
            REG32(RS422_TX_DATA_REG) = 0xE0000004u;
            REG32(RS422_TX_LEN_REG)  = 4u;
            REG32(RS422_TX_CTRL_REG) = 0x00000001u;
            break;

        default:
            FmtErrCnt++;
            break;
    }

    /* 更新全局错误状态 */
    if ((FmtErrCnt + RangeErrCnt + TimeoutCnt) > 100u)
    {
        YCRecvCnt.tc_state |= 0x20u;
    }
}

/* ---- 查询各类错误总数 ---- */
//
//
static U32 GetTotalErrCount(void)
{
    return (FmtErrCnt + RangeErrCnt + TimeoutCnt + BcstErrCnt);
}

/* ---- 环形缓冲：查可用消息数 ---- */
//
//
int ringGetCount(volatile ringBuf *rBuf)
{
    int cnt;
    if (rBuf->pw >= rBuf->pr)
    {
        cnt = (int)(rBuf->pw - rBuf->pr);
    }
    else
    {
        cnt = (int)(rBuf->bufSize - rBuf->pr + rBuf->pw);
    }
    return cnt;
}

/* ---- 环形缓冲：取一条消息 ---- */
//
//
int ringGetOne(volatile ringBuf *rBuf, char *cell, unsigned int count)
{
    unsigned int i;
    char *src;

    if (rBuf->pr == rBuf->pw)
    {
        return RING_EMPTY;
    }

    src = (char *)(&(rBuf->pCell[(rBuf->pr) * (rBuf->cellSize)]));
    for (i = 0u; i < count; i++)
    {
        cell[i] = src[i];
    }

    rBuf->pr = (rBuf->pr + 1u) % rBuf->bufSize;

    return RING_OK;
}

/* ---- 环形缓冲：写一条消息 ---- */
//
//
void ringPutOne(volatile ringBuf *rBuf, char *cell)
{
    unsigned int i;
    char *dst;
    U32 next_pw;

    next_pw = (rBuf->pw + 1u) % rBuf->bufSize;
    if (next_pw == rBuf->pr)
    {
        return;
    }

    dst = (char *)(&(rBuf->pCell[(rBuf->pw) * (rBuf->cellSize)]));
    for (i = 0u; i < rBuf->cellSize; i++)
    {
        dst[i] = cell[i];
    }

    rBuf->pw = next_pw;
}

/* ---- 广播数据采集和遥控处理（主任务） ---- */
//从环形缓冲取广播数据到guangboGet，拼遥控帧
//
void cgYC(void)
{
    U32 i;
    U8  SUM = 0u;
    U8 *pU8 = TcFrame;

    //
    //
    //
    //
    if (ringGetCount(&guangbodata) > 0)
    {
        ringGetOne(&guangbodata, (char *)(&guangboGet[0]), 2);
    }

    memCopy((U8 *)(&guangboGet[0]), &pU8[CGYC_GUANGBO_DATA], GUANGBO_WORD_CNT);

    for (i = 0u; i < (U32)TC_CHECKSUM_OFFSET; i++)
    {
        SUM = (U8)(SUM + pU8[i]);
    }
    pU8[TC_CHECKSUM_OFFSET] = SUM;

    YCRecvCnt.last_guangbo0 = guangboGet[0];
    YCRecvCnt.last_guangbo1 = guangboGet[1];
    YCRecvCnt.guangbo_cnt++;
}

/* ---- 状态遥测上报 ---- */
//
//
void StatusUpload(void)
{
    U32 status_word;

    status_word = (U32)YCRecvCnt.guangbo_cnt;
    REG32(RS422_TX_DATA_REG) = status_word;
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;

    REG32(RS422_TX_DATA_REG) = (U32)BcstErrCnt;
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;

    REG32(RS422_TX_DATA_REG) = (U32)RcSeqCnt;
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;

    REG32(RS422_TX_DATA_REG) = FmtErrCnt;
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;

    REG32(RS422_TX_DATA_REG) = RangeErrCnt;
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;

    REG32(RS422_TX_DATA_REG) = TimeoutCnt;
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;

    //printf("StatusUpload: guangbo=%u err=%u\n", YCRecvCnt.guangbo_cnt, BcstErrCnt);
    REG32(RS422_TX_DATA_REG) = GetTotalErrCount();
    REG32(RS422_TX_LEN_REG)  = 4u;
    REG32(RS422_TX_CTRL_REG) = 0x00000001u;
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

/* ---- 1553B RT中断服务程序 ---- */
//广播帧收到后触发，读消息寄存器
//分两步写guangboGet
void rt_1553_isr(void)
{
    U32  int_status;
    U32  gbdata0;
    char cell_buf[2];

    int_status = REG32(MIL1553_INT_STATUS);
    REG32(MIL1553_INT_STATUS) = int_status;

    gbdata0    = REG32(MIL1553_MEM_BASE);
    MsgData0   = gbdata0;

    guangboGet[1] = (U8)(gbdata0 & 0xFFu);
    guangboGet[0] = (U8)((gbdata0 >> 8) & 0xFFu);

    cell_buf[0] = (char)guangboGet[0];
    cell_buf[1] = (char)guangboGet[1];
    ringPutOne(&guangbodata, cell_buf);

    LastIsrTick = LoopMetric;

    WdtCnt++;
    if (WdtCnt >= 10000u)
    {
        YCRecvCnt.tc_state |= 0x80u;
    }
}
