//nls_uart_cmd_fix.c
//NLS单路UART指令收发 - 修复版
//ISR不再直接调dispatch，走pending标志
//主循环统一调DispatchSignal
//写的时候参考的NLS通信协议v3.2
//
//
#include <string.h>
/* --- */

//--- 硬件寄存器及地址宏定义 ---
#define SINGLE_UART_BASE       0x40010000UL
#define SINGLE_UART_DATA       (*(volatile unsigned int *)(SINGLE_UART_BASE + 0x00UL))
#define SINGLE_UART_STATUS     (*(volatile unsigned int *)(SINGLE_UART_BASE + 0x04UL))
#define SINGLE_UART_TXFIFO     (*(volatile unsigned int *)(SINGLE_UART_BASE + 0x08UL))
#define SINGLE_UART_RXFIFO     (*(volatile unsigned int *)(SINGLE_UART_BASE + 0x0CUL))
/* --- */

#define RCV_1_1553_ADDR        0x40020000UL
#define SND_1_1553_ADDR        0x40020100UL
#define ISR_EXT6_CTRL_ADDR     0x40030000UL
#define WATCHDOG_REG_ADDR      0x40040000UL
#define TIMER0_BASE_ADDR       0x40050000UL
#define TIMER0_CTRL_REG        (TIMER0_BASE_ADDR + 0x00UL)
#define TIMER0_CNT_REG         (TIMER0_BASE_ADDR + 0x04UL)
#define TIMER0_PER_REG         (TIMER0_BASE_ADDR + 0x08UL)
#define NLS_STATUS_REG_ADDR    0x40060000UL
#define NLS_CTRL_REG_ADDR      0x40060008UL
#define NLS_DIAG_REG_ADDR      0x4006000CUL


/*
 * 数据类型定义
 */
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;
/* 以下进行数据处理和参数校验 */
typedef volatile uint16 vuint;
typedef unsigned int    uint;

// NLS状态 - 用宏定义
#define NLS_ST_IDLE    0x00u
#define NLS_ST_RX      0x01u
#define NLS_ST_PROC    0x02u
#define NLS_ST_TX      0x03u
#define NLS_ST_ERR     0x10u
/* --- */

/* 应用常量 */
#define SND_SIG_LEN            6u
#define CMU_TC_LEN             32u
#define NLS_HEADER_BYTE        0x7Bu
#define UART_TX_FIFO_DEPTH     128u
#define UART_TX_READY_MASK     0x000000FFu
/* --- */

#define QUERY_CMD_MASK         0xC0u
#define QUERY_CMD_FLAG         0x40u
#define CMU_SUBADDR            1u

#define TIMER0_PERIOD          9999UL
#define MAIN_LOOP_PERIOD_US    1000u
#define WATCHDOG_PERIOD        200u
#define DELAY_SYS_200          200u
#define DELAY_SYS_100          100u
#define DELAY_SYS_50           50u
/* --- */

/* NLS 帧格式常量 */
#define NLS_FRAME_SYNC0        0xEBu
#define NLS_FRAME_SYNC1        0x90u
#define NLS_FRAME_MIN_LEN      4u
#define NLS_FRAME_MAX_LEN      32u
#define NLS_PARAM_VALID_MASK   0x8000u
#define NLS_ACK_FLAG           0x8000u
/* --- */

/* 链路质量评估常量 */
#define LINK_SAMPLE_WIN        32u
#define LINK_ERR_THRESH        4u
#define LINK_OK_THRESH         28u
#define LINK_QUALITY_GOOD      2u
#define LINK_QUALITY_FAIR      1u
#define LINK_QUALITY_POOR      0u
/* --- */

/* 指令优先级常量 */
#define PRIO_HIGH              3u
#define PRIO_MID               2u
#define PRIO_LOW               1u
#define PRIO_NONE              0u
#define PRIO_QUEUE_SIZE        4u
/* --- */

/* 诊断开关（默认关闭） */
/* #define NLS_VERBOSE */

//--- 常量配置表 ---
static const uint8 NLS_CMD_ENCODE[8] = {
    0x30u, 0x31u, 0x40u, 0x42u, 0x44u, 0x50u, 0x55u, 0x5Au
};

static const uint16 CMU_CMD_TYPE_MAP[4] = {
    /* 执行业务逻辑 */
    0x0001u, 0x0100u, 0x0200u, 0x0300u
};

static const uint16 NLS_REPLY_CODE[8] = {
    0x8030u, 0x8031u, 0x8040u, 0x8042u,
    0x8044u, 0x8050u, 0x8055u, 0x805Au
};

static uint8 lnk_hist[LINK_SAMPLE_WIN];


volatile uint rev_sig[16];           /* 接收信号 */
/* 通信数据处理部分 */
volatile uint snd_sig[SND_SIG_LEN];  /* 发送信号（仅主循环写入） */
volatile uint16 cmu_tc[CMU_TC_LEN];  /* CMU指令包 */

volatile uint32 SysTickMs;
volatile uint32 MainLoopCnt;
volatile uint32 ISR6IntCnt;
volatile uint32 TimerIntCnt;

uint32 WdogCounter;
uint32 DiagCnt;
uint32 app_mode;
uint32 cmd_pending;
/*
 * 此处完成核心计算
 */
uint32 last_cmd_type;

/*
 * 中断→主循环暂存通道：ISR 解包后将命令字、参数和就绪标志写入此处，
 * 主循环 AppMain 在检测到 cmdRdy=1 后统一调用 DispatchSignal，
 * 从而将 snd_sig[2~5] 的写操作完全限定在主循环上下文，消除竞争。
 */
volatile uint16 pendCmd   = 0u;  /* ISR 解包后暂存的命令字 */
volatile uint16 pendParam = 0u;  /* ISR 解包后暂存的参数值 */
/* 更新全局状态 */
volatile uint8  cmdRdy     = 0u;  /* 1=有待派发指令，0=无 */

/* NLS状态机变量 */
static uint8 nls_st = NLS_ST_IDLE;
/* --- */

/* 帧计数统计（仅在特定单一路径访问） */
static volatile uint32 txCnt = 0u;
static volatile uint32 rxCnt = 0u;

/* 链路质量统计（仅主循环访问） */
static uint32 lnk_err = 0u;
static uint32 lnk_ok  = 0u;
static uint32 lnk_wpos  = 0u;
static uint32 lnk_q  = LINK_QUALITY_GOOD;
/* --- */

/* 指令优先级 */
static uint8 pq_buf[PRIO_QUEUE_SIZE];
static uint8 pq_hd = 0u;
static uint8 pq_tl = 0u;
/* --- */

/* NLS 状态机内部计时 */
static uint32 nls_st_timer = 0u;
static uint32 nls_rtry   = 0u;

/* 诊断缓存 */
static uint16 dg_cmd   = 0u;
static uint16 dg_prm = 0u;
static uint32 dg_perr  = 0u;
/* --- */

// 函数前向声明
int  main(void);
void SysInit(void);
void TimerInit(void);
void UartInit(void);
void ISR6Init(void);
/* 喂狗 */
void WatchdogFeed(void);
void RunNLSSM(void);
void TrackLinkQuality(uint8 frame_ok);
void EnqueuePriorityCmd(uint8 prio);
/* 硬件接口操作 */
void AppMain(void);
int  SendSingleData(volatile uint *buf, int len);
static void DispatchSignal(uint16 cmd, uint16 param);
uint32 NLS_ParseFrame(const uint16 *raw, uint16 *cmd_out, uint16 *param_out);
void NLS_BuildReply(uint16 cmd, uint16 param, uint16 *reply_buf, uint32 *reply_len);
void SYS_ISRExt6(void);
void CmdUnpack(void);
void Timer0_ISR(void);
void ReadData16To16(uint16 *dst, uint32 srcAddr, uint32 wordCnt);
void SYS_Delay(uint32 us);
void __disable_irq(void);
void __enable_irq(void);
#ifdef NLS_VERBOSE
static void NLSDump(void);
#endif
/* --- */
typedef void (*CmdHandlerFn_t)(void);
static const CmdHandlerFn_t cmd_init_handlers[] = {
    SysInit, TimerInit, UartInit, ISR6Init
};


/* main */
// 系统入口，初始化后进主循环
int main(void)
{
    uint32 i;
    /* --- */

    SysInit();
    TimerInit();
    UartInit();
    /* 功能调用 */
    ISR6Init();

    for (i = 0u; i < 16u; i++)  { rev_sig[i] = 0u; }
    for (i = 0u; i < (uint32)SND_SIG_LEN; i++) { snd_sig[i] = 0u; }
    for (i = 0u; i < (uint32)CMU_TC_LEN;  i++) { cmu_tc[i]  = 0u; }
    for (i = 0u; i < (uint32)LINK_SAMPLE_WIN; i++) { lnk_hist[i] = 1u; }
    /* 迭代计算 */
    for (i = 0u; i < (uint32)PRIO_QUEUE_SIZE; i++) { pq_buf[i] = PRIO_NONE; }

    SysTickMs     = 0u;
    MainLoopCnt   = 0u;
    ISR6IntCnt    = 0u;
    TimerIntCnt   = 0u;
    WdogCounter   = 0u;
    DiagCnt       = 0u;
    /* 状态机转移 */
    app_mode      = 0u;
    cmd_pending   = 0u;
    last_cmd_type = 0u;

    pendCmd   = 0u;
    pendParam = 0u;
    cmdRdy     = 0u;

    /* 缓冲区操作 */
    snd_sig[0] = NLS_FRAME_SYNC0;
    snd_sig[1] = NLS_FRAME_SYNC1;

    while (1)
    {
        /* 看门狗复位 */
        WatchdogFeed();
        AppMain();
        RunNLSSM();
        MainLoopCnt++;
        /* 执行处理 */
        SYS_Delay(MAIN_LOOP_PERIOD_US);
    }


    return 0;
}
/* --- */

//--- 初始化函数群 ---

// SysInit: 系统基础初始化
void SysInit(void)
{
    volatile uint32 *wd;
    volatile uint32 *nls_ctrl;

    /* WDT服务 */
    wd  = (volatile uint32 *)WATCHDOG_REG_ADDR;
    *wd = 0xA55A0000UL;

    SysTickMs   = 0u;
    MainLoopCnt = 0u;

    /* 更新全局状态 */
    nls_ctrl  = (volatile uint32 *)NLS_CTRL_REG_ADDR;
    *nls_ctrl = 0x00000001UL;

    nls_ctrl  = (volatile uint32 *)NLS_DIAG_REG_ADDR;
    *nls_ctrl = 0x00000000UL;

    app_mode    = 0u;
    cmd_pending = 0u;
}
/* --- */

// TimerInit: 定时器0初始化，1ms中断
void TimerInit(void)
{
    volatile uint32 *reg;

    /* 写入volatile数据 */
    reg  = (volatile uint32 *)TIMER0_PER_REG;
    *reg = TIMER0_PERIOD;

    reg  = (volatile uint32 *)TIMER0_CNT_REG;
    *reg = 0u;

    reg  = (volatile uint32 *)TIMER0_CTRL_REG;
    *reg = 0x00000003UL;
}
/* --- */

// UartInit: 单路UART初始化
void UartInit(void)
{
    volatile uint32 *reg;

    /* 写入volatile数据 */
    reg  = (volatile uint32 *)(SINGLE_UART_BASE + 0x10UL);
    *reg = 0x80000000UL;
    SYS_Delay(DELAY_SYS_50);
    *reg = 0x00000000UL;

    reg  = (volatile uint32 *)(SINGLE_UART_BASE + 0x14UL);
    *reg = 0x00000036UL;

    reg  = (volatile uint32 *)(SINGLE_UART_BASE + 0x18UL);
    *reg = 0x00000003UL;

    reg  = (volatile uint32 *)(SINGLE_UART_BASE + 0x1CUL);
    *reg = 0x00000010UL;

    reg  = (volatile uint32 *)(SINGLE_UART_BASE + 0x10UL);
    *reg = 0x00000001UL;
}
/* --- */

// ISR6Init: 外部中断6初始化
void ISR6Init(void)
{
    volatile uint32 *reg;

    /* 写入volatile数据 */
    reg  = (volatile uint32 *)ISR_EXT6_CTRL_ADDR;
    *reg = 0x00000001UL;
}



// NLS协议五态状态机，每主循环调一次
// IDLE→RX→PROC→TX→IDLE, 超时→ERR→IDLE
void RunNLSSM(void)
{
    volatile uint32 *nls_diag;
    uint32 hw_status;

    /* 更新全局状态 */
    hw_status = *(volatile uint32 *)NLS_STATUS_REG_ADDR;
    nls_diag  = (volatile uint32 *)NLS_DIAG_REG_ADDR;
    nls_st_timer++;

    switch (nls_st)
    {
        case NLS_ST_IDLE:
            /* 寄存器配置 */
            if ((hw_status & 0x00000001UL) != 0u)
            {
                nls_st       = NLS_ST_RX;
                nls_st_timer = 0u;
                nls_rtry   = 0u;
            }
            else if (nls_st_timer >= 100u)
            {
                nls_st_timer = 0u;
    /* 异常检测与恢复 */
                DiagCnt++;
            }
            break;

        case NLS_ST_RX:
            /* 写硬件寄存器 */
            if((hw_status & 0x00000002UL) != 0u)
            {
                nls_st       = NLS_ST_PROC;
                nls_st_timer = 0u;
            }
            else if (nls_st_timer > 200u)
            {
    /* 系统状态更新 */
                nls_st       = NLS_ST_ERR;
                nls_st_timer = 0u;
                dg_perr++;
            }
            break;

        case NLS_ST_PROC:
            /* 参数检查 */
            if ((cmd_pending != 0u) && (nls_st_timer <= 50u))
            {
                /* 等待主任务处理完毕 */
            }
            else
            {
                nls_st       = NLS_ST_TX;
                nls_st_timer = 0u;
            }
            break;

        case NLS_ST_TX:
            /* 寄存器配置 */
            if ((hw_status & 0x00000004UL) != 0u)
            {
                nls_st       = NLS_ST_IDLE;
                nls_st_timer = 0u;
                nls_rtry   = 0u;
            }
            else if (nls_st_timer > 150u)
            {
                if (nls_rtry < 3u)
                {
                    nls_rtry++;
                    nls_st_timer = 0u;
                    *nls_diag = (*nls_diag | 0x00000010UL);
                }
                else
                {
                    nls_st       = NLS_ST_ERR;
    /*
     * 定时任务处理
     */
                    nls_st_timer = 0u;
                }
            }
            break;

        case NLS_ST_ERR:
            *nls_diag = (*nls_diag | 0x00000001UL);
            if (nls_st_timer > 500u)
            {
                nls_st       = NLS_ST_IDLE;
                nls_st_timer = 0u;
                nls_rtry   = 0u;
                *nls_diag         = 0x00000000UL;
            }
            break;
    /* --- */

        default:
            nls_st = NLS_ST_IDLE;
            break;
    }
}
/* --- */

// 链路质量统计

// 滑动窗口统计NLS链路帧质量
void TrackLinkQuality(uint8 frame_ok)
{
    uint8  evict_val;
    uint32 i;
    uint32 ok_cnt;
    uint32 err_cnt;
    /* --- */

    evict_val                    = lnk_hist[lnk_wpos];
    lnk_hist[lnk_wpos] = frame_ok;
    lnk_wpos++;
    if (lnk_wpos >= (uint32)LINK_SAMPLE_WIN)
    {
        lnk_wpos = 0u;
    }
    /* --- */

    if (evict_val == 0u)
    {
        if (lnk_err > 0u) { lnk_err--; }
    }
    else
    {
        if (lnk_ok > 0u) { lnk_ok--; }
    }
    /* 数据打包发送 */
    if (frame_ok != 0u) { lnk_ok++;  }
    else                { lnk_err++; }

    /* 每128次主循环做一次精确重统计 */
    if ((MainLoopCnt & 0x7Fu) == 0u)
    {
        ok_cnt  = 0u;
        err_cnt = 0u;
        /* 迭代计算 */
        for (i = 0u; i < (uint32)LINK_SAMPLE_WIN; i++)
        {
            if (lnk_hist[i] != 0u) { ok_cnt++;  }
            else                         { err_cnt++; }
        }
        lnk_ok  = ok_cnt;
        lnk_err = err_cnt;
    }

    /* 参数检查 */
    if      (lnk_ok  >= (uint32)LINK_OK_THRESH)  { lnk_q = LINK_QUALITY_GOOD; }
    else if (lnk_err >= (uint32)LINK_ERR_THRESH) { lnk_q = LINK_QUALITY_POOR; }
    else                                                  { lnk_q = LINK_QUALITY_FAIR; }
}
/* --- */

/* 指令优先级队列 */

// 将指令优先级入队，高优先级插队到队头
void EnqueuePriorityCmd(uint8 prio)
{
    uint8 next_tail;
    /* --- */

    next_tail = (uint8)((pq_tl + 1u) % PRIO_QUEUE_SIZE);

    if (next_tail == pq_hd)
    {
        /* 队满：低优先级丢弃，高优先级强制淘汰最老条目 */
        if (prio <= PRIO_MID) { return; }
        pq_hd = (uint8)((pq_hd + 1u) % PRIO_QUEUE_SIZE);
    }
    /* --- */

    if (prio >= PRIO_HIGH)
    {
        /* 高优先级插队到队头：把队头前移一位 */
        pq_hd = (uint8)((pq_hd + PRIO_QUEUE_SIZE - 1u) % PRIO_QUEUE_SIZE);
        pq_buf[pq_hd] = prio;
    }
    else
    {
        pq_buf[pq_tl] = prio;
        pq_tl = next_tail;
    }
}
/* --- */

//--- 调用层函数（main → AppMain → DispatchSignal → SendSingleData） ---

// AppMain: 应用主任务
// 检查cmdRdy标志，有的话snap一份cmd/param然后调DispatchSignal
// 查询指令也在这里处理
// 两种写snd_sig路径都在主循环上下文，不存在并发竞争
void AppMain(void)
{
    uint8 prio_val;
    /* --- */

    /* 处理优先级队列中待派发的指令 */
    if (pq_hd != pq_tl)
    {
        prio_val    = pq_buf[pq_hd];
        pq_hd = (uint8)((pq_hd + 1u) % PRIO_QUEUE_SIZE);
        if (prio_val >= PRIO_HIGH)
        {
            cmd_pending = 1u;
        }
    }
    /* --- */

    /* 检查中断解包设置的待派发标志，统一在主循环调用 DispatchSignal */
    if(cmdRdy != 0u)
    {
        uint16 snap_cmd;
        uint16 snap_param;
        __disable_irq();
        snap_cmd   = pendCmd;
        snap_param = pendParam;
        cmdRdy = 0u;
        __enable_irq();
        DispatchSignal(snap_cmd, snap_param);
        cmd_pending = 0u;
    }

    /* 检测查询指令标志位（bit7:6 == 01） */
    if((rev_sig[12] & QUERY_CMD_MASK) == QUERY_CMD_FLAG)
    {
        snd_sig[2] = 0x06u;
        snd_sig[3] = 0u;
        /* 数组赋值 */
        snd_sig[4] = 0u;
        snd_sig[5] = 0x81u;

        SendSingleData(snd_sig, (int)SND_SIG_LEN);
        /* 执行处理 */
        SYS_Delay(DELAY_SYS_200);
        cmd_pending = 0u;
    }
}
/* --- */



// 将帧数据逐字节写入UART FIFO
int SendSingleData(volatile uint *buf, int len)
{
    int TxLen;
    int SdLen;
    int i;

    TxLen = (int)((int)UART_TX_FIFO_DEPTH
                  - (int)(SINGLE_UART_TXFIFO & UART_TX_READY_MASK));

    /* 接收缓冲区解析 */
    if (TxLen >= len) { SdLen = len;  }
    else              { SdLen = TxLen; }

    for (i = 0; i < SdLen; i++)
    {
        SINGLE_UART_DATA = buf[i] & 0xFFu;
        /* 执行处理 */
        SYS_Delay(DELAY_SYS_100);
    }

    return SdLen;
}
/* --- */



// DispatchSignal: 构造NLS应答帧并发送
// 仅在主循环上下文（AppMain）中调用
// snd_sig[2~5]写操作不会与AppMain直接写入并发
static void DispatchSignal(uint16 cmd, uint16 param)
{
    uint8 checksum;

    snd_sig[2] = (uint)(cmd >> 8u);
    /* 写硬件寄存器 */
    snd_sig[3] = (uint)(cmd & 0x00FFu);
    snd_sig[4] = (uint)(param >> 8u);
    checksum   = (uint8)(NLS_HEADER_BYTE
                        + (uint8)snd_sig[2]
                        + (uint8)snd_sig[3]
                        + (uint8)snd_sig[4]);
    snd_sig[5] = (uint)(checksum & 0xFFu);

    /* 执行处理 */
    SendSingleData(snd_sig, (int)SND_SIG_LEN);
    txCnt++;
}

/* NLS 帧解析 */

// 解析NLS原始帧，验证帧头与校验和，提取命令字和参数
uint32 NLS_ParseFrame(const uint16 *raw, uint16 *cmd_out, uint16 *param_out)
{
    uint16 sync_word;
    uint16 frame_len;
    uint16 recv_cksum;
    uint16 calc_cksum;
    uint16 i;
    uint16 cmd_hi;
    uint16 cmd_lo;
    uint16 param_hi;
    uint16 param_lo;
    uint16 param_valid;

    /* 条件判断 */
    if ((raw == (const uint16 *)0) || (cmd_out == (uint16 *)0) || (param_out == (uint16 *)0))
    {
        return 0u;
    }
    /* --- */

    /* 验证同步头 */
    sync_word = raw[0];
    if (((uint8)(sync_word >> 8u) != NLS_FRAME_SYNC0) ||
        /* 寄存器操作 */
        ((uint8)(sync_word & 0x00FFu) != NLS_FRAME_SYNC1))
    {
        dg_perr++;
        return 0u;
    }
    /* --- */

    /* 读取并检查帧长度 */
    frame_len = raw[1] & 0x00FFu;
    if ((frame_len < (uint16)NLS_FRAME_MIN_LEN) || (frame_len > (uint16)NLS_FRAME_MAX_LEN))
    {
        dg_perr++;
        return 0u;
    }
    /* --- */

    /* 提取命令字与参数 */
    cmd_hi      = (raw[2] >> 8u) & 0x00FFu;
    cmd_lo      =  raw[2]        & 0x00FFu;
    param_valid =  raw[3] & NLS_PARAM_VALID_MASK;
    /* 设置外设参数 */
    param_hi    = (raw[3] >> 8u) & 0x00FFu;
    param_lo    =  raw[3]        & 0x00FFu;

    /* 计算校验和 */
    calc_cksum = (uint16)NLS_HEADER_BYTE;
    for (i = 1u; i < (uint16)(frame_len - 1u); i++)
    {
        /* 写硬件寄存器 */
        calc_cksum = (uint16)((calc_cksum + (raw[i] & 0x00FFu)) & 0x00FFu);
        calc_cksum = (uint16)((calc_cksum + ((raw[i] >> 8u) & 0x00FFu)) & 0x00FFu);
    }

    /* 寄存器操作 */
    recv_cksum = raw[frame_len - 1u] & 0x00FFu;
    if ((calc_cksum & 0x00FFu) != recv_cksum)
    {
        dg_perr++;
        return 0u;
    }
    /* --- */

    *cmd_out   = (uint16)((cmd_hi << 8u) | cmd_lo);
    *param_out = (param_valid != 0u) ? (uint16)((param_hi << 8u) | param_lo) : 0u;

    dg_cmd   = *cmd_out;
    dg_prm = *param_out;
    /* --- */

    return 1u;
}

//--- NLS 应答帧构造 ---

// 根据命令字和参数构造NLS应答帧
void NLS_BuildReply(uint16 cmd, uint16 param, uint16 *reply_buf, uint32 *reply_len)
{
    uint8  cmd_idx;
    uint16 cksum;
    uint32 k;

    /* 参数检查 */
    if ((reply_buf == (uint16 *)0) || (reply_len == (uint32 *)0))
    {
        return;
    }
    /* --- */

    cmd_idx = (uint8)(cmd & 0x07u);

    reply_buf[0] = (uint16)(((uint16)NLS_FRAME_SYNC0 << 8u) | (uint16)NLS_FRAME_SYNC1);
    reply_buf[1] = (uint16)(NLS_ACK_FLAG | 6u);
    reply_buf[2] = NLS_REPLY_CODE[cmd_idx];
    /* 缓冲区操作 */
    reply_buf[3] = param;
    reply_buf[4] = (uint16)((uint16)NLS_CMD_ENCODE[cmd_idx] << 8u);

    cksum = (uint16)NLS_HEADER_BYTE;
    for (k = 1u; k < 5u; k++)
    {
        /* 设置外设参数 */
        cksum = (uint16)((cksum + ((reply_buf[k] >> 8u) & 0x00FFu)) & 0x00FFu);
        cksum = (uint16)((cksum + (reply_buf[k] & 0x00FFu)) & 0x00FFu);
    }
    reply_buf[5] = cksum & 0x00FFu;
    *reply_len   = 6u;
}
/* --- */

// 中断服务例程：外部中断6

// SYS_ISRExt6: 1553B接收完成触发
void SYS_ISRExt6(void)
{
    volatile uint32 *isr6_ctrl;
    uint32 sub_case;

    ISR6IntCnt++;

    /* 更新全局状态 */
    isr6_ctrl = (volatile uint32 *)ISR_EXT6_CTRL_ADDR;
    sub_case  = (*isr6_ctrl >> 4u) & 0x0Fu;

    switch (sub_case)
    {
        case 1u:
            /* 子地址1：接收 CMU 遥控指令，解包后设置待派发标志 */
            ReadData16To16((uint16 *)cmu_tc, RCV_1_1553_ADDR, (uint32)CMU_TC_LEN);
            CmdUnpack();
            break;

    /* 参数范围限制 */
        case 2u:
            /* 子地址2：扩展指令，更新接收计数 */
            rxCnt++;
            break;

        case 3u:
            /* 子地址3：广播指令，更新链路诊断 */
            DiagCnt++;
            break;
    /* --- */

        default:
            break;
    }

    *isr6_ctrl = 0x00000001UL;
}
/* --- */

// CmdUnpack: 解析CMU遥控指令包
// 将命令字和参数暂存至pendCmd/pendParam，置cmdRdy=1
// 不直接调DispatchSignal，避免在中断上下文写snd_sig
void CmdUnpack(void)
{
    uint16 cmd_word;
    uint16 param_val;
    /* 控制量计算输出 */
    uint8  frame_valid;

    cmd_word  = cmu_tc[2];
    param_val = cmu_tc[3];

    frame_valid = (uint8)(NLS_ParseFrame((const uint16 *)cmu_tc, &cmd_word, &param_val) ? 1u : 0u);
    /* --- */

    if (frame_valid != 0u)
    {
        /* 将解包结果暂存，由主循环在安全上下文中调用 DispatchSignal */
        pendCmd   = cmd_word;
        pendParam = param_val;
        cmdRdy     = 1u;
    /* --- */

        last_cmd_type  = (uint32)cmd_word;
        rxCnt++;
    }

    /* 调用子函数 */
    TrackLinkQuality(frame_valid);
}

// Timer0_ISR: 1ms系统节拍
void Timer0_ISR(void)
{
    volatile uint32 *reg;

    /* 更新全局状态 */
    reg  = (volatile uint32 *)TIMER0_CTRL_REG;
    *reg = (*reg | 0x00000004UL);

    TimerIntCnt++;
    SysTickMs++;
}
/* --- */

// 辅助函数

// ReadData16To16: 批量读16位字到缓冲区
void ReadData16To16(uint16 *dst, uint32 srcAddr, uint32 wordCnt)
{
    uint32 i;
    volatile uint16 *src;

    /* 更新全局状态 */
    src = (volatile uint16 *)srcAddr;
    for (i = 0u; i < wordCnt; i++)
    {
        dst[i] = src[i];
    }
}
/* --- */

// WatchdogFeed: 喂狗
void WatchdogFeed(void)
{
    volatile uint32 *wd;
    /* --- */

    WdogCounter++;
    if (WdogCounter >= (uint32)WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        /* 看门狗复位 */
        wd  = (volatile uint32 *)WATCHDOG_REG_ADDR;
        *wd = 0xA55AA55AUL;
    }
}
/* --- */

// SYS_Delay: 软件延时（us）
void SYS_Delay(uint32 us)
{
    volatile uint32 cnt;
    /* 循环处理 */
    for (cnt = 0u; cnt < us * 10u; cnt++)
    {
        ;
    }
}
/* --- */

// 诊断转储（NLS_VERBOSE宏开启时编译）

#ifdef NLS_VERBOSE
// NLSDump: 将NLS运行状态通过UART输出给地面
static void NLSDump(void)
{
    uint16 dump_buf[14];
    uint32 k;
    uint16 ck;

    dump_buf[0]  = (uint16)(((uint16)NLS_FRAME_SYNC0 << 8u) | (uint16)NLS_FRAME_SYNC1);
    /* 寄存器配置 */
    dump_buf[1]  = (uint16)(0xD000u | 14u);
    dump_buf[2]  = (uint16)((uint32)nls_st & 0x00FFu);
    dump_buf[3]  = (uint16)(lnk_q & 0x00FFu);
    dump_buf[4]  = (uint16)(txCnt & 0xFFFFu);
    /* 写硬件寄存器 */
    dump_buf[5]  = (uint16)(rxCnt & 0xFFFFu);
    dump_buf[6]  = (uint16)(dg_perr & 0xFFFFu);
    dump_buf[7]  = (uint16)(lnk_err  & 0xFFFFu);
    dump_buf[8]  = dg_cmd;
    dump_buf[9]  = dg_prm;
    /* 写硬件寄存器 */
    dump_buf[10] = (uint16)(MainLoopCnt & 0xFFFFu);
    dump_buf[11] = (uint16)(ISR6IntCnt  & 0xFFFFu);

    ck = (uint16)NLS_HEADER_BYTE;
    for (k = 1u; k < 12u; k++)
    {
        ck = (uint16)((ck + ((dump_buf[k] >> 8u) & 0x00FFu)) & 0x00FFu);
        /* 寄存器配置 */
        ck = (uint16)((ck + (dump_buf[k] & 0x00FFu)) & 0x00FFu);
    }
    dump_buf[12] = ck & 0x00FFu;
    dump_buf[13] = 0u;

    /* 循环处理 */
    for (k = 0u; k < 13u; k++)
    {
        SINGLE_UART_DATA = (uint32)(dump_buf[k] >> 8u) & 0xFFu;
        SYS_Delay(DELAY_SYS_50);
        /* 设置外设参数 */
        SINGLE_UART_DATA = (uint32)(dump_buf[k] & 0x00FFu);
        SYS_Delay(DELAY_SYS_50);
    }
}
#endif /* NLS_VERBOSE */
