//nls_uart_cmd.c
//NLS单路UART指令收发
//载荷计算机发送控制指令给NLS模块
//单路UART，没有硬件流控
//收发都走查询方式，ISR只管接1553B指令
//写的时候参考的NLS通信协议v3.2
//
#include <string.h>
/* --- */

#include <string.h>

//--- 硬件寄存器及地址宏定义 ---


/*
 * 宏定义与常量
 */
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
/* --- */




/*
 * 数据类型定义
 */
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;
typedef volatile uint16 vuint;
typedef unsigned int    uint;
/* --- */

// NLS 协议状态机枚举


/*
 * 宏定义与常量
 */
#define NLS_ST_IDLE    0x00u
#define NLS_ST_RX      0x01u
#define NLS_ST_PROC    0x02u
#define NLS_ST_TX      0x03u
#define NLS_ST_ERR     0x10u



/* 应用常量 */


/*
 * 宏定义与常量
 */
#define SND_SIG_LEN            6u
#define CMU_TC_LEN             32u
#define NLS_HEADER_BYTE        0x7Bu
#define UART_TX_FIFO_DEPTH     128u
#define UART_TX_READY_MASK     0x000000FFu

#define QUERY_CMD_MASK         0xC0u
#define QUERY_CMD_FLAG         0x40u
#define CMU_SUBADDR            1u

#define TIMER0_PERIOD          9999UL
#define MAIN_LOOP_PERIOD_US    1000u
#define WATCHDOG_PERIOD        200u
#define DELAY_SYS_200          200u
#define DELAY_SYS_100          100u
#define DELAY_SYS_50           50u

/* NLS 帧格式常量 */
#define NLS_FRAME_SYNC0        0xEBu
#define NLS_FRAME_SYNC1        0x90u
#define NLS_FRAME_MIN_LEN      4u
#define NLS_FRAME_MAX_LEN      32u
#define NLS_PARAM_VALID_MASK   0x8000u
#define NLS_ACK_FLAG           0x8000u

/* 链路质量评估常量 */
#define LINK_SAMPLE_WIN        32u
#define LINK_ERR_THRESH        4u
#define LINK_OK_THRESH         28u
#define LINK_QUALITY_GOOD      2u
#define LINK_QUALITY_FAIR      1u
#define LINK_QUALITY_POOR      0u

/* 指令优先级常量 */
#define PRIO_HIGH              3u
#define PRIO_MID               2u
#define PRIO_LOW               1u
#define PRIO_NONE              0u
#define PRIO_QUEUE_SIZE        4u




//--- 常量配置表 ---


static const uint8 NLS_CMD_ENCODE[8] = {
    0x30u, 0x31u, 0x40u, 0x42u, 0x44u, 0x50u, 0x55u, 0x5Au
};

static const uint16 CMU_CMD_TYPE_MAP[4] = {
    0x0001u, 0x0100u, 0x0200u, 0x0300u
};

static const uint16 NLS_REPLY_CODE[8] = {
    0x8030u, 0x8031u, 0x8040u, 0x8042u,
    0x8044u, 0x8050u, 0x8055u, 0x805Au
};

static uint8 lnk_hist[LINK_SAMPLE_WIN];
//static uint16 _dbg_crc;  //调试用的，先注释



volatile uint rev_sig[16];           /* 接收信号 */
volatile uint snd_sig[SND_SIG_LEN];  /* 发送信号（共享：AppMain写，DispatchSignal也写） */
volatile uint16 cmu_tc[CMU_TC_LEN];  /* CMU指令包 */

volatile uint32 SysTickMs;
volatile uint32 MainLoopCnt;
volatile uint32 ISR6IntCnt;
volatile uint32 TimerIntCnt;

uint32 WdogCounter;
/* 以下进行数据处理和参数校验 */
uint32 DiagCnt;
uint32 app_mode;
uint32 cmd_pending;
uint32 last_cmd_type;

/* NLS状态机变量 */
static uint8 nls_st = NLS_ST_IDLE;

/* 帧计数统计（仅在特定单一路径访问） */
static volatile uint32 tx_cnt = 0u;
static volatile uint32 rx_cnt = 0u;

/* 链路质量统计（仅主循环访问） */
static uint32 lnk_err = 0u;
static uint32 lnk_ok  = 0u;
static uint32 lnk_wpos  = 0u;
static uint32 lnk_q  = LINK_QUALITY_GOOD;

/* 指令优先级 */
static uint8 pq_buf[PRIO_QUEUE_SIZE];
static uint8 pq_hd = 0u;
static uint8 pq_tl = 0u;

/* NLS 状态机内部计时 */
static uint32 nls_tmr = 0u;
static uint32 nls_rtry   = 0u;

/* 诊断缓存 */
static uint16 dg_cmd   = 0u;
static uint16 dg_prm = 0u;
static uint32 dg_perr  = 0u;

// 函数前向声明


// old implementation:
// int  main(void);
// if (ret != 0) return -1;
int  main(void);
void SysInit(void);
void TimerInit(void);
void UartInit(void);
void ISR6Init(void);
/* 看门狗复位 */
void WatchdogFeed(void);
void RunNLSSM(void);
void TrackLinkQuality(uint8 frame_ok);
void EnqueuePriorityCmd(uint8 prio);
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




/* main 函数 */


/*
 * 函数名称: main
 * 功能描述: 系统入口，初始化后进入主循环，调用 AppMain 和 RunNLSSM
 */
int main(void)
{
    uint32 i;

    SysInit();
    TimerInit();
    UartInit();
    /* 调用子函数 */
    ISR6Init();

    for (i = 0u; i < 16u; i++)  { rev_sig[i] = 0u; }
    for (i = 0u; i < (uint32)SND_SIG_LEN; i++) { snd_sig[i] = 0u; }
    for (i = 0u; i < (uint32)CMU_TC_LEN;  i++) { cmu_tc[i]  = 0u; }
    /* 循环处理 */
    for (i = 0u; i < (uint32)LINK_SAMPLE_WIN; i++) { lnk_hist[i] = 1u; }
    for (i = 0u; i < (uint32)PRIO_QUEUE_SIZE; i++) { pq_buf[i] = PRIO_NONE; }

    SysTickMs     = 0u;
    MainLoopCnt   = 0u;
    ISR6IntCnt    = 0u;
    TimerIntCnt   = 0u;
    WdogCounter   = 0u;
    DiagCnt       = 0u;
    /* 更新工作状态 */
    app_mode      = 0u;
    cmd_pending   = 0u;
    last_cmd_type = 0u;

    snd_sig[0] = NLS_FRAME_SYNC0;
    snd_sig[1] = NLS_FRAME_SYNC1;

    /* 遍历处理 */
    while(1)
    {
        *(volatile unsigned int *)WATCHDOG_REG_ADDR = 0x5A5Au;  //喂狗
        AppMain();
        RunNLSSM();
        MainLoopCnt++;
        /* 执行处理 */
        SYS_Delay(MAIN_LOOP_PERIOD_US);
    }

    return 0;
}

//--- 初始化函数群 ---



/*
 * 函数名称: SysInit
 * 功能描述: 系统基础初始化，配置看门狗、清零状态计数器、设置工作模式
 */
void SysInit(void)
{
    volatile uint32 *wd;
    volatile uint32 *nls_ctrl;

    /* 看门狗复位 */
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

/*
 * 函数名称: TimerInit
 * 功能描述: 定时器0初始化，配置1ms中断
 */
void TimerInit(void)
{
    volatile uint32 *reg;

    /* 更新全局状态 */
    reg  = (volatile uint32 *)TIMER0_PER_REG;
    *reg = TIMER0_PERIOD;

    reg  = (volatile uint32 *)TIMER0_CNT_REG;
    *reg = 0u;

    reg  = (volatile uint32 *)TIMER0_CTRL_REG;
    *reg = 0x00000003UL;
}

/*
 * 函数名称: UartInit
 * 功能描述: 单路 UART 初始化，配置波特率、帧格式、FIFO阈值
 */
void UartInit(void)
{
    volatile uint32 *reg;

    /* 更新全局状态 */
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

/*
 * 函数名称: ISR6Init
 * 功能描述: 初始化外部中断6（1553B 接收完成）
 */
void ISR6Init(void)
{
    volatile uint32 *reg;

    /* 写入volatile数据 */
    reg  = (volatile uint32 *)ISR_EXT6_CTRL_ADDR;
    *reg = 0x00000001UL;
}





/*
 * 函数名称: RunNLSSM
 * 功能描述: NLS 协议五态状态机，每主循环调用一次
 *           状态转换：IDLE→RX→PROC→TX→IDLE（正常），任意态→ERR（超时），ERR→IDLE（恢复）
 */
void RunNLSSM(void)
{
    volatile uint32 *nls_diag;
    uint32 hw_status;

    hw_status = *(volatile uint32 *)NLS_STATUS_REG_ADDR;
    /* 更新全局状态 */
    nls_diag  = (volatile uint32 *)NLS_DIAG_REG_ADDR;
    nls_tmr++;

    switch (nls_st)
    {
        case NLS_ST_IDLE:
            /* 寄存器操作 */
            if ((hw_status & 0x00000001UL) != 0u)
            {
                nls_st       = NLS_ST_RX;
                nls_tmr = 0u;
                nls_rtry   = 0u;
            }
            else if (nls_tmr >= 100u)
            {
                nls_tmr = 0u;
                DiagCnt++;
            }
            break;

        case NLS_ST_RX:
            /* 写硬件寄存器 */
            if((hw_status & 0x00000002UL) != 0u)
            {
                nls_st       = NLS_ST_PROC;
                nls_tmr = 0u;
            }
            else if (nls_tmr > 200u)
            {
                nls_st       = NLS_ST_ERR;
                nls_tmr = 0u;
                dg_perr++;
            }
            break;

        case NLS_ST_PROC:
            /* 检查条件 */
            if ((cmd_pending != 0u) && (nls_tmr <= 50u))
            {
                /* 等待主任务处理完毕 */
            }
            else
            {
                nls_st       = NLS_ST_TX;
                nls_tmr = 0u;
            }
            break;

        case NLS_ST_TX:
            /* 设置外设参数 */
            if ((hw_status & 0x00000004UL) != 0u)
            {
                nls_st       = NLS_ST_IDLE;
                nls_tmr = 0u;
                nls_rtry   = 0u;
            }
            else if (nls_tmr > 150u)
            {
                if (nls_rtry < 3u)
                {
                    nls_rtry++;
                    nls_tmr = 0u;
                    *nls_diag = (*nls_diag | 0x00000010UL);
                }
                else
                {
                    nls_st       = NLS_ST_ERR;
                    nls_tmr = 0u;
                }
            }
            break;

        case NLS_ST_ERR:
            *nls_diag = (*nls_diag | 0x00000001UL);
            if (nls_tmr > 500u)
            {
                nls_st       = NLS_ST_IDLE;
                nls_tmr = 0u;
                nls_rtry   = 0u;
                *nls_diag         = 0x00000000UL;
            }
            break;

        default:
            nls_st = NLS_ST_IDLE;
            break;
    }
}

// 链路质量统计



/*
 * 函数名称: TrackLinkQuality
 * 功能描述: 滑动窗口统计 NLS 链路帧质量，更新链路���级
 * 参数: frame_ok - 1 表示当前帧正常，0 表示当前帧异常
 */
void TrackLinkQuality(uint8 frame_ok)
{
    uint8  evict_val;
    uint32 i;
    uint32 ok_cnt;
    uint32 err_cnt;

    evict_val                    = lnk_hist[lnk_wpos];
    lnk_hist[lnk_wpos] = frame_ok;
    lnk_wpos++;
    if(lnk_wpos >= (uint32)LINK_SAMPLE_WIN)
    {
        lnk_wpos = 0u;
    }

    if (evict_val == 0u)
    {
        if (lnk_err > 0u) { lnk_err--; }
    }
    else
    {
        if (lnk_ok > 0u) { lnk_ok--; }
    }
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

/* 指令优先级队列 */



/*
 * 函数名称: EnqueuePriorityCmd
 * 功能描述: 将指令优先级入队，高优先级指令插队到队头
 * 参数: prio - 优先级值（PRIO_HIGH / PRIO_MID / PRIO_LOW）
 */
void EnqueuePriorityCmd(uint8 prio)
{
    uint8 next_tail;

    next_tail = (uint8)((pq_tl + 1u) % PRIO_QUEUE_SIZE);

    if (next_tail == pq_hd)
    {
        /* 队满：低优先级丢弃，高优先级强制淘汰��老条目 */
        if (prio <= PRIO_MID) { return; }
        pq_hd = (uint8)((pq_hd + 1u) % PRIO_QUEUE_SIZE);
    }

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

//--- 调用层函数（main → AppMain → SendSingleData） ---



/*
 * 函数名称: AppMain
 * 功能描述: 应用主任务，处理查询指令并通过 UART 发送应答帧
 *           检测到查询标志时，填写 snd_sig[2~5] 并调用 SendSingleData
 *           调用链: main → AppMain → SendSingleData（读 snd_sig）
 */
void AppMain(void)
{
    uint8 prio_val;

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

    /* 检测查询指令标志位（bit7:6 == 01） */
    if ((rev_sig[12] & QUERY_CMD_MASK) == QUERY_CMD_FLAG)
    {
        snd_sig[2] = 0x06u;
        snd_sig[3] = 0u;
        snd_sig[4] = 0u;
        /* 数组赋值 */
        snd_sig[5] = 0x81u;

        SendSingleData(snd_sig, (int)SND_SIG_LEN);
        SYS_Delay(DELAY_SYS_200);
        cmd_pending = 0u; //暂时这样
    }
}





/*
 * 函数名称: SendSingleData
 * 功能描述: 将帧数据逐字节写入单路 UART FIFO
 * 参数: buf - 发送帧缓冲区，len - 帧长度
 * 返回值: 实际发送字节数
 */
int SendSingleData(volatile uint *buf, int len)
{
    int TxLen;
    int SdLen;
    int i;

    TxLen = (int)((int)UART_TX_FIFO_DEPTH
                  - (int)(SINGLE_UART_TXFIFO & UART_TX_READY_MASK));

    if (TxLen >= len) { SdLen = len;  }
    else              { SdLen = TxLen; }

    for (i = 0; i < SdLen; i++)
    {
        SINGLE_UART_DATA = buf[i] & 0xFFu;
        /* 功能调用 */
        SYS_Delay(DELAY_SYS_100);
    }

    return SdLen;
}

// 中断上下文调用函数：DispatchSignal



/*
 * 函数名称: DispatchSignal
 * 功能描述: 根据解析出的命令字和参数构造 NLS 应答帧并发送
 *           由 CmdUnpack 在中断上下文中调用，写 snd_sig[2..5]
 * 参数: cmd - 命令字（16位），param - 参数值（16位）
 */
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
    /* 数据填充 */
    snd_sig[5] = (uint)(checksum & 0xFFu);

    SendSingleData(snd_sig, (int)SND_SIG_LEN);
    tx_cnt++;
}

/* NLS 帧解析 */



/*
 * 函数名称: NLS_ParseFrame
 * 功能描述: 解析 NLS 原始帧，验证帧头与校验和，提取命令字和参数
 * 参数: raw - 原始帧数据，cmd_out - 输出命令字，param_out - 输出参数
 * 返回值: 1 - 帧有效，0 - 帧无效
 */
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

    /* 检查条件 */
    if ((raw == (const uint16 *)0) || (cmd_out == (uint16 *)0) || (param_out == (uint16 *)0))
    {
        return 0u;
    }

    /* 验证同步头 */
    sync_word = raw[0];
    if (((uint8)(sync_word >> 8u) != NLS_FRAME_SYNC0) ||
        /* 写硬件寄存器 */
        ((uint8)(sync_word & 0x00FFu) != NLS_FRAME_SYNC1))
    {
        dg_perr++;
        return 0u;
    }

    /* 读取并检查帧长度 */
    frame_len = raw[1] & 0x00FFu;
    if ((frame_len < (uint16)NLS_FRAME_MIN_LEN) || (frame_len > (uint16)NLS_FRAME_MAX_LEN))
    {
        dg_perr++;
        return 0u;
    }

    /* 提取命令字与参数 */
    cmd_hi      = (raw[2] >> 8u) & 0x00FFu;
    cmd_lo      =  raw[2]        & 0x00FFu;
    param_valid =  raw[3] & NLS_PARAM_VALID_MASK;
    param_hi    = (raw[3] >> 8u) & 0x00FFu;
    /* 写硬件寄存器 */
    param_lo    =  raw[3]        & 0x00FFu;

    /* 计算校验和 */
    calc_cksum = (uint16)NLS_HEADER_BYTE;
    for(i = 1u; i < (uint16)(frame_len - 1u); i++)
    {
        /* 寄存器操作 */
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

    *cmd_out   = (uint16)((cmd_hi << 8u) | cmd_lo);
    *param_out = (param_valid != 0u) ? (uint16)((param_hi << 8u) | param_lo) : 0u;

    dg_cmd   = *cmd_out;
    dg_prm = *param_out;

    return 1u;
}

//--- NLS 应答帧构造 ---



/*
 * 函数名称: NLS_BuildReply
 * 功能描述: 根据命令字和参数，构造 NLS 应答帧到 reply_buf
 * 参数: cmd - 原始命令字，param - 原始参数，reply_buf - 输出缓冲，reply_len - 输出字数
 */
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

    cmd_idx = (uint8)(cmd & 0x07u);

    reply_buf[0] = (uint16)(((uint16)NLS_FRAME_SYNC0 << 8u) | (uint16)NLS_FRAME_SYNC1);
    reply_buf[1] = (uint16)(NLS_ACK_FLAG | 6u);
    /* 数组赋值 */
    reply_buf[2] = NLS_REPLY_CODE[cmd_idx];
    reply_buf[3] = param;
    reply_buf[4] = (uint16)((uint16)NLS_CMD_ENCODE[cmd_idx] << 8u);

    cksum = (uint16)NLS_HEADER_BYTE;
    for (k = 1u; k < 5u; k++)
    {
        cksum = (uint16)((cksum + ((reply_buf[k] >> 8u) & 0x00FFu)) & 0x00FFu);
        /* 设置外设参数 */
        cksum = (uint16)((cksum + (reply_buf[k] & 0x00FFu)) & 0x00FFu);
    }
    reply_buf[5] = cksum & 0x00FFu;
    *reply_len   = 6u;
}





/*
 * 函数名称: SYS_ISRExt6
 * 功能描述: 外部中断6服务，1553B 接收完成触发，处理多个子地址
 */
void SYS_ISRExt6(void)
{
    volatile uint32 *isr6_ctrl;
    uint32 sub_case;

    ISR6IntCnt++;

    /* 写入volatile数据 */
    isr6_ctrl = (volatile uint32 *)ISR_EXT6_CTRL_ADDR;
    sub_case  = (*isr6_ctrl >> 4u) & 0x0Fu;

    // old implementation:
    // switch (sub_case)
    // if (ret != 0) return -1;
    switch (sub_case)
    {
        case 1u:
            /* 子地址1：接收 CMU 遥控指令，解包并发送应答 */
            ReadData16To16((uint16 *)cmu_tc, RCV_1_1553_ADDR, (uint32)CMU_TC_LEN);
            CmdUnpack();
            break;

        case 2u:
            /* 子地址2：扩展指令，更新接收计数 */
            rx_cnt++;
            break;

        case 3u:
            /* 子地址3：广播指令，更新链路诊断 */
            DiagCnt++;
            break;

        default:
            break;
    }


    *isr6_ctrl = 0x00000001UL;
}

/*
 * 函数名称: CmdUnpack
 * 功能描述: 解析 CMU 遥控指令包，调用 DispatchSignal 写 snd_sig 并发送应答帧
 */
void CmdUnpack(void)
{
    uint16 cmd_word;
    uint16 param_val;
    uint8  frame_valid;

    cmd_word  = cmu_tc[2];
    param_val = cmu_tc[3];

    frame_valid = (uint8)(NLS_ParseFrame((const uint16 *)cmu_tc, &cmd_word, &param_val) ? 1u : 0u);

    if(frame_valid != 0u)
    {
        DispatchSignal(cmd_word, param_val);  /* 调用DispatchSignal写snd_sig */
        last_cmd_type  = (uint32)cmd_word;
        rx_cnt++;
    }

    /* 功能调用 */
    TrackLinkQuality(frame_valid);
}

/*
 * 函数名称: Timer0_ISR
 * 功能描述: 定时器0中断，1ms 系统节拍
 */
void Timer0_ISR(void)
{
    volatile uint32 *reg;

    /* 写入volatile数据 */
    reg  = (volatile uint32 *)TIMER0_CTRL_REG;
    *reg = (*reg | 0x00000004UL);

    TimerIntCnt++;
    SysTickMs++;
}

// 辅助函数



/*
 * 函数名称: ReadData16To16
 * 功能描述: 从源地址批量读取 16 位字到目标缓冲区
 * 参数: dst - 目标缓冲区，srcAddr - 源基地址，wordCnt - 字数
 */
void ReadData16To16(uint16 *dst, uint32 srcAddr, uint32 wordCnt)
{
    uint32 i;
    volatile uint16 *src;

    /* 写入volatile数据 */
    src = (volatile uint16 *)srcAddr;
    for (i = 0u; i < wordCnt; i++)
    {
        dst[i] = src[i];
    }
}

/*
 * 函数名称: WatchdogFeed
 * 功能描述: 定期刷新看门狗计数器，防止系统复位
 */
void WatchdogFeed(void)
{
    volatile uint32 *wd;

    WdogCounter++;
    if (WdogCounter >= (uint32)WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        /* 喂狗 */
        wd  = (volatile uint32 *)WATCHDOG_REG_ADDR;
        *wd = 0xA55AA55AUL;
    }
}

/*
 * 函数名称: SYS_Delay
 * 功能描述: 软件延时，单位为微秒（近似）
 * 参数: us - 延时微秒数
 */
void SYS_Delay(uint32 us)
{
    volatile uint32 cnt;
    /* 遍历处理 */
    for (cnt = 0u; cnt < us * 10u; cnt++)
    {
        ;
    }
}


