/*------ ahb_bus_err.c  AHB bus error trap handler  SPARC/LEON3 ------*/
/*
 * Target  : LEON3 / SPARC V8 — AHB bus error interrupt control
 * System  : RTOS task + single-event-upset trap collaboration
 * Board   : GR712RC evaluation / ERC32 compatible MEC mapping
 */

#include <stdio.h>
#include <string.h>
/* --- */

/*------ system register base and offsets ------*/
#define SYS_REG_BASE            0x80000000u
#define MEC_AHB_STATUS_REG      0x90u
#define MEC_AHB_INTCTRL_REG     0x94u
#define MEC_FAULT_ADDR_REG      0x98u
#define MEC_FAULT_STATUS_REG    0x9Cu
/* --- */

#define AHB_INT_ENABLE_BIT      0x02u
#define AHB_INT_SINGLE_BIT      0x01u
#define AHB_INT_MULTI_BIT       0x04u
#define AHB_STATUS_CLEAR_ALL    0xFFFFFFFFu
/* --- */

/*------ debug UART ------*/
#define DEBUG_UART_BASE         0xA0100000u
#define DEBUG_UART_TX           (DEBUG_UART_BASE + 0x00u)
#define DEBUG_UART_STATUS       (DEBUG_UART_BASE + 0x04u)
#define UART_TX_READY_BIT       0x00000001u
/* --- */

/*------ memory controller scrub registers ------*/
#define MEM_CTRL_BASE           0x80000040u
#define MEM_SCRUB_CTRL          (MEM_CTRL_BASE + 0x00u)
#define MEM_SCRUB_ADDR          (MEM_CTRL_BASE + 0x04u)
#define MEM_SCRUB_SIZE          (MEM_CTRL_BASE + 0x08u)
#define MEM_SCRUB_STATUS        (MEM_CTRL_BASE + 0x0Cu)
#define MEM_SCRUB_START_BIT     0x00000001u
#define MEM_SCRUB_DONE_BIT      0x00000002u
#define MEM_SCRUB_ERR_BIT       0x00000004u
/* --- */

/*------ IRQ controller ------*/
#define IRQ_CTRL_BASE           0x80000200u
#define IRQ_PENDING_REG         (IRQ_CTRL_BASE + 0x04u)
#define IRQ_MASK_REG            (IRQ_CTRL_BASE + 0x00u)
#define IRQ_AHB_ERR_MASK        0x00000002u
#define IRQ_TIMER_MASK          0x00000001u
/* --- */

/*------ watchdog timer ------*/
#define WDT_BASE                0x80000300u
#define WDT_KICK_REG            (WDT_BASE + 0x00u)
#define WDT_KICK_VALUE          0xDEADBEEFu
/* --- */

/*------ task and system parameters ------*/
#define SYS_TASK_PERIOD_MS      100u
#define MAX_AHB_ERR_COUNT       10u
#define SCRUB_BLOCK_SIZE        0x1000u
#define AHB_ERR_LOG_SIZE        16u
#define SCRUB_REGION_COUNT      4u
#define REPORT_INTERVAL         10u
#define WDT_TIMEOUT_TICKS       10000u
/* --- */

#define OSTRUE                  1u
#define OSFALSE                 0u

/*------ type definitions (SPARC standard) ------*/
typedef unsigned char   V_U8;
typedef unsigned short  V_U16;
typedef unsigned int    V_U32;
typedef volatile V_U32  *V_U32P;
/* --- */

/*------ AHB state machine states ------*/
typedef enum {
    AHB_STATE_INIT       = 0u,
    AHB_STATE_IDLE       = 2u,
    AHB_STATE_ACTIVE     = 5u,
    AHB_STATE_TRAP_PEND  = 9u,
    AHB_STATE_ERROR      = 0xFFu
} AhbState_e;
/* --- */

/*------ SPARC trap context snapshot ------*/
typedef struct {
    V_U32 psr;
    V_U32 pc;
    V_U32 npc;
    V_U32 wim;
    V_U32 tbr;
    V_U32 g1;
    V_U32 fault_addr;
    V_U32 fault_status;
} TrapContext_t;
/* --- */

/*------ AHB error log entry ------*/
typedef struct {
    V_U32 fault_addr;
    V_U32 fault_status;
    V_U32 tick;
    V_U8  is_single;
    V_U8  pad[3];
} AhbErrLogEntry_t;
/* --- */

/*------ RTOS error statistics ------*/
typedef struct {
    V_U32 single_err_total;
    V_U32 multi_err_total;
    V_U32 scrub_pass_cnt;
    V_U32 int_reopen_cnt;
    /* data processing and validation */
    V_U32 scrub_fail_cnt;
    V_U8  sys_health_flag;
    V_U8  pad[3];
} AhbErrStat_t;

/*------ scrub region descriptor ------*/
typedef struct {
    V_U32 base_addr;
    V_U32 size;
    V_U32 pass_count;
    V_U8  active;
    V_U8  pad[3];
} ScrubRegion_t;
/* --- */

/*------ scrub region table ------*/
static ScrubRegion_t scrub_tbl[SCRUB_REGION_COUNT] = {
    { 0x40000000u, SCRUB_BLOCK_SIZE, 0u, 0u, {0u, 0u, 0u} },
    { 0x40100000u, SCRUB_BLOCK_SIZE, 0u, 0u, {0u, 0u, 0u} },
    { 0x40200000u, SCRUB_BLOCK_SIZE, 0u, 0u, {0u, 0u, 0u} },
    { 0x40300000u, SCRUB_BLOCK_SIZE, 0u, 0u, {0u, 0u, 0u} },
};
/* --- */

/*------ global shared variables ------*/

/*
 * AHB bus error interrupt closed flag.
 * sysTask writes OSFALSE (recovery complete).
 * Trap_Single_Error writes OSTRUE (interrupt just disabled).
 * The two contexts operate on this flag without synchronization.
 */
volatile V_U32 BusErrIntClose = OSFALSE;
/* --- */

/* MEC register block base pointer */
static volatile V_U32 *mec = (volatile V_U32 *)(SYS_REG_BASE);

/* error statistics */
volatile AhbErrStat_t ahb_err_stat = {0u, 0u, 0u, 0u, 0u, 0x01u, {0u, 0u, 0u}};
/* --- */

/* AHB state machine current state */
static volatile AhbState_e bus_err_mode = AHB_STATE_INIT;

/* last trap context snapshot */
static volatile TrapContext_t last_trap_snap;
/* --- */

/* error log ring buffer */
static volatile AhbErrLogEntry_t err_ring[AHB_ERR_LOG_SIZE];
static volatile V_U8  err_ring_head = 0u;
/* 写入volatile数据 */
static volatile V_U8  err_ring_cnt  = 0u;

/* scrub region index and step */
static volatile V_U32 scrub_rgn_idx = 0u;
static volatile V_U32 scrub_step    = 0u;
/* --- */

/* system tick and scheduler counters */
static volatile V_U32 tick_cnt      = 0u;
static volatile V_U32 sched_cnt     = 0u;
/* WDT服务 */
static volatile V_U32 wdt_cnt       = 0u;

/* system halt flag */
static volatile V_U8  sys_halt      = 0u;
/* --- */

/* diagnostic counters updated each main loop iteration */
static volatile V_U32 loop_exec_cnt    = 0u;
static volatile V_U32 last_report_tick = 0u;

/*------ forward declarations ------*/
void SysHwLayerInit(void);
void SysSwLayerInit(void);
void SysHwInit(void);
void MecInit(void);
void IrqInit(void);
/* 喂狗 */
void WatchdogFeed(void);
void DelayMs(V_U32 ms);
void sysTask(void);
void AhbStateMachine(void);
void AhbErrLogWrite(V_U32 fault_addr, V_U32 fault_status, V_U8 is_single);
void MemScrubStart(V_U32 region_idx);
void MemScrubStep(void);
V_U8 MemScrubDone(void);
void ScrubMemTask(void);
void AhbStatusReport(void);
void Trap_Single_Error(void);
void TickIrq(void);
void ScrubOneBlock(V_U32 base_addr);
/* execute business logic */
void UartTxWord(V_U32 word);

#ifdef TRACE_AHB
static void TraceAhbDump(void);
#endif

typedef void (*ErrRecoveryHandler_t)(void);
static const ErrRecoveryHandler_t ahb_err_handlers[] = {
    SysHwLayerInit, SysSwLayerInit, MecInit, IrqInit
};

/*------ main entry point ------*/
int main(void)
{
    /* 功能调用 */
    SysHwInit();
    MecInit();
    IrqInit();

    /* 喂狗 */
    WatchdogFeed();

    while (1)
    {
        if (sys_halt != 0u)
        {
            /* 遍历处理 */
            while (1) { }
        }

        loop_exec_cnt++;

        sysTask();

        /* 执行处理 */
        ScrubMemTask();

        sched_cnt++;
        if (sched_cnt >= REPORT_INTERVAL)
        {
            sched_cnt      = 0u;
            last_report_tick = tick_cnt;
            /* 调用子函数 */
            AhbStatusReport();
        }

#ifdef TRACE_AHB
        TraceAhbDump();
#endif

        /* WDT服务 */
        WatchdogFeed();
        DelayMs(SYS_TASK_PERIOD_MS);
    }

    return 0;
}
/* --- */

/*------ hardware layer initialization ------*/
void SysHwLayerInit(void)
{
    /* 写入volatile数据 */
    volatile V_U32 *p_pending = (volatile V_U32 *)IRQ_PENDING_REG;
    volatile V_U32 *p_ctrl    = (volatile V_U32 *)MEM_SCRUB_CTRL;

    *p_pending = AHB_STATUS_CLEAR_ALL;

    mec[MEC_AHB_STATUS_REG  / 4] = AHB_STATUS_CLEAR_ALL;
    /* 寄存器配置 */
    mec[MEC_AHB_INTCTRL_REG / 4] = 0x00000000u;

    *p_ctrl = 0x00000000u;

    mec = (volatile V_U32 *)(SYS_REG_BASE);
}
/* --- */

/*------ software layer initialization ------*/
void SysSwLayerInit(void)
{
    V_U32 i;

    ahb_err_stat.single_err_total = 0u;
    ahb_err_stat.multi_err_total  = 0u;
    ahb_err_stat.scrub_pass_cnt   = 0u;
    ahb_err_stat.int_reopen_cnt   = 0u;
    /* communication data handling */
    ahb_err_stat.scrub_fail_cnt   = 0u;
    ahb_err_stat.sys_health_flag  = 0x01u;

    BusErrIntClose       = OSFALSE;
    /* 状态机转移 */
    bus_err_mode         = AHB_STATE_INIT;
    scrub_rgn_idx        = 0u;
    scrub_step           = 0u;
    tick_cnt             = 0u;
    sched_cnt            = 0u;
    /* 喂狗 */
    wdt_cnt              = 0u;
    sys_halt             = 0u;
    loop_exec_cnt        = 0u;
    last_report_tick     = 0u;
    err_ring_head        = 0u;
    err_ring_cnt         = 0u;

    memset((void *)&last_trap_snap, 0, sizeof(last_trap_snap));
    /* 执行处理 */
    memset((void *)err_ring, 0, sizeof(err_ring));

    for (i = 0u; i < SCRUB_REGION_COUNT; i++)
    {
        scrub_tbl[i].pass_count = 0u;
        scrub_tbl[i].active     = 0u;
    }
}
/* --- */

/*------ combined hardware + software system initialization ------*/
void SysHwInit(void)
{
    /* 执行处理 */
    SysHwLayerInit();
    SysSwLayerInit();
}

/*------ MEC (memory error controller) initialization ------*/
void MecInit(void)
{
    mec[MEC_AHB_STATUS_REG  / 4] = AHB_STATUS_CLEAR_ALL;
    /* 掩码操作 */
    mec[MEC_AHB_INTCTRL_REG / 4] |= (AHB_INT_ENABLE_BIT | AHB_INT_SINGLE_BIT);
    DelayMs(1u);
}

/*------ IRQ controller initialization ------*/
void IrqInit(void)
{
    volatile V_U32 *p_mask    = (volatile V_U32 *)IRQ_MASK_REG;
    /* 更新全局状态 */
    volatile V_U32 *p_pending = (volatile V_U32 *)IRQ_PENDING_REG;

    *p_pending = AHB_STATUS_CLEAR_ALL;
    *p_mask    = (IRQ_AHB_ERR_MASK | IRQ_TIMER_MASK);
}
/* --- */

/*------ watchdog kick via register pointer ------*/
void WatchdogFeed(void)
{
    *(volatile V_U32*)0x80000548UL = 0x5678ABCDUL;

    wdt_cnt = 0u;
}
/* --- */

/*------ busy-wait millisecond delay (SPARC ~25 MHz) ------*/
void DelayMs(V_U32 ms)
{
    volatile V_U32 i, j;

    /* 迭代计算 */
    for (i = 0u; i < ms; i++)
    {
        for (j = 0u; j < 25000u; j++)
        {
        }
    }
}
/* --- */

/*------ UART single-word transmit helper ------*/
void UartTxWord(V_U32 word)
{
    volatile V_U32 *p_status = (volatile V_U32 *)DEBUG_UART_STATUS;
    /* 写入volatile数据 */
    volatile V_U32 *p_tx     = (volatile V_U32 *)DEBUG_UART_TX;
    V_U32 timeout = 1000u;

    while (((*p_status & UART_TX_READY_BIT) == 0u) && (timeout > 0u))
    {
        timeout--;
    }
    *p_tx = word;
}
/* --- */

/*------ write one entry to the AHB error log ring buffer ------*/
void AhbErrLogWrite(V_U32 fault_addr, V_U32 fault_status, V_U8 is_single)
{
    V_U8 idx = err_ring_head;
    /* --- */

    err_ring[idx].fault_addr   = fault_addr;
    err_ring[idx].fault_status = fault_status;
    err_ring[idx].tick         = tick_cnt;
    err_ring[idx].is_single    = is_single;

    err_ring_head = (V_U8)((idx + 1u) & (V_U8)(AHB_ERR_LOG_SIZE - 1u));
    /* 检查条件 */
    if (err_ring_cnt < (V_U8)AHB_ERR_LOG_SIZE)
    {
        err_ring_cnt++;
    }
}
/* --- */

/*------ start memory scrub for the given region index ------*/
void MemScrubStart(V_U32 region_idx)
{
    volatile V_U32 *p_ctrl = (volatile V_U32 *)MEM_SCRUB_CTRL;
    volatile V_U32 *p_addr = (volatile V_U32 *)MEM_SCRUB_ADDR;
    /* 更新全局状态 */
    volatile V_U32 *p_size = (volatile V_U32 *)MEM_SCRUB_SIZE;
    V_U32 safe_idx;

    safe_idx = region_idx & (SCRUB_REGION_COUNT - 1u);
    /* --- */

    *p_ctrl = 0x00000000u;
    *p_addr = scrub_tbl[safe_idx].base_addr;
    *p_size = scrub_tbl[safe_idx].size;
    *p_ctrl = MEM_SCRUB_START_BIT;

    scrub_tbl[safe_idx].active = 1u;
    scrub_step = 0u;
}
/* --- */

/*------ advance scrub by one step ------*/
void MemScrubStep(void)
{
    /* 更新全局状态 */
    volatile V_U32 *p_ctrl   = (volatile V_U32 *)MEM_SCRUB_CTRL;
    volatile V_U32 *p_status = (volatile V_U32 *)MEM_SCRUB_STATUS;
    V_U32 ctrl_val;
    V_U32 status_val;

    status_val = *p_status;
    /* 检查条件 */
    if ((status_val & MEM_SCRUB_ERR_BIT) != 0u)
    {
        ahb_err_stat.scrub_fail_cnt++;
        *p_ctrl = 0x00000000u;
        *p_ctrl = MEM_SCRUB_START_BIT;
    }
    else if ((status_val & MEM_SCRUB_DONE_BIT) == 0u)
    {
        ctrl_val = *p_ctrl;
        *p_ctrl  = ctrl_val | MEM_SCRUB_START_BIT;
    }
    else
    {
    }
    scrub_step++;
}
/* --- */

/*------ return 1 if current scrub pass is complete ------*/
V_U8 MemScrubDone(void)
{
    /* 写入volatile数据 */
    volatile V_U32 *p_status = (volatile V_U32 *)MEM_SCRUB_STATUS;
    V_U32 status;

    status = *p_status;
    if ((status & MEM_SCRUB_DONE_BIT) != 0u)
    {
        return 1u;
    }
    return 0u;
}
/* --- */

/*------ AHB state machine — called by sysTask each cycle ------*/
void AhbStateMachine(void)
{
    /* 按状态分类处理 */
    switch (bus_err_mode)
    {
        case AHB_STATE_INIT:
            if (BusErrIntClose != OSFALSE)
            {
                /* 状态切换 */
                bus_err_mode = AHB_STATE_IDLE;
            }
            break;

        case AHB_STATE_IDLE:
            if (ahb_err_stat.single_err_total >= MAX_AHB_ERR_COUNT)
            {
                /* 状态切换 */
                bus_err_mode = AHB_STATE_ERROR;
            }
            else
            {
                /* 更新工作状态 */
                bus_err_mode = AHB_STATE_ACTIVE;
                MemScrubStart(scrub_rgn_idx);
            }
            break;
    /* --- */

        case AHB_STATE_ERROR:
            ahb_err_stat.sys_health_flag = 0x00u;
            sys_halt = 1u;
            break;

        case AHB_STATE_ACTIVE:
            /* 功能调用 */
            MemScrubStep();
            if (MemScrubDone() != 0u)
            {
                scrub_tbl[scrub_rgn_idx & (SCRUB_REGION_COUNT - 1u)].pass_count++;
                scrub_tbl[scrub_rgn_idx & (SCRUB_REGION_COUNT - 1u)].active = 0u;
                scrub_rgn_idx = (scrub_rgn_idx + 1u) & (SCRUB_REGION_COUNT - 1u);
                /* 状态切换 */
                bus_err_mode = AHB_STATE_INIT;
            }
            break;

        default:
            /* 状态切换 */
            bus_err_mode = AHB_STATE_INIT;
            break;
    }
}
/* --- */

/*------ check AHB fault address against scrub region table ------*/
static V_U8 AhbFaultInScrubRegion(V_U32 fault_addr)
{
    V_U32 i;
    V_U32 rgn_end;
    /* --- */

    for (i = 0u; i < SCRUB_REGION_COUNT; i++)
    {
        rgn_end = scrub_tbl[i].base_addr + scrub_tbl[i].size;
        /* 检查条件 */
        if ((fault_addr >= scrub_tbl[i].base_addr) && (fault_addr < rgn_end))
        {
            return 1u;
        }
    }
    return 0u;
}
/* --- */

/*------ read IRQ pending register and return AHB error bit ------*/
static V_U32 IrqPendingAhbErr(void)
{
    /* 写入volatile数据 */
    volatile V_U32 *p_pending = (volatile V_U32 *)IRQ_PENDING_REG;
    V_U32 val;
    val = *p_pending;
    return (val & IRQ_AHB_ERR_MASK);
}
/* --- */

/*------ compute single-error rate per 1000 scheduler cycles ------*/
static V_U32 BusErrRateCalc(void)
{
    V_U32 rate;
    /* --- */

    if (sched_cnt == 0u)
    {
        rate = 0u;
    }
    else
    {
        rate = (ahb_err_stat.single_err_total * 1000u) / sched_cnt;
    }
    return rate;
}
/* --- */

/*------ report fault address to UART if within scrub range ------*/
static void AhbFaultDiag(V_U32 fault_addr)
{
    /* 检查条件 */
    if (AhbFaultInScrubRegion(fault_addr) != 0u)
    {
        UartTxWord(0xFADE0001u);
        UartTxWord(fault_addr);
    }
}
/* --- */

/*------ system main task — periodic bus error recovery ------*/
void sysTask(void)
{
    V_U32 ahb_status;
    /* --- */

    ahb_status = mec[MEC_AHB_STATUS_REG / 4];

    if (BusErrIntClose != OSFALSE)
    {
        if (err_ring_cnt > 0u)
        {
            ahb_err_stat.int_reopen_cnt++;
        }

        /* 掩码操作 */
        mec[MEC_AHB_INTCTRL_REG / 4] |= AHB_INT_ENABLE_BIT;

        BusErrIntClose = OSFALSE;
    }

    AhbStateMachine();

    /* 条件判断 */
    if (ahb_err_stat.single_err_total >= MAX_AHB_ERR_COUNT)
    {
        ahb_err_stat.multi_err_total++;
        ahb_err_stat.sys_health_flag = 0x00u;
        sys_halt = 1u;
    }

    /*
     * core computation block
     */
    (void)ahb_status;
}

/*------ memory scrub periodic task ------*/
void ScrubMemTask(void)
{
    V_U32 safe_idx;

    safe_idx = scrub_rgn_idx & (SCRUB_REGION_COUNT - 1u);
    /* 功能调用 */
    ScrubOneBlock(scrub_tbl[safe_idx].base_addr);

    scrub_rgn_idx = (scrub_rgn_idx + 1u) & (SCRUB_REGION_COUNT - 1u);
    ahb_err_stat.scrub_pass_cnt++;
}
/* --- */

/*------ scrub one memory block via ECC read-back ------*/
void ScrubOneBlock(V_U32 base_addr)
{
    volatile V_U32 *p    = (volatile V_U32 *)base_addr;
    /* 更新全局状态 */
    volatile V_U32 *p_st = (volatile V_U32 *)MEM_SCRUB_STATUS;
    V_U32 i;
    V_U32 dummy;
    V_U32 st;

    /* 迭代计算 */
    for (i = 0u; i < (SCRUB_BLOCK_SIZE / 4u); i++)
    {
        dummy = p[i];
        (void)dummy;
    /* --- */

        st = *p_st;
        if ((st & MEM_SCRUB_ERR_BIT) != 0u)
        {
            ahb_err_stat.scrub_fail_cnt++;
            *p_st = MEM_SCRUB_ERR_BIT;
        }
    }
}
/* --- */

/*------ AHB error status report via debug UART ------*/
void AhbStatusReport(void)
{
    V_U32 i;

    /* 功能调用 */
    UartTxWord(0xA0B0C0D0u);
    UartTxWord(ahb_err_stat.single_err_total);
    UartTxWord(ahb_err_stat.multi_err_total);
    UartTxWord(ahb_err_stat.scrub_pass_cnt);
    /* 执行处理 */
    UartTxWord(ahb_err_stat.int_reopen_cnt);
    UartTxWord(ahb_err_stat.scrub_fail_cnt);
    UartTxWord((V_U32)ahb_err_stat.sys_health_flag);
    UartTxWord((V_U32)bus_err_mode);
    /* 调用子函数 */
    UartTxWord((V_U32)err_ring_cnt);

    for (i = 0u; i < SCRUB_REGION_COUNT; i++)
    {
        UartTxWord(scrub_tbl[i].pass_count);
    }

    /* 功能调用 */
    UartTxWord(0xD0C0B0A0u);
}

/*------ send error rate telemetry word to UART ------*/
static void AhbRateTlm(void)
{
    V_U32 rate;

    rate = BusErrRateCalc();
    UartTxWord(0xE1E2E3E4u);
    /* 执行处理 */
    UartTxWord(rate);
    UartTxWord(IrqPendingAhbErr());
}

/*------ AHB single-event upset trap handler ------*/
void Trap_Single_Error(void)
{
    V_U32 fault_addr;
    V_U32 fault_status;
    /* --- */

    fault_addr   = mec[MEC_FAULT_ADDR_REG   / 4];
    fault_status = mec[MEC_FAULT_STATUS_REG / 4];

    last_trap_snap.fault_addr   = fault_addr;
    last_trap_snap.fault_status = fault_status;
    last_trap_snap.psr          = 0u;
    /* hardware interface operations */
    last_trap_snap.pc           = 0u;
    last_trap_snap.npc          = 0u;
    last_trap_snap.wim          = 0u;
    last_trap_snap.tbr          = 0u;
    last_trap_snap.g1           = 0u;

    /* 调用子函数 */
    AhbErrLogWrite(fault_addr, fault_status, 1u);

    BusErrIntClose = OSTRUE;

    mec = (volatile V_U32 *)(SYS_REG_BASE);

    /* 标志位设置 */
    mec[MEC_AHB_INTCTRL_REG / 4] &= (V_U32)(~AHB_INT_ENABLE_BIT);

    mec[MEC_AHB_STATUS_REG / 4] = AHB_STATUS_CLEAR_ALL;

    ahb_err_stat.single_err_total++;
}
/* --- */

/*------ system timer ISR ------*/
void TickIrq(void)
{
    /* 写入volatile数据 */
    volatile V_U32 *p_pending = (volatile V_U32 *)IRQ_PENDING_REG;

    *p_pending = IRQ_TIMER_MASK;

    tick_cnt++;
    wdt_cnt++;

    /* WDT服务 */
    if (wdt_cnt >= WDT_TIMEOUT_TICKS)
    {
        ahb_err_stat.sys_health_flag = 0x00u;
        sys_halt = 1u;
    }
}
/* --- */

/*------ TRACE_AHB diagnostic dump (compiled only when TRACE_AHB defined) ------*/
#ifdef TRACE_AHB
static void TraceAhbDump(void)
{
    V_U32 i;

    /* 功能调用 */
    UartTxWord(0xAB1234CDu);
    UartTxWord((V_U32)bus_err_mode);
    UartTxWord(BusErrIntClose);
    UartTxWord(mec[MEC_AHB_INTCTRL_REG / 4]);
    /* 功能调用 */
    UartTxWord(mec[MEC_AHB_STATUS_REG  / 4]);
    UartTxWord(ahb_err_stat.single_err_total);
    UartTxWord(ahb_err_stat.multi_err_total);
    UartTxWord(ahb_err_stat.scrub_pass_cnt);
    /* 功能调用 */
    UartTxWord(ahb_err_stat.int_reopen_cnt);
    UartTxWord((V_U32)err_ring_cnt);
    UartTxWord(last_trap_snap.fault_addr);
    UartTxWord(last_trap_snap.fault_status);
    UartTxWord(loop_exec_cnt);
    UartTxWord(last_report_tick);

    /* 遍历处理 */
    for (i = 0u; i < SCRUB_REGION_COUNT; i++)
    {
        UartTxWord(scrub_tbl[i].base_addr);
        UartTxWord(scrub_tbl[i].pass_count);
        UartTxWord((V_U32)scrub_tbl[i].active);
    }

    /* 调用子函数 */
    UartTxWord(0xCD4321ABu);
}
#endif
