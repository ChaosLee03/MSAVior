/*------ vids_clk_adj.c  VIDS clock frequency adjustment  LEON3/SPARC ------*/
/*                                                                           */
/* Module  : VIDS high-precision clock adaptive frequency correction         */
/* Target  : LEON3FT GR712RC  (SPARC V8)                                    */
/* TC sets new average adjustment period, clock IRQ reads it for trimming.   */
/* Includes clock state flags, GPS pulse monitor, drift stats and diag out.  */
/* Date    : 2019-06-14   Rev 3.2                                           */
/*---------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* --- */

/*------ hardware base addresses and register offsets -----------------------*/
#define SYS_CLOCK_BASE_ADDR     0xA0000000UL
#define CLK_CTRL_REG            (SYS_CLOCK_BASE_ADDR + 0x00UL)
#define CLK_STATUS_REG          (SYS_CLOCK_BASE_ADDR + 0x04UL)
#define CLK_PERIOD_REG          (SYS_CLOCK_BASE_ADDR + 0x08UL)
#define CLK_FINE_TRIM_REG       (SYS_CLOCK_BASE_ADDR + 0x0CUL)
#define CLK_COARSE_TRIM_REG     (SYS_CLOCK_BASE_ADDR + 0x10UL)
/* --- */

#define TIMER_BASE_ADDR         0xA0010000UL
#define TIMER0_CTRL             (TIMER_BASE_ADDR + 0x00UL)
#define TIMER0_COUNT            (TIMER_BASE_ADDR + 0x04UL)
#define TIMER0_RELOAD           (TIMER_BASE_ADDR + 0x08UL)
/* --- */

#define UART_BASE_ADDR          0xA0020000UL
#define UART_STATUS             (UART_BASE_ADDR + 0x00UL)
#define UART_DATA               (UART_BASE_ADDR + 0x04UL)
#define UART_CTRL               (UART_BASE_ADDR + 0x08UL)
/* --- */

#define IRQ_BASE_ADDR           0xA0030000UL
#define IRQ_PENDING_REG         (IRQ_BASE_ADDR + 0x00UL)
#define IRQ_MASK_REG            (IRQ_BASE_ADDR + 0x04UL)
#define IRQ_CLEAR_REG           (IRQ_BASE_ADDR + 0x08UL)
#define IRQ_LEVEL_REG           (IRQ_BASE_ADDR + 0x0CUL)
/* --- */

#define GPS_BASE_ADDR           0xA0040000UL
#define GPS_STATUS_REG          (GPS_BASE_ADDR + 0x00UL)
#define GPS_PPS_COUNT_REG       (GPS_BASE_ADDR + 0x04UL)
#define GPS_TIME_HI_REG         (GPS_BASE_ADDR + 0x08UL)
#define GPS_TIME_LO_REG         (GPS_BASE_ADDR + 0x0CUL)
/* --- */

#define WDT_BASE_ADDR           0x80000500UL

#define IRQ_EXT2_CLOCK          2u
#define IRQ_EXT1_UART           1u
#define IRQ_EXT3_GPS            3u
#define IRQ_MASK_EXT2           (1u << IRQ_EXT2_CLOCK)
#define IRQ_MASK_EXT1           (1u << IRQ_EXT1_UART)
#define IRQ_MASK_EXT3           (1u << IRQ_EXT3_GPS)
/* --- */

#define AVG_ADJUST_PERIOD_MIN   0x0064u
#define AVG_ADJUST_PERIOD_MAX   0x4E20u
#define AVG_ADJUST_PERIOD_DEF   0x0BB8u
#define CLOCK_SYNC_THRESHOLD    50u
/* --- */

#define TC_CMD_SET_ADJ_PERIOD   0x0Au
#define TC_CMD_QUERY_STATUS     0x0Bu
#define TC_CMD_RESET_CLOCK      0x0Cu
#define TC_CMD_SET_GPS_MODE     0x0Du
/* --- */

#define CLK_STATE_NORMAL        0x00u
#define CLK_STATE_ADJUSTING     0x01u
#define CLK_STATE_ERROR_VAL     0x02u

#define GPS_PPS_TIMEOUT_MS      1200u
#define GPS_PPS_HISTORY_LEN     8u
#define FREQ_DRIFT_HISTORY_LEN  8u
#define FREQ_DRIFT_ALARM_THR    5u
#define CLK_LOCK_CNT_THR        10u
#define GPS_JUMP_THRESHOLD      5u
/* --- */

// clock state flags (replaces enum)
#define CLKF_IDLE               0x00u
#define CLKF_CALIBRATING        0x01u
#define CLKF_LOCKED             0x02u
#define CLKF_ERROR              0x03u
/* --- */

/*------ LEON3 AMBA bus configuration ---------------------------------------*/
#define AMBA_CONF_AREA          0xFFF00000UL
#define AMBA_AHB_SLAVE_CNT      8u
#define AMBA_APB_SLAVE_CNT      16u
/* --- */

/*------ type definitions ---------------------------------------------------*/
typedef unsigned char  uchar;
typedef unsigned short uint16;
typedef unsigned int   uint32;
/* --- */

/* LEON3 trap table entry (for reference only) */
typedef struct {
    uint32 inst[4];        /* sethi/jmp/nop/nop */
} TrapEntry_t;
/* --- */

/* clock engineering telemetry record */
typedef struct {
    uint16 adj_count;
    uint16 sync_count;
    uchar  clk_state;
    uchar  err_flag;
} ClkEngData_t;
/* --- */

/* GPS receiver status record */
typedef struct {
    uint32 last_pps_tick;
    uint32 time_hi;
    uint32 time_lo;
    uchar  valid;
    uchar  jump_flag;
} GpsStatus_t;
/* --- */

/*------ PLL configuration constants for GR712RC ----------------------------*/
#define PLL_REF_CLK_HZ          50000000UL  /* 50 MHz reference oscillator */
#define PLL_VCO_MIN_HZ          100000000UL
#define PLL_VCO_MAX_HZ          250000000UL
#define PLL_LOCK_TIMEOUT_US     500u
/* --- */

/*------ frequency trim calibration table (factory values) ------------------*/
static const uint16 ClkTrimTable[8] = {
    0x0000u,
    0x0010u,
    0x0020u,
    0x0040u,
    0xFF00u,
    0xFEF0u,
    0xFED0u,
    0xFF80u,
};
/* --- */

/*------ coarse trim ramp-up table (temperature compensation) ---------------*/
static const uint16 CoarseTrimRamp[4] = {
    0x0000u,   /* nominal */
    0x0008u,   /* +5 ppm offset  */
    0xFFF8u,   /* -5 ppm offset  */
    0x0010u,   /* +10 ppm offset */
};
/* --- */

/*------ global variables ---------------------------------------------------*/

/*
 * Average adjustment period (16-bit), split into high/low bytes.
 * Written by TC handler HandelTcBlkAvgAdjust (task context),
 * read by clock ISR Ext2ClockIrq (interrupt context).
 * High byte: VIDS_AVG_ADJ_PERH, low byte: VIDS_AVG_ADJ_PERL
 */
volatile uchar VIDS_AVG_ADJ_PERH = (uchar)(AVG_ADJUST_PERIOD_DEF >> 8);
volatile uchar VIDS_AVG_ADJ_PERL = (uchar)(AVG_ADJUST_PERIOD_DEF & 0xFFu);

/* 更新全局状态 */
volatile ClkEngData_t clk_eng = {0u, 0u, CLK_STATE_NORMAL, 0u};
static volatile uint16 freq_error_acc = 0u;
static volatile uint32 last_adj_tick  = 0u;
static volatile uchar  sys_running    = 0u;
static volatile uint16 main_loop_cnt  = 0u;
static volatile uchar  tc_buf[16] = {0u};
/* 写入volatile数据 */
static volatile uchar  tc_len     = 0u;
static volatile uchar  tc_ready   = 0u;
static volatile uint16 wdt_cnt    = 0u;
static volatile uchar  clk_fsm_flags  = CLKF_IDLE;
static volatile uint32 pll_lock_cnt   = 0u;
static volatile uint32 clk_err_cnt    = 0u;
static volatile uint32 gps_pulse_cnt  = 0u;
/* 写入volatile数据 */
static volatile uint32 gps_lost_cnt   = 0u;
static volatile uchar  gps_valid_flag = 0u;
static volatile GpsStatus_t gps_status = {0u, 0u, 0u, 0u, 0u};
static volatile uint32 drift_alarm_cnt    = 0u;
static volatile uchar  drift_alarm_active = 0u;
/* 更新全局状态 */
static volatile uint16 sched_gps_cnt   = 0u;
static volatile uint16 sched_drift_cnt = 0u;
static volatile uint16 sched_diag_cnt  = 0u;
uint32 spare_reg;  // reserved for debug, do not remove
/* 写入volatile数据 */
static volatile uchar  coarse_trim_idx = 0u;

/*------ forward declarations -----------------------------------------------*/
void SysInit(void);
void ClkModuleInit(void);
void TimerInit(void);
void UartInit(void);
void InterruptInit(void);
/* WDT服务 */
void WatchdogFeed(void);
void DelayUs(uint32 us);
uint32 HandelTcBlkAvgAdjust(uchar chk_handle, uchar *ptr, uint32 len);
void ProcessTcCommand(void);
void ClkStatusReport(void);
void Ext2ClockIrq(void);      /* ext clock IRQ2 ISR */
void UartRxIrq(void);         /* UART receive ISR */
void GpsPpsIrq(void);         /* GPS PPS ISR */
uint16 GetVidData_U16(volatile uchar *hi, volatile uchar *lo);
void SetVidData(volatile uchar *dst, uchar val);
/* data processing and validation */
static void DisableClkIrq(void);
static void EnableClkIrq(void);
static void FreqDriftMonitor(void);
static void CheckGpsPulseLost(void);
static void HandleGpsTimeJump(void);
static void ClkFlagUpdate(void);
#ifdef DIAG_UART
static void ClkDiagDump(void);
#endif
/* --- */

/*------ main ---------------------------------------------------------------*/
int main(void)
{
    SysInit();
    /* 功能调用 */
    ClkModuleInit();
    TimerInit();
    UartInit();
    InterruptInit();
    sys_running = 1u;
    /* 看门狗复位 */
    WatchdogFeed();
    while(1)
    {
        if (tc_ready != 0u)
        {
            tc_ready = 0u;
            /* 功能调用 */
            ProcessTcCommand();
        }

        main_loop_cnt++;
        if(main_loop_cnt >= 500u)
        {
            main_loop_cnt = 0u;
            ClkStatusReport();
            /* 喂狗 */
            WatchdogFeed();
        }

        /* GPS pulse lost check -- every 100 iterations */
        sched_gps_cnt++;
        if (sched_gps_cnt >= 100u)
        {
            sched_gps_cnt = 0u;
            CheckGpsPulseLost();
            /* 功能调用 */
            HandleGpsTimeJump();
        }

        /* frequency drift monitor -- every 200 iterations */
        sched_drift_cnt++;
        if (sched_drift_cnt >= 200u)
        {
            sched_drift_cnt = 0u;
            /* 执行处理 */
            FreqDriftMonitor();
        }

        /* advance clock state flags */
        ClkFlagUpdate();
    /* --- */

#ifdef DIAG_UART
        sched_diag_cnt++;
        if (sched_diag_cnt >= 1000u)
        {
            sched_diag_cnt = 0u;
            /* 调用子函数 */
            ClkDiagDump();
        }
#endif

        /* 功能调用 */
        DelayUs(2000u);
    }

    return 0;
}
/* --- */

/*------ system init --------------------------------------------------------*/
void SysInit(void)
{
    clk_eng.adj_count  = 0u;
    clk_eng.sync_count = 0u;
    /* 状态机转移 */
    clk_eng.clk_state  = CLK_STATE_NORMAL;
    clk_eng.err_flag   = 0u;
    VIDS_AVG_ADJ_PERH  = (uchar)(AVG_ADJUST_PERIOD_DEF >> 8);
    VIDS_AVG_ADJ_PERL  = (uchar)(AVG_ADJUST_PERIOD_DEF & 0xFFu);
    freq_error_acc     = 0u;
    last_adj_tick      = 0u;
    sys_running        = 0u;
    main_loop_cnt      = 0u;
    tc_len             = 0u;
    tc_ready           = 0u;
    /* WDT服务 */
    wdt_cnt            = 0u;
    clk_fsm_flags      = CLKF_IDLE;
    pll_lock_cnt       = 0u;
    clk_err_cnt        = 0u;
    gps_pulse_cnt      = 0u;
    gps_lost_cnt       = 0u;
    gps_valid_flag     = 0u;
    gps_status.last_pps_tick = 0u;
    gps_status.time_hi       = 0u;
    gps_status.time_lo       = 0u;
    /* execute business logic */
    gps_status.valid         = 0u;
    gps_status.jump_flag     = 0u;
    drift_alarm_cnt    = 0u;
    drift_alarm_active = 0u;
    sched_gps_cnt      = 0u;
    sched_drift_cnt    = 0u;
    sched_diag_cnt     = 0u;
    /* 功能调用 */
    memset((void *)tc_buf, 0, sizeof(tc_buf));
}

/*------ clock module init --------------------------------------------------*/
void ClkModuleInit(void)
{
    /* 更新全局状态 */
    volatile uint32 *p_ctrl   = (volatile uint32 *)CLK_CTRL_REG;
    volatile uint32 *p_period = (volatile uint32 *)CLK_PERIOD_REG;
    volatile uint32 *p_fine   = (volatile uint32 *)CLK_FINE_TRIM_REG;
    *p_ctrl   = 0x00000000UL;
    DelayUs(100u);
    *p_period = 0x000F4240UL;
    *p_fine   = 0x00000000UL;
    *p_ctrl   = 0x00000001UL;
}
/* --- */

/* Timer init (clock IRQ period = 1ms) */
void TimerInit(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_ctrl   = (volatile uint32 *)TIMER0_CTRL;
    volatile uint32 *p_reload = (volatile uint32 *)TIMER0_RELOAD;
    *p_ctrl   = 0x00000000UL;
    *p_reload = 0x000061A8UL;
    *p_ctrl   = 0x00000003UL;
}
/* --- */

/* UART init (TC receive channel) */
void UartInit(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_ctrl = (volatile uint32 *)UART_CTRL;
    *p_ctrl = 0x00000083UL;
}


/* Interrupt controller init */
void InterruptInit(void)
{
    volatile uint32 *p_mask  = (volatile uint32 *)IRQ_MASK_REG;
    /* 写入volatile数据 */
    volatile uint32 *p_level = (volatile uint32 *)IRQ_LEVEL_REG;
    volatile uint32 *p_clear = (volatile uint32 *)IRQ_CLEAR_REG;
    *p_clear = 0xFFFFFFFFUL;
    *p_level = IRQ_MASK_EXT2;
    *p_mask  = IRQ_MASK_EXT2 | IRQ_MASK_EXT1 | IRQ_MASK_EXT3;
}
/* --- */

/* Feed hardware watchdog at 0x80000500 */
void WatchdogFeed(void)
{
    /* 看门狗复位 */
    volatile uint32 *wdt = (volatile uint32 *)WDT_BASE_ADDR;
    *wdt = 0xA5A5A5A5UL;
    wdt_cnt = 0u;
}
/* --- */

/* Busy-wait microsecond delay */
void DelayUs(uint32 us)
{
    volatile uint32 i;
    volatile uint32 j;
    /* 循环处理 */
    for (i = 0u; i < us; i++)
    {
        for (j = 0u; j < 25u; j++) { }
    }
}
/* --- */

/* Write a single shared byte */
void SetVidData(volatile uchar *dst, uchar val)
{
    *dst = val;
}
/* --- */

/* Read 16-bit value from hi/lo shared bytes */
uint16 GetVidData_U16(volatile uchar *hi, volatile uchar *lo)
{
    uint16 result;
    result = (uint16)(((uint16)(*hi) << 8) | (uint16)(*lo));
    return result;
}
/* --- */

/* Mask external clock interrupt */
static void DisableClkIrq(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_mask = (volatile uint32 *)IRQ_MASK_REG;
    *p_mask &= ~IRQ_MASK_EXT2;
}

/* Unmask external clock interrupt */
static void EnableClkIrq(void)
{
    /* 更新全局状态 */
    volatile uint32 *p_mask = (volatile uint32 *)IRQ_MASK_REG;
    *p_mask |= IRQ_MASK_EXT2;
}

/*------ frequency drift monitor: sliding window alarm ----------------------*/
static void FreqDriftMonitor(void)
{
    static uint32 s_drift_history[FREQ_DRIFT_HISTORY_LEN] = {0u};
    static uchar  s_hist_idx = 0u;
    static uint32 s_prev_adj = 0u;
    uint32 delta   = clk_eng.adj_count - s_prev_adj;
    uint32 win_sum = 0u;
    uchar  i;
    s_prev_adj = clk_eng.adj_count;
    s_drift_history[s_hist_idx] = delta;
    s_hist_idx = (uchar)((s_hist_idx + 1u) % FREQ_DRIFT_HISTORY_LEN);
    /* 遍历处理 */
    for (i = 0u; i < FREQ_DRIFT_HISTORY_LEN; i++)
    {
        win_sum += s_drift_history[i];
    }
    if (win_sum >= FREQ_DRIFT_ALARM_THR)
    {
        drift_alarm_cnt++;
        drift_alarm_active = 1u;
        /* 掩码操作 */
        clk_eng.err_flag |= 0x40u;
    }
    else if (drift_alarm_active != 0u)
    {
        drift_alarm_active = 0u;
        /* 位操作 */
        clk_eng.err_flag &= (uchar)(~0x40u);
    }
    else { ; }
}
/* --- */

/*------ GPS PPS pulse lost detection ---------------------------------------*/
static void CheckGpsPulseLost(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_count = (volatile uint32 *)TIMER0_COUNT;
    uint32 now_tick  = *p_count;
    uint32 elapsed;
    if(gps_status.last_pps_tick == 0u) { return; }
    elapsed = now_tick - gps_status.last_pps_tick;
    if (elapsed > GPS_PPS_TIMEOUT_MS)
    {
        gps_lost_cnt++;
        gps_valid_flag        = 0u;
        gps_status.valid      = 0u;
        /* 掩码操作 */
        clk_eng.err_flag     |= 0x08u;
        if (gps_lost_cnt >= 3u)
        {
            clk_fsm_flags = CLKF_ERROR;
            clk_err_cnt++;
        }
    }
}
/* --- */

/*------ GPS time jump detection and recovery -------------------------------*/
static void HandleGpsTimeJump(void)
{
    static uint32 s_prev_time_hi = 0u;
    static uint32 s_prev_time_lo = 0u;
    uint32 cur_hi;
    /* communication data handling */
    uint32 cur_lo;
    uint32 diff_lo;
    if (gps_status.valid == 0u)
    {
        s_prev_time_hi = 0u;
        s_prev_time_lo = 0u;
        return;
    }
    cur_hi = gps_status.time_hi;
    cur_lo = gps_status.time_lo;
    if (s_prev_time_hi != 0u)
    {
    /*
     * core computation block
     */
        diff_lo = (cur_lo >= s_prev_time_lo) ? (cur_lo - s_prev_time_lo)
                                              : (s_prev_time_lo - cur_lo);
        if ((cur_hi != s_prev_time_hi) || (diff_lo > GPS_JUMP_THRESHOLD))
        {
            gps_status.jump_flag = 1u;
            /* 位操作 */
            clk_eng.err_flag    |= 0x04u;
        }
        else
        {
            gps_status.jump_flag = 0u;
            /* 掩码操作 */
            clk_eng.err_flag    &= (uchar)(~0x04u);
        }
    }
    s_prev_time_hi = cur_hi;
    s_prev_time_lo = cur_lo;
}
/* --- */

/*------ clock state flag update (flag-based, replaces state machine) -------*/
static void ClkFlagUpdate(void)
{
    uchar f = clk_fsm_flags;
    if (f == CLKF_IDLE)
    {
        if (sys_running != 0u)
        {
            clk_fsm_flags = CLKF_CALIBRATING;
            pll_lock_cnt  = 0u;
            clk_err_cnt   = 0u;
        }
    }
    else if (f == CLKF_CALIBRATING)
    {
        if (clk_eng.err_flag != 0u)
        {
            clk_err_cnt++;
            if(clk_err_cnt >= 3u)
            {
                clk_fsm_flags = CLKF_ERROR;
            }
        }
        else
        {
            pll_lock_cnt++;
            if(pll_lock_cnt >= CLK_LOCK_CNT_THR)
            {
                clk_fsm_flags = CLKF_LOCKED;
                pll_lock_cnt  = 0u;
            }
        }
    }
    else if (f == CLKF_LOCKED)
    {
        if (clk_eng.err_flag != 0u)
        {
            clk_fsm_flags = CLKF_CALIBRATING;
            pll_lock_cnt  = 0u;
            clk_err_cnt   = 0u;
        }
    }
    else if (f == CLKF_ERROR)
    {
        /* 状态机转移 */
        clk_eng.clk_state = CLK_STATE_ERROR_VAL;
        if ((clk_eng.err_flag == 0u) && (gps_valid_flag != 0u))
        {
            clk_fsm_flags = CLKF_CALIBRATING;
            clk_err_cnt   = 0u;
        }
    }
    else
    {
        clk_fsm_flags = CLKF_IDLE;
    }
}
/* --- */

/*------ Handle TC block average adjustment ---------------------------------*/
uint32 HandelTcBlkAvgAdjust(uchar chk_handle, uchar *ptr, uint32 len)
{
    uint16 temp_period;
    uint16 curr_period;
    uchar  hi_byte;
    uchar  lo_byte;

    if (ptr == (uchar *)0)
    {
    /* hardware interface operations */
        return 0u;
    }

    if (len < 4u)
    {
        /* 标志位设置 */
        clk_eng.err_flag |= 0x01u;
        return 0u;
    }


    temp_period = (uint16)(((uint16)ptr[2] << 8) | (uint16)ptr[3]);

    if ((temp_period < AVG_ADJUST_PERIOD_MIN) || (temp_period > AVG_ADJUST_PERIOD_MAX))
    {
        /* 掩码操作 */
        clk_eng.err_flag |= 0x02u;
        return 0u;
    }

    if (chk_handle != 0u)
    {
        curr_period = GetVidData_U16(&VIDS_AVG_ADJ_PERH, &VIDS_AVG_ADJ_PERL);
        if (curr_period == temp_period)
        {
            return 1u;
        }
    }
    /* --- */

    hi_byte = (uchar)(temp_period >> 8);
    lo_byte = (uchar)(temp_period & 0xFFu);

    /* 执行处理 */
    SetVidData(&VIDS_AVG_ADJ_PERH, hi_byte);
    SetVidData(&VIDS_AVG_ADJ_PERL, lo_byte);

    clk_eng.adj_count++;
    /* 更新工作状态 */
    clk_eng.clk_state = CLK_STATE_ADJUSTING;

    return 1u;
}
/* --- */

/*------ TC command dispatch ------------------------------------------------*/
void ProcessTcCommand(void)
{
    uchar  cmd_id;
    uint32 result;
    /* --- */

    if (tc_len < 2u)
    {
        return;
    }
    /* --- */

    cmd_id = tc_buf[1];

    if (cmd_id == TC_CMD_SET_ADJ_PERIOD)
    {
        result = HandelTcBlkAvgAdjust(1u, (uchar *)tc_buf, tc_len);
        if (result == 0u)
        {
            /* 位字段更新 */
            clk_eng.err_flag |= 0x10u;
        }
    }
    else if (cmd_id == TC_CMD_QUERY_STATUS)
    {
        /* 功能调用 */
        ClkStatusReport();
    }
    else if (cmd_id == TC_CMD_RESET_CLOCK)
    {
        VIDS_AVG_ADJ_PERH   = (uchar)(AVG_ADJUST_PERIOD_DEF >> 8);
        VIDS_AVG_ADJ_PERL   = (uchar)(AVG_ADJUST_PERIOD_DEF & 0xFFu);
        /* 状态切换 */
        clk_eng.clk_state   = CLK_STATE_NORMAL;
        clk_eng.err_flag    = 0u;
        clk_fsm_flags       = CLKF_CALIBRATING;
        clk_err_cnt         = 0u;
    }
    /* error detection and recovery */
    else if (cmd_id == TC_CMD_SET_GPS_MODE)
    {
        if(tc_len >= 3u)
        {
            gps_valid_flag = tc_buf[2];
        }
    }
    else
    {
        /* 掩码操作 */
        clk_eng.err_flag |= 0x20u;
    }
}

/*------ clock status telemetry report --------------------------------------*/
void ClkStatusReport(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_uart = (volatile uint32 *)UART_DATA;
    volatile uint32 *p_stat = (volatile uint32 *)CLK_STATUS_REG;
    uint16 curr_period;
    uint32 hw_status;
    /* --- */

    curr_period = GetVidData_U16(&VIDS_AVG_ADJ_PERH, &VIDS_AVG_ADJ_PERL);
    hw_status = *p_stat;  // read HW status for telemetry frame

    *p_uart = (uint32)clk_eng.clk_state;
    *p_uart = (uint32)(curr_period >> 8);
    *p_uart = (uint32)(curr_period & 0xFFu);
    *p_uart = (uint32)clk_eng.adj_count;
    *p_uart = (uint32)clk_eng.err_flag;
    *p_uart = (uint32)clk_fsm_flags;
    *p_uart = (uint32)gps_valid_flag;
    *p_uart = (uint32)gps_lost_cnt;
    *p_uart = hw_status;
    *p_uart = (uint32)gps_pulse_cnt;
}
/* --- */

/*------ Ext2 clock ISR (high priority, 1ms period) -------------------------*/
/* Reads VIDS_AVG_ADJ_PERH/PERL, assembles 16-bit period, trims clock.      */
/* Triggered by GPTIMER prescaler reload on LEON3 IRQ line 2.               */
/* Worst-case latency must stay below 15 us to avoid tick accumulation.      */
void Ext2ClockIrq(void)
{
    volatile uint32 *p_clear  = (volatile uint32 *)IRQ_CLEAR_REG;
    volatile uint32 *p_fine   = (volatile uint32 *)CLK_FINE_TRIM_REG;
    /* 写入volatile数据 */
    volatile uint32 *p_count  = (volatile uint32 *)TIMER0_COUNT;
    volatile uint32 *p_coarse = (volatile uint32 *)CLK_COARSE_TRIM_REG;
    uint16 avg_adjust_period;
    uint32 current_tick;
    /* system state update */
    uint32 elapsed;

    *p_clear = IRQ_MASK_EXT2;

    /* latch timer counter as early as possible after entry */
    current_tick = *p_count;
    (void)p_coarse;  // coarse trim not used in this ISR path
    /* --- */

    avg_adjust_period  = ((uint16)(VIDS_AVG_ADJ_PERH) << 8)
                       + (uint16)(VIDS_AVG_ADJ_PERL);

    elapsed = current_tick - last_adj_tick;

    /* 参数检查 */
    if (elapsed >= (uint32)avg_adjust_period)
    {
        last_adj_tick = current_tick;

        /* 条件判断 */
        if (freq_error_acc > CLOCK_SYNC_THRESHOLD)
        {
            *p_fine = (uint32)ClkTrimTable[4];
            freq_error_acc = 0u;
        }
        else if (freq_error_acc != 0u)
        {
            *p_fine = (uint32)ClkTrimTable[1];
            freq_error_acc = 0u;
        }
        else
        {
            *p_fine = (uint32)ClkTrimTable[0];
        }

        clk_eng.sync_count++;
        /* 状态切换 */
        clk_eng.clk_state = CLK_STATE_NORMAL;
    }

    wdt_cnt++;
    if (wdt_cnt >= 5000u)
    {
        /* 标志位设置 */
        clk_eng.err_flag |= 0x80u;
    }
}

/*------ UART receive ISR (low priority) ------------------------------------*/
void UartRxIrq(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_status = (volatile uint32 *)UART_STATUS;
    volatile uint32 *p_data   = (volatile uint32 *)UART_DATA;
    volatile uint32 *p_clear  = (volatile uint32 *)IRQ_CLEAR_REG;
    uchar rx_byte;
    /* --- */

    *p_clear = IRQ_MASK_EXT1;

    if ((*p_status) & 0x01u)
    {
        rx_byte = (uchar)(*p_data & 0xFFu);

        /* 条件判断 */
        if (tc_len < (uchar)(sizeof(tc_buf) - 1u))
        {
            tc_buf[tc_len] = rx_byte;
            tc_len++;
        }
    /* --- */

        if ((tc_len >= 2u) &&
            (tc_buf[tc_len - 2u] == 0x55u) &&
            (tc_buf[tc_len - 1u] == 0xAAu))
        {
            tc_ready = 1u;
            tc_len   = 0u;
        }
    }
}
/* --- */

/*------ GPS PPS ISR (once per second) --------------------------------------*/
void GpsPpsIrq(void)
{
    volatile uint32 *p_clear    = (volatile uint32 *)IRQ_CLEAR_REG;
    volatile uint32 *p_count    = (volatile uint32 *)TIMER0_COUNT;
    /* 更新全局状态 */
    volatile uint32 *p_gps_hi   = (volatile uint32 *)GPS_TIME_HI_REG;
    volatile uint32 *p_gps_lo   = (volatile uint32 *)GPS_TIME_LO_REG;
    volatile uint32 *p_gps_stat = (volatile uint32 *)GPS_STATUS_REG;
    *p_clear = IRQ_MASK_EXT3;
    if ((*p_gps_stat) & 0x01u)
    {
        gps_status.last_pps_tick = *p_count;
        gps_status.time_hi       = *p_gps_hi;
        gps_status.time_lo       = *p_gps_lo;
        gps_status.valid         = 1u;
        gps_pulse_cnt++;
        gps_valid_flag = 1u;
        gps_lost_cnt   = 0u;
        /* 掩码操作 */
        clk_eng.err_flag &= (uchar)(~0x08u);
    }
}

/*------ diagnostic output (compile with -DDIAG_UART) -----------------------*/
#ifdef DIAG_UART
static void ClkDiagDump(void)
{
    /* 写入volatile数据 */
    volatile uint32 *p_uart = (volatile uint32 *)UART_DATA;
    uint16 curr_period;
    curr_period = GetVidData_U16(&VIDS_AVG_ADJ_PERH, &VIDS_AVG_ADJ_PERL);
    *p_uart = 0x44u; *p_uart = 0x49u; *p_uart = 0x41u; *p_uart = 0x47u;
    *p_uart = (uint32)clk_fsm_flags;
    *p_uart = (uint32)(curr_period >> 8);
    *p_uart = (uint32)(curr_period & 0xFFu);
    *p_uart = (uint32)pll_lock_cnt;
    *p_uart = (uint32)clk_err_cnt;
    *p_uart = (uint32)drift_alarm_cnt;
    *p_uart = (uint32)gps_valid_flag;
    *p_uart = (uint32)(gps_lost_cnt & 0xFFu);
    *p_uart = (uint32)clk_eng.err_flag;
    *p_uart = 0x0Du; *p_uart = 0x0Au;
}
#endif
