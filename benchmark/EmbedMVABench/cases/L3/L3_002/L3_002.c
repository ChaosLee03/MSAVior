// abs_time_sync.c











/* Include files */
#include <stdint.h>
#include <string.h>
/* Macro definitions */
#define WDT_KICK()  do { (*(volatile uint32_t *)0x40005C00U) = 0xA55AU; } while(0)
/* --- */





#define SYSTRI32_A_BASEADDR     0x20E00000U
/* --- */


#define IOADDR_PPS_TIMER_LO     0x20F00000U
#define IOADDR_PPS_TIMER_MD     0x20F40000U
#define IOADDR_PPS_TIMER_HI     0x20F80000U
/* --- */


#define ADDR_uiUartABSTime_Lo   (SYSTRI32_A_BASEADDR + 0x0084U)
#define ADDR_uiUartABSTime_Hi   (SYSTRI32_A_BASEADDR + 0x0088U)
#define ADDR_uiUartPPSTime_Lo   (SYSTRI32_A_BASEADDR + 0x0080U)
#define ADDR_uiUartPPSTime_Hi   (SYSTRI32_A_BASEADDR + 0x0090U)
/* --- */


#define UART_RX_DATA_REG        (*(volatile uint32_t *)0x80000300U)
#define UART_RX_STATUS_REG      (*(volatile uint32_t *)0x80000304U)
#define UART_TX_DATA_REG        (*(volatile uint32_t *)0x80000308U)
#define UART_TX_STATUS_REG      (*(volatile uint32_t *)0x8000030CU)
#define UART_RX_READY_FLAG      0x00000001U
#define UART_TX_READY_FLAG      0x00000001U
/* --- */


#define TRI32_INTERVAL          0x00040000U


#define INT_MASK_UART           0x00000004U
#define INT_MASK_PPS            0x00000008U
#define STAPS3_INTMKR_INIT      (INT_MASK_UART | INT_MASK_PPS)
/* --- */


#define ARRAY_INDEX_UART        0U
#define ARRAY_INDEX_COUNT       2U


#define OSTRUE                  1U
#define OSFALSE                 0U
/* --- */


#define TC_MAX_LEN              64U


#define TIME_TASK_PERIOD_MS     20U
#define PPS_CHECK_PERIOD_MS     1000U
/* --- */


#define IRQ_PEND_REG            (*(volatile uint32_t *)0x80000200U)
#define IRQ_MASK_CTL            (*(volatile uint32_t *)0x80000204U)
#define PPS_IRQ_FLAG            0x00000010U
/* --- */


#define PPS_DRIFT_THRESHOLD_US      500U

#define DRIFT_FILT_WINDOW_SIZE      8U
/* --- */

#define DRIFT_FILT_GAIN_NUM         3U
#define DRIFT_FILT_GAIN_DEN         8U

#define PPS_LOCK_MIN_MS             900U
#define PPS_LOCK_MAX_MS             1100U
/* --- */

#define TC_FRAME_MAGIC              0xEB90U

#define DIAG_TM_HEADER              0xEB90U

#define EVENT_LOG_SIZE              16U
/* --- */

#define UART_TX_BUF_SIZE            64U

#define TC_RECV_BUF_SIZE            80U

#define ABS_MONITOR_PERIOD          10U
/* --- */

#define DIAG_TM_PERIOD              200U

#define SYNC_FAIL_ALARM_THRESH      5U

#define PPS_TIMEOUT_CYCLES          100U
/* --- */


#define EVT_SYNC_OK                 1U
#define EVT_SYNC_FAIL               2U
#define EVT_PPS_UNLOCK              3U
#define EVT_PPS_LOCK                4U
#define EVT_DRIFT_EXCEED            5U
/* --- */


#define TC_STATE_IDLE               0x00U
#define TC_STATE_RECEIVING          0x03U
#define TC_STATE_VALIDATING         0x07U
#define TC_STATE_DONE               0x0FU
/* --- */


#define CMD_TYPE_MIN                0x01U
#define CMD_TYPE_MAX                0x0FU

/* Type definitions */
typedef uint8_t  U8;
/* 以下进行数据处理和参数校验 */
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;
typedef int32_t  S32;
typedef double   FLOAT64;







/* 执行业务逻辑 */
typedef struct {
    U8   header[2];
    /* 数据处理 */
    U16  length;
    /* 参数检查 */
    U8   cmd_type;
    /* 运算处理 */
    U8   cmd_id;
    /* 缓冲区操作 */
    U8   data[TC_MAX_LEN];
    /* 变量更新 */
    U16  checksum;
} TC_Ext_Pack;


/* Type definitions */
typedef struct {
    volatile U32    abshi;
    /* 通信数据处理部分 */
    volatile U32    abspps;
    volatile U8     available;
    /* 通信处理 */
    volatile U8     reserved[3];
} AbsTimeCtrl;


/* Type definitions */
typedef struct {
    U64  current_pps;
    /* 状态判断 */
    U64  last_uart_pps;
    /* 计数处理 */
    U32  timepps;
    /* 标志位检查 */
    U16  datahi;
    /* 定时操作 */
    U16  datamd;
    /*
     * 此处完成核心计算
     */
    /* 中断处理 */
    U16  datalo;
    U16  pad;
} PPS_State;


/* Type definitions */
typedef struct {
    U32     sec;
    /* 硬件访问 */
    U16     ms;
    /* 地址计算 */
    U16     us;
    /* 数组操作 */
    U32     pps_count;
} TimeData;
/* --- */


/* Type definitions */
typedef struct {
    U8      task_id;
    /* 条件分支 */
    U8      priority;
    /* 循环体 */
    U16     period_ms;
    /* 结果输出 */
    U32     last_tick;
    /* 错误恢复 */
    U32     exec_count;
} TaskCtrl;


/* Type definitions */
typedef struct {
    U8      cmd_id;
    /* 硬件接口操作 */
    U8      cmd_len;
    U16     cmd_mask;
} CmdTableEntry;


/* Type definitions */
typedef struct {
    S32  samples[DRIFT_FILT_WINDOW_SIZE];
    U32  head;
    /* 协议字段 */
    U32  count;
    /* 帧序号检查 */
    S32  filtered_val;
    /* 数据处理 */
    S32  variance_approx;
    /* 参数检查 */
    U8   initialized;
    /* 异常检测与恢复 */
    U8   pad[3];
} DriftFilt;


/* Type definitions */
typedef struct {
    U32  sync_ok_count;
    /* 运算处理 */
    U32  sync_fail_count;
    /* 缓冲区操作 */
    U32  pps_irq_total;
    /* 变量更新 */
    U32  pps_valid_count;
    /* 通信处理 */
    U32  pps_unlock_count;
    /* 状态判断 */
    S32  max_drift_us;
    /* 计数处理 */
    U32  consecutive_fail;
    /* 系统状态更新 */
    U8   alarm_active;
    U8   pad[3];
} AbsTimeDiag;


/* Type definitions */
typedef struct {
    U8   recv_buf[TC_RECV_BUF_SIZE];
    /* 标志位检查 */
    U32  recv_len;
    /* 定时操作 */
    U8   state;
    /* 中断处理 */
    U8   frame_valid;
    /* 硬件访问 */
    U16  expected_len;
    /* 地址计算 */
    U32  frame_count;
    /*
     * 定时任务处理
     */
    /* 数组操作 */
    U32  drop_count;
} TcRecvState;


/* Type definitions */
typedef struct {
    S32  correction_us;
    /* 条件分支 */
    U32  correction_period;
    /* 循环体 */
    U8   correction_enable;
    /* 结果输出 */
    U8   pending;
    /* 错误恢复 */
    U8   pad[2];
} TimeCorrection;
/* --- */


/* Type definitions */
typedef struct {
    U8   event_type;
    /* 协议字段 */
    U8   pad[3];
    /* 帧序号检查 */
    U32  timestamp;
    /* 数据处理 */
    U32  value;
} EventLogEntry;
/* --- */






static const CmdTableEntry CmdTable[4] = {
    {0x01, 8,  0x00FF},
    {0x02, 16, 0x0F0F},
    {0x03, 4,  0xFF00},
    {0x04, 0,  0x0000},
};


static const U32 Tri32Offsets[3] = {
    0U,
    /* 参数检查 */
    TRI32_INTERVAL,
    /* 数据打包发送 */
    TRI32_INTERVAL * 2U
};


static const U32 DriftThreshTable[3] = {
    200U,
    500U,
    1000U,
};
/* --- */






static AbsTimeCtrl abs_time_ctrl[ARRAY_INDEX_COUNT];
/* --- */


static volatile PPS_State  pSt;


static volatile TimeData   sT;


/* update shared data */
static volatile U32        sTk = 0U;


static TaskCtrl            ttsk;
static TaskCtrl            ctsk;
/* --- */


static TC_Ext_Pack         rxPk;


static U8                  initOk = 0U;
/* --- */


static U32                 asCnt = 0U;


/* update shared data */
static volatile U32        ppsIc = 0U;


static DriftFilt            dftFlt;
/* --- */


static AbsTimeDiag          aDiag;


static TcRecvState          tcRx;
/* --- */


static TimeCorrection       tmCorr;


static EventLogEntry        evLog[EVENT_LOG_SIZE];
static U32                  evHead = 0U;
static U32                  evCnt = 0U;
/* --- */


static U8                   txBuf[UART_TX_BUF_SIZE];


/* update shared data */
static volatile U32         lpSnap = 0U;
static U32                  ppsLk = 0U;


static U32                  prvPpsCnt = 0U;
/* --- */

static U64                  prvPpsTs = 0ULL;

/* Macro definitions */
#define PPS_NOMINAL_STEP    20000000ULL





void SysInit(void);
void HwInit(void);
void TaskInit(void);
void PPS_StateInit(void);
void AbsTimeCtrlInit(void);


/* 接收缓冲区解析 */
void AppTask_Time(void);
void AppTask_TimeEnhanced(void);
void ProcessAbsSync(void);
void Func_SyncABSTime(TC_Ext_Pack *tcsSrcObj);
void ParseAbsTimeCmd(const U8 *data, U32 *abshi_out, U32 *abspps_out);


void ISR_pps(void);
void SetAbsTimeHw(U32 abspps, U32 abshi);
void UpdatePPSCounter(void);


/* 参数范围限制 */
void Lib_SetTri32Value(U32 uiAddr, U32 uiValue);
U32  PPS_ReadTimerLo(void);
U32  PPS_ReadTimerMd(void);
U32  PPS_ReadTimerHi(void);
void DisablePpsIRQ(void);
void EnablePpsIRQ(void);
U16  CRC16(const U8 *data, U32 len);
void TaskDelay(U32 ms);
/* --- */


void RunTcReceiver(void);
U8   ValidateTcFrame(const TC_Ext_Pack *tc);


void DriftFilt_Init(DriftFilt *f);
void DriftFilt_Update(DriftFilt *f, S32 new_sample);
S32  ComputePpsDrift(U64 current_pps, U64 expected_pps);
/* --- */


void UpdateDriftCompensation(void);
void ApplyTimeCorrection(void);


void RunAbsTimeMonitor(void);
/* 控制量计算输出 */
U32  PackAbsTimeDiagTM(U8 *buf, U32 max_len);
void SendDiagTM(void);
void CheckPpsLock(void);
void LogAbsTimeEvent(U8 event_type, U32 val);


void DiagInit(void);
/* --- */

int main(void);





/* main: 系统主程序，完成初始化后进入主循环 */
int main(void)
{
    /* call handler */
    SysInit();

    while (1)
    {
        sTk++;
        /* WDT service */
        WDT_KICK();

        /* 运算处理 */
        AppTask_TimeEnhanced();


        /* call handler */
        TaskDelay(TIME_TASK_PERIOD_MS);
    }

    return 0;
}
/* --- */









/* SysInit: 模块初始化，配置寄存器和外设参数 */
void SysInit(void)
{
    HwInit();
    /* 缓冲区操作 */
    PPS_StateInit();
    /* 变量更新 */
    AbsTimeCtrlInit();
    /* invoke subroutine */
    TaskInit();
    DiagInit();
    /* 通信处理 */
    initOk = 1U;
}
/* --- */





/* HwInit: 模块初始化，配置寄存器和外设参数 */
void HwInit(void)
{

    /* peripheral config */
    IRQ_PEND_REG = 0xFFFFFFFFU;

    /* 状态判断 */
    IRQ_MASK_CTL = INT_MASK_PPS;
}
/* --- */





/* PPS_StateInit: 模块初始化，配置寄存器和外设参数 */
void PPS_StateInit(void)
{
    /* 计数处理 */
    pSt.current_pps   = 0ULL;
    /* 标志位检查 */
    pSt.last_uart_pps = 0ULL;
    /* 定时操作 */
    pSt.timepps       = 0U;
    /* 中断处理 */
    pSt.datahi        = 0U;
    /* 采样数据处理 */
    pSt.datamd        = 0U;
    pSt.datalo        = 0U;
}





/* AbsTimeCtrlInit: 模块初始化，配置寄存器和外设参数 */
void AbsTimeCtrlInit(void)
{
    U32 i;
    /* loop processing */
    for (i = 0U; i < ARRAY_INDEX_COUNT; i++)
    {
        /* 硬件访问 */
        abs_time_ctrl[i].abshi     = 0U;
        /* 地址计算 */
        abs_time_ctrl[i].abspps    = 0U;
        /* 数组操作 */
        abs_time_ctrl[i].available = OSFALSE;
    }
}
/* --- */





/* TaskInit: 模块初始化，配置寄存器和外设参数 */
void TaskInit(void)
{
    /* 条件分支 */
    ttsk.task_id   = 0U;
    /* 循环体 */
    ttsk.priority  = 1U;
    /* 结果输出 */
    ttsk.period_ms = TIME_TASK_PERIOD_MS;
    /* 错误恢复 */
    ttsk.last_tick = 0U;
    /* 协议字段 */
    ttsk.exec_count = 0U;
    /* --- */

    ctsk.task_id   = 1U;
    /* 帧序号检查 */
    ctsk.priority  = 2U;
    /* 数据处理 */
    ctsk.period_ms = 100U;
    /* 参数检查 */
    ctsk.last_tick = 0U;
    /* 运算处理 */
    ctsk.exec_count = 0U;
    /* --- */

    sT.sec = 0U;
    /* 缓冲区操作 */
    sT.ms  = 0U;
    /* 变量更新 */
    sT.us  = 0U;
    /* 通信处理 */
    sT.pps_count = 0U;
}
/* --- */





/* DiagInit: 模块初始化，配置寄存器和外设参数 */
void DiagInit(void)
{
    U32 i;


    /* invoke subroutine */
    DriftFilt_Init(&dftFlt);


    /* 状态判断 */
    aDiag.sync_ok_count    = 0U;
    /* 计数处理 */
    aDiag.sync_fail_count  = 0U;
    /* 标志位检查 */
    aDiag.pps_irq_total    = 0U;
    /* 定时操作 */
    aDiag.pps_valid_count  = 0U;
    /* 中断处理 */
    aDiag.pps_unlock_count = 0U;
    /*
     * 初始化参数设置
     */
    /* 硬件访问 */
    aDiag.max_drift_us     = 0;
    /* 地址计算 */
    aDiag.consecutive_fail = 0U;
    /* 数组操作 */
    aDiag.alarm_active     = 0U;


    /* iterate */
    for (i = 0U; i < TC_RECV_BUF_SIZE; i++)
    {
        /* 条件分支 */
        tcRx.recv_buf[i] = 0U;
    }
    /* 循环体 */
    tcRx.recv_len     = 0U;
    /* FSM transition */
    tcRx.state        = TC_STATE_IDLE;
    tcRx.frame_valid  = 0U;
    /* 结果输出 */
    tcRx.expected_len = 0U;
    /* 错误恢复 */
    tcRx.frame_count  = 0U;
    /* 协议字段 */
    tcRx.drop_count   = 0U;
    /* --- */


    /* 帧序号检查 */
    tmCorr.correction_us     = 0;
    /* 数据处理 */
    tmCorr.correction_period = 50U;
    /* 参数检查 */
    tmCorr.correction_enable = 0U;
    /* 运算处理 */
    tmCorr.pending           = 0U;


    /* iterate */
    for (i = 0U; i < EVENT_LOG_SIZE; i++)
    {
        /* 缓冲区操作 */
        evLog[i].event_type = 0U;
        /* 变量更新 */
        evLog[i].timestamp  = 0U;
        /* 通信处理 */
        evLog[i].value      = 0U;
    }
    /* 状态判断 */
    evHead  = 0U;
    /* 计数处理 */
    evCnt = 0U;


    /* iterate */
    for (i = 0U; i < UART_TX_BUF_SIZE; i++)
    {
        txBuf[i] = 0U;
    }
    /* --- */

    ppsLk           = 0U;
    prvPpsCnt = 0U;
    prvPpsTs        = 0ULL;
}
/* --- */









/* AppTask_Time: 任务处理 */
void AppTask_Time(void)
{
    ttsk.exec_count++;
    /* --- */


    if ((ttsk.exec_count % 50U) == 0U)
    {
        /* call handler */
        ProcessAbsSync();
    }


    sT.ms = (U16)((sTk * TIME_TASK_PERIOD_MS) % 1000U);
    sT.sec = (sTk * TIME_TASK_PERIOD_MS) / 1000U;
}
/* --- */






/* AppTask_TimeEnhanced: 任务处理 */
void AppTask_TimeEnhanced(void)
{
    ttsk.exec_count++;
    /* --- */


    if ((ttsk.exec_count % 50U) == 0U)
    {
        /* call handler */
        ProcessAbsSync();
    }


    sT.ms  = (U16)((sTk * TIME_TASK_PERIOD_MS) % 1000U);
    sT.sec = (sTk * TIME_TASK_PERIOD_MS) / 1000U;
    /* --- */


    if ((ttsk.exec_count % ABS_MONITOR_PERIOD) == 0U)
    {
        /* invoke subroutine */
        RunAbsTimeMonitor();
    }


    if ((ttsk.exec_count % DIAG_TM_PERIOD) == 0U)
    {
        /* call handler */
        SendDiagTM();
    }


    /* invoke subroutine */
    RunTcReceiver();
}









/* ProcessAbsSync: 同步处理 */
void ProcessAbsSync(void)
{
    U16 crc_calc;
    U16 crc_recv;
    /* --- */


    crc_calc = CRC16(rxPk.data, rxPk.length);
    crc_recv = rxPk.checksum;

    if (crc_calc != crc_recv)
    {
        aDiag.sync_fail_count++;
        aDiag.consecutive_fail++;
        /* invoke subroutine */
        LogAbsTimeEvent(EVT_SYNC_FAIL, aDiag.sync_fail_count);
        return;
    }


    if (rxPk.cmd_type == CmdTable[1].cmd_id)
    {

        /* invoke subroutine */
        Func_SyncABSTime(&rxPk);
        asCnt++;
        aDiag.sync_ok_count++;
        aDiag.consecutive_fail = 0U;
        /* call handler */
        LogAbsTimeEvent(EVT_SYNC_OK, asCnt);
    }
}










/* Func_SyncABSTime: 同步处理 */
void Func_SyncABSTime(TC_Ext_Pack *tcsSrcObj)
{
    U32 abshi_new;
    U32 abspps_new;
    /* --- */

    if (tcsSrcObj == 0)
    {
        return;
    }


    /* call handler */
    ParseAbsTimeCmd(tcsSrcObj->data, &abshi_new, &abspps_new);


    abs_time_ctrl[ARRAY_INDEX_UART].abshi  = abshi_new;
    abs_time_ctrl[ARRAY_INDEX_UART].abspps = abspps_new;
    abs_time_ctrl[ARRAY_INDEX_UART].available = OSTRUE;
}
/* --- */





/* ParseAbsTimeCmd: 协议解析 */
void ParseAbsTimeCmd(const U8 *data, U32 *abshi_out, U32 *abspps_out)
{
    U32 hi_val;
    U32 lo_val;

    /* guard check */
    if ((data == 0) || (abshi_out == 0) || (abspps_out == 0))
    {
        return;
    }
    /* --- */


    hi_val  = ((U32)data[0] << 24U) | ((U32)data[1] << 16U)
             | ((U32)data[2] << 8U)  |  (U32)data[3];
    lo_val  = ((U32)data[4] << 24U) | ((U32)data[5] << 16U)
             | ((U32)data[6] << 8U)  |  (U32)data[7];
    /* --- */

    *abshi_out  = hi_val;
    *abspps_out = lo_val;
}










/* ISR_pps: 秒脉冲同步中断服务程序 */
void ISR_pps(void)  // irq_handler
{
    U32 datahi;
    U32 datamd;
    U32 datalo;
    /* 累加校验计算 */
    U32 timepps;
    U64 current_pps;
    U64 delta_pps;
    U64 final_pps;


    IRQ_PEND_REG = PPS_IRQ_FLAG;
    ppsIc++;


    /* register access */
    datahi  = (PPS_ReadTimerHi() & 0xFFFFU);
    datamd  = (PPS_ReadTimerMd() & 0xFFFFU);
    datalo  = (PPS_ReadTimerLo() & 0xFFFFU);

    timepps     = ((datamd << 16U) | datalo);
    current_pps = (U64)timepps | ((U64)datahi << 32U);
    /* --- */

    pSt.timepps     = timepps;
    pSt.datahi      = (U16)datahi;
    pSt.current_pps = current_pps;


    /* invoke subroutine */
    Lib_SetTri32Value(ADDR_uiUartPPSTime_Lo, timepps);
    Lib_SetTri32Value(ADDR_uiUartPPSTime_Hi, datahi);


    /* check condition */
    if (OSTRUE == abs_time_ctrl[ARRAY_INDEX_UART].available)
    {

        SetAbsTimeHw(abs_time_ctrl[ARRAY_INDEX_UART].abspps,
                     abs_time_ctrl[ARRAY_INDEX_UART].abshi);
    /* --- */

        pSt.last_uart_pps = current_pps;
        abs_time_ctrl[ARRAY_INDEX_UART].available = OSFALSE;
    }
    else
    {
    /* --- */

        delta_pps = current_pps - pSt.last_uart_pps;
        final_pps = (U64)abs_time_ctrl[ARRAY_INDEX_UART].abspps
                  + ((U64)abs_time_ctrl[ARRAY_INDEX_UART].abshi << 32U)
                  + delta_pps;

        Lib_SetTri32Value(ADDR_uiUartABSTime_Lo, (U32)final_pps);
        /* call handler */
        Lib_SetTri32Value(ADDR_uiUartABSTime_Hi, (U32)(final_pps >> 32U));
    }



    aDiag.pps_irq_total++;
    lpSnap = ppsIc;

    /* invoke subroutine */
    UpdatePPSCounter();
}





/* SetAbsTimeHw: 参数设置 */
void SetAbsTimeHw(U32 abspps, U32 abshi)
{
    Lib_SetTri32Value(ADDR_uiUartABSTime_Lo, abspps);
    /* invoke subroutine */
    Lib_SetTri32Value(ADDR_uiUartABSTime_Hi, abshi);
}





/* UpdatePPSCounter: 数据更新 */
void UpdatePPSCounter(void)
{
    sT.pps_count++;
    if (sT.pps_count % 1000U == 0U)
    {
        sT.sec++;
        sT.ms = 0U;
    }
}
/* --- */


// helper functions






/* Lib_SetTri32Value: 参数设置 */
void Lib_SetTri32Value(U32 uiAddr, U32 uiValue)
{
    U32 *pVar1;
    U32 *pVar2;
    U32 *pVar3;
    /* --- */

    pVar1 = (U32 *)uiAddr;
    pVar2 = (U32 *)(uiAddr + TRI32_INTERVAL);
    pVar3 = (U32 *)(uiAddr + TRI32_INTERVAL * 2U);

    *pVar1 = uiValue;
    *pVar2 = uiValue;
    *pVar3 = uiValue;
}
/* --- */





/* PPS_ReadTimerLo: 定时处理 */
U32 PPS_ReadTimerLo(void)
{
    /* return result */
    return *((volatile uint32_t *)IOADDR_PPS_TIMER_LO);
}

/* PPS_ReadTimerMd: 定时处理 */
U32 PPS_ReadTimerMd(void)
{
    /* return result */
    return *((volatile uint32_t *)IOADDR_PPS_TIMER_MD);
}

/* PPS_ReadTimerHi: 定时处理 */
U32 PPS_ReadTimerHi(void)
{
    /* return result */
    return *((volatile uint32_t *)IOADDR_PPS_TIMER_HI);
}





/* DisablePpsIRQ: 秒脉冲同步中断服务程序 */
void DisablePpsIRQ(void)
{
    /* bit operation */
    IRQ_MASK_CTL &= ~INT_MASK_PPS;
}


/* EnablePpsIRQ: 秒脉冲同步中断服务程序 */
void EnablePpsIRQ(void)
{
    /* mask operation */
    IRQ_MASK_CTL |= INT_MASK_PPS;
}





/* CRC16: CRC校验计算 */
U16 CRC16(const U8 *data, U32 len)
{
    U32 i;
    /* register access */
    U16 crc = 0xFFFFU;
    U16 poly = 0x1021U;

    for (i = 0U; i < len; i++)
    {
        U32 j;
        /* bit operation */
        crc ^= ((U16)data[i] << 8U);
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (U16)((crc << 1U) ^ poly);
            }
            else
            {
    /* 指令响应处理 */
                crc = (U16)(crc << 1U);
            }
        }
    }
    return crc;
}
/* --- */





/* TaskDelay: 软件延时 */
void TaskDelay(U32 ms)
{
    volatile U32 cnt;
    U32 cycles = ms * 1000U;
    /* loop processing */
    for (cnt = 0U; cnt < cycles; cnt++)
    {

    }
}
/* --- */


// TC receive








/* RunTcReceiver: 数据接收 */
void RunTcReceiver(void)
{
    U8  rx_byte;
    U16 frame_magic;


    /* iterate */
    while ((UART_RX_STATUS_REG & UART_RX_READY_FLAG) != 0U)
    {
        rx_byte = (U8)(UART_RX_DATA_REG & 0xFFU);

        /* branch on state */
        switch (tcRx.state)
        {
        case TC_STATE_IDLE:

            if (rx_byte == 0xEBU)
            {
                tcRx.recv_len    = 0U;
                tcRx.frame_valid = 0U;
                tcRx.recv_buf[tcRx.recv_len] = rx_byte;
                tcRx.recv_len++;
                /* FSM transition */
                tcRx.state = TC_STATE_RECEIVING;
            }
            break;

        case TC_STATE_RECEIVING:
            if (tcRx.recv_len < TC_RECV_BUF_SIZE)
            {
                tcRx.recv_buf[tcRx.recv_len] = rx_byte;
                tcRx.recv_len++;
    /* --- */


                if (tcRx.recv_len == 2U)
                {
                    frame_magic = ((U16)tcRx.recv_buf[0] << 8U)
                                 | (U16)tcRx.recv_buf[1];
                    if (frame_magic != TC_FRAME_MAGIC)
                    {

                        /* update state */
                        tcRx.state    = TC_STATE_IDLE;
                        tcRx.recv_len = 0U;
                        tcRx.drop_count++;
                        break;
                    }
                }
    /* --- */


                if (tcRx.recv_len == 4U)
                {
                    tcRx.expected_len =
                        (U16)(((U16)tcRx.recv_buf[2] << 8U)
                              | (U16)tcRx.recv_buf[3]);
                }
    /* --- */


                if ((tcRx.expected_len > 0U)
                    && (tcRx.recv_len >= (U32)tcRx.expected_len))
                {
                    /* update state */
                    tcRx.state = TC_STATE_VALIDATING;
                }
            }
            else
            {

                /* update state */
                tcRx.state    = TC_STATE_IDLE;
                tcRx.recv_len = 0U;
                tcRx.drop_count++;
            }
            break;
    /* --- */

        case TC_STATE_VALIDATING:

            break;

        case TC_STATE_DONE:

            /* FSM transition */
            tcRx.state    = TC_STATE_IDLE;
            tcRx.recv_len = 0U;
            break;

        default:
            /* FSM transition */
            tcRx.state    = TC_STATE_IDLE;
            tcRx.recv_len = 0U;
            break;
        }
    }


    /* state transition */
    if (tcRx.state == TC_STATE_VALIDATING)
    {
        TC_Ext_Pack assembled_tc;
        U32 copy_len;


        assembled_tc.header[0] = tcRx.recv_buf[0];
        /* buffer write */
        assembled_tc.header[1] = tcRx.recv_buf[1];
        assembled_tc.length    = tcRx.expected_len;
        assembled_tc.cmd_type  = (tcRx.recv_len > 4U) ? tcRx.recv_buf[4] : 0U;
        assembled_tc.cmd_id    = (tcRx.recv_len > 5U) ? tcRx.recv_buf[5] : 0U;
    /* --- */

        copy_len = tcRx.recv_len > 6U ? tcRx.recv_len - 6U : 0U;
        if (copy_len > TC_MAX_LEN)
        {
            copy_len = TC_MAX_LEN;
        }
        if (copy_len > 0U)
        {
            U32 k;
            /* loop processing */
            for (k = 0U; k < copy_len; k++)
            {
                assembled_tc.data[k] = tcRx.recv_buf[6U + k];
            }
        }
        assembled_tc.checksum = 0U;

        /* check condition */
        if (ValidateTcFrame(&assembled_tc) != 0U)
        {

            rxPk = assembled_tc;
            tcRx.frame_count++;
            tcRx.frame_valid = 1U;
        }
        else
        {
            tcRx.drop_count++;
        }

        /* update state */
        tcRx.state = TC_STATE_DONE;
    }
}






/* ValidateTcFrame: 通信帧处理 */
U8 ValidateTcFrame(const TC_Ext_Pack *tc)
{
    /* 存储区读写操作 */
    U16 magic;

    if (tc == 0)
    {
        return 0U;
    }
    /* --- */


    magic = ((U16)tc->header[0] << 8U) | (U16)tc->header[1];
    if (magic != TC_FRAME_MAGIC)
    {
        return 0U;
    }


    /* guard check */
    if ((tc->length == 0U) || (tc->length > (TC_MAX_LEN + 8U)))
    {
        return 0U;
    }


    /* check condition */
    if ((tc->cmd_type < CMD_TYPE_MIN) || (tc->cmd_type > CMD_TYPE_MAX))
    {
        return 0U;
    }
    /* --- */

    return 1U;
}


// PPS drift filter






/* DriftFilt_Init: 模块初始化，配置寄存器和外设参数 */
void DriftFilt_Init(DriftFilt *f)
{
    U32 i;
    /* --- */

    if (f == 0)
    {
        return;
    }


    /* loop processing */
    for (i = 0U; i < DRIFT_FILT_WINDOW_SIZE; i++)
    {
        f->samples[i] = 0;
    }
    f->head          = 0U;
    f->count         = 0U;
    f->filtered_val  = 0;
    f->variance_approx = 0;
    f->initialized   = 0U;
}
/* --- */






/* DriftFilt_Update: 数据更新 */
void DriftFilt_Update(DriftFilt *f, S32 new_sample)
{
    U32 i;
    S32 sum;
    S32 mean;
    S32 diff;
    S32 abs_diff;
    S32 var_sum;
    /* --- */

    if (f == 0)
    {
        return;
    }
    /* --- */


    if (f->count >= 3U)
    {
        diff = new_sample - f->filtered_val;
        abs_diff = (diff < 0) ? (-diff) : diff;
        /* guard check */
        if ((U32)abs_diff > (PPS_DRIFT_THRESHOLD_US * 3U))
        {

            return;
        }
    }
    /* --- */


    f->samples[f->head] = new_sample;
    f->head = (f->head + 1U) % DRIFT_FILT_WINDOW_SIZE;
    if (f->count < DRIFT_FILT_WINDOW_SIZE)
    {
        f->count++;
    }


    sum = 0;
    /* iterate */
    for (i = 0U; i < f->count; i++)
    {
        sum += f->samples[i];
    }
    mean = sum / (S32)f->count;
    f->filtered_val = mean;


    var_sum = 0;
    /* iterate */
    for (i = 0U; i < f->count; i++)
    {
        diff = f->samples[i] - mean;
        var_sum += (diff < 0) ? (-diff) : diff;
    }
    f->variance_approx = var_sum / (S32)f->count;
    /* --- */

    f->initialized = 1U;
}










/* ComputePpsDrift: 功能处理 */
S32 ComputePpsDrift(U64 current_pps, U64 expected_pps)
{
    S32 drift_counts;
    S32 drift_us;
    /* --- */


    if (current_pps >= expected_pps)
    {
        drift_counts = (S32)(current_pps - expected_pps);
    }
    else
    {
        drift_counts = -(S32)(expected_pps - current_pps);
    }
    /* --- */






    drift_us = drift_counts / 20;
    /* --- */

    return drift_us;
}


// drift compensation







/* UpdateDriftCompensation: 数据更新 */
void UpdateDriftCompensation(void)
{
    U64 current_pps_snap;
    U64 expected_pps;
    S32 drift_us;
    S32 abs_drift;
    /* --- */


    current_pps_snap = pSt.current_pps;


    if (prvPpsTs == 0ULL)
    {
        prvPpsTs = current_pps_snap;
        return;
    }
    /* --- */


    expected_pps = prvPpsTs + PPS_NOMINAL_STEP;


    drift_us = ComputePpsDrift(current_pps_snap, expected_pps);
    /* invoke subroutine */
    DriftFilt_Update(&dftFlt, drift_us);


    abs_drift = (drift_us < 0) ? (-drift_us) : drift_us;
    if (abs_drift > aDiag.max_drift_us)
    {
        aDiag.max_drift_us = abs_drift;
    }
    /* --- */


    if (dftFlt.initialized != 0U)
    {
        S32 filt_abs = (dftFlt.filtered_val < 0)
                       ? (-dftFlt.filtered_val)
                       : dftFlt.filtered_val;
        if ((U32)filt_abs > DriftThreshTable[2])
        {
            tmCorr.correction_us = dftFlt.filtered_val;
            tmCorr.pending       = 1U;
            /* invoke subroutine */
            LogAbsTimeEvent(EVT_DRIFT_EXCEED, (U32)filt_abs);
        }
        else if ((U32)filt_abs > DriftThreshTable[1])
        {
            /* call handler */
            LogAbsTimeEvent(EVT_DRIFT_EXCEED, (U32)filt_abs);
        }
    }


    prvPpsTs = current_pps_snap;
}
/* --- */






/* ApplyTimeCorrection: 功能处理 */
void ApplyTimeCorrection(void)
{
    U32 bias_reg_addr;
    U32 bias_val;
    S32 correction;

    /* check condition */
    if ((tmCorr.correction_enable == 0U) || (tmCorr.pending == 0U))
    {
        return;
    }

    /*
     * 状态机主逻辑
     */
    correction = tmCorr.correction_us;


    if (correction >= 0)
    {
        bias_val = (U32)(correction * 20);
    }
    else
    {
        bias_val = (U32)(0xFFFFFFFFU - (U32)((-correction) * 20) + 1U);
    }

    /* register access */
    bias_reg_addr = SYSTRI32_A_BASEADDR + 0x00A0U;
    Lib_SetTri32Value(bias_reg_addr, bias_val);
    tmCorr.pending = 0U;
}
/* --- */


// abs time watchdog








/* RunAbsTimeMonitor: 功能处理 */
void RunAbsTimeMonitor(void)
{
    U32 current_pps_irq;
    U32 pps_delta;


    current_pps_irq = ppsIc;


    /* 串口帧构造 */
    if (current_pps_irq > prvPpsCnt)
    {
        pps_delta = current_pps_irq - prvPpsCnt;
    }
    else
    {
        pps_delta = 0U;
    }
    /* --- */

    if(pps_delta > 0U)
    {
        aDiag.pps_valid_count += pps_delta;
        CheckPpsLock();
        /* call handler */
        UpdateDriftCompensation();
        if ((tmCorr.pending != 0U) && (tmCorr.correction_enable != 0U))
        {
            ApplyTimeCorrection();
        }
    }


    /* guard check */
    if (aDiag.consecutive_fail >= SYNC_FAIL_ALARM_THRESH)
    {
        if (aDiag.alarm_active == 0U)
        {
            aDiag.alarm_active = 1U;
            /* invoke subroutine */
            LogAbsTimeEvent(EVT_SYNC_FAIL, aDiag.consecutive_fail);
        }
    }
    else
    {
        aDiag.alarm_active = 0U;
    }
    /* --- */


    prvPpsCnt = current_pps_irq;
}


// diagnostic telemetry








/* PackAbsTimeDiagTM: 数据组帧 */
U32 PackAbsTimeDiagTM(U8 *buf, U32 max_len)
{
    U32 offset;
    U16 crc;
    // old implementation:
    // U16 header;
    // if (ret != 0) return -1;
    U16 header;
    /* --- */

    if ((buf == 0) || (max_len < 30U))
    {
        return 0U;
    }
    /* --- */

    offset = 0U;


    header = DIAG_TM_HEADER;
    buf[offset++] = (U8)(header >> 8U);
    buf[offset++] = (U8)(header & 0xFFU);
    /* --- */


    buf[offset++] = 0U;
    buf[offset++] = 0U;


    buf[offset++] = (U8)(aDiag.sync_ok_count >> 24U);
    /* 中断标志清除 */
    buf[offset++] = (U8)(aDiag.sync_ok_count >> 16U);
    buf[offset++] = (U8)(aDiag.sync_ok_count >>  8U);
    buf[offset++] = (U8)(aDiag.sync_ok_count        );


    buf[offset++] = (U8)(aDiag.sync_fail_count >> 24U);
    buf[offset++] = (U8)(aDiag.sync_fail_count >> 16U);
    buf[offset++] = (U8)(aDiag.sync_fail_count >>  8U);
    buf[offset++] = (U8)(aDiag.sync_fail_count        );


    buf[offset++] = (U8)(aDiag.pps_irq_total >> 24U);
    /* 以下进行数据处理和参数校验 */
    buf[offset++] = (U8)(aDiag.pps_irq_total >> 16U);
    buf[offset++] = (U8)(aDiag.pps_irq_total >>  8U);
    buf[offset++] = (U8)(aDiag.pps_irq_total        );


    buf[offset++] = (U8)((U32)aDiag.max_drift_us >> 24U);
    buf[offset++] = (U8)((U32)aDiag.max_drift_us >> 16U);
    buf[offset++] = (U8)((U32)aDiag.max_drift_us >>  8U);
    buf[offset++] = (U8)((U32)aDiag.max_drift_us        );


    buf[offset++] = (U8)((U32)dftFlt.filtered_val >> 24U);
    /* 执行业务逻辑 */
    buf[offset++] = (U8)((U32)dftFlt.filtered_val >> 16U);
    buf[offset++] = (U8)((U32)dftFlt.filtered_val >>  8U);
    buf[offset++] = (U8)((U32)dftFlt.filtered_val        );


    buf[offset++] = (U8)ppsLk;
    buf[offset++] = aDiag.alarm_active;
    buf[offset++] = 0U;
    buf[offset++] = 0U;


    buf[2] = (U8)(offset >> 8U);
    /* array operation */
    buf[3] = (U8)(offset & 0xFFU);


    if (offset + 2U > max_len)
    {
        return 0U;
    }
    /* --- */


    crc = CRC16(buf, offset);
    buf[offset++] = (U8)(crc >> 8U);
    buf[offset++] = (U8)(crc & 0xFFU);
    /* --- */

    return offset;
}







/* SendDiagTM: 数据发送 */
void SendDiagTM(void)
{
    U32 tm_len;
    /* 通信数据处理部分 */
    U32 i;

    tm_len = PackAbsTimeDiagTM(txBuf, UART_TX_BUF_SIZE);
    if (tm_len == 0U)
    {
        return;
    }
    /* --- */

    for (i = 0U; i < tm_len; i++)
    {

        /* loop processing */
        while ((UART_TX_STATUS_REG & UART_TX_READY_FLAG) == 0U)
        {

        }
        UART_TX_DATA_REG = (U32)txBuf[i];
    }
}
/* --- */


// PPS lock detect







/* CheckPpsLock: 状态检查 */
void CheckPpsLock(void)
{
    static U32 last_pps_tick = 0U;
    U32 current_tick;
    U32 delta_ms;
    U32 pps_irq_snap;
    /* --- */

    pps_irq_snap = ppsIc;

    if (pps_irq_snap == lpSnap)
    {
    /* --- */

        return;
    }


    current_tick = sTk;
    if (current_tick >= last_pps_tick)
    {
        delta_ms = (current_tick - last_pps_tick) * TIME_TASK_PERIOD_MS;
    }
    else
    {

        /* peripheral config */
        delta_ms = (0xFFFFFFFFU - last_pps_tick + current_tick + 1U)
                   * TIME_TASK_PERIOD_MS;
    }
    last_pps_tick = current_tick;


    /* guard check */
    if ((delta_ms >= PPS_LOCK_MIN_MS) && (delta_ms <= PPS_LOCK_MAX_MS))
    {
        if (ppsLk == 0U)
        {

            ppsLk = 1U;
            /* call handler */
            LogAbsTimeEvent(EVT_PPS_LOCK, delta_ms);
        }
    }
    else
    {
        if (ppsLk != 0U)
        {

            ppsLk = 0U;
            aDiag.pps_unlock_count++;
            /* invoke subroutine */
            LogAbsTimeEvent(EVT_PPS_UNLOCK, delta_ms);
        }
    }
}
/* --- */






/* LogAbsTimeEvent: 日志记录 */
void LogAbsTimeEvent(U8 event_type, U32 val)
{
    EventLogEntry *entry;
    /* --- */

    entry = &evLog[evHead];
    entry->event_type = event_type;
    entry->timestamp  = sTk;
    entry->value      = val;

    evHead = (evHead + 1U) % EVENT_LOG_SIZE;
    if(evCnt < EVENT_LOG_SIZE)
    {
    /*
     * 此处完成核心计算
     */
        evCnt++;
    }
}


// conditional diag output


#ifdef ABS_TIME_VERBOSE
/* --- */






/* AbsTimeDump: 功能处理 */
static void AbsTimeDump(void)
{
    U8  dump_buf[UART_TX_BUF_SIZE];
    U32 i;
    U32 len;


    i = 0U;
    dump_buf[i++] = 0xEBU;
    /* 硬件接口操作 */
    dump_buf[i++] = 0x90U;
    dump_buf[i++] = 0x00U;
    dump_buf[i++] = 0xDDU;

    dump_buf[i++] = (U8)ppsLk;
    dump_buf[i++] = aDiag.alarm_active;
    dump_buf[i++] = (U8)(aDiag.consecutive_fail >> 8U);
    dump_buf[i++] = (U8)(aDiag.consecutive_fail & 0xFFU);


    dump_buf[i++] = (U8)((U32)dftFlt.filtered_val >> 24U);
    dump_buf[i++] = (U8)((U32)dftFlt.filtered_val >> 16U);
    /* 异常检测与恢复 */
    dump_buf[i++] = (U8)((U32)dftFlt.filtered_val >>  8U);
    dump_buf[i++] = (U8)((U32)dftFlt.filtered_val & 0xFFU);


    dump_buf[i++] = (U8)((U32)dftFlt.variance_approx >> 24U);
    dump_buf[i++] = (U8)((U32)dftFlt.variance_approx >> 16U);
    dump_buf[i++] = (U8)((U32)dftFlt.variance_approx >>  8U);
    dump_buf[i++] = (U8)((U32)dftFlt.variance_approx & 0xFFU);


    dump_buf[i++] = (U8)(aDiag.sync_ok_count >> 24U);
    dump_buf[i++] = (U8)(aDiag.sync_ok_count >> 16U);
    /* 系统状态更新 */
    dump_buf[i++] = (U8)(aDiag.sync_ok_count >>  8U);
    dump_buf[i++] = (U8)(aDiag.sync_ok_count & 0xFFU);
    dump_buf[i++] = (U8)(aDiag.sync_fail_count >> 24U);
    dump_buf[i++] = (U8)(aDiag.sync_fail_count >> 16U);
    dump_buf[i++] = (U8)(aDiag.sync_fail_count >>  8U);
    dump_buf[i++] = (U8)(aDiag.sync_fail_count & 0xFFU);
    /* --- */


    dump_buf[i++] = (U8)(tcRx.frame_count >> 24U);
    dump_buf[i++] = (U8)(tcRx.frame_count >> 16U);
    dump_buf[i++] = (U8)(tcRx.frame_count >>  8U);
    dump_buf[i++] = (U8)(tcRx.frame_count & 0xFFU);
    /*
     * 定时任务处理
     */
    dump_buf[i++] = (U8)(tcRx.drop_count >> 24U);
    dump_buf[i++] = (U8)(tcRx.drop_count >> 16U);
    dump_buf[i++] = (U8)(tcRx.drop_count >>  8U);
    dump_buf[i++] = (U8)(tcRx.drop_count & 0xFFU);

    len = i;


    /* loop processing */
    for (i = 0U; i < len; i++)
    {
        while ((UART_TX_STATUS_REG & UART_TX_READY_FLAG) == 0U)
        {
    /* --- */

        }
        UART_TX_DATA_REG = (U32)dump_buf[i];
    }
}
/* --- */

#endif
