/*------ star_time_sync.c  Satellite star time synchronization  SPARC/LEON3 ------*/
/*
 * Target  : LEON3 / SPARC V8 — satellite on-board star time management
 * System  : Multi-task star time sync with 1553B command interrupt
 * Board   : GR712RC evaluation / LEON3 compatible peripheral mapping
 * Origin  : VBUG00017073
 * Clock   : 4 MHz system clock, GPTIMER prescaled to 100 kHz
 * Desc    : Maintains floating-point star time and counter snapshot fields.
 *           Task1 periodically synchronizes star time; Ext6 ISR handles
 *           calibration commands that update time parameters.
 *           Provides drift diagnostics, sync history, telemetry packing
 *           and bus monitoring facilities for star time evaluation.
 *           Star time wraps at 86400.0 seconds (one sidereal day).
 *           Telemetry output via APBUART at low rate for ground debug.
 */

#include <stdint.h>
#include <string.h>

/*------ peripheral register bases (LEON3 AMBA) ------*/

/*------ SPARC LEON3 AMBA APB peripheral base addresses ------*/
#define APBUART_BASE            0x80000100U
#define GPTIMER_BASE            0x80000300U
#define IRQCTRL_BASE            0x80000200U
#define WDT_BASE                0x80000500U

/*------ GPTIMER register offsets ------*/
#define GPTIMER_SCALER          (*(volatile uint32_t *)(GPTIMER_BASE + 0x00U))
#define GPTIMER_RELOAD          (*(volatile uint32_t *)(GPTIMER_BASE + 0x04U))
#define GPTIMER_COUNTER         (*(volatile uint32_t *)(GPTIMER_BASE + 0x08U))
#define GPTIMER_CTRL            (*(volatile uint32_t *)(GPTIMER_BASE + 0x0CU))
#define GPTIMER_CTRL_EN_BIT     0x00000001U  /* timer enable bit            */
#define GPTIMER_CTRL_RL_BIT     0x00000002U  /* auto-reload bit             */
#define GPTIMER_CTRL_IE_BIT     0x00000004U  /* interrupt enable bit        */

/*------ interrupt controller registers ------*/
#define IRQCTRL_PEND            (*(volatile uint32_t *)(IRQCTRL_BASE + 0x04U))
#define IRQCTRL_MASK            (*(volatile uint32_t *)(IRQCTRL_BASE + 0x08U))
#define IRQCTRL_CLEAR           (*(volatile uint32_t *)(IRQCTRL_BASE + 0x0CU))

/*------ APBUART data / status ------*/
#define APBUART_DATA            (*(volatile uint32_t *)(APBUART_BASE + 0x00U))
#define APBUART_STATUS          (*(volatile uint32_t *)(APBUART_BASE + 0x04U))
#define APBUART_CTRL            (*(volatile uint32_t *)(APBUART_BASE + 0x08U))
#define APBUART_SCALER          (*(volatile uint32_t *)(APBUART_BASE + 0x0CU))
#define APBUART_TX_READY_MASK   0x00000004U   /* TX FIFO ready bit         */
#define APBUART_RX_READY_MASK   0x00000001U   /* RX data available bit     */

/*------ 1553B bus memory ------*/
#define BUSRAM_BASE             0x40000000U
#define RCV_1_1553_ADDR         (BUSRAM_BASE + 0x0000U)
#define RCV_2_1553_ADDR         (BUSRAM_BASE + 0x1000U)
#define BUSRAM_STATUS_WORD_OFF  0x0800U      /* status word offset          */

/*------ interrupt masks ------*/
#define IRQ_EXT6_MASK           0x00000040U  /* external interrupt 6 (1553B) */
#define IRQ_EXT5_MASK           0x00000020U  /* external interrupt 5         */
#define IRQ_TIMER_MASK          0x00000008U  /* timer interrupt              */

/*------ star time constants ------*/
#define STAR_TIMER_FREQ_HZ      100000U      /* star timer frequency 100kHz */
#define STAR_TIMER_PERIOD_US    10U          /* count period 10us           */
#define STAR_TIME_SCALE_F       0.00001      /* scale factor (sec/count)    */
#define MAX_STAR_TIME_SEC       86400.0      /* maximum star time (1 day)   */
#define STAR_TIME_WRAP_HALF     43200.0      /* half-day for wrap detect    */
#define CMU_TC_BUF_SIZE         32U          /* CMU command buffer size     */
#define PACK_BUF_SIZE           64U          /* data pack buffer size       */
#define TASK1_PERIOD_SEC        0.01         /* Task1 scheduling period (s) */

/*------ time sync command identifiers ------*/
#define CMD_TIME_SYNC_ID        0x5A5AU      /* calibration command ID      */
#define CMD_ADDR_CMU            0x01U        /* CMU address                 */

/*------ diagnostics and sync history constants ------*/
#define WDT_KICK_VALUE          0xDEADBEEFUL /* watchdog kick magic value   */
#define SYNC_HIST_BUF_SIZE      8U           /* sync history ring size      */
#define DRIFT_ALARM_THRESHOLD   0.5          /* drift alarm threshold (sec) */
#define TM_SEND_PERIOD          50U          /* TM send period (loop count) */
#define TM_FRAME_MAGIC          0xEB90U      /* telemetry frame header      */
#define TM_FRAME_MAX_LEN        128U         /* max telemetry frame length  */
#define SYNC_EVAL_PERIOD        100U         /* quality evaluation period   */
#define SYNC_SM_FAULT_WAIT      5U           /* FAULT state wait rounds     */
#define SYNC_SM_DRIFT_CNT_MAX   3U           /* consecutive drift threshold */
#define CMD_ID_RANGE_MAX        0xFFFFU      /* max valid command ID        */
#define CMD_FRAME_MIN_LEN       4U           /* min CMU frame length        */
#define CRC16_POLY              0x1021U      /* CRC16-CCITT polynomial      */
#define STAR_TIME_VALID_MIN     0.0          /* star time valid lower bound */
#define STAR_TIME_VALID_MAX     86400.0      /* star time valid upper bound */
#define DIAG_HIST_MAX_ENTRIES   8U           /* drift history ring capacity */
#define UART_DIAG_BUF_SIZE      64U          /* UART diag buffer size       */

/*------ type aliases ------*/
typedef uint8_t  U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;
typedef int32_t  S32;
typedef float    float32;
typedef double   float64;

/*------ struct type definitions ------*/
/*       All structs use explicit padding for SPARC alignment rules.       */
/*       Volatile qualifier on ISR-shared fields per LEON3 ABI.            */

/* 1553B command word */
typedef struct {
    struct {
        U16  Address   : 5;   /* RT address        */
        U16  TR_bit    : 1;   /* transmit/receive  */
        U16  SubAddr   : 5;   /* sub-address       */
        U16  WordCount : 5;   /* word count        */
    } Bit;
    U16 Raw;                  /* raw command word   */
} Command_Word;

/* star time synchronization state                                         */
/* Multiple tightly coupled fields shared between Task1 and Ext6 ISR.      */
/* Fields marked volatile are written by ISR and read by main task.         */
typedef struct {
    volatile float64  StarTime;              /* star time float value (sec) */
    volatile U32      timeupdata_new;        /* latest counter snapshot     */
    volatile U32      timeupdata_new_NLS;    /* NLS counter snapshot        */
    volatile U32      timeupdata_old;        /* previous counter snapshot   */
    volatile U32      timeupdata_int;        /* ISR-side shadow snapshot    */
    volatile U8       cmdtimeflag;           /* calibration command flag    */
    volatile U8       SyncDone;              /* sync-done flag              */
    U16               Reserved;              /* alignment padding           */
} StarTimeState;

/* CMU calibration command data */
typedef struct {
    U16  cmdtime[3];   /* calibration data: [0]=high,[1]=mid,[2]=low (ms)  */
    U16  pad;
} CmdTimeData;

/* data packing state (Task2 exclusive, no ISR contention) */
typedef struct {
    volatile float64  StarTime1;            /* packed star time (corrected) */
    volatile U8       flg05s;               /* 0.5s offset flag            */
    U8                pad[3];
    U32               temp_time1;           /* time high word              */
    U16               temp_time2;           /* time low word               */
    U16               pad2;
} PackState;

/* task control block */
typedef struct {
    U8    task_id;
    U8    running;
    U16   period_ms;
    U32   exec_count;
    U32   last_tick;
} TaskCtrlBlock;

/* time quality grade enumeration */
typedef enum {
    TIME_Q_GOOD    = 0U,   /* good: small drift, frequent sync             */
    TIME_Q_FAIR    = 1U,   /* fair: moderate drift or longer sync interval  */
    TIME_Q_POOR    = 2U,   /* poor: large drift or long unsynchronized      */
    TIME_Q_INVALID = 3U    /* invalid: no sync completed or SM fault        */
} TimeQuality_e;

/* star time diagnostic statistics                                         */
/* Tracks sync/calib counts and maintains drift ring buffer for quality     */
/* evaluation.  Updated only from main task context.                        */
typedef struct {
    U32     sync_count;                         /* cumulative sync count    */
    U32     calib_count;                        /* cumulative calib count   */
    float64 max_drift;                          /* historical max drift (s) */
    float64 drift_hist[DIAG_HIST_MAX_ENTRIES];  /* recent drift ring buf   */
    U8      drift_hist_idx;                     /* drift history write ptr  */
    U8      pad[3];
} StarTimeDiag;

/* sync history record */
typedef struct {
    U32     timestamp_tick;    /* record moment SysTick                     */
    float64 star_time_snap;   /* StarTime snapshot at record moment         */
    float64 delta_sec;        /* delta this cycle (sec)                     */
    U8      is_calibrated;    /* whether triggered by calibration (1=yes)   */
    U8      pad[3];
} SyncHistory;

/* telemetry packet (packed into 16-byte aligned frame for downlink) */
typedef struct {
    U16  frame_head;       /* frame header magic 0xEB90                     */
    U16  seq_num;          /* frame sequence number                         */
    U32  star_time_high;   /* star time high word (integer seconds)         */
    U16  star_time_low;    /* star time low word (sub-second, ms)           */
    U8   time_quality;     /* time quality grade                            */
    U8   sync_count_lsb;   /* sync count low byte                          */
    U16  checksum;         /* checksum                                      */
    U8   padding[4];       /* pad to 16-byte alignment                     */
} TmPacket;

/* bus monitor structure */
typedef struct {
    U32  recv_frame_cnt;     /* received frame count                        */
    U32  parse_fail_cnt;     /* parse failure count                         */
    U32  last_recv_tick;     /* SysTick at last frame reception              */
    U8   alarm_flag;         /* timeout alarm flag                          */
    U8   pad[3];
} BusMonitor;

/*------ constant configuration tables ------*/
/*       ROM-resident lookup tables and configuration constants.           */
/*       Placed in .rodata section by LEON3 linker script.                 */

/* 1553B channel base address table */
static const U32 Chan1553_BaseAddr[2] = {
    RCV_1_1553_ADDR,
    RCV_1_1553_ADDR + 0x1000U
};

/* star time scale factor lookup (for different precision modes) */
static const float64 StarTimeScaleTable[4] = {
    0.00001,    /* standard: 10us count  */
    0.000001,   /* high:     1us count   */
    0.0001,     /* low:      100us count */
    0.00005     /* medium:   50us count  */
};

/* pack output buffer size table */
static const U32 PackBufSizeTable[3] = {32U, 64U, 128U};

/* bus timeout threshold (SysTick periods) */
static const U32 BusTimeoutTicks = 200U;

/* drift evaluation weight coefficients (recent samples weighted higher) */
static const float64 DriftWeightTable[DIAG_HIST_MAX_ENTRIES] = {
    0.05, 0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25
};

/*------ global variables (including volatile shared variables) ------*/
/*       Variables accessed from both main task and ISR contexts are        */
/*       declared volatile.  No SPARC atomic guarantee on multi-word        */
/*       accesses; higher-level protocol required for consistency.          */

/* core shared star time state (Task1 and Ext6 ISR both access) */
static volatile StarTimeState  StarState;

/* calibration command data (ISR writes, task reads) */
static volatile CmdTimeData    CmdTime;

/* data packing state (Task2 exclusive) */
static volatile PackState      PkState;

/* CMU command receive buffer (ISR fills, task reads) */
static volatile U16            cmu_tc[CMU_TC_BUF_SIZE];

/* 1553B command word (ISR reads) */
static volatile Command_Word   Command_OBDH;

/* task control blocks */
static TaskCtrlBlock           Task1_Ctrl;
static TaskCtrlBlock           Task2_Ctrl;

/* system tick (incremented each scheduling cycle) */
static volatile U32            SysTick = 0U;

/* data transmit buffer */
static U8                      SendBuf[PACK_BUF_SIZE];

/* system initialization done flag */
static U8                      InitDone = 0U;

/* star time diagnostics (main task only) */
static StarTimeDiag            star_diag;

/* sync history ring buffer */
static SyncHistory             sync_hist[SYNC_HIST_BUF_SIZE];

/* sync history write pointer */
static U8                      sync_hist_idx = 0U;

/* bus monitor data */
static BusMonitor              bus_mon;

/* current time quality grade */
static TimeQuality_e           time_quality = TIME_Q_INVALID;

/* SysTick at last sync (for drift calculation) */
static U32                     last_sync_tick = 0U;

/* StarTime baseline at last calibration (for drift expectation) */
static float64                 calib_base_startime = 0.0;

/* SysTick at last calibration (for drift expectation) */
static U32                     calib_base_tick = 0U;

/* UART diagnostic frame transmit buffer */
static U8                      uart_diag_buf[UART_DIAG_BUF_SIZE];

/* telemetry frame sequence number */
static U16                     tm_seq_num = 0U;

/*------ forward declarations ------*/
/*       Initialization, task, ISR and utility function prototypes.        */

void SysInit(void);
void TimerHwInit(void);
void Bus1553Init(void);
void StarTimeStateInit(void);
void TaskCtrlInit(void);

void LoopTask1(void);
void LoopTask2(void);
void SynStarTime(void);
void CMU_cmd_respond(void);
void Pack_PK_Data(void);
void pack_data(void);

void SYS_ISRExt6(void);
void CmdUnpack(void);
U32  Lib_GetStarTimer(void);
void ReadData16To16(volatile U16 *dst, U32 src_addr, U32 count);
void DisableExt6IRQ(void);
void EnableExt6IRQ(void);
U16  CalcPackChecksum(const U8 *buf, U32 len);

/* diagnostics and star time management forward declarations */
void DiagInit(void);
void RecordSyncHistory(float64 new_star_time, float64 delta, U8 is_calibrated);
void EvalTimeQuality(void);
void ComputeStarTimeDrift(void);
void RunStarTimeSyncSM(void);
U32  PackTmFrame(U8 *buf, U32 max_len);
void SendTmViaTxBuf(const U8 *buf, U32 len);
void RunBusMonitor(void);
U8   ValidateCmuFrame(const volatile U16 *buf, U32 len);
U16  CalcCRC16_Star(const U8 *data, U32 len);
void WatchdogFeed(void);

#ifdef STAR_TIME_VERBOSE
static void StarTimeDump(void);
#endif

int  main(void);

/*------ main entry point ------*/
/*       Two-task cooperative scheduler with ISR-driven 1553B input.       */
/*       Task1 runs every tick; Task2 every 5th tick; CMU check every 10th.*/

int main(void)
{
    /* system initialization */
    SysInit();

    /* main loop (simulates dual-task concurrent scheduling) */
    while (1)
    {
        SysTick++;

        /* Task1: star time synchronization (high priority) */
        LoopTask1();

        /* Task2: data packing (lower priority) */
        if ((SysTick % 5U) == 0U)
        {
            LoopTask2();
        }

        /* process CMU calibration command response */
        if ((SysTick % 10U) == 0U)
        {
            CMU_cmd_respond();
        }

        /* bus monitor (every main loop iteration) */
        RunBusMonitor();

        /* feed hardware watchdog each iteration */
        WatchdogFeed();
    }

    return 0;
}

/*------ initialization function group ------*/
/*       Hardware peripherals, 1553B bus, star time state, task blocks.    */

/*------ SysInit: top-level system initialization entry ------*/
void SysInit(void)
{
    TimerHwInit();
    Bus1553Init();
    StarTimeStateInit();
    TaskCtrlInit();
    DiagInit();
    InitDone = 1U;
}

/*------ TimerHwInit: configure GPTIMER as star time counter ------*/
void TimerHwInit(void)
{
    /* set prescaler (counter runs at 100kHz, assuming 4MHz sysclk) */
    GPTIMER_SCALER = (U32)(40U - 1U);
    /* enable timer, free-running mode */
    GPTIMER_CTRL   = 0x00000005U;
}

/*------ Bus1553Init: initialize 1553B bus controller interface ------*/
/*       Sets up command word, clears receive buffer, enables Ext6 IRQ.    */
void Bus1553Init(void)
{
    U32 i;

    /* clear CMU command buffer */
    for (i = 0U; i < CMU_TC_BUF_SIZE; i++)
    {
        cmu_tc[i] = 0U;
    }

    /* initialize command word */
    Command_OBDH.Raw    = 0U;
    Command_OBDH.Bit.Address   = 0U;
    Command_OBDH.Bit.TR_bit    = 0U;
    Command_OBDH.Bit.SubAddr   = 0U;
    Command_OBDH.Bit.WordCount = 0U;

    /* enable external interrupt 6 (1553B) */
    IRQCTRL_MASK |= IRQ_EXT6_MASK;
}

/*------ StarTimeStateInit: initialize star time state struct ------*/
void StarTimeStateInit(void)
{
    StarState.StarTime           = 0.0;
    StarState.timeupdata_new     = 0U;
    StarState.timeupdata_new_NLS = 0U;
    StarState.timeupdata_old     = 0U;
    StarState.timeupdata_int     = 0U;
    StarState.cmdtimeflag        = 0U;
    StarState.SyncDone           = 0U;
    StarState.Reserved           = 0U;

    CmdTime.cmdtime[0] = 0U;
    CmdTime.cmdtime[1] = 0U;
    CmdTime.cmdtime[2] = 0U;

    PkState.StarTime1  = 0.0;
    PkState.flg05s     = 0U;
    PkState.temp_time1 = 0U;
    PkState.temp_time2 = 0U;
}

/*------ TaskCtrlInit: initialize task control blocks ------*/
/*       Task1 (sync, 10ms) and Task2 (pack, 50ms) control blocks.        */
void TaskCtrlInit(void)
{
    Task1_Ctrl.task_id    = 1U;
    Task1_Ctrl.running    = 0U;
    Task1_Ctrl.period_ms  = 10U;
    Task1_Ctrl.exec_count = 0U;
    Task1_Ctrl.last_tick  = 0U;

    Task2_Ctrl.task_id    = 2U;
    Task2_Ctrl.running    = 0U;
    Task2_Ctrl.period_ms  = 50U;
    Task2_Ctrl.exec_count = 0U;
    Task2_Ctrl.last_tick  = 0U;

    /* initialize transmit buffer */
    {
        U32 i;
        for (i = 0U; i < PACK_BUF_SIZE; i++)
        {
            SendBuf[i] = 0U;
        }
    }
}

/*------ call-chain layer-1 functions (called directly from main) ------*/
/*       LoopTask1: high priority star time sync                           */
/*       LoopTask2: lower priority telemetry packing                       */

/*------ LoopTask1: star time sync and CMU response ------*/
void LoopTask1(void)
{
    float64  snap_star_time;
    float64  delta_sec;
    U32      exec_mod;

    Task1_Ctrl.exec_count++;
    Task1_Ctrl.last_tick = SysTick;
    Task1_Ctrl.running   = 1U;

    /* snapshot star time before sync for delta calculation */
    snap_star_time = StarState.StarTime;

    /* star time synchronization (layer-2 call) */
    SynStarTime();

    /* compute this sync delta */
    delta_sec = StarState.StarTime - snap_star_time;
    if (delta_sec < 0.0)
    {
        /* star time wrapped around, correct delta */
        delta_sec += MAX_STAR_TIME_SEC;
    }

    /* update diagnostic sync count */
    star_diag.sync_count++;

    /* record this sync event in history */
    RecordSyncHistory(StarState.StarTime, delta_sec, 0U);

    /* update last sync tick */
    last_sync_tick = SysTick;

    /* run star time sync state machine */
    RunStarTimeSyncSM();

    /* periodically evaluate time quality */
    exec_mod = Task1_Ctrl.exec_count % SYNC_EVAL_PERIOD;
    if (exec_mod == 0U)
    {
        EvalTimeQuality();
        ComputeStarTimeDrift();
    }

    Task1_Ctrl.running = 0U;
}

/*------ LoopTask2: telemetry data packing ------*/
void LoopTask2(void)
{
    U32 tm_len;
    U32 exec_mod;

    Task2_Ctrl.exec_count++;
    Task2_Ctrl.last_tick = SysTick;
    Task2_Ctrl.running   = 1U;

    /* data packing (layer-2 call) */
    pack_data();

    /* pack and send TM frame every TM_SEND_PERIOD cycles */
    exec_mod = Task2_Ctrl.exec_count % TM_SEND_PERIOD;
    if (exec_mod == 0U)
    {
        tm_len = PackTmFrame(uart_diag_buf, UART_DIAG_BUF_SIZE);
        if (tm_len > 0U)
        {
            SendTmViaTxBuf(uart_diag_buf, tm_len);
        }
    }

    Task2_Ctrl.running = 0U;
}

/*------ call-chain layer-2 functions ------*/

/*------ SynStarTime: read counter delta and update StarTime ------*/
/*       Reads GPTIMER, computes elapsed ticks, accumulates StarTime.      */
/*       call chain layer-2: main -> LoopTask1 -> SynStarTime              */
void SynStarTime(void)
{
    U32 T;
    U32 cur_new;

    /* save previous count value as old reference */
    StarState.timeupdata_old = StarState.timeupdata_new;   /* read timeupdata_new */

    /* obtain latest star timer count, update timeupdata_new and NLS */
    cur_new = Lib_GetStarTimer();
    StarState.timeupdata_new     = cur_new;                /* write timeupdata_new */
    StarState.timeupdata_new_NLS = Lib_GetStarTimer();     /* write timeupdata_new_NLS */

    /* compute time delta (old - new, counter counts down) */
    if (StarState.timeupdata_old >= StarState.timeupdata_new)
    {
        T = StarState.timeupdata_old - StarState.timeupdata_new;
    }
    else
    {
        /* overflow wraparound handling */
        T = (0xFFFFFFFFU - StarState.timeupdata_new) + StarState.timeupdata_old + 1U;
    }

    /* update star time */
    StarState.StarTime = StarState.StarTime + (float64)T * STAR_TIME_SCALE_F;

    /* star time wraparound handling */
    if (StarState.StarTime >= MAX_STAR_TIME_SEC)
    {
        StarState.StarTime -= MAX_STAR_TIME_SEC;
    }

    /* mark synchronization completed for this cycle */
    StarState.SyncDone = 1U;
}

/*------ pack_data: top-level data packing (layer-2) ------*/
/*       Invokes Pack_PK_Data then serializes result to SendBuf.           */
/*       call chain layer-2: main -> LoopTask2 -> pack_data                */
void pack_data(void)
{
    /* pack telemetry data (layer-3 call) */
    Pack_PK_Data();

    /* write packed result into transmit buffer */
    SendBuf[0] = (U8)((PkState.temp_time1 >> 24U) & 0xFFU);
    SendBuf[1] = (U8)((PkState.temp_time1 >> 16U) & 0xFFU);
    SendBuf[2] = (U8)((PkState.temp_time1 >> 8U)  & 0xFFU);
    SendBuf[3] = (U8)(PkState.temp_time1 & 0xFFU);
    SendBuf[4] = (U8)((PkState.temp_time2 >> 8U)  & 0xFFU);
    SendBuf[5] = (U8)(PkState.temp_time2 & 0xFFU);
}

/*------ call-chain layer-3 functions ------*/
/*       Functions at the deepest nesting level in the call graph.         */

/*------ Pack_PK_Data: generate time fields in TM packet ------*/
/*       Accesses shared StarTime and counter snapshots.  Subject to       */
/*       potential inconsistency if ISR modifies fields mid-read.          */
/*       call chain: main -> LoopTask2 -> pack_data -> Pack_PK_Data        */
void Pack_PK_Data(void)
{
    U32 dt;
    float64 st1;

    /* compute frame time correction (using timeupdata_new and NLS) */
    if (StarState.timeupdata_new >= StarState.timeupdata_new_NLS)
    {
        dt = StarState.timeupdata_new - StarState.timeupdata_new_NLS;
    }
    else
    {
        dt = (0xFFFFFFFFU - StarState.timeupdata_new_NLS) + StarState.timeupdata_new + 1U;
    }

    /* frame star time = current star time + correction */
    st1 = StarState.StarTime + (float64)dt / (float64)STAR_TIMER_FREQ_HZ;

    /* 0.5s offset correction */
    if (PkState.flg05s != 0U)
    {
        st1 -= 0.5;
        PkState.flg05s = 0U;
    }

    PkState.StarTime1 = st1;

    /* encode as hex time words */
    PkState.temp_time1 = (U32)(st1 * 1000.0 / 65536.0);
    PkState.temp_time2 = (U16)((U32)(st1 * 1000.0) -
                                (U32)((float64)PkState.temp_time1 * 65536.0));
}

/*------ CMU_cmd_respond: handle calibration command response ------*/
/*       When cmdtimeflag is set by CmdUnpack (ISR context), this          */
/*       function reconstructs absolute star time from 3-word CMU data     */
/*       and resets StarTime directly.  Records calibration baseline       */
/*       for subsequent drift evaluation.                                  */
/*       Called from main loop, cooperates with ISR -> CmdUnpack.          */
void CMU_cmd_respond(void)
{
    if (StarState.cmdtimeflag == 0U)
    {
        return;
    }

    StarState.cmdtimeflag = 0U;

    /* reset star time from calibration command (direct write to StarTime) */
    StarState.StarTime = (float64)CmdTime.cmdtime[0] * 65536.0
                       + (float64)CmdTime.cmdtime[1]
                       + (float64)CmdTime.cmdtime[2] * 0.001;

    /* record calibration baseline for later drift calculation */
    calib_base_startime = StarState.StarTime;
    calib_base_tick     = SysTick;

    /* feed watchdog after calibration update completes */
    WatchdogFeed();

    /* mark sync completed after calibration */
    StarState.SyncDone = 1U;
}

/*------ interrupt service routines ------*/
/*       Ext6 is wired to the 1553B bus controller receive-complete        */
/*       interrupt line.  SPARC LEON3 auto-vectors to registered handler.  */

/*------ SYS_ISRExt6: external interrupt 6, 1553B receive done ------*/
/*       Reads command word from BUSRAM, dispatches by RT address.         */
/*       For CMU address: reads 32 words into cmu_tc[], calls CmdUnpack.   */
void SYS_ISRExt6(void)
{
    /* clear interrupt pending flag in IRQ controller */
    IRQCTRL_CLEAR = IRQ_EXT6_MASK;

    /* read command word from bus shared RAM */
    Command_OBDH.Raw = (U16)(*((volatile U32 *)BUSRAM_BASE));

    /* extract RT address field (bits 15:11) */
    Command_OBDH.Bit.Address = (U16)(Command_OBDH.Raw >> 11U) & 0x1FU;

    switch (Command_OBDH.Bit.Address)
    {
        case CMD_ADDR_CMU:
            /* read 32 words from 1553B channel 0 into CMU buffer */
            ReadData16To16(cmu_tc, Chan1553_BaseAddr[0], 32U);
            /* parse CMU command frame */
            CmdUnpack();
            /* update bus monitor receive frame count and timestamp */
            bus_mon.recv_frame_cnt++;
            bus_mon.last_recv_tick = SysTick;
            break;

        default:
            break;
    }
}

/*------ CmdUnpack: parse CMU frame in Ext6 interrupt context ------*/
/*       Validates frame header (first 4 words non-zero), then             */
/*       dispatches by frame ID.  For CMD_TIME_SYNC_ID: records star       */
/*       timer snapshot into shadow variable timeupdata_int, extracts      */
/*       3-word calibration payload, and sets cmdtimeflag for main task.   */
/*       call chain layer-2: SYS_ISRExt6 -> CmdUnpack                     */
void CmdUnpack(void)
{
    U32 i;
    U16 frame_id;

    /* frame header accumulative check (simplified) */
    for (i = 0U; i < 4U; i++)
    {
        if (cmu_tc[i] == 0U)
        {
            bus_mon.parse_fail_cnt++;
            return;
        }
    }

    /* classify processing by frame ID */
    frame_id = cmu_tc[1];

    switch (frame_id)
    {
        case CMD_TIME_SYNC_ID:
            StarState.timeupdata_int = Lib_GetStarTimer();  /* write timeupdata_int */

            /* extract calibration data */
            CmdTime.cmdtime[0] = cmu_tc[2];  /* high 16 bits */
            CmdTime.cmdtime[1] = cmu_tc[3];  /* mid  16 bits */
            CmdTime.cmdtime[2] = cmu_tc[4];  /* low  16 bits (ms) */

            /* set calibration flag (checked by CMU_cmd_respond in main task) */
            StarState.cmdtimeflag = 1U;

            /* update diagnostic calibration count */
            star_diag.calib_count++;

            /* record calibration trigger tick */
            last_sync_tick = SysTick;

            break;

        default:
            break;
    }
}

/*------ helper functions ------*/

/*------ Lib_GetStarTimer: read GPTIMER current count ------*/
U32 Lib_GetStarTimer(void)
{
    return GPTIMER_COUNTER;
}

/*------ ReadData16To16: read 16-bit word array from 1553B bus memory ------*/
void ReadData16To16(volatile U16 *dst, U32 src_addr, U32 count)
{
    U32 i;
    volatile U16 *src = (volatile U16 *)src_addr;

    for (i = 0U; i < count; i++)
    {
        dst[i] = src[i];
    }
}

/*------ DisableExt6IRQ: mask external interrupt 6 (critical section) ------*/
void DisableExt6IRQ(void)
{
    IRQCTRL_MASK &= ~IRQ_EXT6_MASK;
}

/*------ EnableExt6IRQ: unmask external interrupt 6 ------*/
void EnableExt6IRQ(void)
{
    IRQCTRL_MASK |= IRQ_EXT6_MASK;
}

/*------ CalcPackChecksum: compute packet checksum ------*/
U16 CalcPackChecksum(const U8 *buf, U32 len)
{
    U32 i;
    U16 sum = 0U;

    for (i = 0U; i < len; i++)
    {
        sum += (U16)buf[i];
    }
    return sum;
}

/*------ WatchdogFeed: kick hardware watchdog via MMIO ------*/
void WatchdogFeed(void)
{
    *(volatile uint32_t *)0x80000510UL = 0xDEADBEEFUL;
}

/*------ diagnostic initialization ------*/

/*------ DiagInit: initialize all diagnostic data structures ------*/
/*       Called from SysInit() to ensure known state before run.    */
void DiagInit(void)
{
    U32 i;

    /* initialize star time diagnostic statistics */
    star_diag.sync_count    = 0U;
    star_diag.calib_count   = 0U;
    star_diag.max_drift     = 0.0;
    star_diag.drift_hist_idx = 0U;

    for (i = 0U; i < DIAG_HIST_MAX_ENTRIES; i++)
    {
        star_diag.drift_hist[i] = 0.0;
    }

    /* initialize sync history ring buffer */
    sync_hist_idx = 0U;
    for (i = 0U; i < SYNC_HIST_BUF_SIZE; i++)
    {
        sync_hist[i].timestamp_tick  = 0U;
        sync_hist[i].star_time_snap  = 0.0;
        sync_hist[i].delta_sec       = 0.0;
        sync_hist[i].is_calibrated   = 0U;
    }

    /* initialize bus monitor */
    bus_mon.recv_frame_cnt  = 0U;
    bus_mon.parse_fail_cnt  = 0U;
    bus_mon.last_recv_tick  = 0U;
    bus_mon.alarm_flag      = 0U;

    /* initialize time quality and related state */
    time_quality      = TIME_Q_INVALID;
    last_sync_tick    = 0U;
    calib_base_startime = 0.0;
    calib_base_tick   = 0U;
    tm_seq_num        = 0U;

    /* clear UART diagnostic buffer */
    for (i = 0U; i < UART_DIAG_BUF_SIZE; i++)
    {
        uart_diag_buf[i] = 0U;
    }
}

/*------ sync history recording ------*/

/*------ RecordSyncHistory: write sync event to ring buffer ------*/
/*       Saves SysTick, StarTime snapshot, delta and calib flag.   */
/*       Write pointer advances circularly up to SYNC_HIST_BUF.   */
/*                                                                 */
/*       new_star_time  -- StarTime after this sync (sec)          */
/*       delta          -- delta this sync cycle (sec)             */
/*       is_calibrated  -- calibration-triggered (1=yes, 0=no)    */
void RecordSyncHistory(float64 new_star_time, float64 delta, U8 is_calibrated)
{
    U8 idx;

    /* current write pointer position */
    idx = sync_hist_idx;

    /* write history entry */
    sync_hist[idx].timestamp_tick = SysTick;
    sync_hist[idx].star_time_snap = new_star_time;
    sync_hist[idx].delta_sec      = delta;
    sync_hist[idx].is_calibrated  = is_calibrated;

    /* advance ring write pointer */
    idx++;
    if (idx >= (U8)SYNC_HIST_BUF_SIZE)
    {
        idx = 0U;
    }
    sync_hist_idx = idx;
}

/*------ time quality evaluation ------*/

/*------ EvalTimeQuality: assess star time quality grade ------*/
/*       Examines sync history and system state to set grade.  */
/*       Dimensions:                                           */
/*         1. recent delta deviations vs alarm threshold       */
/*         2. sufficient consecutive sync count                */
/*         3. SysTick gap since last sync                      */
void EvalTimeQuality(void)
{
    U32     i;
    U32     good_cnt;
    U32     tick_gap;
    float64 abs_delta;
    float64 max_recent_delta;

    /* if no sync ever completed, mark invalid */
    if (star_diag.sync_count == 0U)
    {
        time_quality = TIME_Q_INVALID;
        return;
    }

    /* find maximum absolute delta in recent history */
    max_recent_delta = 0.0;
    good_cnt = 0U;

    for (i = 0U; i < SYNC_HIST_BUF_SIZE; i++)
    {
        if (sync_hist[i].timestamp_tick == 0U)
        {
            /* slot not yet written, skip */
            continue;
        }

        abs_delta = sync_hist[i].delta_sec;
        if (abs_delta < 0.0)
        {
            abs_delta = -abs_delta;
        }

        if (abs_delta > max_recent_delta)
        {
            max_recent_delta = abs_delta;
        }

        if (abs_delta < DRIFT_ALARM_THRESHOLD)
        {
            good_cnt++;
        }
    }

    /* compute SysTick gap since last sync */
    if (SysTick >= last_sync_tick)
    {
        tick_gap = SysTick - last_sync_tick;
    }
    else
    {
        tick_gap = (0xFFFFFFFFU - last_sync_tick) + SysTick + 1U;
    }

    /* set quality grade based on evaluation results */
    if ((good_cnt >= 6U) && (tick_gap < BusTimeoutTicks))
    {
        time_quality = TIME_Q_GOOD;
    }
    else if ((good_cnt >= 4U) && (tick_gap < (BusTimeoutTicks * 2U)))
    {
        time_quality = TIME_Q_FAIR;
    }
    else if (good_cnt >= 2U)
    {
        time_quality = TIME_Q_POOR;
    }
    else
    {
        time_quality = TIME_Q_INVALID;
    }
}

/*------ star time drift computation ------*/

/*------ ComputeStarTimeDrift: compute drift vs calibration baseline ------*/
/*       expected = calib StarTime + (SysTick - calib_base_tick) * period  */
/*       Drift stored into star_diag ring buffer; max drift updated.       */
void ComputeStarTimeDrift(void)
{
    U32     tick_elapsed;
    float64 expected_star_time;
    float64 actual_star_time;
    float64 drift;
    float64 abs_drift;
    U8      hist_idx;

    /* if no calibration occurred, cannot compute baseline drift */
    if (star_diag.calib_count == 0U)
    {
        return;
    }

    /* compute SysTick elapsed since last calibration */
    if (SysTick >= calib_base_tick)
    {
        tick_elapsed = SysTick - calib_base_tick;
    }
    else
    {
        tick_elapsed = (0xFFFFFFFFU - calib_base_tick) + SysTick + 1U;
    }

    /* expected star time = calib baseline + elapsed ticks * seconds/tick */
    /* Task1 executes every SysTick, period approx 10ms */
    expected_star_time = calib_base_startime
                         + (float64)tick_elapsed * 0.01;

    /* read current star time */
    actual_star_time = StarState.StarTime;

    /* compute drift (with wraparound correction) */
    drift = actual_star_time - expected_star_time;
    if (drift > (MAX_STAR_TIME_SEC / 2.0))
    {
        drift -= MAX_STAR_TIME_SEC;
    }
    else if (drift < -(MAX_STAR_TIME_SEC / 2.0))
    {
        drift += MAX_STAR_TIME_SEC;
    }

    abs_drift = drift;
    if (abs_drift < 0.0)
    {
        abs_drift = -abs_drift;
    }

    /* update historical maximum drift */
    if (abs_drift > star_diag.max_drift)
    {
        star_diag.max_drift = abs_drift;
    }

    /* write into drift history ring buffer */
    hist_idx = star_diag.drift_hist_idx;
    star_diag.drift_hist[hist_idx] = drift;
    hist_idx++;
    if (hist_idx >= (U8)DIAG_HIST_MAX_ENTRIES)
    {
        hist_idx = 0U;
    }
    star_diag.drift_hist_idx = hist_idx;
}

/*------ star time synchronization state machine ------*/

/*------ RunStarTimeSyncSM: 4-phase time sync state machine ------*/
/*       Called each LoopTask1 iteration.                          */
/*       Phases:                                                   */
/*         TSYNC_PHASE_WAIT  -- await first SyncDone               */
/*         TSYNC_PHASE_CHECK -- validate StarTime in legal range   */
/*         TSYNC_PHASE_RUN   -- normal operation, monitor drift    */
/*         TSYNC_PHASE_ERR   -- error recovery, clear and restart  */
/*       Uses function-local static variables for state tracking.  */
void RunStarTimeSyncSM(void)
{
    /* phase enumeration (local scope) */
    typedef enum {
        TSYNC_PHASE_WAIT  = 0U,
        TSYNC_PHASE_CHECK = 1U,
        TSYNC_PHASE_RUN   = 2U,
        TSYNC_PHASE_ERR   = 3U
    } TsyncPhase_e;

    static TsyncPhase_e  phase       = TSYNC_PHASE_WAIT;
    static U32           drift_cnt   = 0U;
    static U32           err_wait    = 0U;

    float64 cur_star_time;
    float64 cur_drift;
    float64 abs_drift;

    cur_star_time = StarState.StarTime;

    switch (phase)
    {
        /*------ WAIT: await first sync completion ------*/
        case TSYNC_PHASE_WAIT:
            if (StarState.SyncDone != 0U)
            {
                phase     = TSYNC_PHASE_CHECK;
                drift_cnt = 0U;
            }
            break;

        /*------ CHECK: validate star time legality ------*/
        case TSYNC_PHASE_CHECK:
            if ((cur_star_time >= STAR_TIME_VALID_MIN) &&
                (cur_star_time <  STAR_TIME_VALID_MAX))
            {
                phase     = TSYNC_PHASE_RUN;
                drift_cnt = 0U;
            }
            else
            {
                /* star time out of range, enter error phase */
                phase    = TSYNC_PHASE_ERR;
                err_wait = 0U;
            }
            break;

        /*------ RUN: normal operation, monitor drift ------*/
        case TSYNC_PHASE_RUN:
            /* get most recent drift history entry */
            {
                U8 prev_idx;
                if (star_diag.drift_hist_idx == 0U)
                {
                    prev_idx = (U8)(DIAG_HIST_MAX_ENTRIES - 1U);
                }
                else
                {
                    prev_idx = star_diag.drift_hist_idx - 1U;
                }
                cur_drift = star_diag.drift_hist[prev_idx];
            }

            abs_drift = cur_drift;
            if (abs_drift < 0.0)
            {
                abs_drift = -abs_drift;
            }

            if (abs_drift > DRIFT_ALARM_THRESHOLD)
            {
                drift_cnt++;
                if (drift_cnt >= SYNC_SM_DRIFT_CNT_MAX)
                {
                    /* consecutive drift exceeded, wait for calibration */
                    if (star_diag.calib_count > 0U)
                    {
                        phase     = TSYNC_PHASE_CHECK;
                        drift_cnt = 0U;
                    }
                    else
                    {
                        phase    = TSYNC_PHASE_ERR;
                        err_wait = 0U;
                    }
                }
            }
            else
            {
                /* drift within threshold, reset counter */
                drift_cnt = 0U;
            }
            break;

        /*------ ERR: wait then clear star time, restart ------*/
        case TSYNC_PHASE_ERR:
            err_wait++;
            if (err_wait >= SYNC_SM_FAULT_WAIT)
            {
                /* clear star time, await re-synchronization */
                StarState.StarTime  = 0.0;
                StarState.SyncDone  = 0U;
                phase     = TSYNC_PHASE_WAIT;
                err_wait  = 0U;
                drift_cnt = 0U;
            }
            break;

        default:
            phase = TSYNC_PHASE_WAIT;
            break;
    }
}

/*------ telemetry frame packing ------*/

/*------ PackTmFrame: pack star time and diagnostics into TM frame ------*/
/*       Frame format:                                                    */
/*         bytes 0-1  : header magic 0xEB90 (big-endian)                  */
/*         bytes 2-3  : sequence number (big-endian)                      */
/*         bytes 4-7  : star time high (integer sec*1000, BE 32-bit)      */
/*         bytes 8-9  : star time low  (sub-sec ms, BE 16-bit)            */
/*         byte  10   : time quality grade                                */
/*         byte  11   : sync count low byte                               */
/*         byte  12   : calib count low byte                              */
/*         byte  13   : alarm flag                                        */
/*         bytes 14-15: CRC16 over bytes 0-13                             */
/*                                                                        */
/*       buf     -- output buffer pointer                                 */
/*       max_len -- buffer capacity (bytes)                               */
/*       returns : actual bytes written; 0 if buffer too small            */
U32 PackTmFrame(U8 *buf, U32 max_len)
{
    U32  star_time_ms;
    U32  star_time_high;
    U16  star_time_low;
    U16  crc_val;
    float64 st;

    /* fixed frame length is 16 bytes */
    if (max_len < 16U)
    {
        return 0U;
    }

    /* read current star time (single read to avoid inconsistency) */
    st = StarState.StarTime;

    /* convert star time to milliseconds integer */
    star_time_ms   = (U32)(st * 1000.0);
    star_time_high = star_time_ms / 65536U;
    star_time_low  = (U16)(star_time_ms % 65536U);

    /* write frame header magic (big-endian) */
    buf[0] = (U8)((TM_FRAME_MAGIC >> 8U) & 0xFFU);
    buf[1] = (U8)(TM_FRAME_MAGIC & 0xFFU);

    /* write sequence number (big-endian) */
    buf[2] = (U8)((tm_seq_num >> 8U) & 0xFFU);
    buf[3] = (U8)(tm_seq_num & 0xFFU);
    tm_seq_num++;

    /* write star time high word (big-endian 32-bit) */
    buf[4] = (U8)((star_time_high >> 24U) & 0xFFU);
    buf[5] = (U8)((star_time_high >> 16U) & 0xFFU);
    buf[6] = (U8)((star_time_high >> 8U)  & 0xFFU);
    buf[7] = (U8)(star_time_high & 0xFFU);

    /* write star time low word (big-endian 16-bit) */
    buf[8] = (U8)((star_time_low >> 8U) & 0xFFU);
    buf[9] = (U8)(star_time_low & 0xFFU);

    /* write time quality grade */
    buf[10] = (U8)time_quality;

    /* write sync count low byte */
    buf[11] = (U8)(star_diag.sync_count & 0xFFU);

    /* write calib count low byte */
    buf[12] = (U8)(star_diag.calib_count & 0xFFU);

    /* write alarm flag */
    buf[13] = bus_mon.alarm_flag;

    /* compute CRC16 over bytes 0-13 */
    crc_val = CalcCRC16_Star(buf, 14U);
    buf[14] = (U8)((crc_val >> 8U) & 0xFFU);
    buf[15] = (U8)(crc_val & 0xFFU);

    return 16U;
}

/*------ UART telemetry transmit ------*/

/*------ SendTmViaTxBuf: send TM frame via APBUART byte-by-byte ------*/
/*       Polls TX FIFO ready bit before each byte write.               */
/*       May block on large payloads; used for low-rate diag output.   */
/*                                                                     */
/*       buf -- data buffer pointer                                    */
/*       len -- byte count to send                                     */
void SendTmViaTxBuf(const U8 *buf, U32 len)
{
    U32 i;
    U32 wait_cnt;

    for (i = 0U; i < len; i++)
    {
        /* wait for TX FIFO ready (poll up to 1000 times to avoid hang) */
        wait_cnt = 0U;
        while ((APBUART_STATUS & APBUART_TX_READY_MASK) == 0U)
        {
            wait_cnt++;
            if (wait_cnt >= 1000U)
            {
                /* timeout, abort this byte and remaining */
                return;
            }
        }

        /* write to data register, triggers transmission */
        APBUART_DATA = (U32)buf[i];
    }
}

/*------ bus monitor ------*/

/*------ RunBusMonitor: check 1553B bus receive status ------*/
/*       Compares SysTick gap vs BusTimeoutTicks threshold.  */
/*       Sets alarm flag on timeout; tracks parse fail rate.  */
void RunBusMonitor(void)
{
    U32 tick_gap;

    /* compute gap since last frame reception */
    if (SysTick >= bus_mon.last_recv_tick)
    {
        tick_gap = SysTick - bus_mon.last_recv_tick;
    }
    else
    {
        tick_gap = (0xFFFFFFFFU - bus_mon.last_recv_tick) + SysTick + 1U;
    }

    /* set alarm if gap exceeds threshold */
    if ((tick_gap > BusTimeoutTicks) && (bus_mon.last_recv_tick != 0U))
    {
        bus_mon.alarm_flag = 1U;
    }
    else
    {
        bus_mon.alarm_flag = 0U;
    }

    /* diagnostics: if parse fail rate exceeds 10%, degrade quality */
    if ((bus_mon.recv_frame_cnt > 10U) &&
        (bus_mon.parse_fail_cnt > (bus_mon.recv_frame_cnt / 10U)))
    {
        if (time_quality == TIME_Q_GOOD)
        {
            time_quality = TIME_Q_FAIR;
        }
    }
}

/*------ CMU frame validation ------*/

/*------ ValidateCmuFrame: check CMU frame format validity ------*/
/*       Validation rules:                                       */
/*         1. first 4 words all non-zero (header present)        */
/*         2. frame ID within legal range (nonzero, <= max)      */
/*         3. buffer length >= CMD_FRAME_MIN_LEN                 */
/*                                                               */
/*       buf -- CMU frame buffer (16-bit word array)             */
/*       len -- valid word count in buffer                       */
/*       returns: 1 = valid, 0 = invalid                        */
U8 ValidateCmuFrame(const volatile U16 *buf, U32 len)
{
    U32 i;
    U16 frame_id;

    /* check minimum length */
    if (len < CMD_FRAME_MIN_LEN)
    {
        return 0U;
    }

    /* check first 4 words non-zero */
    for (i = 0U; i < CMD_FRAME_MIN_LEN; i++)
    {
        if (buf[i] == 0U)
        {
            return 0U;
        }
    }

    /* check frame ID validity */
    frame_id = buf[1];
    if (frame_id == 0U)
    {
        return 0U;
    }

    /* check frame ID does not exceed upper limit */
    if ((U32)frame_id > (U32)CMD_ID_RANGE_MAX)
    {
        return 0U;
    }

    return 1U;
}

/*------ CRC16 computation ------*/

/*------ CalcCRC16_Star: CRC16-CCITT over data buffer ------*/
/*       Polynomial 0x1021, init 0xFFFF, bit-by-bit.         */
/*       No lookup table, suitable for SPARC targets.        */
/*                                                           */
/*       data -- data pointer                                */
/*       len  -- data length (bytes)                         */
/*       returns: 16-bit CRC value                           */
U16 CalcCRC16_Star(const U8 *data, U32 len)
{
    U32 i;
    U32 bit;
    U16 crc;
    U8  byte_val;

    crc = 0xFFFFU;

    for (i = 0U; i < len; i++)
    {
        byte_val = data[i];

        /* process bit-by-bit (MSB first) */
        for (bit = 0U; bit < 8U; bit++)
        {
            if (((U16)(crc ^ (U16)((U16)byte_val << 8U)) & 0x8000U) != 0U)
            {
                crc = (U16)((crc << 1U) ^ CRC16_POLY);
            }
            else
            {
                crc = (U16)(crc << 1U);
            }
            byte_val = (U8)(byte_val << 1U);
        }
    }

    return crc;
}

/*------ conditional compilation diagnostic output ------*/

#ifdef STAR_TIME_VERBOSE

/*------ StarTimeDump: output star time diagnostics via UART ------*/
/*       Outputs current StarTime (integer sec), sync/calib counts, */
/*       time quality grade, and recent drift history sign+magnitude */
/*       as hex byte sequence for ground tool parsing.               */
/*       Only linked when STAR_TIME_VERBOSE macro is defined.        */
static void StarTimeDump(void)
{
    U8  dump_buf[32];
    U32 st_sec;
    U32 idx;
    U16 crc_dump;

    /* integer seconds of star time */
    st_sec = (U32)StarState.StarTime;

    /* build diagnostic output frame */
    dump_buf[0]  = 0xDBU;                             /* diag frame tag    */
    dump_buf[1]  = 0x01U;                             /* version           */
    dump_buf[2]  = (U8)((st_sec >> 24U) & 0xFFU);    /* StarTime high     */
    dump_buf[3]  = (U8)((st_sec >> 16U) & 0xFFU);
    dump_buf[4]  = (U8)((st_sec >> 8U)  & 0xFFU);
    dump_buf[5]  = (U8)(st_sec & 0xFFU);              /* StarTime low      */
    dump_buf[6]  = (U8)(star_diag.sync_count  & 0xFFU);
    dump_buf[7]  = (U8)(star_diag.calib_count & 0xFFU);
    dump_buf[8]  = (U8)time_quality;
    dump_buf[9]  = bus_mon.alarm_flag;

    /* write sign+magnitude bytes for last 4 drift history entries */
    for (idx = 0U; idx < 4U; idx++)
    {
        float64 dval;
        U8 sign_byte;
        U8 mag_byte;

        dval = star_diag.drift_hist[idx];
        if (dval < 0.0)
        {
            sign_byte = 0x01U;
            dval = -dval;
        }
        else
        {
            sign_byte = 0x00U;
        }
        /* drift magnitude encoding: unit 0.01s, saturate at 255 */
        mag_byte = (dval * 100.0 > 255.0) ? 0xFFU : (U8)(dval * 100.0);

        dump_buf[10U + idx * 2U]       = sign_byte;
        dump_buf[10U + idx * 2U + 1U]  = mag_byte;
    }

    /* CRC16 over bytes 0-17 */
    crc_dump = CalcCRC16_Star(dump_buf, 18U);
    dump_buf[18] = (U8)((crc_dump >> 8U) & 0xFFU);
    dump_buf[19] = (U8)(crc_dump & 0xFFU);

    /* send 20-byte diagnostic frame via UART */
    SendTmViaTxBuf(dump_buf, 20U);
}

#endif /* STAR_TIME_VERBOSE */
