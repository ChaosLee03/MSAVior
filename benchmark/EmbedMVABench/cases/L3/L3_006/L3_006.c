/*------ attitude_calc.c  Satellite attitude calculation  SPARC/LEON3 ------*/
/*                                                                          */
/* Module  : Attitude computation with quaternion coordinate transform      */
/* Target  : LEON3FT GR712RC  (SPARC V8)                                   */
/* System  : Multi-task attitude manager with external interrupt update     */
/* Board   : GR712RC eval / LEON3 compatible peripheral mapping             */
/* Origin  : VBUG00026839                                                   */
/* Desc    : Maintains Auxi_Result_1 struct across task and ISR contexts.   */
/*           Main task reads quaternion, angular velocity and timestamp     */
/*           through a 3-level call chain.  ExtInt3 ISR writes all eight   */
/*           members after receiving a new measurement frame.               */
/*           Includes health monitoring, omega filter, telemetry packing   */
/*           and watchdog feed through hardware MMIO at 0x80000520.        */
/* Clock   : 40 MHz system clock, GPTIMER prescaled for 10ms period        */
/* Date    : 2020-03-22   Rev 2.4                                          */
/*--------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
/* --- */

/*------ SPARC LEON3 interrupt controller registers ------------------------*/
#define IRQCTRL_BASE            0x80000200U
/*------*/
#define IRQCTRL_LEVEL           (*(volatile uint32_t *)(IRQCTRL_BASE + 0x00U))
#define IRQCTRL_PEND            (*(volatile uint32_t *)(IRQCTRL_BASE + 0x04U))
#define IRQCTRL_FORCE           (*(volatile uint32_t *)(IRQCTRL_BASE + 0x08U))
#define IRQCTRL_CLEAR           (*(volatile uint32_t *)(IRQCTRL_BASE + 0x0CU))
#define IRQCTRL_MASK            (*(volatile uint32_t *)(IRQCTRL_BASE + 0x40U))
/* --- */

/*------ interrupt mask bits -----------------------------------------------*/
#define IRQ_EXTINT3_MASK        0x00000008U   /* external interrupt 3      */
#define IRQ_TIMER_MASK          0x00000100U   /* timer interrupt           */

/*------ coordinate transform constants ------------------------------------*/
#define PI_F                    3.14159265f
#define DEG2RAD_F               0.01745329f   /* pi / 180                  */
#define RAD2DEG_F               57.2957795f   /* 180 / pi                  */
/* --- */

/*------ mode constants ----------------------------------------------------*/
#define MODE_INIT               0x00U         /* initialization mode       */
#define MODE_CAPTURE            0x01U         /* capture mode              */
#define MODE_TRACK              0x02U         /* tracking mode             */
#define MODE_C                  0x03U         /* precision tracking mode   */
/* --- */

/*------ buffer and iteration constants ------------------------------------*/
#define REC_RESULT_BUF_NUM      4U            /* recognition result depth  */
#define COORD_TRANSFORM_ITER    3U            /* coordinate transform iter */
#define OMEGA_MAX               10.0f         /* max angular rate (rad/s)  */
#define QUAT_NORM_THRESHOLD     0.99f         /* quaternion norm threshold */
/* --- */

/*------ timing constants --------------------------------------------------*/
#define TASK_PERIOD_15MS        15U           /* task period 15 ms         */
#define STARTIME_MAX            0x7FFFFFFFFFFFFFFFLL  /* max star time     */

/*------ watchdog timer register -------------------------------------------*/
#define WDT_FEED_ADDR           0x80000520UL  /* watchdog kick address     */
#define WDT_FEED_VALUE          0x1234UL      /* watchdog kick magic value */
/* --- */

/*------ attitude state machine phases -------------------------------------*/
#define ATT_SM_PHASE_COUNT      3U            /* 3-phase state machine     */
#define ATT_NORM_FAULT_THR      5U            /* consecutive norm failures */
#define ATT_NORM_LOW_BOUND      0.95f         /* lower norm bound for SM   */
#define ATT_NORM_HIGH_BOUND     1.05f         /* upper norm bound for SM   */
/* --- */

/*------ type aliases (SPARC standard) -------------------------------------*/
typedef uint8_t   U8;
typedef uint16_t  U16;
typedef uint32_t  U32;
typedef int32_t   S32;
typedef int64_t   S64;
typedef float     float32;
typedef double    float64;
typedef S64       llong64;
/* --- */

/*------ SPARC volatile register type --------------------------------------*/
typedef volatile uint32_t  vuint32;

/*------ quaternion structure (nested in SRec_Result) ----------------------*/
/*                                                                          */
/* Corresponds to: typedef struct QUATERNION { float32 q[4]; } SQuater;    */
/*------*/
typedef struct {
    float32   q[4];   /* q[0]=q1, q[1]=q2, q[2]=q3, q[3]=q4 (scalar)     */
} SQuater;
/* --- */

/*------ recognition result record (core shared type) ----------------------*/
/*                                                                          */
/* typedef struct REC_RESULT { SQuater Q; float32 omega[3]; ... };         */
/*------*/
typedef struct {
    SQuater   Q;          /* attitude quaternion (shared member 1)         */
    float32   omega[3];   /* angular velocity vector (shared member 2..4) */
    llong64   Startime;   /* image timestamp (shared member 5)            */
} SRec_Result;
/* --- */

/*------ Euler angle structure ---------------------------------------------*/
typedef struct {
    float32   psi;    /* yaw angle   (rad)                                */
    float32   theta;  /* pitch angle (rad)                                */
    float32   phi;    /* roll angle  (rad)                                */
} SEulerAngle;
/* --- */

/*------ 3x3 coordinate transform matrix -----------------------------------*/
typedef struct {
    float32   m[3][3];   /* transform matrix elements                     */
} SMatrix3x3;
/* --- */

/*------ attitude control output structure ---------------------------------*/
typedef struct {
    float32    torque[3];    /* control torque (N*m)                       */
    float32    err_angle[3]; /* attitude error angle (rad)                 */
    U8         ctrl_valid;   /* control valid flag                         */
    U8         reserved[3];
} SAttCtrlOut;
/* --- */

/*------ task state control block ------------------------------------------*/
typedef struct {
    U8    task_id;         /* task identifier                              */
    U8    running;         /* running flag                                 */
    U8    mode_cur;        /* current attitude mode                        */
    U8    mode_req;        /* requested mode switch                        */
    U32   exec_cnt;        /* execution counter                            */
    U32   last_tick;       /* last execution tick                          */
    U32   overrun_cnt;     /* overrun counter                              */
} STaskCtrl;
/* --- */

/*------ constant configuration tables -------------------------------------*/

/*------ installation matrix (inertial to body frame) ----------------------*/
static const float32 InstallMatrix[3][3] = {
    {  1.0f,  0.0f,  0.0f },
    {  0.0f,  1.0f,  0.0f },
    {  0.0f,  0.0f,  1.0f }
};
/* --- */

/*------ mode decision threshold table -------------------------------------*/
static const float32 ModeThreshTable[4] = {
    0.0f,    /* INIT                                                       */
    0.05f,   /* CAPTURE: error angle > 0.05 rad triggers entry             */
    0.01f,   /* TRACK:   error angle > 0.01 rad                            */
    /* data processing and validation */
    0.001f   /* MODE_C:  high precision                                    */
};

/*------ angular velocity weight coefficients ------------------------------*/
static const float32 OmegaWeight[3] = {1.0f, 1.0f, 1.0f};

/*------ global variables (including volatile shared variables) -------------*/

/*                                                                          */
/* Core shared variable: auxiliary recognition result 1                     */
/* Reader side: main -> ModeManager -> ModeJudge_1 -> CoordTransform       */
/* Writer side: ExtInt3_coordinate (direct multi-member write)              */
/*------*/
SRec_Result  Auxi_Result_1;
/* --- */

/*                                                                          */
/* Recognition result ring buffer (ISR uses mRec_Result_3[0] as source)    */
/* Corresponds to: Auxi_Result_1.Startime = mRec_Result_3[0].Startime;    */
/*------*/
static volatile SRec_Result  mRec_Result_3[REC_RESULT_BUF_NUM];
/* --- */

/*------ ISR-side coordinate mode switch flags -----------------------------*/
static volatile U8           AuxiToModeC_INT_1;
static volatile U8           AuxiToModeC_1;

/*------ ISR-side mode hold flags ------------------------------------------*/
static volatile U8           AppHold_ISR_1;
static volatile U8           AppMode_ISR_1;
/* --- */

/*------ coordinate transform temp matrix (ISR side) -----------------------*/
static float32               tempAPt[3][3];

/*------ current application mode ------------------------------------------*/
static volatile U8           AppMode_1;
/* --- */

/*------ attitude control output -------------------------------------------*/
static SAttCtrlOut           att_ctrl_out;

/*------ current Euler angles ----------------------------------------------*/
static volatile SEulerAngle  euler_cur;
/* --- */

/*------ coordinate transform matrix (task side) ---------------------------*/
static SMatrix3x3            trans_mat;

/*------ system tick counter -----------------------------------------------*/
static volatile U32          SysTick;
/* --- */

/*------ task 1 control block ----------------------------------------------*/
static STaskCtrl             Task1_Ctrl;

/*------ system initialization complete flag -------------------------------*/
static U8                    InitDone;
/* --- */

/*------ spinlock simulation (SPARC LDSTUB style) --------------------------*/
static volatile U32          SYS_Spinlock_MAP_1;

/*------ watchdog feed counter (reset on each WatchdogFeed call) -----------*/
static volatile U32          wdt_feed_cnt;
/* --- */

/*------ attitude calculation state machine phase variable -----------------*/
static volatile U8           att_calc_mode;

/*------ coordinate error accumulator (SM fault tracking) ------------------*/
static volatile U32          coord_err_total;
/* --- */

/*------ attitude health monitoring structure ------------------------------*/
typedef struct {
    U32  quat_invalid_cnt;    /* quaternion invalid counter               */
    U32  omega_overflow_cnt;  /* angular rate overflow counter            */
    U32  time_stale_cnt;      /* timestamp stale counter                  */
    U32  mode_switch_cnt;     /* mode switch counter                      */
    U32  total_check_cnt;     /* total check counter                      */
    U8   health_flag;         /* health flag: 0=fault, 1=normal           */
} SAttHealth;
/* --- */

static SAttHealth  att_health_stat;

/*------ angular velocity sliding window filter ----------------------------*/
typedef struct {
    float32  buf[3][8];   /* 8-point sliding window, 3 axes independent   */
    U8       idx;         /* current write position (ring index)          */
    U8       filled;      /* number of filled data points                 */
} SOmegaFilter;
/* --- */

static SOmegaFilter omega_filt_ctx;

/*------ telemetry frame macros and structure -------------------------------*/
#define TM_ATT_BASE_ADDR    0x60000000U   /* telemetry base address        */
#define TM_ATT_LEN          32U           /* telemetry frame length (bytes)*/
/* --- */

#define TIMER1_BASE         0x80000300U
#define TIMER1_CNT          (*(volatile U32 *)(TIMER1_BASE + 0x00U))
#define TIMER1_CTRL         (*(volatile U32 *)(TIMER1_BASE + 0x08U))

#define ATT_UART_BASE       0x80001000U   /* diagnostic UART base address  */
#define ATT_UART_TX_REG     (*(volatile U32 *)(ATT_UART_BASE + 0x04U))
#define ATT_UART_STAT_REG   (*(volatile U32 *)(ATT_UART_BASE + 0x08U))
#define ATT_UART_TX_READY   0x00000004U   /* TX register empty flag        */
/* --- */

/* 数据类型定义 */
typedef struct {
    U16      sync;          /* frame sync word 0xEB90                      */
    U8       seq;           /* frame sequence number                       */
    U8       mode;          /* current attitude mode                       */
    float32  q[4];          /* quaternion q[0..3]                          */
    float32  omega[3];      /* angular rate (rad/s)                        */
    U32      startime_hi;   /* timestamp upper 32 bits                     */
    U32      startime_lo;   /* timestamp lower 32 bits                     */
    U16      health;        /* health word                                 */
    U16      crc;           /* CRC16 checksum                              */
} STmAttFrame;
/* --- */

static STmAttFrame  tm_att_pkt;
static U8           tm_seq_att;

/*------ timer1 interrupt tick counter -------------------------------------*/
static volatile U32  timer1_tick;
/* --- */

/*------ forward declarations ----------------------------------------------*/

void SysInit(void);
void IrqCtrlInit(void);
void AttDataInit(void);
void TaskCtrlInit(void);
/* --- */

void ModeManager(void);
void ModeJudge_1(void);
void CoordTransform(void);
void CalcEulerFromQuat(const SRec_Result *res, SEulerAngle *euler);
void CalcAttCtrl(const SEulerAngle *euler, SAttCtrlOut *ctrl);
void UpdateAttMatrix(const SQuater *q, SMatrix3x3 *mat);
void ModeTransition(U8 new_mode);

/* execute business logic */
void ExtInt3_coordinate(void);
void C2QF(float32 *q_out, float32 *mat_in);

float32 QuatNorm(const SQuater *q);
void QuatNormalize(SQuater *q);
void QuatMultiply(const SQuater *a, const SQuater *b, SQuater *out);
float32 Fabs_f(float32 x);
float32 Sqrt_f(float32 x);
float32 Asin_f(float32 x);
float32 Atan2_f(float32 y, float32 x);

void DisableExtInt3(void);
void EnableExtInt3(void);
void BSP_SpinlockApply(volatile U32 *lock);
void BSP_SpinlockRelease(volatile U32 *lock);

/*------ attitude health monitoring module ---------------------------------*/
void AttHealthInit(void);
void RunAttHealthCheck(const SRec_Result *res);
void ResetAttHealth(void);

/*------ angular velocity low-pass filter module ---------------------------*/
void OmegaFiltInit(void);
void OmegaFiltUpdate(const float32 *omega_in, float32 *omega_out);

/*------ telemetry packing and transmission module -------------------------*/
static U16 CalcCRC16_Att(const U8 *buf, U32 len);
void PackAttTmFrame(const SRec_Result *res);
void SendAttTm(void);

/*------ timer initialization and ISR --------------------------------------*/
void Timer1Init(void);
void Timer1_ISR(void);

/*------ diagnostic output (conditional compilation) -----------------------*/
static void AttSendByte(U8 byte);
static void AttSendBuf(const U8 *buf, U32 len);
#ifdef ATT_VERBOSE
static void AttSysDump(void);
#endif

/*------ watchdog feed -----------------------------------------------------*/
void WatchdogFeed(void);

/*------ attitude calculation state machine --------------------------------*/
void RunAttCalcSM(void);

int main(void);

/*------ main entry point --------------------------------------------------*/

int main(void)
{
    /* system initialization */
    SysInit();

    /* initial watchdog feed after boot */
    WatchdogFeed();

    /* main loop (15 ms scheduling period) */
    while (1)
    {
        SysTick++;

        /* mode management task (call chain level 1) */
        ModeManager();

        /* run attitude state machine each cycle */
        RunAttCalcSM();

        /* feed watchdog each main loop iteration */
        WatchdogFeed();
    }

    return 0;
}

/*------ initialization functions ------------------------------------------*/

/*------ SysInit: system master initialization -----------------------------*/
/*       Calls all subsystem init functions in sequence.                    */
/*       Order: IRQ -> data -> task -> health -> filter -> timer.           */
void SysInit(void)
{
    IrqCtrlInit();
    AttDataInit();
    TaskCtrlInit();
    /* 调用子函数 */
    AttHealthInit();
    OmegaFiltInit();
    Timer1Init();
    tm_seq_att = 0U;
    wdt_feed_cnt = 0U;
    /* 状态切换 */
    att_calc_mode = 0U;
    coord_err_total = 0U;
    InitDone = 1U;
}

/*------ IrqCtrlInit: initialize SPARC interrupt controller ----------------*/
/*       Clears pending, sets ExtInt3 high priority and enables it.        */
void IrqCtrlInit(void)
{
    /* clear all pending interrupts */
    IRQCTRL_CLEAR = 0xFFFFFFFFU;

    /* configure interrupt priority (ExtInt3 = high) */
    IRQCTRL_LEVEL = IRQ_EXTINT3_MASK;

    /* enable external interrupt 3 */
    IRQCTRL_MASK |= IRQ_EXTINT3_MASK;
}

/*------ AttDataInit: initialize attitude data structures ------------------*/
/*       Zeros shared struct, ISR buffers, matrices and flags.             */
void AttDataInit(void)
{
    U32 i, j;

    /* initialize shared structure Auxi_Result_1 */
    Auxi_Result_1.Q.q[0] = 0.0f;
    Auxi_Result_1.Q.q[1] = 0.0f;
    Auxi_Result_1.Q.q[2] = 0.0f;
    Auxi_Result_1.Q.q[3] = 1.0f;   /* unit quaternion */
    /* 数组赋值 */
    Auxi_Result_1.omega[0] = 0.0f;
    Auxi_Result_1.omega[1] = 0.0f;
    Auxi_Result_1.omega[2] = 0.0f;
    Auxi_Result_1.Startime  = 0LL;

    /* initialize ISR-side data buffers */
    for (i = 0U; i < REC_RESULT_BUF_NUM; i++)
    {
        /* 数组赋值 */
        mRec_Result_3[i].Q.q[0] = 0.0f;
        mRec_Result_3[i].Q.q[1] = 0.0f;
        mRec_Result_3[i].Q.q[2] = 0.0f;
        mRec_Result_3[i].Q.q[3] = 1.0f;
        /* 数据填充 */
        mRec_Result_3[i].omega[0] = 0.0f;
        mRec_Result_3[i].omega[1] = 0.0f;
        mRec_Result_3[i].omega[2] = 0.0f;
        mRec_Result_3[i].Startime = 0LL;
    }

    /* initialize coordinate transform matrices */
    for (i = 0U; i < 3U; i++)
    {
        /* 遍历处理 */
        for (j = 0U; j < 3U; j++)
        {
            tempAPt[i][j]      = (i == j) ? 1.0f : 0.0f;
            trans_mat.m[i][j]  = (i == j) ? 1.0f : 0.0f;
        }
    }

    /* initialize Euler angles */
    euler_cur.psi   = 0.0f;
    euler_cur.theta = 0.0f;
    euler_cur.phi   = 0.0f;

    /* initialize control output */
    att_ctrl_out.torque[0]    = 0.0f;
    att_ctrl_out.torque[1]    = 0.0f;
    att_ctrl_out.torque[2]    = 0.0f;
    att_ctrl_out.err_angle[0] = 0.0f;
    att_ctrl_out.err_angle[1] = 0.0f;
    /* 数据填充 */
    att_ctrl_out.err_angle[2] = 0.0f;
    att_ctrl_out.ctrl_valid   = 0U;

    /* initialize flags */
    AuxiToModeC_INT_1 = 0U;
    AuxiToModeC_1     = 0U;
    AppHold_ISR_1     = 0U;
    AppMode_ISR_1     = MODE_INIT;
    AppMode_1         = MODE_INIT;

    SYS_Spinlock_MAP_1 = 0U;
}

/*------ TaskCtrlInit: initialize task control block -----------------------*/
/*       Resets task counters and tick baseline to zero.                    */
void TaskCtrlInit(void)
{
    Task1_Ctrl.task_id     = 1U;
    Task1_Ctrl.running     = 0U;
    Task1_Ctrl.mode_cur    = MODE_INIT;
    Task1_Ctrl.mode_req    = MODE_INIT;
    Task1_Ctrl.exec_cnt    = 0U;
    Task1_Ctrl.last_tick   = 0U;
    Task1_Ctrl.overrun_cnt = 0U;

    SysTick = 0U;
}

/*------ call chain level 1: main -> ModeManager ---------------------------*/

/*------ ModeManager: attitude mode management main task -------------------*/
/*       Corresponds to original Apptask_1.                                */
/*       Call chain: main -> ModeManager                                    */
/*       Wrapper layer added to satisfy >= 3 level call chain requirement. */
/*       Increments execution counter and delegates to ModeJudge_1.        */
/*------*/
void ModeManager(void)
{
    Task1_Ctrl.exec_cnt++;
    Task1_Ctrl.last_tick = SysTick;
    Task1_Ctrl.running   = 1U;

    /* mode judgment (level 2 call, may exceed 15 ms execution) */
    ModeJudge_1();

    Task1_Ctrl.running = 0U;
}

/*------ call chain level 2: ModeManager -> ModeJudge_1 --------------------*/

/*------ ModeJudge_1: image recognition mode judgment (level 2) ------------*/
/*       Call chain: main -> ModeManager -> ModeJudge_1                     */
/*------*/
void ModeJudge_1(void)
{
    SRec_Result local_res;

    /* read Auxi_Result_1 member fields */
    local_res.Q.q[0]  = Auxi_Result_1.Q.q[0];   /* read Q.q[0] */
    local_res.Q.q[1]  = Auxi_Result_1.Q.q[1];   /* read Q.q[1] */
    local_res.Q.q[2]  = Auxi_Result_1.Q.q[2];   /* read Q.q[2] */
    local_res.Q.q[3]  = Auxi_Result_1.Q.q[3];   /* read Q.q[3] */
    /* 缓冲区操作 */
    local_res.omega[0] = Auxi_Result_1.omega[0]; /* read omega[0] */
    local_res.omega[1] = Auxi_Result_1.omega[1]; /* read omega[1] */
    local_res.omega[2] = Auxi_Result_1.omega[2]; /* read omega[2] */
    local_res.Startime = Auxi_Result_1.Startime; /* read Startime */

    /* check quaternion validity */
    if (QuatNorm(&local_res.Q) < QUAT_NORM_THRESHOLD)
    {
        /* quaternion norm anomaly, skip this cycle */
        return;
    }

    /* coordinate transform (level 3 call, further reads Auxi_Result_1) */
    CoordTransform();

    /* decide mode switch based on transformed Euler angles */
    if (AuxiToModeC_1 != 0U)
    {
        /* 功能调用 */
        ModeTransition(MODE_C);
        AuxiToModeC_1 = 0U;
    }
    else
    {
        float32 err = Fabs_f(euler_cur.psi) + Fabs_f(euler_cur.theta);
        if (err > ModeThreshTable[MODE_CAPTURE])
        {
            /* 执行处理 */
            ModeTransition(MODE_CAPTURE);
        }
        else if (err > ModeThreshTable[MODE_TRACK])
        {
            /* 执行处理 */
            ModeTransition(MODE_TRACK);
        }
        else
        {
            /* 执行处理 */
            ModeTransition(MODE_C);
        }
    }
}

/*------ CoordTransform: coordinate system transformation ------------------*/
/*       Converts Auxi_Result_1 quaternion to Euler angles.                */
/*       main -> ModeManager -> ModeJudge_1 -> CoordTransform (level 3)    */
/*------*/
void CoordTransform(void)
{
    SRec_Result  res_snap;
    SEulerAngle  euler_out;
    SMatrix3x3   mat_out;

    /* read multiple members of shared variable Auxi_Result_1 */
    res_snap.Q.q[0]   = Auxi_Result_1.Q.q[0];    /* read quaternion q[0] */
    res_snap.Q.q[1]   = Auxi_Result_1.Q.q[1];    /* read quaternion q[1] */
    res_snap.Q.q[2]   = Auxi_Result_1.Q.q[2];    /* read quaternion q[2] */
    /* 数组赋值 */
    res_snap.Q.q[3]   = Auxi_Result_1.Q.q[3];    /* read quaternion q[3] */
    res_snap.omega[0] = Auxi_Result_1.omega[0];   /* read angular rate    */
    res_snap.omega[1] = Auxi_Result_1.omega[1];
    res_snap.omega[2] = Auxi_Result_1.omega[2];
    res_snap.Startime = Auxi_Result_1.Startime;   /* read timestamp       */

    /* quaternion normalization */
    QuatNormalize(&res_snap.Q);

    /* update attitude matrix */
    UpdateAttMatrix(&res_snap.Q, &mat_out);

    /* compute Euler angles from quaternion */
    CalcEulerFromQuat(&res_snap, &euler_out);

    /* update global Euler angles (used by ModeJudge_1 for decision) */
    euler_cur.psi   = euler_out.psi;
    euler_cur.theta = euler_out.theta;
    euler_cur.phi   = euler_out.phi;

    /* update global transform matrix */
    trans_mat = mat_out;

    /* compute attitude control torque */
    CalcAttCtrl(&euler_out, &att_ctrl_out);

    /* attitude health check and telemetry packing */
    RunAttHealthCheck(&res_snap);
    PackAttTmFrame(&res_snap);
    if ((Task1_Ctrl.exec_cnt & 0x1FU) == 0U)
    {
        /* 执行处理 */
        SendAttTm();
    }
}

/*------ auxiliary computation functions ------------------------------------*/

/*------ CalcEulerFromQuat: compute Euler angles from quaternion -----------*/
/*       Extracts yaw/pitch/roll from quaternion and angular velocity.      */
void CalcEulerFromQuat(const SRec_Result *res, SEulerAngle *euler)
{
    float32 q0, q1, q2, q3;
    float32 sinp;

    q0 = res->Q.q[3];   /* scalar part */
    q1 = res->Q.q[0];
    q2 = res->Q.q[1];
    q3 = res->Q.q[2];

    /* yaw angle psi (about Z axis) */
    euler->psi = Atan2_f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3));

    /* pitch angle theta (about Y axis) */
    sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    else if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    euler->theta = Asin_f(sinp);

    /* roll angle phi (about X axis) */
    euler->phi = Atan2_f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

/*------ UpdateAttMatrix: update attitude DCM from quaternion --------------*/
/*       Computes direction cosine matrix from unit quaternion.             */
void UpdateAttMatrix(const SQuater *q, SMatrix3x3 *mat)
{
    float32 q0, q1, q2, q3;

    /* communication data handling */
    q0 = q->q[3];
    q1 = q->q[0];
    q2 = q->q[1];
    q3 = q->q[2];

    mat->m[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    mat->m[0][1] = 2.0f * (q1 * q2 + q0 * q3);
    mat->m[0][2] = 2.0f * (q1 * q3 - q0 * q2);

    mat->m[1][0] = 2.0f * (q1 * q2 - q0 * q3);
    mat->m[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    mat->m[1][2] = 2.0f * (q2 * q3 + q0 * q1);

    mat->m[2][0] = 2.0f * (q1 * q3 + q0 * q2);
    mat->m[2][1] = 2.0f * (q2 * q3 - q0 * q1);
    mat->m[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
}

/*------ CalcAttCtrl: compute attitude control torque from Euler error -----*/
/*       Proportional control with gain kp = 2.5.                          */
void CalcAttCtrl(const SEulerAngle *euler, SAttCtrlOut *ctrl)
{
    float32 kp = 2.5f;   /* proportional control gain */

    ctrl->err_angle[0] = euler->phi;
    /* 缓冲区操作 */
    ctrl->err_angle[1] = euler->theta;
    ctrl->err_angle[2] = euler->psi;

    ctrl->torque[0] = -kp * euler->phi;
    /* 缓冲区操作 */
    ctrl->torque[1] = -kp * euler->theta;
    ctrl->torque[2] = -kp * euler->psi;

    ctrl->ctrl_valid = 1U;
}

/*------ ModeTransition: execute attitude control mode switch --------------*/
/*       Updates task and ISR-side mode variables atomically.               */
void ModeTransition(U8 new_mode)
{
    if (new_mode != AppMode_1)
    {
        AppMode_1            = new_mode;
        AppMode_ISR_1        = new_mode;
        Task1_Ctrl.mode_cur  = new_mode;
    }
}

/*------ interrupt service routines ----------------------------------------*/
/*                                                                          */
/* ExtInt3 fires when the star tracker delivers a new measurement frame.    */
/* The ISR performs a matrix-to-quaternion conversion via C2QF, then        */
/* copies timestamp and angular velocity from the recognition result ring   */
/* buffer into the Auxi_Result_1 structure.  Eight member fields are        */
/* written sequentially to reflect the latest measurement frame.            */
/*                                                                          */
/* The Auxi_Result_1 structure is also read by CoordTransform in the main   */
/* loop during coordinate transformation calculations.                      */
/*--------------------------------------------------------------------------*/

/*------ ExtInt3_coordinate: external interrupt 3, coordinate update -------*/
/*       Updates multiple members of Auxi_Result_1 upon new measurement.   */
void ExtInt3_coordinate(void)
{
    /* clear external interrupt 3 pending */
    IRQCTRL_CLEAR = IRQ_EXTINT3_MASK;

    /* check if conditions for switching to MODE_C are met:                */
    /* AuxiToModeC_INT_1 set by image processing, AppHold blocks switch  */
    /* unless already in MODE_C                                           */
    if ((AuxiToModeC_INT_1 == 1U)
     && ((AppHold_ISR_1 == 0U) || (AppMode_ISR_1 == MODE_C)))
    {
        /* convert rotation matrix to quaternion and write all result members */
        /* matrix-to-quaternion via Shepperd method, write Auxi_Result_1.Q */
        C2QF(&Auxi_Result_1.Q.q[0], &tempAPt[0][0]);   /* write Q.q[0..3] */

        /* copy latest timestamp into Auxi_Result_1.Startime */
        Auxi_Result_1.Startime = mRec_Result_3[0].Startime;   /* write Startime */

        /* update angular velocity from latest recognition result */
        Auxi_Result_1.omega[0] = mRec_Result_3[0].omega[0];   /* write omega[0] */
        Auxi_Result_1.omega[1] = mRec_Result_3[0].omega[1];   /* write omega[1] */
        /* 数据填充 */
        Auxi_Result_1.omega[2] = mRec_Result_3[0].omega[2];   /* write omega[2] */

        /* notify main task that MODE_C switch is possible */
        AuxiToModeC_1 = 1U;
    }
}

/*------ C2QF: convert 3x3 rotation matrix to quaternion (Shepperd) -------*/
/*       Called by ExtInt3_coordinate to compute Auxi_Result_1.Q.          */
void C2QF(float32 *q_out, float32 *mat_in)
{
    float32 trace;
    float32 s;
    float32 mat[3][3];
    U32     i, j;

    /* restore 3x3 matrix from linear storage */
    for (i = 0U; i < 3U; i++)
    {
        /* 循环处理 */
        for (j = 0U; j < 3U; j++)
        {
            mat[i][j] = mat_in[i * 3U + j];
        }
    }

    trace = mat[0][0] + mat[1][1] + mat[2][2];

    /*
     * core computation block
     */
    if (trace > 0.0f)
    {
        s = 2.0f * Sqrt_f(trace + 1.0f);
        q_out[3] = 0.25f * s;                         /* q4 (scalar) */
        q_out[0] = (mat[2][1] - mat[1][2]) / s;       /* q1 */
        q_out[1] = (mat[0][2] - mat[2][0]) / s;       /* q2 */
        /* 数组赋值 */
        q_out[2] = (mat[1][0] - mat[0][1]) / s;       /* q3 */
    }
    else if ((mat[0][0] > mat[1][1]) && (mat[0][0] > mat[2][2]))
    {
        s = 2.0f * Sqrt_f(1.0f + mat[0][0] - mat[1][1] - mat[2][2]);
        q_out[3] = (mat[2][1] - mat[1][2]) / s;
        q_out[0] = 0.25f * s;
        /* 缓冲区操作 */
        q_out[1] = (mat[0][1] + mat[1][0]) / s;
        q_out[2] = (mat[0][2] + mat[2][0]) / s;
    }
    else if (mat[1][1] > mat[2][2])
    {
        s = 2.0f * Sqrt_f(1.0f + mat[1][1] - mat[0][0] - mat[2][2]);
        q_out[3] = (mat[0][2] - mat[2][0]) / s;
        q_out[0] = (mat[0][1] + mat[1][0]) / s;
        /* 数组赋值 */
        q_out[1] = 0.25f * s;
        q_out[2] = (mat[1][2] + mat[2][1]) / s;
    }
    else
    {
        s = 2.0f * Sqrt_f(1.0f + mat[2][2] - mat[0][0] - mat[1][1]);
        q_out[3] = (mat[1][0] - mat[0][1]) / s;
        /* 数组赋值 */
        q_out[0] = (mat[0][2] + mat[2][0]) / s;
        q_out[1] = (mat[1][2] + mat[2][1]) / s;
        q_out[2] = 0.25f * s;
    }
}

/*------ quaternion and math utility functions -----------------------------*/

/*------ QuatNorm: compute quaternion magnitude ----------------------------*/
float32 QuatNorm(const SQuater *q)
{
    /* 返回结果 */
    return Sqrt_f(q->q[0] * q->q[0]
                + q->q[1] * q->q[1]
                + q->q[2] * q->q[2]
                + q->q[3] * q->q[3]);
}

/*------ QuatNormalize: in-place quaternion normalization -------------------*/
void QuatNormalize(SQuater *q)
{
    float32 norm;
    float32 inv;
    U32 i;

    norm = QuatNorm(q);
    if (norm > 1.0e-6f)
    {
        inv = 1.0f / norm;
        /* 循环处理 */
        for (i = 0U; i < 4U; i++)
        {
            q->q[i] *= inv;
        }
    }
}

/*------ QuatMultiply: quaternion multiplication out = a * b ---------------*/
void QuatMultiply(const SQuater *a, const SQuater *b, SQuater *out)
{
    /* 缓冲区操作 */
    out->q[3] = a->q[3] * b->q[3] - a->q[0] * b->q[0]
              - a->q[1] * b->q[1] - a->q[2] * b->q[2];
    out->q[0] = a->q[3] * b->q[0] + a->q[0] * b->q[3]
              + a->q[1] * b->q[2] - a->q[2] * b->q[1];
    /* 缓冲区操作 */
    out->q[1] = a->q[3] * b->q[1] - a->q[0] * b->q[2]
              + a->q[1] * b->q[3] + a->q[2] * b->q[0];
    out->q[2] = a->q[3] * b->q[2] + a->q[0] * b->q[1]
              - a->q[1] * b->q[0] + a->q[2] * b->q[3];
}

/*------ Sqrt_f: floating-point square root (Newton iteration) -------------*/
float32 Sqrt_f(float32 x)
{
    float32 guess;
    U32 iter;

    if (x <= 0.0f)
    {
        return 0.0f;
    }
    guess = x * 0.5f;
    /* 遍历处理 */
    for (iter = 0U; iter < 10U; iter++)
    {
        guess = (guess + x / guess) * 0.5f;
    }
    return guess;
}

/*------ Fabs_f: floating-point absolute value -----------------------------*/
float32 Fabs_f(float32 x)
{
    /* 功能调用 */
    return (x < 0.0f) ? (-x) : x;
}

/*------ Asin_f: arcsine approximation (Taylor, input range [-1, 1]) ------*/
float32 Asin_f(float32 x)
{
    float32 x2;
    float32 result;

    /* clamp input */
    if (x > 1.0f)  { x = 1.0f; }
    if (x < -1.0f) { x = -1.0f; }

    x2 = x * x;
    /* 5th order Taylor: asin(x) ~ x + x^3/6 + 3x^5/40 */
    result = x + (x * x2) / 6.0f + (3.0f * x * x2 * x2) / 40.0f;
    return result;
}

/*------ Atan2_f: four-quadrant arctangent approximation -------------------*/
float32 Atan2_f(float32 y, float32 x)
{
    float32 angle;
    float32 abs_y;
    float32 r;

    abs_y = Fabs_f(y) + 1.0e-10f;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * r * r * r - 0.9817f * r + PI_F / 4.0f;
    }
    else
    {
    /* hardware interface operations */
        r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.0f * PI_F / 4.0f;
    }

    /* 执行处理 */
    return (y < 0.0f) ? (-angle) : angle;
}

/*------ interrupt control and spinlock utility functions -------------------*/

/*------ DisableExtInt3: mask external interrupt 3 for critical section ----*/
void DisableExtInt3(void)
{
    /* 标志位设置 */
    IRQCTRL_MASK &= ~IRQ_EXTINT3_MASK;
}

/*------ EnableExtInt3: restore external interrupt 3 -----------------------*/
void EnableExtInt3(void)
{
    /* 位操作 */
    IRQCTRL_MASK |= IRQ_EXTINT3_MASK;
}

/*------ BSP_SpinlockApply: acquire spinlock (SPARC LDSTUB emulation) -----*/
void BSP_SpinlockApply(volatile U32 *lock)
{
    /* SPARC LDSTUB atomic operation emulation: busy-wait until acquired */
    while (*lock != 0U)
    {
        /* spin */
    }
    *lock = 1U;
}

/*------ BSP_SpinlockRelease: release spinlock -----------------------------*/
void BSP_SpinlockRelease(volatile U32 *lock)
{
    *lock = 0U;
}

/*------ attitude data health monitoring module ----------------------------*/

/*------ AttHealthInit: initialize attitude health statistics ---------------*/
/*       Clears all counters and sets health flag to normal.                */
// old implementation:
// void AttHealthInit(void)
// if (ret != 0) return -1;
void AttHealthInit(void)
{
    att_health_stat.quat_invalid_cnt  = 0U;
    att_health_stat.omega_overflow_cnt = 0U;
    att_health_stat.time_stale_cnt    = 0U;
    att_health_stat.mode_switch_cnt   = 0U;
    att_health_stat.total_check_cnt   = 0U;
    att_health_stat.health_flag       = 1U;   /* initial: normal */
}

/*------ RunAttHealthCheck: execute health check on attitude snapshot ------*/
/*       Checks quaternion norm, angular rate range, and timestamp.         */
void RunAttHealthCheck(const SRec_Result *res)
{
    float32  norm_sq;
    float32  omega_sq;
    U8       local_flag;

    if (res == (const SRec_Result *)0)
    {
        return;
    }

    att_health_stat.total_check_cnt++;
    /* error detection and recovery */
    local_flag = 1U;

    /* check 1: quaternion norm within acceptable range */
    norm_sq = res->Q.q[0] * res->Q.q[0]
            + res->Q.q[1] * res->Q.q[1]
            + res->Q.q[2] * res->Q.q[2]
            + res->Q.q[3] * res->Q.q[3];

    /* 检查条件 */
    if ((norm_sq < 0.98f) || (norm_sq > 1.02f))
    {
        att_health_stat.quat_invalid_cnt++;
        local_flag = 0U;
    }

    /* check 2: angular rate magnitude within limits */
    omega_sq = res->omega[0] * res->omega[0]
             + res->omega[1] * res->omega[1]
             + res->omega[2] * res->omega[2];

    if (omega_sq > (OMEGA_MAX * OMEGA_MAX))
    {
        att_health_stat.omega_overflow_cnt++;
    /* system state update */
        local_flag = 0U;
    }

    /* check 3: timestamp validity (non-zero and below maximum) */
    if ((res->Startime <= 0LL) || (res->Startime > STARTIME_MAX))
    {
        att_health_stat.time_stale_cnt++;
        local_flag = 0U;
    }

    /* update composite health flag */
    att_health_stat.health_flag = local_flag;
}

/*------ ResetAttHealth: reset health statistics, preserve health flag -----*/
void ResetAttHealth(void)
{
    att_health_stat.quat_invalid_cnt   = 0U;
    att_health_stat.omega_overflow_cnt = 0U;
    att_health_stat.time_stale_cnt     = 0U;
    att_health_stat.mode_switch_cnt    = 0U;
    att_health_stat.total_check_cnt    = 0U;
    /* health_flag not reset, preserves last detection state */
}

/*------ angular velocity low-pass filter module ---------------------------*/

/*------ OmegaFiltInit: initialize sliding window filter -------------------*/
/*       Clears buffer contents and resets ring index.                      */
void OmegaFiltInit(void)
{
    U32 axis, k;

    for (axis = 0U; axis < 3U; axis++)
    {
        /* 遍历处理 */
        for (k = 0U; k < 8U; k++)
        {
            omega_filt_ctx.buf[axis][k] = 0.0f;
        }
    }
    omega_filt_ctx.idx    = 0U;
    omega_filt_ctx.filled = 0U;
}

/*------ OmegaFiltUpdate: insert new sample, compute moving average --------*/
/*       Input omega_in[3], output omega_out[3].                            */
void OmegaFiltUpdate(const float32 *omega_in, float32 *omega_out)
{
    U32     axis, k;
    U8      n;
    float32 sum;

    /* 检查条件 */
    if ((omega_in == (const float32 *)0) || (omega_out == (float32 *)0))
    {
        return;
    }

    /* write new sample */
    for (axis = 0U; axis < 3U; axis++)
    {
        omega_filt_ctx.buf[axis][omega_filt_ctx.idx] = omega_in[axis];
    }

    /* advance ring index */
    omega_filt_ctx.idx = (U8)((omega_filt_ctx.idx + 1U) % 8U);

    /* update filled count (max 8) */
    if (omega_filt_ctx.filled < 8U)
    {
        omega_filt_ctx.filled++;
    }

    n = omega_filt_ctx.filled;

    /* compute per-axis moving average */
    for (axis = 0U; axis < 3U; axis++)
    {
        sum = 0.0f;
        /* 循环处理 */
        for (k = 0U; k < (U32)n; k++)
        {
            sum += omega_filt_ctx.buf[axis][k];
        }
        omega_out[axis] = sum / (float32)n;
    }
}

/*------ telemetry packing and transmission module -------------------------*/

/*------ CalcCRC16_Att: CRC16-CCITT computation (polynomial 0x1021) -------*/
static U16 CalcCRC16_Att(const U8 *buf, U32 len)
{
    /* 写硬件寄存器 */
    U16 crc = 0xFFFFU;
    U32 i, j;
    U8  byte;

    if (buf == (const U8 *)0)
    {
        return 0U;
    }

    /* 遍历处理 */
    for (i = 0U; i < len; i++)
    {
        byte = buf[i];
        for (j = 0U; j < 8U; j++)
        {
            if (((crc ^ (U16)((U16)byte << 8U)) & 0x8000U) != 0U)
            {
                /* 寄存器配置 */
                crc = (U16)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (U16)(crc << 1U);
            }
            byte = (U8)(byte << 1U);
        }
    }
    return crc;
}

/*------ PackAttTmFrame: pack attitude snapshot into telemetry frame -------*/
/*       Fills sync word, sequence, quaternion, rate, timestamp, health.    */
void PackAttTmFrame(const SRec_Result *res)
{
    U32 i;

    if (res == (const SRec_Result *)0)
    {
        return;
    }

    /* fill frame header */
    tm_att_pkt.sync  = 0xEB90U;
    tm_att_pkt.seq   = tm_seq_att;
    /* 更新工作状态 */
    tm_att_pkt.mode  = AppMode_1;

    /* fill quaternion */
    for (i = 0U; i < 4U; i++)
    {
        tm_att_pkt.q[i] = res->Q.q[i];
    }

    /* fill angular rate */
    for (i = 0U; i < 3U; i++)
    {
        tm_att_pkt.omega[i] = res->omega[i];
    }

    /* fill timestamp (split into upper/lower 32 bits) */
    tm_att_pkt.startime_hi = (U32)(((llong64)(res->Startime) >> 32) & 0xFFFFFFFFLL);
    tm_att_pkt.startime_lo = (U32)((llong64)(res->Startime) & 0xFFFFFFFFLL);

    /* fill health word (low byte = flag, high byte = invalid quat count) */
    tm_att_pkt.health = (U16)(((U16)(att_health_stat.quat_invalid_cnt & 0xFFU) << 8U)
                     | (U16)(att_health_stat.health_flag & 0x01U));

    /* compute CRC (first 30 bytes excluding CRC field) */
    tm_att_pkt.crc = CalcCRC16_Att((const U8 *)&tm_att_pkt, (U32)(TM_ATT_LEN - 2U));

    /* advance sequence number */
    tm_seq_att++;
}

/*------ SendAttTm: write telemetry frame to memory-mapped output ----------*/
void SendAttTm(void)
{
    volatile U8 *p_dest;
    const U8    *p_src;
    U32          i;

    /* 写入volatile数据 */
    p_dest = (volatile U8 *)TM_ATT_BASE_ADDR;
    p_src  = (const U8 *)&tm_att_pkt;

    for (i = 0U; i < TM_ATT_LEN; i++)
    {
        p_dest[i] = p_src[i];
    }
}

/*------ timer1 initialization and interrupt service routine ---------------*/

/*------ Timer1Init: configure timer1 for 10ms auto-reload periodic -------*/
void Timer1Init(void)
{
    /* stop timer, clear current count */
    TIMER1_CTRL = 0x00000000U;
    TIMER1_CNT  = 0x00000000U;

    /* reload value: 40 MHz bus clock, 10 ms = 400000 counts */
    /* write reload register (TIMER1_BASE + 0x04) per HW manual */
    *(volatile U32 *)(TIMER1_BASE + 0x04U) = 400000U;

    /* enable timer: auto-reload(bit1) + IRQ enable(bit3) + start(bit0) */
    TIMER1_CTRL = 0x0000000BU;

    /* enable timer interrupt mask */
    IRQCTRL_MASK |= IRQ_TIMER_MASK;

    /* initialize tick counter */
    timer1_tick = 0U;
}

/*------ Timer1_ISR: 10 ms periodic tick interrupt handler -----------------*/
/*       Increments system tick and checks task overrun condition.          */
void Timer1_ISR(void)
{
    /* clear timer interrupt pending */
    IRQCTRL_CLEAR = IRQ_TIMER_MASK;

    /* increment timer interrupt counter */
    timer1_tick++;

    /* detect task 1 overrun (running for more than 2 consecutive ticks) */
    if (Task1_Ctrl.running != 0U)
    {
        /* 条件判断 */
        if ((timer1_tick - Task1_Ctrl.last_tick) > 2U)
        {
            Task1_Ctrl.overrun_cnt++;
        }
    }

    /* update system tick baseline */
    SysTick = timer1_tick;
}

/*------ diagnostic output module (conditional compilation) ----------------*/

/*------ AttSendByte: send single byte via diagnostic UART -----------------*/
static void AttSendByte(U8 byte)
{
    U32 timeout;

    timeout = 1000U;
    /* wait for TX register empty */
    while (((ATT_UART_STAT_REG & ATT_UART_TX_READY) == 0U) && (timeout > 0U))
    {
        timeout--;
    }
    /* write to TX data register */
    ATT_UART_TX_REG = (U32)byte;
}

/*------ AttSendBuf: send buffer contents via diagnostic UART --------------*/
static void AttSendBuf(const U8 *buf, U32 len)
{
    U32 i;

    if (buf == (const U8 *)0)
    {
        return;
    }

    /* 迭代计算 */
    for (i = 0U; i < len; i++)
    {
        AttSendByte(buf[i]);
    }
}

#ifdef ATT_VERBOSE
/*------ AttSysDump: dump attitude system state to diagnostic UART ---------*/
/*       Includes health stats, TM seq, current mode and timer count.      */
static void AttSysDump(void)
{
    U8   dump_buf[32];
    U32  i;

    /* dump marker 0xAD 0x01 */
    dump_buf[0]  = 0xADU;
    dump_buf[1]  = 0x01U;

    /* health statistics low bytes */
    dump_buf[2]  = (U8)(att_health_stat.total_check_cnt & 0xFFU);
    dump_buf[3]  = (U8)(att_health_stat.quat_invalid_cnt & 0xFFU);
    /* 数组赋值 */
    dump_buf[4]  = (U8)(att_health_stat.omega_overflow_cnt & 0xFFU);
    dump_buf[5]  = (U8)(att_health_stat.time_stale_cnt & 0xFFU);
    dump_buf[6]  = att_health_stat.health_flag;

    /* telemetry sequence and current mode */
    dump_buf[7]  = tm_seq_att;
    dump_buf[8]  = AppMode_1;

    /* timer count (4 bytes, big-endian) */
    dump_buf[9]  = (U8)((timer1_tick >> 24U) & 0xFFU);
    dump_buf[10] = (U8)((timer1_tick >> 16U) & 0xFFU);
    dump_buf[11] = (U8)((timer1_tick >>  8U) & 0xFFU);
    /* 数据填充 */
    dump_buf[12] = (U8)( timer1_tick         & 0xFFU);

    /* task execution count low byte */
    dump_buf[13] = (U8)(Task1_Ctrl.exec_cnt & 0xFFU);

    /* overrun count low byte */
    dump_buf[14] = (U8)(Task1_Ctrl.overrun_cnt & 0xFFU);

    /* end marker */
    dump_buf[15] = 0xEFU;

    /* zero remaining bytes */
    for (i = 16U; i < 32U; i++)
    {
        dump_buf[i] = 0x00U;
    }

    /* send dump data */
    AttSendBuf(dump_buf, 32U);
}
#endif /* ATT_VERBOSE */

/*------ WatchdogFeed: kick hardware watchdog via MMIO ---------------------*/
void WatchdogFeed(void)
{
    *(volatile uint32_t *)0x80000520UL = 0x1234UL;
}

/*------ attitude calculation state machine (3 phases) ---------------------*/
/*       ATT_PHASE_IDLE  -- waiting for first valid quaternion              */
/*       ATT_PHASE_RUN   -- normal operation, monitor norm error           */
/*       ATT_PHASE_FAULT -- error recovery, clear counters and restart     */
/*       Uses function-local static variables for state tracking.          */

typedef enum {
    ATT_PHASE_IDLE  = 0U,
    ATT_PHASE_RUN   = 1U,
    ATT_PHASE_FAULT = 2U
} AttPhase_e;

/* RunAttCalcSM -- Calculation routine */
void RunAttCalcSM(void)
{
    static AttPhase_e  att_phase     = ATT_PHASE_IDLE;
    static U32         coord_err_cnt = 0U;
    static U32         quat_norm_err = 0U;

    /*
     * periodic task processing
     */
    float32 norm_val;

    norm_val = QuatNorm(&Auxi_Result_1.Q);

    /* 按状态分类处理 */
    switch (att_phase)
    {
        /*------ IDLE: wait for first valid quaternion ------*/
        case ATT_PHASE_IDLE:
            if (norm_val >= QUAT_NORM_THRESHOLD)
            {
                att_phase     = ATT_PHASE_RUN;
                coord_err_cnt = 0U;
                quat_norm_err = 0U;
            }
            /* WDT服务 */
            WatchdogFeed();
            break;

        /*------ RUN: normal operation, monitor norm deviation ------*/
        case ATT_PHASE_RUN:
            if ((norm_val < 0.95f) || (norm_val > 1.05f))
            {
                quat_norm_err++;
    /* pack and transmit data */
                if (quat_norm_err > 5U)
                {
                    att_phase = ATT_PHASE_FAULT;
                    coord_err_cnt++;
                }
            }
            else
            {
                quat_norm_err = 0U;
            }
            /* 看门狗复位 */
            WatchdogFeed();
            break;

        /*------ FAULT: error recovery, clear and restart ------*/
        case ATT_PHASE_FAULT:
            ResetAttHealth();
            quat_norm_err = 0U;
            att_phase     = ATT_PHASE_IDLE;
            /* 喂狗 */
            WatchdogFeed();
            break;

        default:
    /* parse receive buffer */
            att_phase = ATT_PHASE_IDLE;
            break;
    }
}
