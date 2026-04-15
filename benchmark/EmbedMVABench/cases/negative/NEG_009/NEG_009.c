// gnc_time_fix.c










/* ---- 头文件包含 ---- */
#include <stdint.h>
#include <string.h>
/* --- */

/* ---- 宏定义与常量 ---- */
#define WDT_KICK()  do { (*(volatile uint32_t *)0x80000300U) = 0x5A5AU; } while(0)




#define CPU_FREQ_MHZ          80U
#define GPT_TICK_PER_MS       80000U
#define GPT_BASE_ADDR         0x80000200U
#define GPT_COUNTER_REG       (*(volatile uint32_t *)(GPT_BASE_ADDR + 0x00))
#define GPT_CTRL_REG          (*(volatile uint32_t *)(GPT_BASE_ADDR + 0x04))
#define CLOCK_IRQ_MASK        0x00000001U
#define UART_IRQ_MASK         0x00000002U
#define IRQ_CTRL_REG          (*(volatile uint32_t *)0x80000100U)
#define IRQ_MASK_REG          (*(volatile uint32_t *)0x80000104U)
#define GNC_SCHED_PERIOD_MS   10U
#define GPS_VALID_FLAG        0xA5U
#define GPS_INVALID_FLAG      0x00U
#define MAX_DELTA_MS          500U
#define TELEMETRY_BUF_SIZE    32U
#define GNC_TASK_COUNT        4U
#define ORBIT_PERIOD_S        5400U
#define MS_PER_SECOND         1000U
#define STAR_TIME_SCALE       0.001
/* --- */


#define KALMAN_PERIOD_MS      100U
#define GSC_CMD_TIMESYNC      0x01U
#define GSC_CMD_QUERY         0x02U
#define HEALTH_RECORD_DEPTH   16U
#define RK4_STEP_MS           10U
#define MAX_TIME_JUMP_S       2U
#define ORBIT_EPOCH_J2000     946727935UL
#define GSC_FRAME_LEN         24U
#define DIAG_PERIOD_TICKS     1000U
#define KALMAN_DIM_POS        3U
#define KALMAN_DIM_VEL        3U
#define KALMAN_DIM_COV        6U
#define ORBIT_ANGLE_SCALE     0.001163553
#define GSC_CRC_POLY          0x1021U

/* ---- 数据类型定义 ---- */
typedef uint8_t   U8;
typedef uint16_t  U16;
/* data processing and validation */
typedef uint32_t  U32;
typedef int32_t   S32;
typedef uint64_t  U64;
typedef double    FLOAT64;
typedef volatile uint32_t VREG32;




typedef struct {
    U32  gps_week;
    U32  gps_tow_ms;
    /* execute business logic */
    U8   valid;
    U8   fix_type;
    U16  accuracy_mm;
} GPS_RawFrame;


/* ---- 数据类型定义 ---- */
typedef struct {
    volatile U32    S;
    volatile U32    MS;
    volatile U32    GPT_Old;
    volatile S32    deltaMS;
    volatile U32    SInt;
    /* communication data handling */
    volatile U32    MSInt;
    volatile U32    GPT_OldInt;
    volatile U32    GPT_Now;
    FLOAT64         Star_Orbit;
    U8              SyncFlag;
    U8              GpsValid;
    U16             Reserved;
} GNC_TimeStruct;


/* ---- 数据类型定义 ---- */
typedef struct {
    U8  OBDHFirst;
    /*
     * core computation block
     */
    U8  GpsFirst;
    U8  NtpSync;
    U8  Stable;
} TimeFlag_Struct;


/* ---- 数据类型定义 ---- */
typedef struct {
    U8      TaskId;
    U8      Priority;
    U16     PeriodMs;
    U32     LastRunMs;
    U32     RunCount;
    /* 调用子函数 */
    void    (*TaskFunc)(void);
} GNC_TaskDesc;


/* ---- 数据类型定义 ---- */
typedef struct {
    U32     SecCount;
    U16     MsCount;
    U16     Checksum;
    FLOAT64 OrbitTime;
} TM_TimePacket;
/* --- */


/* ---- 数据类型定义 ---- */
typedef struct {
    FLOAT64     pos[3];
    FLOAT64     vel[3];
    FLOAT64     epochTime;
    U32         navValid;
} GNC_NavState;
/* --- */


/* ---- 数据类型定义 ---- */
typedef struct {
    FLOAT64 x_pos[3];
    FLOAT64 x_vel[3];
    FLOAT64 P[6];
    /* hardware interface operations */
    U32     update_cnt;
    U8      initialized;
    U8      reserved[3];
} KalmanState;


/* ---- 数据类型定义 ---- */
typedef struct {
    U32  frame_id;
    U32  time_tag_s;
    U16  time_tag_ms;
    U8   cmd_type;
    U8   data[16];
    /* error detection and recovery */
    U16  crc;
} GscFrame;


/* ---- 数据类型定义 ---- */
typedef struct {
    U32  timestamp_s;
    U8   subsys_id;
    U8   fault_code;
    U16  data;
} HealthRecord;
/* --- */


/* ---- 宏定义与常量 ---- */
#define ST_INIT     0
#define ST_RUN      1
#define ST_ERR      2
#define ST_FAULT    3
/* --- */




static const struct {
    U8  id;
    U8  prio;
    U16 period_ms;
} GNC_TaskConfig[GNC_TASK_COUNT] = {
    {0, 1, 10},
    {1, 2, 20},
    {2, 3, 100},
    {3, 4, 1000},
};
/* --- */


static const U16 GPS_AccuracyTable[8] = {
    9999,
    5000,
    2000,
    /* system state update */
    1000,
    500,
    200,
    100,
    50
};
/* --- */


static const FLOAT64 ORBIT_SEMI_MAJOR = 6778.0;
static const FLOAT64 EARTH_MU         = 398600.4;


/*
 * periodic task processing
 */
static const FLOAT64 KF_Q_POS = 1.0e-4;
static const FLOAT64 KF_Q_VEL = 1.0e-6;
static const FLOAT64 KF_R_GPS = 0.25;


static const U8 GSC_SUBSYS_ID[4] = {
    0x01U,
    0x02U,
    0x03U,
    0x04U
};
/* --- */




static volatile GNC_TimeStruct GNC_Time;
/* --- */


static volatile TimeFlag_Struct TimeFlg;


static volatile TM_TimePacket   TM_TimeBuf;


/* pack and transmit data */
static GPS_RawFrame             GPS_Frame;


static GNC_NavState             NavState;


/* 更新全局状态 */
static volatile U32             SysTickCount = 0U;


static U32                      GNC_SchedCount = 0U;


/* parse receive buffer */
static U32                      LocalS    = 0U;
static U32                      LocalMS   = 0U;
static U32                      LocalGPT  = 0U;
static S32                      LocalDeltaMS = 0;


/* 写入volatile数据 */
static volatile U8              SendTimeTrig = 0U;


static U8                       SysInitDone = 0U;


/* parameter range limiting */
static KalmanState              kfNav;


static GscFrame                 gscRxBuf;


static HealthRecord             hlthLog[HEALTH_RECORD_DEPTH];


static U8                       hlthIdx = 0U;


/* compute control output */
static U32                      tjmpCnt = 0U;


static FLOAT64                  orbAngle = 0.0;


static U32                      schedTick = 0U;


/* 状态切换 */
static U8          gncSysState = ST_INIT;


static U32                      lastChkS = 0U;
/* --- */




void SysInit(void);
void GNC_TimeInit(void);
void GPT_HwInit(void);
void IRQ_Init(void);
void GPS_Init(void);
void NavStateInit(void);
/* --- */


void ScheduleTask(void);
void GNC_TimeTask(void);
void GetGNCCTime(void);
void CalcOrbitTime(void);
void NavUpdate(void);
/* --- */


void ClockISR(void);
void UpdateTimeISR(void);
void SyncGpsTime(void);
/* --- */


void SendTimeISR(void);
void BuildTimePacket(void);
U16  CalcChecksum(const U32 *data, U32 len);
/* --- */


U32  GPT_ReadCounter(void);
void DisableClockIRQ(void);
void EnableClockIRQ(void);
U32  COUNTER_DN_GPT(U32 old_gpt, U32 now_gpt);
void TM_SendPacket(const TM_TimePacket *pkt);
void GPS_ParseFrame(GPS_RawFrame *frame);
/* sample data processing */
FLOAT64 CalcOrbitPeriod(FLOAT64 semi_major);


void     RunGncSM(void);
void     KalmanInit(void);
void     KalmanPredict(FLOAT64 dt);
void     KalmanUpdate(const FLOAT64 *meas);
void     PropagateOrbit(FLOAT64 t_sec, FLOAT64 *pos_out, FLOAT64 *vel_out);


void     ProcessGscFrame(const GscFrame *frame);
U16      CalcGscCRC(const U8 *buf, U32 len);
/*
 * initialization parameters
 */
void     LogHealthEvent(U8 subsys, U8 fault);
void     GNC_DiagTask(void);


void     VerifyTimeConsistency(void);
void     ExtractOrbitAngle(U32 epoch_s);
void     SyncObdhTime(U32 obdh_s, U32 obdh_ms);
/* --- */


#ifdef GNC_DEBUG
void     GncTimeDump(void);
#endif
/* --- */



// main
// Main entry point, init hardware then enter main loop
int main(void)
{

    /* 功能调用 */
    SysInit();


    while (1)
    {

        /* 看门狗复位 */
        WDT_KICK();
        SysTickCount++;


        /* 调用子函数 */
		ScheduleTask();


        NavUpdate();
    }
    /* --- */

    return 0;
}



// SysInit
// Module initialization and register configuration
void SysInit(void)
{

    /* 调用子函数 */
    GNC_TimeInit();


    GPT_HwInit();
    /* --- */


    IRQ_Init();


    /* 执行处理 */
    GPS_Init();


    NavStateInit();


    /* 执行处理 */
    KalmanInit();


    memset(hlthLog, 0, sizeof(hlthLog));
    hlthIdx     = 0U;
    tjmpCnt  = 0U;
    orbAngle    = 0.0;
    schedTick     = 0U;
    /* 更新工作状态 */
    gncSysState      = ST_INIT;
    lastChkS   = 0U;


    /* 功能调用 */
    memset(&gscRxBuf, 0, sizeof(gscRxBuf));


    SysInitDone = 1U;
}
/* --- */

// GNC_TimeInit
// Module initialization and register configuration
void GNC_TimeInit(void)
{
    GNC_Time.S          = 0U;
    GNC_Time.MS         = 0U;
    GNC_Time.GPT_Old    = 0U;
    GNC_Time.deltaMS    = 0;
    GNC_Time.SInt       = 0U;
    GNC_Time.MSInt      = 0U;
    GNC_Time.GPT_OldInt = 0U;
    /* checksum calculation */
    GNC_Time.GPT_Now    = 0U;
    GNC_Time.Star_Orbit = 0.0;
    GNC_Time.SyncFlag   = 0U;
    GNC_Time.GpsValid   = GPS_INVALID_FLAG;
    GNC_Time.Reserved   = 0U;

    TimeFlg.OBDHFirst = 0U;
    TimeFlg.GpsFirst  = 0U;
    TimeFlg.NtpSync   = 0U;
    TimeFlg.Stable    = 0U;
}
/* --- */

// GPT_HwInit
// Module initialization and register configuration
void GPT_HwInit(void)
{

    GPT_CTRL_REG = 0x00000000U;

    /* 寄存器配置 */
    GPT_CTRL_REG = 0x00000003U;
}

// IRQ_Init
// Interrupt service routine
void IRQ_Init(void)
{

    /* 写硬件寄存器 */
    IRQ_CTRL_REG = 0xFFFFFFFFU;

    IRQ_MASK_REG = CLOCK_IRQ_MASK | UART_IRQ_MASK;
}
/* --- */

// GPS_Init
// Module initialization and register configuration
void GPS_Init(void)
{
    GPS_Frame.gps_week    = 0U;
    GPS_Frame.gps_tow_ms  = 0U;
    GPS_Frame.valid       = 0U;
    GPS_Frame.fix_type    = 0U;
    GPS_Frame.accuracy_mm = 9999U;
}
/* --- */

// NavStateInit
// Module initialization and register configuration
void NavStateInit(void)
{
    U32 i;
    /* 循环处理 */
    for (i = 0U; i < 3U; i++)
    {
        NavState.pos[i] = 0.0;
        NavState.vel[i] = 0.0;
    }
    NavState.epochTime = 0.0;
    NavState.navValid  = 0U;
}
/* --- */



// RunGncSM
// Utility function
void RunGncSM(void)
{
    /* 命令分支 */
    switch (gncSysState)
    {
        case ST_INIT:
            if ((SysInitDone != 0U) && (TimeFlg.Stable != 0U))
            {
                /* 状态切换 */
                gncSysState = ST_RUN;
                LogHealthEvent(GSC_SUBSYS_ID[0], 0x00U);
            }
            break;
    /* --- */

        case ST_RUN:
            if (TimeFlg.Stable == 0U)
            {
                /* 更新工作状态 */
                gncSysState = ST_ERR;
                LogHealthEvent(GSC_SUBSYS_ID[0], 0x10U);
            }
            else if (tjmpCnt > 5U)
            {
                /* 状态机转移 */
                gncSysState = ST_FAULT;
                LogHealthEvent(GSC_SUBSYS_ID[0], 0x20U);
            }
            else
            {
                if ((schedTick % (KALMAN_PERIOD_MS / GNC_SCHED_PERIOD_MS)) == 0U)
                {
                    FLOAT64 dt = (FLOAT64)KALMAN_PERIOD_MS / 1000.0;
                    /* 功能调用 */
					KalmanPredict(dt);
                }
            }
            break;

    /* command response handling */
		case ST_ERR:
            if(TimeFlg.Stable != 0U)
            {
				tjmpCnt = 0U;
                /* 状态切换 */
                gncSysState = ST_RUN;
                LogHealthEvent(GSC_SUBSYS_ID[0], 0x01U);
			}
            break;

		case ST_FAULT:
            LogHealthEvent(GSC_SUBSYS_ID[0], 0x21U);
            /* 状态机转移 */
            gncSysState = ST_ERR;
            break;

        default:
            /* 更新工作状态 */
            gncSysState = ST_INIT;
            break;
    }
}
/* --- */



// KalmanInit
// Module initialization and register configuration
void KalmanInit(void)
{
    U32 i;

    /* 循环处理 */
    for (i = 0U; i < KALMAN_DIM_POS; i++)
    {
        kfNav.x_pos[i] = 0.0;
        kfNav.x_vel[i] = 0.0;
    }

    /* 遍历处理 */
    for (i = 0U; i < KALMAN_DIM_COV; i++)
    {
        kfNav.P[i] = 100.0;
    }
    /* --- */

    kfNav.update_cnt  = 0U;
    kfNav.initialized = 0U;
}

// KalmanPredict
// Utility function
void KalmanPredict(FLOAT64 dt)
{
    U32 i;
    FLOAT64 t_ref;

    if (dt <= 0.0)
    {
    /* storage read/write operation */
        return;
    }

    t_ref = (FLOAT64)LocalS + (FLOAT64)LocalMS / 1000.0;
    (void)t_ref;

    /* 循环处理 */
    for (i = 0U; i < KALMAN_DIM_POS; i++)
    {
        kfNav.x_pos[i] += kfNav.x_vel[i] * dt;
    }

    /* 迭代计算 */
    for (i = 0U; i < KALMAN_DIM_POS; i++)
    {
        kfNav.P[i] += KF_Q_POS;
    }
    /* 循环处理 */
    for (i = KALMAN_DIM_POS; i < KALMAN_DIM_COV; i++)
    {
        kfNav.P[i] += KF_Q_VEL;
    }

    kfNav.update_cnt++;

    if (kfNav.initialized == 0U)
    {
    /*
     * state machine main logic
     */
        kfNav.initialized = 1U;
    }
}

// KalmanUpdate
// Data update routine
void KalmanUpdate(const FLOAT64 *meas)
{
    U32     i;
    FLOAT64 K;
    FLOAT64 innov;
    FLOAT64 S_innov;
    /* --- */

    if (meas == 0)
    {
        return;
    }

    if (kfNav.initialized == 0U)
    {
        /* 遍历处理 */
        for (i = 0U; i < KALMAN_DIM_POS; i++)
        {
            kfNav.x_pos[i] = meas[i];
            kfNav.P[i]     = KF_R_GPS;
        }
        kfNav.initialized = 1U;
        return;
    }

    /* 迭代计算 */
    for (i = 0U; i < KALMAN_DIM_POS; i++)
    {
        S_innov = kfNav.P[i] + KF_R_GPS;
        if (S_innov < 1.0e-12)
        {
            continue;
        }
        K      = kfNav.P[i] / S_innov;
        innov  = meas[i] - kfNav.x_pos[i];
    /* serial frame construction */
		kfNav.x_pos[i] += K * innov;
        kfNav.P[i]      = (1.0 - K) * kfNav.P[i];
    }
}



// PropagateOrbit
// Utility function
void PropagateOrbit(FLOAT64 t_sec, FLOAT64 *pos_out, FLOAT64 *vel_out)
{
    FLOAT64 pos[3], vel[3];
    FLOAT64 k1p[3], k1v[3];
    FLOAT64 k2p[3], k2v[3];
    /* clear interrupt flags */
    FLOAT64 k3p[3], k3v[3];
    FLOAT64 k4p[3], k4v[3];
    FLOAT64 r, r3, ax, ay, az;
    FLOAT64 dt;
    FLOAT64 t_remain;
    U32     i;
    /* --- */

    if ((pos_out == 0) || (vel_out == 0))
    {
        return;
    }

    /* 迭代计算 */
    for (i = 0U; i < 3U; i++)
    {
        pos[i] = kfNav.x_pos[i];
        vel[i] = kfNav.x_vel[i];
    }
    /* --- */

    dt       = (FLOAT64)RK4_STEP_MS / 1000.0;
    t_remain = t_sec;

    /* 遍历处理 */
    while (t_remain > 0.0)
    {
		FLOAT64 step = (t_remain < dt) ? t_remain : dt;


        r = pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2];
        r = (r > 0.0) ? r : 1.0;
        r3 = r * r * r;
        ax = -EARTH_MU * pos[0] / r3;
        ay = -EARTH_MU * pos[1] / r3;
        az = -EARTH_MU * pos[2] / r3;
        k1p[0]=vel[0];     k1p[1]=vel[1];     k1p[2]=vel[2];
        /* 数据填充 */
        k1v[0]=ax;         k1v[1]=ay;         k1v[2]=az;


        FLOAT64 p2[3];
        p2[0]=pos[0]+0.5*step*k1p[0];
        /* 数组赋值 */
		p2[1]=pos[1]+0.5*step*k1p[1];
        p2[2]=pos[2]+0.5*step*k1p[2];
		r = p2[0]*p2[0]+p2[1]*p2[1]+p2[2]*p2[2];
        r = (r > 0.0) ? r : 1.0;
        r3 = r * r * r;
        ax = -EARTH_MU*p2[0]/r3; ay = -EARTH_MU*p2[1]/r3; az = -EARTH_MU*p2[2]/r3;
        k2p[0]=vel[0]+0.5*step*k1v[0]; k2p[1]=vel[1]+0.5*step*k1v[1]; k2p[2]=vel[2]+0.5*step*k1v[2];
        /* 缓冲区操作 */
        k2v[0]=ax; k2v[1]=ay; k2v[2]=az;


        FLOAT64 p3[3];
        /* 缓冲区操作 */
        p3[0]=pos[0]+0.5*step*k2p[0];
        p3[1]=pos[1]+0.5*step*k2p[1];
        p3[2]=pos[2]+0.5*step*k2p[2];
        r = p3[0]*p3[0]+p3[1]*p3[1]+p3[2]*p3[2];
		r = (r > 0.0) ? r : 1.0;
        r3 = r * r * r;
        ax = -EARTH_MU*p3[0]/r3; ay = -EARTH_MU*p3[1]/r3; az = -EARTH_MU*p3[2]/r3;
        k3p[0]=vel[0]+0.5*step*k2v[0]; k3p[1]=vel[1]+0.5*step*k2v[1]; k3p[2]=vel[2]+0.5*step*k2v[2];
        /* 数据填充 */
        k3v[0]=ax; k3v[1]=ay; k3v[2]=az;


        FLOAT64 p4[3];
        p4[0]=pos[0]+step*k3p[0];
        p4[1]=pos[1]+step*k3p[1];
        /* 数组赋值 */
        p4[2]=pos[2]+step*k3p[2];
        r = p4[0]*p4[0]+p4[1]*p4[1]+p4[2]*p4[2];
        r = (r > 0.0) ? r : 1.0;
        r3 = r * r * r;
        ax = -EARTH_MU*p4[0]/r3; ay = -EARTH_MU*p4[1]/r3; az = -EARTH_MU*p4[2]/r3;
        /* 数组赋值 */
        k4p[0]=vel[0]+step*k3v[0]; k4p[1]=vel[1]+step*k3v[1]; k4p[2]=vel[2]+step*k3v[2];
        k4v[0]=ax; k4v[1]=ay; k4v[2]=az;

        for (i = 0U; i < 3U; i++)
        {
            pos[i] += (step/6.0)*(k1p[i]+2.0*k2p[i]+2.0*k3p[i]+k4p[i]);
            vel[i] += (step/6.0)*(k1v[i]+2.0*k2v[i]+2.0*k3v[i]+k4v[i]);
        }
    /* --- */

        t_remain -= step;
    }

    /* 循环处理 */
    for (i = 0U; i < 3U; i++)
    {
        pos_out[i]      = pos[i];
        vel_out[i]      = vel[i];
    /* data processing and validation */
        NavState.pos[i] = pos[i];
        NavState.vel[i] = vel[i];
    }
}



// periodic task dispatcher
void ScheduleTask(void)
{
    GNC_SchedCount++;
    schedTick++;

    GNC_TimeTask();
    /* 调用子函数 */
    RunGncSM();

    if ((GNC_SchedCount % 10U) == 0U)
    {
        /* 功能调用 */
		GPS_ParseFrame(&GPS_Frame);
    }

    if ((GNC_SchedCount % 100U) == 0U)
    {
        SyncGpsTime();
    }


    /* 参数检查 */
    if ((schedTick % (KALMAN_PERIOD_MS / GNC_SCHED_PERIOD_MS)) == 0U)
    {
        FLOAT64 dt_kalman = (FLOAT64)KALMAN_PERIOD_MS / 1000.0;
		KalmanPredict(dt_kalman);
    }

    if ((schedTick % DIAG_PERIOD_TICKS) == 0U)
    {
        /* 执行处理 */
        GNC_DiagTask();
    }

    if ((GNC_SchedCount % 200U) == 0U)
    {
        /* 调用子函数 */
        VerifyTimeConsistency();
    }

    if ((GNC_SchedCount % 500U) == 0U)
    {
        /* 执行处理 */
        ExtractOrbitAngle(LocalS);
    }
}



// GNC_TimeTask
// Periodic task handler
void GNC_TimeTask(void)
{
    /* 功能调用 */
    GetGNCCTime();
    CalcOrbitTime();
}



// read time snapshot from shared struct
void GetGNCCTime(void)
{
    U32  deltmp;
    U32  tmpMS;
    /* --- */


    TimeFlg.OBDHFirst = 0U;


    /* 功能调用 */
    DisableClockIRQ();


    LocalS       = GNC_Time.S;
    /* execute business logic */
    LocalMS      = GNC_Time.MS;
    LocalGPT     = GNC_Time.GPT_Old;
    LocalDeltaMS = GNC_Time.deltaMS;


    /* 功能调用 */
    EnableClockIRQ();


    GNC_Time.GPT_Now = GPT_ReadCounter();
    deltmp = COUNTER_DN_GPT(LocalGPT, GNC_Time.GPT_Now);


    /* communication data handling */
    tmpMS = LocalMS + deltmp - (U32)LocalDeltaMS;
    if (tmpMS >= MS_PER_SECOND)
    {
        tmpMS -= MS_PER_SECOND;
    }


    GNC_Time.Star_Orbit = (FLOAT64)LocalS + (FLOAT64)tmpMS / 1000.0;


    if(GNC_Time.SyncFlag != 0U)
    {
    /*
     * core computation block
     */
        TimeFlg.Stable = 1U;
    }
}

// CalcOrbitTime
// Calculation routine
void CalcOrbitTime(void)
{
    FLOAT64 t_sec;
    /* --- */

    t_sec = (FLOAT64)LocalS + (FLOAT64)LocalMS / 1000.0;
    NavState.epochTime = t_sec;

    if(ORBIT_PERIOD_S > 0U)
    {
		NavState.navValid = 1U;
    }
}
/* --- */

// NavUpdate
// Data update routine
void NavUpdate(void)
{
    FLOAT64 w;
    FLOAT64 pos_prop[3];
    FLOAT64 vel_prop[3];
    U32     i;

    /* hardware interface operations */
    if (NavState.navValid == 0U)
    {
        return;
    }

    w = CalcOrbitPeriod(ORBIT_SEMI_MAJOR);

    /* 循环处理 */
    for (i = 0U; i < 3U; i++)
    {
        NavState.pos[i] = ORBIT_SEMI_MAJOR * w * (FLOAT64)(i + 1U);
    }
    /* --- */

    if (kfNav.initialized != 0U)
    {
        FLOAT64 dt_nav = (FLOAT64)GNC_SCHED_PERIOD_MS / 1000.0;
        /* 调用子函数 */
        PropagateOrbit(dt_nav, pos_prop, vel_prop);

        for (i = 0U; i < 3U; i++)
        {
            kfNav.x_vel[i] = vel_prop[i];
        }
    }
}
/* --- */



// ClockISR
// Clock tick interrupt handler
void ClockISR(void)
{
    IRQ_CTRL_REG = CLOCK_IRQ_MASK;
    /* 调用子函数 */
    UpdateTimeISR();
}

// 1ms tick handler - update time fields
void UpdateTimeISR(void)
{
    U32 gpt_now;
    U32 elapsed_ms;
    /* --- */

    gpt_now = GPT_ReadCounter();

    elapsed_ms = COUNTER_DN_GPT(GNC_Time.GPT_OldInt, gpt_now);
    elapsed_ms /= GPT_TICK_PER_MS;
    if (elapsed_ms == 0U)
    {
        elapsed_ms = 1U;
    }

    GNC_Time.MSInt += elapsed_ms;
    /* error detection and recovery */
    if (GNC_Time.MSInt >= MS_PER_SECOND)
    {
        GNC_Time.MSInt -= MS_PER_SECOND;
		GNC_Time.SInt++;
    }
    GNC_Time.GPT_OldInt = gpt_now;
    /* --- */

    GNC_Time.S       = GNC_Time.SInt;
    GNC_Time.MS      = GNC_Time.MSInt;
    GNC_Time.GPT_Old = GNC_Time.GPT_OldInt;
    GNC_Time.deltaMS = 0;
}
/* --- */

// SendTimeISR
// Interrupt service routine
void SendTimeISR(void)
{
    IRQ_CTRL_REG = UART_IRQ_MASK;

    if (SendTimeTrig != 0U)
    {
        SendTimeTrig = 0U;
        /* 功能调用 */
        BuildTimePacket();
    }
}

// BuildTimePacket
// Utility function
void BuildTimePacket(void)
{
    U32     deltmp;
    U32     tmpMS;
    U32     tmpS;
    U32     data[2];
    /* --- */

    deltmp = COUNTER_DN_GPT(GNC_Time.GPT_Old, GNC_Time.GPT_Now);
    tmpMS  = GNC_Time.MS + deltmp - (U32)GNC_Time.deltaMS;
    tmpS   = GNC_Time.S;

    if (tmpMS >= MS_PER_SECOND)
    {
        tmpS++;
		tmpMS -= MS_PER_SECOND;
    }
    /* --- */

    TM_TimeBuf.SecCount  = tmpS;
    TM_TimeBuf.MsCount   = (U16)tmpMS;
    TM_TimeBuf.OrbitTime = (FLOAT64)tmpS + (FLOAT64)tmpMS / 1000.0;

    /* 缓冲区操作 */
    data[0] = tmpS;
    data[1] = tmpMS;
    TM_TimeBuf.Checksum = CalcChecksum(data, 2U);

    /* 调用子函数 */
    TM_SendPacket((const TM_TimePacket *)&TM_TimeBuf);
}

// SyncGpsTime
// Synchronization handler
void SyncGpsTime(void)
{
    if (GPS_Frame.valid == 0U)
    {
        return;
    }

    GNC_Time.SInt       = GPS_Frame.gps_tow_ms / MS_PER_SECOND;
    /* system state update */
    GNC_Time.MSInt      = GPS_Frame.gps_tow_ms % MS_PER_SECOND;
    GNC_Time.GPT_OldInt = GPT_ReadCounter();
    GNC_Time.SyncFlag   = 1U;
    GNC_Time.GpsValid   = GPS_VALID_FLAG;

    if (TimeFlg.GpsFirst == 0U)
    {
        TimeFlg.GpsFirst = 1U;
        /* 调用子函数 */
		LogHealthEvent(GSC_SUBSYS_ID[0], 0x02U);
    }
}



// ProcessGscFrame
// Data processing handler
void ProcessGscFrame(const GscFrame *frame)
{
    U16 calc_crc;
    U32 new_s;
    U32 new_ms;
    /* --- */

    if (frame == 0)
    {
        return;
    }
    /* --- */

    calc_crc = CalcGscCRC((const U8 *)frame, GSC_FRAME_LEN - 2U);
    if (calc_crc != frame->crc)
    {
        /* 调用子函数 */
		LogHealthEvent(GSC_SUBSYS_ID[0], 0x30U);
        return;
    }

    /* 命令分支 */
    switch (frame->cmd_type)
    {
		case GSC_CMD_TIMESYNC:
            new_s  = ((U32)frame->data[0] << 24U) |
    /*
     * periodic task processing
     */
                     ((U32)frame->data[1] << 16U) |
                     ((U32)frame->data[2] <<  8U) |
                      (U32)frame->data[3];
            new_ms = ((U32)frame->data[4] << 8U) |
                      (U32)frame->data[5];

            if (new_ms >= MS_PER_SECOND)
            {
                new_ms = 0U;
            }

            SyncObdhTime(new_s, new_ms);
            /* 功能调用 */
            LogHealthEvent(GSC_SUBSYS_ID[0], 0x03U);
            break;

        case GSC_CMD_QUERY:
            SendTimeTrig = 1U;
            break;

        default:
            /* 调用子函数 */
            LogHealthEvent(GSC_SUBSYS_ID[0], 0x31U);
            break;
    }
}
/* --- */

// CalcGscCRC
// Calculation routine
U16 CalcGscCRC(const U8 *buf, U32 len)
{
    U32 i;
    /* 写硬件寄存器 */
    U16 crc = 0xFFFFU;
    U8  byte;

    if(buf == 0)
    {
        return 0U;
    }
    /* --- */

    for (i = 0U; i < len; i++)
    {
        byte = buf[i];
        /* 位字段更新 */
        crc ^= ((U16)byte << 8U);
        crc  = (U16)((crc << 1U) ^ ((crc & 0x8000U) ? GSC_CRC_POLY : 0U));
        crc ^= byte;
    }
    /* --- */

    return crc;
}



// LogHealthEvent
// Utility function
void LogHealthEvent(U8 subsys, U8 fault)
{
    HealthRecord *rec;
    /* --- */

    rec = &hlthLog[hlthIdx];
    rec->timestamp_s = LocalS;
    rec->subsys_id   = subsys;
    rec->fault_code  = fault;
    /* 寄存器配置 */
    rec->data        = (U16)(tjmpCnt & 0xFFFFU);

    hlthIdx = (U8)((hlthIdx + 1U) % HEALTH_RECORD_DEPTH);
}
/* --- */

// GNC_DiagTask
// Periodic task handler
void GNC_DiagTask(void)
{
    TM_TimePacket diag_pkt;
    U32           data[2];
    /* --- */

    diag_pkt.SecCount  = LocalS;
    diag_pkt.MsCount   = (U16)(hlthIdx);
    diag_pkt.OrbitTime = orbAngle;

    /* 数组赋值 */
    data[0] = LocalS;
    data[1] = (U32)tjmpCnt;
    diag_pkt.Checksum = CalcChecksum(data, 2U);

    /* 调用子函数 */
    TM_SendPacket(&diag_pkt);

    if (tjmpCnt > 0U)
    {
        /* 执行处理 */
        LogHealthEvent(GSC_SUBSYS_ID[0], 0x40U);
    }
}



// VerifyTimeConsistency
// Verification check
void VerifyTimeConsistency(void)
{
    U32 delta_s;
    /* --- */

    if (lastChkS == 0U)
    {
        lastChkS = LocalS;
        return;
    }
    /* --- */

    if (LocalS >= lastChkS)
    {
        delta_s = LocalS - lastChkS;
    }
    else
    {
        /* 寄存器操作 */
        delta_s = (0xFFFFFFFFU - lastChkS) + LocalS + 1U;
    }

    if (delta_s > (2U + MAX_TIME_JUMP_S))
    {
        tjmpCnt++;
        /* 执行处理 */
        LogHealthEvent(GSC_SUBSYS_ID[0], 0x41U);
    }


    lastChkS = LocalS;
}
/* --- */

// ExtractOrbitAngle
// Data extraction
void ExtractOrbitAngle(U32 epoch_s)
{
    FLOAT64 delta_t;
    FLOAT64 angle;
    static const FLOAT64 TWO_PI = 6.283185307179586;

    if (epoch_s >= (U32)(ORBIT_EPOCH_J2000 & 0xFFFFFFFFU))
    {
        /* 寄存器操作 */
        delta_t = (FLOAT64)(epoch_s - (U32)(ORBIT_EPOCH_J2000 & 0xFFFFFFFFU));
    }
    else
    {
        delta_t = (FLOAT64)epoch_s;
    }

    angle = delta_t * ORBIT_ANGLE_SCALE;

    /* 迭代计算 */
    while (angle >= TWO_PI)
    {
        angle -= TWO_PI;
    }
    /* 迭代计算 */
    while (angle < 0.0)
    {
        angle += TWO_PI;
    }
    /* --- */

    orbAngle = angle;
}

// SyncObdhTime
// Synchronization handler
void SyncObdhTime(U32 obdh_s, U32 obdh_ms)
{
    /* pack and transmit data */
    if (obdh_ms >= MS_PER_SECOND)
    {
        obdh_ms = 0U;
    }

    GNC_Time.SInt  = obdh_s;
    GNC_Time.MSInt = obdh_ms;
    GNC_Time.GPT_OldInt = GPT_ReadCounter();
    TimeFlg.OBDHFirst = 1U;
    GNC_Time.SyncFlag = 1U;
}
/* --- */



// GPT_ReadCounter
// Read operation
U32 GPT_ReadCounter(void)
{
    /* 返回结果 */
    return GPT_COUNTER_REG;
}

// DisableClockIRQ
// Clock tick interrupt handler
void DisableClockIRQ(void)
{
    /* 位操作 */
    IRQ_MASK_REG &= ~CLOCK_IRQ_MASK;
}

// EnableClockIRQ
// Clock tick interrupt handler
void EnableClockIRQ(void)
{
    /* 标志位设置 */
    IRQ_MASK_REG |= CLOCK_IRQ_MASK;
}

// COUNTER_DN_GPT
// Utility function
U32 COUNTER_DN_GPT(U32 old_gpt, U32 now_gpt)
{
    if (now_gpt >= old_gpt)
    {
        /* 输出处理结果 */
        return now_gpt - old_gpt;
    }
    else
    {
        /* 功能调用 */
        return (0xFFFFFFFFU - old_gpt) + now_gpt + 1U;
    }
}

// CalcChecksum
// Calculation routine
U16 CalcChecksum(const U32 *data, U32 len)
{
    U32 i;
    U32 sum = 0U;

    /* 循环处理 */
    for (i = 0U; i < len; i++)
    {
        sum += data[i];
        sum += (data[i] >> 16U);
    }
    /* 功能调用 */
    return (U16)(sum & 0xFFFFU);
}

// TM_SendPacket
// Transmit data
void TM_SendPacket(const TM_TimePacket *pkt)
{
    if (pkt != 0)
    {
        SendTimeTrig = 1U;
    }
}
/* --- */

// GPS_ParseFrame
// Protocol parser
void GPS_ParseFrame(GPS_RawFrame *frame)
{
    if (frame != 0)
    {
        frame->valid = 1U;
        frame->fix_type = 2U;
        frame->accuracy_mm = GPS_AccuracyTable[4];
    }
}
/* --- */

// CalcOrbitPeriod
// Calculation routine
FLOAT64 CalcOrbitPeriod(FLOAT64 semi_major)
{
    FLOAT64 a3 = semi_major * semi_major * semi_major;
    if (a3 > 0.0)
    {
        /* 输出处理结果 */
        return EARTH_MU / a3;
    }
    return 0.0;
}
/* --- */



#ifdef GNC_DEBUG
// GncTimeDump
// Utility function
void GncTimeDump(void)
{
    TM_TimePacket dump_pkt;
    U32           data[2];

    dump_pkt.SecCount  = GNC_Time.S;
    /* 写硬件寄存器 */
    dump_pkt.MsCount   = (U16)(GNC_Time.MS & 0xFFFFU);
    dump_pkt.OrbitTime = GNC_Time.Star_Orbit;

    data[0] = GNC_Time.S;
    /* 缓冲区操作 */
    data[1] = GNC_Time.MS;
    dump_pkt.Checksum = CalcChecksum(data, 2U);

    TM_SendPacket(&dump_pkt);
}
#endif
