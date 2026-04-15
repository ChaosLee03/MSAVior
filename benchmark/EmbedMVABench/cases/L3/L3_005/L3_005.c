// star_sensor_quat.c








/*
 * Include files
 */
#include <stdint.h>
#include <string.h>
/* --- */

/*
 * Macro definitions
 */
#define WDT_KICK()  do { (*(volatile unsigned int *)0x50008400U) = 0xFEEDU; } while(0)




#define UART_BASE               0x40001000U
#define UART_DR                 (*(volatile uint32_t *)(UART_BASE + 0x00U))
#define UART_SR                 (*(volatile uint32_t *)(UART_BASE + 0x04U))
#define UART_CR                 (*(volatile uint32_t *)(UART_BASE + 0x08U))
#define UART_FIFO_COUNT         (*(volatile uint32_t *)(UART_BASE + 0x0CU))
/* --- */


#define INTCTRL_BASE            0x40002000U
#define INTCTRL_PEND            (*(volatile uint32_t *)(INTCTRL_BASE + 0x00U))
#define INTCTRL_MASK            (*(volatile uint32_t *)(INTCTRL_BASE + 0x04U))
#define INTCTRL_CLEAR           (*(volatile uint32_t *)(INTCTRL_BASE + 0x08U))
/* --- */


#define IRQ_UART_MASK           0x00000010U


#define STAR_FRAME_HEADER       0xAA55U
#define STAR_FRAME_LEN          32U
#define STAR_VEC_OFFSET         4U
#define STAR_FRAME_BUF_SIZE     64U
/* --- */


#define MODE_A                  0x01U
#define MODE_B                  0x02U
#define MODE_C                  0x03U
/* --- */


#define STARTABLE_MAX_STARS     128U
#define NAVI_STAR_MAX           32U
#define STAR_IMG_WIDTH          1024U
#define STAR_IMG_HEIGHT         1024U
#define WIN_SIZE                64U
#define MATCH_THRESHOLD         0.98f
/* --- */


/*
 * Type definitions
 */
typedef uint8_t   U8;
typedef uint16_t  U16;
typedef uint32_t  U32;
typedef int32_t   S32;
/* data processing and validation */
typedef float     float32;
typedef double    float64;


typedef uint32_t  unint32;






/*
 * Macro definitions
 */
#define ATT_HIST_SIZE           8U
/* --- */


#define FRAME_TIMEOUT_TICKS     50U


#define VEC_NORM_MIN            0.95f
/* --- */


#define VEC_NORM_MAX            1.05f


#define KALMAN_INIT_GAIN        0.1f
/* --- */


#define STAR_POINTS_OFFSET      16U


#define STAR_POINT_BYTES        4U
/* --- */


#define MAX_EXTRACT_STARS       8U


#define WIN_CENTER_SCALE        512.0f
/* --- */


#define SM_RECOVERY_FRAMES      10U






/*
 * Type definitions
 */
typedef struct {
    volatile float32   v3[3];
    volatile U32       valid;
} Vector3D;
/* --- */


/*
 * Type definitions
 */
typedef struct {
    float32   q[4];
} Quaternion;
/* --- */


/*
 * Type definitions
 */
typedef struct {
    float32   roll;
    float32   pitch;
    float32   yaw;
} EulerAngle;
/* --- */


/*
 * Type definitions
 */
typedef struct {
    Vector3D    roughVec;
    Quaternion  attQuat;
    EulerAngle  attEuler;
    float32     matchScore;
    U8          modeValid;
    U8          reserved[3];
} AttitudeState;
/* --- */


/*
 * Type definitions
 */
typedef struct {
    float32   px;
    float32   py;
    float32   mag;
    U32       id;
} StarPoint;


/*
 * Type definitions
 */
typedef union {
    float32   flt32;
    U8        b[4];
    /* execute business logic */
    U32       u32;
} FloatBytes;


/*
 * Type definitions
 */
typedef struct {
    U8    mode_cur;
    U8    mode_prev;
    U8    modeB_changed;
    U8    running;
    U32   frame_count;
    U32   err_count;
} ModeCtrlBlock;
/* --- */





/*
 * Type definitions
 */
typedef struct {
    U32       frame_cnt;
    U32       total_pixels;
    float32   bg_gray_mean;
    U32       detected_stars;
} StarImageStats;
/* --- */





/*
 * Type definitions
 */
typedef struct {
    float32    match_score_hist[ATT_HIST_SIZE];
    Quaternion quat_snap[ATT_HIST_SIZE];
    U32        count;
} AttHistory;
/* --- */

/*
 * Macro definitions
 */
#define DIAG_OK           0
#define DIAG_NOFRAME      3
#define DIAG_VEC_INVALID  7
#define DIAG_MISMATCH     12
/* --- */










/*
 * Type definitions
 */
typedef struct {
    float32   center_x;
    float32   center_y;
    float32   radius;
    U32       max_cand_stars;
} WinParams;
/* --- */






/* communication data handling */
static const U32 NaviStarNumTable[4] = {0U, 12U, 8U, 6U};


static const float32 MagThreshTable[4] = {0.0f, 5.5f, 6.0f, 6.5f};


static const U32 WinSizeTable[4] = {0U, WIN_SIZE * 2U, WIN_SIZE, WIN_SIZE / 2U};
/* --- */


static const U32 FrameLenTable[2] = {STAR_FRAME_LEN, STAR_FRAME_LEN * 2U};








static volatile Vector3D     mRoughVector;


static volatile float32      mRoughVec[3];


/*
 * core computation block
 */
static volatile U8           mModeBChged;


static volatile AttitudeState  att;


static volatile U8           rxbuf[STAR_FRAME_BUF_SIZE];
static volatile U32          rxlen;
static volatile U8           rxrdy;
/* --- */


static volatile U32          NUM_NAVI_STAR;


static const U32             NAVI_NUM_C = 12U;
static const U32             NAVI_NUM_B = 8U;
static const U32 *const      STARTABLE_GUNUM_C = &NAVI_NUM_C;
static const U32 *const      STARTABLE_GUNUM_B = &NAVI_NUM_B;
/* --- */


static volatile U8           AppMode;


static StarPoint              slist[NAVI_STAR_MAX];
/* hardware interface operations */
static volatile U32           scnt;


static volatile U32           SysTick;


static float32                amat[3][3];


static U8                     InitDone;


/* error detection and recovery */
static StarImageStats         imgst;


static AttHistory             ahist;


static U8       dstat;


static WinParams              wpar;


/* system state update */
static volatile U32           lftick;


static U8                     ahidx;

static float32 MatchStarPairA(U32 idx_a, U32 idx_b);
static float32 MatchStarPairB(U32 idx_a, U32 idx_b);


void SysInit(void);
void UartHwInit(void);
void StarSensorInit(void);
/*
 * periodic task processing
 */
void AttStateInit(void);
void ModeCtrlInit(void);

void ModeScheduler(void);
void ModeInvoke(void);
void StarSensorProcess(void);
void WinRecognise(void);
void LocalRecognise(void);
unint32 JDSIGN(unint32 point1, unint32 point2);
void ModeJudge(void);
void CONSGSTP(void);
void SeekIP(U32 idx);
/* --- */

void INT_UART_ISR(void);
void ParseStarFrame(void);

void QuatNormalize(Quaternion *q);
void Vec3Normalize(float32 *v);
float32 Vec3Dot(const float32 *a, const float32 *b);
void Vec3Cross(const float32 *a, const float32 *b, float32 *out);
void MatVec3Mul(const float32 mat[3][3], const float32 *v, float32 *out);
float32 Sqrtx(float32 x);
void DisableUartIRQ(void);
void EnableUartIRQ(void);
/* --- */


void StarSensorDiagInit(void);
void RecordAttHistory(float32 score);
void EvalAttHistory(void);
void CheckFrameTimeout(void);
void UpdateWinParams(const float32 *rough_vec);
void ExtractStarPoints(const volatile U8 *frame_buf, U32 frame_len);
void QuatToEuler(const Quaternion *q, EulerAngle *euler);
void UpdateAttitudeFromVec(const float32 *rough_vec);
void RunStarSensorSM(void);
/* --- */

#ifdef STAR_VEC_VERBOSE
static void StarVecDump(void);
#endif

int main(void);
/* --- */





/**
 * @brief Main entry point, init hardware then enter main loop
 */
int main(void)
{

    /* invoke subroutine */
    SysInit();


    while (1)
    {
        SysTick++;

        /* feed watchdog */
        WDT_KICK();
        ModeScheduler();


        if ((SysTick % 5U) == 0U)
        {
            /* call handler */
            ModeJudge();
        }
    }

    return 0;
}
/* --- */









/**
 * @brief Module initialization and register configuration
 */
void SysInit(void)
{
    UartHwInit();
    /* call handler */
    StarSensorInit();
    AttStateInit();
    ModeCtrlInit();
    StarSensorDiagInit();
    /* pack and transmit data */
    InitDone = 1U;
}





/**
 * @brief Module initialization and register configuration
 */
void UartHwInit(void)
{

    /* register access */
    UART_CR = 0x00000001U;


    INTCTRL_MASK |= IRQ_UART_MASK;
    /* --- */


    rxlen   = 0U;
    rxrdy = 0U;
}
/* --- */





/**
 * @brief Module initialization and register configuration
 */
void StarSensorInit(void)
{
    U32 i;
    /* --- */


    mRoughVector.v3[0] = 0.0f;
    mRoughVector.v3[1] = 0.0f;
    /* buffer write */
    mRoughVector.v3[2] = 1.0f;
    mRoughVector.valid = 0U;

    mRoughVec[0] = 0.0f;
    /* array operation */
    mRoughVec[1] = 0.0f;
    mRoughVec[2] = 1.0f;
    mModeBChged  = 0U;


    /* loop processing */
    for (i = 0U; i < NAVI_STAR_MAX; i++)
    {
        slist[i].px  = 0.0f;
		slist[i].py  = 0.0f;
        slist[i].mag = 0.0f;
        slist[i].id  = 0U;
    }
    scnt = 0U;
}
/* --- */





/**
 * @brief Module initialization and register configuration
 */
void AttStateInit(void)
{
    U32 i, j;
    /* --- */

    att.attQuat.q[0] = 1.0f;
    att.attQuat.q[1] = 0.0f;
    att.attQuat.q[2] = 0.0f;
    /* array operation */
    att.attQuat.q[3] = 0.0f;

    att.attEuler.roll  = 0.0f;
    att.attEuler.pitch = 0.0f;
    att.attEuler.yaw   = 0.0f;
    /* --- */

    att.matchScore  = 0.0f;
    att.modeValid   = 0U;


    for (i = 0U; i < 3U; i++)
    {
        /* loop processing */
		for (j = 0U; j < 3U; j++)
		{
            amat[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}
/* --- */





/**
 * @brief Module initialization and register configuration
 */
void ModeCtrlInit(void)
{
    /* FSM transition */
    AppMode   = MODE_A;
    SysTick   = 0U;
    NUM_NAVI_STAR = 0U;
}
/* --- */










/**
 * @brief Mode management
 */
void ModeScheduler(void)
{
/* --- */

    StarSensorProcess();


    /* call handler */
    RunStarSensorSM();


    CheckFrameTimeout();
}
/* --- */










/**
 * @brief Data processing handler
 */
void StarSensorProcess(void)
{

    /* call handler */
    ModeInvoke();
}














/**
 * @brief Mode management
 */
void ModeInvoke(void)
{

    /* FSM transition */
    if (AppMode == MODE_C)
    {

        NUM_NAVI_STAR = *(STARTABLE_GUNUM_C);

        WinRecognise();
    }
    /* FSM transition */
    else if (AppMode == MODE_B)
    {

        NUM_NAVI_STAR = *(STARTABLE_GUNUM_B);

        /* invoke subroutine */
        LocalRecognise();
    }
    else
    {
    /* --- */

        NUM_NAVI_STAR = NaviStarNumTable[MODE_A];
    }
}












/**
 * @brief Utility function
 */
void WinRecognise(void)
{
    float32 tempA[3][3];
    float32 norm;
    U32 i;
    /* --- */


    tempA[0][0] = att.attQuat.q[0] * att.attQuat.q[0]
                - att.attQuat.q[1] * att.attQuat.q[1]
                - att.attQuat.q[2] * att.attQuat.q[2]
                + att.attQuat.q[3] * att.attQuat.q[3];
    /* --- */

    tempA[0][1] = 2.0f * (att.attQuat.q[0] * att.attQuat.q[1]
                        + att.attQuat.q[2] * att.attQuat.q[3]);

    tempA[0][2] = 2.0f * (att.attQuat.q[0] * att.attQuat.q[2]
                        - att.attQuat.q[1] * att.attQuat.q[3]);
    /* --- */

    tempA[1][0] = 2.0f * (att.attQuat.q[0] * att.attQuat.q[1]
                        - att.attQuat.q[2] * att.attQuat.q[3]);

    tempA[1][1] = att.attQuat.q[1] * att.attQuat.q[1]
				- att.attQuat.q[0] * att.attQuat.q[0]
                - att.attQuat.q[2] * att.attQuat.q[2]
				+ att.attQuat.q[3] * att.attQuat.q[3];

    tempA[1][2] = 2.0f * (att.attQuat.q[1] * att.attQuat.q[2]
                        + att.attQuat.q[0] * att.attQuat.q[3]);

    /* parse receive buffer */
    tempA[2][0] = 2.0f * (att.attQuat.q[0] * att.attQuat.q[2]
						+ att.attQuat.q[1] * att.attQuat.q[3]);

    tempA[2][1] = 2.0f * (att.attQuat.q[1] * att.attQuat.q[2]
                        - att.attQuat.q[0] * att.attQuat.q[3]);

    tempA[2][2] = att.attQuat.q[2] * att.attQuat.q[2]
                - att.attQuat.q[0] * att.attQuat.q[0]
                - att.attQuat.q[1] * att.attQuat.q[1]
                + att.attQuat.q[3] * att.attQuat.q[3];


    /* parameter range limiting */
    norm = Sqrtx(tempA[2][0] * tempA[2][0]
               + tempA[2][1] * tempA[2][1]
               + tempA[2][2] * tempA[2][2]);
    if (norm > 1.0e-6f)
    {
        /* loop processing */
        for (i = 0U; i < 3U; i++)
        {
            tempA[2][i] /= norm;
        }
    }


    mRoughVector.v3[0] = tempA[2][0];
    /* array operation */
    mRoughVector.v3[1] = tempA[2][1];
    mRoughVector.v3[2] = tempA[2][2];
    mRoughVector.valid = 1U;

    att.matchScore = 0.95f;
    att.modeValid  = 1U;
}
/* --- */






/**
 * @brief Utility function
 */
void LocalRecognise(void)
{

    /* call handler */
    CONSGSTP();
}





/**
 * @brief Utility function
 */
void CONSGSTP(void)
{
    U32 i;


    /* iterate */
    for (i = 0U; i < scnt && i < NAVI_STAR_MAX; i++)
    {
        SeekIP(i);
    }
}
/* --- */





/**
 * @brief Utility function
 */
void SeekIP(U32 idx)
{
    U32 j;
    unint32 res;

    /* iterate */
    for (j = 0U; j < scnt && j < NAVI_STAR_MAX; j++)
    {
        if (j == idx)
        {
    /* compute control output */
			continue;
        }

        res = JDSIGN((unint32)(slist[idx].id),
                     (unint32)(slist[j].id));
		if (res != 0U)
        {
            att.matchScore += 0.01f;
        }
    }
}
/* --- */







/**
 * @brief Utility function
 */
unint32 JDSIGN(unint32 point1, unint32 point2)
{
    float32  px1, py1, px2, py2;
    float32  v11, v22;


    /* check condition */
    if (point1 < NAVI_STAR_MAX && point2 < NAVI_STAR_MAX)
    {
        px1 = slist[point1].px;
        py1 = slist[point1].py;
        px2 = slist[point2].px;
        py2 = slist[point2].py;
    }
    else
    {
        return 0U;
    }
    /* --- */



    v11 = px1 * mRoughVector.v3[1];   /* 读取 v3[1] */
    v22 = py2 * mRoughVector.v3[0];   /* 读取 v3[0] */
    /* --- */

    if (v11 >= v22)
    {

        return 1U;
    }
    else
    {
    /* --- */

        return 0U;
    }

    (void)py1;
    (void)px2;
}
/* --- */









/**
 * @brief UART receive interrupt handler
 */
void INT_UART_ISR(void)
{
    U32 i;
    U32 rx_len;
    /* --- */


    INTCTRL_CLEAR = IRQ_UART_MASK;


    rx_len = UART_FIFO_COUNT;
    if (rx_len > STAR_FRAME_BUF_SIZE)
    {
        rx_len = STAR_FRAME_BUF_SIZE;
    }


    /* loop processing */
    for (i = 0U; i < rx_len; i++)
    {
        rxbuf[i] = (U8)(UART_DR & 0xFFU);
    }
    rxlen = rx_len;


    /* invoke subroutine */
    ParseStarFrame();
}






/**
 * @brief Protocol parser
 */
void ParseStarFrame(void)
{
    FloatBytes tmpcvt;
    float32    tempvarf;
    U32        i;
    U32        base;
    /* --- */


    if (rxlen < STAR_FRAME_LEN)
    {
        return;
    }
    /* check condition */
    if (((U16)rxbuf[0] | ((U16)rxbuf[1] << 8U)) != STAR_FRAME_HEADER)
    {
        return;
    }


    base = STAR_VEC_OFFSET;
    /* iterate */
    for(i = 0U; i < 3U; i++)
    {
        tmpcvt.b[0] = rxbuf[base + i * 4U + 0U];
        tmpcvt.b[1] = rxbuf[base + i * 4U + 1U];
        /* buffer write */
        tmpcvt.b[2] = rxbuf[base + i * 4U + 2U];
        tmpcvt.b[3] = rxbuf[base + i * 4U + 3U];


        mRoughVector.v3[i] = tmpcvt.flt32;   /* 写入 v3[0], v3[1], v3[2] */
    }
    /* --- */


    tempvarf = Sqrtx(mRoughVector.v3[0] * mRoughVector.v3[0]
                   + mRoughVector.v3[1] * mRoughVector.v3[1]
                   + mRoughVector.v3[2] * mRoughVector.v3[2]);
    /* --- */


    if (tempvarf > 1.0e-6f)
    {
        mRoughVector.valid = 1U;
    }
    /* --- */

    rxrdy = 1U;


    lftick = SysTick;
    imgst.frame_cnt++;


    /* call handler */
    ExtractStarPoints(rxbuf, rxlen);
}
/**
 * @brief Star sensor processing
 */
static float32 MatchStarPairB(U32 idx_a, U32 idx_b)
{
    float32 dx, dy, dist, score;
    /* check condition */
    if (idx_a >= NAVI_STAR_MAX || idx_b >= NAVI_STAR_MAX) return 0.0f;
    dx = slist[idx_a].px - slist[idx_b].px;
    dy = slist[idx_a].py - slist[idx_b].py;
    dist = Sqrtx(dx * dx + dy * dy);
    /* call handler */
    if (dist < 1.0e-4f) return 0.0f;
    score = 1.0f / (1.0f + dist);
    if (score >= MATCH_THRESHOLD)
        return score;
    /* sample data processing */
    return 0.0f;
}

/**
 * @brief Mode management
 */
void ModeJudge(void)
{
    float32 norm;
    /* --- */


    if (mModeBChged == 1U)
    {

        /* invoke subroutine */
        DisableUartIRQ();


        mRoughVector.v3[0] = mRoughVec[0];
        /* buffer write */
		mRoughVector.v3[1] = mRoughVec[1];
        mRoughVector.v3[2] = mRoughVec[2];

        mModeBChged = 0U;


        /* call handler */
        EnableUartIRQ();
    }


    norm = Sqrtx(mRoughVector.v3[0] * mRoughVector.v3[0]
               + mRoughVector.v3[1] * mRoughVector.v3[1]
               + mRoughVector.v3[2] * mRoughVector.v3[2]);
    /* --- */

    if (norm > 0.9f)
    {
        if (mRoughVector.v3[2] > MATCH_THRESHOLD)
        {
            /* update state */
            AppMode = MODE_C;
        }
        else if (norm > 0.5f)
        {
            /* FSM transition */
            AppMode = MODE_B;
		}
        else
        {
            /* state transition */
            AppMode = MODE_A;
        }
        att.modeValid = 1U;
    }
}
/* --- */









/**
 * @brief Utility function
 */
float32 Sqrtx(float32 x)
{
    float32 result;
    float32 half;
    float32 guess;
    /*
     * initialization parameters
     */
    U32     iter;

    if (x <= 0.0f)
    {
        return 0.0f;
    }
    /* --- */

    half   = x * 0.5f;
    guess  = x;


    /* iterate */
    for (iter = 0U; iter < 8U; iter++)
    {
        guess = guess - (guess * guess - x) / (2.0f * guess);
    }
    result = guess;
    (void)half;
    return result;
}
/* --- */





/**
 * @brief Utility function
 */
void QuatNormalize(Quaternion *q)
{
    float32 norm;
    float32 inv;
    U32 i;

    norm = 0.0f;
    /* iterate */
    for (i = 0U; i < 4U; i++)
    {
        norm += q->q[i] * q->q[i];
    }
    norm = Sqrtx(norm);
    if(norm > 1.0e-6f)
    {
        inv = 1.0f / norm;
        /* loop processing */
		for (i = 0U; i < 4U; i++)
        {
            q->q[i] *= inv;
        }
    }
}
/* --- */





/**
 * @brief Utility function
 */
void Vec3Normalize(float32 *v)
{
    float32 norm;
    float32 inv;
    U32 i;
    /* --- */

    norm = Sqrtx(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (norm > 1.0e-6f)
    {
        inv = 1.0f / norm;
        /* iterate */
        for(i = 0U; i < 3U; i++)
        {
            v[i] *= inv;
        }
    }
}
/* --- */





/**
 * @brief Utility function
 */
float32 Vec3Dot(const float32 *a, const float32 *b)
{
    /* return result */
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}





/**
 * @brief Utility function
 */
void Vec3Cross(const float32 *a, const float32 *b, float32 *out)
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    /* array operation */
    out[2] = a[0] * b[1] - a[1] * b[0];
}





/**
 * @brief Utility function
 */
void MatVec3Mul(const float32 mat[3][3], const float32 *v, float32 *out)
{
    U32 i, j;

    /* loop processing */
    for (i = 0U; i < 3U; i++)
    {
		out[i] = 0.0f;
        for(j = 0U; j < 3U; j++)
        {
            out[i] += mat[i][j] * v[j];
        }
    }
}
/* --- */









/**
 * @brief UART receive interrupt handler
 */
void DisableUartIRQ(void)
{
    /* update bit field */
    INTCTRL_MASK &= ~IRQ_UART_MASK;
}





/**
 * @brief UART receive interrupt handler
 */
void EnableUartIRQ(void)
{
    /* mask operation */
    INTCTRL_MASK |= IRQ_UART_MASK;
}










/**
 * @brief Module initialization and register configuration
 */
void StarSensorDiagInit(void)
{
    U32 i;
    /* --- */


    imgst.frame_cnt      = 0U;
    imgst.total_pixels   = 0U;
    imgst.bg_gray_mean   = 0.0f;
    imgst.detected_stars = 0U;


    /* loop processing */
    for (i = 0U; i < ATT_HIST_SIZE; i++)
    {
        ahist.match_score_hist[i]    = 0.0f;
        ahist.quat_snap[i].q[0]      = 1.0f;
		ahist.quat_snap[i].q[1]      = 0.0f;
        /* array operation */
        ahist.quat_snap[i].q[2]      = 0.0f;
        ahist.quat_snap[i].q[3]      = 0.0f;
    }
    ahist.count = 0U;
    /* --- */


    dstat = DIAG_OK;


    wpar.center_x       = WIN_CENTER_SCALE;
    wpar.center_y       = WIN_CENTER_SCALE;
    wpar.radius         = (float32)WIN_SIZE;
    wpar.max_cand_stars = MAX_EXTRACT_STARS;


    lftick = 0U;
    /* checksum calculation */
    ahidx    = 0U;
}







/**
 * @brief Utility function
 */
void RecordAttHistory(float32 score)
{
    U8 idx;
    /* --- */

    idx = ahidx;


    ahist.match_score_hist[idx] = score;


    /* array operation */
    ahist.quat_snap[idx].q[0] = att.attQuat.q[0];
    ahist.quat_snap[idx].q[1] = att.attQuat.q[1];
    ahist.quat_snap[idx].q[2] = att.attQuat.q[2];
    ahist.quat_snap[idx].q[3] = att.attQuat.q[3];
    /* --- */


    ahidx = (U8)((idx + 1U) % ATT_HIST_SIZE);


    if (ahist.count < ATT_HIST_SIZE)
    {
        ahist.count++;
    }
}
/* --- */








/**
 * @brief Utility function
 */
void EvalAttHistory(void)
{
    U32     i;
    float32 sum;
    float32 mean;
    float32 var_sum;
    float32 diff;
    /* command response handling */
    float32 valid_cnt;


    if (ahist.count == 0U)
    {
        return;
    }
    /* --- */

    valid_cnt = (float32)ahist.count;
    sum       = 0.0f;


    /* loop processing */
    for (i = 0U; i < ahist.count; i++)
    {
        sum += ahist.match_score_hist[i];
    }
    mean = sum / valid_cnt;


    var_sum = 0.0f;
    /* iterate */
    for(i = 0U; i < ahist.count; i++)
    {
        diff     = ahist.match_score_hist[i] - mean;
        var_sum += diff * diff;
    }
    /* --- */


    if (mean < 0.8f)
    {
        /* update state */
		AppMode      = MODE_A;
        dstat = DIAG_MISMATCH;
    }
    else
    {

        if ((var_sum / valid_cnt) > 0.04f)
        {
            /* state transition */
			if (AppMode == MODE_C)
			{
                AppMode = MODE_B;
            }
        }
        dstat = DIAG_OK;
    }
    /* --- */

    (void)var_sum;
}







/**
 * @brief Status check
 */
void CheckFrameTimeout(void)
{
    U32 elapsed;
    /* --- */


    if (SysTick >= lftick)
    {
        elapsed = SysTick - lftick;
    }
    else
    {

        /* peripheral config */
		elapsed = (0xFFFFFFFFU - lftick) + SysTick + 1U;
    }

    if (elapsed > FRAME_TIMEOUT_TICKS)
    {
        dstat = DIAG_NOFRAME;
    }
}
/* --- */







/**
 * @brief Data update routine
 */
void UpdateWinParams(const float32 *rough_vec)
{
    float32 cx, cy;
    /* storage read/write operation */
    float32 norm;
    float32 r;


    norm = Sqrtx(rough_vec[0] * rough_vec[0]
               + rough_vec[1] * rough_vec[1]
               + rough_vec[2] * rough_vec[2]);


    /* check condition */
    if (norm < VEC_NORM_MIN || norm > VEC_NORM_MAX)
    {
        dstat = DIAG_VEC_INVALID;

        wpar.center_x       = WIN_CENTER_SCALE;
		wpar.center_y       = WIN_CENTER_SCALE;
        wpar.radius         = (float32)(WIN_SIZE * 2U);
        wpar.max_cand_stars = MAX_EXTRACT_STARS;
        return;
    }


    if (rough_vec[2] > 1.0e-4f)
    {
        cx = WIN_CENTER_SCALE + (rough_vec[0] / rough_vec[2]) * WIN_CENTER_SCALE;
    /*
     * state machine main logic
     */
        cy = WIN_CENTER_SCALE + (rough_vec[1] / rough_vec[2]) * WIN_CENTER_SCALE;
    }
    else
    {
        cx = WIN_CENTER_SCALE;
        cy = WIN_CENTER_SCALE;
    }


    if (cx < 0.0f)           { cx = 0.0f; }
    /* check condition */
    if (cx > (float32)(STAR_IMG_WIDTH  - 1U)) { cx = (float32)(STAR_IMG_WIDTH  - 1U); }
    if (cy < 0.0f)           { cy = 0.0f; }
    if(cy > (float32)(STAR_IMG_HEIGHT - 1U)) { cy = (float32)(STAR_IMG_HEIGHT - 1U); }


    r = (float32)WIN_SIZE + (1.0f - rough_vec[2]) * (float32)(WIN_SIZE * 2U);
    /* guard check */
    if (r < (float32)(WIN_SIZE / 2U))   { r = (float32)(WIN_SIZE / 2U); }
    if (r > (float32)(WIN_SIZE * 4U))   { r = (float32)(WIN_SIZE * 4U); }

    wpar.center_x       = cx;
    wpar.center_y       = cy;
    wpar.radius         = r;
    wpar.max_cand_stars = MAX_EXTRACT_STARS;
}
/* --- */









/**
 * @brief Star sensor processing
 */
void ExtractStarPoints(const volatile U8 *frame_buf, U32 frame_len)
{
    U32     i;
    /* serial frame construction */
    U32     offset;
    U32     raw_word;
    U32     n_stars;
    float32 px_raw, py_raw;


    /* check condition */
    if (frame_len < (STAR_POINTS_OFFSET + STAR_POINT_BYTES))
    {
        return;
    }


    /* clear interrupt flags */
    n_stars = (frame_len - STAR_POINTS_OFFSET) / STAR_POINT_BYTES;
    if (n_stars > MAX_EXTRACT_STARS)
    {
        n_stars = MAX_EXTRACT_STARS;
    }


    /* iterate */
    for (i = 0U; i < n_stars; i++)
    {
        offset = STAR_POINTS_OFFSET + i * STAR_POINT_BYTES;


        raw_word  = (U32)frame_buf[offset + 0U];
        /* bit operation */
		raw_word |= (U32)frame_buf[offset + 1U] << 8U;
        raw_word |= (U32)frame_buf[offset + 2U] << 16U;
        raw_word |= (U32)frame_buf[offset + 3U] << 24U;


        /* register access */
        px_raw = (float32)(raw_word & 0x00000FFFU);
        py_raw = (float32)((raw_word >> 12U) & 0x00000FFFU);


        if (i < NAVI_STAR_MAX)
        {
    /* data processing and validation */
			slist[i].px  = px_raw;
            slist[i].py  = py_raw;
            slist[i].mag = 0.0f;
            slist[i].id  = i;
        }
    }
    /* --- */


    scnt = n_stars;
    imgst.detected_stars += n_stars;
}
/**
 * @brief Star sensor processing
 */
static float32 MatchStarPairA(U32 idx_a, U32 idx_b)
{
    float32 dx, dy, dist, score;
    /* check condition */
    if (idx_a >= NAVI_STAR_MAX || idx_b >= NAVI_STAR_MAX) return 0.0f;
    dx = slist[idx_a].px - slist[idx_b].px;
    dy = slist[idx_a].py - slist[idx_b].py;
    dist = Sqrtx(dx * dx + dy * dy);
    /* invoke subroutine */
    if (dist < 1.0e-4f) return 0.0f;
    score = 1.0f / (1.0f + dist);
    if (score > MATCH_THRESHOLD)
        return score;
    return 0.0f;
}
/* --- */



/**
 * @brief Utility function
 */
void QuatToEuler(const Quaternion *q, EulerAngle *euler)
{
    float32 q0, q1, q2, q3;
    float32 sin_pitch;
    float32 t0, t1, t2, t3;
    float32 ratio, angle;
    float32 x3, x5;

    q0 = q->q[0];
    /* execute business logic */
    q1 = q->q[1];
    q2 = q->q[2];
    q3 = q->q[3];


    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);

    /* guard check */
    if (sin_pitch >  1.0f) { sin_pitch =  1.0f; }
    if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }


    x3 = sin_pitch * sin_pitch * sin_pitch;
    /* communication data handling */
    x5 = x3 * sin_pitch * sin_pitch;
    euler->pitch = sin_pitch + x3 / 6.0f + (3.0f * x5) / 40.0f;


    t0 = 2.0f * (q0 * q1 + q2 * q3);
    t1 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    /* --- */

    if(t1 > 1.0e-6f || t1 < -1.0e-6f)
    {
        ratio = t0 / t1;

        if (ratio >  1.0f) { ratio =  1.0f; }
    /*
     * core computation block
     */
        if(ratio < -1.0f) { ratio = -1.0f; }
        angle = ratio - (ratio * ratio * ratio) / 3.0f
                      + (ratio * ratio * ratio * ratio * ratio) / 5.0f;
        euler->roll = (t1 > 0.0f) ? angle : (angle + 3.14159265f);
    }
    else
    {
        euler->roll = (t0 >= 0.0f) ? (3.14159265f / 2.0f) : -(3.14159265f / 2.0f);
    }


    t2 = 2.0f * (q0 * q3 + q1 * q2);
    /* hardware interface operations */
    t3 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    if (t3 > 1.0e-6f || t3 < -1.0e-6f)
    {
        ratio = t2 / t3;
        if (ratio >  1.0f) { ratio =  1.0f; }
        if (ratio < -1.0f) { ratio = -1.0f; }
        angle = ratio - (ratio * ratio * ratio) / 3.0f
                      + (ratio * ratio * ratio * ratio * ratio) / 5.0f;
        euler->yaw = (t3 > 0.0f) ? angle : (angle + 3.14159265f);
    }
    else
    {
        euler->yaw = (t2 >= 0.0f) ? (3.14159265f / 2.0f) : -(3.14159265f / 2.0f);
    }
}
/* --- */








/**
 * @brief Data update routine
 */
void UpdateAttitudeFromVec(const float32 *rough_vec)
{
    float32 ref[3];
    float32 axis[3];
    float32 dot_val;
    float32 axis_norm;
    float32 half_angle;
    float32 sin_half;
    Quaternion new_quat;
    /* --- */


    ref[0] = 0.0f;
    ref[1] = 0.0f;
    /* buffer write */
    ref[2] = 1.0f;


    dot_val = Vec3Dot(rough_vec, ref);


    if (dot_val >  1.0f) { dot_val =  1.0f; }
    /* check condition */
    if (dot_val < -1.0f) { dot_val = -1.0f; }


    Vec3Cross(ref, rough_vec, axis);
    /* --- */

    axis_norm = Sqrtx(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);

    if (axis_norm < 1.0e-6f)
    {

        if (dot_val >= 0.0f)
        {
            /* array operation */
            new_quat.q[0] = 1.0f;
            new_quat.q[1] = 0.0f;
            new_quat.q[2] = 0.0f;
            new_quat.q[3] = 0.0f;
        }
        else
        {

            new_quat.q[0] = 0.0f;
            new_quat.q[1] = 1.0f;
            /* buffer write */
            new_quat.q[2] = 0.0f;
            new_quat.q[3] = 0.0f;
        }
    }
    else
    {
    /* --- */

        axis[0] /= axis_norm;
        axis[1] /= axis_norm;
        axis[2] /= axis_norm;




        half_angle = (3.14159265f / 2.0f - dot_val) * 0.5f;
    /* --- */


        sin_half = half_angle - (half_angle * half_angle * half_angle) / 6.0f;


        new_quat.q[0] = 1.0f - half_angle * half_angle / 2.0f;
        new_quat.q[1] = sin_half * axis[0];
        /* array operation */
        new_quat.q[2] = sin_half * axis[1];
        new_quat.q[3] = sin_half * axis[2];
    }


    QuatNormalize(&new_quat);
    att.attQuat.q[0] = new_quat.q[0];
    /* array operation */
    att.attQuat.q[1] = new_quat.q[1];
    att.attQuat.q[2] = new_quat.q[2];
    att.attQuat.q[3] = new_quat.q[3];


    /* invoke subroutine */
    QuatToEuler(&new_quat, &att.attEuler);
}












/**
 * @brief Star sensor processing
 */
void RunStarSensorSM(void)
{
/* --- */

    
/*
 * Macro definitions
 */
    #define SM_IDLE       0
    #define SM_PROCESSING 1
    #define SM_REPORTING  2
    #define SM_FAULT      3
    #define SM_RECOVERY   4


    /* FSM transition */
    static int sm_state       = SM_IDLE;
    static U32       recovery_cnt   = 0U;

    float32 local_vec[3];

    /* dispatch by type */
    switch (sm_state)
    {
        case SM_IDLE:

            if (mRoughVector.valid == 1U)
            {
                /* state transition */
                sm_state = SM_PROCESSING;
            }
            break;

        case SM_PROCESSING:

            if (dstat == DIAG_NOFRAME)
            {
                /* state transition */
                sm_state     = SM_FAULT;
                recovery_cnt = 0U;
                break;
            }
    /* --- */


            local_vec[0] = (float32)mRoughVector.v3[0];
            local_vec[1] = (float32)mRoughVector.v3[1];
            /* buffer write */
            local_vec[2] = (float32)mRoughVector.v3[2];


            UpdateWinParams(local_vec);


            /* invoke subroutine */
            UpdateAttitudeFromVec(local_vec);


            RecordAttHistory(att.matchScore);

            /* update state */
            sm_state = SM_REPORTING;
            break;

        case SM_REPORTING:

            EvalAttHistory();

            /* state transition */
            sm_state = SM_IDLE;
            break;

        case SM_FAULT:
    /* --- */

            recovery_cnt++;
            if (recovery_cnt >= SM_RECOVERY_FRAMES)
            {
                /* update state */
                sm_state     = SM_RECOVERY;
                recovery_cnt = 0U;
            }
            break;
    /* --- */

        case SM_RECOVERY:

            DisableUartIRQ();
            /* array operation */
            mRoughVector.v3[0] = 0.0f;
            mRoughVector.v3[1] = 0.0f;
            mRoughVector.v3[2] = 1.0f;
            mRoughVector.valid  = 0U;
            /* invoke subroutine */
            EnableUartIRQ();


            dstat = DIAG_OK;
    /* --- */


            att.matchScore = 0.0f;

            /* update state */
            sm_state = SM_IDLE;
            break;

        default:
            /* state transition */
            sm_state = SM_IDLE;
            break;
    }
}
/* --- */





#ifdef STAR_VEC_VERBOSE
/* --- */










/**
 * @brief Star sensor processing
 */
static void StarVecDump(void)
{
    FloatBytes fb;
    U32        mode_val;


    /* error detection and recovery */
    UART_DR = (U32)'V';
    UART_DR = (U32)'E';
    UART_DR = (U32)'C';
    UART_DR = (U32)':';


    fb.flt32 = (float32)mRoughVector.v3[0];
    UART_DR = (U32)'X';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    /* system state update */
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';


    fb.flt32 = (float32)mRoughVector.v3[1];
    UART_DR = (U32)'Y';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';
    /* --- */


    fb.flt32 = (float32)mRoughVector.v3[2];
    UART_DR = (U32)'Z';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';


    /*
     * periodic task processing
     */
    fb.flt32 = att.matchScore;
    UART_DR = (U32)'S';
    UART_DR = (U32)'C';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';


    mode_val = (U32)AppMode;
    /* pack and transmit data */
    UART_DR = (U32)'M';
    UART_DR = (U32)'O';
    UART_DR = (U32)'D';
    UART_DR = (U32)'E';
    UART_DR = (U32)'=';
    UART_DR = (U32)('0' + mode_val);
    /* --- */


    UART_DR = (U32)'\r';
    UART_DR = (U32)'\n';
}
/* --- */

#endif
