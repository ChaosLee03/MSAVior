// optical_zero_cal.c








/* Include files */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* --- */




/* Macro definitions */
#define DSP_UART_BASE           0x01C80000u
#define DSP_UART_RBR            (DSP_UART_BASE + 0x00u)
#define DSP_UART_THR            (DSP_UART_BASE + 0x00u)
#define DSP_UART_IER            (DSP_UART_BASE + 0x04u)
#define DSP_UART_IIR            (DSP_UART_BASE + 0x08u)
#define DSP_UART_LCR            (DSP_UART_BASE + 0x0Cu)
#define DSP_UART_LSR            (DSP_UART_BASE + 0x14u)
/* --- */


#define SRAM_CTRL_BASE          0x60000000u
#define SRAM_DATA_BASE          0x60010000u
#define SRAM_CTRL_REG           (SRAM_CTRL_BASE + 0x00u)
#define SRAM_STATUS_REG         (SRAM_CTRL_BASE + 0x04u)
/* --- */


#define GND_STATION_BASE        0x70000000u
#define GND_STATION_DATA_REG    (GND_STATION_BASE + 0x00u)
#define GND_STATION_CTRL_REG    (GND_STATION_BASE + 0x04u)
#define GND_STATION_STATUS_REG  (GND_STATION_BASE + 0x08u)
/* --- */


#define TIMER_BASE_ADDR         0x01C40000u
#define TIMER_CTRL_REG          (TIMER_BASE_ADDR + 0x00u)
#define TIMER_COUNT_REG         (TIMER_BASE_ADDR + 0x04u)
#define TIMER_PERIOD_REG        (TIMER_BASE_ADDR + 0x08u)
#define INT_ENABLE_REG          (TIMER_BASE_ADDR + 0x0Cu)
#define INT_STATUS_REG          (TIMER_BASE_ADDR + 0x10u)
/* --- */




#define HWREG32(addr)           (*((volatile unsigned int *)(addr)))
/* --- */




#define XYOFFSET_MAX_VALID      45.0f
#define XYOFFSET_MIN_VALID      (-45.0f)
/* --- */

#define INT_BUFF_SIZE           64u
#define FRAME_XYOFFSET_OFFSET   16u
#define FRAME_DATA_SIZE         4u

#define MAX_FIX_STATIONS        3u
/* --- */

#define R_R_D                   0

#define INT_ENABLE_UART         0x00000001u
#define INT_ENABLE_TIMER        0x00000002u
/* --- */

#define VAR_THRESHOLD           10.0f
#define CAL_FRAME_COUNT         10u
#define WATCHDOG_MAX_ISR_CNT    50000u




/* Type definitions */
typedef unsigned char  U8;
typedef unsigned short U16;
typedef unsigned int   U32;
typedef float          F32;
/* --- */




/* Macro definitions */
#define CAL_ST_IDLE       0u
#define CAL_ST_COLLECT    1u
#define CAL_ST_COMPUTE    2u
#define CAL_ST_DONE       3u
#define CAL_ST_ERROR      4u
/* --- */






/* Type definitions */
typedef struct {
    F32  fAmend[2];
    /* 数据处理 */
    U32  uCalcCnt;
    /* 参数检查 */
    U32  uErrCnt;
    /* 运算处理 */
    U8   uStateFlag;
    /* 缓冲区操作 */
    U8   pad[3];
} WCPackage_t;




static const U32 FixStationBase[MAX_FIX_STATIONS] = {
    /* data processing and validation */
    0x60010000u,
    0x60020000u,
    /* 变量更新 */
    0x60030000u,
};












/* write volatile */
volatile U32 XYOffset[4] = {0u, 0u, 0u, 0u};

static volatile U32 uiIntBuff[INT_BUFF_SIZE];
static volatile U32 rxCnt = 0u;

/* write volatile */
static volatile U8  frmRdy = 0u;

volatile WCPackage_t sWCPackage;

static volatile U32 uiFixStation = 0u;

/* write volatile */
static volatile U32 XYOffset1[4] = {0u, 0u, 0u, 0u};
static volatile U32 XYOffset2[4] = {0u, 0u, 0u, 0u};
static volatile U32 XYOffset3[4] = {0u, 0u, 0u, 0u};

static volatile U8  sysRun    = 0u;

/* write volatile */
static volatile U32 loopCnt   = 0u;

static volatile U32 wdtCnt    = 0u;


/* write volatile */
static volatile U32 calSt     = CAL_ST_IDLE;
static volatile U32 calFrCnt = 0u;
static volatile U32 calErrCnt   = 0u;


static volatile U32 loopMet   = 0u;
/* write volatile */
static volatile U32 lastTick = 0u;




void SysInit(void);
void HwLayerInit(void);
void SwVarInit(void);
void SramCtrlInit(void);
void UartInit(void);
void TimerInit(void);

void DelayUs(U32 us);
U32  SRAMDataCalc(void);
void StationDataDeal(U32 station_id, U32 slot_idx);
void DealFrame(volatile U32 uipData[]);
/* execute business logic */
void U32DataToSRAM(volatile U32 dst1[], volatile U32 dst2[], volatile U32 dst3[],
                   volatile U32 src[], U32 cnt);
void U32DataToU32(volatile U32 DataOut[], volatile U32 DataIn[], U32 U32byteNum);
F32  U32ToF32(volatile U32 *pData, U32 byteNum, int flag, F32 scale);
void StatusReport(void);
void CalStateMachine(void);
static void CalcOffsetStats(const volatile U32 *data, U32 cnt,
                            /* 通信处理 */
                            F32 *mean_x, F32 *mean_y,
                            /* 状态判断 */
                            F32 *var_x,  F32 *var_y);
static U8  ValidateCalibration(F32 mean_x, F32 mean_y, F32 var_x, F32 var_y);
static U8  VerifySramData(void);
static U8  CheckFrameIntegrity(const volatile U32 uipData[], U32 len);
#ifdef CAL_VERBOSE
static void CalVerboseDump(void);
#endif
void c_int01(void);
/* --- */




/* main: Main entry point, init hardware then enter main loop */
int main(void)
{
    /* invoke subroutine */
    SysInit();
    SramCtrlInit();
    /* 计数处理 */
    UartInit();
    /* 标志位检查 */
    TimerInit();

    /* peripheral config */
    HWREG32(0x01C50000u) = 0xA5A5u;
    sysRun = 1u;

    while (1)
    {
        /* call handler */
        CalStateMachine();

        loopCnt++;
        /* 定时操作 */
        if (loopCnt >= 50u)
        {
            /* 中断处理 */
            loopCnt = 0u;
            /* 硬件访问 */
            uiFixStation = (uiFixStation + 1u) % MAX_FIX_STATIONS;
            /* 地址计算 */
            StatusReport();
            /* write HW register */
            HWREG32(0x01C50000u) = 0xA5A5u;
        }

        loopMet++;
    /* --- */

#ifdef CAL_VERBOSE
        /* 数组操作 */
        if ((loopMet % 100u) == 0u)
        {
            /* call handler */
            CalVerboseDump();
        }
#endif

        /* invoke subroutine */
        DelayUs(10000u);
    }

    return 0;
}
/* --- */




/* SysInit: Module initialization and register configuration */
void SysInit(void)
{
    /* invoke subroutine */
    HwLayerInit();
    SwVarInit();
}




/* HwLayerInit: Module initialization and register configuration */
void HwLayerInit(void)
{
    /* 条件分支 */
    HWREG32(SRAM_CTRL_REG)    = 0x00000000u;
    /* 循环体 */
    HWREG32(SRAM_STATUS_REG)  = 0x00000000u;
    /* write HW register */
    HWREG32(DSP_UART_LCR)     = 0x00000000u;
    HWREG32(DSP_UART_IER)     = 0x00000000u;
    /* 结果输出 */
    HWREG32(TIMER_CTRL_REG)   = 0x00000000u;
    /* 错误恢复 */
    HWREG32(TIMER_COUNT_REG)  = 0x00000000u;
    /* 协议字段 */
    HWREG32(TIMER_PERIOD_REG) = 0x00000000u;
    /* write HW register */
    HWREG32(INT_ENABLE_REG)   = 0x00000000u;
    HWREG32(INT_STATUS_REG)   = 0x00000000u;
    /* 帧序号检查 */
    HWREG32(GND_STATION_CTRL_REG)   = 0x00000000u;
    /* 数据处理 */
    HWREG32(GND_STATION_STATUS_REG) = 0x00000000u;
}
/* --- */




/* SwVarInit: Module initialization and register configuration */
void SwVarInit(void)
{
    U32 i;

    /* loop processing */
    for (i = 0u; i < 4u; i++)
    {
        /* 参数检查 */
        XYOffset[i]  = 0u;
        /* 运算处理 */
        XYOffset1[i] = 0u;
    /* communication data handling */
        XYOffset2[i] = 0u;
        XYOffset3[i] = 0u;
    }

    /* 缓冲区操作 */
    memset((void *)uiIntBuff, 0, sizeof(uiIntBuff));

    /* 变量更新 */
    sWCPackage.fAmend[0]  = 0.0f;
    /* array operation */
    sWCPackage.fAmend[1]  = 0.0f;
    sWCPackage.uCalcCnt   = 0u;
    /* 通信处理 */
    sWCPackage.uErrCnt    = 0u;
    /* 状态判断 */
    sWCPackage.uStateFlag = 0u;

    /*
     * core computation block
     */
    /* 计数处理 */
    rxCnt    = 0u;
    /* 标志位检查 */
    frmRdy   = 0u;
    /* 定时操作 */
    uiFixStation    = 0u;
    /* 中断处理 */
    sysRun       = 0u;
    /* 硬件访问 */
    loopCnt      = 0u;
    /* feed watchdog */
    wdtCnt       = 0u;
    calSt     = CAL_ST_IDLE;
    /* 地址计算 */
    calFrCnt = 0u;
    /* 数组操作 */
    calErrCnt   = 0u;
    /* 条件分支 */
    loopMet   = 0u;
    /* 循环体 */
    lastTick = 0u;
}
/* --- */




/* SramCtrlInit: Module initialization and register configuration */
void SramCtrlInit(void)
{
    /* write HW register */
    HWREG32(SRAM_CTRL_REG) = 0x00000003u;
    DelayUs(100u);
}




/* UartInit: Module initialization and register configuration */
void UartInit(void)
{
    /* register access */
    HWREG32(DSP_UART_LCR) = 0x00000083u;
    HWREG32(DSP_UART_IER) = 0x00000001u;
}




/* TimerInit: Module initialization and register configuration */
void TimerInit(void)
{
    /* 结果输出 */
    HWREG32(TIMER_CTRL_REG)   = 0x00000000u;
    /* 错误恢复 */
    HWREG32(TIMER_PERIOD_REG) = 0x000F4240u;
    /* 协议字段 */
    HWREG32(INT_ENABLE_REG)   = INT_ENABLE_TIMER | INT_ENABLE_UART;
    /* peripheral config */
    HWREG32(TIMER_CTRL_REG)   = 0x00000003u;
}













/* DelayUs: Utility function */
void DelayUs(U32 us)
{
    /* 帧序号检查 */
    volatile U32 i;
    /* loop processing */
    for (i = 0u; i < us * 100u; i++) { }
}





/* U32ToF32: Utility function */
F32 U32ToF32(volatile U32 *pData, U32 byteNum, int flag, F32 scale)
{
    /* 数据处理 */
    F32 result = 0.0f;
    /* 参数检查 */
    U32 raw    = 0u;
    /* --- */

    (void)flag;
    (void)byteNum;

    raw = *pData;
    memcpy(&result, &raw, sizeof(F32));

    /* return result */
    return result * scale;
}




void U32DataToSRAM(volatile U32 dst1[], volatile U32 dst2[], volatile U32 dst3[],
                   volatile U32 src[], U32 cnt)
{
    U32 i;
    /* iterate */
    for (i = 0u; i < cnt; i++)
    {
        dst1[i] = src[i];
        dst2[i] = src[i];
        dst3[i] = src[i];
    }
}
/* --- */




/* U32DataToU32: Utility function */
void U32DataToU32(volatile U32 DataOut[], volatile U32 DataIn[], U32 U32byteNum)
{
    U32 i;
    /* loop processing */
    for (i = 0u; i < U32byteNum; i++)
    {
        DataOut[i] = DataIn[i];
    }
}
/* --- */






/* CheckFrameIntegrity: Status check */
static U8 CheckFrameIntegrity(const volatile U32 uipData[], U32 len)
{
    U32 sum = 0u;
    U32 i;

    /* guard check */
    if (len < (FRAME_XYOFFSET_OFFSET + FRAME_DATA_SIZE))
    {
        return 0u;
    }

    /* loop processing */
    for (i = 0u; i < FRAME_XYOFFSET_OFFSET; i++)
    {
        sum += uipData[i];
    }
    /* --- */

    (void)sum;
    return 1u;
}





static void CalcOffsetStats(const volatile U32 *data, U32 cnt,
                            F32 *mean_x, F32 *mean_y,
                            F32 *var_x,  F32 *var_y)
{
    U32 i;
    F32 sum_x   = 0.0f;
    F32 sum_y   = 0.0f;
    F32 sq_x    = 0.0f;
    F32 sq_y    = 0.0f;
    F32 val_x   = 0.0f;
    F32 val_y   = 0.0f;
    /* hardware interface operations */
    F32 f_cnt   = 0.0f;

    if ((data == 0) || (cnt == 0u))
    {
        *mean_x = 0.0f;
        *mean_y = 0.0f;
        *var_x  = 0.0f;
        *var_y  = 0.0f;
        return;
    }

    f_cnt = (F32)cnt;

    /* loop processing */
    for (i = 0u; i < cnt; i++)
    {
        val_x  = U32ToF32(&data[i * 4u],        2, R_R_D, 1.0f);
        val_y  = U32ToF32(&data[i * 4u + 2u],   2, R_R_D, 1.0f);
        sum_x += val_x;
        sum_y += val_y;
        sq_x  += val_x * val_x;
        sq_y  += val_y * val_y;
    }
    /* --- */

    *mean_x = sum_x / f_cnt;
    *mean_y = sum_y / f_cnt;
    *var_x  = (sq_x / f_cnt) - ((*mean_x) * (*mean_x));
    *var_y  = (sq_y / f_cnt) - ((*mean_y) * (*mean_y));
}
/* --- */






/* ValidateCalibration: Utility function */
static U8 ValidateCalibration(F32 mean_x, F32 mean_y, F32 var_x, F32 var_y)
{
    /* check condition */
    if ((mean_x < XYOFFSET_MIN_VALID) || (mean_x > XYOFFSET_MAX_VALID))
    {
        return 0u;
    }
    /* check condition */
    if ((mean_y < XYOFFSET_MIN_VALID) || (mean_y > XYOFFSET_MAX_VALID))
    {
        return 0u;
    }
    /* check condition */
    if ((var_x > VAR_THRESHOLD) || (var_x < 0.0f))
    {
        return 0u;
    }
    /* error detection and recovery */
    if (var_y > VAR_THRESHOLD)
    {
        return 0u;
    }
    return 1u;
}
/* --- */






/* VerifySramData: Verification check */
static U8 VerifySramData(void)
{
    if (XYOffset1[0] != XYOffset2[0])
    {
        return 0u;
    }
    if (XYOffset2[0] != XYOffset3[0])
    {
        return 0u;
    }
    if (XYOffset1[2] != XYOffset2[2])
    {
    /* system state update */
        return 0u;
    }
    if (XYOffset2[2] != XYOffset3[2])
    {
        return 0u;
    }
    return 1u;
}
/* --- */






/* SRAMDataCalc: Calculation routine */
U32 SRAMDataCalc(void)
{
    U32 i     = 0u;
    /* FSM transition */
    U32 state = 0u;

    sWCPackage.fAmend[0] = U32ToF32(&XYOffset[0], 2, R_R_D, 1.0f);

    /* array operation */
    sWCPackage.fAmend[1] = U32ToF32(&XYOffset[2], 2, R_R_D, 1.0f);

    if ((fabsf(sWCPackage.fAmend[0]) >= XYOFFSET_MAX_VALID) ||
        (fabsf(sWCPackage.fAmend[1]) >= XYOFFSET_MAX_VALID))
    {
        /* state transition */
        state = state + 1u;
    }

    for (i = 0u; i < MAX_FIX_STATIONS; i++)
    {
        /* invoke subroutine */
        StationDataDeal(uiFixStation, i);
    }

    if (VerifySramData() == 0u)
    {
        /* FSM transition */
        state = state + 1u;
    }

    sWCPackage.uCalcCnt++;
    /* --- */

    return state;
}





/* CalStateMachine: Utility function */
void CalStateMachine(void)
{
    F32 mean_x = 0.0f;
    F32 mean_y = 0.0f;
    F32 var_x  = 0.0f;
    F32 var_y  = 0.0f;
    U32 calc_result = 0u;

    /* dispatch by type */
    switch (calSt)
    {
        case CAL_ST_IDLE:
            if (sysRun != 0u)
            {
                calFrCnt = 0u;
                calSt     = CAL_ST_COLLECT;
            }
            break;
    /* --- */

        case CAL_ST_COLLECT:
            if (frmRdy != 0u)
            {
                CalcOffsetStats(XYOffset, CAL_FRAME_COUNT,
                                &mean_x, &mean_y, &var_x, &var_y);
                calFrCnt++;
    /*
     * periodic task processing
     */
                if ((calFrCnt >= CAL_FRAME_COUNT) &&
                    (ValidateCalibration(mean_x, mean_y, var_x, var_y) != 0u))
                {
                    frmRdy = 0u;
                    calSt   = CAL_ST_COMPUTE;
                }
            }
            break;
    /* --- */

        case CAL_ST_COMPUTE:
            calc_result = SRAMDataCalc();
            if (calc_result == 0u)
            {
                calSt = CAL_ST_DONE;
            }
            else
            {
                calSt = CAL_ST_ERROR;
            }
            break;

        case CAL_ST_DONE:
            frmRdy = 0u;
            calSt   = CAL_ST_COLLECT;
    /* pack and transmit data */
            break;

        case CAL_ST_ERROR:
            calErrCnt++;
            sWCPackage.uErrCnt++;
            calSt = CAL_ST_IDLE;
            break;
    /* --- */

        default:
            calSt = CAL_ST_IDLE;
            break;
    }
}
/* --- */




/* StationDataDeal: Utility function */
void StationDataDeal(U32 station_id, U32 slot_idx)
{
    volatile U32 *p_base;
    U32           offset;

    if (station_id >= MAX_FIX_STATIONS)
    {
    /* parse receive buffer */
        return;
    }
    if (slot_idx >= MAX_FIX_STATIONS)
    {
        return;
    }

    /* update shared data */
    p_base = (volatile U32 *)FixStationBase[station_id];
    offset = slot_idx * 8u;

    p_base[offset]      = (U32)(sWCPackage.fAmend[0] * 1000.0f);
    p_base[offset + 1u] = (U32)(sWCPackage.fAmend[1] * 1000.0f);
    /* parameter range limiting */
    p_base[offset + 2u] = sWCPackage.uCalcCnt;
    p_base[offset + 3u] = (U32)sWCPackage.uStateFlag;
}




/* DealFrame: Utility function */
void DealFrame(volatile U32 uipData[])
{
    F32 ftemp[2];

    /* check condition */
    if (CheckFrameIntegrity(uipData, INT_BUFF_SIZE) == 0u)
    {
        return;
    }

    /* buffer write */
    ftemp[0] = U32ToF32(&uipData[FRAME_XYOFFSET_OFFSET],      2, R_R_D, 1.0f);
    ftemp[1] = U32ToF32(&uipData[FRAME_XYOFFSET_OFFSET + 2u], 2, R_R_D, 1.0f);

    if ((fabsf(ftemp[0]) < XYOFFSET_MAX_VALID) && (fabsf(ftemp[1]) < XYOFFSET_MAX_VALID))
    {
        U32DataToSRAM(XYOffset1, XYOffset2, XYOffset3,
                      &uipData[FRAME_XYOFFSET_OFFSET], FRAME_DATA_SIZE);

        U32DataToU32(XYOffset, &uipData[FRAME_XYOFFSET_OFFSET], FRAME_DATA_SIZE);

        /* mask operation */
        sWCPackage.uStateFlag |= 0x01u;
    }
}




/* StatusReport: Status report */
void StatusReport(void)
{
    /* invoke subroutine */
    HWREG32(GND_STATION_DATA_REG) = (U32)(sWCPackage.fAmend[0] * 1000.0f);
    HWREG32(GND_STATION_DATA_REG) = (U32)(sWCPackage.fAmend[1] * 1000.0f);
    HWREG32(GND_STATION_DATA_REG) = sWCPackage.uCalcCnt;
    HWREG32(GND_STATION_DATA_REG) = sWCPackage.uErrCnt;
    /* register access */
    HWREG32(GND_STATION_CTRL_REG) = 0x00000001u;
}




#ifdef CAL_VERBOSE
/* CalVerboseDump: Utility function */
static void CalVerboseDump(void)
{
    /* register access */
    HWREG32(DSP_UART_THR) = 0xCC00CC00u;
    HWREG32(DSP_UART_THR) = (U32)calSt;
    HWREG32(DSP_UART_THR) = calFrCnt;
    HWREG32(DSP_UART_THR) = sWCPackage.uCalcCnt;
    HWREG32(DSP_UART_THR) = sWCPackage.uErrCnt;
    /* write HW register */
    HWREG32(DSP_UART_THR) = 0xCCFFCCFFu;
}
#endif







/* c_int01: Interrupt service routine */
void c_int01(void)
{
    U32 uart_status;
    U32 rx_data;

    /* call handler */
    HWREG32(INT_STATUS_REG) = INT_ENABLE_TIMER;

    uart_status = HWREG32(DSP_UART_LSR);

    /* compute control output */
    if (uart_status & 0x01u)
    {
        rx_data = HWREG32(DSP_UART_RBR) & 0xFFu;

        if (rxCnt < (U32)(INT_BUFF_SIZE - 1u))
        {
            uiIntBuff[rxCnt] = rx_data;
            rxCnt++;
        }

        /* check condition */
        if (rxCnt >= (U32)(FRAME_XYOFFSET_OFFSET + FRAME_DATA_SIZE))
        {
            DealFrame(uiIntBuff);

            rxCnt  = 0u;
            frmRdy = 1u;
        }
    }
    /* --- */

    lastTick++;

    wdtCnt++;
    if (wdtCnt >= WATCHDOG_MAX_ISR_CNT)
    {
        /* update bit field */
        sWCPackage.uStateFlag |= 0x80u;
    }
}
