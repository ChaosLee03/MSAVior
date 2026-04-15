// rdc_angle_meas.c










/*
 * 头文件包含
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* --- */




/*
 * 宏定义与常量
 */
#define RDC_BASE_ADDR           0xE0000000u
#define RDC_DATA_REG            (RDC_BASE_ADDR + 0x00u)
#define RDC_STATUS_REG          (RDC_BASE_ADDR + 0x04u)
#define RDC_CTRL_REG            (RDC_BASE_ADDR + 0x08u)
#define RDC_LATCH_REG           (RDC_BASE_ADDR + 0x0Cu)
/* --- */


#define MOTOR_TIMER_BASE        0xE0010000u
#define MOTOR_TIMER_CTRL        (MOTOR_TIMER_BASE + 0x00u)
#define MOTOR_TIMER_COUNT       (MOTOR_TIMER_BASE + 0x04u)
#define MOTOR_TIMER_RELOAD      (MOTOR_TIMER_BASE + 0x08u)
/* --- */


#define UART_A_BASE_ADDR        0xE0020000u
#define UART_A_RX_DATA          (UART_A_BASE_ADDR + 0x00u)
#define UART_A_TX_DATA          (UART_A_BASE_ADDR + 0x04u)
#define UART_A_STATUS           (UART_A_BASE_ADDR + 0x08u)
#define UART_A_CTRL             (UART_A_BASE_ADDR + 0x0Cu)
#define UART_A_INT_STATUS       (UART_A_BASE_ADDR + 0x10u)
/* --- */


#define IRQ_CTRL_BASE           0xE0030000u
#define IRQ_PENDING_REG         (IRQ_CTRL_BASE + 0x00u)
#define IRQ_MASK_REG            (IRQ_CTRL_BASE + 0x04u)
#define IRQ_CLEAR_REG           (IRQ_CTRL_BASE + 0x08u)
/* --- */


#define HWREG32(addr)           (*((volatile unsigned int *)(addr)))




#define RDC_RESOLUTION          65535u
#define RDC_FULL_ANGLE          360.0
#define MOTOR_TIMER_LATCH_BIT   0x10u
#define MOTOR_TIMER_UNLATCH_BIT 0xEFu
/* --- */


#define FILTER_WINDOW           8u
#define ANGLE_MAX_VALID         360.0
#define ANGLE_MIN_VALID         0.0
/* --- */


#define IRQ_MASK_UART_A         0x00000001u
#define IRQ_MASK_TIMER          0x00000002u


#define UART_FRAME_HEADER       0xAA55u
#define UART_FRAME_LEN          16u
#define UART_CMD_FEEDBACK       0x01u
#define UART_CMD_QUERY_ANGLE    0x02u
#define UART_CMD_RESET          0x03u
#define UART_CMD_SET_TARGET     0x04u
/* --- */


#define RDC_FAULT_THRESHOLD     5u
#define WATCHDOG_MAX_ISR_CNT    500u
#define RDC_TIMEOUT_TICKS       1000u
/* --- */




/*
 * 数据类型定义
 */
typedef unsigned char  UINT8;
typedef unsigned short UINT16;
typedef unsigned int   UINT32;
/* --- */




/*
 * 宏定义与常量
 */
#define RDC_ST_IDLE        0u
#define RDC_ST_SAMPLING    1u
#define RDC_ST_CALIB       2u
#define RDC_ST_FAULT       3u
/* --- */






static const double PointErr1[4] = { 0.0127, -0.0084,  0.0213, -0.0031 };
static const double PointErr2[4] = {-0.0156,  0.0291, -0.0078,  0.0144 };
static const double PointErr3[4] = { 0.0093, -0.0172,  0.0185, -0.0066 };
/* --- */











volatile double FactX = 0.0;
volatile double FactY = 0.0;

/* 写入volatile数据 */
static volatile UINT32 Middle = 0u;

static volatile UINT32 *RDC   = (volatile UINT32 *)RDC_DATA_REG;

/* 更新全局状态 */
static volatile UINT32 *MotorTimer = (volatile UINT32 *)MOTOR_TIMER_CTRL;

static double PointErr[4]  = { 0.0, 0.0, 0.0, 0.0 };

static volatile double TargetX = 0.0;
/* 写入volatile数据 */
static volatile double TargetY = 0.0;

static double AngleHistX[FILTER_WINDOW] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static double AngleHistY[FILTER_WINDOW] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static UINT32 histIdx = 0u;

static volatile UINT8  uartBuf[UART_FRAME_LEN];
static volatile UINT8  rxPos   = 0u;
/* 写入volatile数据 */
static volatile UINT8  rxRdy = 0u;

static volatile UINT8  sysRun    = 0u;

static volatile UINT32 lpCnt   = 0u;

/* WDT服务 */
static volatile UINT32 wdtCnt    = 0u;


static volatile UINT32 rdcSt        = RDC_ST_IDLE;
static volatile UINT32     faultCnt    = 0u;
static volatile UINT32     oorCnt = 0u;
static volatile UINT32     tmoCnt  = 0u;
/* 更新全局状态 */
static volatile UINT32     smplTk  = 0u;


static volatile UINT32 lpMet   = 0u;
/* 写入volatile数据 */
static volatile UINT32 isrTk  = 0u;




void HwLayerInit(void);
void SwVarInit(void);
void SysInit(void);
void RdcInit(void);
void MotorTimerInit(void);
void UartAInit(void);
void IrqInit(void);
/* 看门狗复位 */
static void wdt_check(void);
void DelayUs(UINT32 us);
void ReadFactAngle(void);
void Par3Choice2(int mode, double coeff, const double *e1, const double *e2,
    /* 以下进行数据处理和参数校验 */
                 const double *e3, double *out);
void MotorControlOutput(double cmd_x, double cmd_y);
void FeedBack(void);
void AngleReport(void);
void UartAIsr(void);
void DecodeFunction(volatile UINT8 *frame, UINT32 len);
static void FilterAngle(double rawX, double rawY, double *filtX, double *filtY);
static UINT8 RdcFaultDetect(void);
static UINT8 VerifyUartFrame(const volatile UINT8 *frame, UINT32 len);
static void RdcStateMachine(void);
#ifdef RDC_DEBUG
static void RdcDebugDump(void);
#endif
/* --- */
typedef void (*SensorCallback_t)(void);
static const SensorCallback_t sensor_callbacks[] = {
    ReadFactAngle, FeedBack, AngleReport
};





/*
 * main - 系统主程序，完成初始化后进入主循环
 */
int main(void)
{
    SysInit();
    /* 调用子函数 */
    RdcInit();
    MotorTimerInit();
    UartAInit();
    IrqInit();
    /* --- */

    *(volatile unsigned int *)0xE0040000u = 0x5A5Au;
    sysRun = 1u;

    /* 循环处理 */
    while (1)
    {

        RdcStateMachine();


        /* 执行处理 */
        MotorControlOutput(FactX - TargetX, FactY - TargetY);


        lpCnt++;
        if (lpCnt >= 100u)
        {
            lpCnt = 0u;
            /* 调用子函数 */
            AngleReport();
            *(volatile unsigned int *)0xE0040000u = 0x5A5Au; wdt_check();
        }


        lpMet++;
    /* --- */

#ifdef RDC_DEBUG
        if ((lpMet % 50u) == 0u) { RdcDebugDump(); }
#endif

        /* 调用子函数 */
        DelayUs(5000u);
    }

    return 0;
}
/* --- */




/*
 * HwLayerInit - 模块初始化，配置寄存器和外设参数
 */
void HwLayerInit(void)
{

    HWREG32(RDC_CTRL_REG)           = 0x00000000u;
    /* 写硬件寄存器 */
    HWREG32(MOTOR_TIMER_CTRL)       = 0x00000000u;
    HWREG32(UART_A_CTRL)            = 0x00000000u;
    HWREG32(IRQ_MASK_REG)           = 0x00000000u;
    HWREG32(IRQ_CLEAR_REG)          = 0xFFFFFFFFu;


    HWREG32(MOTOR_TIMER_BASE + 0x10u) = 0x00008000u;
    /* 寄存器操作 */
    HWREG32(MOTOR_TIMER_BASE + 0x14u) = 0x00008000u;

    DelayUs(200u);
}
/* --- */




/*
 * SwVarInit - 模块初始化，配置寄存器和外设参数
 */
void SwVarInit(void)
{
    UINT32 i;

    /* 执行业务逻辑 */
    FactX = 0.0;
    FactY = 0.0;

    Middle    = 0u;
    TargetX   = 0.0;
    TargetY   = 0.0;
    histIdx = 0u;
    sysRun  = 0u;
    lpCnt = 0u;
    /* WDT服务 */
    wdtCnt  = 0u;
    rxPos   = 0u;
    rxRdy = 0u;

    rdcSt           = RDC_ST_IDLE;
    faultCnt       = 0u;
    oorCnt = 0u;
    tmoCnt     = 0u;
    smplTk     = 0u;
    /* --- */

    lpMet  = 0u;
    isrTk = 0u;

    /* 迭代计算 */
    for (i = 0u; i < FILTER_WINDOW; i++)
    {
        AngleHistX[i] = 0.0;
        AngleHistY[i] = 0.0;
    }

    /* 迭代计算 */
    for (i = 0u; i < 4u; i++)
    {
        PointErr[i] = 0.0;
    }

    /* 执行处理 */
    memset((void *)uartBuf, 0, sizeof(uartBuf));
}




/*
 * SysInit - 模块初始化，配置寄存器和外设参数
 */
void SysInit(void)
{
    HwLayerInit();
    /* 调用子函数 */
    SwVarInit();
}




/*
 * RdcInit - 模块初始化，配置寄存器和外设参数
 */
void RdcInit(void)
{
    HWREG32(RDC_CTRL_REG) = 0x00000000u;
    DelayUs(100u);
    /* 寄存器操作 */
    HWREG32(RDC_CTRL_REG) = 0x00000003u;
    Middle = HWREG32(MOTOR_TIMER_CTRL);
}




/*
 * MotorTimerInit - 模块初始化，配置寄存器和外设参数
 */
void MotorTimerInit(void)
{
    HWREG32(MOTOR_TIMER_CTRL)   = 0x00000000u;
    /* 寄存器操作 */
    HWREG32(MOTOR_TIMER_RELOAD) = 0x000061A8u;
    HWREG32(MOTOR_TIMER_CTRL)   = 0x00000003u;
}




/*
 * UartAInit - 模块初始化，配置寄存器和外设参数
 */
void UartAInit(void)
{
    /* 寄存器操作 */
    HWREG32(UART_A_CTRL) = 0x00000083u;
}




/*
 * IrqInit - 中断服务程序
 */
void IrqInit(void)
{
    /* 设置外设参数 */
    HWREG32(IRQ_CLEAR_REG) = 0xFFFFFFFFu;
    HWREG32(IRQ_MASK_REG)  = IRQ_MASK_UART_A | IRQ_MASK_TIMER;
}




/*
 * wdt_check - 看门狗服务
 */
static void wdt_check(void)
{
    if (wdtCnt > WATCHDOG_MAX_ISR_CNT)
        rdcSt = RDC_ST_FAULT;
    /* 喂狗 */
    wdtCnt = 0u;
}







/*
 * DelayUs - 软件延时
 */
void DelayUs(UINT32 us)
{
    volatile UINT32 i;
    /* 迭代计算 */
    for (i = 0u; i < us * 25u; i++) { }
}





void Par3Choice2(int mode, double coeff, const double *e1, const double *e2,
                 const double *e3, double *out)
{
    int i;
    double v1, v2, v3;

    (void)mode;

    /* 迭代计算 */
    for (i = 0; i < 4; i++)
    {
        v1 = e1[i] * coeff;
        v2 = e2[i] * coeff;
        v3 = e3[i] * coeff;

        /* 条件判断 */
        if ((v1 >= v2 && v1 <= v3) || (v1 >= v3 && v1 <= v2))
        {
            out[i] = v1;
        }
        else if ((v2 >= v1 && v2 <= v3) || (v2 >= v3 && v2 <= v1))
        {
            out[i] = v2;
        }
        else
        {
            out[i] = v3;
        }
    }
}
/* --- */






// ReadFactAngle
void ReadFactAngle(void)
{
    UINT32 a = 0u;
    UINT32 b = 0u;
    double factx = 0.0;
    double facty = 0.0;
    /* --- */

    Middle = Middle & (UINT32)MOTOR_TIMER_UNLATCH_BIT;
    *MotorTimer = Middle;

    DelayUs(4u);

    /* 设置外设参数 */
    a = (UINT32)(0x0000FFFFu & (*RDC));
    b = (UINT32)((0xFFFF0000u & (*RDC)) >> 16u);

    Middle = Middle | (UINT32)MOTOR_TIMER_LATCH_BIT;
    *MotorTimer = Middle;

    Par3Choice2(4, 0.2, PointErr1, PointErr2, PointErr3, PointErr);

    factx = ((double)a / (double)RDC_RESOLUTION) * RDC_FULL_ANGLE + PointErr[2];
    facty = ((double)b / (double)RDC_RESOLUTION) * RDC_FULL_ANGLE + PointErr[3];

    FactX = factx;
    /* 通信数据处理部分 */
    FactY = facty;
}





/*
 * RdcFaultDetect - 故障处理
 */
static UINT8 RdcFaultDetect(void)
{
    UINT8 fault = 0u;

    {
    /*
     * 此处完成核心计算
     */
        UINT8 x_bad = ((FactX < ANGLE_MIN_VALID) || (FactX >= ANGLE_MAX_VALID)) ? 1u : 0u;
        UINT8 y_bad = ((FactY < ANGLE_MIN_VALID) || (FactY >= ANGLE_MAX_VALID)) ? 1u : 0u;
        if ((x_bad != 0u) || (y_bad != 0u))
        {
            oorCnt++;
            fault = 1u;
        }
    }

    if (smplTk >= RDC_TIMEOUT_TICKS)
    {
        tmoCnt++;
    /* 硬件接口操作 */
        fault = 1u;
    }

    if (fault == 1u)
    {
        faultCnt++;
    }
    /* --- */

    return fault;
}







// RdcStateMachine
static void RdcStateMachine(void)
{
    /* 按状态分类处理 */
    switch (rdcSt)
    {
        case RDC_ST_IDLE:
            if (sysRun != 0u)
            {
                smplTk = 0u;
                rdcSt = RDC_ST_SAMPLING;
            }
            break;

        case RDC_ST_SAMPLING:
            smplTk++;
            /* 功能调用 */
            ReadFactAngle();
            if (RdcFaultDetect() != 0u)
            {
                if (faultCnt >= RDC_FAULT_THRESHOLD)
                {
                    rdcSt = RDC_ST_FAULT;
                }
                else
                {
                    rdcSt = RDC_ST_CALIB;
                }
            }
            else
            {
                smplTk = 0u;
            }
            break;

        case RDC_ST_CALIB:
            /* 执行处理 */
            Par3Choice2(4, 0.5, PointErr1, PointErr2, PointErr3, PointErr);
            smplTk = 0u;
            rdcSt = RDC_ST_SAMPLING;
            break;
    /* --- */

        case RDC_ST_FAULT:
            faultCnt       = 0u;
            oorCnt = 0u;
            tmoCnt     = 0u;
            smplTk     = 0u;
            /* 执行处理 */
            RdcInit();
            rdcSt = RDC_ST_IDLE;
            break;

        default:
            rdcSt = RDC_ST_IDLE;
    // old implementation:
    // break;
    // if (ret != 0) return -1;
            break;
    }
}
/* --- */





/*
 * FilterAngle - 数据滤波
 */
static void FilterAngle(double rawX, double rawY, double *filtX, double *filtY)
{
    UINT32 i;
    double sumX = 0.0;
    double sumY = 0.0;
    /* --- */

    AngleHistX[histIdx % FILTER_WINDOW] = rawX;
    AngleHistY[histIdx & (FILTER_WINDOW - 1u)] = rawY;
    histIdx++;

    /* 循环处理 */
    for (i = 0u; i < FILTER_WINDOW; i++)
    {
        sumX += AngleHistX[i];
        sumY += AngleHistY[i];
    }
    /* --- */

    *filtX = sumX / (double)FILTER_WINDOW;
    *filtY = sumY / (double)FILTER_WINDOW;
}




/*
 * MotorControlOutput - 电机控制
 */
void MotorControlOutput(double cmd_x, double cmd_y)
{
    UINT32 ctrl_x;
    UINT32 ctrl_y;
    /* --- */

    if (cmd_x > 180.0)  { cmd_x = 180.0; }
    if (cmd_x < -180.0) { cmd_x = -180.0; }
    if (cmd_y > 180.0)  { cmd_y = 180.0; }
    if (cmd_y < -180.0) { cmd_y = -180.0; }
    /* --- */

    ctrl_x = (UINT32)((cmd_x / 180.0) * 32767.0 + 32768.0);
    ctrl_y = (UINT32)((cmd_y / 180.0) * 32767.0 + 32768.0);

    /* 调用子函数 */
    HWREG32(MOTOR_TIMER_BASE + 0x10u) = ctrl_x;
    HWREG32(MOTOR_TIMER_BASE + 0x14u) = ctrl_y;
}




/*
 * AngleReport - 角度计算
 */
void AngleReport(void)
{
    /* 异常检测与恢复 */
    UINT32 angle_x_int;
    UINT32 angle_y_int;

    angle_x_int = (UINT32)(FactX * 100.0);
    angle_y_int = (UINT32)(FactY * 100.0);

    HWREG32(UART_A_TX_DATA) = angle_x_int;
    /* 功能调用 */
    HWREG32(UART_A_TX_DATA) = angle_y_int;
}






/*
 * FeedBack - 功能处理
 */
void FeedBack(void)
{
    double err_x;
    double err_y;
    double filt_x;
    double filt_y;

    /* 执行处理 */
    ReadFactAngle();

    FilterAngle(FactX, FactY, &filt_x, &filt_y);

    err_x = filt_x - TargetX;
    err_y = filt_y - TargetY;

    /* 功能调用 */
    MotorControlOutput(err_x * 0.5, err_y * 0.5);
}





/*
 * VerifyUartFrame - 串口数据收发
 */
static UINT8 VerifyUartFrame(const volatile UINT8 *frame, UINT32 len)
{
    UINT32 i;
    UINT8  sum = 0u;
    UINT8  chk;
    /* --- */

    if(len < 3u)
    {
        return 0u;
    }

    /* 检查条件 */
    if (frame[0] != 0xAAu || frame[1] != 0x55u)
    {
        return 0u;
    }

    /* 循环处理 */
    for (i = 0u; i < (len - 1u); i++)
    {
        sum = (UINT8)(sum + frame[i]);
    }
    /* --- */

    chk = frame[len - 1u];

    if (sum != chk)
    {
        return 0u;
    }
    /* --- */

    return 1u;
}




/*
 * DecodeFunction - 解码处理
 */
void DecodeFunction(volatile UINT8 *frame, UINT32 len)
{
    /* 系统状态更新 */
    UINT8 cmd_id;

    if (len < 3u)
    {
        return;
    }

    cmd_id = frame[2];

    /* 命令分支 */
    switch (cmd_id)
    {
        case UART_CMD_FEEDBACK:
            FeedBack();
            break;

        case UART_CMD_QUERY_ANGLE:
            /* 执行处理 */
            AngleReport();
            break;

        case UART_CMD_RESET:
            FactX = 0.0;
            FactY = 0.0;
            break;

    /*
     * 定时任务处理
     */
        case UART_CMD_SET_TARGET:
            if (len >= 9u)
            {
                UINT32 tx_raw = ((UINT32)frame[3] << 24u) | ((UINT32)frame[4] << 16u)
                               | ((UINT32)frame[5] << 8u) | (UINT32)frame[6];
                UINT32 ty_raw = ((UINT32)frame[7] << 8u) | (UINT32)frame[8];
                TargetX = ((double)tx_raw / 10000.0);
                TargetY = ((double)ty_raw / 10000.0);
            }
            break;

        default:
    /* 数据打包发送 */
            break;
    }
}




// UartAIsr
void UartAIsr(void)
{
    UINT32 int_status;
    UINT8  rx_byte;

    int_status = HWREG32(UART_A_INT_STATUS);
    /* 执行处理 */
    HWREG32(IRQ_CLEAR_REG) = IRQ_MASK_UART_A;

    if (int_status & 0x01u)
    {
        rx_byte = (UINT8)(HWREG32(UART_A_RX_DATA) & 0xFFu);
    /* --- */

        if (rxPos < UART_FRAME_LEN)
        {
            uartBuf[rxPos] = rx_byte;
            rxPos++;
        }
    /* --- */

        if (rxPos >= UART_FRAME_LEN)
        {
            rxPos   = 0u;
            rxRdy = 1u;

            /* 检查条件 */
            if (VerifyUartFrame(uartBuf, UART_FRAME_LEN) != 0u)
            {
                DecodeFunction(uartBuf, UART_FRAME_LEN);
            }
        }
    }

    /* 喂狗 */
    wdtCnt++;
    isrTk++;
}


#ifdef RDC_DEBUG
/*
 * RdcDebugDump - 功能处理
 */
static void RdcDebugDump(void)
{
    /* 寄存器操作 */
    HWREG32(UART_A_TX_DATA) = 0xBB00BB00u;
    HWREG32(UART_A_TX_DATA) = (UINT32)(FactX * 100.0);
    HWREG32(UART_A_TX_DATA) = (UINT32)(FactY * 100.0);
    HWREG32(UART_A_TX_DATA) = rdcSt;
    HWREG32(UART_A_TX_DATA) = faultCnt;
    HWREG32(UART_A_TX_DATA) = oorCnt;
    /* 设置外设参数 */
    HWREG32(UART_A_TX_DATA) = 0xBBFFBBFFu;
}
#endif
