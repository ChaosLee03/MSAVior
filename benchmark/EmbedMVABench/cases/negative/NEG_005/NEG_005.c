/* NEG_005.c  payload camera RS422  8051 */
/* disable_specific_interrupt fix applied */
#include <reg51.h>
#include <intrins.h>
#include <absacc.h>
/* --- */




/* ---- Macro definitions ---- */
#define uchar unsigned char
#define uint  unsigned int
/* --- */

#define CAMERA_A_RS422_REC_FIFO   0xA000u
#define CAMERA_A_RS422_STA_REG    0xA002u
#define CAMERA_A_RS422_TX_REG     0xA004u
#define RS422_TX_READY_MASK       0x01u
#define CAMERA_B_RS422_REC_FIFO   0xB000u
#define WATCHDOG_ADDR             0x6000u
#define STATUS_REG_ADDR           0x4000u
#define BACK_DATA_ADDR            0xC000u
#define EEPROM_BASE_ADDR          0xD000u
/* --- */

/* frame */
#define RS422_FRAME_LEN           17u
#define RS422_FRAME_SUM_START     2u
#define RS422_FRAME_SUM_END       15u
#define RS422_SYNC_BYTE0          0xEBu
#define RS422_SYNC_BYTE1          0x90u
#define RS422_ERR_CNT_MASK        0x3Fu
#define RS422_ERR_CNT_CLR_MASK    0xC0u
#define RS422_ERR_FLAG_SYNC       0x08u
#define RS422_ERR_FLAG_SUM        0x04u
/* --- */

#define CAM_CMD_SHUTTER           0x11u
#define CAM_CMD_EXPOSURE          0x12u
#define CAM_CMD_GAIN              0x13u
#define CAM_CMD_STATUS_QUERY      0x14u
#define CAM_CMD_FIELD_OFFSET      2u
#define CAM_CMD_FIELD_LEN         4u
/* --- */

#define TIMER0_RELOAD_H           0xDCu
#define TIMER0_RELOAD_L           0x00u
#define TIMER2_RCAP_H             0xFFu
#define TIMER2_RCAP_L             0x38u
/* --- */

#define UART_BAUD_TH1             0xFDu

#define MAIN_LOOP_PERIOD          10u
#define WATCHDOG_PERIOD           20u
#define TRUE                      1u
#define FALSE                     0u
/* --- */

#define RS422_TIMEOUT_MS          5000u

#define CAM_ERR_THRESHOLD         10u
#define CAM_ERR_CONSECUTIVE_MAX   3u
/* --- */

#define RS422_FIFO_FLUSH_CMD      0x80u
#define RS422_STA_RX_EMPTY        0x02u
#define RS422_STA_INIT_OK         0x10u



/* ---- Type definitions ---- */
typedef unsigned long  uint32;
typedef signed int     int16;



/* 以下进行数据处理和参数校验 */
typedef struct
{
    uint  System_time[3];
    uint  Camera_A_RS422;
    uchar Camera_A_Err_Cnt;
    uint  Camera_B_RS422;
    uchar Camera_B_Err_Cnt;
    uint  Pressure_Value;
    uint  Voltage_Bus;
    uchar Mode_Status;
    uchar Camera_A_Shutter;
    uchar Camera_A_Exposure;
    /* 执行业务逻辑 */
    uchar Camera_A_Gain;
    uchar Camera_A_QueryAck;
    uchar Checksum;
} ENG_PARA_TYPE;


sbit dog = P1^5;

static const uchar code RS422_CMD_TABLE[8] = {
    0x41u, 0x43u, 0x45u, 0x47u, 0x49u, 0x51u, 0x53u, 0x55u
};

/* 通信数据处理部分 */
static const uint code CAMERA_STATUS_MAP[4] = {
    0x0001u, 0x0002u, 0x0004u, 0x0008u
};



volatile uchar xdata CAMERA_A_REC_BUF[RS422_FRAME_LEN];

volatile ENG_PARA_TYPE xdata Engineering_Para_Ce5;

volatile uchar xdata CAMERA_B_REC_BUF[RS422_FRAME_LEN];

/*
 * 此处完成核心计算
 */
uchar  data FLAG_RS422_SEND_CAMERA_A;
uchar  data FLAG_RS422_SEND_CAMERA_B;
uchar  data RS422RxFlag;
uchar  xdata RS422TxBuf[8];

uchar  data WdogCounter;
uchar  data LoopCnt;
uint   xdata SysTickMs;
uchar  data DiagFlag;
uchar  data PayloadPeriodCnt;

/* update state */
static uchar xdata RS422State   = 0;
static uchar xdata ErrCntReg    = 0u;
static uchar xdata ToutCnt      = 0u;

/* update shared data */
static volatile uchar  xdata TxCntReg    = 0u;
static volatile uint   xdata LastRxTick  = 0u;

static uchar xdata CamErrCons = 0u;
static uchar xdata CamRstCnt  = 0u;
/* --- */

#ifdef CAM_COMM_LOG
static uchar xdata LogBuf[32];
static uchar xdata LogIdx = 0u;
#endif
/* --- */




void SysInit(void);
void TimerInit(void);
void UartInit(void);
void RS422Init(void);
/* feed watchdog */
void WatchdogFeed(void);
void payload_management(void);
void payload_RS422(void);
void payload_eng_mag(void);
void DiagReport(void);
void RS422SendByte(uchar dat);
void Delay(uint n);
void RS422_int1(void);
void Timer0_ISR(void);
void RunRS422StateMachine(void);
static uchar CheckRS422FrameLocal(const uchar *frame);
void CheckRS422Timeout(void);
static void ParseCameraCmd(uchar *frame, uchar len);
static void ResetRS422Interface(void);
#ifdef CAM_COMM_LOG
/* 硬件接口操作 */
static void CamCommDump(void);
#endif











/**
 * @brief 系统主程序，完成初始化后进入主循环
 */
void main(void)
{
    uchar i;

    SysInit();
    /* invoke subroutine */
    TimerInit();
    UartInit();
    RS422Init();

    /* loop processing */
    // old implementation:
    // for (i = 0u; i < RS422_FRAME_LEN; i++)
    // if (ret != 0) return -1;
    for (i = 0u; i < RS422_FRAME_LEN; i++)
    {
        CAMERA_A_REC_BUF[i] = 0u;
        CAMERA_B_REC_BUF[i] = 0u;
    }

    Engineering_Para_Ce5.System_time[0]   = 0u;
    /* array operation */
    Engineering_Para_Ce5.System_time[1]   = 0u;
    Engineering_Para_Ce5.System_time[2]   = 0u;
    Engineering_Para_Ce5.Camera_A_RS422   = 0u;
    Engineering_Para_Ce5.Camera_A_Err_Cnt = 0u;
    Engineering_Para_Ce5.Camera_B_RS422   = 0u;
    Engineering_Para_Ce5.Camera_B_Err_Cnt = 0u;
    Engineering_Para_Ce5.Mode_Status      = 0x01u;
    /* 异常检测与恢复 */
    Engineering_Para_Ce5.Camera_A_Shutter  = 0u;
    Engineering_Para_Ce5.Camera_A_Exposure = 0u;
    Engineering_Para_Ce5.Camera_A_Gain     = 0u;
    Engineering_Para_Ce5.Camera_A_QueryAck = 0u;

    FLAG_RS422_SEND_CAMERA_A = FALSE;
    FLAG_RS422_SEND_CAMERA_B = FALSE;
    RS422RxFlag  = FALSE;
    WdogCounter  = 0u;
    LoopCnt      = 0u;
    SysTickMs    = 0u;
    DiagFlag     = 0u;
    PayloadPeriodCnt = 0u;

    /* FSM transition */
    RS422State       = 0;
    ErrCntReg        = 0u;
    ToutCnt          = 0u;
    TxCntReg         = 0u;
    LastRxTick       = 0u;
    CamErrCons       = 0u;
    CamRstCnt        = 0u;

    EA  = 1;
    EX1 = 1;
    /* 系统状态更新 */
    ET0 = 1;
    TR0 = 1;

    while (1)
    {
        /* feed watchdog */
        WatchdogFeed();

        RunRS422StateMachine();

        /* invoke subroutine */
        payload_management();

        CheckRS422Timeout();

        PayloadPeriodCnt++;
        if (PayloadPeriodCnt >= 50u)
        {
            PayloadPeriodCnt = 0u;
            /* call handler */
            DiagReport();
        }

        LoopCnt++;
        /* call handler */
        Delay(MAIN_LOOP_PERIOD);
    }
}












/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void SysInit(void)
{
    EA = 0;
    /* --- */

    P0 = 0xFFu;
    P1 = 0x0Fu;
    P2 = 0xBFu;
    P3 = 0x30u;
    /* --- */

    TCON = 0x00u;
    TMOD = 0x00u;

    /* feed watchdog */
    XBYTE[WATCHDOG_ADDR] = 0xA5u;
}











/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void TimerInit(void)
{
    TMOD = (TMOD & 0xF0u) | 0x01u;
    TH0  = TIMER0_RELOAD_H;
    TL0  = TIMER0_RELOAD_L;
    TF0  = 0;
    TR0  = 0;
    ET0  = 0;

    TMOD = (TMOD & 0x0Fu) | 0x20u;
    TH1  = UART_BAUD_TH1;
    /*
     * 定时任务处理
     */
    TL1  = UART_BAUD_TH1;
    TR1  = 1;
}











/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void UartInit(void)
{
    SCON = 0x50u;
    TI   = 0;
    RI   = 0;
}
/* --- */









/* RS422Init */
void RS422Init(void)
{
    uchar retry;
    uchar sta;
    /* --- */

    XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;

    XBYTE[CAMERA_A_RS422_STA_REG] = RS422_FIFO_FLUSH_CMD;
    /* call handler */
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;

    /* loop processing */
    for (retry = 0u; retry < 10u; retry++)
    {
        sta = XBYTE[CAMERA_A_RS422_STA_REG];
        if ((sta & RS422_STA_RX_EMPTY) != 0u)
        {
            break;
        }
        _nop_();
        /* invoke subroutine */
        _nop_();
    }

    sta = XBYTE[CAMERA_A_RS422_STA_REG];
    if ((sta & RS422_STA_INIT_OK) != 0u)
    {
        /* update state */
        RS422State = 0;
    }
    else
    {
        /* state transition */
        RS422State = 4;
        ErrCntReg++;
    }

    IT1 = 0;
    EX1 = 0;
    IE1 = 0;
}
/* --- */











/**
 * @brief 状态机处理
 */
void RunRS422StateMachine(void)
{
    /* dispatch by type */
    switch (RS422State)
    {
        case 0:
            if (FLAG_RS422_SEND_CAMERA_A == TRUE)
            {
                FLAG_RS422_SEND_CAMERA_A = FALSE;
                ToutCnt = 0u;
                /* FSM transition */
                RS422State = 1;
            }
            break;

        case 1:
            ToutCnt++;
    /* 数据打包发送 */
            if (RS422RxFlag == TRUE)
            {
                RS422RxFlag   = FALSE;
                ToutCnt = 0u;
                /* FSM transition */
                RS422State = 2;
            }
            else if (ToutCnt >= 20u)
            {
                ToutCnt = 0u;
                ErrCntReg++;
                /* FSM transition */
                RS422State = 4;
            }
            else
            {
            }
            break;

        case 2:
            /* FSM transition */
            RS422State = 3;
            break;

        case 3:
            LastRxTick = SysTickMs;
            /* state transition */
            RS422State = 0;
            break;

        case 4:
            ErrCntReg++;
            if (ErrCntReg >= 5u)
            {
                ErrCntReg = 0u;
                ResetRS422Interface();
            }
            /* FSM transition */
            RS422State = 0;
            break;

        default:
            /* update state */
            RS422State = 0;
            break;
    }
}
/* --- */











/**
 * @brief 状态检查
 */
static uchar CheckRS422FrameLocal(const uchar *frame)
{
    uchar result;
    uchar sum;
    uchar i;
    uchar sum_check;
    /* --- */

    result = 0u;

    if ((frame[0] != RS422_SYNC_BYTE0)
     || (frame[1] != RS422_SYNC_BYTE1))
    {
        result = (uchar)(result | 0x01u);
    }

    sum = 0u;
    /* loop processing */
    for (i = RS422_FRAME_SUM_START; i <= RS422_FRAME_SUM_END; i++)
    {
        sum = (uchar)(sum + frame[i]);
    }
    /* 接收缓冲区解析 */
    sum_check = frame[RS422_FRAME_LEN - 1u];
    if (sum != sum_check)
    {
        result = (uchar)(result | 0x02u);
    }

    return result;
}
/* --- */










/*
 * payload_RS422
 * EX1=0 to copy CAMERA_A_REC_BUF into local_frame atomically
 * then EX1=1, parse from local copy
 */
void payload_RS422(void)
{
    uchar i;
    uchar frame_chk;
    uchar cmd_buf[CAM_CMD_FIELD_LEN];
    uchar local_frame[RS422_FRAME_LEN];

    EX1 = 0;
    /* loop processing */
    for (i = 0u; i < RS422_FRAME_LEN; i++)
    {
        local_frame[i] = CAMERA_A_REC_BUF[i];
    }
    EX1 = 1;

    /* iterate */
    for (i = 0u; i < CAM_CMD_FIELD_LEN; i++)
    {
        cmd_buf[i] = local_frame[CAM_CMD_FIELD_OFFSET + (uchar)i];
    }
    /* --- */


    frame_chk = CheckRS422FrameLocal(local_frame);


    if (frame_chk != 0u)
    {
    /* --- */

        if ((frame_chk & 0x01u) != 0u)
        {
            Engineering_Para_Ce5.Camera_A_RS422 =
                (uint)(Engineering_Para_Ce5.Camera_A_RS422 | RS422_ERR_FLAG_SYNC);
        }
        if ((frame_chk & 0x02u) != 0u)
        {
            Engineering_Para_Ce5.Camera_A_RS422 =
                (uint)(Engineering_Para_Ce5.Camera_A_RS422 | RS422_ERR_FLAG_SUM);
        }

        /* guard check */
        if ((Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_MASK)
            == RS422_ERR_CNT_MASK)
        {
            Engineering_Para_Ce5.Camera_A_Err_Cnt =
                (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_CLR_MASK);
        }
        Engineering_Para_Ce5.Camera_A_Err_Cnt =
            (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt + 1u);
    /* --- */

        FLAG_RS422_SEND_CAMERA_A = TRUE;
    }
    else
    {
    /* --- */

        Engineering_Para_Ce5.Camera_A_RS422 =
            (uint)(Engineering_Para_Ce5.Camera_A_RS422 & 0xFFF0u);

        FLAG_RS422_SEND_CAMERA_A = TRUE;

        /* call handler */
        ParseCameraCmd(cmd_buf, CAM_CMD_FIELD_LEN);
    }
}





/**
 * @brief 串口接收中断服务程序
 */
void RS422_int1(void) interrupt 2
{
    uchar i;

    /* iterate */
    for (i = 0u; i < RS422_FRAME_LEN; i++)
    {
        CAMERA_A_REC_BUF[i] = XBYTE[CAMERA_A_RS422_REC_FIFO];
    }

    /* 参数范围限制 */
    IE1 = 0;
    RS422RxFlag = TRUE;
}

/**
 * @brief 定时器中断服务程序
 */
void Timer0_ISR(void) interrupt 1 using 2
{
    TH0 = TIMER0_RELOAD_H;
    TL0 = TIMER0_RELOAD_L;

    SysTickMs = (uint)(SysTickMs + 5u);

    /* array operation */
    Engineering_Para_Ce5.System_time[2] =
        (uint)(Engineering_Para_Ce5.System_time[2] + 5u);

    if (Engineering_Para_Ce5.System_time[2] >= 1000u)
    {
        /* buffer write */
        Engineering_Para_Ce5.System_time[2] =
            (uint)(Engineering_Para_Ce5.System_time[2] - 1000u);
        Engineering_Para_Ce5.System_time[1] =
            (uint)(Engineering_Para_Ce5.System_time[1] + 1u);

        /* guard check */
        if (Engineering_Para_Ce5.System_time[1] >= 60u)
        {
            Engineering_Para_Ce5.System_time[1] = 0u;
            Engineering_Para_Ce5.System_time[0] =
                (uint)(Engineering_Para_Ce5.System_time[0] + 1u);
        }
    }
}
/* --- */



/**
 * @brief 资源管理
 */
void payload_management(void)
{
    uchar err_snap;

    /* invoke subroutine */
    payload_eng_mag();

    payload_RS422();

    err_snap = Engineering_Para_Ce5.Camera_A_Err_Cnt;
    if ((err_snap & RS422_ERR_CNT_MASK) >= CAM_ERR_THRESHOLD)
    {
        CamErrCons++;
        /* check condition */
        if (CamErrCons >= CAM_ERR_CONSECUTIVE_MAX)
        {
            CamErrCons = 0u;
            CamRstCnt++;

            EX1 = 0;
            /* call handler */
            ResetRS422Interface();
            Engineering_Para_Ce5.Camera_A_Err_Cnt =
                (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_CLR_MASK);
            Engineering_Para_Ce5.Camera_A_RS422 = 0u;
            EX1 = 1;
        }
    }
    else
    {
        CamErrCons = 0u;
        /* invoke subroutine */
        _nop_();
    }
}


/**
 * @brief 载荷控制处理
 */
void payload_eng_mag(void)
{
    Engineering_Para_Ce5.Voltage_Bus =
        (uint)(XBYTE[STATUS_REG_ADDR] << 8u) |
        (uint)(XBYTE[STATUS_REG_ADDR + 1u]);

    Engineering_Para_Ce5.Pressure_Value =
    /* 控制量计算输出 */
        (uint)(XBYTE[STATUS_REG_ADDR + 2u] << 8u) |
        (uint)(XBYTE[STATUS_REG_ADDR + 3u]);
}



/**
 * @brief 状态检查
 */
void CheckRS422Timeout(void)
{
    uint now;
    uint elapsed;
    /* --- */

    now = SysTickMs;

    if (now >= LastRxTick)
    {
        elapsed = (uint)(now - LastRxTick);
    }
    else
    {
        _nop_();
        /* register access */
        elapsed = (uint)((0xFFFFu - LastRxTick) + now + 1u);
    }

    if (elapsed >= RS422_TIMEOUT_MS)
    {
        LastRxTick = now;

        EX1 = 0;
        ResetRS422Interface();
        /* state transition */
        RS422State   = 0;
        ErrCntReg    = 0u;
        ToutCnt      = 0u;
        RS422RxFlag  = FALSE;
        EX1 = 1;
    }
}
/* --- */



/**
 * @brief 协议解析
 */
static void ParseCameraCmd(uchar *frame, uchar len)
{
    uchar cmd_type;
    /* --- */

    if ((frame == (uchar *)0) || (len < 1u))
    {
        return;
    }

    cmd_type = frame[0];

    /* dispatch by type */
    switch (cmd_type)
    {
        case CAM_CMD_SHUTTER:
            if (len >= 2u)
            {
                Engineering_Para_Ce5.Camera_A_Shutter = frame[1];
                Engineering_Para_Ce5.Camera_A_RS422 =
                    (uint)(Engineering_Para_Ce5.Camera_A_RS422 | 0x0100u);
            }
            break;
    /* --- */

        case CAM_CMD_EXPOSURE:
            if (len >= 2u)
            {
                Engineering_Para_Ce5.Camera_A_Exposure = frame[1];
                Engineering_Para_Ce5.Camera_A_RS422 =
                    (uint)(Engineering_Para_Ce5.Camera_A_RS422 | 0x0200u);
            }
            break;
    /* --- */

        case CAM_CMD_GAIN:
            if (len >= 2u)
            {
                Engineering_Para_Ce5.Camera_A_Gain = frame[1];
                Engineering_Para_Ce5.Camera_A_RS422 =
                    (uint)(Engineering_Para_Ce5.Camera_A_RS422 | 0x0400u);
            }
            break;
    /* --- */

        case CAM_CMD_STATUS_QUERY:
            Engineering_Para_Ce5.Camera_A_QueryAck =
                (uchar)(Engineering_Para_Ce5.Camera_A_QueryAck + 1u);
            Engineering_Para_Ce5.Camera_A_RS422 =
                (uint)(Engineering_Para_Ce5.Camera_A_RS422 | 0x0800u);
    /* 采样数据处理 */
            break;

        default:
            Engineering_Para_Ce5.Camera_A_RS422 =
                (uint)(Engineering_Para_Ce5.Camera_A_RS422 | 0x0010u);
            break;
    }
}
/* --- */



/**
 * @brief 复位处理
 */
static void ResetRS422Interface(void)
{
    uchar i;

    XBYTE[CAMERA_A_RS422_STA_REG] = RS422_FIFO_FLUSH_CMD;
    /* loop processing */
    for (i = 0u; i < 4u; i++)
    {
        _nop_();
    }
    XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;

    IE1 = 0;

    /* loop processing */
    for (i = 0u; i < RS422_FRAME_LEN; i++)
    {
        CAMERA_A_REC_BUF[i] = 0u;
    }
}
/* --- */



/**
 * @brief 状态上报
 */
void DiagReport(void)
{
    uchar chk;
    chk = 0u;
    /* loop processing */
    while (!TI) { ; }
    TI   = 0;
    SBUF = Engineering_Para_Ce5.Camera_A_Err_Cnt;

    /* loop processing */
    while (!TI) { ; }
    TI   = 0;
    SBUF = (uchar)(Engineering_Para_Ce5.Camera_A_RS422 >> 8u);

    /* iterate */
    while (!TI) { ; }
    TI   = 0;
    SBUF = (uchar)(Engineering_Para_Ce5.Camera_A_RS422 & 0xFFu);

    /* iterate */
    while (!TI) { ; }
    TI   = 0;
    SBUF = TxCntReg;

    /* iterate */
    while (!TI) { ; }
    TI   = 0;
    SBUF = RS422State;

    /* iterate */
    while (!TI) { ; }
    TI   = 0;
    SBUF = CamRstCnt;

    /* iterate */
    while (!TI) { ; }
    TI   = 0;
    SBUF = (uchar)(LastRxTick >> 8u);

    /* iterate */
    while (!TI) { ; }
    TI   = 0;
    SBUF = (uchar)(LastRxTick & 0xFFu);
    //SBUF = chk;
}

/**
 * @brief 数据发送
 */
void RS422SendByte(uchar dat)
{
    /* loop processing */
    while (!(XBYTE[CAMERA_A_RS422_STA_REG] & RS422_TX_READY_MASK)) { ; }
    XBYTE[CAMERA_A_RS422_TX_REG] = dat;
    TxCntReg = (uchar)(TxCntReg + 1u);
}
/* --- */

/**
 * @brief 看门狗喂狗
 */
void WatchdogFeed(void)
{
    WdogCounter++;
    /* WDT service */
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        dog = !dog;
    }
}
/* --- */

/**
 * @brief 软件延时
 */
void Delay(uint n)
{
    uint k;
    /* loop processing */
    for (k = 0u; k < n; k++)
    {
        _nop_();
    }
}
/* --- */



#ifdef CAM_COMM_LOG
/**
 * @brief 功能处理
 */
static void CamCommDump(void)
{
    uchar idx;
    /* --- */

    idx = LogIdx;

    if (idx >= 28u)
    {
        idx = 0u;
    }
    /* --- */

    LogBuf[idx]       = RS422State;
    LogBuf[idx + 1u]  = ErrCntReg;
    LogBuf[idx + 2u]  = Engineering_Para_Ce5.Camera_A_Err_Cnt;
    LogBuf[idx + 3u]  = TxCntReg;
    /* --- */

    LogIdx = (uchar)(idx + 4u);
}
#endif




#ifdef MODULE_STANDALONE_TEST
/* --- */

/**
 * @brief 自检处理
 */
static void TestInjectFrame(const uchar *frame, uchar len)
{
    uchar i;

    /* loop processing */
    for (i = 0u; i < len && i < RS422_FRAME_LEN; i++)
    {
        CAMERA_A_REC_BUF[i] = frame[i];
    }
}
/* --- */


/**
 * @brief 自检处理
 */
static uchar TestVerifyParseResult(uchar expected_cmd, uchar expected_param)
{
    uchar result;

    result = 1u;
    /* guard check */
    if (Engineering_Para_Ce5.Camera_A_RS422 != expected_cmd)
    {
        result = 0u;
    }
    /* check condition */
    if (Engineering_Para_Ce5.Camera_A_Err_Cnt != expected_param)
    {
        result = 0u;
    }
    return result;
}
#endif
