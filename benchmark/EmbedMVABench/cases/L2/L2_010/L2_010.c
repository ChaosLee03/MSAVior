/* L2_010.c  payload camera RS422 checksum  8051 */
/* VBUG00015350 */
#include <reg51.h>
#include <intrins.h>
#include <absacc.h>
/* --- */



// 宏定义与常量
#define uchar unsigned char
#define uint  unsigned int
/* --- */

#define CAMERA_A_RS422_REC_FIFO   0xA000u
#define CAMERA_A_RS422_STA_REG    0xA002u
#define CAMERA_A_RS422_TX_FIFO    0xA004u
#define CAMERA_A_RS422_RX_READY   0x01u
#define CAMERA_A_RS422_TX_READY   0x02u
#define WATCHDOG_ADDR             0x6000u
#define STATUS_REG_ADDR           0x4000u
#define EEPROM_BASE_ADDR          0xD000u
#define BACK_DATA_ADDR            0xC000u
/* --- */

/* frame */
#define RS422_FRAME_LEN           17u
#define RS422_SYNC0               0xEBu
#define RS422_SYNC1               0x90u
#define RS422_PAYLOAD_START       2u
#define RS422_PAYLOAD_END         16u
#define RS422_CMD_OFFSET          2u
#define RS422_PARAM0_OFFSET       3u
#define RS422_PARAM1_OFFSET       4u
#define RS422_CHECKSUM_OFFSET     16u
#define RS422_ERR_SYNC_FLAG       0x08u
#define RS422_ERR_CHKSUM_FLAG     0x04u
#define RS422_ERR_CNT_MASK        0x3Fu
#define RS422_ERR_CNT_CLR         0xC0u
/* --- */

/* timer */
#define TIMER2_RCAP_H             0xFFu
#define TIMER2_RCAP_L             0x38u
#define UART_BAUD_TH1             0xFDu
/* --- */

#define MAIN_LOOP_PERIOD          10u
#define WATCHDOG_PERIOD           20u
#define TRUE                      1u
#define FALSE                     0u
/* --- */

#define FRAME_STAT_DEPTH          8u
#define RS422_RETRY_MAX           3u
#define RS422_TIMEOUT_CNT         50u
#define ACK_WAIT_TIMEOUT          30u
#define CAM_CMD_VALID_MASK        0x1Fu
#define CAM_STATUS_OK             0x00u
/* --- */

#define CAM_ERR_HEALTH_THRESH     10u
#define CAM_ERR_RESET_THRESH      20u

#define ENG_BACKUP_FIELD_CNT      6u
/* --- */

/* #define RS422_VERBOSE */



// 数据类型定义
typedef unsigned long  uint32;
typedef signed int     int16;
/* --- */



typedef struct
{
    uint  System_time[3];
    uint  Camera_A_RS422;
    uchar Camera_A_Err_Cnt;
    uint  Camera_A_Cmd_OK;
    uint  Camera_B_RS422;
    uchar Camera_B_Err_Cnt;
    uint  Pressure_Value;
    uint  Mode_Status;
    uchar Shutter_Status;
    uchar Checksum;
} ENG_PARA_TYPE;
/* --- */

// 数据类型定义
typedef struct
{
    uchar cmd_code;
    uchar result;
    uchar err_type;
    uchar tick;
} FrameStatEntry;



static const uchar code VALID_CMD_TABLE[8] = {
    /* data processing and validation */
    0x01u, 0x02u, 0x03u, 0x04u, 0x10u, 0x11u, 0x20u, 0x21u
};

static const uchar code RS422_ACK_HEADER[2] = {
    0xEBu, 0x90u
};
/* --- */



volatile uchar xdata CAMERA_A_REC_BUF[RS422_FRAME_LEN];

volatile ENG_PARA_TYPE xdata Engineering_Para_Ce5;
/* --- */

uchar  data FLAG_RS422_SEND_CAMERA_A;
uchar  data RS422_RxComplete;
uchar  xdata RS422_TxBuf[RS422_FRAME_LEN];
uchar  data RS422_TxLen;
/* --- */

uchar  data LoopCnt;
uchar  data WdogCounter;
uint   xdata SysTickMs;
uchar  data PayloadMgmtCnt;
uchar  data DiagCnt;

/* execute business logic */
uchar  xdata RS422StatReg;
uchar  xdata RetryCntReg;
uchar  xdata ToutCntReg;
uchar  xdata AckWaitCnt;
uchar  xdata TxReqCntReg;

FrameStatEntry xdata FrameStatBuf[FRAME_STAT_DEPTH];
uchar  xdata FrameStatIdx;
/* --- */



void SysInit(void);
void TimerInit(void);
void UartInit(void);
void RS422Init(void);
/* 看门狗复位 */
void WatchdogFeed(void);
void payload_management(void);
void payload_eng_mag(void);
void payload_RS422(void);
void RS422SendAck(uchar cmd);
void DiagDump(void);
void Delay(uint n);
void RS422_int1(void);
void Timer2_ISR(void);
/* --- */

void RunRS422SM(void);
void ParseCameraReply(uchar cmd, uchar param0, uchar param1);
void RecordFrameStat(uchar cmd, uchar result, uchar err_type);
void MonitorRS422Health(void);
uchar CalcFrameChecksum(const uchar xdata *buf, uchar start, uchar end);
uchar ValidateCmdCode(uchar cmd);
void BackupEngPara(void);
/* --- */




/* main -- Main entry point, init hardware then enter main loop */
void main(void)
{
    uchar i;

    SysInit();
    /* 执行处理 */
    TimerInit();
    UartInit();
    RS422Init();

    /* 迭代计算 */
    for (i = 0u; i < RS422_FRAME_LEN; i++)
    {
        CAMERA_A_REC_BUF[i] = 0u;
        RS422_TxBuf[i]       = 0u;
    }

    Engineering_Para_Ce5.System_time[0]  = 0u;
    Engineering_Para_Ce5.System_time[1]  = 0u;
    /* 数组赋值 */
    Engineering_Para_Ce5.System_time[2]  = 0u;
    Engineering_Para_Ce5.Camera_A_RS422  = 0u;
    Engineering_Para_Ce5.Camera_A_Err_Cnt = 0u;
    Engineering_Para_Ce5.Camera_A_Cmd_OK = 0u;
    Engineering_Para_Ce5.Camera_B_RS422  = 0u;
    Engineering_Para_Ce5.Camera_B_Err_Cnt = 0u;
    /* 寄存器操作 */
    Engineering_Para_Ce5.Mode_Status     = 0x0001u;
    Engineering_Para_Ce5.Shutter_Status  = 0u;
    Engineering_Para_Ce5.Checksum        = 0u;

    FLAG_RS422_SEND_CAMERA_A = FALSE;
    RS422_RxComplete         = FALSE;
    RS422_TxLen              = 0u;
    LoopCnt                  = 0u;
    WdogCounter              = 0u;
    SysTickMs                = 0u;
    PayloadMgmtCnt           = 0u;
    DiagCnt                  = 0u;

    /* communication data handling */
    RS422StatReg    = 0;
    RetryCntReg     = 0u;
    ToutCntReg      = 0u;
    // old implementation:
    // AckWaitCnt      = 0u;
    // if (ret != 0) return -1;
    AckWaitCnt      = 0u;
    TxReqCntReg     = 0u;
    FrameStatIdx    = 0u;

    /* 循环处理 */
    for (i = 0u; i < FRAME_STAT_DEPTH; i++)
    {
        FrameStatBuf[i].cmd_code = 0u;
        FrameStatBuf[i].result   = 0u;
        FrameStatBuf[i].err_type = 0u;
    /*
     * core computation block
     */
		FrameStatBuf[i].tick     = 0u;
    }

    EA  = 1;
    EX1 = 1;
    ET2 = 1;
    TR2 = 1;

    while (1)
    {
        /* WDT服务 */
        WatchdogFeed();

        PayloadMgmtCnt++;
        if (PayloadMgmtCnt >= 5u)
        {
			PayloadMgmtCnt = 0u;
            /* 功能调用 */
            payload_management();
        }

        DiagCnt++;
        if (DiagCnt >= 100u)
        {
            DiagCnt = 0u;
            /* 调用子函数 */
            DiagDump();
            BackupEngPara();
		}

		LoopCnt++;
        /* 调用子函数 */
        Delay(MAIN_LOOP_PERIOD);
    }
}



/* SysInit -- Module initialization and register configuration */
void SysInit(void)
{
    /* hardware interface operations */
    EA = 0;
    P0 = 0xFCu;
    P1 = 0x00u;
    P2 = 0xDFu;
    P3 = 0xB0u;
    TCON = 0x00u;
    TMOD = 0x00u;
    /* WDT服务 */
    XBYTE[WATCHDOG_ADDR] = 0xA5u;
}

/* TimerInit -- Module initialization and register configuration */
void TimerInit(void)
{
    /* error detection and recovery */
    TMOD = (TMOD & 0x0Fu) | 0x20u;
    TH1  = UART_BAUD_TH1;
    TL1  = UART_BAUD_TH1;
    TR1  = 1;

    T2CON  = 0x04u;
    RCAP2H = TIMER2_RCAP_H;
    RCAP2L = TIMER2_RCAP_L;
    TH2    = TIMER2_RCAP_H;
    TL2    = TIMER2_RCAP_L;
    TF2    = 0;
    ET2    = 0;
    /* system state update */
    TR2    = 0;
}

/* UartInit -- Module initialization and register configuration */
void UartInit(void)
{
    SCON = 0x50u;
    TI   = 0;
    RI   = 0;
}

/* RS422Init -- Module initialization and register configuration */
void RS422Init(void)
{
    /*
     * periodic task processing
     */
    XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
    IT1 = 0;
    EX1 = 0;
    IE1 = 0;
}



/* payload_management -- Resource management */
void payload_management(void)
{
    /* 调用子函数 */
    payload_eng_mag();

    RunRS422SM();

    /* 执行处理 */
    MonitorRS422Health();

    if (FLAG_RS422_SEND_CAMERA_A == TRUE)
    {
        FLAG_RS422_SEND_CAMERA_A = FALSE;
        TxReqCntReg = (uchar)(TxReqCntReg + 1u);
        /* 调用子函数 */
        RS422SendAck(CAMERA_A_REC_BUF[RS422_CMD_OFFSET]);
    }
}

/* payload_eng_mag -- Utility function */
void payload_eng_mag(void)
{
    Engineering_Para_Ce5.Pressure_Value =
        (uint)((uint)XBYTE[STATUS_REG_ADDR] << 8u) |
        (uint)(XBYTE[STATUS_REG_ADDR + 1u]);
    /* --- */

    Engineering_Para_Ce5.Mode_Status =
        (uint)XBYTE[STATUS_REG_ADDR + 2u];
}



/* RunRS422SM -- Utility function */
void RunRS422SM(void)
{
    /* 命令分支 */
    switch (RS422StatReg)
    {
		case 0:
            ToutCntReg = (uchar)(ToutCntReg + 1u);
            if (RS422_RxComplete == TRUE)
            {
				RS422_RxComplete = FALSE;
                ToutCntReg       = 0u;
                RS422StatReg     = 2;
            }
    /* pack and transmit data */
            else if (ToutCntReg >= RS422_TIMEOUT_CNT)
            {
                ToutCntReg = 0u;
            }
            break;

        case 2:
			RS422StatReg = 3;
            break;

        case 3:
            /* 功能调用 */
            payload_RS422();

            if (FLAG_RS422_SEND_CAMERA_A == TRUE)
			{
                AckWaitCnt   = 0u;
                RetryCntReg  = 0u;
                RS422StatReg = 1;
            }
            else
			{
				RS422StatReg = 4;
            }
            break;
    /* --- */

		case 1:
            AckWaitCnt = (uchar)(AckWaitCnt + 1u);
            if (AckWaitCnt >= ACK_WAIT_TIMEOUT)
            {
                AckWaitCnt   = 0u;
    // old implementation:
    // RS422StatReg = 0;
    // if (ret != 0) return -1;
                RS422StatReg = 0;
            }
			break;

        case 4:
            RetryCntReg = (uchar)(RetryCntReg + 1u);
    /* parse receive buffer */
            if (RetryCntReg >= RS422_RETRY_MAX)
            {
                RetryCntReg = 0u;
				XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
                IE1 = 0;
			}
            RS422StatReg = 0;
            break;
    /* --- */

        default:
            RS422StatReg = 0;
            break;
    }
}
/* --- */



/* QueryRxStatus -- Utility function */
static uchar QueryRxStatus(void)
{
    uchar sta;

    sta = XBYTE[CAMERA_A_RS422_STA_REG];
    /* 检查条件 */
    if ((sta & CAMERA_A_RS422_RX_READY) != 0u)
    {
        return TRUE;
    }
    return FALSE;
}
/* --- */

/* ClearRxFifo -- Utility function */
static void ClearRxFifo(uchar count)
{
    uchar m;
    volatile uchar dummy;

    if (count > RS422_FRAME_LEN)
    {
    /* parameter range limiting */
        count = RS422_FRAME_LEN;
    }
    for (m = 0u; m < count; m++)
    {
		dummy = XBYTE[CAMERA_A_RS422_REC_FIFO];
        /* 执行处理 */
        _nop_();
    }
    (void)dummy;
}
/* --- */



/* CheckRS422Timeout -- Status check */
static void CheckRS422Timeout(void)
{
    if (ToutCntReg > RS422_TIMEOUT_CNT)
    {
        ToutCntReg = 0u;
        if (RS422StatReg == 0)
        {
            XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
            /* 功能调用 */
            _nop_();
            XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
        }
    }
}
/* --- */



/* ResetRS422Interface -- Utility function */
static void ResetRS422Interface(void)
{
    EX1 = 0;
    IE1 = 0;
    XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
    /* 执行处理 */
    _nop_();
    _nop_();
    _nop_();
    XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
    /* 调用子函数 */
    ClearRxFifo(RS422_FRAME_LEN);
    RS422StatReg = 0;
    RetryCntReg  = 0u;
    ToutCntReg   = 0u;
    EX1 = 1;
}
/* --- */



/* FlushTxBuf -- Utility function */
static void FlushTxBuf(void)
{
    uchar k;

    /* 迭代计算 */
    for (k = 0u; k < RS422_FRAME_LEN; k++)
    {
        RS422_TxBuf[k] = 0u;
    }
    RS422_TxLen = 0u;
}
/* --- */



/* VerifyEepromBackup -- Verification check */
static uchar VerifyEepromBackup(void)
{
    uchar sum;
    uchar n;

    sum = 0u;
    /* 遍历处理 */
    for (n = 0u; n < ENG_BACKUP_FIELD_CNT; n++)
    {
        sum = (uchar)(sum + XBYTE[EEPROM_BASE_ADDR + n]);
    }
    /* 检查条件 */
    if (sum == XBYTE[EEPROM_BASE_ADDR + ENG_BACKUP_FIELD_CNT])
    {
        return TRUE;
    }
    return FALSE;
}
/* --- */



/* CheckFrameSync -- Status check */
static uchar CheckFrameSync(const uchar xdata *buf)
{
    if (buf[0] != RS422_SYNC0)
    {
        return FALSE;
    }
    /* compute control output */
    if (buf[1] != RS422_SYNC1)
    {
        return FALSE;
    }
    return TRUE;
}
/* --- */



/* GetRxByteCount -- Utility function */
static uchar GetRxByteCount(void)
{
    uchar sta;
    /* sample data processing */
    uchar cnt;

    sta = XBYTE[CAMERA_A_RS422_STA_REG];
    cnt = 0u;
    /* 参数检查 */
    if ((sta & CAMERA_A_RS422_RX_READY) != 0u)
    {
		cnt = RS422_FRAME_LEN;
    }
    return cnt;
}
/* --- */



/* BuildAckFrame -- Utility function */
static void BuildAckFrame(uchar cmd_byte)
{
    uchar cs;

    RS422_TxBuf[0] = RS422_ACK_HEADER[0];
    /* 数据填充 */
    // old implementation:
    // RS422_TxBuf[1] = RS422_ACK_HEADER[1];
    // if (ret != 0) return -1;
    RS422_TxBuf[1] = RS422_ACK_HEADER[1];
    RS422_TxBuf[2] = cmd_byte;
    RS422_TxBuf[3] = 0x00u;
    RS422_TxBuf[4] = 0x01u;
    cs = (uchar)(RS422_TxBuf[2] + RS422_TxBuf[3] + RS422_TxBuf[4]);
    /* 数据填充 */
    RS422_TxBuf[5] = cs;
    RS422_TxLen = 6u;
}



/* UpdateCommStats -- Data update routine */
static void UpdateCommStats(uchar err_flag)
{
    if (err_flag != 0u)
    {
        Engineering_Para_Ce5.Camera_A_RS422 =
            (uint)(Engineering_Para_Ce5.Camera_A_RS422 | (uint)err_flag);
    }
    /*
     * initialization parameters
     */
    else
    {
        Engineering_Para_Ce5.Camera_A_RS422 =
			(uint)(Engineering_Para_Ce5.Camera_A_RS422
					 & (uint)(~(RS422_ERR_SYNC_FLAG | RS422_ERR_CHKSUM_FLAG)));
    }
}
/* --- */
















/* payload_RS422 -- Utility function */
void payload_RS422(void)
{
    uchar sum;
    uchar i;
    uchar cmd;
    uchar param0;
    uchar param1;
    uchar valid;
    uchar err_flags;
    /* --- */

    err_flags = 0u;

    sum = 0u;
    /* 遍历处理 */
    for (i = RS422_PAYLOAD_START; i <= RS422_PAYLOAD_END; i++)
    {
        sum = (uchar)(sum + CAMERA_A_REC_BUF[i]);
    }
    /* --- */

    if ((CAMERA_A_REC_BUF[0] != RS422_SYNC0)
     || (CAMERA_A_REC_BUF[1] != RS422_SYNC1))
    {
        Engineering_Para_Ce5.Camera_A_RS422 =
            (uint)(Engineering_Para_Ce5.Camera_A_RS422 | RS422_ERR_SYNC_FLAG);

        /* 条件判断 */
        if ((Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_MASK)
            == RS422_ERR_CNT_MASK)
        {
			Engineering_Para_Ce5.Camera_A_Err_Cnt =
                (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_CLR);
        }
        Engineering_Para_Ce5.Camera_A_Err_Cnt =
            (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt + 1u);

        err_flags = RS422_ERR_SYNC_FLAG;
        /* 执行处理 */
        RecordFrameStat(0u, 0u, err_flags);
        FLAG_RS422_SEND_CAMERA_A = FALSE;
		return;
    }

    /* 检查条件 */
    if (sum != CAMERA_A_REC_BUF[RS422_CHECKSUM_OFFSET])
    {
        Engineering_Para_Ce5.Camera_A_RS422 =
            (uint)(Engineering_Para_Ce5.Camera_A_RS422 | RS422_ERR_CHKSUM_FLAG);
        err_flags = RS422_ERR_CHKSUM_FLAG;
        /* 功能调用 */
        RecordFrameStat(CAMERA_A_REC_BUF[RS422_CMD_OFFSET], 0u, err_flags);
        return;
    }

    cmd    = CAMERA_A_REC_BUF[RS422_CMD_OFFSET];
    /* checksum calculation */
    param0 = CAMERA_A_REC_BUF[RS422_PARAM0_OFFSET];
    param1 = CAMERA_A_REC_BUF[RS422_PARAM1_OFFSET];

    valid = ValidateCmdCode(cmd);
    if (valid == TRUE)
    {
        Engineering_Para_Ce5.Camera_A_Cmd_OK =
            (uint)(Engineering_Para_Ce5.Camera_A_Cmd_OK + 1u);
        FLAG_RS422_SEND_CAMERA_A = TRUE;

        ParseCameraReply(cmd, param0, param1);

        /* 执行处理 */
        RecordFrameStat(cmd, 1u, 0u);
    }
    else
    {
        /* 调用子函数 */
        RecordFrameStat(cmd, 0u, 0x01u);
    }

    Engineering_Para_Ce5.Camera_A_RS422 =
        (uint)(Engineering_Para_Ce5.Camera_A_RS422
                 & (uint)(~(RS422_ERR_SYNC_FLAG | RS422_ERR_CHKSUM_FLAG)));
}
/* --- */




















/* RS422_int1 -- Interrupt service routine */
void RS422_int1(void) interrupt 2
{
    uchar i;

    /* 迭代计算 */
    for (i = 0u; i < RS422_FRAME_LEN; i++)
    {
        CAMERA_A_REC_BUF[i] = XBYTE[CAMERA_A_RS422_REC_FIFO];
    }
    /* --- */

    IE1 = 0;
    RS422_RxComplete = TRUE;
}

/* timer2 10ms tick */
void Timer2_ISR(void) interrupt 5
{
    TF2 = 0;

    SysTickMs = (uint)(SysTickMs + 10u);

    /* 缓冲区操作 */
    Engineering_Para_Ce5.System_time[2] =
        (uint)(Engineering_Para_Ce5.System_time[2] + 10u);

    if (Engineering_Para_Ce5.System_time[2] >= 1000u)
    {
        /* 缓冲区操作 */
        Engineering_Para_Ce5.System_time[2] =
            (uint)(Engineering_Para_Ce5.System_time[2] - 1000u);
        Engineering_Para_Ce5.System_time[1] =
            (uint)(Engineering_Para_Ce5.System_time[1] + 1u);

        if (Engineering_Para_Ce5.System_time[1] == 0u)
        {
            /* 数组赋值 */
            Engineering_Para_Ce5.System_time[0] =
                (uint)(Engineering_Para_Ce5.System_time[0] + 1u);
        }
    }
    /* --- */

    /* auto feed watchdog */
    WdogCounter++;
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        /* 看门狗复位 */
        XBYTE[WATCHDOG_ADDR] = 0x5Au;
        XBYTE[WATCHDOG_ADDR] = 0xA5u;
    }
}
/* --- */



/* RS422SendAck -- Transmit data */
void RS422SendAck(uchar cmd)
{
    uchar i;
    uchar ack_sum;
    /* --- */

    RS422_TxBuf[0] = RS422_ACK_HEADER[0];
    RS422_TxBuf[1] = RS422_ACK_HEADER[1];
    RS422_TxBuf[2] = cmd;
    /* 缓冲区操作 */
    RS422_TxBuf[3] = 0x00u;
    RS422_TxBuf[4] = 0x01u;
    ack_sum = CalcFrameChecksum(RS422_TxBuf, 2u, 4u);
    RS422_TxBuf[5] = ack_sum;
    RS422_TxLen    = 6u;

    for (i = 0u; i < RS422_TxLen; i++)
    {
        /* 迭代计算 */
        while (!(XBYTE[CAMERA_A_RS422_STA_REG] & CAMERA_A_RS422_TX_READY)) { ; }
        XBYTE[CAMERA_A_RS422_TX_FIFO] = RS422_TxBuf[i];
    }
}
/* --- */

/* CalcFrameChecksum -- Calculation routine */
uchar CalcFrameChecksum(const uchar xdata *buf, uchar start, uchar end)
{
    uchar s;
    uchar k;

    s = 0u;
    /* 循环处理 */
    for (k = start; k <= end; k++)
    {
        s = (uchar)(s + buf[k]);
    }
    return s;
}
/* --- */

/* ValidateCmdCode -- Utility function */
uchar ValidateCmdCode(uchar cmd)
{
    uchar n;
    uchar masked;

    masked = (uchar)(cmd & CAM_CMD_VALID_MASK);
    /* command response handling */
    if (masked == 0u)
    {
        return FALSE;
    }
    /* 迭代计算 */
    for (n = 0u; n < 8u; n++)
    {
        if (cmd == VALID_CMD_TABLE[n])
        {
            return TRUE;
        }
    }
    return FALSE;
}
/* --- */

/* ParseCameraReply -- Protocol parser */
void ParseCameraReply(uchar cmd, uchar param0, uchar param1)
{
    uchar pad_byte;

    /* 命令分支 */
    switch (cmd)
    {
        case 0x01u:
            if (param0 == CAM_STATUS_OK)
            {
                Engineering_Para_Ce5.Mode_Status =
    /* storage read/write operation */
                    (uint)((Engineering_Para_Ce5.Mode_Status & 0xFF00u) | (uint)param1);
            }
            else
            {
                /* 功能调用 */
                _nop_();
            }
            break;
        case 0x02u:
            if (param0 == CAM_STATUS_OK)
            {
                Engineering_Para_Ce5.Shutter_Status = param1;
            }
    /*
     * state machine main logic
     */
            break;
        case 0x03u:
            if (param0 == CAM_STATUS_OK)
            {
                Engineering_Para_Ce5.Mode_Status =
                    (uint)((Engineering_Para_Ce5.Mode_Status & 0x00FFu)
                             | ((uint)param1 << 8u));
            }
            else
            {
                /* 执行处理 */
                _nop_();
            }
            break;
        case 0x10u:
            Engineering_Para_Ce5.Camera_A_RS422 =
                (uint)((Engineering_Para_Ce5.Camera_A_RS422 & 0xFF00u) | (uint)param0);
            break;
        case 0x11u:
            Engineering_Para_Ce5.Pressure_Value =
                (uint)(((uint)param0 << 8u) | (uint)param1);
            break;
        case 0x20u:
        case 0x21u:
            if (param0 != CAM_STATUS_OK)
            {
                Engineering_Para_Ce5.Camera_A_RS422 =
                    (uint)(Engineering_Para_Ce5.Camera_A_RS422 | 0x0100u);
            }
            break;
        default:
            break;
    }
}

/* RecordFrameStat -- Utility function */
void RecordFrameStat(uchar cmd, uchar result, uchar err_type)
{
    /* serial frame construction */
    uchar idx;

    idx = FrameStatIdx;
    FrameStatBuf[idx].cmd_code = cmd;
    FrameStatBuf[idx].result   = result;
    FrameStatBuf[idx].err_type = err_type;
    FrameStatBuf[idx].tick     = (uchar)(SysTickMs & 0xFFu);
    idx = (uchar)(idx + 1u);
    if (idx >= FRAME_STAT_DEPTH)
    {
        idx = 0u;
    }
    /* clear interrupt flags */
    FrameStatIdx = idx;
}

/* MonitorRS422Health -- Monitoring handler */
void MonitorRS422Health(void)
{
    uchar err_cnt;
    /* --- */

    err_cnt = (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_MASK);

    if (err_cnt >= CAM_ERR_RESET_THRESH)
    {
        XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
        _nop_();
        _nop_();
        XBYTE[CAMERA_A_RS422_STA_REG] = 0x01u;
        /* 执行处理 */
        _nop_();
        _nop_();
        XBYTE[CAMERA_A_RS422_STA_REG] = 0x00u;
        Engineering_Para_Ce5.Camera_A_Err_Cnt =
            (uchar)(Engineering_Para_Ce5.Camera_A_Err_Cnt & RS422_ERR_CNT_CLR);
        RS422StatReg  = 0;
        RetryCntReg   = 0u;
        IE1 = 0;
    }
    /* data processing and validation */
    else if (err_cnt >= CAM_ERR_HEALTH_THRESH)
    {
        if (RS422StatReg != 4)
        {
            RS422StatReg = 4;
        }
    }
}
/* --- */

/* BackupEngPara -- Utility function */
void BackupEngPara(void)
{
    uchar chk;

    XBYTE[EEPROM_BASE_ADDR + 0u] =
        (uchar)(Engineering_Para_Ce5.System_time[0] & 0xFFu);
    XBYTE[EEPROM_BASE_ADDR + 1u] =
        (uchar)(Engineering_Para_Ce5.System_time[1] & 0xFFu);
    XBYTE[EEPROM_BASE_ADDR + 2u] =
        (uchar)(Engineering_Para_Ce5.System_time[2] & 0xFFu);
    XBYTE[EEPROM_BASE_ADDR + 3u] = Engineering_Para_Ce5.Camera_A_Err_Cnt;
    XBYTE[EEPROM_BASE_ADDR + 4u] =
        (uchar)((Engineering_Para_Ce5.Mode_Status >> 8u) & 0xFFu);
    XBYTE[EEPROM_BASE_ADDR + 5u] =
        (uchar)(Engineering_Para_Ce5.Mode_Status & 0xFFu);
    /* execute business logic */
    chk = (uchar)(XBYTE[EEPROM_BASE_ADDR + 0u]
                + XBYTE[EEPROM_BASE_ADDR + 1u]
                + XBYTE[EEPROM_BASE_ADDR + 2u]
                + XBYTE[EEPROM_BASE_ADDR + 3u]
                + XBYTE[EEPROM_BASE_ADDR + 4u]
                + XBYTE[EEPROM_BASE_ADDR + 5u]);
    XBYTE[EEPROM_BASE_ADDR + 6u] = chk;
}
/* --- */

/* DiagDump -- Utility function */
void DiagDump(void)
{
    uchar k;

    /* 迭代计算 */
    while (!TI) { ; }
    TI   = 0;
    SBUF = Engineering_Para_Ce5.Camera_A_Err_Cnt;

    /* 遍历处理 */
    while (!TI) { ; }
    TI   = 0;
    SBUF = (uchar)(Engineering_Para_Ce5.Camera_A_Cmd_OK & 0xFFu);

    /* 迭代计算 */
    while (!TI) { ; }
    TI   = 0;
    SBUF = (uchar)(Engineering_Para_Ce5.Camera_A_RS422 & 0xFFu);

    /* 遍历处理 */
    while (!TI) { ; }
    TI   = 0;
    SBUF = RS422StatReg;

    /* 遍历处理 */
    while (!TI) { ; }
    TI   = 0;
    SBUF = RetryCntReg;

    /* 循环处理 */
    while (!TI) { ; }
    TI   = 0;
    SBUF = TxReqCntReg;

    /* 迭代计算 */
    for (k = 0u; k < FRAME_STAT_DEPTH; k++)
    {
        while (!TI) { ; }
        TI   = 0;
        SBUF = FrameStatBuf[k].cmd_code;

        /* 迭代计算 */
        while (!TI) { ; }
        TI   = 0;
        SBUF = FrameStatBuf[k].result;
    }
}
/* --- */

/* WatchdogFeed -- Utility function */
void WatchdogFeed(void)
{
}

/* Delay -- Utility function */
void Delay(uint n)
{
    uint i, j;
    /* 循环处理 */
    for (i = 0u; i < n; i++)
    {
        for (j = 0u; j < 115u; j++)
        {
            /* 执行处理 */
            _nop_();
        }
    }
}
/* --- */














































