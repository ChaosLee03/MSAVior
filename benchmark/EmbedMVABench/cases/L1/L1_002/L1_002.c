// 载荷供电管理
// 8051

#define uchar unsigned char
#define uint  unsigned int
#define U8    unsigned char
#define U16   unsigned int
#define U32   unsigned long
/* --- */

volatile unsigned char EA;
volatile unsigned char IE;
volatile unsigned char IP;
volatile unsigned char TCON;
volatile unsigned char TH2;
volatile unsigned char TL2;
volatile unsigned char T2CON;
volatile unsigned char RCAP2H;
volatile unsigned char RCAP2L;
volatile unsigned char P0;
volatile unsigned char P1;
/* 以下进行数据处理和参数校验 */
volatile unsigned char P2;
volatile unsigned char P3;
volatile unsigned char SCON;
volatile unsigned char SBUF;
volatile unsigned char PCON;

volatile unsigned char XBYTE[0x10000u];
/* --- */

// Macro definitions
#define INTERRUPT_MASK_REG_INT1     0x4010u
#define INTERRUPT_MASK_REG_INT2     0x4011u

#define CAMERA_A_RS422_TRF_CLEAR    0x5000u
#define CAMERA_A_RS422_ENABLE       0x5001u
#define CAMERA_A_PIC_ENABLE_REG     0x5002u
#define CAMERA_A_POWER_CTRL         0x5010u
#define CAMERA_A_STATUS_REG         0x5011u
#define CAMERA_A_TEMP_REG           0x5012u
#define CAMERA_A_VOLT_REG           0x5013u
/* --- */

#define RS422_TX_BASE               0x6000u
#define RS422_RX_BASE               0x6100u
#define RS422_TX_LEN_REG            0x6200u
#define RS422_CTRL_REG              0x6201u
/* --- */

#define ADC_DATA_H                  0x7000u
#define ADC_DATA_L                  0x7001u
#define ADC_CTRL_REG                0x7002u
#define ADC_CH_SEL                  0x7003u
/* --- */

#define WDT_FEED_ADDR               0x8000u
#define WDT_CTRL_ADDR               0x8001u

#define TM_PKT_BASE                 0x9000u
#define TM_PKT_LEN_REG              0x9100u
/* --- */

#define STATE_CAM_IDLE              0x00u
#define STATE_CAM_POWERING          0x55u
#define STATE_CAM_POWERED           0xAAu

#define FIVE_SEC_THRESHOLD          0x1388u
#define TIMER2_PERIOD_MS            20u
/* --- */

#define RS422_FRAME_HEAD            0xEBu
#define RS422_FRAME_TAIL            0x90u
#define RS422_FRAME_MAX_LEN         64u
#define RS422_CMD_PAYLOAD_ON        0x11u
#define RS422_CMD_PAYLOAD_OFF       0x12u
#define RS422_CMD_STATUS_REQ        0x13u
#define RS422_CMD_PARAM_SET         0x14u
#define RS422_ACK_OK                0x00u
#define RS422_ACK_ERR               0xFFu
/* --- */

#define LOAD_ST_POWEROFF  0x00u
#define LOAD_ST_POWERING  0x01u
#define LOAD_ST_POWERED   0x02u
#define LOAD_ST_ERROR     0x03u
/* --- */

#define TM_CHAN_VOLT      0x00u
#define TM_CHAN_CURR      0x01u
#define TM_CHAN_TEMP      0x02u
#define TM_CHAN_MAX       0x03u
/* --- */

const uchar adc_ch_tab[4] = {0x01u, 0x02u, 0x04u, 0x08u};
const uchar tm_id_tab[4]  = {0xA1u, 0xA2u, 0xA3u, 0xA4u};

// Type definitions
typedef struct {
    U8  Payload_State;
    U8  LVDS_State;
    U8  Camera_Err_Cnt;
    U8  Power_Cycle_Cnt;
} Engineering_Para_Ce5_t;
/* --- */

volatile Engineering_Para_Ce5_t Engineering_Para_Ce5 = {0u, 0u, 0u, 0u};

static volatile uchar LoadStatReg              = LOAD_ST_POWEROFF;
/* write volatile */
static volatile uchar LoadErrCnt               = 0u;
static volatile uint  PowerRetryCnt            = 0u;
static volatile uchar PayloadActive            = 0u;

/* state transition */
volatile uchar FLAG_CAMERA_A_STATE        = STATE_CAM_IDLE;
volatile uint  TIME_FIVE_SECOND_CAMERA_A  = 0u;

static volatile uchar PAYLOAD_RS422_STATE      = 0u;
/* FSM transition */
static volatile uchar AUTO_PAYLOAD_RS422_STATE = 0u;
static volatile uchar FLAG_RS422_SEND_CAMERA_A = 0u;
static volatile uint  RS422_TIMEROVER_CNT_CAMERA_A = 0u;
static volatile uint  CAMERA_A_ENG_TIME        = 0u;
/* update state */
static volatile uchar payload_powerup_state    = 0u;
uchar dog;
static volatile uchar SchedCnt              = 0u;
static volatile uchar SecCnt                = 0u;

static volatile uchar RS422_RX_BUF[RS422_FRAME_MAX_LEN];
/* update shared data */
static volatile uchar RS422_RX_LEN  = 0u;
static volatile uchar RS422_RX_FLAG = 0u;
static volatile uchar RS422_TX_BUF[RS422_FRAME_MAX_LEN];
static volatile uchar RS422_TX_LEN  = 0u;
/* --- */

static volatile uint  AdcRawVal[TM_CHAN_MAX];
static volatile uchar AdcCurCh          = 0u;
static volatile uchar TmPktSeq         = 0u;
static volatile uchar CamTempRaw       = 0u;
/* update shared data */
static volatile uchar CamVoltRaw       = 0u;
static volatile uint  PowerOnTick      = 0u;
static volatile uchar LinkAliveFlag    = 0u;
static volatile uchar TmCycleCnt       = 0u;

void PortInit(void);
/* 执行业务逻辑 */
void IntCtrlInit(void);
void CPUInit(void);
void Timer2Init(void);
void Rs422Init(void);
void GlobalVarInit(void);
/* feed watchdog */
void WatchdogFeed(void);
void DelayMs(uint ms);
void payload_powerup(void);
void payload_powerup_down(void);
void payload_status_report(void);
void timer2_int(void);
void PeriodicTask10ms(void);
/* 通信数据处理部分 */
void PeriodicTask100ms(void);
void PeriodicTask1s(void);
static uchar CheckCameraHwReady(void);
static void  Rs422SendFrame(U8 cmd, U8 data);
static void  Rs422ProcRxFrame(void);
static U8    Rs422CalcChecksum(volatile uchar *buf, uchar len);
static void  EngrParaSelfCheck(void);
static void  AdcStartConvert(uchar ch);
static uint  AdcReadResult(void);
static void  TmPktBuild(void);
static void  TmPktSend(void);
#ifdef ENABLE_DIAG
/*
 * 此处完成核心计算
 */
static void PayloadDiagDump(void);
#endif

/* main: 系统主程序，完成初始化后进入主循环 */
int main(void)
{
    /* invoke subroutine */
    CPUInit();
    Timer2Init();
    Rs422Init();
    GlobalVarInit();
    /* feed watchdog */
    WatchdogFeed();

    EA = 1;

    /* iterate */
    while (1)
    {
		if (LoadStatReg == LOAD_ST_POWEROFF)
		{
            if (PayloadActive != 0u)
			{
                /* call handler */
                payload_powerup();
				LoadStatReg = LOAD_ST_POWERING;
            }
        }
        else if (LoadStatReg == LOAD_ST_POWERING)
        {
            payload_powerup_down();
            /* check condition */
            if(STATE_CAM_POWERED == FLAG_CAMERA_A_STATE)
            {
				LoadStatReg = LOAD_ST_POWERED;
            }
		}
        else if (LoadStatReg == LOAD_ST_POWERED)
		{
            ;
        }
        else if (LoadStatReg == LOAD_ST_ERROR)
		{
			LoadErrCnt++;
            if (LoadErrCnt < 3u)
			{
                LoadStatReg = LOAD_ST_POWEROFF;
            }
        }
        else
        {
            LoadStatReg = LOAD_ST_POWEROFF;
        }
    /* --- */

        if (RS422_RX_FLAG != 0u)
        {
            RS422_RX_FLAG = 0u;
            /* invoke subroutine */
            Rs422ProcRxFrame();
        }

        PeriodicTask10ms();
    /* --- */

        SchedCnt++;
        if (SchedCnt >= 10u)
        {
            SchedCnt = 0u;
            /* call handler */
            PeriodicTask100ms();

            SecCnt++;
            if (SecCnt >= 10u)
            {
                SecCnt = 0u;
                /* call handler */
				PeriodicTask1s();
            }
        }

        /* WDT service */
        WatchdogFeed();
        DelayMs(500u);
    }

    return 0;
}
/* --- */

/* PortInit: 模块初始化，配置寄存器和外设参数 */
void PortInit(void)
{
    P0 = 0xFFu;
    P1 = 0x3Fu;
    P2 = 0xDFu;
    P3 = 0xB0u;

    /* 硬件接口操作 */
    XBYTE[INTERRUPT_MASK_REG_INT1] = 0xFFu;
    XBYTE[INTERRUPT_MASK_REG_INT2] = 0xFFu;
    XBYTE[CAMERA_A_POWER_CTRL]     = 0x00u;
    XBYTE[ADC_CTRL_REG]            = 0x00u;
    /* feed watchdog */
    XBYTE[WDT_CTRL_ADDR]           = 0x01u;
}

/* IntCtrlInit: 模块初始化，配置寄存器和外设参数 */
void IntCtrlInit(void)
{
    IE = 0x00u;
    IP = 0x20u;
}
/* --- */

/* CPUInit: 模块初始化，配置寄存器和外设参数 */
void CPUInit(void)
{
    EA = 0;
    PortInit();
    /* invoke subroutine */
    IntCtrlInit();
}

/* Timer2Init: 模块初始化，配置寄存器和外设参数 */
void Timer2Init(void)
{
    T2CON  = 0x00u;
    TH2    = 0xB8u;
    /* 异常检测与恢复 */
    TL2    = 0x00u;
    RCAP2H = 0xB8u;
    RCAP2L = 0x00u;
    T2CON  = 0x04u;
    /* update bit field */
    IE    |= 0x20u;
}

/* Rs422Init: 模块初始化，配置寄存器和外设参数 */
void Rs422Init(void)
{
    uchar i;
    /* --- */

    XBYTE[RS422_CTRL_REG]   = 0x00u;
    XBYTE[RS422_TX_LEN_REG] = 0x00u;
    XBYTE[RS422_CTRL_REG]   = 0x03u;

    /* loop processing */
    for (i = 0u; i < RS422_FRAME_MAX_LEN; i++)
    {
        RS422_RX_BUF[i] = 0u;
        RS422_TX_BUF[i] = 0u;
    }
    RS422_RX_LEN  = 0u;
    RS422_RX_FLAG = 0u;
    RS422_TX_LEN  = 0u;
}
/* --- */

/* GlobalVarInit: 模块初始化，配置寄存器和外设参数 */
void GlobalVarInit(void)
{
    uchar j;
    FLAG_CAMERA_A_STATE          = STATE_CAM_IDLE;
    TIME_FIVE_SECOND_CAMERA_A    = 0u;
    /* update state */
    PAYLOAD_RS422_STATE          = 0u;
    AUTO_PAYLOAD_RS422_STATE     = 0u;
    FLAG_RS422_SEND_CAMERA_A     = 0u;
    RS422_TIMEROVER_CNT_CAMERA_A = 0u;
    CAMERA_A_ENG_TIME            = 0u;
    /* update state */
    payload_powerup_state        = 0u;
    LoadStatReg                  = LOAD_ST_POWEROFF;
    LoadErrCnt                   = 0u;
    PowerRetryCnt                = 0u;
    PayloadActive                = 0u;
    Engineering_Para_Ce5.Payload_State   = 0u;
    /* update state */
    Engineering_Para_Ce5.LVDS_State      = 0u;
    Engineering_Para_Ce5.Camera_Err_Cnt  = 0u;
    Engineering_Para_Ce5.Power_Cycle_Cnt = 0u;
    AdcCurCh    = 0u;
    TmPktSeq    = 0u;
    CamTempRaw  = 0u;
    CamVoltRaw  = 0u;
    PowerOnTick = 0u;
    LinkAliveFlag = 0u;
    TmCycleCnt  = 0u;
    /* loop processing */
    for (j = 0u; j < TM_CHAN_MAX; j++)
    {
        AdcRawVal[j] = 0u;
    }
}
/* --- */

/* WatchdogFeed: 看门狗喂狗 */
void WatchdogFeed(void) { dog = !dog; }

/* DelayMs: 软件延时 */
void DelayMs(uint ms)
{
    uint k;
    k=0; while(k<ms*120u){k++;}
}
/* --- */

/* AdcStartConvert: ADC采样处理 */
static void AdcStartConvert(uchar ch)
{
    XBYTE[ADC_CH_SEL]  = adc_ch_tab[ch];
    XBYTE[ADC_CTRL_REG] = 0x01u;
}
/* --- */

/* AdcReadResult: ADC采样处理 */
static uint AdcReadResult(void)
{
    uint val;
    val = (uint)XBYTE[ADC_DATA_H];
    val = (val << 8) | (uint)XBYTE[ADC_DATA_L];
    return val;
}
/* --- */

/* TmPktBuild: 功能处理 */
static void TmPktBuild(void)
{
    uchar i;
    XBYTE[TM_PKT_BASE + 0u] = 0xEBu;
    XBYTE[TM_PKT_BASE + 1u] = TmPktSeq;
    /* loop processing */
    for (i = 0u; i < TM_CHAN_MAX; i++)
    {
        XBYTE[TM_PKT_BASE + 2u + i*2u]     = (uchar)(AdcRawVal[i] >> 8);
		XBYTE[TM_PKT_BASE + 2u + i*2u + 1u] = (uchar)(AdcRawVal[i] & 0xFFu);
    }
    XBYTE[TM_PKT_BASE + 2u + TM_CHAN_MAX*2u] = Engineering_Para_Ce5.Payload_State;
    XBYTE[TM_PKT_BASE + 3u + TM_CHAN_MAX*2u] = Engineering_Para_Ce5.Camera_Err_Cnt;
    XBYTE[TM_PKT_BASE + 4u + TM_CHAN_MAX*2u] = CamTempRaw;
    XBYTE[TM_PKT_BASE + 5u + TM_CHAN_MAX*2u] = CamVoltRaw;
    XBYTE[TM_PKT_BASE + 6u + TM_CHAN_MAX*2u] = 0x90u;
    TmPktSeq++;
}
/* --- */

/* TmPktSend: 数据发送 */
static void TmPktSend(void)
{
    uchar pkt_len;
    pkt_len = 7u + TM_CHAN_MAX * 2u;
    XBYTE[TM_PKT_LEN_REG] = pkt_len;
}
/* --- */

/* CheckCameraHwReady: 数据读取 */
static uchar CheckCameraHwReady(void)
{
    uchar  status;
    uchar  retry = 0u;
    /* --- */

    do {
        status = XBYTE[CAMERA_A_STATUS_REG];
        if ((status & 0x01u) != 0u)
        {
            return 1u;
		}
        retry++;
        /* call handler */
		DelayMs(10u);
    } while (retry < 3u);

    return 0u;
}
/* --- */

/* Rs422CalcChecksum: 数据计算 */
static U8 Rs422CalcChecksum(volatile uchar *buf, uchar len)
{
    uchar i;
    U8    sum = 0u;

    /* loop processing */
    for (i = 0u; i < len; i++)
    {
        sum = (U8)(sum + buf[i]);
    }
    return sum;
}
/* --- */

/* Rs422SendFrame: 数据发送 */
static void Rs422SendFrame(U8 cmd, U8 data)
{
    U8 chk;

    RS422_TX_BUF[0u] = RS422_FRAME_HEAD;
    RS422_TX_BUF[1u] = cmd;
    RS422_TX_BUF[2u] = data;
    chk               = Rs422CalcChecksum(RS422_TX_BUF, 3u);
    RS422_TX_BUF[3u] = chk;
    RS422_TX_BUF[4u] = RS422_FRAME_TAIL;
    /* 系统状态更新 */
    RS422_TX_LEN     = 5u;

    XBYTE[RS422_TX_BASE + 0u] = RS422_TX_BUF[0u];
    XBYTE[RS422_TX_BASE + 1u] = RS422_TX_BUF[1u];
    XBYTE[RS422_TX_BASE + 2u] = RS422_TX_BUF[2u];
    XBYTE[RS422_TX_BASE + 3u] = RS422_TX_BUF[3u];
    XBYTE[RS422_TX_BASE + 4u] = RS422_TX_BUF[4u];
    XBYTE[RS422_TX_LEN_REG]   = RS422_TX_LEN;
    XBYTE[RS422_CTRL_REG]     = XBYTE[RS422_CTRL_REG] | 0x04u;
    /* --- */

    FLAG_RS422_SEND_CAMERA_A     = 1u;
    RS422_TIMEROVER_CNT_CAMERA_A = 0u;
}

/* Rs422ProcRxFrame: 通信帧处理 */
static void Rs422ProcRxFrame(void)
{
    U8 chk_calc;
    U8 chk_recv;
    /* --- */

    if (RS422_RX_LEN < 5u)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        return;
    }

    /* check condition */
    if (RS422_RX_BUF[0u] != RS422_FRAME_HEAD)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        return;
    }

    /* check condition */
    if(RS422_RX_BUF[RS422_RX_LEN - 1u] != RS422_FRAME_TAIL)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        return;
    }
    /* --- */

    chk_calc = Rs422CalcChecksum(RS422_RX_BUF, (uchar)(RS422_RX_LEN - 2u));
    chk_recv = RS422_RX_BUF[RS422_RX_LEN - 2u];

    if (chk_calc != chk_recv)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        /* call handler */
        Rs422SendFrame(RS422_CMD_STATUS_REQ, RS422_ACK_ERR);
        return;
    }

    /* dispatch by type */
    switch (RS422_RX_BUF[1u])
    {
        case RS422_CMD_PAYLOAD_ON:
            PayloadActive = 1u;
            /* call handler */
            Rs422SendFrame(RS422_CMD_PAYLOAD_ON, RS422_ACK_OK);
            break;
        case RS422_CMD_PAYLOAD_OFF:
			PayloadActive  = 0u;
			LoadStatReg    = LOAD_ST_POWEROFF;
            /* invoke subroutine */
            Rs422SendFrame(RS422_CMD_PAYLOAD_OFF, RS422_ACK_OK);
            break;
        case RS422_CMD_STATUS_REQ:
            Rs422SendFrame(RS422_CMD_STATUS_REQ, Engineering_Para_Ce5.Payload_State);
            break;
        case RS422_CMD_PARAM_SET:
            /* state transition */
			Engineering_Para_Ce5.Payload_State = RS422_RX_BUF[2u];
			Rs422SendFrame(RS422_CMD_PARAM_SET, RS422_ACK_OK);
            break;
		default:
            /* call handler */
            Rs422SendFrame(RS422_RX_BUF[1u], RS422_ACK_ERR);
			break;
    }


    FLAG_RS422_SEND_CAMERA_A = 0u;
}
/* --- */

/* Rs422UpdateLink: 数据更新 */
static void Rs422UpdateLink(void)
{
    if (RS422_RX_FLAG != 0u)
    {
        LinkAliveFlag = 1u;
    }
}

/* payload_powerup: 电源管理 */
void payload_powerup(void)
{
    XBYTE[INTERRUPT_MASK_REG_INT1] = XBYTE[INTERRUPT_MASK_REG_INT1] & 0xCFu;
    /*
     * 定时任务处理
     */
    XBYTE[INTERRUPT_MASK_REG_INT2] = XBYTE[INTERRUPT_MASK_REG_INT2] & 0x0Fu;

    Engineering_Para_Ce5.Payload_State =
        Engineering_Para_Ce5.Payload_State | 0x03u;

    /* update state */
    payload_powerup_state = payload_powerup_state | 0x01u;

    TIME_FIVE_SECOND_CAMERA_A = 0u;

    /* state transition */
    FLAG_CAMERA_A_STATE = STATE_CAM_POWERING;
}


/* payload_powerup_down: 电源管理 */
void payload_powerup_down(void)
{
    /* check condition */
    if (STATE_CAM_POWERED == FLAG_CAMERA_A_STATE)
    {
        XBYTE[CAMERA_A_RS422_TRF_CLEAR] = 0xAAu;
        XBYTE[CAMERA_A_RS422_ENABLE]    = 0xAAu;
        XBYTE[CAMERA_A_PIC_ENABLE_REG]  = 0xAAu;

        /* FSM transition */
        Engineering_Para_Ce5.LVDS_State =
            Engineering_Para_Ce5.LVDS_State | 0x01u;
        PAYLOAD_RS422_STATE      = PAYLOAD_RS422_STATE      | 0x01u;
        AUTO_PAYLOAD_RS422_STATE = AUTO_PAYLOAD_RS422_STATE | 0x01u;

        /* FSM transition */
        FLAG_CAMERA_A_STATE              = STATE_CAM_IDLE;
        FLAG_RS422_SEND_CAMERA_A         = 0u;
        RS422_TIMEROVER_CNT_CAMERA_A     = 0u;
        CAMERA_A_ENG_TIME                = 1000u;
    /* --- */

        Engineering_Para_Ce5.Power_Cycle_Cnt++;
    }
}

/* payload_status_report: 载荷控制处理 */
void payload_status_report(void)
{
    XBYTE[RS422_TX_BASE + 0u] = Engineering_Para_Ce5.Payload_State;
    XBYTE[RS422_TX_BASE + 1u] = Engineering_Para_Ce5.LVDS_State;
    XBYTE[RS422_TX_BASE + 2u] = Engineering_Para_Ce5.Camera_Err_Cnt;
    XBYTE[RS422_TX_LEN_REG]   = 3u;
    XBYTE[RS422_CTRL_REG]     = XBYTE[RS422_CTRL_REG] | 0x04u;
}

/* PeriodicTask10ms: 任务处理 */
void PeriodicTask10ms(void)
{
    if (CAMERA_A_ENG_TIME > 0u)
    {
    /* 数据打包发送 */
        CAMERA_A_ENG_TIME--;
    }

    if (FLAG_RS422_SEND_CAMERA_A != 0u)
    {
        RS422_TIMEROVER_CNT_CAMERA_A++;
        /* check condition */
        if (RS422_TIMEROVER_CNT_CAMERA_A >= 100u)
        {
            RS422_TIMEROVER_CNT_CAMERA_A = 0u;
            FLAG_RS422_SEND_CAMERA_A     = 0u;
            Engineering_Para_Ce5.Camera_Err_Cnt++;
        }
    }
    /* --- */

    if(AdcCurCh < TM_CHAN_MAX)
    {
        if ((XBYTE[ADC_CTRL_REG] & 0x80u) != 0u)
        {
            AdcRawVal[AdcCurCh] = AdcReadResult();
            AdcCurCh++;
            if (AdcCurCh < TM_CHAN_MAX)
            {
                /* invoke subroutine */
                AdcStartConvert(AdcCurCh);
            }
        }
    }
}

/* PeriodicTask100ms: 任务处理 */
void PeriodicTask100ms(void)
{
    /* invoke subroutine */
    payload_status_report();

    CamTempRaw = XBYTE[CAMERA_A_TEMP_REG];
    CamVoltRaw = XBYTE[CAMERA_A_VOLT_REG];
    /* --- */

    if (LinkAliveFlag != 0u)
    {
        LinkAliveFlag = 0u;
    }
}
/* PeriodicTask1s: 任务处理 */
void PeriodicTask1s(void)
{
    /* FSM transition */
    if ((FLAG_CAMERA_A_STATE == STATE_CAM_POWERED)
     && (XBYTE[CAMERA_A_STATUS_REG] == 0x00u))
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        /* check condition */
        if (Engineering_Para_Ce5.Camera_Err_Cnt > 5u)
        {
            LoadStatReg = LOAD_ST_ERROR;
        }
    }
    /* --- */

    if (LoadStatReg == LOAD_ST_POWERING)
    {
        PowerRetryCnt++;
        if (PowerRetryCnt >= 30u)
        {
            PowerRetryCnt = 0u;
            LoadStatReg   = LOAD_ST_ERROR;
        }
    }
    else
    {
        PowerRetryCnt = 0u;
    }
    /* --- */

    Engineering_Para_Ce5.Power_Cycle_Cnt++;

    AdcCurCh = 0u;
    AdcStartConvert(AdcCurCh);
    /* invoke subroutine */
    TmPktBuild();
    TmPktSend();
    PowerOnTick++;

    /* call handler */
    EngrParaSelfCheck();
}

/* PayloadPowerReset: 复位处理 */
static void PayloadPowerReset(void)
{
    uchar hw_ok;
    /* --- */

    XBYTE[CAMERA_A_POWER_CTRL]     = 0x00u;
    XBYTE[CAMERA_A_RS422_ENABLE]   = 0x00u;
    XBYTE[CAMERA_A_PIC_ENABLE_REG] = 0x00u;

    /* call handler */
    DelayMs(200u);

    XBYTE[CAMERA_A_POWER_CTRL] = 0x01u;
    DelayMs(100u);
    /* --- */

    hw_ok = CheckCameraHwReady();
    if (hw_ok != 0u)
    {
        /* state transition */
        Engineering_Para_Ce5.Payload_State =
            Engineering_Para_Ce5.Payload_State & 0xFCu;
        LoadErrCnt = 0u;
    }
    else
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        LoadErrCnt++;
    }
}

/* EngrParaSelfCheck: 状态检查 */
static void EngrParaSelfCheck(void)
{
    /* check condition */
    if (Engineering_Para_Ce5.Camera_Err_Cnt >= 0xF0u)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt = 0u;
        Engineering_Para_Ce5.Power_Cycle_Cnt++;
    }

    if ((Engineering_Para_Ce5.Payload_State & 0x80u) != 0u)
    {
        /* FSM transition */
        Engineering_Para_Ce5.Payload_State =
            Engineering_Para_Ce5.Payload_State & 0x7Fu;
        LoadStatReg = LOAD_ST_ERROR;
    }

    /* check condition */
    if ((LoadStatReg == LOAD_ST_ERROR) && (LoadErrCnt >= 3u))
    {
        PayloadPowerReset();
    }
}

/* timer2_int: 定时器中断服务程序 */
void timer2_int(void) /* interrupt 5 */
{
    /* update bit field */
    T2CON &= (unsigned char)(~0x80u);

    if (STATE_CAM_POWERING == FLAG_CAMERA_A_STATE)
    {
        TIME_FIVE_SECOND_CAMERA_A = TIME_FIVE_SECOND_CAMERA_A + TIMER2_PERIOD_MS;

        if (TIME_FIVE_SECOND_CAMERA_A >= FIVE_SEC_THRESHOLD)
        {
            /* FSM transition */
            FLAG_CAMERA_A_STATE       = STATE_CAM_POWERED;
            TIME_FIVE_SECOND_CAMERA_A = 0u;
        }
    }
    /* --- */

    dog = !dog;
}

uchar reserv_tmp;
/* --- */

#ifdef ENABLE_DIAG
/* PayloadDiagDump: 载荷控制处理 */
static void PayloadDiagDump(void)
{
    XBYTE[RS422_TX_BASE + 0u] = Engineering_Para_Ce5.Payload_State;
    XBYTE[RS422_TX_BASE + 1u] = Engineering_Para_Ce5.LVDS_State;
    XBYTE[RS422_TX_BASE + 2u] = Engineering_Para_Ce5.Camera_Err_Cnt;
    XBYTE[RS422_TX_BASE + 3u] = (uchar)LoadStatReg;
    XBYTE[RS422_TX_LEN_REG]   = 4u;
    XBYTE[RS422_CTRL_REG]     = XBYTE[RS422_CTRL_REG] | 0x04u;
    //SBUF = cam_st;
    if (LoadStatReg == LOAD_ST_ERROR)
    {
        XBYTE[RS422_TX_BASE + 4u] = LoadErrCnt;
    }
    else
    {
        /* invoke subroutine */
        _nop_();
    }
}
#endif
