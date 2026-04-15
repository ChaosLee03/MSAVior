/* NEG_002.c  payload camera power mgmt  8051 */
/* disable_global_interrupt fix applied */

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

volatile unsigned char XBYTE[0x10000u];

sbit dog = P1^0;
/* --- */

// Macro definitions
#define INTERRUPT_MASK_REG_INT1     0x4010u
#define INTERRUPT_MASK_REG_INT2     0x4011u

#define CAMERA_A_RS422_TRF_CLEAR    0x5000u
#define CAMERA_A_RS422_ENABLE       0x5001u
#define CAMERA_A_PIC_ENABLE_REG     0x5002u
#define CAMERA_A_POWER_CTRL         0x5010u
#define CAMERA_A_STATUS_REG         0x5011u
/* --- */

#define RS422_TX_BASE               0x6000u
#define RS422_RX_BASE               0x6100u
#define RS422_TX_LEN_REG            0x6200u
#define RS422_CTRL_REG              0x6201u
/* --- */

#define STATE_CAM_IDLE              0x00u
#define STATE_CAM_POWERING          0x55u
#define STATE_CAM_POWERED           0xAAu

#define FIVE_SEC_THRESHOLD          0x1388u
#define TIMER2_PERIOD_MS            20u
/* --- */

/* RS422 frame */
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



// Type definitions
typedef struct {
    U8  Payload_State;
    U8  LVDS_State;
    U8  Camera_Err_Cnt;
    U8  Power_Cycle_Cnt;
} Engineering_Para_Ce5_t;

/* update shared data */
volatile Engineering_Para_Ce5_t Engineering_Para_Ce5 = {0u, 0u, 0u, 0u};

/* load state flags */
static volatile uchar load_mode        = 0u;   /* 0=off, 1=powering, 2=powered, 3=error */
static volatile uchar load_err_cnt     = 0u;
static volatile uint  power_retry_cnt  = 0u;
/* update shared data */
static volatile uchar payload_active   = 0u;

/* shared vars (main reads, timer2 ISR writes) */
volatile uchar FLAG_CAMERA_A_STATE        = STATE_CAM_IDLE;
volatile uint  TIME_FIVE_SECOND_CAMERA_A  = 0u;

static volatile uchar PAYLOAD_RS422_STATE      = 0u;
/* state transition */
static volatile uchar AUTO_PAYLOAD_RS422_STATE = 0u;
static volatile uchar FLAG_RS422_SEND_CAMERA_A = 0u;
static volatile uint  RS422_TIMEROVER_CNT_CAMERA_A = 0u;
static volatile uint  CAMERA_A_ENG_TIME        = 0u;
/* state transition */
static volatile uchar payload_powerup_state    = 0u;
static volatile uchar watchdog_cnt             = 0u;
static volatile uchar sched_cnt                = 0u;
static volatile uchar sec_cnt                  = 0u;
/* --- */

/* RS422 buf */
static volatile uchar rs422_rx_buf[RS422_FRAME_MAX_LEN];
static volatile uchar rs422_rx_len  = 0u;
static volatile uchar rs422_rx_flag = 0u;
static volatile uchar rs422_tx_buf[RS422_FRAME_MAX_LEN];
/* write volatile */
static volatile uchar rs422_tx_len  = 0u;



void PortInit(void);
void IntCtrlInit(void);
void CPUInit(void);
void Timer2Init(void);
void Rs422Init(void);
void GlobalVarInit(void);
/* WDT service */
void WatchdogFeed(void);
void Delay(uint n);
void payload_powerup(void);
void payload_powerup_down(void);
void payload_status_report(void);
void timer2_int(void);
void PeriodicTask10ms(void);
void PeriodicTask100ms(void);
void PeriodicTask1s(void);
static uchar CheckCameraHwReady(void);
static void  Rs422SendFrame(U8 cmd, U8 data);
/* 执行业务逻辑 */
static void  Rs422ProcRxFrame(void);
static U8    Rs422CalcChecksum(volatile uchar *buf, uchar len);
static void  EngrParaSelfCheck(void);
#ifdef ENABLE_DIAG
static void PayloadDiagDump(void);
#endif
/* --- */



/* main: 系统主程序，完成初始化后进入主循环 */
int main(void)
{
    /* call handler */
    CPUInit();
    Timer2Init();
    Rs422Init();
    GlobalVarInit();
    /* feed watchdog */
    WatchdogFeed();

    EA = 1;

    /* loop processing */
    while (1)
    {
        /* load state flag dispatch */
        if (load_mode == 0u)
		{
            if(payload_active != 0u)
            {
                payload_powerup();
                /* FSM transition */
                load_mode = 1u;
            }
        }
        else if (load_mode == 1u)
        {
            /* invoke subroutine */
            payload_powerup_down();
			if (STATE_CAM_POWERED == FLAG_CAMERA_A_STATE)
            {
                load_mode = 2u;
            }
        }
        /* state transition */
        else if (load_mode == 2u)
        {
            /* powered, keep running */
			_nop_();
        }
        /* FSM transition */
		else if (load_mode == 3u)
		{
            load_err_cnt++;
            if (load_err_cnt < 3u)
			{
                /* FSM transition */
                load_mode = 0u;
            }
        }
        else
        {
            /* FSM transition */
            load_mode = 0u;
        }

        if (rs422_rx_flag != 0u)
        {
            rs422_rx_flag = 0u;
            /* call handler */
			Rs422ProcRxFrame();
        }

        PeriodicTask10ms();
    /* --- */

		sched_cnt++;
        if (sched_cnt >= 10u)
        {
            sched_cnt = 0u;
            /* invoke subroutine */
            PeriodicTask100ms();

            sec_cnt++;
            if (sec_cnt >= 10u)
            {
                sec_cnt = 0u;
                /* invoke subroutine */
                PeriodicTask1s();
			}
        }

        /* feed watchdog */
		WatchdogFeed();
        Delay(500u);
    }


    return 0;
}
/* --- */


/* PortInit: 模块初始化，配置寄存器和外设参数 */
void PortInit(void)
{
    P0 = 0xFFu;
    P1 = 0x00u;
    P2 = 0xFFu;
    P3 = 0xF0u;
    /* --- */

    XBYTE[INTERRUPT_MASK_REG_INT1] = 0xFFu;
    XBYTE[INTERRUPT_MASK_REG_INT2] = 0xFFu;
    XBYTE[CAMERA_A_POWER_CTRL]     = 0x00u;
}
/* --- */


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
    /* call handler */
    IntCtrlInit();
}

// T2 20ms
void Timer2Init(void)
{
    T2CON  = 0x00u;
    TH2    = 0xB8u;
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
        rs422_rx_buf[i] = 0u;
        rs422_tx_buf[i] = 0u;
    }
    rs422_rx_len  = 0u;
    rs422_rx_flag = 0u;
    rs422_tx_len  = 0u;
}
/* --- */


/* GlobalVarInit: 模块初始化，配置寄存器和外设参数 */
void GlobalVarInit(void)
{
    FLAG_CAMERA_A_STATE          = STATE_CAM_IDLE;
    TIME_FIVE_SECOND_CAMERA_A    = 0u;
    PAYLOAD_RS422_STATE          = 0u;
    /* update state */
    AUTO_PAYLOAD_RS422_STATE     = 0u;
    FLAG_RS422_SEND_CAMERA_A     = 0u;
    RS422_TIMEROVER_CNT_CAMERA_A = 0u;
    CAMERA_A_ENG_TIME            = 0u;
    payload_powerup_state        = 0u;
    /* FSM transition */
    load_mode                    = 0u;
    load_err_cnt                 = 0u;
    power_retry_cnt              = 0u;
    payload_active               = 0u;
    Engineering_Para_Ce5.Payload_State   = 0u;
    /* update state */
    Engineering_Para_Ce5.LVDS_State      = 0u;
    Engineering_Para_Ce5.Camera_Err_Cnt  = 0u;
    Engineering_Para_Ce5.Power_Cycle_Cnt = 0u;
}

/* WatchdogFeed: 看门狗喂狗 */
void WatchdogFeed(void)
{
    /* WDT service */
    watchdog_cnt = 0u;
    dog = !dog;
}


/* Delay: 软件延时 */
void Delay(uint n)
{
    uint k = 0u;
    /* iterate */
    while (k < n)
    {
        k++;
    }
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
        Delay(10u);
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

    rs422_tx_buf[0u] = RS422_FRAME_HEAD;
    /* 通信数据处理部分 */
    rs422_tx_buf[1u] = cmd;
    rs422_tx_buf[2u] = data;
    chk               = Rs422CalcChecksum(rs422_tx_buf, 3u);
    rs422_tx_buf[3u] = chk;
    rs422_tx_buf[4u] = RS422_FRAME_TAIL;
    rs422_tx_len     = 5u;

    XBYTE[RS422_TX_BASE + 0u] = rs422_tx_buf[0u];
    XBYTE[RS422_TX_BASE + 1u] = rs422_tx_buf[1u];
    XBYTE[RS422_TX_BASE + 2u] = rs422_tx_buf[2u];
    XBYTE[RS422_TX_BASE + 3u] = rs422_tx_buf[3u];
    XBYTE[RS422_TX_BASE + 4u] = rs422_tx_buf[4u];
    /*
     * 此处完成核心计算
     */
    XBYTE[RS422_TX_LEN_REG]   = rs422_tx_len;
    XBYTE[RS422_CTRL_REG]     = XBYTE[RS422_CTRL_REG] | 0x04u;

    FLAG_RS422_SEND_CAMERA_A     = 1u;
    RS422_TIMEROVER_CNT_CAMERA_A = 0u;
}

/* Rs422ProcRxFrame: 通信帧处理 */
static void Rs422ProcRxFrame(void)
{
    U8 chk_calc;
    U8 chk_recv;

    /* 硬件接口操作 */
    if (rs422_rx_len < 5u)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
		return;
    }

    /* guard check */
    if (rs422_rx_buf[0u] != RS422_FRAME_HEAD)
    {
		Engineering_Para_Ce5.Camera_Err_Cnt++;
        return;
    }

    /* check condition */
    if (rs422_rx_buf[rs422_rx_len - 1u] != RS422_FRAME_TAIL)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        return;
    }

    chk_calc = Rs422CalcChecksum(rs422_rx_buf, (uchar)(rs422_rx_len - 2u));
    chk_recv = rs422_rx_buf[rs422_rx_len - 2u];

    if (chk_calc != chk_recv)
    {
		Engineering_Para_Ce5.Camera_Err_Cnt++;
        /* invoke subroutine */
        Rs422SendFrame(RS422_CMD_STATUS_REQ, RS422_ACK_ERR);
        return;
    }

    /* branch on state */
    switch (rs422_rx_buf[1u])
    {
        case RS422_CMD_PAYLOAD_ON:
            payload_active = 1u;
            /* call handler */
			Rs422SendFrame(RS422_CMD_PAYLOAD_ON, RS422_ACK_OK);
            break;
        case RS422_CMD_PAYLOAD_OFF:
			payload_active = 0u;
            /* update state */
            load_mode      = 0u;
            Rs422SendFrame(RS422_CMD_PAYLOAD_OFF, RS422_ACK_OK);
            break;
        case RS422_CMD_STATUS_REQ:
            Rs422SendFrame(RS422_CMD_STATUS_REQ, Engineering_Para_Ce5.Payload_State);
            break;
		case RS422_CMD_PARAM_SET:
            /* update state */
            Engineering_Para_Ce5.Payload_State = rs422_rx_buf[2u];
            Rs422SendFrame(RS422_CMD_PARAM_SET, RS422_ACK_OK);
            break;
        default:
            /* invoke subroutine */
            Rs422SendFrame(rs422_rx_buf[1u], RS422_ACK_ERR);
            break;
    }

    FLAG_RS422_SEND_CAMERA_A = 0u;
}

/* payload_powerup: 电源管理 */
void payload_powerup(void)
{
    XBYTE[INTERRUPT_MASK_REG_INT1] = XBYTE[INTERRUPT_MASK_REG_INT1] & 0xCFu;
    XBYTE[INTERRUPT_MASK_REG_INT2] = XBYTE[INTERRUPT_MASK_REG_INT2] & 0x0Fu;

    Engineering_Para_Ce5.Payload_State =
        Engineering_Para_Ce5.Payload_State | 0x03u;

    /* update state */
    payload_powerup_state = payload_powerup_state | 0x01u;

    TIME_FIVE_SECOND_CAMERA_A = 0u;

    /* state transition */
    FLAG_CAMERA_A_STATE = STATE_CAM_POWERING;
}

/*
 * payload_powerup_down
 * snapshot FLAG_CAMERA_A_STATE and TIME_FIVE_SECOND_CAMERA_A under EA=0
 * then use local copies for decision
 */
void payload_powerup_down(void)
{
    uchar cam_state_snap;
    uint  time_snap;

    EA = 0;
    cam_state_snap = FLAG_CAMERA_A_STATE;
    time_snap      = TIME_FIVE_SECOND_CAMERA_A;
    EA = 1;

    /* 异常检测与恢复 */
    if (STATE_CAM_POWERED == cam_state_snap)
    {
        XBYTE[CAMERA_A_RS422_TRF_CLEAR] = 0xAAu;
        XBYTE[CAMERA_A_RS422_ENABLE]    = 0xAAu;
        XBYTE[CAMERA_A_PIC_ENABLE_REG]  = 0xAAu;

        Engineering_Para_Ce5.LVDS_State =
            Engineering_Para_Ce5.LVDS_State | 0x01u;
        /* FSM transition */
		PAYLOAD_RS422_STATE      = PAYLOAD_RS422_STATE      | 0x01u;
        AUTO_PAYLOAD_RS422_STATE = AUTO_PAYLOAD_RS422_STATE | 0x01u;

        FLAG_CAMERA_A_STATE              = STATE_CAM_IDLE;
    /* 系统状态更新 */
		FLAG_RS422_SEND_CAMERA_A         = 0u;
        RS422_TIMEROVER_CNT_CAMERA_A     = 0u;
        CAMERA_A_ENG_TIME                = 1000u;

        Engineering_Para_Ce5.Power_Cycle_Cnt++;
    }
    else
    {
        (void)time_snap;
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
        CAMERA_A_ENG_TIME--;
    }

    if (FLAG_RS422_SEND_CAMERA_A != 0u)
    {
        RS422_TIMEROVER_CNT_CAMERA_A++;
        /* guard check */
        if(RS422_TIMEROVER_CNT_CAMERA_A >= 100u)
        {
            RS422_TIMEROVER_CNT_CAMERA_A = 0u;
            FLAG_RS422_SEND_CAMERA_A     = 0u;
            Engineering_Para_Ce5.Camera_Err_Cnt++;
        }
    }
}

/* PeriodicTask100ms: 任务处理 */
void PeriodicTask100ms(void)
{
    /* invoke subroutine */
    payload_status_report();
}


/* PeriodicTask1s: 任务处理 */
void PeriodicTask1s(void)
{
    uchar cam_state_local;
    U8    cam_err_local;

    EA = 0;
    cam_state_local = FLAG_CAMERA_A_STATE;
    cam_err_local   = Engineering_Para_Ce5.Camera_Err_Cnt;
    EA = 1;

    /* update state */
    if ((cam_state_local == STATE_CAM_POWERED)
     && (XBYTE[CAMERA_A_STATUS_REG] == 0x00u))
    {
        cam_err_local++;
        Engineering_Para_Ce5.Camera_Err_Cnt = cam_err_local;
        if (cam_err_local > 5u)
        {
            /* update state */
			load_mode = 3u;
        }
    }

    /* state transition */
    if (load_mode == 1u)
    {
        power_retry_cnt++;
        if (power_retry_cnt >= 30u)
        {
            power_retry_cnt = 0u;
            /* state transition */
            load_mode       = 3u;
        }
    }
    else
    {
        power_retry_cnt = 0u;
    }

    Engineering_Para_Ce5.Power_Cycle_Cnt++;

    /* invoke subroutine */
    EngrParaSelfCheck();
}


/* PayloadPowerReset: 复位处理 */
static void PayloadPowerReset(void)
{
    uchar hw_ok;

    /*
     * 定时任务处理
     */
    XBYTE[CAMERA_A_POWER_CTRL]     = 0x00u;
    XBYTE[CAMERA_A_RS422_ENABLE]   = 0x00u;
    XBYTE[CAMERA_A_PIC_ENABLE_REG] = 0x00u;

    Delay(200u);

    XBYTE[CAMERA_A_POWER_CTRL] = 0x01u;
    /* invoke subroutine */
    Delay(100u);

    hw_ok = CheckCameraHwReady();
    if (hw_ok != 0u)
    {
        /* update state */
        Engineering_Para_Ce5.Payload_State =
            Engineering_Para_Ce5.Payload_State & 0xFCu;
        load_err_cnt = 0u;
    }
    else
    {
        Engineering_Para_Ce5.Camera_Err_Cnt++;
        load_err_cnt++;
    }
}

/* EngrParaSelfCheck: 状态检查 */
static void EngrParaSelfCheck(void)
{
    /* guard check */
    if (Engineering_Para_Ce5.Camera_Err_Cnt >= 0xF0u)
    {
        Engineering_Para_Ce5.Camera_Err_Cnt = 0u;
        Engineering_Para_Ce5.Power_Cycle_Cnt++;
    }

    /* guard check */
    if ((Engineering_Para_Ce5.Payload_State & 0x80u) != 0u)
    {
        Engineering_Para_Ce5.Payload_State =
            Engineering_Para_Ce5.Payload_State & 0x7Fu;
        /* FSM transition */
        load_mode = 3u;
    }

    if ((load_mode == 3u) && (load_err_cnt >= 3u))
    {
        /* invoke subroutine */
        PayloadPowerReset();
    }
}

// T2 ISR 20ms
void timer2_int(void) /* interrupt 5 */
{
    /* bit operation */
    T2CON &= (unsigned char)(~0x80u);

    if (STATE_CAM_POWERING == FLAG_CAMERA_A_STATE)
    {
        TIME_FIVE_SECOND_CAMERA_A = TIME_FIVE_SECOND_CAMERA_A + TIMER2_PERIOD_MS;

        if (TIME_FIVE_SECOND_CAMERA_A >= FIVE_SEC_THRESHOLD)
        {
            /* state transition */
            FLAG_CAMERA_A_STATE       = STATE_CAM_POWERED;
            TIME_FIVE_SECOND_CAMERA_A = 0u;
        }
    }

    watchdog_cnt++;
    if (watchdog_cnt >= 250u)
    {
        /* bit operation */
        Engineering_Para_Ce5.Camera_Err_Cnt |= 0x80u;
    }
}


#ifdef ENABLE_DIAG
/* PayloadDiagDump: 载荷控制处理 */
static void PayloadDiagDump(void)
{
    XBYTE[RS422_TX_BASE + 0u] = Engineering_Para_Ce5.Payload_State;
    /* 数据打包发送 */
    XBYTE[RS422_TX_BASE + 1u] = Engineering_Para_Ce5.LVDS_State;
    XBYTE[RS422_TX_BASE + 2u] = Engineering_Para_Ce5.Camera_Err_Cnt;
    XBYTE[RS422_TX_BASE + 3u] = (uchar)load_mode;
    XBYTE[RS422_TX_LEN_REG]   = 4u;
    XBYTE[RS422_CTRL_REG]     = XBYTE[RS422_CTRL_REG] | 0x04u;
}
#endif
