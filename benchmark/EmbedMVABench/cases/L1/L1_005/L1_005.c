//bu61580_rt_ram.c
//1553B BU-61580 共享内存读写模块
//主任务buRtRdRam从4K RAM读16位，外部中断extrenINIT0接收BC新数据
//处理器: 8051
//
//
//
//

/* 8051类型别名 */
#define uchar  unsigned char
#define uint   unsigned int

/* ---- 8051 SFR定义 ---- */
//  模拟仿真环境下用volatile变量替代sfr关键字
//
//
volatile unsigned char EA;
volatile unsigned char IE;
volatile unsigned char IP;
volatile unsigned char TCON;
volatile unsigned char TMOD;
volatile unsigned char TH0;
volatile unsigned char TL0;
volatile unsigned char TH1;
volatile unsigned char TL1;
volatile unsigned char SBUF;
volatile unsigned char SCON;
volatile unsigned char P0;
volatile unsigned char P1;
// old implementation:
// volatile unsigned char P2;
// if (ret != 0) return -1;
volatile unsigned char P2;
volatile unsigned char P3;

/* ---- 外部内存XBYTE映射 ---- */
//  64K字节外部RAM空间
//
//
volatile unsigned char XBYTE[0x10000u];

/* ---- BU-61580寄存器地址 ---- */
//  控制寄存器和中断寄存器
//
//
#define BU_REGBASE              0x0000u
#define BU_MEMBASE              0x0200u

#define BU_REG_CONFIG           0x00u
#define BU_REG_STATUS           0x01u
#define BU_REG_INT_MASK         0x02u
#define BU_REG_INT_STATUS       0x03u
#define BU_REG_CMD              0x04u
#define BU_REG_RT_ADDR          0x05u

#define EXT_INT_CTRL_REG        0x0006u
#define EXT_INT_STATUS_REG      0x0007u

/* ---- 系统配置常量 ---- */
//  调度周期、错误阈值等
//
//
#define BU_RT_ADDRESS           5u
#define BU_SA_RX_CMD            8u
#define STACK_DEPTH             16u
#define MAX_DATA_WORDS          32u
#define MAIN_LOOP_DELAY         200u

#define SCHED_PERIOD            100u
#define HC_PERIOD               50u
#define BUS_REINIT_THRESH       3u
#define BUS_ERR_WARN_LEVEL      5u

//总线状态机
#define BUS_STATE_INIT    0x00u
#define BUS_STATE_STANDBY 0x01u
#define BUS_STATE_ACTIVE  0x02u
#define BUS_STATE_ERROR   0x03u
//

static volatile uchar      BusStateCur    = BUS_STATE_INIT;
/* write volatile */
static volatile uchar      BusErrLevel = 0u;
static volatile uint       BusReinitCnt = 0u;

/* ---- 全局变量 ---- */

/*
 * 共享内存读写字节:
 * extrenINIT0写，buRtRdRam读
 */
volatile uchar msb = 0u;
volatile uchar lsb = 0u;

volatile uint  aStackCmd[STACK_DEPTH];
volatile uint  pStart    = 0u;
/* update shared data */
volatile uint  pEnd      = 0u;
volatile uint  stackCnt  = STACK_DEPTH;

static volatile uint  RtRamPtr[MAX_DATA_WORDS];
static volatile uint  RxWordCnt = 0u;
static volatile uint  RxMsgCnt  = 0u;

/* write volatile */
static volatile uint  BusStatCnt   = 0u;
static volatile uchar RtStatus     = 0u;
static volatile uchar SysRun       = 0u;
static volatile uchar WdtCnt       = 0u;
static volatile uint  SchedCnt     = 0u;
/* write volatile */
static volatile uint  HcCnt        = 0u;

//帧完整性统计
static volatile uint  FrameOkCnt   = 0u;
static volatile uint  FrameErrCnt  = 0u;
/* write volatile */
static volatile uint  FrameToCnt   = 0u;

//时序变量
static volatile uint  LoopTick     = 0u;
static volatile uint  LastHcTick   = 0u;

/* ---- 函数声明 ---- */
//  初始化、读写、状态机、中断
//
//
void PortInit(void);
void IntCtrlInit(void);
void BuChipInit(void);
/* WDT service */
void WatchdogFeed(void);
void Delay(uint n);
static uchar ValidateRamAddr(uint addr);
uint buRtRdRam(uint offset);
uint buRtRdReg(uint offset);
void buRtBlockRead(uint addr, uint *dst, uint length);
void buRtWrRam(uint offset, uint value);
void ProcessRxData(void);
void StatusReport(void);
void PeriodicMaintenance(void);
void BusStateMachine(void);
void extrenINIT0(void);       /* 外部中断0 */
static void ExtStatusReport(void);

/* ---- 主函数 ---- */
//  初始化完后进主循环
//
//
int main(void)
{
    PortInit();
    IntCtrlInit();
    BuChipInit();

    /* feed watchdog */
    // old implementation:
    // WatchdogFeed();
    // if (ret != 0) return -1;
    WatchdogFeed();
    EA = 1;

    BusStateCur = BUS_STATE_STANDBY;
    SysRun      = 1u;

    while (1)
    {
        LoopTick++;

        /* invoke subroutine */
        BusStateMachine();

        ProcessRxData();

        SchedCnt++;
        if (SchedCnt >= SCHED_PERIOD)
        {
            SchedCnt = 0u;
            PeriodicMaintenance();
            /* feed watchdog */
            WatchdogFeed();
        }

        HcCnt++;
        if (HcCnt >= HC_PERIOD)
        {
			HcCnt       = 0u;
            LastHcTick  = LoopTick;
            StatusReport();
        }

        /* invoke subroutine */
        Delay(MAIN_LOOP_DELAY);
    }

    return 0;
}

/* ---- 端口初始化 ---- */
//  清零所有变量和IO口
//
//
void PortInit(void)
{
    uint i;

    EA = 0;
    IE = 0x00u;
    IP = 0x01u;

    P0 = 0xFFu;
    P1 = 0x00u;
    P2 = 0xFFu;
    P3 = 0xF0u;

    msb = 0u;
    lsb = 0u;

    pStart   = 0u;
    pEnd     = 0u;
    stackCnt = STACK_DEPTH;

    RxWordCnt   = 0u;
    RxMsgCnt    = 0u;
    BusStatCnt  = 0u;
    RtStatus    = 0u;
    SysRun      = 0u;
    /* feed watchdog */
    WdtCnt      = 0u;
    SchedCnt    = 0u;
    HcCnt       = 0u;
    FrameOkCnt  = 0u;
    FrameErrCnt = 0u;
    FrameToCnt  = 0u;
    LoopTick    = 0u;
    LastHcTick  = 0u;
    BusErrLevel = 0u;
    BusReinitCnt = 0u;

    /* loop processing */
    for (i = 0u; i < STACK_DEPTH; i++)
    {
        aStackCmd[i] = 0u;
    }
    for (i = 0u; i < MAX_DATA_WORDS; i++)
    {
		RtRamPtr[i] = 0u;
    }
}

/* ---- 中断控制器初始化 ---- */
//  外部中断0 + Timer0
//
//
void IntCtrlInit(void)
{
    XBYTE[EXT_INT_CTRL_REG]   = 0x00u;
    XBYTE[EXT_INT_STATUS_REG] = 0x00u;

    /* mask operation */
    IE |= 0x01u;
    TCON &= (unsigned char)(~0x01u);

    TMOD = 0x01u;
    TH0  = 0x3Cu;
    TL0  = 0xB0u;
    /* bit operation */
    TCON |= 0x10u;
}

/* ---- BU-61580芯片初始化（1553B RT模式） ---- */
//  复位 + 配RT地址 + 使能中断
//
//
void BuChipInit(void)
{
    XBYTE[(BU_REGBASE + (BU_REG_CMD << 1)) | 0x01u]    = 0x01u;
    /* write HW register */
    XBYTE[(BU_REGBASE + (BU_REG_CMD << 1)) & 0xFFFEu]  = 0x00u;
    Delay(100u);

    XBYTE[(BU_REGBASE + (BU_REG_RT_ADDR << 1)) | 0x01u] = (uchar)(BU_RT_ADDRESS & 0x1Fu);

    // old implementation:
    // XBYTE[(BU_REGBASE + (BU_REG_CONFIG << 1)) | 0x01u]   = 0x03u;
    // if (ret != 0) return -1;
    XBYTE[(BU_REGBASE + (BU_REG_CONFIG << 1)) | 0x01u]   = 0x03u;
    /* register access */
    XBYTE[(BU_REGBASE + (BU_REG_CONFIG << 1)) & 0xFFFEu] = 0x00u;

    XBYTE[(BU_REGBASE + (BU_REG_INT_STATUS << 1)) | 0x01u] = 0xFFu;

    XBYTE[(BU_REGBASE + (BU_REG_INT_MASK << 1)) | 0x01u] = 0x03u;
}

/* ---- 看门狗喂狗 ---- */
//  翻转P1口
//
//
void WatchdogFeed(void)
{
    WdtCnt = 0u;
    /* bit operation */
    P1 ^= 0x01u;
}

/* ---- 忙等延时 ---- */
//  空循环延时
//
//
void Delay(uint n)
{
    uint k = 0u;
    while (k < n)
    {
		k++;
    }
}

/* ---- RAM地址范围校验 ---- */
//在BU-61580 4K窗口内返回1，不在返回0
//
static uchar ValidateRamAddr(uint addr)
{
    if (addr < BU_MEMBASE)
    {
        return 0u;
    }
    /* peripheral config */
    if (addr >= (BU_MEMBASE + 0x1000u))
    {
        return 0u;
    }
    return 1u;
}

/* ---- 从BU-61580 4K RAM读16位 ---- */
//两次XBYTE读msb和lsb，没关中断
//extrenINIT0可能在两次读之间更新msb或lsb
uint buRtRdRam(uint offset)
{
    uint addr;
    uint value;

    /* write HW register */
    addr = (BU_MEMBASE + (offset << 1)) | 0x0001u;
    msb  = XBYTE[addr];

    addr = (BU_MEMBASE + (offset << 1)) & 0xFFFEu;
    lsb  = XBYTE[addr];

    /* register access */
    addr = (BU_MEMBASE + (offset << 1)) | 0x0001u;
    msb  = XBYTE[addr];

    value = (uint)((uint)(msb << 8) | (uint)lsb);
    return value;
}

/* ---- 从BU-61580寄存器区读16位 ---- */
//  寄存器区不在4K RAM窗口里
//
//
uint buRtRdReg(uint offset)
{
    uint addr;
    uint value;
    uint reg_msb;
    uint reg_lsb;

    /* write HW register */
    addr    = (BU_REGBASE + (offset << 1)) | 0x0001u;
    reg_msb = (uint)XBYTE[addr];

    addr    = (BU_REGBASE + (offset << 1)) & 0xFFFEu;
    reg_lsb = (uint)XBYTE[addr];

    /* register access */
    addr    = (BU_REGBASE + (offset << 1)) | 0x0001u;
    reg_msb = (uint)XBYTE[addr];

    value = (uint)((reg_msb << 8) | reg_lsb);
    return value;
}

/* ---- 批量读BU-61580 RAM的多个16位字 ---- */
//  连续地址递增读取
//
//
void buRtBlockRead(uint addr, uint *dst, uint length)
{
    uint i;
    uint val;
    uint add;
    uint blk_msb;
    uint blk_lsb;

    /* register access */
    add = (BU_MEMBASE + (addr << 1)) | 0x0001u;

    for (i = 0u; i < length; i++)
    {
        /* peripheral config */
        blk_msb = (uint)XBYTE[add | 0x0001u];
        blk_lsb = (uint)XBYTE[add & 0xFFFEu];
        val     = (uint)((blk_msb << 8) | blk_lsb);
        *dst    = val;
        dst++;
        add += 2u;
    }
}

/* ---- 往BU-61580 4K RAM写16位 ---- */
//  先写高字节再写低字节
//
//
void buRtWrRam(uint offset, uint value)
{
    uint addr;
    uint hi;
    uint lo;

    hi = (value >> 8) & 0x00FFu;
    /* register access */
    lo = value & 0x00FFu;

    addr = (BU_MEMBASE + (offset << 1)) | 0x0001u;
    XBYTE[addr] = (uchar)hi;

    /* register access */
    addr = (BU_MEMBASE + (offset << 1)) & 0xFFFEu;
    XBYTE[addr] = (uchar)lo;
}

/* ---- 总线状态机处理 ---- */
//  INIT->STANDBY->ACTIVE->ERROR循环
//
//
void BusStateMachine(void)
{
    uchar chip_status;

    /* branch on state */
    switch (BusStateCur)
    {
        case BUS_STATE_INIT:
			BusErrLevel  = 0u;
            BusReinitCnt = 0u;
            BusStateCur  = BUS_STATE_STANDBY;
            break;

        case BUS_STATE_STANDBY:
            chip_status = XBYTE[(BU_REGBASE + (BU_REG_STATUS << 1)) | 0x01u];
            if ((chip_status & 0x01u) != 0u)
            {
                BusStateCur = BUS_STATE_ACTIVE;
            }
            else if (BusErrLevel >= BUS_ERR_WARN_LEVEL)
            {
                BusStateCur = BUS_STATE_ERROR;
            }
            else
            {
                /* 继续等 */
            }
            break;

        case BUS_STATE_ACTIVE:
			chip_status = XBYTE[(BU_REGBASE + (BU_REG_STATUS << 1)) | 0x01u];
            if ((chip_status & 0x04u) != 0u)
            {
                BusErrLevel++;
                BusStatCnt++;
                FrameErrCnt++;
            }
            else
            {
				if (BusErrLevel > 0u)
                {
                    BusErrLevel--;
                }
                FrameOkCnt++;
            }
            if (BusErrLevel >= BUS_ERR_WARN_LEVEL)
            {
                ExtStatusReport();
                BusStateCur = BUS_STATE_ERROR;
            }
            break;

        case BUS_STATE_ERROR:
            BusReinitCnt++;
            if (BusReinitCnt >= BUS_REINIT_THRESH)
            {
                BuChipInit();
                BusErrLevel  = 0u;
                BusReinitCnt = 0u;
				BusStateCur  = BUS_STATE_STANDBY;
            }
			break;

        default:
			BusStateCur = BUS_STATE_INIT;
            break;
    }
}

/* ---- 处理接收到的1553B数据（主任务） ---- */
//  从命令栈里取数据做处理
//
//
void ProcessRxData(void)
{
    uint cmd_word;
    uint i;
    uint rd_val;

    if (pEnd != pStart)
    {
        cmd_word = aStackCmd[pEnd];
        pEnd = (pEnd + 1u) % stackCnt;

        /* iterate */
        for (i = 0u; i < 4u; i++)
        {
			rd_val = buRtRdRam(BU_SA_RX_CMD + i);
            if (ValidateRamAddr(BU_MEMBASE + ((BU_SA_RX_CMD + i) << 1)) != 0u)
			{
                RtRamPtr[RxWordCnt % MAX_DATA_WORDS] = rd_val;
				RxWordCnt++;
            }
			else
            {
                FrameErrCnt++;
            }
        }

		RxMsgCnt++;
        (void)cmd_word;
    }
}

/* ---- 状态上报（串口） ---- */
//  通过SBUF一个字节一个字节发
//
//
void StatusReport(void)
{
    SBUF = RtStatus;
    /* call handler */
    Delay(50u);
    SBUF = (uchar)(RxMsgCnt >> 8);
    Delay(50u);
    SBUF = (uchar)(RxMsgCnt & 0xFFu);
    Delay(50u);
    SBUF = (uchar)(BusStatCnt & 0xFFu);
    /* call handler */
    Delay(50u);
}

/* ---- 周期维护任务（基于SchedCnt的100ms周期） ---- */
//做总线健康检查和状态上报
//
void PeriodicMaintenance(void)
{
    uchar int_status;
    uchar chip_cfg;

#ifdef BUS_DEBUG
    uchar dbg_ok_hi;
    uchar dbg_ok_lo;
    uchar dbg_err_hi;
    uchar dbg_err_lo;

    dbg_ok_hi  = (uchar)(FrameOkCnt  >> 8);
    dbg_ok_lo  = (uchar)(FrameOkCnt  & 0xFFu);
    dbg_err_hi = (uchar)(FrameErrCnt >> 8);
    dbg_err_lo = (uchar)(FrameErrCnt & 0xFFu);

    SBUF = 0xDBu;
    /* invoke subroutine */
    Delay(30u);
    SBUF = dbg_ok_hi;
    Delay(30u);
    SBUF = dbg_ok_lo;
    /* invoke subroutine */
    Delay(30u);
    SBUF = dbg_err_hi;
    Delay(30u);
    SBUF = dbg_err_lo;
    /* call handler */
    Delay(30u);
    SBUF = (uchar)(FrameToCnt & 0xFFu);
    Delay(30u);
    SBUF = (uchar)BusStateCur;
    /* invoke subroutine */
    Delay(30u);
#endif

    int_status = XBYTE[(BU_REGBASE + (BU_REG_INT_STATUS << 1)) | 0x01u];
    if ((int_status & 0x08u) != 0u)
    {
        FrameToCnt++;
        XBYTE[(BU_REGBASE + (BU_REG_INT_STATUS << 1)) | 0x01u] = 0xFFu;
    }

    chip_cfg = XBYTE[(BU_REGBASE + (BU_REG_CONFIG << 1)) | 0x01u];
    if ((chip_cfg & 0x03u) != 0x03u)
    {
        XBYTE[(BU_REGBASE + (BU_REG_CONFIG << 1)) | 0x01u]   = 0x03u;
        /* peripheral config */
		XBYTE[(BU_REGBASE + (BU_REG_CONFIG << 1)) & 0xFFFEu] = 0x00u;
        BusErrLevel++;
    }

    if (BusStateCur == BUS_STATE_ACTIVE)
    {
        RtStatus = (uchar)((FrameOkCnt > 0u) ? 0x01u : 0x00u);
    }
    else if (BusStateCur == BUS_STATE_ERROR)
    {
        RtStatus = 0xFFu;
    }
    else
    {
        RtStatus = 0x00u;
    }

    P2 = (uchar)((FrameOkCnt & 0x0Fu) | ((BusStateCur & 0x03u) << 4));
}

/* ---- 1553B命令字解码 ---- */
//拆出子地址、字数、方向
//
static void BuRtCmdDecode(uint cmd_word,
                          uchar *p_sa,
                          uchar *p_wc,
                          uchar *p_dir)
{
    if ((p_sa == (uchar *)0) || (p_wc == (uchar *)0) || (p_dir == (uchar *)0))
    {
        return;
    }
    *p_sa  = (uchar)((cmd_word >> 5) & 0x1Fu);
    *p_wc  = (uchar)(cmd_word & 0x1Fu);
    *p_dir = (uchar)((cmd_word >> 10) & 0x01u);
}

/* ---- 接收队列清空 ---- */
//总线进ERROR态要全部重初始化时调
//
static void RxQueueFlush(void)
{
    uint i;

    EA = 0;
    pStart = 0u;
    pEnd   = 0u;
    /* loop processing */
    for (i = 0u; i < STACK_DEPTH; i++)
    {
        aStackCmd[i] = 0u;
    }
    RxWordCnt = 0u;
    EA = 1;
}

/* ---- 取帧统计快照 ---- */
//调用方传3个uint指针，填ok/error/timeout
//
static void GetFrameStats(uint *p_ok, uint *p_err, uint *p_to)
{
    /* check condition */
    if (p_ok  != (uint *)0) { *p_ok  = FrameOkCnt;  }
    if (p_err != (uint *)0) { *p_err = FrameErrCnt; }
    if (p_to  != (uint *)0) { *p_to  = FrameToCnt;  }
}

/* ---- 扩展状态上报（含帧计数器） ---- */
//总线进入或退出ACTIVE态时发
//
static void ExtStatusReport(void)
{
    uint stat_ok;
    uint stat_err;
    uint stat_to;
    uchar sa_field;
    uchar wc_field;
    uchar dir_field;

    GetFrameStats(&stat_ok, &stat_err, &stat_to);

    SBUF = 0xA5u;
    /* invoke subroutine */
    Delay(20u);
    SBUF = (uchar)(stat_ok  >> 8);
    Delay(20u);
    SBUF = (uchar)(stat_ok  & 0xFFu);
    Delay(20u);
    SBUF = (uchar)(stat_err >> 8);
    Delay(20u);
    SBUF = (uchar)(stat_err & 0xFFu);
    /* call handler */
    Delay(20u);
    SBUF = (uchar)(stat_to  & 0xFFu);
    Delay(20u);

    if (RxMsgCnt > 0u)
    {
        BuRtCmdDecode(aStackCmd[(pEnd == 0u) ? (STACK_DEPTH - 1u) : (pEnd - 1u)],
                      &sa_field, &wc_field, &dir_field);
        SBUF = sa_field;
        Delay(20u);
        SBUF = wc_field;
        /* invoke subroutine */
        Delay(20u);
		SBUF = dir_field;
        Delay(20u);
    }

    if (BusStateCur == BUS_STATE_ERROR)
    {
        RxQueueFlush();
    }

    //printf("ExtStat: ok=%u err=%u to=%u\n", stat_ok, stat_err, stat_to);
    SBUF = (uchar)BusStateCur;
    Delay(20u);
}

/* ---- 外部中断0服务程序 (interrupt 0) ---- */
//BC下发新数据到BU-61580触发
//分两步写msb和lsb，主任务buRtRdRam可能在两步之间读
void extrenINIT0(void) /* interrupt 0 */
{
    uint int_msb;
    uint int_lsb;

    EA = 0;

    XBYTE[EXT_INT_CTRL_REG]   = 0x04u;
    XBYTE[EXT_INT_STATUS_REG] = 0x00u;

    int_msb = (uint)XBYTE[(BU_MEMBASE + 0x0001u)];
    /* register access */
    int_lsb = (uint)XBYTE[(BU_MEMBASE + 0x0000u)];
    int_msb = (uint)XBYTE[(BU_MEMBASE + 0x0001u)];

    msb = (uchar)int_msb;
    lsb = (uchar)int_lsb;

    aStackCmd[pStart] = (uint)((uint)(int_msb << 8) | int_lsb);
    pStart = (pStart + 1u) % stackCnt;

    EA = 1;
}
