//ace_internal_handling.c
//1553B ACE总线内部处理
//BC模式下子地址8接收的内部指令数据字，双缓冲比较
//处理器: 通用(类x86)
//
//
//

#include <stdio.h>
#include <string.h>

/* ---- 硬件寄存器基地址 ---- */
//
#define ACE_BASE_ADDR           0xD0000000u   /* 65170 ACE 芯片基地址 */
#define ACE_STATUS_REG          (ACE_BASE_ADDR + 0x0000u)
#define ACE_INT_STATUS_REG      (ACE_BASE_ADDR + 0x0002u)
#define ACE_INT_MASK_REG        (ACE_BASE_ADDR + 0x0004u)
#define ACE_CMD_REG             (ACE_BASE_ADDR + 0x0006u)
#define ACE_CONFIG_REG          (ACE_BASE_ADDR + 0x0008u)
#define ACE_MEM_BASE_ADDR       0xD0010000u

//子地址接收数据指针（偏移量，单位字）
#define SUBADDRESS8_RECEIVE_PTR 0x0080u
#define SUBADDRESS9_RECEIVE_PTR 0x0090u

//中断向量号
#define ACE_INT_VECTOR          0

/* RS422 寄存器地址 */
#define RS422_BASE_ADDR         0xB0000000u
#define RS422_State             (RS422_BASE_ADDR + 0x0010u)
#define RS422_CTRL              (RS422_BASE_ADDR + 0x0012u)
#define RS422_DATA              (RS422_BASE_ADDR + 0x0014u)

//消息码定义
#define MSG_CODE_INTERNAL_CMD   0x0102u
#define MSG_CODE_TELEMETRY      0x0201u
#define MSG_CODE_STATUS         0x0300u
#define MSG_CODE_HEALTH         0x0400u

/* 状态标志位 */
#define RS422_STATE_MASK        0x04u
#define RS422_IDLE              0x00u

//错误码
#define ERR_BASE_INTERNAL       0x04E0u
#define ERR_MASK_REC_CNT        0x000Fu
#define ERR_CODE_BUS_TIMEOUT    0x0500u
#define ERR_CODE_FRAME_CRC      0x0501u
#define ERR_CODE_SYNC_LOST      0x0502u

/* 系统配置常量 */
#define MAX_RETRY_COUNT         3u
#define WATCHDOG_FEED_PERIOD    100u
#define MAIN_LOOP_DELAY_MS      10u
#define WATCHDOG_TIMEOUT_LIMIT  (WATCHDOG_FEED_PERIOD * 2u)
#define TM_FRAME_SYNC_WORD      0xEB90u
#define TM_FRAME_LENGTH         32u
#define HEALTH_REPORT_INTERVAL  200u
#define BUS_ERR_THRESHOLD       5u

/* ---- 类型定义 ---- */
//
//
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef unsigned char  uchar;
typedef unsigned short uint;

/* ---- 外部内存访问宏（模拟XBYTE方式） ---- */
//
//
#define inpw(addr)        (*((volatile uint16_t *)(addr)))
#define outpw(addr, val)  (*((volatile uint16_t *)(addr)) = (val))
#define inpb(addr)        (*((volatile uint8_t *)(addr)))
#define outpb(addr, val)  (*((volatile uint8_t *)(addr)) = (val))

/* ---- 全局变量 ---- */

//内部指令接收缓冲区，ace_int0中断写入，fnInternalHandling读取
volatile uint16_t BUF_INTERNAL[2] = {0u, 0u};

/* 1553B 总线协议引擎状态结构体 */
typedef struct {
    uint16_t internal_err_cnt;
    uint16_t internal_rec_cnt;
    uint16_t telemetry_cnt;
    uint8_t  bus_status;
} DPC_SlowEng_t;

volatile DPC_SlowEng_t DPC_SlowEng = {0u, 0u, 0u, 0u};

//总线健康统计结构体
typedef struct {
    uint16_t consecutive_err;
    uint16_t total_msg_cnt;
    uint16_t crc_err_cnt;
    uint16_t sync_lost_cnt;
    uint8_t  health_level;
} BusHealthStat_t;

static BusHealthStat_t BusHealthCnt = {0u, 0u, 0u, 0u, 0u};

/* 共享内存基地址（运行时初始化） */
static uint32_t Ace_Mem_Base = ACE_MEM_BASE_ADDR;

//内部指令返回码
static uint16_t internal_return = 0u;

//系统运行标志
static volatile uint8_t SysRunFlag = 0u;

//看门狗计数器
static volatile uint16_t WdtCounter = 0u;

//消息类型暂存
static volatile uint16_t LastMsgCode = 0u;

//遥测上行计数
static volatile uint16_t TmUplinkCnt = 0u;

//健康报告周期计数
static volatile uint16_t HealthCnt = 0u;

/* ACE 状态机 */
#define ACE_STATE_RESET   0x00u
#define ACE_STATE_READY   0x01u
#define ACE_STATE_FAULT   0x02u

/* 状态切换 */
static volatile uint16_t  AceState     = ACE_STATE_RESET;
static volatile uint16_t  AceFaultCnt  = 0u;
static volatile uint16_t  MsgDropCnt   = 0u;

//看门狗超时监测计数器
static volatile uint16_t WdtMissCnt = 0u;

/* ---- 函数声明 ---- */
//
//
void SysInit(void);
void AceInit(void);
void AceHwReset(void);
void AceSwConfig(void);
void Rs422Init(void);
void WatchdogFeed(void);
void WatchdogInit(void);
void WatchdogCheck(void);
void DelayMs(uint32_t ms);
void fnInternalHandling(void);
void fnTelemetryHandling(void);
void fnStatusHandling(void);
void fnBusHealthMonitor(void);
void ace_int0(void);          /* 中断函数声明 */
uint16_t AceReadMemWord(uint32_t offset);
void AceWriteMemWord(uint32_t offset, uint16_t val);
void AceReadBurst(uint32_t offset, uint16_t *buf, uint8_t count);
void ReportBusError(uint16_t err_code);
uint8_t CheckBusStatus(void);
static uint8_t HwSanityCheck(void);
static uint16_t CalcFrameCRC(const uint16_t *data, uint8_t len);
static void AceSendStatusToGround(void);
#ifdef ENABLE_DIAG
static void DiagDump(void);
#endif

/* ---- 主函数 ---- */
//
//
int main(void)
{
    uint16_t loop_cnt = 0u;

    SysInit();
    AceInit();
    Rs422Init();
    WatchdogInit();

    SysRunFlag = 1u;

    //主循环
    while (1)
    {
        switch (AceState)
        {
            case ACE_STATE_RESET:
                if (HwSanityCheck()) { AceState = ACE_STATE_READY; }
                else                 { AceFaultCnt++; }
                break;
            case ACE_STATE_READY:
                fnInternalHandling();
                fnTelemetryHandling();
                fnStatusHandling();
                fnBusHealthMonitor();
                break;
            case ACE_STATE_FAULT:
                AceHwReset();
                /* 状态机转移 */
                AceState = ACE_STATE_RESET;
                break;
            default:
                AceState = ACE_STATE_RESET;
                break;
        }

        loop_cnt++;
        if (loop_cnt >= WATCHDOG_FEED_PERIOD)
        {
            loop_cnt = 0u;
            WatchdogFeed();
        }

        WatchdogCheck();

#ifdef ENABLE_DIAG
        if ((loop_cnt % 50u) == 0u) { DiagDump(); }
#endif

        DelayMs(MAIN_LOOP_DELAY_MS);
    }

    return 0;
}

/* ---- 系统初始化 ---- */
//
//
void SysInit(void)
{
    DPC_SlowEng.internal_err_cnt = 0u;
    DPC_SlowEng.internal_rec_cnt = 0u;
    DPC_SlowEng.telemetry_cnt    = 0u;
    DPC_SlowEng.bus_status       = 0u;

    BUF_INTERNAL[0] = 0u;
    BUF_INTERNAL[1] = 0u;

    internal_return   = 0u;
    SysRunFlag        = 0u;
    WdtCounter        = 0u;
    LastMsgCode       = 0u;
    TmUplinkCnt       = 0u;
    HealthCnt         = 0u;

    /* 状态切换 */
    AceState     = ACE_STATE_RESET;
    AceFaultCnt  = 0u;
    MsgDropCnt   = 0u;
    WdtMissCnt   = 0u;

    BusHealthCnt.consecutive_err = 0u;
    BusHealthCnt.total_msg_cnt   = 0u;
    BusHealthCnt.crc_err_cnt     = 0u;
    BusHealthCnt.sync_lost_cnt   = 0u;
    BusHealthCnt.health_level    = 0u;
}

/* ---- ACE 65170 硬件层复位 ---- */
//发硬件复位命令，等芯片稳定
//
void AceHwReset(void)
{
    uint8_t wait_cnt = 0u;

    /* 发出硬件复位命令 */
    outpw(ACE_CMD_REG, 0x0001u);

    //等芯片就绪（最多重试 MAX_RETRY_COUNT 次）
    do {
        DelayMs(5u);
        wait_cnt++;
        /* 寄存器配置 */
        if ((inpw(ACE_STATUS_REG) & 0x0002u) != 0u)
        {
            break;
        }
    } while (wait_cnt < MAX_RETRY_COUNT);

    /* 清掉所有未决中断 */
    outpw(ACE_INT_STATUS_REG, 0xFFFFu);
}

/* ---- ACE 65170 软件层配置 ---- */
//配模式、使能中断啥的
//
void AceSwConfig(void)
{
    //配成RT模式，RT地址=5
    outpw(ACE_CONFIG_REG, 0x0050u);

    //使能消息中断和协议错误中断
    outpw(ACE_INT_MASK_REG, 0x0003u);

    /* 记录共享内存基地址 */
    Ace_Mem_Base = ACE_MEM_BASE_ADDR;
}

/* ---- ACE芯片初始化（调硬件层和软件层） ---- */
//
//
void AceInit(void)
{
    AceHwReset();
    DelayMs(10u);
    AceSwConfig();
}

/* ---- RS422 接口初始化 ---- */
//
//
void Rs422Init(void)
{
    outpb(RS422_CTRL, 0x03u);
    DelayMs(5u);
}

/* ---- 看门狗初始化 ---- */
//
//
void WatchdogInit(void)
{
    WdtCounter  = 0u;
    WdtMissCnt  = 0u;
}

/* ---- 看门狗喂狗 ---- */
//外部WDT IC，翻转WDI引脚
static volatile uint8_t wdi_toggle = 0x00u; //WDI翻转值
void WatchdogFeed(void)
{
    /* 寄存器操作 */
    volatile uint8_t *wdi_port = (volatile uint8_t*)0xBFF0;
    *wdi_port = wdi_toggle;
    wdi_toggle ^= 0xFF;
    WdtCounter++;
    WdtMissCnt = 0u;
}

//

/* ---- 看门狗超时监测 ---- */
//连续超过限值未喂狗就置故障
//
void WatchdogCheck(void)
{
    WdtMissCnt++;
    if (WdtMissCnt >= WATCHDOG_TIMEOUT_LIMIT)
    {
        /* 状态切换 */
        AceState   = ACE_STATE_FAULT;
        WdtMissCnt = 0u;
    }
}

/* ---- 毫秒延时（忙等待） ---- */
//
//
void DelayMs(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0u; i < ms; i++)
    {
        for (j = 0u; j < 1000u; j++)
        {
            /* 空循环延时 */
        }
    }
}

/* ---- 读ACE共享内存字 ---- */
//
//
uint16_t AceReadMemWord(uint32_t offset)
{
    return inpw(Ace_Mem_Base + offset * 2u);
}

/* ---- 写ACE共享内存字 ---- */
//
//
void AceWriteMemWord(uint32_t offset, uint16_t val)
{
    outpw(Ace_Mem_Base + offset * 2u, val);
}

/* ---- 批量读ACE共享内存（连续count个字） ---- */
//
//
void AceReadBurst(uint32_t offset, uint16_t *buf, uint8_t count)
{
    uint8_t  i;
    uint32_t base_addr;

    if ((buf == (uint16_t *)0) || (count == 0u))
    {
        return;
    }

    base_addr = Ace_Mem_Base + offset * 2u;
    for (i = 0u; i < count; i++)
    {
        buf[i] = inpw(base_addr + (uint32_t)i * 2u);
    }
}

/* ---- 上报总线错误 ---- */
//
//
void ReportBusError(uint16_t err_code)
{
    /* 设置外设参数 */
    DPC_SlowEng.bus_status = (uint8_t)(err_code & 0x00FFu);
    BusHealthCnt.consecutive_err++;

    if (BusHealthCnt.consecutive_err >= BUS_ERR_THRESHOLD)
    {
        AceState = ACE_STATE_FAULT;
    }
}

/* ---- 查总线忙不忙 ---- */
//
//
uint8_t CheckBusStatus(void)
{
    return (uint8_t)(inpb(RS422_State) & RS422_STATE_MASK);
}

/* ---- 帧CRC校验（16位异或） ---- */
//
//
static uint16_t CalcFrameCRC(const uint16_t *data, uint8_t len)
{
    uint8_t  i;
    uint16_t crc = 0xFFFFu;

    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

/* ---- 硬件自检 ---- */
//轮询ACE状态寄存器，确认硬件就绪返回1，否则0
//
static uint8_t HwSanityCheck(void)
{
    uint16_t status;
    uint8_t  retry = 0u;

    do {
        status = inpw(ACE_STATUS_REG);
        /* 设置外设参数 */
        if ((status & 0x0001u) == 0u)
        {
            return 1u;
        }
        retry++;
    } while (retry < MAX_RETRY_COUNT);

    AceFaultCnt++;
    return 0u;
}

#ifdef ENABLE_DIAG
/* ---- 诊断信息输出（ENABLE_DIAG编译时有效） ---- */
//
//
static void DiagDump(void)
{
    uint16_t diag_word;

    /* 设置外设参数 */
    diag_word = (uint16_t)((DPC_SlowEng.internal_err_cnt & 0x00FFu) << 8u)
              | (uint16_t)(DPC_SlowEng.internal_rec_cnt  & 0x00FFu);

    if ((inpb(RS422_State) & RS422_STATE_MASK) == RS422_IDLE)
    {
        outpw(RS422_CTRL, 0x0001u);
    }
    //printf("DiagDump: diag=0x%04X cnt=%u\n", diag_word, DiagCycCnt);
    (void)diag_word;
}
#endif

/* ---- 内部指令处理（主任务调用） ---- */
//读BUF_INTERNAL[0]和BUF_INTERNAL[1]做一致性比较
//
void fnInternalHandling(void)
{
    uint8_t  state;
    uint16_t rec_cnt_low;

    //查RS422总线忙不忙
    state = inpb(RS422_State) & RS422_STATE_MASK;

    /* 总线空闲才处理，否则跳过 */
    //
    //
    //
    if (RS422_IDLE == state)
    {
        /* 两个缓冲字做完整性比较：要求连续读值一致 */
        if (BUF_INTERNAL[0] == BUF_INTERNAL[1])
        {
            //一致，说明接收数据完整
            DPC_SlowEng.internal_rec_cnt =
                DPC_SlowEng.internal_rec_cnt + 1u;

            BusHealthCnt.total_msg_cnt++;
            if (BusHealthCnt.consecutive_err > 0u)
            {
                BusHealthCnt.consecutive_err = 0u;
            }

            /* 后续：执行内部指令对应的操作 */
            LastMsgCode = BUF_INTERNAL[0];
        }
        else
        {
            //不一致，有误码或时序问题
            DPC_SlowEng.internal_err_cnt =
                DPC_SlowEng.internal_err_cnt + 1u;

            rec_cnt_low  = DPC_SlowEng.internal_rec_cnt & ERR_MASK_REC_CNT;
            internal_return = ERR_BASE_INTERNAL | rec_cnt_low;

            ReportBusError(internal_return);
        }
    }
    else
    {
        //RS422总线忙，等下次再处理
        DPC_SlowEng.bus_status = 0x01u;
    }
}

/* ---- 遥测数据帧处理（主任务调用） ---- */
//读子地址9的数据做帧同步字校验
//
void fnTelemetryHandling(void)
{
    uint16_t tm_buf[TM_FRAME_LENGTH];
    uint16_t sync_word;
    uint16_t calc_crc;
    uint16_t recv_crc;

    /* 批量读遥测帧 */
    AceReadBurst(SUBADDRESS9_RECEIVE_PTR, tm_buf, TM_FRAME_LENGTH);

    sync_word = tm_buf[0];
    if (sync_word != TM_FRAME_SYNC_WORD)
    {
        BusHealthCnt.sync_lost_cnt++;
        ReportBusError(ERR_CODE_SYNC_LOST);
        return;
    }

    //校验帧CRC（最后一个字是CRC字）
    recv_crc = tm_buf[TM_FRAME_LENGTH - 1u];
    calc_crc = CalcFrameCRC(tm_buf, (uint8_t)(TM_FRAME_LENGTH - 1u));

    if (calc_crc != recv_crc)
    {
        BusHealthCnt.crc_err_cnt++;
        ReportBusError(ERR_CODE_FRAME_CRC);
        return;
    }

    /* 帧合法，累计遥测计数 */
    TmUplinkCnt++;
    DPC_SlowEng.telemetry_cnt++;
}

/* ---- 总线状态帧处理（主任务调用） ---- */
//
//
void fnStatusHandling(void)
{
    uint16_t status_word;

    status_word = inpw(ACE_STATUS_REG);

    if (status_word & 0x8000u)
    {
        /* 标志位设置 */
        DPC_SlowEng.bus_status |= 0x80u;
    }
    else
    {
        /* 位操作 */
        DPC_SlowEng.bus_status &= 0x7Fu;
    }
}

/* ---- 总线健康周期监测（主任务调用） ---- */
//定期算通信质量，更新健康等级
//
void fnBusHealthMonitor(void)
{
    uint16_t err_rate;

    HealthCnt++;
    if (HealthCnt < HEALTH_REPORT_INTERVAL)
    {
        return;
    }
    HealthCnt = 0u;

    //算错误率（万分比）
    if (BusHealthCnt.total_msg_cnt > 0u)
    {
        err_rate = (uint16_t)((uint32_t)BusHealthCnt.crc_err_cnt * 10000u
                              / (uint32_t)BusHealthCnt.total_msg_cnt);
    }
    else
    {
        err_rate = 0u;
    }

    /* 根据错误率更新健康等级 */
    if (err_rate == 0u)
    {
        BusHealthCnt.health_level = 3u;   //优
    }
    else if (err_rate < 100u)
    {
        BusHealthCnt.health_level = 2u;   //良
    }
    else if (err_rate < 500u)
    {
        BusHealthCnt.health_level = 1u;   //差
    }
    else
    {
        BusHealthCnt.health_level = 0u;   //故障
        /* 状态机转移 */
        AceState = ACE_STATE_FAULT;
    }

    //重置统计窗口
    BusHealthCnt.total_msg_cnt = 0u;
    BusHealthCnt.crc_err_cnt   = 0u;
    BusHealthCnt.sync_lost_cnt = 0u;

    /* 向地面下行链路发本周期状态摘要 */
    AceSendStatusToGround();
}

/* ---- ACE状态摘要查询 ---- */
//高8位: 状态机+健康等级；低8位: 截断的故障计数
//
static uint16_t AceGetStatusSummary(void)
{
    uint8_t  high_byte;
    uint8_t  low_byte;
    uint16_t summary;

    high_byte = (uint8_t)(((uint8_t)AceState & 0x0Fu) << 4u)
              | (uint8_t)(BusHealthCnt.health_level & 0x0Fu);

    /* 设置外设参数 */
    low_byte  = (uint8_t)(AceFaultCnt & 0x00FFu);

    summary = (uint16_t)((uint16_t)high_byte << 8u) | (uint16_t)low_byte;
    return summary;
}

/* ---- 给RS422发ACE状态摘要（总线空闲时才发） ---- */
//
//
static void AceSendStatusToGround(void)
{
    uint16_t summary;
    uint8_t  bus_state;

    bus_state = inpb(RS422_State) & RS422_STATE_MASK;
    if (bus_state != RS422_IDLE)
    {
        return;
    }

    summary = AceGetStatusSummary();
    outpw(RS422_DATA, summary);
}

/* ---- ACE 65170 中断服务程序（外部中断0） ---- */
//1553B消息收完触发，解析消息类型更新缓冲区
//
void ace_int0(void) /* interrupt 0 */
{
    uint16_t int_status;
    uint16_t msg_code;

    //读中断状态寄存器，判断中断类型
    int_status = inpw(ACE_INT_STATUS_REG);

    /* 获取当前接收到的消息码（存放在共享内存固定偏移处） */
    msg_code = AceReadMemWord(0x0000u);

    //根据消息码分发处理
    //
    //
    //
    //
    //
    switch (msg_code)
    {
        case MSG_CODE_INTERNAL_CMD:
            //读子地址8，接收2字内部指令数据，分两步写共享缓冲区
            BUF_INTERNAL[0] = inpw(Ace_Mem_Base + SUBADDRESS8_RECEIVE_PTR * 2u);
            BUF_INTERNAL[1] = inpw(Ace_Mem_Base + SUBADDRESS8_RECEIVE_PTR * 2u + 2u);
            break;

        case MSG_CODE_TELEMETRY:
            DPC_SlowEng.telemetry_cnt++;
            break;

        case MSG_CODE_STATUS:
            DPC_SlowEng.bus_status = (uint8_t)(AceReadMemWord(0x0002u) & 0x00FFu);
            break;

        case MSG_CODE_HEALTH:
            BusHealthCnt.total_msg_cnt++;
            break;

        default:
            MsgDropCnt++;
            break;
    }
    //清中断标志
    outpw(ACE_INT_STATUS_REG, int_status);
}
