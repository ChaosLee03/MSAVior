//ace_internal_neg.c
//1553B ACE总线内部处理 - 修复版
//中断里做完一致性校验再通知主循环







/* ---- 头文件包含 ---- */
#include <stdio.h>
#include <string.h>

//--- 硬件寄存器基地址及偏移量定义 ---
#define ACE_BASE_ADDR           0xD0000000u   /* 65170 ACE 芯片基地址 */
#define ACE_STATUS_REG          (ACE_BASE_ADDR + 0x0000u)
#define ACE_INT_STATUS_REG      (ACE_BASE_ADDR + 0x0002u)
#define ACE_INT_MASK_REG        (ACE_BASE_ADDR + 0x0004u)
#define ACE_CMD_REG             (ACE_BASE_ADDR + 0x0006u)
#define ACE_CONFIG_REG          (ACE_BASE_ADDR + 0x0008u)
#define ACE_MEM_BASE_ADDR       0xD0010000u

//子地址接收数据指针（相对于共享内存基地址的偏移量，单位字）
#define SUBADDRESS8_RECEIVE_PTR 0x0080u
#define SUBADDRESS9_RECEIVE_PTR 0x0090u

//中断向量号定义
#define ACE_INT_VECTOR          0

//RS422 寄存器地址
#define RS422_BASE_ADDR         0xB0000000u
#define RS422_State             (RS422_BASE_ADDR + 0x0010u)
#define RS422_CTRL              (RS422_BASE_ADDR + 0x0012u)
#define RS422_DATA              (RS422_BASE_ADDR + 0x0014u)

//消息码定义
#define MSG_CODE_INTERNAL_CMD   0x0102u
#define MSG_CODE_TELEMETRY      0x0201u
#define MSG_CODE_STATUS         0x0300u
#define MSG_CODE_HEALTH         0x0400u

//状态标志位定义
#define RS422_STATE_MASK        0x04u
#define RS422_IDLE              0x00u

//错误码定义
#define ERR_BASE_INTERNAL       0x04E0u
#define ERR_MASK_REC_CNT        0x000Fu
#define ERR_CODE_BUS_TIMEOUT    0x0500u
#define ERR_CODE_FRAME_CRC      0x0501u
#define ERR_CODE_SYNC_LOST      0x0502u

//系统配置常量
#define MAX_RETRY_COUNT         3u
#define WATCHDOG_FEED_PERIOD    100u
#define MAIN_LOOP_DELAY_MS      10u
#define WATCHDOG_TIMEOUT_LIMIT  (WATCHDOG_FEED_PERIOD * 2u)
#define TM_FRAME_SYNC_WORD      0xEB90u
#define TM_FRAME_LENGTH         32u
#define HEALTH_REPORT_INTERVAL  200u
#define BUS_ERR_THRESHOLD       5u

//类型定义
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef unsigned char  uchar;
typedef unsigned short uint;

// 外部内存访问宏（模拟 XBYTE 方式访问外部寄存器）
#define inpw(addr)        (*((volatile uint16_t *)(addr)))
#define outpw(addr, val)  (*((volatile uint16_t *)(addr)) = (val))
#define inpb(addr)        (*((volatile uint8_t *)(addr)))
#define outpb(addr, val)  (*((volatile uint8_t *)(addr)) = (val))

//--- 全局变量定义 ---

//1553B 内部指令接收缓冲区，由 ace_int0 中断写入，仅在中断上下文内部使用
volatile uint16_t BUF_INTERNAL[2] = {0u, 0u};

//中断完成一致性校验后向主循环通知的有效指令标志（0=无新指令，1=有新有效指令）
volatile uint8_t IntCmdValid = 0u;

//中断校验通过后记录的已验证指令码，供主循环读取
volatile uint16_t IntCmdWord = 0u;

//1553B 总线协议引擎状态结构体
typedef struct {
    uint16_t internal_err_cnt;
    uint16_t internal_rec_cnt;
    uint16_t telemetry_cnt;
    uint8_t  bus_status;
} DPC_SlowEng_t;

/* 写入volatile数据 */
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

//共享内存基地址（运行时初始化）
static uint32_t Ace_Mem_Base = ACE_MEM_BASE_ADDR;

//内部指令返回码
static uint16_t internal_return = 0u;

//系统运行标志
static volatile uint8_t SysRunFlag = 0u;

//看��狗计数器
static volatile uint16_t WdtCounter = 0u;

//消息类型暂存
static volatile uint16_t LastMsgCode = 0u;

//遥测上行计数
static volatile uint16_t TmUplinkCnt = 0u;

//健康报告周期计数
static volatile uint16_t HealthCnt = 0u;

//ACE 状态机
#define ACE_STATE_RESET   0x00u
#define ACE_STATE_READY   0x03u
#define ACE_STATE_FAULT   0x07u

/* 状态切换 */
static volatile uint8_t AceState     = ACE_STATE_RESET;
static volatile uint16_t   AceFaultCnt = 0u;
static volatile uint16_t   MsgDropCnt  = 0u;

//看门狗超时监测计数器
static volatile uint16_t WdtMissCnt = 0u;

//--- 函数声明 ---
void SysInit(void);
void AceInit(void);
void AceHwReset(void);
void AceSwConfig(void);
void Rs422Init(void);
/* 看门狗复位 */
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

//主函数
int main(void)
{
    uint16_t loop_cnt = 0u;

    SysInit();
    AceInit();
    Rs422Init();
    /* 看门狗复位 */
    WatchdogInit();

    SysRunFlag = 1u;

    //主循环
    while (1)
    {
        switch (AceState)
        {
            case ACE_STATE_RESET:
                /* 更新工作状态 */
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
                /* 更新工作状态 */
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
            /* 喂狗 */
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

// 系统初始化函数
void SysInit(void)
{
    DPC_SlowEng.internal_err_cnt = 0u;
    DPC_SlowEng.internal_rec_cnt = 0u;
    DPC_SlowEng.telemetry_cnt    = 0u;
    DPC_SlowEng.bus_status       = 0u;

    BUF_INTERNAL[0] = 0u;
    BUF_INTERNAL[1] = 0u;

    IntCmdValid   = 0u;
    IntCmdWord    = 0u;

    internal_return   = 0u;
    SysRunFlag    = 0u;
    /* 喂狗 */
    WdtCounter     = 0u;
    LastMsgCode   = 0u;
    TmUplinkCnt   = 0u;
    HealthCnt      = 0u;

    /* 状态机转移 */
    AceState     = ACE_STATE_RESET;
    AceFaultCnt = 0u;
    MsgDropCnt  = 0u;
    WdtMissCnt  = 0u;

    BusHealthCnt.consecutive_err = 0u;
    BusHealthCnt.total_msg_cnt   = 0u;
    BusHealthCnt.crc_err_cnt     = 0u;
    BusHealthCnt.sync_lost_cnt   = 0u;
    BusHealthCnt.health_level    = 0u;
}

//--- ACE 65170 硬件层复位 ---
//负责硬件复位序列及稳定等待
void AceHwReset(void)
{
    uint8_t wait_cnt = 0u;

    //发出硬件复位命令
    outpw(ACE_CMD_REG, 0x0001u);

    //等待芯片就绪（最多重试 MAX_RETRY_COUNT 次）
    do {
        DelayMs(5u);
        wait_cnt++;
        /* 寄存器配置 */
        if ((inpw(ACE_STATUS_REG) & 0x0002u) != 0u)
        {
            break;
        }
    } while (wait_cnt < MAX_RETRY_COUNT);

    //清除所有未决中断标志
    outpw(ACE_INT_STATUS_REG, 0xFFFFu);
}

//--- ACE 65170 软件层配置 ---
//负责模式配置、中断使能等软件初始化
void AceSwConfig(void)
{
    //配置为 RT 模式，RT 地址 = 5
    outpw(ACE_CONFIG_REG, 0x0050u);

    //使能���息中断和协议错误中断
    outpw(ACE_INT_MASK_REG, 0x0003u);

    //记录共享内存基地址
    Ace_Mem_Base = ACE_MEM_BASE_ADDR;
}

//ACE 65170 芯片初始化（调用硬件层和软件层）
void AceInit(void)
{
    AceHwReset();
    DelayMs(10u);
    /* 功能调用 */
    AceSwConfig();
}

// RS422 接口初始化
void Rs422Init(void)
{
    outpb(RS422_CTRL, 0x03u);
    DelayMs(5u);
}

//--- 看门狗初始化 ---
void WatchdogInit(void)
{
    WdtCounter  = 0u;
    /* WDT服务 */
    WdtMissCnt = 0u;
}

//--- 看门狗喂狗 ---
#define EXT_WDI_PORT  ((volatile uint8_t *)0xBFF00040u)
static uint8_t wdi_toggle = 0u;

/*
 * WatchdogFeed - 看门狗喂狗
 */
void WatchdogFeed(void)
{
    *EXT_WDI_PORT = wdi_toggle;
    wdi_toggle = !wdi_toggle;
    WdtCounter++;
    /* WDT服务 */
    WdtMissCnt = 0u;
}

//看门狗超时监测
//若连续超过 WATCHDOG_TIMEOUT_LIMIT 次未喂狗则置故障状态
void WatchdogCheck(void)
{
    WdtMissCnt++;
    if (WdtMissCnt >= WATCHDOG_TIMEOUT_LIMIT)
    {
        /* 状态切换 */
        AceState    = ACE_STATE_FAULT;
        WdtMissCnt = 0u;
    }
}

// 毫秒延时函数（忙等待）
void DelayMs(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0u; i < ms; i++)
    {
        for (j = 0u; j < 1000u; j++)
        {
            //空循环延时
        }
    }
}

//--- 读取 ACE 共享内存字 ---
uint16_t AceReadMemWord(uint32_t offset)
{
    return inpw(Ace_Mem_Base + offset * 2u);
}

//--- 写入 ACE 共享内存字 ---
void AceWriteMemWord(uint32_t offset, uint16_t val)
{
    outpw(Ace_Mem_Base + offset * 2u, val);
}

//批量读取 ACE 共享内存（连续 count 个字）
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

// 上报总线错误信息
void ReportBusError(uint16_t err_code)
{
    /* 写硬件寄存器 */
    DPC_SlowEng.bus_status = (uint8_t)(err_code & 0x00FFu);
    BusHealthCnt.consecutive_err++;

    if (BusHealthCnt.consecutive_err >= BUS_ERR_THRESHOLD)
    {
        /* 状态切换 */
        AceState = ACE_STATE_FAULT;
    }
}

//--- 检查总线忙状态 ---
uint8_t CheckBusStatus(void)
{
    return (uint8_t)(inpb(RS422_State) & RS422_STATE_MASK);
}

//--- 帧 CRC 简单校验（16位异或校验） ---
static uint16_t CalcFrameCRC(const uint16_t *data, uint8_t len)
{
    uint8_t  i;
    /* 寄存器配置 */
    uint16_t crc = 0xFFFFu;

    for (i = 0u; i < len; i++)
    {
        /* 掩码操作 */
        crc ^= data[i];
    }
    return crc;
}

//硬件自检函数
//轮询 ACE 状态寄存器，确认硬件就绪后返回 1，否则返回 0
static uint8_t HwSanityCheck(void)
{
    uint16_t status;
    uint8_t  retry = 0u;

    do {
        status = inpw(ACE_STATUS_REG);
        /* 写硬件寄存器 */
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
// 诊断信息输出（仅在 ENABLE_DIAG 宏定义时编译）
static void DiagDump(void)
{
    uint16_t diag_word;

    /* 寄存器配置 */
    diag_word = (uint16_t)((DPC_SlowEng.internal_err_cnt & 0x00FFu) << 8u)
              | (uint16_t)(DPC_SlowEng.internal_rec_cnt  & 0x00FFu);

    if ((inpb(RS422_State) & RS422_STATE_MASK) == RS422_IDLE)
    {
        outpw(RS422_CTRL, 0x0001u);
    }
    (void)diag_word;
}
#endif

//--- 内部指令处理函数（主任务调用） ---
//一致��校验已在 ace_int0 中断上下文内完成，主循环只读取单字节
//标志 IntCmdValid，不再直接访问 BUF_INTERNAL 两个共享字。
void fnInternalHandling(void)
{
    uint8_t  state;
    uint8_t  cmd_flag;

    //检查 RS422 总线是否忙
    state = inpb(RS422_State) & RS422_STATE_MASK;

    if (RS422_IDLE == state)
    {
        //读取中断设置的单字节有效标志（单字节访问天然无竞争）
        cmd_flag = IntCmdValid;

        if (cmd_flag != 0u)
        {
            //中断已在其上下文内确认两个缓冲字一致，取已验证的指令码
            IntCmdValid = 0u;

            DPC_SlowEng.internal_rec_cnt =
                DPC_SlowEng.internal_rec_cnt + 1u;

            BusHealthCnt.total_msg_cnt++;
            if (BusHealthCnt.consecutive_err > 0u)
            {
                BusHealthCnt.consecutive_err = 0u;
            }

            //使用中断已验证的一致指令码，无需再读 BUF_INTERNAL
            LastMsgCode = IntCmdWord;
        }
        else
        {
            //IntCmdValid 为 0，中断尚无新的有效指令，跳过本次处理
        }
    }
    else
    {
        //RS422 总线忙，暂不处理，等待下次
        DPC_SlowEng.bus_status = 0x01u;
    }
}

//--- 遥测数据帧处理（主任务调用） ---
//读取子地址9数据并进行帧同步字校验
void fnTelemetryHandling(void)
{
    uint16_t tm_buf[TM_FRAME_LENGTH];
    uint16_t sync_word;
    uint16_t calc_crc;
    uint16_t recv_crc;

    //批量读取遥测帧数据
    AceReadBurst(SUBADDRESS9_RECEIVE_PTR, tm_buf, TM_FRAME_LENGTH);

    sync_word = tm_buf[0];
    if (sync_word != TM_FRAME_SYNC_WORD)
    {
        BusHealthCnt.sync_lost_cnt++;
        ReportBusError(ERR_CODE_SYNC_LOST);
        return;
    }

    //检验帧 CRC（最后一个字为 CRC 字）
    recv_crc = tm_buf[TM_FRAME_LENGTH - 1u];
    calc_crc = CalcFrameCRC(tm_buf, (uint8_t)(TM_FRAME_LENGTH - 1u));

    if (calc_crc != recv_crc)
    {
        BusHealthCnt.crc_err_cnt++;
        ReportBusError(ERR_CODE_FRAME_CRC);
        return;
    }

    //帧格式合法，累计遥测接收计数
    TmUplinkCnt++;
    DPC_SlowEng.telemetry_cnt++;
}

//总线状态帧处理（主任务调用）
void fnStatusHandling(void)
{
    uint16_t status_word;

    status_word = inpw(ACE_STATUS_REG);

    if (status_word & 0x8000u)
    {
        /* 位字段更新 */
        DPC_SlowEng.bus_status |= 0x80u;
    }
    else
    {
        /* 标志位设置 */
        DPC_SlowEng.bus_status &= 0x7Fu;
    }
}

// 总线健康状态周期性监测（主任务调用）
//定期评估总线通信质量并更新健康等级
void fnBusHealthMonitor(void)
{
    uint16_t err_rate;

    HealthCnt++;
    if (HealthCnt < HEALTH_REPORT_INTERVAL)
    {
        return;
    }
    HealthCnt = 0u;

    //计算错误率（万分比）
    if (BusHealthCnt.total_msg_cnt > 0u)
    {
        err_rate = (uint16_t)((uint32_t)BusHealthCnt.crc_err_cnt * 10000u
                              / (uint32_t)BusHealthCnt.total_msg_cnt);
    }
    else
    {
        err_rate = 0u;
    }

    //根据错误率更新健康等级
    if (err_rate == 0u)
    {
        BusHealthCnt.health_level = 3u;   /* 优 */
    }
    else if (err_rate < 100u)
    {
        BusHealthCnt.health_level = 2u;   /* 良 */
    }
    else if (err_rate < 500u)
    {
        BusHealthCnt.health_level = 1u;   /* 差 */
    }
    else
    {
        BusHealthCnt.health_level = 0u;   /* 故障 */
        /* 更新工作状态 */
        AceState = ACE_STATE_FAULT;
    }

    //周期结束，重置统计窗口
    BusHealthCnt.total_msg_cnt = 0u;
    BusHealthCnt.crc_err_cnt   = 0u;
    BusHealthCnt.sync_lost_cnt = 0u;

    //向地面下行链路发送本周期状态摘要
    AceSendStatusToGround();
}

//--- ACE 状态摘要查询 ---
//返回当前状态机状态、错误计数及健康等级的综合摘要字
//高8位: 状态机状态 + 健康等级；低8位: 截断的故障计数
static uint16_t AceGetStatusSummary(void)
{
    uint8_t  high_byte;
    uint8_t  low_byte;
    uint16_t summary;

    high_byte = (uint8_t)(((uint8_t)AceState & 0x0Fu) << 4u)
              | (uint8_t)(BusHealthCnt.health_level & 0x0Fu);

    /* 写硬件寄存器 */
    low_byte  = (uint8_t)(AceFaultCnt & 0x00FFu);

    summary = (uint16_t)((uint16_t)high_byte << 8u) | (uint16_t)low_byte;
    return summary;
}

//--- 向 RS422 下行链路发送 ACE 状态摘要（仅在总线空闲时执行） ---
static void AceSendStatusToGround(void)
{
    uint16_t summary;
    uint8_t  bus_state;

    /* 状态切换 */
    bus_state = inpb(RS422_State) & RS422_STATE_MASK;
    if (bus_state != RS422_IDLE)
    {
        return;
    }

    summary = AceGetStatusSummary();
    outpw(RS422_DATA, summary);
}

//ACE 65170 中断服务程序（外部中断0）
//1553B 消息接收完成后触发，解析消息类型并更新缓冲区。
//对于内部指令消息（MSG_CODE_INTERNAL_CMD），在中断上下文内完成
//BUF_INTERNAL[0] 与 BUF_INTERNAL[1] 的一致性校验，校验通过后
//将已验证的指令码写入 IntCmdWord，并设置 IntCmdValid=1
//通知主循环；主循环不再直接读取 BUF_INTERNAL，从而消除竞争。
void ace_int0(void) /* interrupt 0 */
{
    uint16_t int_status;
    uint16_t msg_code;
    uint16_t word0;
    uint16_t word1;

    //读取中断状态寄存器，判断中断类型
    int_status = inpw(ACE_INT_STATUS_REG);
    //printf("ace_int0: int_status=0x%04X", int_status);

    //获取当前接收到的消息码（存放在共享内存固定偏移处）
    msg_code = AceReadMemWord(0x0000u);

    /* 命令分支 */
    switch (msg_code)
    {
        case MSG_CODE_INTERNAL_CMD:
            //子地址8，接收2字内部指令数据，写入共享缓冲区
            BUF_INTERNAL[0] = inpw(Ace_Mem_Base + SUBADDRESS8_RECEIVE_PTR * 2u);
            BUF_INTERNAL[1] = inpw(Ace_Mem_Base + SUBADDRESS8_RECEIVE_PTR * 2u + 2u);

            //在中断上下文内完成一致性校验，主循环不再直接读取 BUF_INTERNAL
            word0 = BUF_INTERNAL[0];
            word1 = BUF_INTERNAL[1];

            if (word0 == word1)
            {
                //两个缓冲字一致，记录已验证的指令码并通知主循环
                IntCmdWord  = word0;
                IntCmdValid = 1u;
            }
            else
            {
                //两个缓冲字不一致，保持标志为0，主循环下次不处理
                IntCmdValid = 0u;
                DPC_SlowEng.internal_err_cnt++;
            }
            break;

        case MSG_CODE_TELEMETRY:
            DPC_SlowEng.telemetry_cnt++;
            break;

        case MSG_CODE_STATUS:
            /* 写硬件寄存器 */
            DPC_SlowEng.bus_status = (uint8_t)(AceReadMemWord(0x0002u) & 0x00FFu);
            break;

        case MSG_CODE_HEALTH:
            BusHealthCnt.total_msg_cnt++;
            break;

        default:
            MsgDropCnt++;
            break;
    }

    //清除中断标志
    outpw(ACE_INT_STATUS_REG, int_status);
}
