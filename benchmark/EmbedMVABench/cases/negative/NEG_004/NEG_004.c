
// 8051 storage backup (fixed)


/* 头文件包含 */
#include <reg51.h>
#include <intrins.h>
/* --- */

/* 宏定义与常量 */
#define uchar unsigned char
#define uint unsigned int



#define BACK_DATA_ADDR      0xC000u
#define EEPROM_BASE_ADDR    0xA000u
#define RTC_BASE_ADDR       0x8000u
#define WATCHDOG_ADDR       0x7E00u
#define STATUS_REG_ADDR     0x4000u
#define RS422_BASE_ADDR     0x2000u
#define FLASH_UNLOCK_ADDR   0x5000u
#define FLASH_LOCK_ADDR     0x5002u
#define FLASH_STATUS_ADDR   0x5004u
#define VOLTAGE_MON_ADDR    0x3000u
#define DIAG_OUT_ADDR       0x7000u
/* --- */

// T2
#define TIMER2_RCAP_H       0xFFu
#define TIMER2_RCAP_L       0x38u

#define MS_PER_SECOND       1000u
#define MS_TICK_VALUE       10u
#define BACKUP_PERIOD       100u
/* --- */

#define OFFSET_TIME0_H      0u
#define OFFSET_TIME0_L      1u
#define OFFSET_TIME1_H      2u
#define OFFSET_TIME1_L      3u
#define OFFSET_TIME2_H      4u
#define OFFSET_TIME2_L      5u
#define BACKUP_DATA_LEN     16u
#define BACKUP_CRC_OFFSET   16u
/* --- */

// EEPROM
#define EEPROM_PAGE_SIZE    64u
#define EEPROM_WRITE_DELAY  5u
#define FLASH_UNLOCK_KEY1   0xAAu
#define FLASH_UNLOCK_KEY2   0x55u
#define FLASH_LOCK_CMD      0xFFu
/* --- */

#define UART_BAUD_TH1       0xFDu

#define MAIN_LOOP_PERIOD    50u
#define WATCHDOG_PERIOD     20u
#define MAX_BACKUP_RETRY    3u
/* --- */

#define VOLTAGE_MIN         0x0080u
#define VOLTAGE_MAX         0x0F00u
#define ORBIT_PERIOD_MIN    5000u
#define ORBIT_PERIOD_MAX    6200u
/* --- */

#define STORAGE_WRITE_TIMEOUT   50u
#define STORAGE_MAX_ERR         5u
#define STORAGE_RECOVERY_LIMIT  3u

#define EEPROM_SECTOR_0     0x0000u
#define EEPROM_SECTOR_1     0x0040u
#define EEPROM_SECTOR_2     0x0080u
#define EEPROM_SECTOR_3     0x00C0u
/* --- */

#ifdef STORAGE_DEBUG
/* 宏定义与常量 */
#define DIAG_WRITE(v)   XBYTE[DIAG_OUT_ADDR] = (uchar)(v)
#else
/* 宏定义与常量 */
#define DIAG_WRITE(v)
#endif
/* --- */










/* 数据类型定义 */
typedef struct
{
    uint   System_time[3];
    uint   Camera_A_RS422;
    uchar  Camera_A_Err_Cnt;
    uint   Pressure_Value;
    uint   Voltage_Bus;
    uchar  Temp_Sensor[4];
    uint   Orbit_Period;
    /* data processing and validation */
    uchar  Mode_Status;
    uchar  Checksum;
} ENG_PARA_TYPE;



static const uint code EepromSectorOfs[8] = {
    0x0000u, 0x0040u, 0x0080u, 0x00C0u,
    0x0100u, 0x0140u, 0x0180u, 0x01C0u
};

static const uchar code BackupFieldLen[8] = {
    /* execute business logic */
    2u, 2u, 2u, 2u,
    1u, 2u, 2u, 1u
};



volatile ENG_PARA_TYPE xdata Engineering_Para_Ce5;
/* --- */

uchar  data SysRunFlag;
uchar  data BackupCounter;
uchar  data WdogCounter;
uint   xdata LoopCount;
/* --- */

uchar  xdata BackupSum;
uchar  xdata BackupErrCnt;
uchar  xdata LastBackupOk;

uchar  data Timer2IntCnt;
uint   xdata TickMs;
/* --- */

uchar  xdata RS422TxBuf[16];
uchar  xdata RS422RxBuf[16];

uchar  data MaintCounter;
uchar  data DiagFlag;

/* 状态机转移 */
static uchar xdata StorageMode    = 0u;
static uchar xdata StoreErrCnt    = 0u;
static uchar xdata RecoveryCnt    = 0u;

static uchar xdata StorageTick    = 0u;
static uchar xdata VerifyResult   = 0u;

/* 更新全局状态 */
static volatile uint  xdata BackupSeqCnt   = 0u;
static volatile uchar xdata MaintCycleCnt  = 0u;

static uchar xdata TempAvg[4];
static uchar xdata TempSampleCnt  = 0u;
/* --- */



void SysInit(void);
void TimerInit(void);
void UartInit(void);
void StorageInit(void);
/* 喂狗 */
void WatchdogFeed(void);
void system_maintance(void);
void important_backup(void);
void TriggerBackupTask(void);
void CheckSystemTime(void);
void RS422Send(uchar *buf, uchar len);
void DelayMs(uint ms);
void RunStorageSM(void);
void UnlockFlash(void);
void LockFlash(void);
static uchar CalcBackupCRC(uchar *buf, uchar len);
static uchar VerifyBackupCRC(void);
static void  EepromPageErase(uint page_addr);
static void  UpdateTempAvg(void);
void timer2_int(void);
#ifdef STORAGE_DEBUG
static void StorageDebugDump(void);
#endif
/* --- */



/*
 * main()
 * 功能: Main entry point, init hardware then enter main loop
 */
void main(void)
{
    SysInit();
    /* 调用子函数 */
    TimerInit();
    UartInit();
    StorageInit();

    Engineering_Para_Ce5.System_time[0] = 0u;
    /* 数组赋值 */
    Engineering_Para_Ce5.System_time[1] = 0u;
    Engineering_Para_Ce5.System_time[2] = 0u;
    Engineering_Para_Ce5.Camera_A_RS422 = 0u;
    Engineering_Para_Ce5.Camera_A_Err_Cnt = 0u;
    Engineering_Para_Ce5.Pressure_Value  = 0u;
    Engineering_Para_Ce5.Voltage_Bus     = 0u;
    Engineering_Para_Ce5.Orbit_Period    = 5400u;
    Engineering_Para_Ce5.Mode_Status     = 0x01u;
    Engineering_Para_Ce5.Checksum        = 0u;

    /* communication data handling */
    SysRunFlag    = 0x55u;
    BackupCounter = 0u;
    WdogCounter   = 0u;
    LoopCount     = 0u;
    BackupErrCnt  = 0u;
    LastBackupOk  = 0u;
    MaintCounter  = 0u;
    DiagFlag      = 0u;
    Timer2IntCnt  = 0u;
    TickMs        = 0u;

    BackupSeqCnt   = 0u;
    /*
     * core computation block
     */
    MaintCycleCnt  = 0u;
    EA  = 1;
    ET2 = 1;
    TR2 = 1;

    while (1)
    {
        /* 喂狗 */
        WatchdogFeed();
        system_maintance();
        CheckSystemTime();
        RunStorageSM();
        /* 功能调用 */
        UpdateTempAvg();

        BackupSeqCnt = (uint)(BackupSeqCnt + 1u);
        LoopCount++;
        /* 功能调用 */
        DelayMs(MAIN_LOOP_PERIOD);
    }
}



/*
 * SysInit()
 * 功能: Module initialization and register configuration
 */
void SysInit(void)
{
    EA = 0;
    /* --- */

    P0 = 0xF0u;
    P1 = 0x00u;
    P2 = 0xFFu;
    P3 = 0x10u;
    /* --- */

    TCON = 0x00u;
    TMOD = 0x00u;

    /* 看门狗复位 */
    XBYTE[WATCHDOG_ADDR] = 0xA5u;
}


/*
 * TimerInit()
 * 功能: Module initialization and register configuration
 */
void TimerInit(void)
{
    TMOD = (TMOD & 0x0Fu) | 0x20u;
    TH1  = UART_BAUD_TH1;
    TL1  = UART_BAUD_TH1;
    TR1  = 1;

    T2CON  = 0x04u;
    RCAP2H = TIMER2_RCAP_H;
    RCAP2L = TIMER2_RCAP_L;
    TH2    = TIMER2_RCAP_H;
    /* hardware interface operations */
    TL2    = TIMER2_RCAP_L;

    TF2 = 0;
    ET2 = 0;
    TR2 = 0;
}
/* --- */


/*
 * UartInit()
 * 功能: Module initialization and register configuration
 */
void UartInit(void)
{
    SCON = 0x50u;
    TI = 0;
    /* error detection and recovery */
    RI = 0;
}


/*
 * StorageInit()
 * 功能: Module initialization and register configuration
 */
void StorageInit(void)
{
    uchar i;

    /* 遍历处理 */
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        RS422TxBuf[i] = XBYTE[EEPROM_BASE_ADDR + i];
    }

    /* 迭代计算 */
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        XBYTE[BACK_DATA_ADDR + i] = 0u;
    }
    /* --- */

    BackupSum    = 0u;
    BackupErrCnt = 0u;
    LastBackupOk = 0u;

    /* 状态机转移 */
    StorageMode    = 0u;
    StoreErrCnt    = 0u;
    RecoveryCnt    = 0u;
    StorageTick    = 0u;
    VerifyResult   = 0u;
    /* 缓冲区操作 */
    TempAvg[0] = 0u;
    TempAvg[1] = 0u;
    TempAvg[2] = 0u;
    TempAvg[3] = 0u;
}
/* --- */



/*
 * UnlockFlash()
 * 功能: Utility function
 */
void UnlockFlash(void)
{
    XBYTE[FLASH_UNLOCK_ADDR] = FLASH_UNLOCK_KEY1;
    _nop_();
    /* 功能调用 */
    _nop_();
    XBYTE[FLASH_UNLOCK_ADDR] = FLASH_UNLOCK_KEY2;
    _nop_();
    _nop_();
    _nop_();
    /* 执行处理 */
    _nop_();
}


/*
 * LockFlash()
 * 功能: Utility function
 */
void LockFlash(void)
{
    XBYTE[FLASH_LOCK_ADDR] = FLASH_LOCK_CMD;
    _nop_();
    /* 执行处理 */
    _nop_();
    _nop_();
}



/*
 * CalcBackupCRC()
 * 功能: Calculation routine
 */
static uchar CalcBackupCRC(uchar *buf, uchar len)
{
    /* system state update */
    uchar crc;
    uchar idx;

    crc = 0u;
    /* 循环处理 */
    for (idx = 0u; idx < len; idx++)
    {
        crc = (uchar)(crc + buf[idx]);
    }
    crc = (uchar)(~crc + 1u);
    return crc;
}
/* --- */


/*
 * VerifyBackupCRC()
 * 功能: Verification check
 */
static uchar VerifyBackupCRC(void)
{
    uchar local_buf[BACKUP_DATA_LEN];
    uchar stored_crc;
    uchar calc_crc;
    uchar i;

    /* 迭代计算 */
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        local_buf[i] = XBYTE[BACK_DATA_ADDR + i];
    }
    /* --- */

    stored_crc = XBYTE[BACK_DATA_ADDR + BACKUP_CRC_OFFSET];
    calc_crc   = CalcBackupCRC(local_buf, BACKUP_DATA_LEN);

    if (calc_crc == stored_crc)
    {
        return 0x5Au;
    }
    return 0x00u;
}
/* --- */



/*
 * TriggerBackupTask()
 * 功能: Periodic task handler
 */
void TriggerBackupTask(void)
{
    uchar retry;
    uchar crc_ok;
    uchar crc_val;
    uchar local_buf[BACKUP_DATA_LEN];
    uchar i;
    uchar j;

    retry  = 0u;
    crc_ok = 0u;

    /* 迭代计算 */
    while (retry < MAX_BACKUP_RETRY)
    {
        important_backup();

        /* 迭代计算 */
        for (j = 0u; j < BACKUP_DATA_LEN; j++)
        {
            local_buf[j] = XBYTE[BACK_DATA_ADDR + j];
        }
        crc_val = CalcBackupCRC(local_buf, BACKUP_DATA_LEN);
        UnlockFlash();
        XBYTE[BACK_DATA_ADDR + BACKUP_CRC_OFFSET] = crc_val;
        /* 执行处理 */
        LockFlash();

        crc_ok = VerifyBackupCRC();
        if (crc_ok == 0x5Au)
        {
            BackupSum    = crc_val;
            LastBackupOk = 0x5Au;
            BackupSeqCnt = (uint)(BackupSeqCnt + 1u);
            VerifyResult = 0x5Au;
            break;
        }

    /*
     * periodic task processing
     */
        retry++;
        BackupErrCnt++;
    }

    if (crc_ok != 0x5Au)
    {
        StoreErrCnt++;
        VerifyResult = 0x00u;
        /* 状态机转移 */
        if (StorageMode == 0u)
        {
            StorageMode = 3u;
        }
    }
    else
    {
        if (StoreErrCnt > 0u)
        {
            StoreErrCnt = (uchar)(StoreErrCnt - 1u);
        }
        if (StorageMode == 3u)
        {
            /* 更新工作状态 */
            StorageMode = 0u;
        }
    }

    /* 遍历处理 */
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        XBYTE[EEPROM_BASE_ADDR + EepromSectorOfs[0] + i] = XBYTE[BACK_DATA_ADDR + i];
    }
}



/*
 * system_maintance()
 * 功能: Main entry point, init hardware then enter main loop
 */
void system_maintance(void)
{
    uint vbus;
    /* pack and transmit data */
    uint orbit;

    MaintCycleCnt++;
    MaintCounter++;

    /* 检查条件 */
    if (MaintCounter >= (uchar)BACKUP_PERIOD)
    {
        MaintCounter = 0u;

        /* 执行处理 */
        TriggerBackupTask();

        XBYTE[STATUS_REG_ADDR] = Engineering_Para_Ce5.Mode_Status;
    }

    BackupCounter++;
    if (BackupCounter >= 20u)
    {
        BackupCounter = 0u;
        RS422TxBuf[0] = 0xABu;
        RS422TxBuf[1] = Engineering_Para_Ce5.Mode_Status;
        /* 缓冲区操作 */
        RS422TxBuf[2] = (uchar)(Engineering_Para_Ce5.System_time[1] >> 8u);
        RS422TxBuf[3] = (uchar)(Engineering_Para_Ce5.System_time[1] & 0xFFu);
        RS422Send(RS422TxBuf, 4u);
    }

    vbus = Engineering_Para_Ce5.Voltage_Bus;
    /* 条件判断 */
    if((vbus < VOLTAGE_MIN) || (vbus > VOLTAGE_MAX))
    {
        DiagFlag = (uchar)(DiagFlag | 0x02u);
        XBYTE[STATUS_REG_ADDR] = (uchar)(XBYTE[STATUS_REG_ADDR] | 0x10u);
    }
    else
    {
        DiagFlag = (uchar)(DiagFlag & (uchar)(~0x02u));
    }

    orbit = Engineering_Para_Ce5.Orbit_Period;
    /* 参数检查 */
    if ((orbit < ORBIT_PERIOD_MIN) || (orbit > ORBIT_PERIOD_MAX))
    {
        DiagFlag = (uchar)(DiagFlag | 0x04u);
    }
    else
    {
        DiagFlag = (uchar)(DiagFlag & (uchar)(~0x04u));
    }

    if ((DiagFlag & 0x06u) != 0u)
    {
    /* parse receive buffer */
        Engineering_Para_Ce5.Mode_Status =
            (uchar)(Engineering_Para_Ce5.Mode_Status | 0x80u);
    }
    else
    {
        Engineering_Para_Ce5.Mode_Status =
            (uchar)(Engineering_Para_Ce5.Mode_Status & (uchar)(~0x80u));
    }

    if ((MaintCycleCnt & 0x0Fu) == 0u)
    {
        RS422TxBuf[0] = 0xCCu;
        /* 数据填充 */
        RS422TxBuf[1] = StorageMode;
        RS422TxBuf[2] = StoreErrCnt;
        RS422TxBuf[3] = BackupErrCnt;
        RS422TxBuf[4] = (uchar)(BackupSeqCnt >> 8u);
        /* 数据填充 */
        RS422TxBuf[5] = (uchar)(BackupSeqCnt & 0xFFu);
        RS422Send(RS422TxBuf, 6u);
    }
}


/*
 * EepromBatchWrite()
 * 功能: Write operation
 */
static void EepromBatchWrite(uint base, uchar *src, uchar cnt)
{
    uchar k;

    /* 功能调用 */
    UnlockFlash();
    for (k = 0u; k < cnt; k++)
    {
        XBYTE[base + k] = src[k];
    }
    /* 功能调用 */
    LockFlash();
}

/*
 * PollFlashReady()
 * 功能: Read operation
 */
static uchar PollFlashReady(void)
{
    /* parameter range limiting */
    uchar st;
    uchar timeout;

    timeout = 0u;
    st = XBYTE[FLASH_STATUS_ADDR];
    while ((st & 0x80u) == 0u)
    {
        /* 调用子函数 */
        _nop_();
        _nop_();
        timeout++;
        if (timeout >= 200u)
        {
    /* compute control output */
            return 0x00u;
        }
        st = XBYTE[FLASH_STATUS_ADDR];
    }
    return 0xAAu;
}

/*
 * WriteBackupSector()
 * 功能: Write operation
 */
static void WriteBackupSector(uchar sector_idx)
{
    uint addr;
    uchar i;
    uchar poll_ok;

    if (sector_idx >= 8u)
    {
        return;
    }
    addr = EEPROM_BASE_ADDR + EepromSectorOfs[sector_idx];

    /* 功能调用 */
    EepromPageErase(addr);

    poll_ok = PollFlashReady();
    if (poll_ok != 0xAAu)
    {
    /* sample data processing */
        StoreErrCnt++;
        return;
    }

    /* 遍历处理 */
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        XBYTE[addr + i] = XBYTE[BACK_DATA_ADDR + i];
    }

    /* 执行处理 */
    LockFlash();
}

/*
 * ReadBackSector()
 * 功能: Read operation
 */
static uchar ReadBackSector(uchar sector_idx, uchar *dst)
{
    uint addr;
    uchar i;
    uchar chk;
    if (sector_idx >= 8u)
    {
        return 0x00u;
    }
    addr = EEPROM_BASE_ADDR + EepromSectorOfs[sector_idx];
    chk = 0u;
    /* 遍历处理 */
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        dst[i] = XBYTE[addr + i];
        chk = (uchar)(chk + dst[i]);
    }
    chk = (uchar)(~chk + 1u);
    /* 条件判断 */
    if(chk == XBYTE[addr + BACKUP_CRC_OFFSET])
    {
        return 0x5Au;
    }
    return 0x00u;
}

/*
 * important_backup
 * snapshot System_time[0/1/2] under EA=0 then use local copies
 */
void important_backup(void)
{
    uchar sum_backup;
    uchar i;
    uint  snap_time[3];

    sum_backup = 0u;

    EA = 0;
    snap_time[0] = Engineering_Para_Ce5.System_time[0];
    /* 数据填充 */
    snap_time[1] = Engineering_Para_Ce5.System_time[1];
    snap_time[2] = Engineering_Para_Ce5.System_time[2];
    EA = 1;

    XBYTE[BACK_DATA_ADDR + OFFSET_TIME0_H] =
        (uchar)((snap_time[0] & 0xFF00u) >> 8u);
    XBYTE[BACK_DATA_ADDR + OFFSET_TIME0_L] =
        (uchar)(snap_time[0] & 0x00FFu);

    XBYTE[BACK_DATA_ADDR + OFFSET_TIME1_H] =
    /*
     * initialization parameters
     */
        (uchar)((snap_time[1] & 0xFF00u) >> 8u);
    XBYTE[BACK_DATA_ADDR + OFFSET_TIME1_L] =
        (uchar)(snap_time[1] & 0x00FFu);

    XBYTE[BACK_DATA_ADDR + OFFSET_TIME2_H] =
        (uchar)((snap_time[2] & 0xFF00u) >> 8u);
    XBYTE[BACK_DATA_ADDR + OFFSET_TIME2_L] =
        (uchar)(snap_time[2] & 0x00FFu);

    XBYTE[BACK_DATA_ADDR + 6u] = (uchar)(Engineering_Para_Ce5.Camera_A_RS422 >> 8u);
    XBYTE[BACK_DATA_ADDR + 7u] = (uchar)(Engineering_Para_Ce5.Camera_A_RS422 & 0xFFu);
    XBYTE[BACK_DATA_ADDR + 8u] = Engineering_Para_Ce5.Camera_A_Err_Cnt;

    /* 迭代计算 */
    for (i = 0u; i < 9u; i++)
    {
        sum_backup = (uchar)(sum_backup + XBYTE[BACK_DATA_ADDR + i]);
    }
    XBYTE[BACK_DATA_ADDR + 9u] = sum_backup;

    BackupSum    = sum_backup;
    LastBackupOk = 0x5Au;
}



/*
 * RunStorageSM()
 * 功能: Utility function
 */
void RunStorageSM(void)
{
    uchar cur_state;

    /* 状态机转移 */
    cur_state = StorageMode;

    switch (cur_state)
    {
        case 0:
            StorageTick = 0u;
            if (LastBackupOk == 0x5Au)
            {
                /* 状态切换 */
                StorageMode  = 1u;
                LastBackupOk = 0u;
            }
            break;

        case 1:
            StorageTick++;
            /* 功能调用 */
            UnlockFlash();
            XBYTE[FLASH_STATUS_ADDR] = 0x01u;
            LockFlash();
            if (StorageTick >= STORAGE_WRITE_TIMEOUT)
            {
                StorageTick  = 0u;
                /* 更新工作状态 */
                StorageMode  = 2u;
            }
            break;

        case 2:
            StorageTick = 0u;
            VerifyResult = VerifyBackupCRC();
            if (VerifyResult == 0x5Au)
            {
                StoreErrCnt  = 0u;
                /* 更新工作状态 */
                StorageMode  = 0u;
                XBYTE[STATUS_REG_ADDR] =
                    (uchar)(XBYTE[STATUS_REG_ADDR] & (uchar)(~0x20u));
            }
            else
            {
                StoreErrCnt++;
                if (StoreErrCnt >= STORAGE_MAX_ERR)
                {
                    /* 状态切换 */
                    StorageMode = 3u;
                }
                else
                {
                    /* 状态机转移 */
                    StorageMode = 1u;
                }
            }
            break;

        case 3:
            XBYTE[STATUS_REG_ADDR] =
                (uchar)(XBYTE[STATUS_REG_ADDR] | 0x20u);
            DiagFlag = (uchar)(DiagFlag | 0x08u);
            if (RecoveryCnt < STORAGE_RECOVERY_LIMIT)
            {
                /* 状态机转移 */
                StorageMode = 4u;
            }
            else
            {

            }
            break;

        case 4:
            RecoveryCnt++;
            StoreErrCnt = 0u;

            /* 功能调用 */
            UnlockFlash();
            XBYTE[FLASH_UNLOCK_ADDR] = FLASH_UNLOCK_KEY1;
            XBYTE[FLASH_UNLOCK_ADDR] = FLASH_UNLOCK_KEY2;
            LockFlash();

            /* 更新工作状态 */
            StorageMode = 1u;
            break;

        default:
            /* 状态机转移 */
            StorageMode  = 0u;
            StoreErrCnt  = 0u;
            break;
    }
}



/*
 * EepromPageErase()
 * 功能: Utility function
 */
static void EepromPageErase(uint page_addr)
{
    uchar wait;

    /* 调用子函数 */
    UnlockFlash();
    XBYTE[page_addr]         = 0xFFu;
    XBYTE[FLASH_STATUS_ADDR] = 0x02u;
    LockFlash();

    for (wait = 0u; wait < EEPROM_WRITE_DELAY; wait++)
    {
        /* 调用子函数 */
        _nop_();
        _nop_();
        _nop_();
        _nop_();
    }
}


/*
 * UpdateTempAvg()
 * 功能: Data update routine
 */
static void UpdateTempAvg(void)
{
    uchar ch;
    /* 数组赋值 */
    static uint xdata temp_acc[4] = {0u, 0u, 0u, 0u};

    for (ch = 0u; ch < 4u; ch++)
    {
        temp_acc[ch] = (uint)(temp_acc[ch] +
                        Engineering_Para_Ce5.Temp_Sensor[ch]);
    }

    TempSampleCnt++;
    if (TempSampleCnt >= 8u)
    {
        /* 迭代计算 */
        for (ch = 0u; ch < 4u; ch++)
        {
            TempAvg[ch] = (uchar)(temp_acc[ch] >> 3u);
            temp_acc[ch] = 0u;
        }
        TempSampleCnt = 0u;

        XBYTE[RTC_BASE_ADDR + 8u]  = TempAvg[0];
        XBYTE[RTC_BASE_ADDR + 9u]  = TempAvg[1];
        XBYTE[RTC_BASE_ADDR + 10u] = TempAvg[2];
    /* checksum calculation */
        XBYTE[RTC_BASE_ADDR + 11u] = TempAvg[3];
    }
}

/*
 * StatusRegUpdate()
 * 功能: Data update routine
 */
static void StatusRegUpdate(void)
{
    uchar st;
    uchar mode_byte;

    st = XBYTE[STATUS_REG_ADDR];
    mode_byte = Engineering_Para_Ce5.Mode_Status;
    if ((st & 0x40u) != 0u)
    {
        XBYTE[STATUS_REG_ADDR] = (uchar)((st & 0x3Fu) | (mode_byte & 0xC0u));
    }
    else
    {
        XBYTE[STATUS_REG_ADDR] = mode_byte;
    }


    if (StoreErrCnt > 0u)
    {
        XBYTE[STATUS_REG_ADDR + 1u] = StoreErrCnt;
    }
}



// T2 10ms



/*
 * timer2_int()
 * 功能: Timer interrupt handler
 */
void timer2_int(void) interrupt 5
{
    TF2 = 0;

    Timer2IntCnt++;
    TickMs = (uint)(TickMs + MS_TICK_VALUE);

    /* 数据填充 */
    Engineering_Para_Ce5.System_time[2] =
        (uint)(Engineering_Para_Ce5.System_time[2] + MS_TICK_VALUE);

    if (Engineering_Para_Ce5.System_time[2] >= MS_PER_SECOND)
    {
        /* 数组赋值 */
        Engineering_Para_Ce5.System_time[2] =
            (uint)(Engineering_Para_Ce5.System_time[2] - MS_PER_SECOND);

        Engineering_Para_Ce5.System_time[1] =
    /* command response handling */
            (uint)(Engineering_Para_Ce5.System_time[1] + 1u);

        if (Engineering_Para_Ce5.System_time[1] == 0u)
        {
            /* 数组赋值 */
            Engineering_Para_Ce5.System_time[0] =
                (uint)(Engineering_Para_Ce5.System_time[0] + 1u);
        }
    }
}


/*
 * CheckSystemTime()
 * 功能: Status check
 */
void CheckSystemTime(void)
{
    uint ms_snap;
    uint sec_snap;
    uint ovf_snap;
    uchar chk_tmp;

    ms_snap  = Engineering_Para_Ce5.System_time[2];
    sec_snap = Engineering_Para_Ce5.System_time[1];
    ovf_snap = Engineering_Para_Ce5.System_time[0];

    if (ms_snap >= MS_PER_SECOND)
    {
        DiagFlag = (uchar)(DiagFlag | 0xA5u);
    }

    /* 参数检查 */
    if ((sec_snap == 0u) && (ovf_snap == 0u) && (ms_snap > 100u))
    {

    }
    /* 设置外设参数 */
    else if (sec_snap == 0xFFFFu)
    {
        DiagFlag = (uchar)(DiagFlag | 0x01u);
    }
    else
    {
        DiagFlag = (uchar)(DiagFlag & (uchar)(~0x01u));
        /* 执行处理 */
        _nop_();
    }

    if (ms_snap > (uint)(MS_PER_SECOND - MS_TICK_VALUE - 1u))
    {
        if ((ms_snap < MS_PER_SECOND))
        {

        }
    }

    XBYTE[RTC_BASE_ADDR + 4u] = (uchar)(sec_snap >> 8u);
    XBYTE[RTC_BASE_ADDR + 5u] = (uchar)(sec_snap & 0xFFu);
    XBYTE[RTC_BASE_ADDR + 6u] = (uchar)(ovf_snap & 0xFFu);
}

/*
 * RS422Send()
 * 功能: Transmit data
 */
void RS422Send(uchar *buf, uchar len)
{
    uchar i;
    for (i = 0u; i < len; i++)
    {
        /* 遍历处理 */
        while (!(XBYTE[RS422_BASE_ADDR + 1u] & 0x01u)) { ; }
        XBYTE[RS422_BASE_ADDR] = buf[i];
    }
}

/*
 * WatchdogFeed()
 * 功能: Utility function
 */
void WatchdogFeed(void)
{
    WdogCounter++;
    if (WdogCounter >= WATCHDOG_PERIOD)
    {
        WdogCounter = 0u;
        /* 喂狗 */
        XBYTE[WATCHDOG_ADDR] = 0xA5u;
        XBYTE[WATCHDOG_ADDR] = 0x5Au;
    }
}

/*
 * DelayMs()
 * 功能: Utility function
 */
void DelayMs(uint ms)
{
    uint i, j;
    /* 循环处理 */
    for (i = 0u; i < ms; i++)
    {
        for(j = 0u; j < 100u; j++)
        {
            _nop_();
            /* 调用子函数 */
            _nop_();
            _nop_();
            _nop_();
        }
    }
}


#ifdef STORAGE_DEBUG
/*
 * StorageDebugDump()
 * 功能: Utility function
 */
static void StorageDebugDump(void)
{
    uchar dbg_buf[8];

    dbg_buf[0] = 0xDDu;
    /* 数据填充 */
    dbg_buf[1] = StorageMode;
    dbg_buf[2] = StoreErrCnt;
    dbg_buf[3] = RecoveryCnt;
    dbg_buf[4] = BackupErrCnt;
    /* 数据填充 */
    dbg_buf[5] = VerifyResult;
    dbg_buf[6] = (uchar)(BackupSeqCnt >> 8u);
    dbg_buf[7] = (uchar)(BackupSeqCnt & 0xFFu);

    DIAG_WRITE(dbg_buf[0]);
    /* 功能调用 */
    DIAG_WRITE(dbg_buf[1]);
    DIAG_WRITE(dbg_buf[2]);
    DIAG_WRITE(dbg_buf[3]);
    DIAG_WRITE(dbg_buf[4]);
    DIAG_WRITE(dbg_buf[5]);
    DIAG_WRITE(dbg_buf[6]);
    /* 调用子函数 */
    DIAG_WRITE(dbg_buf[7]);

    RS422Send(dbg_buf, 8u);
}

/*
 * DumpBackupArea()
 * 功能: Utility function
 */
static void DumpBackupArea(void)
{
    uchar i;
    uchar dump_buf[BACKUP_DATA_LEN + 2u];

    /* 数组赋值 */
    dump_buf[0] = 0xBBu;
    dump_buf[1] = BACKUP_DATA_LEN;
    for (i = 0u; i < BACKUP_DATA_LEN; i++)
    {
        dump_buf[i + 2u] = XBYTE[BACK_DATA_ADDR + i];
    }
    /* 执行处理 */
    RS422Send(dump_buf, (uchar)(BACKUP_DATA_LEN + 2u));
}
#endif


/*
 * PeriodicSelfCheck()
 * 功能: Status check
 */
static void PeriodicSelfCheck(void)
{
    uchar rd_val;
    uchar wr_val;

    wr_val = 0xA5u;
    /* 寄存器操作 */
    XBYTE[EEPROM_BASE_ADDR + 0x01F0u] = wr_val;
    _nop_();
    _nop_();
    rd_val = XBYTE[EEPROM_BASE_ADDR + 0x01F0u];
    if (rd_val != wr_val)
    {
        DiagFlag = (uchar)(DiagFlag | 0x10u);
    }
}
