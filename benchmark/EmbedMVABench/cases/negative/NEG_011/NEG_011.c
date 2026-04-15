// star_quat_fix.c


/*
 * Include files
 */
#include <stdint.h>
#include <string.h>


/* UART 寄存器基地址（32位嵌入式总线） */
#define UART_BASE               0x40001000U
#define UART_DR                 (*(volatile uint32_t *)(UART_BASE + 0x00U))
#define UART_SR                 (*(volatile uint32_t *)(UART_BASE + 0x04U))
#define UART_CR                 (*(volatile uint32_t *)(UART_BASE + 0x08U))
#define UART_FIFO_COUNT         (*(volatile uint32_t *)(UART_BASE + 0x0CU))

/* 中断控制寄存器 */
#define INTCTRL_BASE            0x40002000U
#define INTCTRL_PEND            (*(volatile uint32_t *)(INTCTRL_BASE + 0x00U))
#define INTCTRL_MASK            (*(volatile uint32_t *)(INTCTRL_BASE + 0x04U))
#define INTCTRL_CLEAR           (*(volatile uint32_t *)(INTCTRL_BASE + 0x08U))

/* 中断掩码位 */
#define IRQ_UART_MASK           0x00000010U   /* UART 接收中断 */

/* 星敏感器帧格式常量 */
#define STAR_FRAME_HEADER       0xAA55U       /* 帧头 */
#define STAR_FRAME_LEN          32U           /* 帧长（字节） */
#define STAR_VEC_OFFSET         4U            /* 向量在帧中的偏移（字节） */
#define STAR_FRAME_BUF_SIZE     64U           /* 帧接收缓冲大小 */

/* 识别模式常量 */
#define MODE_A                  0x01U         /* 全天球识别模式 */
#define MODE_B                  0x02U         /* 局部识别模式 */
#define MODE_C                  0x03U         /* 窗口识别模式 */

/* 导航星表相关 */
#define STARTABLE_MAX_STARS     128U          /* 星表最大星数 */
#define NAVI_STAR_MAX           32U           /* 最大导航星数 */
#define STAR_IMG_WIDTH          1024U         /* 图像宽度（像素） */
#define STAR_IMG_HEIGHT         1024U         /* 图像高度（像素） */
#define WIN_SIZE                64U           /* 识别窗口大小 */
#define MATCH_THRESHOLD         0.98f         /* 匹配门限 */

/* 类型别名 */
typedef uint8_t   U8;
typedef uint16_t  U16;
typedef uint32_t  U32;
typedef int32_t   S32;
typedef float     float32;
typedef double    float64;

/* uint32 别名（与源码保持一致） */
typedef uint32_t  unint32;


/* 姿态历史缓冲深度 */
#define ATT_HIST_SIZE           8U

/* 帧超时阈值（SysTick 计数） */
#define FRAME_TIMEOUT_TICKS     50U

/* 向量有效模长范围下限 */
#define VEC_NORM_MIN            0.95f

/* 向量有效模长范围上限 */
#define VEC_NORM_MAX            1.05f

/* 卡尔曼滤波初始增益 */
#define KALMAN_INIT_GAIN        0.1f

/* 星点帧数据偏移（星点列表起始偏移字节） */
#define STAR_POINTS_OFFSET      16U

/* 每颗候选星点占用字节数（px/py 各12位，共3字节，取整为4字节） */
#define STAR_POINT_BYTES        4U

/* 最大候选星提取数量 */
#define MAX_EXTRACT_STARS       8U

/* 窗口中心坐标归一化因子 */
#define WIN_CENTER_SCALE        512.0f

/* 状态机最大恢复帧数 */
#define SM_RECOVERY_FRAMES      10U


/* 三维浮点向量（粗捕获姿态向量核心类型） */
typedef struct {
    volatile float32   v3[3];   /* 三个分量：[0]=X, [1]=Y, [2]=Z */
    volatile U32       valid;   /* 有效性标志 */
} Vector3D;

/* 四元数表示 */
typedef struct {
    float32   q[4];    /* q[0]=标量部分, q[1..3]=向量部分 */
} Quaternion;

/* 姿态角（欧拉角） */
typedef struct {
    float32   roll;    /* 滚转角（rad） */
    float32   pitch;   /* 俯仰角（rad） */
    float32   yaw;     /* 偏航角（rad） */
} EulerAngle;

/* 姿态状态（含多个 Vector3D 嵌套成员） */
typedef struct {
    Vector3D    roughVec;      /* 粗捕获姿态向量（直接映射 mRoughVector） */
    Quaternion  attQuat;       /* 当前姿态四元数 */
    EulerAngle  attEuler;      /* 当前欧拉角 */
    float32     matchScore;    /* 识别匹配得分 */
    U8          modeValid;     /* 当前模式有效标志 */
    U8          reserved[3];   /* 对齐保留 */
} AttitudeState;

/* 导航星候选点 */
typedef struct {
    float32   px;      /* 图像 X 坐标 */
    float32   py;      /* 图像 Y 坐标 */
    float32   mag;     /* 星等 */
    U32       id;      /* 星表编号 */
} StarPoint;

/* 浮点-字节联合体（用于 UART 帧解析） */
typedef union {
    float32   flt32;
    U8        b[4];
    U32       u32;
} FloatBytes;

/* 任务调度控制块 */
typedef struct {
    U8    mode_cur;        /* 当前识别模式 */
    U8    mode_prev;       /* 上一周期模式 */
    U8    modeB_changed;   /* MODE_B 变化标志 */
    U8    running;         /* 任务运行标志 */
    U32   frame_count;     /* 帧计数 */
    U32   err_count;       /* 错误计数 */
} ModeCtrlBlock;

/*
 * Type definitions
 */
typedef struct {
    U32       frame_cnt;         /* 累计处理帧数 */
    U32       total_pixels;      /* 累计有效像素数 */
    float32   bg_gray_mean;      /* 背景灰度均值 */
    U32       detected_stars;    /* 检测到的星点总数 */
} StarImageStats;

/*
 * Type definitions
 */
typedef struct {
    float32    match_score_hist[ATT_HIST_SIZE];   /* 最近8帧匹配得分 */
    Quaternion quat_snap[ATT_HIST_SIZE];          /* 四元数快照 */
    U32        count;                             /* 已填充条目数 */
} AttHistory;

/*
 * Macro definitions
 */
#define DIAG_OK            0x00u
#define DIAG_NOFRAME       0x03u
#define DIAG_VEC_INVALID   0x05u
#define DIAG_MISMATCH      0x09u

/*
 * Type definitions
 */
typedef struct {
    float32   center_x;       /* 窗口中心 X（图像坐标，像素） */
    float32   center_y;       /* 窗口中心 Y（图像坐标，像素） */
    float32   radius;         /* 搜索半径（像素） */
    U32       max_cand_stars; /* 最大候选星点数量 */
} WinParams;


/* 导航星数量配置表（按模式索引） */
static const U32 NaviStarNumTable[4] = {0U, 12U, 8U, 6U};

/* 星等阈值配置表 */
static const float32 MagThreshTable[4] = {0.0f, 5.5f, 6.0f, 6.5f};

/* 窗口尺寸配置表 */
static const U32 WinSizeTable[4] = {0U, WIN_SIZE * 2U, WIN_SIZE, WIN_SIZE / 2U};

/* 帧长度配置（字节） */
static const U32 FrameLenTable[2] = {STAR_FRAME_LEN, STAR_FRAME_LEN * 2U};


static volatile Vector3D     mRoughVector;

/* 影子变量（ISR 写入，主任务在临界区内一次性拷贝） */
static volatile float32      mRoughVec[3];

/* 影子变量更新标志（ISR 置位，主任务在临界区内读后清零） */
static volatile U8           mModeBChged;

/* 姿态状态全局实例 */
static volatile AttitudeState  gAttState;

/* UART 帧接收缓冲 */
static volatile U8           gUartFrameBuf[STAR_FRAME_BUF_SIZE];
static volatile U32          gUartFrameLen;
static volatile U8           gUartFrameReady;

/* 导航星数量（当前模式） */
static volatile U32          NUM_NAVI_STAR;

/* 导航星数量指针（模拟星表指针） */
static const U32             NAVI_NUM_C = 12U;
static const U32             NAVI_NUM_B = 8U;
static const U32 *const      STARTABLE_GUNUM_C = &NAVI_NUM_C;
static const U32 *const      STARTABLE_GUNUM_B = &NAVI_NUM_B;

/* 当前应用模式 */
static volatile U8           AppMode;

/* 候选星点列表 */
static StarPoint              gStarList[NAVI_STAR_MAX];
static volatile U32           gStarCount;

/* 系统节拍 */
static volatile U32           SysTick;

/* 姿态矩阵（用于计算中间结果） */
static float32                gAttMatrix[3][3];

/* 系统初始化完成标志 */
static U8                     InitDone;


static StarImageStats         imgSt;


static AttHistory             attH;


static U8       diagS;


static WinParams              winP;


static volatile U32           lastTk;


static U8                     attHi;


void SysInit(void);
void UartHwInit(void);
void StarSensorInit(void);
void AttStateInit(void);
void ModeCtrlInit(void);

void ModeScheduler(void);
void ModeInvoke(void);
void StarSensorProcess(void);
void WinRecognise(void);
void LocalRecognise(void);
unint32 JDSIGN(unint32 point1, unint32 point2);
void ModeJudge(void);
void CONSGSTP(void);
void SeekIP(U32 idx);

void INT_UART_ISR(void);
void ParseStarFrame(void);

void QuatNormalize(Quaternion *q);
void Vec3Normalize(float32 *v);
float32 Vec3Dot(const float32 *a, const float32 *b);
void Vec3Cross(const float32 *a, const float32 *b, float32 *out);
void MatVec3Mul(const float32 mat[3][3], const float32 *v, float32 *out);
float32 Sqrtx(float32 x);
void DisableUartIRQ(void);
void EnableUartIRQ(void);


void StarSensorDiagInit(void);
void RecordAttHistory(float32 score);
void EvalAttHistory(void);
void CheckFrameTimeout(void);
void UpdateWinParams(const float32 *rough_vec);
void ExtractStarPoints(const volatile U8 *frame_buf, U32 frame_len);
void QuatToEuler(const Quaternion *q, EulerAngle *euler);
void UpdateAttitudeFromVec(const float32 *rough_vec);
void RunStarSensorSM(void);

#ifdef STAR_VEC_VERBOSE
static void StarVecDump(void);
#endif

int main(void);


/**
 * @brief 系统主程序，完成初始化后进入主循环
 */
int main(void)
{
    /* 系统初始化 */
    SysInit();

    /* 主循环 */
    while (1)
    {
        SysTick++;

        /* 模式调度（层1调用） */
        ModeScheduler();

        /* 每5帧执行一次姿态判断 */
        if ((SysTick % 5U) == 0U)
        {
            ModeJudge();
        }
    }

    return 0;
}


/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void SysInit(void)
{
    UartHwInit();
    StarSensorInit();
    AttStateInit();
    ModeCtrlInit();
    StarSensorDiagInit();
    InitDone = 1U;
}

/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void UartHwInit(void)
{
    /* 配置波特率（假设系统时钟 40MHz，目标 115200） */
    UART_CR = 0x00000001U;   /* 使能 UART */

    /* 使能 UART 接收中断 */
    INTCTRL_MASK |= IRQ_UART_MASK;

    /* 清空帧缓冲 */
    gUartFrameLen   = 0U;
    gUartFrameReady = 0U;
}

/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void StarSensorInit(void)
{
    U32 i;

    /* 初始化粗捕获向量 */
    mRoughVector.v3[0] = 0.0f;
    mRoughVector.v3[1] = 0.0f;
    mRoughVector.v3[2] = 1.0f;   /* 初始指向 Z 轴 */
    mRoughVector.valid = 0U;

    mRoughVec[0] = 0.0f;
    mRoughVec[1] = 0.0f;
    mRoughVec[2] = 1.0f;
    mModeBChged  = 0U;

    /* 清空星点列表 */
    for (i = 0U; i < NAVI_STAR_MAX; i++)
    {
        gStarList[i].px  = 0.0f;
        gStarList[i].py  = 0.0f;
        gStarList[i].mag = 0.0f;
        gStarList[i].id  = 0U;
    }
    gStarCount = 0U;
}

/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void AttStateInit(void)
{
    U32 i, j;

    gAttState.attQuat.q[0] = 1.0f;
    gAttState.attQuat.q[1] = 0.0f;
    gAttState.attQuat.q[2] = 0.0f;
    gAttState.attQuat.q[3] = 0.0f;

    gAttState.attEuler.roll  = 0.0f;
    gAttState.attEuler.pitch = 0.0f;
    gAttState.attEuler.yaw   = 0.0f;

    gAttState.matchScore  = 0.0f;
    gAttState.modeValid   = 0U;

    /* 初始化姿态矩阵为单位矩阵 */
    for (i = 0U; i < 3U; i++)
    {
        for (j = 0U; j < 3U; j++)
        {
            gAttMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}


/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void ModeCtrlInit(void)
{
    AppMode   = MODE_A;
    SysTick   = 0U;
    NUM_NAVI_STAR = 0U;
}


/**
 * @brief 任务调度
 */
void ModeScheduler(void)
{
    /* 执行图像处理与模式调用（层2调用） */
    StarSensorProcess();

    /* 运行星敏感器状态机 */
    RunStarSensorSM();

    /* 检查帧超时 */
    CheckFrameTimeout();
}


/**
 * @brief 传感器数据处理
 */
void StarSensorProcess(void)
{
    /* 调用模式调用函数（层3调用） */
    ModeInvoke();
}


/**
 * @brief 模式管理
 */
void ModeInvoke(void)
{
    /* 根据当前模式选择识别算法 */
    if (AppMode == MODE_C)
    {
        /* 获取 MODE_C 导航星数量 */
        NUM_NAVI_STAR = *(STARTABLE_GUNUM_C);
        /* 窗口识别（读取 mRoughVector.v3） */
        WinRecognise();
    }
    else if (AppMode == MODE_B)
    {
        /* 获取 MODE_B 导航星数量 */
        NUM_NAVI_STAR = *(STARTABLE_GUNUM_B);
        /* 局部识别（通过深层调用链读取 mRoughVector.v3） */
        LocalRecognise();
    }
    else
    {
        /* MODE_A：全天球识别，不依赖粗捕获向量 */
        NUM_NAVI_STAR = NaviStarNumTable[MODE_A];
    }
}


/**
 * @brief 功能处理
 */
void WinRecognise(void)
{
    float32 tempA[3][3];
    float32 norm;
    U32 i;

    /* 模拟从图像计算姿态矩阵 */
    tempA[0][0] = gAttState.attQuat.q[0] * gAttState.attQuat.q[0]
                - gAttState.attQuat.q[1] * gAttState.attQuat.q[1]
                - gAttState.attQuat.q[2] * gAttState.attQuat.q[2]
                + gAttState.attQuat.q[3] * gAttState.attQuat.q[3];

    tempA[0][1] = 2.0f * (gAttState.attQuat.q[0] * gAttState.attQuat.q[1]
                        + gAttState.attQuat.q[2] * gAttState.attQuat.q[3]);

    tempA[0][2] = 2.0f * (gAttState.attQuat.q[0] * gAttState.attQuat.q[2]
                        - gAttState.attQuat.q[1] * gAttState.attQuat.q[3]);

    tempA[1][0] = 2.0f * (gAttState.attQuat.q[0] * gAttState.attQuat.q[1]
                        - gAttState.attQuat.q[2] * gAttState.attQuat.q[3]);

    tempA[1][1] = gAttState.attQuat.q[1] * gAttState.attQuat.q[1]
                - gAttState.attQuat.q[0] * gAttState.attQuat.q[0]
                - gAttState.attQuat.q[2] * gAttState.attQuat.q[2]
                + gAttState.attQuat.q[3] * gAttState.attQuat.q[3];

    tempA[1][2] = 2.0f * (gAttState.attQuat.q[1] * gAttState.attQuat.q[2]
                        + gAttState.attQuat.q[0] * gAttState.attQuat.q[3]);

    tempA[2][0] = 2.0f * (gAttState.attQuat.q[0] * gAttState.attQuat.q[2]
                        + gAttState.attQuat.q[1] * gAttState.attQuat.q[3]);

    tempA[2][1] = 2.0f * (gAttState.attQuat.q[1] * gAttState.attQuat.q[2]
                        - gAttState.attQuat.q[0] * gAttState.attQuat.q[3]);

    tempA[2][2] = gAttState.attQuat.q[2] * gAttState.attQuat.q[2]
                - gAttState.attQuat.q[0] * gAttState.attQuat.q[0]
                - gAttState.attQuat.q[1] * gAttState.attQuat.q[1]
                + gAttState.attQuat.q[3] * gAttState.attQuat.q[3];

    /* 归一化第三行向量（与安装矩阵旋转轴 Z 对应） */
    norm = Sqrtx(tempA[2][0] * tempA[2][0]
               + tempA[2][1] * tempA[2][1]
               + tempA[2][2] * tempA[2][2]);
    if (norm > 1.0e-6f)
    {
        for (i = 0U; i < 3U; i++)
        {
            tempA[2][i] /= norm;
        }
    }

    /* 将计算结果写入粗捕获向量（含姿态安装轴与旋转轴 Z 对应） */
    mRoughVector.v3[0] = tempA[2][0];
    mRoughVector.v3[1] = tempA[2][1];
    mRoughVector.v3[2] = tempA[2][2];
    mRoughVector.valid = 1U;

    gAttState.matchScore = 0.95f;
    gAttState.modeValid  = 1U;
}

/**
 * @brief 功能处理
 */
void LocalRecognise(void)
{
    /* 调用局部识别核心搜索（含 CONSGSTP → SeekIP → JDSIGN 子链） */
    CONSGSTP();
}

/**
 * @brief 功能处理
 */
void CONSGSTP(void)
{
    U32 i;

    /* 对每个候选星点执行角距搜索 */
    for (i = 0U; i < gStarCount && i < NAVI_STAR_MAX; i++)
    {
        SeekIP(i);
    }
}

/**
 * @brief 功能处理
 */
void SeekIP(U32 idx)
{
    U32 j;
    unint32 res;

    for (j = 0U; j < gStarCount && j < NAVI_STAR_MAX; j++)
    {
        if (j == idx)
        {
            continue;
        }
        /* 调用角距符号判断（层4，读取 mRoughVector.v3[0..1]） */
        res = JDSIGN((unint32)(gStarList[idx].id),
                     (unint32)(gStarList[j].id));
        if (res != 0U)
        {
            gAttState.matchScore += 0.01f;
        }
    }
}

/**
 * @brief 功能处理
 */
unint32 JDSIGN(unint32 point1, unint32 point2)
{
    float32  px1, py1, px2, py2;
    float32  v11, v22;

    /* 取出两星点图像坐标 */
    if (point1 < NAVI_STAR_MAX && point2 < NAVI_STAR_MAX)
    {
        px1 = gStarList[point1].px;
        py1 = gStarList[point1].py;
        px2 = gStarList[point2].px;
        py2 = gStarList[point2].py;
    }
    else
    {
        return 0U;
    }

    /* 根据粗捕获姿态向量判断角距符号
     * v3[1] 对应 Y 分量，v3[0] 对应 X 分量 */
    v11 = px1 * mRoughVector.v3[1];   /* 读取 v3[1] */
    v22 = py2 * mRoughVector.v3[0];   /* 读取 v3[0] */

    if (v11 >= v22)
    {
        /* 正向角距 */
        return 1U;
    }
    else
    {
        /* 反向角距 */
        return 0U;
    }

    (void)py1;
    (void)px2;
}


/**
 * @brief 串口接收中断服务程序
 */
void INT_UART_ISR(void)  /* isr_handler: uart_rx_irq */
{
    U32 i;
    U32 rx_len;

    /* 清除中断挂起 */
    INTCTRL_CLEAR = IRQ_UART_MASK;

    /* 读取 UART FIFO ��的字节数 */
    rx_len = UART_FIFO_COUNT;
    if (rx_len > STAR_FRAME_BUF_SIZE)
    {
        rx_len = STAR_FRAME_BUF_SIZE;
    }

    /* 读取帧数据 */
    for(i = 0U; i < rx_len; i++)
    {
        gUartFrameBuf[i] = (U8)(UART_DR & 0xFFU);
    }
    gUartFrameLen = rx_len;

    /* 调用串口接收指令处理（层2） */
    ParseStarFrame();
}

/**
 * @brief 通信帧处理
 */
void ParseStarFrame(void)
{
    FloatBytes tmpcvt;
    float32    tempvarf;
    U32        i;
    U32        base;

    /* 验证帧头 */
    if (gUartFrameLen < STAR_FRAME_LEN)
    {
        return;
    }
    if (((U16)gUartFrameBuf[0] | ((U16)gUartFrameBuf[1] << 8U)) != STAR_FRAME_HEADER)
    {
        return;
    }

    /* 从帧中提取三维粗捕获向量（每个分量4字节小端 float32） */
    base = STAR_VEC_OFFSET;
    for (i = 0U; i < 3U; i++)
    {
        tmpcvt.b[0] = gUartFrameBuf[base + i * 4U + 0U];
        tmpcvt.b[1] = gUartFrameBuf[base + i * 4U + 1U];
        tmpcvt.b[2] = gUartFrameBuf[base + i * 4U + 2U];
        tmpcvt.b[3] = gUartFrameBuf[base + i * 4U + 3U];

        /* 写入影子变量，避免中断上下文直接修改共享的 mRoughVector.v3[] */
        mRoughVec[i] = tmpcvt.flt32;   /* 写入影子变量 mRoughVec[0/1/2] */
    }

    /* 计算向量模长（使用影子变量进行归一化校验） */
    tempvarf = Sqrtx(mRoughVec[0] * mRoughVec[0]
                   + mRoughVec[1] * mRoughVec[1]
                   + mRoughVec[2] * mRoughVec[2]);

    /* 若模长有效则置位影子变量更新标志，通知主任务在临界区中安全复制 */
    if (tempvarf > 1.0e-6f)
    {
        mModeBChged = 1U;
    }

    gUartFrameReady = 1U;

    /* 更新帧时间戳和统计计数 */
    lastTk = SysTick;
    imgSt.frame_cnt++;

    /* 提取星点候选列表 */
    ExtractStarPoints(gUartFrameBuf, gUartFrameLen);
}


/**
 * @brief 模式管理
 */
void ModeJudge(void)
{
    float32 norm;

    /* 检查中断侧是否有新数据可用 */
    if (mModeBChged == 1U)
    {
        /* 关 UART 中断（进入临界区） */
        DisableUartIRQ();

        /* 一次性原子拷贝三个分量到粗捕获向量 */
        mRoughVector.v3[0] = mRoughVec[0];
        mRoughVector.v3[1] = mRoughVec[1];
        mRoughVector.v3[2] = mRoughVec[2];

        mModeBChged = 0U;

        /* 恢复 UART 中断（退出临界区） */
        EnableUartIRQ();
    }

    /* 根据粗捕获向量 Z 分量决定模式切换 */
    norm = Sqrtx(mRoughVector.v3[0] * mRoughVector.v3[0]
               + mRoughVector.v3[1] * mRoughVector.v3[1]
               + mRoughVector.v3[2] * mRoughVector.v3[2]);

    if(norm > 0.9f)
    {
        if (mRoughVector.v3[2] > MATCH_THRESHOLD)
        {
            AppMode = MODE_C;
        }
        else if (norm > 0.5f)
        {
            AppMode = MODE_B;
        }
        else
        {
            AppMode = MODE_A;
        }
        gAttState.modeValid = 1U;
    }
}



/**
 * @brief 功能处理
 */
float32 Sqrtx(float32 x)
{
    float32 result;
    float32 half;
    float32 guess;
    U32     iter;

    if (x <= 0.0f)
    {
        return 0.0f;
    }

    half   = x * 0.5f;
    guess  = x;

    /* 牛顿迭代 */
    for (iter = 0U; iter < 8U; iter++)
    {
        guess = guess - (guess * guess - x) / (2.0f * guess);
    }
    result = guess;
    (void)half;
    return result;
}

/**
 * @brief 功能处理
 */
void QuatNormalize(Quaternion *q)
{
    float32 norm;
    float32 inv;
    U32 i;

    norm = 0.0f;
    for (i = 0U; i < 4U; i++)
    {
        norm += q->q[i] * q->q[i];
    }
    norm = Sqrtx(norm);
    if (norm > 1.0e-6f)
    {
        inv = 1.0f / norm;
        for (i = 0U; i < 4U; i++)
        {
            q->q[i] *= inv;
        }
    }
}

/**
 * @brief 功能处理
 */
void Vec3Normalize(float32 *v)
{
    float32 norm;
    float32 inv;
    U32 i;

    norm = Sqrtx(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (norm > 1.0e-6f)
    {
        inv = 1.0f / norm;
        for (i = 0U; i < 3U; i++)
        {
            v[i] *= inv;
        }
    }
}

/**
 * @brief 功能处理
 */
float32 Vec3Dot(const float32 *a, const float32 *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief 功能处理
 */
void Vec3Cross(const float32 *a, const float32 *b, float32 *out)
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * @brief 功能处理
 */
void MatVec3Mul(const float32 mat[3][3], const float32 *v, float32 *out)
{
    U32 i, j;

    for (i = 0U; i < 3U; i++)
    {
        out[i] = 0.0f;
        for (j = 0U; j < 3U; j++)
        {
            out[i] += mat[i][j] * v[j];
        }
    }
}


/**
 * @brief 串口接收中断服务程序
 */
void DisableUartIRQ(void)
{
    /* bit operation */
    INTCTRL_MASK &= ~IRQ_UART_MASK;
}

/**
 * @brief 串口接收中断服务程序
 */
void EnableUartIRQ(void)
{
    INTCTRL_MASK |= IRQ_UART_MASK;
}


/**
 * @brief 模块初始化，配置寄存器和外设参数
 */
void StarSensorDiagInit(void)
{
    U32 i;

    /* 清零图像统计 */
    imgSt.frame_cnt      = 0U;
    imgSt.total_pixels   = 0U;
    imgSt.bg_gray_mean   = 0.0f;
    imgSt.detected_stars = 0U;

    /* 清零姿态历史缓冲 */
    for (i = 0U; i < ATT_HIST_SIZE; i++)
    {
        attH.match_score_hist[i]    = 0.0f;
        attH.quat_snap[i].q[0]      = 1.0f;
        attH.quat_snap[i].q[1]      = 0.0f;
        attH.quat_snap[i].q[2]      = 0.0f;
        attH.quat_snap[i].q[3]      = 0.0f;
    }
    attH.count = 0U;

    /* 初始化诊断状态 */
    diagS = DIAG_OK;

    /* 初始化窗口参数 */
    winP.center_x       = WIN_CENTER_SCALE;
    winP.center_y       = WIN_CENTER_SCALE;
    winP.radius         = (float32)WIN_SIZE;
    winP.max_cand_stars = MAX_EXTRACT_STARS;

    /* 初始化帧时间戳和历史写指针 */
    lastTk = 0U;
    attHi    = 0U;
}

/**
 * @brief 功能处理
 */
void RecordAttHistory(float32 score)
{
    U8 idx;

    idx = attHi;

    /* 写入匹配得分 */
    attH.match_score_hist[idx] = score;

    /* 拍取当前四元数快照 */
    attH.quat_snap[idx].q[0] = gAttState.attQuat.q[0];
    attH.quat_snap[idx].q[1] = gAttState.attQuat.q[1];
    attH.quat_snap[idx].q[2] = gAttState.attQuat.q[2];
    attH.quat_snap[idx].q[3] = gAttState.attQuat.q[3];

    /* 推进环形写指针 */
    attHi = (U8)((idx + 1U) % ATT_HIST_SIZE);

    /* 更新已填充计数（上限为 ATT_HIST_SIZE） */
    if (attH.count < ATT_HIST_SIZE)
    {
        attH.count++;
    }
}

/**
 * @brief 功能处理
 */
void EvalAttHistory(void)
{
    U32     i;
    float32 sum;
    float32 mean;
    float32 var_sum;
    float32 diff;
    float32 valid_cnt;

    /* 须有足够历史数据才评估 */
    if (attH.count == 0U)
    {
        return;
    }

    valid_cnt = (float32)attH.count;
    sum       = 0.0f;

    /* 计算均值 */
    for (i = 0U; i < attH.count; i++)
    {
        sum += attH.match_score_hist[i];
    }
    mean = sum / valid_cnt;

    /* 计算方差 */
    var_sum = 0.0f;
    for (i = 0U; i < attH.count; i++)
    {
        diff     = attH.match_score_hist[i] - mean;
        var_sum += diff * diff;
    }

    /* 均值低于门限则触发全天球重捕获 */
    if (mean < 0.8f)
    {
        /* update state */
        AppMode      = MODE_A;
        diagS = DIAG_MISMATCH;
    }
    else
    {
        /* 方差过大也降为 MODE_B 稳定 */
        if((var_sum / valid_cnt) > 0.04f)
        {
            if (AppMode == MODE_C)
            {
                /* FSM transition */
                AppMode = MODE_B;
            }
        }
        diagS = DIAG_OK;
    }

    (void)var_sum;
}

/**
 * @brief 状态检查
 */
void CheckFrameTimeout(void)
{
    U32 elapsed;

    /* 防溢出计算（SysTick 单调递增） */
    if (SysTick >= lastTk)
    {
        elapsed = SysTick - lastTk;
    }
    else
    {
        /* 溢出回绕：取最大差值 */
        elapsed = (0xFFFFFFFFU - lastTk) + SysTick + 1U;
    }


    if (elapsed > FRAME_TIMEOUT_TICKS)
    {
        diagS = DIAG_NOFRAME;
    }
}

/**
 * @brief 数据更新
 */
void UpdateWinParams(const float32 *rough_vec)
{
    float32 cx, cy;
    float32 norm;
    float32 r;

    /* 计算向量模长 */
    norm = Sqrtx(rough_vec[0] * rough_vec[0]
               + rough_vec[1] * rough_vec[1]
               + rough_vec[2] * rough_vec[2]);

    /* 向量模长越偏离1.0，窗口半径越大以适应不确定性 */
    if (norm < VEC_NORM_MIN || norm > VEC_NORM_MAX)
    {
        diagS = DIAG_VEC_INVALID;
        /* 使用默认窗口参数 */
        winP.center_x       = WIN_CENTER_SCALE;
        winP.center_y       = WIN_CENTER_SCALE;
        winP.radius         = (float32)(WIN_SIZE * 2U);
        winP.max_cand_stars = MAX_EXTRACT_STARS;
        return;
    }


    /* 简单透视投影：假设焦距归一化为1，图像中心为 (512, 512) */
    if (rough_vec[2] > 1.0e-4f)
    {
        cx = WIN_CENTER_SCALE + (rough_vec[0] / rough_vec[2]) * WIN_CENTER_SCALE;
        cy = WIN_CENTER_SCALE + (rough_vec[1] / rough_vec[2]) * WIN_CENTER_SCALE;
    }
    else
    {
        cx = WIN_CENTER_SCALE;
        cy = WIN_CENTER_SCALE;
    }

    /* 限制在图像范围内 */
    if (cx < 0.0f)           { cx = 0.0f; }
    if (cx > (float32)(STAR_IMG_WIDTH  - 1U)) { cx = (float32)(STAR_IMG_WIDTH  - 1U); }
    if (cy < 0.0f)           { cy = 0.0f; }
    if (cy > (float32)(STAR_IMG_HEIGHT - 1U)) { cy = (float32)(STAR_IMG_HEIGHT - 1U); }

    /* 根据向量与参考 Z 轴的偏角调整半径 */
    r = (float32)WIN_SIZE + (1.0f - rough_vec[2]) * (float32)(WIN_SIZE * 2U);
    if (r < (float32)(WIN_SIZE / 2U))   { r = (float32)(WIN_SIZE / 2U); }
    if (r > (float32)(WIN_SIZE * 4U))   { r = (float32)(WIN_SIZE * 4U); }

    winP.center_x       = cx;
    winP.center_y       = cy;
    winP.radius         = r;
    winP.max_cand_stars = MAX_EXTRACT_STARS;
}

/**
 * @brief 星敏感器处理
 */
void ExtractStarPoints(const volatile U8 *frame_buf, U32 frame_len)
{
    U32     i;
    U32     offset;
    U32     raw_word;
    U32     n_stars;
    float32 px_raw, py_raw;

    /* 帧长度不足则放弃提取 */
    if (frame_len < (STAR_POINTS_OFFSET + STAR_POINT_BYTES))
    {
        return;
    }

    /* 计算可提取的星点数量上限 */
    n_stars = (frame_len - STAR_POINTS_OFFSET) / STAR_POINT_BYTES;
    if (n_stars > MAX_EXTRACT_STARS)
    {
        n_stars = MAX_EXTRACT_STARS;
    }

    /* 提取各星点坐标 */
    for(i = 0U; i < n_stars; i++)
    {
        offset = STAR_POINTS_OFFSET + i * STAR_POINT_BYTES;

        /* 读取4字节小端 raw word */
        raw_word  = (U32)frame_buf[offset + 0U];
        raw_word |= (U32)frame_buf[offset + 1U] << 8U;
        raw_word |= (U32)frame_buf[offset + 2U] << 16U;
        raw_word |= (U32)frame_buf[offset + 3U] << 24U;

        /* 低12位为 px，高12位为 py（bit 0..11 = px, bit 12..23 = py） */
        px_raw = (float32)(raw_word & 0x00000FFFU);
        py_raw = (float32)((raw_word >> 12U) & 0x00000FFFU);

        /* 写入全局星点列表 */
        if (i < NAVI_STAR_MAX)
        {
            gStarList[i].px  = px_raw;
            gStarList[i].py  = py_raw;
            gStarList[i].mag = 0.0f;   /* 星等待后续帧数据填充 */
            gStarList[i].id  = i;
        }
    }

    /* 更新全局星点计数 */
    gStarCount = n_stars;
    imgSt.detected_stars += n_stars;
}

/**
 * @brief 功能处理
 */
void QuatToEuler(const Quaternion *q, EulerAngle *euler)
{
    float32 q0, q1, q2, q3;
    float32 sin_pitch;
    float32 t0, t1, t2, t3;
    float32 ratio, angle;
    float32 x3, x5;

    q0 = q->q[0];
    q1 = q->q[1];
    q2 = q->q[2];
    q3 = q->q[3];

    /* ---- pitch（俯仰角）via asin 近似 ---- */
    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    /* 限幅到 [-1, 1] */
    if (sin_pitch >  1.0f) { sin_pitch =  1.0f; }
    if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }

    /* asin 泰勒展开（|x| < 0.5 精度较好） */
    x3 = sin_pitch * sin_pitch * sin_pitch;
    x5 = x3 * sin_pitch * sin_pitch;
    euler->pitch = sin_pitch + x3 / 6.0f + (3.0f * x5) / 40.0f;

    /* ---- roll（滚转角）via atan2 近似 ---- */
    t0 = 2.0f * (q0 * q1 + q2 * q3);
    t1 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    if (t1 > 1.0e-6f || t1 < -1.0e-6f)
    {
        ratio = t0 / t1;
        /* atan(x) ≈ x - x^3/3 + x^5/5（|x| < 1） */
        if (ratio >  1.0f) { ratio =  1.0f; }
        if (ratio < -1.0f) { ratio = -1.0f; }
        angle = ratio - (ratio * ratio * ratio) / 3.0f
                      + (ratio * ratio * ratio * ratio * ratio) / 5.0f;
        euler->roll = (t1 > 0.0f) ? angle : (angle + 3.14159265f);
    }
    else
    {
        euler->roll = (t0 >= 0.0f) ? (3.14159265f / 2.0f) : -(3.14159265f / 2.0f);
    }

    /* ---- yaw（偏航角）via atan2 近似 ---- */
    t2 = 2.0f * (q0 * q3 + q1 * q2);
    t3 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);

    if (t3 > 1.0e-6f || t3 < -1.0e-6f)
    {
        ratio = t2 / t3;
        if (ratio >  1.0f) { ratio =  1.0f; }
        if (ratio < -1.0f) { ratio = -1.0f; }
        angle = ratio - (ratio * ratio * ratio) / 3.0f
                      + (ratio * ratio * ratio * ratio * ratio) / 5.0f;
        euler->yaw = (t3 > 0.0f) ? angle : (angle + 3.14159265f);
    }
    else
    {
        euler->yaw = (t2 >= 0.0f) ? (3.14159265f / 2.0f) : -(3.14159265f / 2.0f);
    }
}

/**
 * @brief 数据更新
 */
void UpdateAttitudeFromVec(const float32 *rough_vec)
{
    float32 ref[3];
    float32 axis[3];
    float32 dot_val;
    float32 axis_norm;
    float32 half_angle;
    float32 sin_half;
    Quaternion new_quat;

    /* 参考方向为 Z 轴正方向 */
    ref[0] = 0.0f;
    ref[1] = 0.0f;
    ref[2] = 1.0f;

    /* 计算点积（等于 cos(theta)） */
    dot_val = Vec3Dot(rough_vec, ref);

    /* 限幅防止数值误差 */
    if (dot_val >  1.0f) { dot_val =  1.0f; }
    if (dot_val < -1.0f) { dot_val = -1.0f; }

    /* 计算旋转轴（叉积） */
    Vec3Cross(ref, rough_vec, axis);

    axis_norm = Sqrtx(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);

    if (axis_norm < 1.0e-6f)
    {
        /* 向量与参考方向平行，无需旋转 */
        if (dot_val >= 0.0f)
        {
            new_quat.q[0] = 1.0f;
            new_quat.q[1] = 0.0f;
            new_quat.q[2] = 0.0f;
            new_quat.q[3] = 0.0f;
        }
        else
        {
            /* 反向：绕 X 轴旋转 180 度 */
            new_quat.q[0] = 0.0f;
            new_quat.q[1] = 1.0f;
            new_quat.q[2] = 0.0f;
            new_quat.q[3] = 0.0f;
        }
    }
    else
    {
        /* 归一化旋转轴 */
        axis[0] /= axis_norm;
        axis[1] /= axis_norm;
        axis[2] /= axis_norm;

        /* 半角余弦近似 */
        half_angle = (3.14159265f / 2.0f - dot_val) * 0.5f;

        /* sin(half_angle) 泰勒近似 */
        sin_half = half_angle - (half_angle * half_angle * half_angle) / 6.0f;

        /* 构造四元数：q = [cos(θ/2), sin(θ/2)*axis] */
        new_quat.q[0] = 1.0f - half_angle * half_angle / 2.0f;
        new_quat.q[1] = sin_half * axis[0];
        new_quat.q[2] = sin_half * axis[1];
        new_quat.q[3] = sin_half * axis[2];
    }

    /* 归一化并更新全局姿态四元数 */
    QuatNormalize(&new_quat);
    gAttState.attQuat.q[0] = new_quat.q[0];
    gAttState.attQuat.q[1] = new_quat.q[1];
    gAttState.attQuat.q[2] = new_quat.q[2];
    gAttState.attQuat.q[3] = new_quat.q[3];

    /* 同步更新欧拉角 */
    QuatToEuler(&new_quat, &gAttState.attEuler);
}

/**
 * @brief 传感器数据处理
 */
void RunStarSensorSM(void)
{
    /* 状态机状态枚举（本地静态） */
        #define SM_IDLE        0x00u
    #define SM_PROCESSING  0x01u
    #define SM_REPORTING   0x02u
    #define SM_FAULT       0x07u
    #define SM_RECOVERY    0x0Au

    static U8 sm_state       = SM_IDLE;
    static U32       recovery_cnt   = 0U;

    float32 local_vec[3];

    switch (sm_state)
    {
        case SM_IDLE:
            /* 等待有效粗捕获向量 */
            if (mRoughVector.valid == 1U)
            {
                sm_state = SM_PROCESSING;
            }
            break;

        case SM_PROCESSING:
            /* 故障检测：帧超时则进入 FAULT */
            if (diagS == DIAG_NOFRAME)
            {
                sm_state     = SM_FAULT;
                recovery_cnt = 0U;
                break;
            }

            /* 快照当前向量分量（不加锁，仅用于状态机内部评估） */
            local_vec[0] = (float32)mRoughVector.v3[0];
            local_vec[1] = (float32)mRoughVector.v3[1];
            local_vec[2] = (float32)mRoughVector.v3[2];

            /* 更新窗口参数 */
            UpdateWinParams(local_vec);

            /* 从向量更新姿态四元数 */
            UpdateAttitudeFromVec(local_vec);

            /* 记录本帧得分到历史缓冲 */
            RecordAttHistory(gAttState.matchScore);

            sm_state = SM_REPORTING;
            break;

        case SM_REPORTING:
            /* 评估近期识别质量，决定是否降模式 */
            EvalAttHistory();

            sm_state = SM_IDLE;
            break;

        case SM_FAULT:
            /* 连续失败帧计数 */
            recovery_cnt++;
            if (recovery_cnt >= SM_RECOVERY_FRAMES)
            {
                sm_state     = SM_RECOVERY;
                recovery_cnt = 0U;
            }
            break;

        case SM_RECOVERY:
            /* 清零向量，等待中断重新填充 */
            DisableUartIRQ();
            mRoughVector.v3[0] = 0.0f;
            mRoughVector.v3[1] = 0.0f;
            mRoughVector.v3[2] = 1.0f;
            mRoughVector.valid  = 0U;
            EnableUartIRQ();

            /* 重置诊断状态 */
            diagS = DIAG_OK;

            /* 重置匹配得分 */
            gAttState.matchScore = 0.0f;

            /* FSM transition */
            sm_state = SM_IDLE;
            break;

        default:
            sm_state = SM_IDLE;
            break;
    }
}


#ifdef STAR_VEC_VERBOSE

/**
 * @brief 星敏感器处理
 */
static void StarVecDump(void)
{
    FloatBytes fb;
    U32        mode_val;

    /* 输出固定前缀标识（逐字节写 UART DR 寄存器） */
    UART_DR = (U32)'V';
    UART_DR = (U32)'E';
    UART_DR = (U32)'C';
    UART_DR = (U32)':';

    /* 输出 v3[0] 原始字节（小端顺序，4字节十六进制） */
    fb.flt32 = (float32)mRoughVector.v3[0];
    UART_DR = (U32)'X';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';

    /* 输出 v3[1] 原始字节 */
    fb.flt32 = (float32)mRoughVector.v3[1];
    UART_DR = (U32)'Y';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';

    /* 输出 v3[2] 原始字节 */
    fb.flt32 = (float32)mRoughVector.v3[2];
    UART_DR = (U32)'Z';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';

    /* 输出匹配得分原始字节 */
    fb.flt32 = gAttState.matchScore;
    UART_DR = (U32)'S';
    UART_DR = (U32)'C';
    UART_DR = (U32)'=';
    UART_DR = (U32)((fb.u32 >> 24U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >> 16U) & 0xFFU);
    UART_DR = (U32)((fb.u32 >>  8U) & 0xFFU);
    UART_DR = (U32)((fb.u32       ) & 0xFFU);
    UART_DR = (U32)' ';

    /* 输出当前应用模式（1字节） */
    mode_val = (U32)AppMode;
    UART_DR = (U32)'M';
    UART_DR = (U32)'O';
    UART_DR = (U32)'D';
    UART_DR = (U32)'E';
    UART_DR = (U32)'=';
    UART_DR = (U32)('0' + mode_val);

    /* 结束符 */
    UART_DR = (U32)'\r';
    UART_DR = (U32)'\n';
}

#endif /* STAR_VEC_VERBOSE */
