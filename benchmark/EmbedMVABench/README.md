# EmbedMVABench

**嵌入式 C 程序多变量原子性违反基准测试集**

EmbedMVABench 是一个面向嵌入式 C 软件静态/动态分析工具评估的基准测试集，收录了来自真实工业与航空航天项目的多变量原子性违反（Multi-variable Atomicity Violation）缺陷案例，以及对应的正确修复版本（负例）。

---

## 项目简介

| 属性 | 描述 |
|------|------|
| 缺陷类型 | 多变量原子性违反（中断/任务上下文竞争） |
| 总案例数 | 36 个完整嵌入式 C 程序 |
| 正例（含缺陷） | 24 个（L1/L2/L3 三级难度） |
| 负例（已修复） | 12 个（正例衍生修复版） |
| 代码规模 | 平均 1123 行，范围 723~1992 行 |
| 来源 | 24 个来自真实工业代码，12 个为正例衍生修复版 |

**目标应用场景：**
- 静态分析工具检测能力评估
- 形式化验证方法测试
- 大模型辅助代码审查评测
- 并发缺陷检测算法研究

---

## 目录结构

```
EmbedMVABench/
├── cases/
│   ├── L1/                    # 难度1：直接冲突，≤2 变量，0层调用（8 个）
│   │   ├── L1_001/
│   │   │   └── L1_001.c       # 每个案例一个独立文件夹
│   │   ├── L1_002/
│   │   │   └── L1_002.c
│   │   └── ...
│   ├── L2/                    # 难度2：1~2层调用，2~4 变量（10 个）
│   ├── L3/                    # 难度3：≥3层调用或多中断源（6 个）
│   └── negative/              # 负例：正确修复版本（12 个）
├── ground_truth/
│   └── annotations.json       # 标注文件（缺陷位置、共享变量、修复策略）
├── metadata/
│   ├── index.csv              # 案例索引（难度/领域/来源/行数/修复策略）
│   ├── statistics.json        # 数据集统计摘要
│   └── vbug_classification.csv  # 原始 VBUG 分类记录
├── tools/
│   ├── validate.py            # 案例质量验证脚本
│   ├── verify_annotations.py  # 标注行号准确性验证
│   └── compile_check/         # GCC 编译检查工具
│       ├── check_all.py       # 全量语法检查（含 8051 预处理）
│       ├── check_warnings.py  # 警告级别检查
│       ├── reg51.h            # 8051 SFR 桩头文件
│       ├── intrins.h          # Keil C51 内联函数桩
│       └── absacc.h           # XBYTE 宏桩
└── README.md
```

每个案例拥有独立文件夹（如 `cases/L1/L1_001/`），便于后续在同目录下存放测试输出、分析报告等衍生文件。

---

## 案例分级标准

| 级别 | 共享变量数 | 调用链深度 | 变量类型 | 中断源数 | 代码规模 |
|------|-----------|-----------|---------|---------|---------|
| **L1** | ≤2 | 0层（直接访问） | 基本类型 | 1个ISR | 723~889 行 |
| **L2** | 2~4 | 1~2层 | 可含结构体成员 | 1~2个ISR | 914~1179 行 |
| **L3** | ≥3 | ≥3层 | 结构体/数组嵌套 | 多ISR或RTOS | 1429~1992 行 |
| **negative** | — | — | — | — | 729~1511 行 |

---

## annotations.json 字段说明

```json
{
  "version": "1.0",
  "cases": [
    {
      "id": "L1_001",           // 案例唯一标识
      "level": "L1",            // 难度级别
      "source": "real",         // 来源：real/derived
      "origin_file": "VBUG00013672",  // 原始 VBUG 文件名（正例）
      "domain": "航空电子",      // 应用领域
      "processor": "通用",       // 目标处理器类型
      "has_defect": true,       // 是否含缺陷
      "defect": {
        "type": "multi_variable_atomicity_violation",
        "shared_variables": ["BUF_INTERNAL[0]", "BUF_INTERNAL[1]"],
        "conflict": {
          "context_A": {
            "role": "reader",
            "function": "fnInternalHandling",
            "lines": [302, 302]   // 代码行号（从第1行计数）
          },
          "context_B": {
            "role": "writer",
            "function": "ace_int0",
            "lines": [388, 389]
          }
        },
        "fix_strategy": "move_to_interrupt_context"
      }
    },
    {
      "id": "NEG_001",          // 负例标识
      "origin_case": "L1_001",  // 对应的正例
      "has_defect": false,
      "defect": null,
      "fix_note": "修复说明..."
    }
  ]
}
```

### 字段含义

| 字段 | 类型 | 含义 |
|------|------|------|
| `id` | string | 案例唯一标识，格式：`L1_001`、`NEG_001` |
| `level` | string | 难度级别：`L1`/`L2`/`L3` |
| `source` | string | 来源：`real`（真实代码）、`derived`（衍生修复） |
| `origin_file` | string | 正例对应的原始 VBUG 文件 |
| `origin_case` | string | 负例对应的正例 ID |
| `has_defect` | bool | 是否含缺陷 |
| `shared_variables` | array | 竞争共享变量名列表 |
| `context_A/B.role` | string | `reader`（读侧）或 `writer`（写侧） |
| `context_A/B.function` | string | 产生竞争的函数名 |
| `context_A/B.lines` | array | 竞争访问代码的起止行号 `[start, end]` |
| `fix_strategy` | string | 修复策略（见下表） |
| `fix_note` | string | 负例的修复说明 |

### 修复策略枚举

| 策略标识 | 含义 | 案例数 |
|---------|------|-------|
| `disable_global_interrupt` | 关全局中断（`EA=0/EA=1`） | 7 |
| `disable_specific_interrupt` | 关特定中断（`EX1=0/ET0=0` 等） | 12 |
| `move_to_interrupt_context` | 将关键计算移入中断上下文 | 4 |
| `restructure_call_chain` | 重构调用链消除共享 | 7 |
| `introduce_shadow_variable_with_critical_section` | 影子变量 + 临界区 | 6 |

---

## index.csv 字段说明

```
id,level,source,origin_file,domain,processor,has_defect,loc,fix_strategy
```

| 字段 | 含义 |
|------|------|
| `id` | 案例标识 |
| `level` | 难度级别 |
| `source` | 代码来源 |
| `origin_file` | 原始 VBUG 文件或来源案例 |
| `domain` | 应用领域 |
| `processor` | 处理器架构（8051/SPARC/通用） |
| `has_defect` | 是否含缺陷 |
| `loc` | 代码行数 |
| `fix_strategy` | 修复策略 |

---

## 使用方法

### 验证数据集完整性

```bash
cd <benchmark根目录>
python EmbedMVABench/tools/validate.py
```

期望输出：`共 36 个案例，0 个问题 / 所有检查通过！`

### 验证标注行号准确性

```bash
python EmbedMVABench/tools/verify_annotations.py
```

期望输出：`PASSED: All annotation references verified for 24 positive cases`

### 编译检查（需安装 GCC）

```bash
python EmbedMVABench/tools/compile_check/check_all.py
```

该脚本对 8051 代码自动进行语法预处理（去除 `interrupt`/`sbit`/`sfr`/`xdata` 等 Keil C51 关键字），使用 GCC `-fsyntax-only` 模式检查所有文件的语法正确性。

### 读取标注信息

```python
import json

with open('EmbedMVABench/ground_truth/annotations.json', encoding='utf-8') as f:
    data = json.load(f)

# 获取所有正例
positive_cases = [c for c in data['cases'] if c['has_defect']]

# 获取某案例的共享变量
case = next(c for c in data['cases'] if c['id'] == 'L2_001')
print(case['defect']['shared_variables'])

# 获取缺陷所在行号
print(case['defect']['conflict']['context_A']['lines'])
```

### 读取案例列表

```python
import csv

with open('EmbedMVABench/metadata/index.csv', encoding='utf-8') as f:
    cases = list(csv.DictReader(f))

# 按难度分组
l1_cases = [c for c in cases if c['level'] == 'L1']
```

---

## 数据统计

| 类别 | 数量 |
|------|------|
| 总案例数 | 36 |
| 正例（含缺陷） | 24 |
| 负例（已修复） | 12 |
| L1 难度案例 | 8 |
| L2 难度案例 | 10 |
| L3 难度案例 | 6 |
| 来自真实代码 | 24 |
| 衍生修复版本 | 12 |
| 平均代码行数 | 1122 行 |

**处理器分布：** 8051（15 例）、通用 32 位（14 例）、SPARC（7 例）

**应用领域分布：** 航空航天/卫星（27 例）、工业控制（5 例）、嵌入式存储/RTOS（4 例）

**负例覆盖：** 每个难度级别均有负例 — L1 级 2 个、L2 级 4 个、L3 级 4 个、L1→SPARC 交叉 1 个、L2→通用 交叉 1 个

---

## 来源说明

本数据集的正例案例核心缺陷片段来自真实工业项目代码库（经脱敏处理），涵盖：

- **航空电子：** MIL-STD-1553B 总线通信、RS422 数据帧处理
- **航天系统：** 卫星 GNC 时间管理、遥测数据广播、载荷相机控制、星时同步、姿态四元数处理
- **工业控制：** 温控 PID 系统、伺服电机角度测量、工业继电器控制、光学传感器标定
- **嵌入式系统：** RTOS AHB 总线管理、星载存储备份、电源管理遥测、CAN/UART 通信

所有代码均经过脱敏处理：变量名保留业务含义但不含公司/型号标识，地址常量均为虚构值，函数逻辑完整保留缺陷特征。

负例（`negative/` 目录）是对应正例的人工修复版本，采用嵌入式 C 常用的原子性保护策略，可作为"正确程序"的参照。

---

*EmbedMVABench v1.1 — 最后更新 2026-03-12*
