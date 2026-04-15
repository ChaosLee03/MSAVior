# DG-MUVI 关联变量识别方法 —— 完整分析流程手册

> 本文档详细描述了 DG 项目中基于 MUVI 思想实现的关联变量识别方法的完整逻辑。
> 目标：给定一个 .c 文件的源代码，你（大模型）能够**严格模拟**该工具的行为，输出该工具会识别的关联变量对。

---

## 一、方法概述

该工具由两个核心阶段组成：

1. **阶段A：多维特征提取**（对应工具 `llvm-multivariable-find`）—— 提取全局变量/数组元素/结构体字段，计算每对变量之间的5个特征维度
2. **阶段B：关联规则挖掘**（对应工具 `llvm-muvi-detector`）—— 基于代码距离窗口的变量共现，通过 FPclose 闭合关联规则挖掘，输出关联变量对

两个阶段是**并行且独立**的两种分析路径，各自输出关联变量结果。在模拟时，你应该**同时执行两个阶段**，并将结果合并。

---

## 二、阶段A：多维特征提取（llvm-multivariable-find）

### A1. 目标变量识别

扫描整个 .c 文件，识别以下类型的变量：

#### 类型1：普通全局变量
- 在所有函数外部、文件作用域内用 `int x;`、`char *p;` 等方式声明的变量
- **排除**：局部变量（函数内部声明的）、函数参数、`static` 局部变量
- **排除**：编译器内建变量（如 `__func__`）

#### 类型2：全局数组的元素
- 如果存在全局数组 `int arr[10];`，则每个被实际访问的元素视为独立变量
- 命名规则：`arr[0]`、`arr[1]`、`arr[2]` 等
- **仅考虑常量下标**的访问，如 `arr[3] = 5;` 会产生变量 `arr[3]`
- 变量下标的访问（如 `arr[i]`）**被忽略**（工具代码中明确跳过了 GetElementPtrInst 中 operand(0) 为 GlobalVariable 的情况）
- 多维数组同理：`matrix[1][2]` 命名为 `matrix[1][2]`

#### 类型3：全局结构体的字段
- 如果存在全局结构体变量 `struct S s;`，则其字段命名为 `s.field1`、`s.field2`
- 嵌套结构体：`s.inner.field` 用点号连接
- 结构体数组：`arr[0].field1`

#### 重要过滤规则
- 忽略名为 `llvm.global_ctors`、`llvm.global_dtors`、`llvm.dbg.declares` 的变量（这些是 LLVM 内部变量，但在源码层面你不会看到它们，可忽略此条）
- **仅关注全局变量和全局结构体/数组的成员**，不关注局部变量

### A2. 逐函数访问信息收集

对每个函数，收集：
- **函数名**
- **函数起止行号**（startline = 函数声明行, endline = 函数体最后一条语句行）
- **变量访问列表**：每次访问记录 `(变量名, 行号)`

收集规则：
- 遍历函数体中的每条语句
- 如果语句中引用了目标变量（全局变量/全局数组元素/全局结构体字段），记录一次访问
- 同一变量在同一行被多次访问只算行号出现一次
- 如果一条语句同时涉及多个全局变量，每个全局变量各记一次

### A3. 计算5个特征维度

对每一对变量 (V1, V2)，计算以下5个特征：

#### 特征1：代码距离近的次数（Close Proximity Count）

```
codedistance = 10  // 阈值：10行
```

**算法**：
1. 对每个函数，将该函数内所有变量访问按行号排序
2. 对于该函数内 V1 的每次访问（行号 L1）和 V2 的每次访问（行号 L2）：
   - 如果 `|L1 - L2| <= codedistance / 2`（即 5 行以内），计数 +1
3. 累加所有函数中的计数，取 V1→V2 和 V2→V1 方向的最大值

**判定**：Close Proximity Count > 0 表示两个变量在代码中经常靠近出现。

#### 特征2：是否为兄弟元素（Is Sibling）

**定义**：两个变量共享相同的"基础名称"（basename）。

**规则**：
- 仅适用于数组元素和结构体字段（type != 普通全局变量）
- 同一全局数组的不同元素互为兄弟。例如 `arr[0]` 和 `arr[1]` 的 basename 都是 `arr`，所以互为兄弟
- 同一全局结构体数组的不同索引元素互为兄弟。例如 `sa[0].x` 和 `sa[1].x` 的 basename 都是 `sa`
- 普通全局变量之间**不计算**兄弟关系

**判定**：Is Sibling = 1 或 0

#### 特征3：是否有相似命名（Has Similar Naming）

使用以下算法判断两个变量名 name1 和 name2 是否"相似"：

```
function isCloseName(name1, name2):
    // 规则1：一个名称是另一个的前缀
    if name2.startsWith(name1) or name1.startsWith(name2):
        return true

    // 找到最长公共前缀
    prefix = longestCommonPrefix(name1, name2)
    if prefix is empty:
        return false

    lastchar = prefix 的最后一个字符

    // 规则2：公共前缀的最后一个字符是分隔符
    if lastchar in ['[', '.', '_']:
        return true

    // 规则3：驼峰命名分界点
    // 即公共前缀最后是小写字母，下一个字符是大写字母，或反之
    i = len(prefix)
    if i < min(len1, len2):
        if isAlpha(lastchar) and ((isLower(lastchar) and isUpper(name1[i])) or (isUpper(lastchar) and isLower(name1[i]))):
            return true

    // 规则4：公共前缀后面跟的是不同数字
    if i < min(len1, len2):
        if isAlpha(lastchar) and isDigit(name1[i]) and isDigit(name2[i]):
            return true

    return false
```

**示例**：
- `count_a` 和 `count_b` → 公共前缀 `count_`，lastchar=`_` → **相似**
- `dataLen` 和 `dataPtr` → 公共前缀 `data`，lastchar=`a`，下一位 `L`(大写) vs `P`(大写)... → 需要检查驼峰规则。`a` 是小写，`L` 是大写 → **相似**
- `arr[0]` 和 `arr[1]` → `arr[0]`.startsWith(`arr[1]`) 不成立，但公共前缀是 `arr[`，lastchar=`[` → **相似**
- `x` 和 `y` → 公共前缀为空 → **不相似**
- `total1` 和 `total2` → 公共前缀 `total`，lastchar=`l`，下一位 `1` 和 `2` 都是数字 → **相似**
- `rx_bytes` 和 `tx_bytes` → 公共前缀为空（`r` != `t`） → **不相似**

**判定**：Has Similar Naming = 1 或 0

#### 特征4：控制依赖次数（Control Dependency Count）

**简化模拟规则**（原工具通过 SDG 的控制依赖边计算，这里给出等价的源码级判断）：

如果变量 V1 出现在一个 `if`/`switch`/`while`/`for` 的**条件表达式**中，而变量 V2 出现在该条件控制的**语句体**中，则 V2 对 V1 有一次控制依赖。

**具体算法**：
1. 找到所有条件语句（if/while/for/switch）
2. 对于条件表达式中出现的每个全局变量 Vc：
   - 在该条件语句控制的语句体中，出现的每个全局变量 Vb，记录：Vb 对 Vc 有控制依赖
3. 对 V1→V2 和 V2→V1 两个方向分别计数，然后相加

**示例**：
```c
if (flag > 0) {     // flag 出现在条件中
    data = 10;       // data 出现在 if 体中
    count++;         // count 出现在 if 体中
}
```
则 `(data, flag)` 控制依赖 +1，`(count, flag)` 控制依赖 +1。

**判定**：Control Dependency Count = 总次数

#### 特征5：数据依赖次数（Data Dependency Count）

**简化模拟规则**（原工具通过 SDG 的 use-def 链和 memory dependency 边计算）：

如果变量 V1 的值被用来**计算** V2 的值（直接或间接赋值），或者 V1 和 V2 出现在**同一条赋值语句**中（一个在左侧、一个在右侧），则存在一次数据依赖。

**具体算法**：
1. 对于每条赋值语句 `Vleft = expr;`：
   - 如果 Vleft 是全局变量，且 expr 中引用了另一个全局变量 Vright：
     - 记录 Vleft 与 Vright 之间有数据依赖
2. 对于函数调用 `func(V1, V2)`：
   - 如果 V1 和 V2 都是全局变量，且作为同一个函数调用的参数出现，记录 V1 与 V2 之间有数据依赖
3. 对 V1→V2 和 V2→V1 两个方向分别计数，然后相加

**判定**：Data Dependency Count = 总次数

### A4. 输出格式

对每一对变量 (V1, V2)，输出：

```json
{
    "Variable Pair": "filename:::V1,filename:::V2",
    "left times": <V1总出现次数>,
    "right times": <V2总出现次数>,
    "Close Proximity Count": <代码距离近的次数>,
    "Is Sibling": <0或1>,
    "Has Similar Naming": <0或1>,
    "Control Dependency Count": <控制依赖次数>,
    "Data Dependency Count": <数据依赖次数>
}
```

### A5. 阶段A关联变量判定规则

该工具本身不在 `llvm-multivariable-find` 中做最终判定，而是输出特征向量供后续使用。但基于工具的使用方式和代码逻辑，**判定一对变量是关联变量的规则**如下：

**满足以下任意一个条件即视为关联变量**：
1. Close Proximity Count >= 2（在多个位置代码距离很近）
2. Is Sibling = 1 **且** Close Proximity Count >= 1（兄弟元素且至少共现一次）
3. Has Similar Naming = 1 **且** Close Proximity Count >= 1（相似命名且至少共现一次）
4. Control Dependency Count >= 1 **且** Close Proximity Count >= 1（有控制依赖且靠近出现）
5. Data Dependency Count >= 1 **且** Close Proximity Count >= 1（有数据依赖且靠近出现）

> 注意：如果两个全局变量在整个文件中**从未在同一个函数内靠近（10行以内）出现过**（Close Proximity Count = 0），即使有相似命名或兄弟关系，该工具也**不太可能**将它们识别为强关联。

---

## 三、阶段B：关联规则挖掘（llvm-muvi-detector）

### B1. 目标变量识别

与阶段A完全一致（A1节），识别同样类型的全局变量/数组元素/结构体字段。

### B2. 为每个变量分配唯一 ID

按照变量**首次出现的顺序**，依次分配从 0 开始的整数 ID。

例如：第一次遇到 `flag` → ID=0，第一次遇到 `counter` → ID=1，等等。

### B3. 逐函数收集访问记录

对每个函数：
- 记录函数名、startline、endline
- 遍历每条语句，如果涉及目标变量，记录 `Record(变量名, 行号, 读写类型)`
  - `StoreInst`（赋值语句左侧）→ write
  - 其他所有引用 → read
- 将 Record 加入该函数的变量集合

### B4. 滑动窗口分组（生成事务数据库）

```
linedistance = 10  // 窗口大小：10行
```

**算法**：
```
对每个函数 F:
    对每个行号 line（从 F.startline 到 F.endline）:
        window = {}
        对 F 中的每个变量访问记录 r:
            if r.line >= line AND r.line < line + 10 AND r.line <= F.endline:
                window.add(r)
        if window 非空:
            // 提取 window 中所有不同变量的 ID 集合
            id_set = {r.id for r in window}
            输出 id_set 作为一条事务
```

**关键点**：
- 窗口是**逐行滑动**的，不是跳跃的
- 同一组变量可能因为在不同行开始的窗口中而被重复输出多次，这会**增大 support 值**
- 一个窗口内如果只有一个变量，该事务也会被记录（但对关联规则挖掘无贡献）

### B5. FPclose 闭合关联规则挖掘

**输入**：B4 生成的事务数据库（每行是一组变量 ID）

**参数**：
- MinSupport = 0（即不做支持度过滤，所有共现都保留）
- MinConfidence = 0（即不做置信度过滤）

**挖掘算法**：Closed Association Rules (using FPclose)

**输出格式**（每条规则）：
```
LHS ==> RHS #SUP: <support> #CONF: <confidence>
```

其中：
- `LHS`：前项（一个或多个变量 ID）
- `RHS`：后项（一个或多个变量 ID）
- `support`：LHS ∪ RHS 在事务数据库中同时出现的次数
- `confidence`：support / (LHS 单独出现的次数)

**因为 MinSupport=0 且 MinConfidence=0**，实际上**所有在某个窗口中共同出现过至少1次的变量对/组都会被挖掘出来**。

### B6. 排序

将所有挖掘出的关联规则按以下优先级排序：
1. **confidence 降序**（confidence 越高越靠前）
2. confidence 相同时，**support 降序**（support 越高越靠前）

### B7. 过滤

排序后，按序号（从 0 开始）遍历所有规则：
- 如果序号是**质数**（2, 3, 5, 7, 11, 13, ...），则**跳过**该条规则
- 否则保留

> 注：这个过滤逻辑在代码中确实存在（`isprime` 函数），可能是一种实验性的降采样策略。在模拟时你也应该执行此步骤，但注意代码中有 bug：`num` 变量只在 `isprime(num)` 为 true 时才递增，导致一旦遇到非质数就不再递增。**实际效果是只过滤掉序号为 2 的规则**（因为 num=0 不是质数→输出→num 不变，num=0 不是质数→输出→...  永远不递增到质数）。所以这个过滤实际上**几乎不生效**。在模拟时可以忽略此过滤步骤。

### B8. 阶段B关联变量判定规则

**最终输出的每条关联规则中涉及的变量互为关联变量**。

由于 MinSupport=0、MinConfidence=0，判定条件实际上非常宽松：

**只要两个全局变量在任何一个10行滑动窗口中共同出现过，它们就会被识别为关联变量。**

但排序后，靠前的结果更可信：
- confidence 越高 → 意味着只要 V1 出现，V2 几乎总是伴随出现
- support 越高 → 意味着共现次数越多

---

## 四、完整模拟执行流程

给定一个 .c 文件，按以下步骤执行：

### 步骤1：识别所有目标变量

1. 找出所有**全局变量**（文件作用域，非 static 局部变量）
2. 找出所有**全局数组的常量下标访问**，每个 `arr[常量]` 视为一个独立变量
3. 找出所有**全局结构体的字段访问**，每个 `s.field` 视为一个独立变量
4. 为每个变量按首次出现顺序分配 ID（从 0 开始）

### 步骤2：逐函数收集访问信息

对每个函数：
1. 确定函数的 startline 和 endline
2. 逐语句扫描，记录每次对目标变量的访问：(变量名, 行号, 读/写)
   - 赋值语句的左侧 → write
   - 其他引用 → read

### 步骤3：执行阶段B（关联规则挖掘）

1. **生成事务数据库**：对每个函数的每一行，生成一个10行窗口内的变量 ID 集合
2. **计算关联规则**：对事务数据库中所有至少共现1次的变量对，计算：
   - `support(V1, V2)` = V1 和 V2 同时出现在同一个窗口中的次数
   - `support(V1)` = V1 出现在任何窗口中的次数
   - `confidence(V1 → V2)` = support(V1, V2) / support(V1)
   - `confidence(V2 → V1)` = support(V1, V2) / support(V2)
3. **排序**：按 confidence 降序，相同则按 support 降序

### 步骤4：执行阶段A（多维特征分析）

对每一对目标变量 (V1, V2)，计算5个特征：
1. Close Proximity Count（10行内共现次数，按行号差 ≤ 5 计算）
2. Is Sibling（是否同数组/同结构体数组的不同元素）
3. Has Similar Naming（按 isCloseName 算法判断）
4. Control Dependency Count（V1 在条件中，V2 在条件体中的次数，双向累加）
5. Data Dependency Count（V1 被用来计算 V2 的次数，双向累加）

### 步骤5：合并输出关联变量

**最终关联变量列表** = 阶段B 识别的关联 ∪ 阶段A 识别的关联

优先级排序：
1. 阶段B中 confidence 高 + support 高的排前面
2. 阶段A中多个特征维度都非零的排前面

---

## 五、输出格式规范

对于每个 .c 文件，输出结果应为：

```
=== 文件名: xxx.c ===

【识别的目标变量】
ID=0: variable_name1 (类型: 全局变量/数组元素/结构体字段)
ID=1: variable_name2 (类型: ...)
...

【关联变量对】(按置信度排序)
排名 | 变量对            | confidence | support | 代码距离 | 兄弟 | 相似命名 | 控制依赖 | 数据依赖
1    | (var1, var2)      | 1.00       | 5       | 3        | 1    | 1        | 2        | 1
2    | (var3, var4)      | 0.80       | 4       | 2        | 0    | 1        | 0        | 3
...

【最终关联变量组】
组1: {var1, var2}
组2: {var3, var4, var5}
...
```

---

## 六、重要注意事项和边界情况

### 6.1 该工具的盲点（会漏检的情况）

1. **局部变量之间的关联**：工具完全忽略局部变量，如果两个关联变量都是局部的，不会被发现
2. **通过指针间接访问的全局变量**：`int *p = &global_var; *p = 5;` —— 这种间接写入不会被识别为对 `global_var` 的写入
3. **变量下标为变量的数组访问**：`arr[i]` 被完全忽略
4. **跨文件关联**：工具按单个编译单元（.c 文件）分析，不会发现跨文件的关联
5. **仅在一个函数中出现的变量**：虽然会被收集，但如果整个程序只有一个函数，滑动窗口内的共现仍可被识别
6. **函数调用传播**：工具**不做**调用图的 Acc_Set 传播（与 MUVI 论文不同）。即如果 `func1()` 内调用了 `func2()`，`func2()` 内访问的全局变量不会被加入 `func1()` 的 Acc_Set

### 6.2 该工具的特点（可能误报的情况）

1. **MinSupport=0 导致大量弱关联**：只要两个全局变量在10行窗口内共现一次就会被报告
2. **滑动窗口逐行滑动导致 support 膨胀**：两个相邻行的变量会被多个窗口覆盖，support 被人为放大
3. **相似命名的误判**：`data` 和 `database` 会被判为相似命名（前缀包含关系）
4. **不区分读写类型**：关联规则挖掘阶段不区分 read/write（虽然收集了读写信息），与 MUVI 论文的9种关联类型不同

### 6.3 与 MUVI 论文的关键差异（在论文对比中需要说明）

| 方面 | MUVI 论文 | DG-MUVI 实现 |
|------|-----------|-------------|
| 变量类型 | 全局变量 + 结构体字段 | 全局变量 + 数组元素 + 结构体字段 |
| 共现粒度 | 函数级 | 10行滑动窗口 |
| 调用图传播 | 向上传播一层 callee 的直接访问 | 不传播 |
| 读写区分 | 区分9种关联类型 | 不区分 |
| 挖掘算法 | FPclose 频繁项集 → 再生成规则 | 直接 FPclose 闭合关联规则 |
| 剪枝 | MinSupport + MinConfidence + 热门变量过滤 | 几乎无剪枝 (support=0, confidence=0) |
| 额外特征 | 无 | 兄弟关系、相似命名、控制依赖、数据依赖 |
| 并发检测 | 多线程 lockset/happens-before | 嵌入式中断场景 |

---

## 七、模拟执行示例

### 输入代码 (example.c)：

```c
int counter = 0;
int total = 0;
int flag = 0;
int data[4] = {0};

void update() {           // line 6
    counter++;             // line 7, write counter
    total += counter;      // line 8, write total, read counter
    flag = 1;              // line 9, write flag
}

void process() {           // line 12
    if (flag) {            // line 13, read flag
        data[0] = counter; // line 14, write data[0], read counter
        data[1] = total;   // line 15, write data[1], read total
    }
}

void reset() {             // line 19
    counter = 0;           // line 20, write counter
    total = 0;             // line 21, write total
    flag = 0;              // line 22, write flag
    data[0] = 0;           // line 23, write data[0]
    data[1] = 0;           // line 24, write data[1]
}
```

### 步骤1：识别目标变量

| ID | 变量名 | 类型 |
|----|--------|------|
| 0 | counter | 全局变量 |
| 1 | total | 全局变量 |
| 2 | flag | 全局变量 |
| 3 | data[0] | 数组元素 |
| 4 | data[1] | 数组元素 |

### 步骤2：逐函数收集

**update()**: startline=6, endline=9
- (counter, 7, write), (total, 8, write), (counter, 8, read), (flag, 9, write)

**process()**: startline=12, endline=16
- (flag, 13, read), (data[0], 14, write), (counter, 14, read), (data[1], 15, write), (total, 15, read)

**reset()**: startline=19, endline=24
- (counter, 20, write), (total, 21, write), (flag, 22, write), (data[0], 23, write), (data[1], 24, write)

### 步骤3：滑动窗口 + 挖掘

**update() 的窗口** (linedistance=10, 函数仅跨6-9行):
- line=6: 窗口[6,16)∩[6,9] → {counter(7), total(8), counter(8), flag(9)} → IDs: {0,1,2}
- line=7: 窗口[7,17)∩[7,9] → {counter(7), total(8), counter(8), flag(9)} → IDs: {0,1,2}
- line=8: 窗口[8,18)∩[8,9] → {total(8), counter(8), flag(9)} → IDs: {0,1,2}
- line=9: 窗口[9,19)∩[9,9] → {flag(9)} → IDs: {2}

**process() 的窗口**:
- line=12: 窗口[12,22)∩[12,16] → {flag(13), data[0](14), counter(14), data[1](15), total(15)} → IDs: {0,1,2,3,4}
- line=13 到 line=16: 类似，都包含 {0,1,2,3,4} 或其子集

**reset() 的窗口**:
- line=19: 窗口[19,29)∩[19,24] → {counter(20), total(21), flag(22), data[0](23), data[1](24)} → IDs: {0,1,2,3,4}
- line=20~24: 类似

**关联规则计算**（简化，取主要结果）：

| 变量对 | support | conf(→) | conf(←) |
|--------|---------|---------|---------|
| (counter, total) | >=6 | 高 | 高 |
| (counter, flag) | >=5 | 高 | 高 |
| (total, flag) | >=5 | 高 | 高 |
| (data[0], data[1]) | >=4 | 高 | 高 |
| (counter, data[0]) | >=3 | 中 | 中 |
| (counter, data[1]) | >=3 | 中 | 中 |
| (total, data[0]) | >=3 | 中 | 中 |
| (total, data[1]) | >=3 | 中 | 中 |
| (flag, data[0]) | >=3 | 中 | 中 |
| (flag, data[1]) | >=3 | 中 | 中 |

### 步骤4：多维特征分析

| 变量对 | 距离近 | 兄弟 | 相似命名 | 控制依赖 | 数据依赖 |
|--------|--------|------|----------|----------|----------|
| (counter, total) | 3 | 0 | 0 | 0 | 2 |
| (counter, flag) | 2 | 0 | 0 | 1 | 0 |
| (total, flag) | 2 | 0 | 0 | 0 | 0 |
| (data[0], data[1]) | 3 | 1 | 1 | 0 | 0 |
| (counter, data[0]) | 2 | 0 | 0 | 1 | 1 |
| (total, data[1]) | 2 | 0 | 0 | 1 | 1 |

### 步骤5：最终输出

```
【关联变量对】(按置信度排序)
排名 | 变量对              | conf  | sup | 距离 | 兄弟 | 命名 | 控制 | 数据
1    | (counter, total)    | ~1.0  | 6   | 3    | 0    | 0    | 0    | 2
2    | (counter, flag)     | ~0.8  | 5   | 2    | 0    | 0    | 1    | 0
3    | (total, flag)       | ~0.8  | 5   | 2    | 0    | 0    | 0    | 0
4    | (data[0], data[1])  | ~0.9  | 4   | 3    | 1    | 1    | 0    | 0
5    | (counter, data[0])  | ~0.5  | 3   | 2    | 0    | 0    | 1    | 1
...

【最终关联变量组】
组1: {counter, total}         — 强关联（高共现 + 数据依赖）
组2: {counter, total, flag}   — 中等关联（高共现）
组3: {data[0], data[1]}       — 强关联（兄弟 + 相似命名 + 高共现）
```

---

## 八、快速参考卡片

当你需要快速分析一个 .c 文件时，按以下优先级检查：

```
□ 1. 列出所有全局变量、全局数组元素（常量下标）、全局结构体字段
□ 2. 对每个函数，按行号记录每个目标变量的访问
□ 3. 对每对变量，检查是否在同一函数的10行窗口内共现
□ 4. 共现过 → 计算 support 和 confidence
□ 5. 检查额外特征：兄弟？相似命名？控制依赖？数据依赖？
□ 6. 按 confidence 降序输出所有关联变量对
□ 7. 将高置信度的关联对合并为关联变量组
```
