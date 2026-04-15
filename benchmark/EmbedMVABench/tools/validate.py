#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EmbedMVABench 案例质量验证脚本"""
import os, json, csv, sys
import io
if sys.stdout.encoding and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

CASES_DIR = "EmbedMVABench/cases"
GT_FILE   = "EmbedMVABench/ground_truth/annotations.json"
IDX_FILE  = "EmbedMVABench/metadata/index.csv"

MIN_LOC = 200
MAX_LOC_DEFAULT = 1200   # L1/L2 最大行数
MAX_LOC_L3 = 2000        # L3 最大行数（复杂场景，允许更多代码）
MAX_LOC_NEG = 2000       # negative 最大行数（含L3衍生大型负例）

FORBIDDEN = ["// BUG", "// DEFECT", "// TODO", "// 缺陷", "// 修复", "// bug", "BUG_HERE", "DEFECT_HERE"]

def check_case_file(filepath, max_loc=None):
    if max_loc is None:
        max_loc = MAX_LOC_DEFAULT
    errors = []
    with open(filepath, encoding="utf-8") as f:
        lines = f.readlines()
    loc = len(lines)
    if loc < MIN_LOC:
        errors.append(f"行数过少: {loc} < {MIN_LOC}")
    if loc > max_loc:
        errors.append(f"行数过多: {loc} > {max_loc}")
    content = "".join(lines)
    for kw in FORBIDDEN:
        if kw in content:
            errors.append(f"含禁止关键词: '{kw}'")
    if "main(" not in content:
        errors.append("缺少 main() 函数")
    has_isr = any(k in content for k in ["interrupt", "_ISR", "isr(", "ISR(", "irq_handler"])
    if not has_isr:
        errors.append("未发现中断处理例程关键词")
    return loc, errors

def check_annotations(gt_data, all_case_ids):
    errors = []
    annotated = {c["id"] for c in gt_data.get("cases", [])}
    for cid in all_case_ids:
        if cid not in annotated:
            errors.append(f"缺少标注: {cid}")
    for case in gt_data.get("cases", []):
        if case.get("has_defect"):
            if not case.get("defects") or len(case["defects"]) == 0:
                errors.append(f"正例 {case['id']} 缺少 defects 数组")
    return errors

def main():
    # 切换到项目根目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(script_dir)  # EmbedMVABench/
    project_dir = os.path.dirname(root_dir)
    os.chdir(project_dir)

    all_case_ids = []
    total_errors = []

    print("=" * 60)
    print("EmbedMVABench 验证脚本")
    print("=" * 60)

    for level in ["L1", "L2", "L3", "negative"]:
        level_dir = os.path.join(CASES_DIR, level)
        if not os.path.exists(level_dir):
            continue
        subdirs = sorted(d for d in os.listdir(level_dir)
                         if os.path.isdir(os.path.join(level_dir, d)))
        if subdirs:
            print(f"\n[{level}] {len(subdirs)} 个案例:")
        if level == "L3":
            max_loc = MAX_LOC_L3
        elif level == "negative":
            max_loc = MAX_LOC_NEG
        else:
            max_loc = MAX_LOC_DEFAULT
        for cid in subdirs:
            fpath = os.path.join(level_dir, cid, cid + ".c")
            all_case_ids.append(cid)
            loc, errs = check_case_file(fpath, max_loc)
            if errs:
                for e in errs:
                    total_errors.append(f"[{cid}] {e}")
                print(f"  FAIL {cid:15s} ({loc:4d} 行) — {errs[0]}")
            else:
                print(f"  OK   {cid:15s} ({loc:4d} 行)")

    print(f"\n[标注检查]")
    if os.path.exists(GT_FILE):
        with open(GT_FILE, encoding="utf-8") as f:
            gt = json.load(f)
        ann_errors = check_annotations(gt, all_case_ids)
        if ann_errors:
            total_errors.extend(ann_errors)
            for e in ann_errors:
                print(f"  MISSING: {e}")
        else:
            print(f"  OK   所有 {len(all_case_ids)} 个案例均有标注")
    else:
        print(f"  WARN annotations.json 不存在")

    print("\n" + "=" * 60)
    print(f"共 {len(all_case_ids)} 个案例，{len(total_errors)} 个问题")
    if total_errors:
        print("\n错误列表：")
        for e in total_errors:
            print(f"  ERROR: {e}")
        sys.exit(1)
    else:
        print("✅ 所有检查通过！")

if __name__ == "__main__":
    main()
