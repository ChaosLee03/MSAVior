#!/usr/bin/env python3
"""EmbedMVABench Final Audit - 6-dimension comprehensive check (D1-D4 automated)."""

import csv
import json
import os
import re
import sys
from collections import Counter

ROOT = os.path.join(os.path.dirname(__file__), "..")
CASES_DIR = os.path.join(ROOT, "cases")

issues = []  # (severity, dimension, case_id, message)

def add_issue(severity, dim, case_id, msg):
    issues.append((severity, dim, case_id, msg))

def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def load_csv(path):
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def read_lines(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.readlines()

def get_case_path(cid, idx):
    level = "negative" if cid.startswith("NEG_") else idx["level"]
    return os.path.join(CASES_DIR, level, cid, f"{cid}.c")

# ============================================================
# Load data
# ============================================================
index_rows = load_csv(os.path.join(ROOT, "metadata", "index.csv"))
annotations_raw = load_json(os.path.join(ROOT, "ground_truth", "annotations.json"))
annotations = annotations_raw["cases"]
stats = load_json(os.path.join(ROOT, "metadata", "statistics.json"))
readme_text = open(os.path.join(ROOT, "README.md"), "r", encoding="utf-8").read()

index_by_id = {r["id"]: r for r in index_rows}
ann_by_id = {a["id"]: a for a in annotations}

ALL_IDS = sorted(index_by_id.keys())

# ============================================================
# D1: Metadata Consistency
# ============================================================
print("=" * 70)
print("D1: Metadata Consistency")
print("=" * 70)

# D1.1: index.csv ↔ annotations.json field matching
for cid in ALL_IDS:
    idx = index_by_id[cid]
    ann = ann_by_id.get(cid)
    if ann is None:
        add_issue("ERROR", "D1", cid, "Missing in annotations.json")
        continue

    # For NEG cases, annotations.json stores the origin level, not "negative"
    # So compare level only for positive cases
    if not cid.startswith("NEG_"):
        if idx["level"] != ann["level"]:
            add_issue("ERROR", "D1", cid,
                      f"level mismatch: index='{idx['level']}' vs ann='{ann['level']}'")

    # domain and processor should always match
    for field in ["domain", "processor"]:
        if idx[field] != ann[field]:
            add_issue("ERROR", "D1", cid,
                      f"{field} mismatch: index='{idx[field]}' vs ann='{ann[field]}'")

    # has_defect
    if idx["has_defect"].lower() != str(ann["has_defect"]).lower():
        add_issue("ERROR", "D1", cid,
                  f"has_defect mismatch: index='{idx['has_defect']}' vs ann='{ann['has_defect']}'")

    # fix_strategy: for positive cases, check first defect's fix_strategy
    if idx["has_defect"].lower() == "true":
        defects = ann.get("defects", []) or []
        first_defect = defects[0] if defects else {}
        ann_fs = first_defect.get("fix_strategy", "")
        idx_fs = idx.get("fix_strategy", "")
        if idx_fs != ann_fs:
            add_issue("ERROR", "D1", cid,
                      f"fix_strategy mismatch: index='{idx_fs}' vs ann='{ann_fs}'")

# D1.2: Negative case origin consistency
for cid in ALL_IDS:
    if not cid.startswith("NEG_"):
        continue
    idx = index_by_id[cid]
    ann = ann_by_id[cid]
    origin_id = idx.get("origin_file", "")
    ann_origin = ann.get("origin_case", "")

    if origin_id != ann_origin:
        add_issue("ERROR", "D1", cid,
                  f"Origin mismatch: index='{origin_id}' vs ann='{ann_origin}'")

    # NEG should inherit domain/processor/fix_strategy from origin
    if origin_id in index_by_id:
        origin = index_by_id[origin_id]
        for field in ["domain", "processor", "fix_strategy"]:
            if idx[field] != origin[field]:
                add_issue("ERROR", "D1", cid,
                          f"NEG '{field}' differs from origin {origin_id}: "
                          f"'{idx[field]}' vs '{origin[field]}'")

# D1.3: statistics.json consistency with index.csv
actual_total = len(index_rows)
actual_positive = sum(1 for r in index_rows if r["has_defect"].lower() == "true")
actual_negative = actual_total - actual_positive
locs = [int(r["loc"]) for r in index_rows]
actual_avg = round(sum(locs) / len(locs), 1)

if stats["total"] != actual_total:
    add_issue("ERROR", "D1", "statistics", f"total: {stats['total']} vs actual {actual_total}")
if stats["positive"] != actual_positive:
    add_issue("ERROR", "D1", "statistics", f"positive: {stats['positive']} vs actual {actual_positive}")
if stats["negative"] != actual_negative:
    add_issue("ERROR", "D1", "statistics", f"negative: {stats['negative']} vs actual {actual_negative}")
if abs(stats["avg_loc"] - actual_avg) > 0.2:
    add_issue("ERROR", "D1", "statistics", f"avg_loc: {stats['avg_loc']} vs actual {actual_avg}")
if stats["loc_range"] != [min(locs), max(locs)]:
    add_issue("ERROR", "D1", "statistics",
              f"loc_range: {stats['loc_range']} vs actual [{min(locs)}, {max(locs)}]")

# by_processor counts
proc_counts = Counter(r["processor"] for r in index_rows)
for proc, count in proc_counts.items():
    if stats["by_processor"].get(proc) != count:
        add_issue("ERROR", "D1", "statistics",
                  f"by_processor[{proc}]: {stats['by_processor'].get(proc)} vs actual {count}")

# by_level counts
level_counts = Counter(r["level"] for r in index_rows)
for lvl, count in level_counts.items():
    if stats["by_level"].get(lvl) != count:
        add_issue("ERROR", "D1", "statistics",
                  f"by_level[{lvl}]: {stats['by_level'].get(lvl)} vs actual {count}")

# by_fix_strategy counts
fs_counts = Counter(r["fix_strategy"] for r in index_rows)
for fs, count in fs_counts.items():
    if stats["by_fix_strategy"].get(fs) != count:
        add_issue("ERROR", "D1", "statistics",
                  f"by_fix_strategy[{fs}]: {stats['by_fix_strategy'].get(fs)} vs actual {count}")

# D1.4: LOC in index.csv matches actual file line count
for cid in ALL_IDS:
    idx = index_by_id[cid]
    fpath = get_case_path(cid, idx)
    if not os.path.exists(fpath):
        add_issue("ERROR", "D1", cid, f"File not found: {fpath}")
        continue
    actual_lines = len(read_lines(fpath))
    csv_loc = int(idx["loc"])
    if actual_lines != csv_loc:
        add_issue("ERROR", "D1", cid, f"LOC mismatch: index.csv={csv_loc}, actual={actual_lines}")

# D1.5: README LOC ranges match actual data
for lvl in ["L1", "L2", "L3"]:
    lvl_locs = [int(r["loc"]) for r in index_rows if r["level"] == lvl]
    if lvl_locs:
        lo, hi = min(lvl_locs), max(lvl_locs)
        pattern = rf'\*\*{lvl}\*\*.*?(\d+)~(\d+)\s*行'
        m = re.search(pattern, readme_text)
        if m:
            readme_lo, readme_hi = int(m.group(1)), int(m.group(2))
            if readme_lo != lo or readme_hi != hi:
                add_issue("ERROR", "D1", f"README-{lvl}",
                          f"LOC range: README={readme_lo}~{readme_hi} vs actual={lo}~{hi}")
        else:
            add_issue("WARN", "D1", f"README-{lvl}", "LOC range pattern not found in README")

neg_locs = [int(r["loc"]) for r in index_rows if r["level"] == "negative"]
if neg_locs:
    lo, hi = min(neg_locs), max(neg_locs)
    m = re.search(r'\*\*negative\*\*.*?(\d+)~(\d+)\s*行', readme_text)
    if m:
        readme_lo, readme_hi = int(m.group(1)), int(m.group(2))
        if readme_lo != lo or readme_hi != hi:
            add_issue("ERROR", "D1", "README-negative",
                      f"LOC range: README={readme_lo}~{readme_hi} vs actual={lo}~{hi}")

d1_count = sum(1 for s, d, *_ in issues if d == "D1")
print(f"  Found {d1_count} issues\n")

# ============================================================
# D2: Zero-Mark Principle
# ============================================================
print("=" * 70)
print("D2: Zero-Mark Principle (no hint markers in code)")
print("=" * 70)

FORBIDDEN_PATTERNS = [
    (r'\bBUG\b', "BUG"),
    (r'\bFIXME\b', "FIXME"),
    (r'\bHACK\b', "HACK"),
    (r'(?<![/\w])\bTODO\b', "TODO"),
    (r'\b缺陷\b', "缺陷"),
    (r'\b漏洞\b', "漏洞"),
    (r'\b原子性违规\b', "原子性违规"),
    (r'\b原子性违反\b', "原子性违反"),
    (r'\batomicity\s*violation\b', "atomicity violation"),
    (r'\brace\s*condition\b', "race condition"),
    (r'\bdata\s*race\b', "data race"),
    (r'\b竞态\b', "竞态"),
    (r'\b竞争条件\b', "竞争条件"),
    (r'\bINTENTIONAL\b', "INTENTIONAL"),
    (r'\b故意\b', "故意"),
    (r'\bDEFECT\b', "DEFECT"),
    (r'\b共享变量\b', "共享变量"),
    (r'\b中断安全\b', "中断安全"),
    (r'\bnon-?atomic\b', "non-atomic"),
    (r'\btorn\s*read\b', "torn read"),
]

for cid in ALL_IDS:
    idx = index_by_id[cid]
    fpath = get_case_path(cid, idx)
    if not os.path.exists(fpath):
        continue
    lines = read_lines(fpath)
    for i, line in enumerate(lines, 1):
        for pattern, label in FORBIDDEN_PATTERNS:
            if re.search(pattern, line, re.IGNORECASE):
                stripped = line.strip()
                add_issue("ERROR", "D2", cid, f"Line {i}: '{label}' found: {stripped[:100]}")

d2_count = sum(1 for s, d, *_ in issues if d == "D2")
print(f"  Found {d2_count} issues\n")

# ============================================================
# D3: Comment Quality
# ============================================================
print("=" * 70)
print("D3: Comment Quality")
print("=" * 70)

comment_stats = []
for cid in ALL_IDS:
    idx = index_by_id[cid]
    fpath = get_case_path(cid, idx)
    if not os.path.exists(fpath):
        continue
    lines = read_lines(fpath)
    total = len(lines)
    comment_lines = 0
    in_block = False
    for line in lines:
        stripped = line.strip()
        if in_block:
            comment_lines += 1
            if "*/" in stripped:
                in_block = False
            continue
        if stripped.startswith("//"):
            comment_lines += 1
        elif stripped.startswith("/*"):
            comment_lines += 1
            if "*/" not in stripped:
                in_block = True
        elif "/*" in stripped:
            comment_lines += 1
            if "*/" not in stripped:
                in_block = True
    ratio = comment_lines / total * 100 if total > 0 else 0
    comment_stats.append((cid, ratio, comment_lines, total))

    if ratio < 14.0:
        add_issue("WARN", "D3", cid, f"Comment ratio {ratio:.1f}% < 14%")
    elif ratio > 32.0:
        add_issue("WARN", "D3", cid, f"Comment ratio {ratio:.1f}% > 32%")

print(f"  {'Case':<12} {'Ratio':>7} {'Comment':>8} {'Total':>7}")
print(f"  {'-'*12} {'-'*7} {'-'*8} {'-'*7}")
for cid, ratio, cl, tl in sorted(comment_stats, key=lambda x: x[1]):
    flag = " <--" if ratio < 14 or ratio > 32 else ""
    print(f"  {cid:<12} {ratio:>6.1f}% {cl:>8} {tl:>7}{flag}")

ratios = [r for _, r, _, _ in comment_stats]
print(f"\n  Average: {sum(ratios)/len(ratios):.1f}%, "
      f"Min: {min(ratios):.1f}%, Max: {max(ratios):.1f}%")

d3_count = sum(1 for s, d, *_ in issues if d == "D3")
print(f"  Found {d3_count} issues\n")

# ============================================================
# D4: Annotation Accuracy (positive cases only)
# ============================================================
print("=" * 70)
print("D4: Annotation Accuracy (line references & symbol existence)")
print("=" * 70)

VALID_STRATEGIES = [
    "move_to_interrupt_context",
    "disable_global_interrupt",
    "disable_specific_interrupt",
    "restructure_call_chain",
    "introduce_shadow_variable_with_critical_section"
]

for cid in ALL_IDS:
    ann = ann_by_id[cid]
    if ann["has_defect"] is False:
        continue

    idx = index_by_id[cid]
    fpath = get_case_path(cid, idx)
    if not os.path.exists(fpath):
        continue
    lines = read_lines(fpath)
    total_lines = len(lines)
    code_text = "".join(lines)

    defects = ann.get("defects", []) or []
    for defect in defects:
        shared_vars = defect.get("shared_variables", [])
        conflict = defect.get("conflict", {}) or {}
        ctx_a = conflict.get("context_A", {}) or {}
        ctx_b = conflict.get("context_B", {}) or {}

        # D4.1: Check shared variables exist (strip array index for matching)
        for sv in shared_vars:
            var_name = sv if isinstance(sv, str) else sv.get("name", "")
            if not var_name:
                continue
            # Extract base name: "BUF[0]" -> "BUF", "struct.field" -> "struct" and "field"
            base = re.split(r'[\[\].]', var_name)[0]
            if base and base not in code_text:
                add_issue("ERROR", "D4", cid, f"Shared var base '{base}' (from '{var_name}') not found")

        # D4.2: Check context_A/B function and line references
        for ctx_name, ctx in [("context_A", ctx_a), ("context_B", ctx_b)]:
            func = ctx.get("function", "")
            ctx_lines = ctx.get("lines", [])

            if func and func not in code_text:
                add_issue("ERROR", "D4", cid, f"{ctx_name}.function '{func}' not found in code")

            if ctx_lines:
                for ln in ctx_lines:
                    if ln < 1 or ln > total_lines:
                        add_issue("ERROR", "D4", cid,
                                  f"{ctx_name}.lines: line {ln} out of range (file has {total_lines})")

                # Check referenced lines contain actual code
                code_line_count = 0
                for ln in ctx_lines:
                    if 1 <= ln <= total_lines:
                        content = lines[ln - 1].strip()
                        if content and not content.startswith("//") and not content.startswith("/*") and content != "*/":
                            code_line_count += 1
                if code_line_count == 0:
                    add_issue("WARN", "D4", cid,
                              f"{ctx_name}.lines [{min(ctx_lines)}..{max(ctx_lines)}] "
                              f"contain no code lines")

        # D4.3: fix_strategy validity
        fs = defect.get("fix_strategy", "")
        if fs not in VALID_STRATEGIES:
            add_issue("ERROR", "D4", cid, f"Invalid fix_strategy: '{fs}'")

        # D4.4: Check shared variables are actually accessed in the referenced lines
        for ctx_name, ctx in [("context_A", ctx_a), ("context_B", ctx_b)]:
            ctx_lines = ctx.get("lines", [])
            if not ctx_lines or not shared_vars:
                continue
            # Get the code in the referenced lines
            ref_code = ""
            for ln in ctx_lines:
                if 1 <= ln <= total_lines:
                    ref_code += lines[ln - 1]
            # Check at least one shared variable base name appears
            found_any = False
            for sv in shared_vars:
                var_name = sv if isinstance(sv, str) else sv.get("name", "")
                base = re.split(r'[\[\].]', var_name)[0]
                if base and base in ref_code:
                    found_any = True
                    break
            if not found_any:
                add_issue("WARN", "D4", cid,
                          f"{ctx_name} lines {ctx_lines} don't reference any shared variable")

d4_count = sum(1 for s, d, *_ in issues if d == "D4")
print(f"  Found {d4_count} issues\n")

# ============================================================
# D5: Positive case structure verification
# ============================================================
print("=" * 70)
print("D5: Positive case structure (ISR/main function existence)")
print("=" * 70)

for cid in ALL_IDS:
    ann = ann_by_id[cid]
    if ann["has_defect"] is False:
        continue
    idx = index_by_id[cid]
    fpath = get_case_path(cid, idx)
    if not os.path.exists(fpath):
        continue
    code_text = open(fpath, "r", encoding="utf-8").read()

    # Should have at least one interrupt-like function
    has_isr = bool(re.search(
        r'(interrupt|__interrupt|ISR|_isr|void\s+\w+_int|#pragma\s+vector|TRAP_HANDLER)',
        code_text, re.IGNORECASE))
    if not has_isr:
        add_issue("WARN", "D5", cid, "No ISR/interrupt pattern detected")

    # Should have a main() function
    has_main = bool(re.search(r'\bmain\s*\(', code_text))
    if not has_main:
        add_issue("WARN", "D5", cid, "No main() function found")

d5_count = sum(1 for s, d, *_ in issues if d == "D5")
print(f"  Found {d5_count} issues\n")

# ============================================================
# D6: Negative case consistency with origin
# ============================================================
print("=" * 70)
print("D6: Negative case file size consistency with origin")
print("=" * 70)

for cid in ALL_IDS:
    if not cid.startswith("NEG_"):
        continue
    idx = index_by_id[cid]
    ann = ann_by_id[cid]
    origin_id = ann.get("origin_case", "")
    if origin_id not in index_by_id:
        add_issue("ERROR", "D6", cid, f"Origin '{origin_id}' not found in index")
        continue

    origin_idx = index_by_id[origin_id]
    neg_loc = int(idx["loc"])
    origin_loc = int(origin_idx["loc"])

    # NEG should be within reasonable range of origin (±30%)
    ratio = neg_loc / origin_loc if origin_loc > 0 else 0
    if ratio < 0.7 or ratio > 1.5:
        add_issue("WARN", "D6", cid,
                  f"LOC ratio vs origin {origin_id}: {ratio:.2f} "
                  f"({neg_loc} vs {origin_loc})")

    # fix_note should exist for negative cases
    fix_note = ann.get("fix_note", "")
    if not fix_note:
        add_issue("WARN", "D6", cid, "No fix_note in annotations.json")

d6_count = sum(1 for s, d, *_ in issues if d == "D6")
print(f"  Found {d6_count} issues\n")

# ============================================================
# Summary
# ============================================================
print("=" * 70)
print("AUDIT SUMMARY")
print("=" * 70)

errors = [(s, d, c, m) for s, d, c, m in issues if s == "ERROR"]
warns = [(s, d, c, m) for s, d, c, m in issues if s == "WARN"]

by_dim = Counter(d for _, d, _, _ in issues)
for dim in sorted(by_dim.keys()):
    e = sum(1 for s, d, _, _ in issues if d == dim and s == "ERROR")
    w = sum(1 for s, d, _, _ in issues if d == dim and s == "WARN")
    print(f"  {dim}: {e} errors, {w} warnings")

print(f"\n  Total: {len(errors)} ERRORS, {len(warns)} WARNINGS")
print()

if errors:
    print("ERRORS:")
    for s, d, c, m in errors:
        print(f"  [{d}] {c}: {m}")
    print()

if warns:
    print("WARNINGS:")
    for s, d, c, m in warns:
        print(f"  [{d}] {c}: {m}")
    print()

if not errors and not warns:
    print("  All automated checks passed!")

sys.exit(1 if errors else 0)
