#!/usr/bin/env python3
"""Check for important warnings (not just errors) in all 36 files."""
import os
import re
import shutil
import subprocess
import sys
import tempfile

GCC = os.environ.get("GCC") or shutil.which("gcc") or "gcc"
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.dirname(BASE)
STUB_DIR = os.path.dirname(os.path.abspath(__file__))

def is_8051_file(filepath):
    with open(filepath, encoding='utf-8', errors='replace') as f:
        content = f.read(3000)
    indicators = ['interrupt', 'sbit ', 'sfr ', 'XBYTE', 'xdata', '_nop_', 'intrins.h',
                  '#define uchar', '#define uint ', 'P0 ', 'P1 ', 'P2 ', 'P3 ']
    return sum(1 for ind in indicators if ind in content) >= 2

def preprocess_8051(filepath):
    with open(filepath, encoding='utf-8', errors='replace') as f:
        lines = f.readlines()
    processed = []
    for line in lines:
        line = re.sub(r'\)\s*interrupt\s+\d+(\s+using\s+\d+)?\s*\{', ') {', line)
        line = re.sub(r'\)\s*interrupt\s+\d+(\s+using\s+\d+)?\s*$', ')', line)
        line = re.sub(r'\)\s+using\s+\d+\s*\{', ') {', line)
        line = re.sub(r'\)\s+using\s+\d+\s*$', ')', line)
        line = re.sub(r'^(\s*)sbit\s+(\w+)\s*=\s*[^;]+;', r'\1static volatile unsigned char \2;', line)
        line = re.sub(r'^(\s*)sfr\s+(\w+)\s*=\s*[^;]+;', r'\1static volatile unsigned char \2;', line)
        line = re.sub(r'\bxdata\s+', '', line)
        line = re.sub(r'\bidata\s+', '', line)
        line = re.sub(r'\bpdata\s+', '', line)
        line = re.sub(r'(\buchar|\buint8|\buint16|\buint32|\bUCHAR|\bunsigned\s+char|\bunsigned\s+int)\s+data\s+', r'\1 ', line)
        line = re.sub(r'\bcode\s+(const\b|static\b)', r'\1', line)
        line = re.sub(r'\bcode\s+', '', line)
        line = re.sub(r'\bbit\s+(\w)', r'unsigned char \1', line)
        line = re.sub(r'\)\s*reentrant', ')', line)
        processed.append(line)
    return ''.join(processed)

def check_file(filepath, file_id):
    is_8051 = is_8051_file(filepath)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.c', delete=False, encoding='utf-8') as tmp:
        if is_8051:
            content = preprocess_8051(filepath)
        else:
            with open(filepath, encoding='utf-8', errors='replace') as f:
                content = f.read()
        tmp.write(content)
        tmp_path = tmp.name
    try:
        cmd = [GCC, '-fsyntax-only', '-std=c99', '-I', STUB_DIR,
               '-Wall',
               '-Wno-unused-variable', '-Wno-unused-function',
               '-Wno-unused-but-set-variable', '-Wno-unused-label',
               '-Wno-implicit-function-declaration',
               '-Wno-sign-compare', '-Wno-parentheses',
               '-Wno-missing-braces',
               tmp_path]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30,
                                encoding='utf-8', errors='replace')
        warnings = []
        if result.returncode != 0:
            for line in result.stderr.split('\n'):
                line = line.strip()
                if not line:
                    continue
                line = line.replace(tmp_path.replace('\\', '/'), file_id)
                line = line.replace(tmp_path, file_id)
                if ': error:' in line or ': fatal error:' in line:
                    warnings.append(('ERROR', line))
                elif ': warning:' in line:
                    # Filter out uninteresting warnings
                    if 'implicit declaration' in line:
                        continue
                    if 'unused' in line:
                        continue
                    warnings.append(('WARN', line))
        return (file_id, warnings)
    except Exception as e:
        return (file_id, [('ERROR', f"EXCEPTION: {e}")])
    finally:
        try:
            os.unlink(tmp_path)
        except:
            pass

def main():
    cases_dir = os.path.join(BASE, 'cases')
    dirs = [('L1', os.path.join(cases_dir, 'L1')),
            ('L2', os.path.join(cases_dir, 'L2')),
            ('L3', os.path.join(cases_dir, 'L3')),
            ('negative', os.path.join(cases_dir, 'negative'))]
    total_issues = 0
    for level, dirpath in dirs:
        subdirs = sorted([d for d in os.listdir(dirpath)
                          if os.path.isdir(os.path.join(dirpath, d))])
        for file_id in subdirs:
            filepath = os.path.join(dirpath, file_id, file_id + '.c')
            fid, issues = check_file(filepath, file_id)
            if issues:
                total_issues += len(issues)
                for severity, msg in issues:
                    print(f"  [{severity}] {msg}")
    if total_issues == 0:
        print("No significant warnings found.")
    else:
        print(f"\n{total_issues} warnings/errors found.")

if __name__ == '__main__':
    main()
