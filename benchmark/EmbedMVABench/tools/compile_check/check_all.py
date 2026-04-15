#!/usr/bin/env python3
"""Compile-check all 36 benchmark C files using GCC with embedded stubs."""
import os
import re
import shutil
import subprocess
import sys
import tempfile

GCC = os.environ.get("GCC") or shutil.which("gcc") or "gcc"
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # -> EmbedMVABench/tools -> EmbedMVABench
BASE = os.path.dirname(BASE)  # -> EmbedMVABench
STUB_DIR = os.path.dirname(os.path.abspath(__file__))

# 8051-specific patterns that need preprocessing
PATTERNS_8051 = {
    # interrupt keyword: void func(void) interrupt N using M
    r'\)\s*interrupt\s+\d+(\s+using\s+\d+)?\s*\{': ') {',
    r'\)\s*interrupt\s+\d+(\s+using\s+\d+)?\s*$': ')',
    # sbit declarations: sbit name = P1^5;
    r'^(\s*)sbit\s+(\w+)\s*=\s*[^;]+;': r'\1static volatile unsigned char \2;',
    # sfr declarations: sfr name = 0x80;
    r'^(\s*)sfr\s+(\w+)\s*=\s*[^;]+;': r'\1static volatile unsigned char \2;',
    # xdata/idata/data/code storage class
    r'\bxdata\b': '/* xdata */',
    r'\bidata\b': '/* idata */',
    r'\bpdata\b': '/* pdata */',
    r'(\w)\s+data\s+': r'\1 /* data */ ',
    # bit type
    r'\bbit\s+(\w)': r'unsigned char \1',
    # reentrant keyword
    r'\)\s*reentrant\s*': ') ',
}

def is_8051_file(filepath):
    """Check if file uses 8051-specific syntax."""
    with open(filepath, encoding='utf-8', errors='replace') as f:
        content = f.read(3000)  # Check first 3000 chars
    indicators = ['interrupt', 'sbit ', 'sfr ', 'XBYTE', 'xdata', '_nop_', 'intrins.h',
                  '#define uchar', '#define uint ', 'P0 ', 'P1 ', 'P2 ', 'P3 ']
    count = sum(1 for ind in indicators if ind in content)
    return count >= 2

def preprocess_8051(filepath):
    """Preprocess 8051 file to make it GCC-compatible."""
    with open(filepath, encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    processed = []
    for line in lines:
        # Handle 'interrupt N using M' on function definitions
        line = re.sub(r'\)\s*interrupt\s+\d+(\s+using\s+\d+)?\s*\{', ') {', line)
        line = re.sub(r'\)\s*interrupt\s+\d+(\s+using\s+\d+)?\s*$', ')', line)
        # Handle standalone 'using N' (register bank selection without interrupt)
        line = re.sub(r'\)\s+using\s+\d+\s*\{', ') {', line)
        line = re.sub(r'\)\s+using\s+\d+\s*$', ')', line)
        # sbit
        line = re.sub(r'^(\s*)sbit\s+(\w+)\s*=\s*[^;]+;', r'\1static volatile unsigned char \2;', line)
        # sfr
        line = re.sub(r'^(\s*)sfr\s+(\w+)\s*=\s*[^;]+;', r'\1static volatile unsigned char \2;', line)
        # storage classes - simply remove them
        line = re.sub(r'\bxdata\s+', '', line)
        line = re.sub(r'\bidata\s+', '', line)
        line = re.sub(r'\bpdata\s+', '', line)
        # 'data' as storage class (tricky - only when after a type and before varname)
        line = re.sub(r'(\buchar|\buint8|\buint16|\buint32|\bUCHAR|\bunsigned\s+char|\bunsigned\s+int)\s+data\s+', r'\1 ', line)
        line = re.sub(r'\bcode\s+(const\b|static\b)', r'\1', line)
        # 'code' qualifier before array declarations (code const -> const)
        line = re.sub(r'\bcode\s+', '', line)
        # bit type
        line = re.sub(r'\bbit\s+(\w)', r'unsigned char \1', line)
        # reentrant
        line = re.sub(r'\)\s*reentrant', ')', line)
        processed.append(line)

    return ''.join(processed)

def check_file(filepath, file_id):
    """Compile-check a single file. Returns (file_id, errors_list)."""
    is_8051 = is_8051_file(filepath)

    # Create temp file with preprocessing if needed
    with tempfile.NamedTemporaryFile(mode='w', suffix='.c', delete=False,
                                      encoding='utf-8') as tmp:
        if is_8051:
            content = preprocess_8051(filepath)
        else:
            with open(filepath, encoding='utf-8', errors='replace') as f:
                content = f.read()

        tmp.write(content)
        tmp_path = tmp.name

    try:
        # GCC flags:
        # -fsyntax-only: only check syntax, don't compile
        # -std=c99: use C99 standard
        # -w: suppress warnings (we only care about errors for now)
        # -I: include stub directory for intrins.h etc.
        cmd = [
            GCC,
            '-fsyntax-only',
            '-std=c99',
            '-I', STUB_DIR,
            '-Wno-all',
            '-Wno-implicit-function-declaration',  # many functions have no body
            tmp_path
        ]

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30,
                                encoding='utf-8', errors='replace')

        errors = []
        if result.returncode != 0:
            # Parse error output, map temp file lines back
            for line in (result.stderr + result.stdout).split('\n'):
                line = line.strip()
                if not line:
                    continue
                # Replace temp filename with original
                line = line.replace(tmp_path.replace('\\', '/'), file_id)
                line = line.replace(tmp_path, file_id)
                # Skip notes, only keep errors and important warnings
                if ': error:' in line or ': fatal error:' in line:
                    errors.append(line)
        return (file_id, is_8051, errors)
    except Exception as e:
        return (file_id, is_8051, [f"EXCEPTION: {e}"])
    finally:
        try:
            os.unlink(tmp_path)
        except:
            pass

def main():
    cases_dir = os.path.join(BASE, 'cases')
    dirs = [
        ('L1', os.path.join(cases_dir, 'L1')),
        ('L2', os.path.join(cases_dir, 'L2')),
        ('L3', os.path.join(cases_dir, 'L3')),
        ('negative', os.path.join(cases_dir, 'negative')),
    ]

    total_errors = 0
    results = []

    for level, dirpath in dirs:
        subdirs = sorted([d for d in os.listdir(dirpath)
                          if os.path.isdir(os.path.join(dirpath, d))])
        for file_id in subdirs:
            filepath = os.path.join(dirpath, file_id, file_id + '.c')
            fid, is_8051, errors = check_file(filepath, file_id)
            tag = '8051' if is_8051 else 'generic'
            results.append((fid, tag, errors))
            total_errors += len(errors)

    # Print results
    print("=" * 70)
    print("EmbedMVABench Compilation Check (GCC -fsyntax-only)")
    print("=" * 70)

    for fid, tag, errors in results:
        if errors:
            print(f"  FAIL {fid:12s} ({tag:7s}) - {len(errors)} error(s)")
            for e in errors:
                print(f"       {e}")
        else:
            print(f"  OK   {fid:12s} ({tag:7s})")

    print()
    print(f"Total: {len(results)} files, {total_errors} errors")
    if total_errors == 0:
        print("All files pass syntax check!")
    else:
        print(f"*** {total_errors} errors need fixing ***")

    return 0 if total_errors == 0 else 1

if __name__ == '__main__':
    sys.exit(main())
