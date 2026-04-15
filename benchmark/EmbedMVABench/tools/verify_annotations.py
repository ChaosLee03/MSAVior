#!/usr/bin/env python3
"""Verify annotations.json line references match actual code files."""
import json
import os
import sys

def main():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ann_path = os.path.join(base, 'ground_truth', 'annotations.json')

    with open(ann_path, encoding='utf-8') as f:
        data = json.load(f)

    errors = []
    for case in data['cases']:
        if not case['has_defect']:
            continue

        cid = case['id']
        level = case['level']
        fpath = os.path.join(base, 'cases', level, cid, f'{cid}.c')
        if not os.path.exists(fpath):
            errors.append(f'{cid}: file not found at {fpath}')
            continue

        with open(fpath, encoding='utf-8') as f:
            lines = f.readlines()

        for defect in case['defects']:
            did = defect.get('defect_id', '?')
            for sv in defect.get('shared_variables', []):
                base_name = sv.split('[')[0].split('.')[-1]
                found = any(base_name in line for line in lines)
                if not found:
                    errors.append(f'{cid}/d{did}: shared variable base "{base_name}" not found in file')

            for ctx_key in ['context_A', 'context_B']:
                ctx = defect['conflict'][ctx_key]
                func = ctx['function']
                ref_lines = ctx['lines']

                func_found = any(func in line for line in lines)
                if not func_found:
                    errors.append(f'{cid}/d{did}: function "{func}" not found in file')

                for ln in ref_lines:
                    if ln > len(lines):
                        errors.append(f'{cid}/d{did}: line {ln} exceeds file length {len(lines)}')

                # verify additional_sites if present
                for site in ctx.get('additional_sites', []):
                    site_func = site['function']
                    site_lines = site['lines']
                    site_found = any(site_func in line for line in lines)
                    if not site_found:
                        errors.append(f'{cid}/d{did}: additional site function "{site_func}" not found in file')
                    for ln in site_lines:
                        if ln > len(lines):
                            errors.append(f'{cid}/d{did}: additional site line {ln} exceeds file length {len(lines)}')

    if errors:
        print(f'FAILED: {len(errors)} errors found:')
        for e in errors:
            print(f'  - {e}')
        sys.exit(1)
    else:
        total_defects = sum(len(c['defects']) for c in data['cases'] if c['has_defect'])
        print(f'PASSED: All annotation references verified for {total_defects} defects across {sum(1 for c in data["cases"] if c["has_defect"])} positive cases')
        sys.exit(0)

if __name__ == '__main__':
    main()
