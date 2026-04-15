# Artifact for ASE 2026 Submission

This repository contains the benchmark dataset and baseline implementation
used in the evaluation of our submission.

## Contents

```
.
├── benchmark/
│   └── EmbedMVABench/      Benchmark dataset (36 embedded C programs
│                           with multi-variable atomicity violations)
└── baselines/
    └── dg-master/          Third-party comparison tool (intCV)
```

## benchmark/EmbedMVABench

A benchmark suite of 36 embedded C programs covering multi-variable
atomicity violations at three difficulty levels (L1/L2/L3) plus
negative (fixed) cases. See `benchmark/EmbedMVABench/README.md` for
the full data-sheet, case index, and annotation schema.

Each case contains a single `*.c` file; ground-truth defect locations
and fix strategies are in `benchmark/EmbedMVABench/ground_truth/annotations.json`.

## baselines/dg-master

`baselines/dg-master/` is a redistributed copy of **intCV**, a third-party
static variable-correlation detector built on top of the DG program-analysis
library. It is included here to make our comparison experiments reproducible.

- Original license: MIT (see `baselines/dg-master/LICENSE`)
- Build instructions: `baselines/dg-master/README.md` and
  `baselines/dg-master/setup_and_run.sh` / `Dockerfile`
- We did not modify intCV's source; only pre-built binaries and
  machine-specific CMake caches were stripped from this snapshot.

## Industrial source code

The industrial project source code used in parts of the evaluation
cannot be made publicly available due to confidentiality agreements
with industry partners.
