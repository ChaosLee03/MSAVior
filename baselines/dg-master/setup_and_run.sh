#!/bin/bash
# ============================================================
# intCV / MUVI Detector - WSL2 一键安装、编译、运行脚本
# 用法: 在 WSL 中运行
#   cd /mnt/d/dg-master
#   bash setup_and_run.sh              # 安装依赖 + 编译
#   bash setup_and_run.sh test.c       # 编译 + 对 test.c 运行 MUVI 检测
# ============================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build-wsl"

# ---- 颜色输出 ----
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ============================================================
# Step 1: 安装依赖
# ============================================================
install_deps() {
    info "检查并安装依赖..."
    local NEEDED=()

    command -v clang   >/dev/null 2>&1 || NEEDED+=(clang)
    command -v cmake   >/dev/null 2>&1 || NEEDED+=(cmake)
    command -v ninja   >/dev/null 2>&1 || NEEDED+=(ninja-build)
    command -v java    >/dev/null 2>&1 || NEEDED+=(default-jre)
    dpkg -s llvm-dev   >/dev/null 2>&1 || NEEDED+=(llvm-dev)

    if [ ${#NEEDED[@]} -eq 0 ]; then
        info "所有依赖已安装"
    else
        info "需要安装: ${NEEDED[*]}"
        sudo apt-get update -qq
        sudo apt-get install -yq --no-install-recommends "${NEEDED[@]}"
        info "依赖安装完成"
    fi
}

# ============================================================
# Step 2: 编译项目
# ============================================================
build_project() {
    info "开始编译项目 (构建目录: $BUILD_DIR)..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    cmake "$PROJECT_DIR" \
        -GNinja \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_C_COMPILER=clang \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo

    cmake --build . -- -j"$(nproc)"

    info "编译完成！"
    info "MUVI 检测器位于: $BUILD_DIR/tools/llvm-my-muvi"
    cd "$PROJECT_DIR"
}

# ============================================================
# Step 3: 对 .c 文件运行 MUVI 检测
# ============================================================
run_muvi() {
    local CFILE="$1"

    if [ ! -f "$CFILE" ]; then
        error "文件不存在: $CFILE"
    fi

    local MUVI_BIN="$BUILD_DIR/tools/llvm-my-muvi"
    if [ ! -f "$MUVI_BIN" ]; then
        error "MUVI 检测器未编译，请先运行: bash setup_and_run.sh"
    fi

    local WORKDIR
    WORKDIR="$(dirname "$(realpath "$CFILE")")"
    local BASENAME
    BASENAME="$(basename "$CFILE" .c)"
    local BCFILE="$WORKDIR/${BASENAME}.bc"

    info "Step 3a: 将 $CFILE 编译为 LLVM bitcode..."
    clang -emit-llvm -c -fno-discard-value-names -g "$CFILE" -o "$BCFILE"
    info "生成 bitcode: $BCFILE"

    # 将 spmf jar 复制到工作目录，方便检测器找到
    cp -f "$PROJECT_DIR/tools/spmf-1.7.jar" "$WORKDIR/spmf-1.7.jar" 2>/dev/null || true

    info "Step 3b: 运行 MUVI 检测器..."
    "$MUVI_BIN" "$BCFILE" -entry=main || true

    info "===== 检测完成 ====="
    info "输出文件 (在 $WORKDIR 目录下):"
    ls -la "$WORKDIR"/muvi_*.txt 2>/dev/null || warn "未生成输出文件 (可能该程序中没有全局变量访问)"
}

# ============================================================
# 主流程
# ============================================================
info "intCV / MUVI Detector 一键脚本"
info "项目目录: $PROJECT_DIR"
echo ""

install_deps
build_project

if [ -n "$1" ]; then
    run_muvi "$1"
else
    echo ""
    info "编译完成。要分析 .c 文件，运行:"
    info "  bash setup_and_run.sh <your_file.c>"
    echo ""
    info "或者手动执行:"
    info "  clang -emit-llvm -c -fno-discard-value-names -g test.c -o test.bc"
    info "  $BUILD_DIR/tools/llvm-my-muvi test.bc -entry=main"
fi
