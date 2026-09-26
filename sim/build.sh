#!/usr/bin/env bash
# 編譯電梯 ST 程式並建立模擬用共享函式庫。
#   1. 取得 matiec（IEC 61131-3 → C 編譯器，OpenPLC / Beremiz 使用的同一套）
#   2. src/*.st 合併成 build/elevator.st 並編譯（= 可直接上傳 OpenPLC 的單一檔案）
#   3. 加上測試用程式編譯成 build/libelevator_sim.so 給 Python 模擬器使用
# 需要：git gcc g++ make autoconf automake flex bison
# 可用環境變數 MATIEC_DIR 指定已編譯好的 matiec 目錄。
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SIM_DIR")"
BUILD_DIR="$SIM_DIR/build"
MATIEC_REPO="https://github.com/beremiz/matiec.git"
MATIEC_REV="3a41303bcb4e50403417bc39c63f88981d7d961e"
MATIEC_DIR="${MATIEC_DIR:-$SIM_DIR/.cache/matiec}"

if [ ! -x "$MATIEC_DIR/iec2c" ]; then
    echo "==> 下載並編譯 matiec 到 $MATIEC_DIR"
    mkdir -p "$(dirname "$MATIEC_DIR")"
    if [ ! -d "$MATIEC_DIR/.git" ]; then
        git clone --quiet "$MATIEC_REPO" "$MATIEC_DIR"
    fi
    (
        cd "$MATIEC_DIR"
        git checkout --quiet "$MATIEC_REV"
        autoreconf -i >/dev/null 2>&1
        ./configure >/dev/null
        make -j"$(nproc 2>/dev/null || echo 2)" >/dev/null 2>&1
    )
fi
IEC2C="$MATIEC_DIR/iec2c"
MATIEC_LIB="$MATIEC_DIR/lib"

mkdir -p "$BUILD_DIR/deploy" "$BUILD_DIR/test"

# ---- 實機程式：合併成單一檔案並檢查語法 ----
cat "$ROOT_DIR"/src/[0-9]*.st > "$BUILD_DIR/elevator.st"
(cd "$BUILD_DIR/deploy" && "$IEC2C" -I "$MATIEC_LIB" "$BUILD_DIR/elevator.st" >/dev/null)
echo "==> build/elevator.st 編譯成功"

# ---- 模擬用：實機程式（不含組態）+ 測試程式 + 測試組態 ----
cat "$ROOT_DIR"/src/0[1-3]_*.st "$SIM_DIR"/st/PRG_TestN.st "$SIM_DIR"/st/test_config.st \
    > "$BUILD_DIR/test/all.st"
(cd "$BUILD_DIR/test" && "$IEC2C" -I "$MATIEC_LIB" all.st >/dev/null)
gcc -shared -fPIC -O1 -w \
    -I "$MATIEC_LIB/C" -I "$BUILD_DIR/test" \
    -o "$BUILD_DIR/libelevator_sim.so" \
    "$SIM_DIR/harness.c" "$BUILD_DIR/test/Config0.c" "$BUILD_DIR/test/Res0.c"
echo "==> build/libelevator_sim.so 建立完成"
