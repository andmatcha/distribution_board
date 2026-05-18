#!/usr/bin/env zsh
set -euo pipefail
unsetopt bg_nice

# ソースコードに記述したprintfの出力を表示するためのスクリプト

BOARD="${1:-${BOARD:-board4}}"
PORT="${PORT:-3344}"
PIN_FREQ="${PIN_FREQ:-1000000}"

case "$BOARD" in
  board1|board2|board3|board4)
    TARGET_CFG="target/stm32f1x.cfg"
    TPIU="stm32f1x.tpiu"
    TRACECLK="${TRACECLK:-64000000}"
    ;;
  board4u)
    TARGET_CFG="target/stm32f4x.cfg"
    TPIU="stm32f4x.tpiu"
    TRACECLK="${TRACECLK:-144000000}"
    ;;
  *)
    echo "Usage: $0 [board1|board2|board3|board4|board4u]" >&2
    exit 2
    ;;
esac

# OpenOCD をバックグラウンドで起動し、nc localhost 3344 を実行する
# 終了時に OpenOCD を確実に停止します

OPENOCD_CMD=(
  openocd -f interface/stlink.cfg -f "$TARGET_CFG" \
    -c "init" \
    -c "$TPIU configure -protocol uart -traceclk $TRACECLK -pin-freq $PIN_FREQ -output :$PORT -formatter off" \
    -c "itm ports on" \
    -c "$TPIU enable" \
    -c "reset run"
)

LOG_FILE="/tmp/openocd_itm_$(date +%Y%m%d_%H%M%S).log"

echo "[run_itm_nc] $BOARD の OpenOCD をバックグラウンドで起動します... (log: $LOG_FILE)" >&2
"${OPENOCD_CMD[@]}" >"$LOG_FILE" 2>&1 &
OPENOCD_PID=$!

cleanup() {
  if kill -0 "$OPENOCD_PID" 2>/dev/null; then
    echo "[run_itm_nc] OpenOCD (PID: $OPENOCD_PID) を停止します" >&2
    kill "$OPENOCD_PID" 2>/dev/null || true
    wait "$OPENOCD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ポート 3344 が開くまで短時間待機
echo "[run_itm_nc] ポート $PORT を待機中..." >&2
for i in {1..100}; do
  if nc -z localhost "$PORT" 2>/dev/null; then
    break
  fi
  sleep 0.1
done

echo "[run_itm_nc] 接続開始: nc localhost $PORT" >&2
nc localhost "$PORT"

echo "[run_itm_nc] nc 終了。クリーンアップします。" >&2
