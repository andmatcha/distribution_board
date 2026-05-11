# distribution_board

STM32F103TBU6 を使った distribution board 向けのファームウェア集です。  
`board1` から `board4` までの各基板ごとに PlatformIO プロジェクトが分かれており、ビルドや書き込みはリポジトリ直下の `Makefile` からまとめて実行できます。

## 構成概要

- `board1`: Base Horizon と Base Roll の 2 系統を扱います。2 個の RS485 エンコーダを読み、Horizon は距離、Roll は位置を CAN 送信します。
- `board2`: 1 個の RS485 エンコーダを連続読み取りし、位置を CAN ID `0x302` で送信します。
- `board3`: `board2` と同じ構成で、位置を CAN ID `0x303` で送信します。
- `board4`: エンコーダ位置を CAN ID `0x304` で送信しつつ、CAN 受信で DC モータ 2 系統とサーボ 1 系統を制御します。
- 各 `boardX` 配下には `platformio.ini`、`src/`、`include/`、STM32CubeMX の `.ioc` ファイル、HAL ドライバ一式が入っています。
- `board4u` は STM32F4 系の別プロジェクトで、この README の仕様整理対象は `board1` から `board4` です。

## 共通設定

- MCU: `STM32F103TBU6`
- System clock: HSE bypass 8 MHz を PLL x8 して 64 MHz。APB1 は 32 MHz、APB2 は 64 MHz。
- CAN: `CAN1`、標準 ID、データフレーム、Normal mode。`PA11` が `CAN_RX`、`PA12` が `CAN_TX`。
- CAN bit timing: Prescaler `2`、BS1 `12TQ`、BS2 `3TQ`、SJW `1TQ`。APB1 32 MHz 前提で 1 Mbps。
- CAN 自動再送: `AutoRetransmission = DISABLE`。送信失敗時の扱いは board ごとの実装に依存します。
- エンコーダ UART: `2000000 bps`、8 bit、1 stop bit、parity none。
- LED: `PA5=Green`、`PA6=Yellow`、`PA7=Red`。

## エンコーダ仕様

board1 から board4 まで、エンコーダ応答は 2 byte の little-endian word として扱います。

```text
受信 word = rx[0] | (rx[1] << 8)
bit  0..13: 14 bit データ
bit 14..15: checksum
checksum = 0x3 ^ ((word >> 0) & 0x3) ^ ... ^ ((word >> 12) & 0x3)
```

- 位置取得コマンドは `0x54` です。復号後の位置は `0..16383` の `uint16_t` として扱います。
- board1 のみ turns 取得コマンド `0x55` も使います。turns は 14 bit signed として扱い、`0x2000` が立っている場合は `0xC000` で符号拡張します。
- board1 のみ Horizon 初期化時に `0x56, 0x75` を送ってエンコーダをリセットします。
- RS485 方向制御は `DE=High` で TX、`DE=Low` で RX です。board2/3/4 と board1 Horizon は `PA8`、board1 Roll は `PA4` を使います。
- board2/3/4 の実装では、チェックサム不一致時にエラーカウンタを増やしますが `data_ready` は立つため、直前に保持していた位置が CAN キューへ入る可能性があります。board1 はチェックサム不一致時に `data_ready` を立てません。

## CAN ID とデータ形式

### 送信

すべて標準 ID、DLC `2`、big-endian です。

| 送信元 | CAN ID | payload | 型/単位 | 内容 |
| --- | --- | --- | --- | --- |
| board1 Base Horizon | `0x300` | `[0]=MSB, [1]=LSB` | `int16_t`, 0.1 mm | Horizon 距離。実装上は `round(total_counts * 50 / 16384)` で計算します。 |
| board1 Base Roll | `0x301` | `[0]=MSB, [1]=LSB` | `uint16_t` | Roll エンコーダ位置。下位 14 bit が有効です。 |
| board2 | `0x302` | `[0]=MSB, [1]=LSB` | `uint16_t` | エンコーダ位置。下位 14 bit が有効です。 |
| board3 | `0x303` | `[0]=MSB, [1]=LSB` | `uint16_t` | エンコーダ位置。下位 14 bit が有効です。 |
| board4 | `0x304` | `[0]=MSB, [1]=LSB` | `uint16_t` | エンコーダ位置。下位 14 bit が有効です。 |

board1 は Horizon/Roll の最新フレームを 1 slot ずつ保持するスケジューラで交互送信します。値が変わったとき、または同値でも 200 ms 経過したときに送信対象になります。CAN bus-off を検出した場合は 1000 ms のクールダウンを入れて復帰を試みます。

board2/3/4 はエンコーダ位置を 8 要素リングバッファへ積み、メインループの `can_control_process_tx()` で空きメールボックス分だけ送信します。リングバッファは head/tail が一致しないようにする実装なので、実効容量は 7 件です。

### board4 の受信

board4 は CAN filter を ID list mode にして `0x208` と `0x1FF` のみ FIFO0 で受信し、`CAN_IT_RX_FIFO0_MSG_PENDING` 割り込みで処理します。

| CAN ID | DLC | payload | 処理 |
| --- | --- | --- | --- |
| `0x208` | `>=5` | `data[0]` | `1` なら DC motor 1 の `dc_motor_push()` を実行します。それ以外は DC motor 1 を停止します。 |
| `0x208` | `>=5` | `data[3]`, `data[4]` | DC motor 2 制御。`data[3] == 1` なら正転 100%、そうでなく `data[4] == 1` なら逆転 100%、どちらでもなければ停止します。両方 `1` の場合は正転が優先です。 |
| `0x1FF` | `>=6` | `int16_t(data[4] << 8 | data[5])` | 負なら `SERVO_DIR_OPEN`、0 なら停止、正なら `SERVO_DIR_CLOSE` を通常モードで実行します。 |

## board1 実装

`init()` で CAN scheduler、Base Horizon、Base Roll を初期化します。`poll()` は Horizon 処理、Roll 処理、CAN scheduler 処理を順に実行し、最後に `HAL_Delay(1)` を入れます。

### Base Horizon

- UART: `USART1` (`PA9=TX`, `PA10=RX`)、DMA RX、RS485 DE は `PA8`。
- 初期化時にエンコーダをリセットし、10 ms 待機した後、現在位置を原点として取り込みます。
- メインループでは position (`0x54`) と turns (`0x55`) を状態機械で順に要求します。各応答待ちは 10 ms timeout です。
- `shifted_pos = (raw_pos - origin_position) & 0x3FFF` とし、`total_counts = turns * 16384 + shifted_pos` を作ります。turns と前回値の食い違いが大きい場合は、前回 `total_counts` から unwrap した予測値を優先します。
- 距離は `distance_tenths_mm = round(total_counts * 50 / 16384)` です。CAN payload では signed 16 bit big-endian で送信します。
- 異常値フィルタは、初回を受理、前回との差が 10 tenth-mm 以下なら即受理、それより大きい遷移は 10 tenth-mm 以内に収まる値が 3 回続いたときに受理します。

### Base Roll

- UART: `USART2` (`PA2=TX`, `PA3=RX`)、DMA RX、RS485 DE は `PA4`。
- position (`0x54`) のみを状態機械で要求します。応答待ちは 10 ms timeout です。
- 新しい位置、または前回 CAN 送信から 200 ms 経過した同じ位置を CAN ID `0x301` で送信します。

## board2 / board3 実装

board2 と board3 は CAN ID 以外同じ流れです。

1. `init()` で LED を Green ON、Red/Yellow OFF にし、CAN を開始します。
2. `USART1` (`PA9=TX`, `PA10=RX`) と RS485 DE `PA8` でエンコーダを初期化し、position (`0x54`) を要求します。
3. UART RX complete callback で 2 byte を復号し、同じ callback 内で次の position 要求をすぐ発行します。
4. `poll()` で `encoder_get_position()` が true なら CAN 送信用リングバッファへ積み、`can_control_process_tx()` で送信します。

board2 は CAN ID `0x302`、board3 は CAN ID `0x303` を使います。

## board4 実装

board4 は board2/3 と同じエンコーダ送信に加えて、CAN 受信でモーターを制御します。

- 起動時に LED を Green ON、Red/Yellow OFF にします。
- エンコーダは `USART1` + RS485 DE `PA8` で position (`0x54`) を連続取得し、CAN ID `0x304` で送信します。
- サーボは `TIM2_CH2` (`PA1`) を使います。TIM2 は prescaler `63`、period `19999` で、実装上 0.5 ms から 2.5 ms の PWM pulse を 0 度から 270 度へ対応させています。初期角度は 270 度です。
- `servo_control()` は通常モードで 1 度ずつ、高速モードで 4 度ずつ現在角度を変えます。`OPEN` は角度を増やし、`CLOSE` は角度を減らし、`STOP` は現在角度を保持します。
- DC motor は TB67H450FNG 想定で、Motor 1 が `TIM3_CH1/PB4` + `PB6`、Motor 2 が `TIM3_CH2/PB5` + `PB7` です。`IN1=PWM`, `IN2=GPIO` として、停止/正転/逆転/ブレーキを切り替えます。
- TIM3 は prescaler `31`、period `19999` です。duty は `0..100%` を `CCR = duty * 19999 / 100` に変換します。
- `dc_motor_push()` は Motor 1 を正転 100% で 1000 ms、その後逆転 100% で 1000 ms 動かして停止します。この処理は `HAL_Delay()` を含むため、CAN RX callback 中で呼ばれると約 2 秒ブロックします。

## 前提ツール

以下のコマンドが使える状態を想定しています。

- `make`
- `pio` または `platformio`
- `st-info`
- `openocd`
- `nc`

`Makefile` では既定で `pio` コマンドを使用します。必要なら `PIO=platformio` のように上書きできます。

## よく使うコマンド

### ビルド

```bash
make build-board1
make build-board2
make build-board3
make build-board4
make build-all
```

各ボードの PlatformIO 環境 `genericSTM32F103TB` でビルドします。

### 書き込み

```bash
make board1
make board2
make board3
make board4
```

上記はそれぞれ `make upload-boardX` のエイリアスで、ST-Link 経由で書き込みます。

### デバイス確認

```bash
make list
make info
```

- `make list`: PlatformIO から接続デバイス一覧を表示します。
- `make info`: `st-info --probe` で ST-Link の接続情報を確認します。

### printf モニタ

```bash
make monitor
```

または `make monitor-board1` から `make monitor-board4` でも同じスクリプトを実行できます。  
内部では `stlink_monitor.zsh` が `openocd` を起動し、ITM 出力を `nc localhost 3344` で表示します。

### 補助コマンド

```bash
make restructure-board1
make restructure-board2
make restructure-board3
make restructure-board4
```

各ボード配下の `restructure.sh` を実行します。コード生成後の整理など、補助用途のスクリプトとして使う想定です。

## 直接 PlatformIO を使う場合

たとえば `board1` を直接ビルドする場合は以下でも実行できます。

```bash
pio run --project-dir board1 -e genericSTM32F103TB
pio run --project-dir board1 -e genericSTM32F103TB -t upload
```
