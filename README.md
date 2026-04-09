# distribution_board

STM32F103TBU6 を使った distribution board 向けのファームウェア集です。  
`board1` から `board4` までの各基板ごとに PlatformIO プロジェクトが分かれており、ビルドや書き込みはリポジトリ直下の `Makefile` からまとめて実行できます。

## 構成概要

- `board1`: ベース系の制御用ファームウェア。`base_horizon`、`base_roll`、`CAN scheduler` などを含みます。
- `board2`: エンコーダ値を取得して CAN 送信する構成です。
- `board3`: `board2` と同系統の構成で、エンコーダ読み取りと CAN 制御を行います。
- `board4`: エンコーダ、CAN 制御に加えて、サーボと DC モータ制御を含みます。
- 各 `boardX` 配下には `platformio.ini`、`src/`、`include/`、STM32CubeMX の `.ioc` ファイル、HAL ドライバ一式が入っています。

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
