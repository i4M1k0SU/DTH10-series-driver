# DTH10/DTH30 series driver

BUFFALO DTH10/DTH30 series ユーザーモード実装

[![CI](https://github.com/i4M1k0SU/DTH10-series-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/i4M1k0SU/DTH10-series-driver/actions/workflows/ci.yml)

## 対象機種

USB VID は `0x0411` (BUFFALO) で共通。

一部のKit (LGDT3305 など) はベンダードライバーの解析から実装したもので、搭載機種も不明。

### DTH10 series

| 型番            | PID      | 発売時期 | 受信         | 検証 | 備考 |
|-----------------|----------|---------|--------------|:--:|-------|
| DT-H10/U2       | `0x015F` | 2009-06 | 地上         | ✅ | - |
| DT-H10/PCI      | `0x016B` | 2009-06 | 地上         | -  | - |
| DT-H10/U7       | `0x017B` | 2009-10 | 地上         | -  | - |
| DT-H33/U2       | `0x017C` | 2009-11 | 地上         | -  | - |
| DT-H33/PCI      | `0x017D` | 2009-11 | 地上         | -  | - |
| DT-F100/U2      | `0x0186` | 2009-12 | 地上         | ✅ | raw モードなし |
| DT-H55/U2W      | `0x0187` | 2010-04 | 地上x2       | -  | - |
| DT-H70/U2       | `0x0199` | 2010-06 | 地上, BS, CS | ✅ | - |
| DT-H70/PCIE     | `0x01BE` | 2010-10 | 地上, BS, CS | -  | - |
| DT-F110/U2      | `0x01D2` | 2010-11 | 地上         | -  | - |
| DT-H11/U2       | `0x01DA` | 2010-11 | 地上         | ✅ | - |
| DT-F200/U2W     | `0x01DB` | 2010-12 | 地上x2       | -  | - |
| DT-H71/U2       | `0x01FF` | 2011-10 | 地上, BS, CS | -  | - |
| DT-F120/U2      | `0x0202` | 2011-11 | 地上         | -  | - |
| DT-H70B/PCIE    | `0x0200` | 2011-12 | 地上, BS, CS | -  | - |
| DT-H30/PCIEM-TS | `0x0218` | -       | 地上         | -  | 東芝 dynabook REGZA PC に搭載の内蔵チューナー |

### DTH30 series

| 型番            | PID                 | 発売時期 | 受信         | 検証 | 備考 |
|-----------------|---------------------|---------|--------------|:--:|-------|
| DT-H30/U2       | `0x0128` , `0x0144` | 2008-05 | 地上         | -  | - |
| DT-H50/U2       | `0x0129`            | 2008-07 | 地上         | ✅ | - |
| DT-H50/U2W      | `0x012A`            | 2008-07 | 地上x2       | -  | INF に定義されているが未発売機種 |

## 使い方

### 必要ファイルの用意

以下の表に従い、使用する機種で必要なファイルを純正ソフトウェアやドライバから入手する。

付属の CD や公式サイトからダウンロードしたインストーラーを展開する (もしくはインストールして取り出す)。

| Series | ファイル        | サイズ (bytes) | SHA256                                                             |
|--------|----------------|---------------:|--------------------------------------------------------------------|
| DTH10  | `CoBDACom.dll` |        351,232 | `7F4CABFD0B104C0E739F1172A10A98FCF24377C6947E3AB6AC0FA856353BCC15` |
| DTH30  | `BufFsUSB.sys` |      1,486,392 | `60872BD9225A8FD7EF402B83CCBCBAE58EDEE40D05C73DA9EBA8BBA90DC4E286` |

### BonDriver

1. Zadig 等で対象機器に WinUSB を割り当てる
1. 以下のファイルを `TVTest.exe` と同じ階層にコピー
    - `BonDriver_DTH.dll`
      - TVTest と同じアーキテクチャのものを使う
      - 地デジと BS/CS を同時に視聴する場合は、コピーして `BonDriver_DTH-T.dll`, `BonDriver_DTH-S.dll` にリネーム
    - `BonDriver_DTH.ini`
      - 同時に視聴する場合は、DLL と同様に `BonDriver_DTH-T.ini`, `BonDriver_DTH-S.ini` にリネーム
    - `dth.exe`
    - DTH10 series: `CoBDACom.dll`
    - DTH30 series: `BufFsUSB.sys`
1. 必要に応じて `BonDriver_DTH.ini` (`BonDriver_DTH-T.ini`, `BonDriver_DTH-S.ini`) を編集
    - 複数台接続する場合、 `Serial=` の指定が必要
      - `dth list` で確認。未指定では動作しない
      - DTH30 series は USB シリアル番号を持たないため、 `dth list` に表示されるユニット名 (`DTH30-0123` など) を指定する
    - `BonDriver_DTH-T.ini`
      - `Spaces=GR`
    - `BonDriver_DTH-S.ini`
      - `Spaces=BS,CS`
1. TVTest を起動し、チャンネルスキャン、視聴

### `dth` コマンド

オプションは `dth <コマンド> --help` を参照。

#### DTH10 series

device モード (既定) では、`CoBDACom.dll` を `dth` 実行ファイルと同じディレクトリに置くか、`tune` / `serve` の `--app-keys <ファイルパス>` オプションで指定する。DLL は実行せず認証情報の読み取りに使うため、Linux / macOS であっても必要。

#### DTH30 series

`BufFsUSB.sys` を `dth` 実行ファイルと同じディレクトリに置くか、`tune` / `info` / `serve` の `--firmware <ファイルパス>` オプションで指定する。

```
# デバイス一覧
dth list

# 地デジ 27ch の TS を取得。device モード (デバイス内蔵 B-CAS リーダーで復号, DTH10 series では `CoBDACom.dll` が必要)
dth tune -c 27 > ch27.ts
# BS15_1 の TS を 10 秒取得。`-o` でも TS をファイルに保存できる
dth tune -c BS15_1 -t 10 -o bs.ts --stats -v

# 地デジ 27ch の TS を放送波そのまま (復号せず) 10 秒取得。BS/CS 対応機種においては、 raw モードを一度使うと、電源を入れ直すまで BS/CS の受信が行えない。BS/CS は `--mode` を付けない device モードのみ扱える。なお、機種によっては、raw モードが存在しない (DT-F100/U2 など)。
dth tune -c 27 --mode raw -t 10 > ch27.ts

# Mirakurun / mirakc 向け (Linux, ソケットのディレクトリを作成・使用できる権限が必要)
dth serve --socket /run/dth/dth.sock    # サーバー起動。ソケットで接続を待ち受ける
dth stream --socket /run/dth/dth.sock -c BS15_0 --program 101 # クライアント動作。ソケット経由で指定した TS を取得する
```

## 開発者向け

### crates

以下の層に分かれる。

- 機種に依存しない共通層 (`dth-common`, `dth-proto`)
- 機種ごとの層 (`dth10-*`, `dth30-*`)
- アプリケーション (`dth-cli`, `dth-bondriver`)

機種ごとの層は `*-core` (USB 非依存) → `*-usb` (トランスポート) → `*-backend` (共通層の `Session` / `Backend` の実装) と積み上がり、`dth-cli` はすべての `*-backend` を順に使う。

| crate | 内容 |
|---|---|
| `crates/dth-common` | 機種に依存しない共通部品 (`std` のみに依存): エラー型 (`error`), チャンネル表・チャンネル名の解釈 (`channels`), ストリーム設定 (raw / device モード、復号する番組。`stream`), TS の受け渡し (`TsQueue`, `queue`), 開いたユニットへの操作 (`Session`, `session`), 機種ファミリーごとのユニットの列挙・事前確認・オープン・情報表示 (`Backend`, `backend`) |
| `crates/dth-proto` | `dth serve` とクライアント (`dth stream`, BonDriver) の間の要求・応答行 (`proto`) と、それを運ぶソケット (Unix ドメインソケット / TCP。`ipc`) |
| `crates/dth10-core` | USB 非依存のプロトコル層: 制御転送エンコード (`protocol`), デバイス情報 TLV (`tlv`), GPIO/I2C/ストリーム制御/PID フィルタ (`device`), 全キット共通の型 (`kit`), TC90517 + MxL13xRF の地デジキット (`tc90517`), MxL13xRF (`mxl13xrf`), キット名による地デジ側キットの選択 (`terrestrial`), PN2031 / BQ200X (ISDB-T) と LGDT3305 (ATSC) のキット (`pn2031`, `bq200x`, `lgdt3305`。実機未検証), BS/CS キット (`lj101`), 割り込みイベント (`events`), リンク暗号の復号 (`cepm`) |
| `crates/dth10-usb` | `nusb` によるトランスポート (Windows: WinUSB / Linux: usbfs / macOS: IOKit) とバルク連続リーダ |
| `crates/dth10-diag` | 開発者向けのハードウェア診断ツール `dth10-diag`: `info [--raw]`, `gpio-read`, `raw`, `events`, `i2c-read`, `i2c-write`, `tune` (ISDB-T の同調・ロック・信号状態。`--file` で raw モード記録), `tune-sat` (ISDB-S の同調・ロック・信号状態・TMCC の TS-ID 表), `scan`, `cepm`, `reset`, `hub-power` (Linux)。共通オプション `--serial`, `--recipient`, `--tuner`, `--drain-events`, `-v`。生のベンダーコマンド・単独の CEPM ハンドシェイク・hub-power は電源を入れ直すまでデバイスをフリーズさせることがあるため、診断用として提供。記録・視聴は `dth` |
| `crates/dth10-backend` | DTH10 series の `Session` / `Backend` 実装: チューナー (`tuner`。USB オープン、キット初期化、選局、CEPM 復号付きリーダスレッド), device モードの PAT/PMT 収集 (`psi`), DTH10 series 固有の設定 (`config`), ユニットの列挙と `CoBDACom.dll` の認証情報の読み込み (`backend`) |
| `crates/dth30-core` | DTH30 series (ViXS XCode2 + PLX NET2280) の USB 非依存層: NET2280 の初期化 (`net2280`), ViXS のメモリ初期化・FW の抽出と起動・コマンド・I2C・ストリーム (`vixs`), フロントエンドのキット (`frontend`), PAT/PMT (`psi`), デバイス全体 (`device`) |
| `crates/dth30-usb` | `nusb` による NET2280 のトランスポート (レジスタアクセス、DMA、バルク連続リーダ) |
| `crates/dth30-diag` | 開発者向けのハードウェア診断ツール `dth30-diag`: `list`, `info`, `probe`, `fw`, `bring-up`, `i2c-read`, `i2c-write`, `tune`, `record` (Raw / Program ストリームの記録) |
| `crates/dth30-backend` | DTH30 series の `Session` / `Backend` 実装: FW の場所の解決、内蔵 B-CAS カードの監視 (抜き差し後の再有効化)、Raw / Program ストリームの読み出し、ランプ制御 |
| `crates/dth-cli` | パッケージ `dth-cli`、バイナリ `dth`。単体のチューナーコマンド (`list` / `info` / `tune`)。Linux (usbfs)、Windows (WinUSB)、macOS (IOKit) で動き、TS を標準出力に書く。`serve` (デバイスを握り続けるデーモン) と `stream` (そのクライアント) で地デジと BS/CS を同時に配れる (Linux ではレコーダー向け、Windows では BonDriver が `dth.exe serve` をサーバーとして起動する。`status` でデーモンの状態を見る)。Mirakurun / mirakc のチューナーコマンド用 |
| `crates/dth-bondriver` | TVTest 用 BonDriver DLL (`BonDriver_DTH.dll`, IBonDriver2)。`dth.exe serve` を起動して接続するクライアント |

### ビルド

ターゲットは `rust-toolchain.toml` により自動で追加される。

Windows 向けには Visual Studio Build Tools の C++ ビルド環境と Windows SDK、macOS 向けには Xcode Command Line Tools が必要。Linux 向けは `.cargo/config.toml` で `rust-lld` を指定しているため、クロス C ツールチェーンは不要。

ビルド成果物は `target/<ターゲット名>/release/` に出力される。

```
# Windows (BonDriver DLL + dth.exe + dth10-diag.exe + dth30-diag.exe)
cargo build --release --target x86_64-pc-windows-msvc   # Windows x86_64
cargo build --release --target i686-pc-windows-msvc     # Windows i686
cargo build --release --target aarch64-pc-windows-msvc  # Windows aarch64 (ARM64)

# Linux (dth + dth10-diag + dth30-diag)
cargo build --release -p dth-cli -p dth10-diag -p dth30-diag --target x86_64-unknown-linux-musl       # Linux x86_64
cargo build --release -p dth-cli -p dth10-diag -p dth30-diag --target i686-unknown-linux-musl         # Linux i686
cargo build --release -p dth-cli -p dth10-diag -p dth30-diag --target aarch64-unknown-linux-musl      # Linux aarch64
cargo build --release -p dth-cli -p dth10-diag -p dth30-diag --target armv7-unknown-linux-musleabihf  # Linux armv7

# macOS (dth + dth10-diag + dth30-diag　*macOS 上でビルドする必要がある)
cargo build --release -p dth-cli -p dth10-diag -p dth30-diag --target x86_64-apple-darwin   # macOS x86_64 (Intel Mac)
cargo build --release -p dth-cli -p dth10-diag -p dth30-diag --target aarch64-apple-darwin  # macOS aarch64 (Apple Silicon)
```

### テスト

テストは実機不要 (プロトコルの往復、IPC、デーモンのスロット判定、DLL 境界の panic 捕捉)。

```
cargo test --workspace

# MSVC で C++ 側をビルドし、本物の `dynamic_cast` で RTTI を検証する
cargo test -p dth-bondriver --features rtti-check
```

## ライセンス

[MIT LICENSE](LICENSE)
