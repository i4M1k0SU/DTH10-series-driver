# DTH10 series driver

BUFFALO DTH10 シリーズ ユーザーモード実装

[![CI](https://github.com/i4M1k0SU/DTH10-series-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/i4M1k0SU/DTH10-series-driver/actions/workflows/ci.yml)

## 対象機種

| 型番 | PID | 発売時期 | 受信 | 検証 |
|---|---|---|---|:-:|
| DT-H10/U2 | `0x015F` | 2009-06 | 地上 | - |
| DT-H10/PCI | `0x016B` | 2009-06 | 地上 | - |
| DT-H10/U7 | `0x017B` | - | 地上 | - |
| DT-H33/U2 | `0x017C` | 2009-11 | 地上 | - |
| DT-H33/PCI | `0x017D` | 2009-11 | 地上 | - |
| DT-F100/U2 | `0x0186` | 2009-12 | 地上 | ✅ |
| DT-H55/U2W | `0x0187` | 2010-04 | 地上 | - |
| DT-H70/U2 | `0x0199` | 2010-06 | 地上, BS, CS | ✅ |
| DT-H70/PCIE | `0x01BE` | 2010-10 | 地上, BS, CS | - |
| DT-F110/U2 | `0x01D2` | 2010-11 | 地上 | - |
| DT-H11/U2 | `0x01DA` | 2010-11 | 地上 | - |
| DT-F200/U2W | `0x01DB` | 2010-12 | 地上 | - |
| DT-H71/U2 | `0x01FF` | - | 地上, BS, CS | - |
| DT-F120/U2 | `0x0202` | - | 地上 | - |
| DT-H70B/PCIE | `0x0200` | - | 地上, BS, CS | - |
| DT-H30/PCIEM-TS | `0x0218` | - | - | - |

USB VID は `0x0411` (BUFFALO) で共通。

一部のKit (PN2031、LGDT3305 など) はベンダードライバーの解析から実装したもので、搭載機種も不明。

## 使い方

### 共通 - `CoBDACom.dll` の用意

デバイス内蔵 B-CAS リーダーの利用や BS/CS の視聴を行う場合は、 `CoBDACom.dll` を用意する必要がある。

純正ソフトウェアから入手する。

付属の CD や公式サイトからダウンロードしたインストーラーを展開する (もしくはインストールして取り出す)。

#### 動作確認済みのファイル
- 351,232 bytes
- SHA256: `7F4CABFD0B104C0E739F1172A10A98FCF24377C6947E3AB6AC0FA856353BCC15`

### BonDriver

1. Zadig 等で対象機器に WinUSB を割り当てる
1. 以下のファイルを `TVTest.exe` と同じ階層にコピー
    - `BonDriver_DTH10.dll`
      - TVTest と同じアーキテクチャのものを使う
      - 地デジと BS/CS を同時に視聴する場合は、コピーして `BonDriver_DTH10-T.dll`, `BonDriver_DTH10-S.dll` にリネーム
    - `BonDriver_DTH10.ini`
      - 同時に視聴する場合は、DLL と同様に `BonDriver_DTH10-T.ini`, `BonDriver_DTH10-S.ini` にリネーム
    - `dth10.exe`
    - `CoBDACom.dll`
1. 必要に応じて `BonDriver_DTH10.ini` (`BonDriver_DTH10-T.ini`, `BonDriver_DTH10-S.ini`) を編集
    - 複数台接続する場合、 `Serial=` の指定が必要
      - `dth10 list` で確認。未指定では動作しない
    - `BonDriver_DTH10-T.ini`
      - `Spaces=GR`
    - `BonDriver_DTH10-S.ini`
      - `Spaces=BS,CS`
1. TVTest を起動し、チャンネルスキャン、視聴

### `dth10` コマンド

オプションは `dth10 <コマンド> --help` を参照。

device モード (既定) では、`CoBDACom.dll` を `dth10` 実行ファイルと同じディレクトリに置くか、`tune` / `serve` の `--app-keys <ファイルパス>` オプションで指定する。DLL は実行せず認証情報の読み取りに使うため、Linux / macOS であっても必要。

```
# デバイス一覧
dth10 list

# 地デジ 27ch の TS を取得。device モード (デバイス内蔵 B-CAS リーダーで復号, `CoBDACom.dll` が必要)
dth10 tune -c 27 > ch27.ts
# BS15_1 の TS を 10 秒取得。`-o` でも TS をファイルに保存できる
dth10 tune -c BS15_1 -t 10 -o bs.ts --stats -v

# 地デジ 27ch の TS を放送波そのまま (復号せず) 10 秒取得。raw モードを一度使うと、電源を入れ直すまで BS/CS の受信が行えない。BS/CS は `--mode` を付けない device モードのみ扱える。なお、機種によっては、raw モードが存在しない (DT-F100/U2 など)。
dth10 tune -c 27 --mode raw -t 10 > ch27.ts

# Mirakurun / mirakc 向け (Linux, ソケットのディレクトリを作成・使用できる権限が必要)
dth10 serve --socket /run/dth10/dth10.sock    # サーバー起動。ソケットで接続を待ち受ける
dth10 stream --socket /run/dth10/dth10.sock -c BS15_0 --program 101 # クライアント動作。ソケット経由で指定した TS を取得する
```

## 開発者向け

### crates

| crate | 内容 |
|---|---|
| `crates/dth10-core` | USB 非依存のプロトコル層: 制御転送エンコード (`protocol`), デバイス情報 TLV (`tlv`), GPIO/I2C/ストリーム制御/PID フィルタ (`device`), 全キット共通の型 (`kit`), TC90517 + MxL13xRF の地デジキット (`tc90517`), MxL13xRF (`mxl13xrf`), キット名による地デジ側キットの選択 (`terrestrial`), PN2031 / BQ200X (ISDB-T) と LGDT3305 (ATSC) のキット (`pn2031`, `bq200x`, `lgdt3305`。実機未検証), BS/CS キット (`lj101`), 割り込みイベント (`events`), リンク暗号の復号 (`cepm`) |
| `crates/dth10-usb` | `nusb` によるトランスポート (Windows: WinUSB / Linux: usbfs / macOS: IOKit) とバルク連続リーダ |
| `crates/dth10-diag` | 開発者向けのハードウェア診断ツール `dth10-diag`: `info [--raw]`, `gpio-read`, `raw`, `events`, `i2c-read`, `i2c-write`, `tune` (ISDB-T の同調・ロック・信号状態。`--file` で raw モード記録), `tune-sat` (ISDB-S の同調・ロック・信号状態・TMCC の TS-ID 表), `scan`, `cepm`, `reset`, `hub-power` (Linux)。共通オプション `--serial`, `--recipient`, `--tuner`, `--drain-events`, `-v`。生のベンダーコマンド・単独の CEPM ハンドシェイク・hub-power は電源を入れ直すまでデバイスをフリーズさせることがあるため、診断用として提供。記録・視聴は `dth10` |
| `crates/dth10-tuner` | BonDriver と `dth10` コマンドが共有するチューナーロジック (チャンネル表、PAT/PMT 収集、device モードの PID リスト、CEPM 復号付きリーダスレッド、`TsQueue`) |
| `crates/dth10-cli` | パッケージ `dth10-cli`、バイナリ `dth10`。単体のチューナーコマンド (`list` / `info` / `tune`)。Linux (usbfs)、Windows (WinUSB)、macOS (IOKit) で動き、TS を標準出力に書く。`serve` (デバイスを握り続けるデーモン) と `stream` (そのクライアント) で地デジと BS/CS を同時に配れる (Linux ではレコーダー向け、Windows では BonDriver が `dth10.exe serve` をサーバーとして起動する。`status` でデーモンの状態を見る)。Mirakurun / mirakc のチューナーコマンド用 |
| `crates/dth10-bondriver` | TVTest 用 BonDriver DLL (`BonDriver_DTH10.dll`, IBonDriver2) |

### ビルド

ターゲットは `rust-toolchain.toml` により自動で追加される。

Windows 向けには Visual Studio Build Tools の C++ ビルド環境と Windows SDK、macOS 向けには Xcode Command Line Tools が必要。Linux 向けは `.cargo/config.toml` で `rust-lld` を指定しているため、クロス C ツールチェーンは不要。

ビルド成果物は `target/<ターゲット名>/release/` に出力される。

```
# Windows (BonDriver DLL + dth10.exe + dth10-diag.exe)
cargo build --release --target x86_64-pc-windows-msvc   # Windows x86_64
cargo build --release --target i686-pc-windows-msvc     # Windows i686
cargo build --release --target aarch64-pc-windows-msvc  # Windows aarch64 (ARM64)

# Linux (dth10 + dth10-diag)
cargo build --release -p dth10-cli -p dth10-diag --target x86_64-unknown-linux-musl       # Linux x86_64
cargo build --release -p dth10-cli -p dth10-diag --target i686-unknown-linux-musl         # Linux i686
cargo build --release -p dth10-cli -p dth10-diag --target aarch64-unknown-linux-musl      # Linux aarch64
cargo build --release -p dth10-cli -p dth10-diag --target armv7-unknown-linux-musleabihf  # Linux armv7

# macOS (dth10 + dth10-diag　*macOS 上でビルドする必要がある)
cargo build --release -p dth10-cli -p dth10-diag --target x86_64-apple-darwin   # macOS x86_64 (Intel Mac)
cargo build --release -p dth10-cli -p dth10-diag --target aarch64-apple-darwin  # macOS aarch64 (Apple Silicon)
```

### テスト

テストは実機不要 (プロトコルの往復、IPC、デーモンのスロット判定、DLL 境界の panic 捕捉)。

```
cargo test --workspace

# MSVC で C++ 側をビルドし、本物の `dynamic_cast` で RTTI を検証する
cargo test -p dth10-bondriver --features rtti-check
```

## ライセンス

[MIT LICENSE](LICENSE)
