# dth30-fw

DTH30 シリーズ（DT-H30/U2、DT-H50/U2）のチップ ViXS XCode2 で動く、自作のファームウェア。
純正ドライバ `BufFsUSB.sys` に入っているファームウェアの代わりに使うことを目指している。純正のコードやデータは含まない。

**開発中。** いま動くのは Raw ストリーム（スクランブルされたままのフル TS）まで。B-CAS カードと復号（Program ストリーム）はまだ無い。
確認した機種は DT-H50/U2 だけ。

## できること

| 項目 | 状態 |
|---|---|
| 起動、ホストとの握手、ハートビート | 動く |
| コマンド 0（ping）、27（OTP の署名。純正と同じ値になる） | 動く |
| コマンド 24（TS 入力の設定） | 動く |
| コマンド 8 種別 1 / 9 / 10（Raw ストリームを開く、閉じる、flush） | 動く |
| コマンド 25 / 26 / 11（CAS オブジェクト、カードのリセット） | 未実装（結果 2 を返す） |
| コマンド 8 種別 5（Program ストリーム、復号） | 未実装（結果 2 を返す） |

## ビルド

CPU は MIPS 4Km（MIPS32 リリース 1、リトルエンディアン、FPU なし）。Rust にこの CPU 向けの出来合いのターゲットは無いので、
`core` をソースからビルドする。`rust-src` コンポーネントが要る。

```sh
rustup component add rust-src
cd firmware/dth30-fw
RUSTC_BOOTSTRAP=1 cargo build --release
```

`RUSTC_BOOTSTRAP=1` は、stable のツールチェーンで `-Z build-std` を使うためのもの。nightly を使うなら不要（`cargo +nightly build --release`）。
PowerShell では `$env:RUSTC_BOOTSTRAP = '1'` としてから `cargo build --release`。

できるのは ELF なので、生のイメージに直す。`rust-objcopy` はツールチェーンに入っている。

```sh
"$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin/rust-objcopy" \
    -O binary target/mipsel-xcode2/release/dth30-fw dth30-fw.bin
```

## 使い方

`--firmware` に `BufFsUSB.sys` の代わりに渡す。イメージの +0x100 に印（`DTH3`）があり、`dth30-core` はそれを見てハッシュの照合なしで読み込む。

```sh
dth30-diag fw dth30-fw.bin
dth30-diag bring-up --firmware dth30-fw.bin
dth30-diag record 27 --raw -o raw.ts --firmware dth30-fw.bin
```

## 作り

- コア 0 だけを使う。割り込みは使わず、ホストからの要求・DMA の完了・ストリームのリングを 1 つのループで見て回る。
- ホストやチップのハードウェアと共有するメモリは、すべて kseg1（キャッシュなし）で触る。自分のコードとデータは kseg0（キャッシュあり）。
- 例外はすべて致命エラーとして扱い、純正と同じ場所に記録を残す（FB 0x530 = 0xDEADC0DE、場所、sp、コア番号。FB 0x560 = 原因、EPC、Status、Cause、BadVAddr、ra）。

| ファイル | 内容 |
|---|---|
| `src/boot.rs` | イメージの先頭（ベクタ表）、リセットからの起動、例外の入口 |
| `src/hw.rs` | レジスタと SDRAM へのアクセス、時計 |
| `src/host.rs` | 握手、コマンド、メッセージ、ハートビート |
| `src/otp.rs` | コマンド 27 |
| `src/dma.rs` | DMA エンジン |
| `src/ts.rs` | TS 入力、デマルチプレクサ、Raw ストリーム |

### メモリの使い方（SDRAM の物理アドレス）

| 範囲 | 内容 |
|---|---|
| 0x000000〜0x00048F | 例外ベクタ（ホストが置き、起動後にファームウェアが置き直す） |
| 0x000010、0x000020〜0x00004C、0x0000B0 | ハートビート、握手、ホストの引数、起動の進み具合 |
| 0x0000C0、0x0000C4 | このファームウェアの印（`DTH3`）と版 |
| 0x001000〜0x060FFF | ホストに渡すプール（コマンドのスロットなど） |
| 0x061000〜0x0623FF | メッセージの本体（8 個） |
| 0x062800〜0x0628FF | DMA の記述子 |
| 0x064000〜0x065FFF | 使わないチャネルの書き込み先 |
| 0x100000〜 | イメージ、`.bss`、スタック |
| 0x800000〜0x9D5FFF | Raw ストリームのリング |

### 純正のファームウェアと違うところ

- 知らないコマンドでは止まらず、結果 2 を返す。
- flush（コマンド 10）は何も送らない（純正は端数を送ってメッセージ 0x0C を出す）。close の後、ホストはエンドポイントの残りを捨てればよい。
- Raw を開いた直後の 1 チャンク（256 パケット）は捨てる。チャネルが前回の残りを先に出すため。
- ホストが読まずにリングがあふれたら、いま書いているチャンクの先頭から送り直す（送るデータは常にパケットの境界から始まる）。
- ウォッチドッグは使わない。
