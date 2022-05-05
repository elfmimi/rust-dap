# RP2040: Run in SRAM

## 概要

SPIフラッシュからプログラムをSRAMに読み込み、SRAM上で実行するバージョンです。\
RaspberryPi Pico や Xiao RP2040 、Pimoroni  Tiny 2040  などで動作します。\
LTO=リンク時最適化とインラインアセンブラによるnop命令の挿入、\
一番内側のループ内での分岐を避けるためunsafeなunwrap_unchecked()を使用\
などにより高速化を図っています。

| 信号名  | ピン番号 |
|:-------|:--------|
|  TXD   |   GP0   |
|  RXD   |   GP1   |
| SWCLK  |   GP2   |
| SWDIO  |   GP4   |
| GROUND |   GND   |


## ビルド手順

`cargo install elf2uf2-rs` で elf2uf2-rs コマンドをインストールして下さい。

- cargo build --release
- elf2uf2-rs target/thumbv6m-none-eabi/release/rust-dap-rp2040-ram rust-dap-rp2040-ram.uf2

## ライセンス

ライセンスは `Apache-2.0 License` に従います。詳しくは [LICENSE](../../LICENSE) ファイルを確認してください。