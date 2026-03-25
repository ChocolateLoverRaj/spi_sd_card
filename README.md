# `spi_sd_card`
[![Crates.io Version](https://img.shields.io/crates/v/spi_sd_card)](https://crates.io/crates/spi_sd_card)
[![docs.rs](https://img.shields.io/docsrs/spi_sd_card)](https://docs.rs/spi_sd_card/latest/spi_sd_card/)

A Rust embedded library for using SD cards through SPI.

## Features
- `no_std` without `alloc`
- No `unsafe` code
- Very fast speeds (fast enough to stream audio)
- Read data
- Writing is not implemented yet

## Usage
- Create a `SpiSdCard`
- Call the `init_card` method to get a `SdCardDisk`
- On the disk you can call `read`, `capacity`, and `get_status` (to check if the card is still present)

Also note that this library requires some features of the `crc` library that are currently not published to `crates.io`. Adding this to your `Cargo.toml` is probably needed:
```toml
[patch.crates-io]
crc = { git = "https://github.com/mrhooray/crc-rs" }
```

## Example
See https://github.com/ChocolateLoverRaj/rust-esp32c3-examples/blob/ca5ab80f178cc1bf08281818cf2877f046f00d45/async_sd_card/src/main.rs for a full example.
