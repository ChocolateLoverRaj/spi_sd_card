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

## Interactions between caller and library
Caller: I want to do <high level thing>, such as initializing the card, reading, or writing.
Library: Be prepared to transfer at least A bytes, writing the first B bytes with this value, and then as you transfer call my fn.
Caller: Prepares buffer of >=A bytes, writes B bytes to it, fills rest with `0xFF`.
Caller: Transfers some, maybe all of bufffer
Caller: Calls library fn
Library: Continue transfering at least A more bytes, expecting B more bytes until the process is done, with a max of C bytes.

## Command 0: Reset
Controller: The microcontroller running this library
Peripheral: The (micro)SD card

First 6 bytes: Controller sends exactly 6 bytes, which is the command, don't care what peripheral sends
Next N bytes (could be as low as 0, usually 2, max maybe 1000 until we timeout): Controller sends `0xFF`s, peripheral sends `0xFF`s
Next 1 byte: Controller sends `0xFF`, peripheral sends `0b0xxxxxxx` (R1).

EXPECTED_N = 2

Efficient implementation with `embedded_hal_async::spi::SpiBus`:
- Obtain an `&mut [u8]` with len 6 + EXPECTED_N + 1
- Fill first 6 bytes with command
- Fill remaining bytes with `0xFF`
- `transfer_in_place`
- If no response received:
  - Obtain an `&mut [u8]` with len of whatever we think it will take (idk), fill with `0xFF`
  - `transfer_in_place`
  - If we reach the timeout, command 0 fails. We can retry sending command 0 again.

Efficient implementation with ESP32 DMA:
- Set up TX buffers / segments
  - Obtain a DMA `&mut [u8]` with len 6
    - Fill first 6 bytes with command
  - Obtain a DMA `&mut [u8]` with len of >= EXPECTED_N + 1
    - Fill with `0xFF`
  - Set up segments so that the first segment, then 2nd, then loops to 2nd again
- Set up RX buffers / segments
  - Obtain a DMA `&mut [u8]` with len 6 + EXPECTED_N + 1
  - Obtain two DMA `&mut [u8]` with whatever len
  - Set up  segments so that the first segment, then a loop of larger segments
- Start TX / RX transfer
- Analyze RX until either the command is complete or there's a timeout

## Caller complexity vs performance
### Least complex but terrible performance: transfering 1 byte at a time
Caller: I want to do <high level command>
Loop:
  Library: Transfer this byte and tell me what was received, or we're done.
  Caller: Transfers 1 byte
  Caller: Calls library fn
