#![no_std]
#![allow(async_fn_in_trait)]
#[cfg(feature = "esp32c3")]
mod dma;

use core::{cmp::min, num::NonZero, time::Duration};

use defmt::warn;
#[cfg(feature = "esp32c3")]
pub use dma::*;
mod shared_spi_bus;
use embedded_timers::clock::Clock;
pub use shared_spi_bus::*;
mod disk;

mod structs;
mod util;
pub use disk::*;
pub use util::*;

use crc::{CRC_7_MMC, CRC_16_XMODEM, Crc};
use embassy_time::{Instant, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{
    delay::DelayNs,
    spi::{ErrorType, SpiBus},
};
pub use structs::*;

pub fn format_command(command_index: u8, argument: u32) -> [u8; 6] {
    let mut command: [u8; 6] = Default::default();
    command[0] = {
        let mut byte = CommandByte0(Default::default());
        byte.set_start_bit(false);
        byte.set_transmission_bit(true);
        byte.set_command_index(command_index);
        byte.0
    };
    command[1..5].copy_from_slice(&argument.to_be_bytes());
    command[5] = {
        let mut byte = CommandByte5(Default::default());
        byte.set_crc7(Crc::<u8>::new(&CRC_7_MMC).checksum(&command[..5]));
        byte.set_end_bit(true);
        byte.0
    };

    command
}

/// Some errors, such as the SpiBus and CsPin error, can happen from any command
/// Other errors are command-specific and may never occur in certain commands
#[derive(Debug)]
pub enum Error<BusError, CsError> {
    /// Error doing SPI transactions
    /// If this error happens, the CS pin might still be set low
    SpiBus(BusError),
    /// Error setting the level of the CS pin
    /// If this happens, the CS pin might still be set low
    CsPin(CsError),
    /// Many commands have this error if the response R1 from the SD card is something that it should not be
    BadR1(R1),
    /// Command 8 - the card's check pattern does not match the check pattern we sent to the card
    CheckPatternMismatch(u8),
    /// Command 8 - the SD Card does not support 3.3V
    VoltageNotSupported,
    /// For many commands, the card sends a checksum with its data response.
    /// If the checksum doesn't match, then this error will occur.
    InvalidChecksum,
    /// Received a byte after the response that is invalid
    BadData(u8),
    /// The card did not respond. This probably means that the card was removed.
    NoResponse,
    /// When we attempted to initialize the card, it responded, but not with the expected response.
    InitFailed,
}

/// Does not modify CS
pub async fn card_command<S: SpiBus>(spi_bus: &mut S, command: &[u8; 6]) -> Result<R1, S::Error> {
    spi_bus.write(command).await?;
    let mut bytes_until_response = 0;
    let r1 = loop {
        let mut buffer = [0xFF; 1];
        spi_bus.transfer_in_place(&mut buffer).await?;
        let r1 = R1::from_bits_retain(buffer[0]);
        if !r1.contains(R1::BIT_7) {
            break r1;
        } else {
            bytes_until_response += 1;
        }
    };
    defmt::info!("bytes until response: {}", bytes_until_response);
    Ok(r1)
}

type Command = [u8; 6];

/// This is now many bytes between the end of a command and the start of a response (R1) we expect.
/// It is better to overestimate and read extra bytes than to have to do an additional transaction.
/// Based on my testing of 4 MicroSD cards, the most I've seen is 2
/// But if other people have MicroSD cards that take longer, we can increase this const.
const BYTES_UNTIL_RESPONSE: usize = 2;
/// The number of 0xFF bytes returned by the card until we consider it a timeout
const CMD_0_MAX_BYTES_UNTIL_RESPONSE: usize = 100;
/// The number of 0xFF bytes returned by the card until we consider it a timeout
const CMD_59_MAX_BYTES_UNTIL_RESPONSE: usize = 100;

/// The buffer can have any data
pub async fn card_command_2<S: SpiBus>(
    spi_bus: &mut S,
    command: &Command,
    buffer: &mut [u8; size_of::<Command>() + BYTES_UNTIL_RESPONSE + 1],
) -> Result<R1, S::Error> {
    let (command_buffer, dummy_buffer) = buffer.split_at_mut(size_of::<Command>());
    command_buffer.copy_from_slice(command);
    dummy_buffer.fill(0xFF);

    spi_bus.transfer_in_place(buffer).await?;

    let response_buffer = &mut buffer[size_of::<Command>()..];
    let mut i = 0;
    loop {
        if i == response_buffer.len() {
            break;
        }
        let r1 = R1::from_bits_retain(response_buffer[i]);
        if !r1.contains(R1::BIT_7) {
            return Ok(r1);
        }
        i += 1;
    }

    // If we still didn't get a response, we probably need to increase BYTES_UNTIL_RESPONSE, or there's no card present
    #[cfg(feature = "defmt")]
    defmt::warn!("Card didn't respond within expected number of bytes");

    todo!("some kind of timeout / maximum bytes we will attempt to read a response")
}

pub async fn command_0<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let result = {
        let before = Instant::now();
        let r1 = card_command_2(spi_bus, &format_command(0, 0), &mut [Default::default(); _])
            .await
            .map_err(Error::SpiBus)?;
        let after = Instant::now();
        defmt::info!("card command took {} us", (after - before).as_micros());
        if r1 == R1::IN_IDLE_STATE {
            Ok(())
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

/// This function assumes that you are providing 2.6V-3.6V to the SD card.
pub async fn command_8<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
    check_pattern: u8,
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    let mut buffer = [0xFF; size_of::<Command>() + BYTES_UNTIL_RESPONSE + 5];
    buffer[..size_of::<Command>()].copy_from_slice(&format_command(8, {
        let mut argument = Command8Argument(Default::default());
        argument.set_pcie1_2v_support(false);
        argument.set_pcie_availability(false);
        argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
        argument.set_check_pattern(check_pattern);
        argument.0
    }));
    cs.set_low().map_err(Error::CsPin)?;
    spi_bus
        .transfer_in_place(&mut buffer)
        .await
        .map_err(Error::SpiBus)?;
    let read_bytes = &buffer[6..];
    let mut i = 0;
    let result = loop {
        if i == read_bytes.len() {
            break None;
        }
        let r1 = R1::from_bits_retain(read_bytes[i]);
        if !r1.contains(R1::BIT_7) {
            break Some(if r1 == R1::IN_IDLE_STATE {
                let response_check_pattern = read_bytes[i + 4];
                if response_check_pattern == check_pattern {
                    let byte_3 = R7Byte3(read_bytes[i + 3]);
                    if byte_3
                        .get_voltage_accepted()
                        .contains(VoltageAccpted::_2_7V_3_6V)
                    {
                        Ok(())
                    } else {
                        Err(Error::VoltageNotSupported)
                    }
                } else {
                    Err(Error::CheckPatternMismatch(response_check_pattern))
                }
            } else {
                Err(Error::BadR1(r1))
            });
        }
        i += 1;
    };

    let result = result.expect("TODO: new attempts or timeout");

    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;

    result
}

pub async fn command_58<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<Ocr, Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(spi_bus, &format_command(58, 0))
        .await
        .map_err(Error::SpiBus)?;
    // We're not allowed to talk to other SPI devices between sending the command and receiving a response
    let result = {
        // CMD58 can be called before or after ACMD41
        // The spec suggests calling it before ACMD41 to check that the voltage is compatible
        // So it's ok if R1 is idle or not idle
        if r1 == R1::IN_IDLE_STATE || r1.is_empty() {
            let mut buffer = [0xFF; 4];
            spi_bus
                .transfer_in_place(&mut buffer)
                .await
                .map_err(Error::SpiBus)?;
            let ocr = Ocr::from_bits_retain(u32::from_be_bytes(buffer));
            Ok(ocr)
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_55<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    spi_bus
        .write(&format_command(55, 0))
        .await
        .map_err(Error::SpiBus)?;
    // We're not allowed to talk to other SPI devices between sending the command and receiving a response
    let result = loop {
        // Timer::after(Duration::from_millis(1000)).await;
        let mut buffer = [0xFF; 1];
        spi_bus
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SpiBus)?;
        let r1 = R1::from_bits_retain(buffer[0]);
        if !r1.contains(R1::BIT_7) {
            // At thsi point, the card could be ready or not ready
            // We can treat either R1 as ok
            break if r1 == R1::IN_IDLE_STATE || r1.is_empty() {
                Ok(())
            } else {
                #[cfg(feature = "defmt")]
                defmt::error!("r1: 0b{:08b}", r1.bits());
                Err(Error::BadR1(r1))
            };
        } else {
            // TODO: Timeout
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

/// Returns if the SD card is idle
pub async fn command_a41<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
    host_supports_hcs: bool,
) -> Result<bool, Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(
        spi_bus,
        &format_command(41, {
            let mut argument = CommandA41Argument::default();
            argument.set(CommandA41Argument::HCS, host_supports_hcs);
            argument.bits()
        }),
    )
    .await
    .map_err(Error::SpiBus)?;
    // We're not allowed to talk to other SPI devices between sending the command and receiving a response
    let result = {
        if r1 == R1::IN_IDLE_STATE || r1.is_empty() {
            Ok(r1.contains(R1::IN_IDLE_STATE))
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_9<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<u128, Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(spi_bus, &format_command(9, 0))
        .await
        .map_err(Error::SpiBus)?;
    let result = if r1.is_empty() {
        cs.set_high().map_err(Error::CsPin)?;
        spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        // We are allowed to talk to other SPI devices at this point
        cs.set_low().map_err(Error::CsPin)?;
        loop {
            let mut buffer = [0xFF; 1];
            spi_bus
                .transfer_in_place(&mut buffer)
                .await
                .map_err(Error::SpiBus)?;
            let byte = buffer[0];
            if byte != 0xFF {
                break;
            } else {
                // TODO: Timeout
            }
        }
        let mut buffer = [0xFF; 18];
        spi_bus
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SpiBus)?;
        let (csd, crc) = buffer.split_at(16);
        let csd = <&[u8; 16]>::try_from(csd).unwrap();
        let crc = u16::from_be_bytes(*<&[u8; 2]>::try_from(crc).unwrap());
        if crc == Crc::<u16>::new(&CRC_16_XMODEM).checksum(csd) {
            Ok(u128::from_be_bytes(*csd))
        } else {
            Err(Error::InvalidChecksum)
        }
    } else {
        return Err(Error::BadR1(r1));
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_59<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
    crc_on: bool,
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(
        spi_bus,
        &format_command(59, {
            let mut argument = Command59Argument::default();
            argument.set(Command59Argument::CRC_ON, crc_on);
            argument.bits()
        }),
    )
    .await
    .map_err(Error::SpiBus)?;
    // We're not allowed to talk to other SPI devices between sending the command and receiving a response
    let result = {
        if r1 == R1::IN_IDLE_STATE || r1.is_empty() {
            Ok(())
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_10<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<u128, Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(spi_bus, &format_command(10, 0))
        .await
        .map_err(Error::SpiBus)?;
    let result = if r1.is_empty() {
        cs.set_high().map_err(Error::CsPin)?;
        spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        // We are allowed to talk to other SPI devices at this point
        cs.set_low().map_err(Error::CsPin)?;
        loop {
            let mut buffer = [0xFF; 1];
            spi_bus
                .transfer_in_place(&mut buffer)
                .await
                .map_err(Error::SpiBus)?;
            let byte = buffer[0];
            if byte != 0xFF {
                break;
            } else {
                // TODO: Timeout
            }
        }
        let mut buffer = [0xFF; 18];
        spi_bus
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SpiBus)?;
        let (cid, crc) = buffer.split_at(16);
        let csd = <&[u8; 16]>::try_from(cid).unwrap();
        let crc = u16::from_be_bytes(*<&[u8; 2]>::try_from(crc).unwrap());
        if crc == Crc::<u16>::new(&CRC_16_XMODEM).checksum(csd) {
            Ok(u128::from_be_bytes(*csd))
        } else {
            Err(Error::InvalidChecksum)
        }
    } else {
        return Err(Error::BadR1(r1));
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_13<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<R2Byte1, Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(spi_bus, &format_command(13, 0))
        .await
        .map_err(Error::SpiBus)?;
    let result = {
        if r1 == R1::IN_IDLE_STATE || r1.is_empty() {
            let mut buffer = [0xFF; 1];
            spi_bus
                .transfer_in_place(&mut buffer)
                .await
                .map_err(Error::SpiBus)?;
            Ok(R2Byte1::from_bits_retain(buffer[0]))
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_17<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
    address: u32,
    buffer: &mut [u8; 512],
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(spi_bus, &format_command(17, address))
        .await
        .map_err(Error::SpiBus)?;
    let result = {
        if r1.is_empty() {
            // TOOD: Can we talk to other SPI devices during this time?
            // Wait for start block token
            let data = loop {
                let mut buffer = [0xFF; 1];
                spi_bus
                    .transfer_in_place(&mut buffer)
                    .await
                    .map_err(Error::SpiBus)?;
                if buffer[0] != 0xFF {
                    break buffer[0];
                } else {
                    // TODO: Timeout
                }
            };
            if data == START_BLOCK_TOKEN {
                buffer.fill(0xFF);
                spi_bus
                    .transfer_in_place(buffer.as_mut_slice())
                    .await
                    .map_err(Error::SpiBus)?;
                let mut crc = [0xFF; 2];
                spi_bus
                    .transfer_in_place(&mut crc)
                    .await
                    .map_err(Error::SpiBus)?;
                if u16::from_be_bytes(crc) == Crc::<u16>::new(&CRC_16_XMODEM).checksum(buffer) {
                    Ok(())
                } else {
                    Err(Error::InvalidChecksum)
                }
            } else {
                Err(Error::BadData(data))
            }
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub async fn command_12<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    spi_bus
        .write(&format_command(12, 0))
        .await
        .map_err(Error::SpiBus)?;
    // The embedded-sdmmc crate skips a byte, so we will too
    // I observed that on a SanDisk Extreme 128GB MicroSD card, CMD12 fails without this because R1 is 0b011111111.
    spi_bus
        .transfer_in_place(&mut [0xFF])
        .await
        .map_err(Error::SpiBus)?;
    let r1 = loop {
        let mut buffer = [0xFF; 1];
        spi_bus
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SpiBus)?;
        let r1 = R1::from_bits_retain(buffer[0]);
        if !r1.contains(R1::BIT_7) {
            break r1;
        } else {
            Timer::after_micros(10).await;
            // TODO: Timeout
        }
    };
    let result = {
        if r1.is_empty() {
            Ok(())
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

/// For now, doesn't actually give you the read data
/// The bigger the buffer you can provide, the more performance we can get out of this
/// Returns the amount successfully read
pub async fn demo_command_18<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
    address: u32,
    count: u32,
    buffer: &mut [u8],
) -> Result<u32, Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(spi_bus, &format_command(18, address))
        .await
        .map_err(Error::SpiBus)?;
    let result = {
        if r1.is_empty() {
            buffer.fill(0xFF);
            let before = Instant::now();
            spi_bus
                .transfer_in_place(buffer)
                .await
                .map_err(Error::SpiBus)?;
            let after = Instant::now();
            defmt::info!(
                "Transferred {} bytes in {} us",
                buffer.len(),
                (after - before).as_micros()
            );
            // let mut success_count = 0;
            // for i in 0..count {

            //     // #[cfg(feature = "defmt")]
            //     // defmt::info!("reading block {}", i);
            //     // TOOD: Can we talk to other SPI devices during this time?
            //     // Wait for start block token
            //     let mut bytes_until_response = 0;
            //     let data = loop {
            //         let mut buffer = [0xFF; 1];
            //         spi_bus
            //             .transfer_in_place(&mut buffer)
            //             .await
            //             .map_err(Error::SpiBus)?;
            //         if buffer[0] != 0xFF {
            //             break buffer[0];
            //         } else {
            //             bytes_until_response += 1;
            //         }
            //     };
            //     defmt::info!("bytes until data response: {}", bytes_until_response);
            //     if data == START_BLOCK_TOKEN {
            //         let mut buffer = [0xFF; 512];
            //         // buffer.fill(0xFF);
            //         spi_bus
            //             .transfer_in_place(buffer.as_mut_slice())
            //             .await
            //             .map_err(Error::SpiBus)?;
            //         let mut crc = [0xFF; 2];
            //         spi_bus
            //             .transfer_in_place(&mut crc)
            //             .await
            //             .map_err(Error::SpiBus)?;
            //         if u16::from_be_bytes(crc)
            //             == Crc::<u16>::new(&CRC_16_XMODEM).checksum(&mut buffer)
            //         {
            //             success_count += 1;
            //             // Ok(())
            //         } else {
            //             // Err(Error::InvalidChecksum)
            //         }
            //     } else {
            //         #[cfg(feature = "defmt")]
            //         defmt::error!("unexpected byte: 0b{:08b}", data);
            //         // Err(Error::BadData(data))
            //     }
            // }
            command_12(spi_bus, cs).await?;
            Ok(0)
        } else {
            Err(Error::BadR1(r1))
        }
    };
    cs.set_high().map_err(Error::CsPin)?;
    spi_bus.write(&[0xFF]).await.map_err(Error::SpiBus)?;
    result
}

pub struct SpiSdCard<Spi, Cs, Delayer> {
    spi: Spi,
    cs: Cs,
    delayer: Delayer,
}

struct CommandResponse<'a> {
    /// R1 + extra bytes
    response: Option<(R1, &'a mut [u8])>,
    /// If `true`, it means that we received at least one byte that was not `0xFF`.
    /// This can be used to guess if an SD card is unplugged;
    data_received: bool,
}

/// The buffer must be at least `size_of::<Command>() + expected_bytes_until_response + 1`
async fn card_command_r1<'a, S: SpiBus>(
    spi: &mut S,
    command: &[u8; 6],
    buffer: &'a mut [u8],
    expected_bytes_until_response: usize,
    max_bytes_until_response: usize,
) -> Result<CommandResponse<'a>, S::Error> {
    let mut is_first_transfer = true;
    let mut bytes_read = 0;
    let mut data_received = false;
    let response = 'read_response: loop {
        if bytes_read >= max_bytes_until_response {
            break None;
        }
        let transfer_len = if is_first_transfer {
            buffer[..size_of::<Command>()].copy_from_slice(command);
            buffer[size_of::<Command>()
                ..size_of::<Command>() + expected_bytes_until_response + size_of::<R1>()]
                .fill(0xFF);
            size_of::<Command>() + expected_bytes_until_response + size_of::<R1>()
        } else {
            buffer.fill(0xFF);
            expected_bytes_until_response + size_of::<R1>()
        };
        spi.transfer_in_place(&mut buffer[..transfer_len]).await?;
        let mut i = if is_first_transfer {
            size_of::<Command>()
        } else {
            0
        };
        loop {
            if i == transfer_len {
                break;
            }
            let byte = buffer[i];
            if byte != 0xFF {
                data_received = true;
                let r1 = R1::from_bits_retain(byte);
                if !r1.contains(R1::BIT_7) {
                    break 'read_response Some((r1, &mut buffer[i + 1..transfer_len]));
                }
            }
            bytes_read += 1;
            i += 1;
        }
        is_first_transfer = false;
    };
    Ok(CommandResponse {
        response,
        data_received,
    })
}

struct ReadOperation<'a> {
    buffer: &'a mut [u8],
    expected_bytes_until_data: usize,
    timeout: Duration,
}

struct WriteOperation<'a> {
    buffer: &'a [u8],
    expected_bytes_until_data: usize,
    timeout: Duration,
}

enum CardCommandOperation<'a> {
    Read(ReadOperation<'a>),
    Write(WriteOperation<'a>),
}

const fn command_buffer_len(
    expected_bytes_until_response: usize,
    response: &mut [u8],
    operation: Option<CardCommandOperation<'_>>,
) -> usize {
    let mut len = 0;
    len += size_of::<Command>();
    len += expected_bytes_until_response;
    len += response.len();
    match operation {
        None => {}
        Some(CardCommandOperation::Read(ReadOperation {
            buffer,
            expected_bytes_until_data,
            timeout,
        })) => {
            len += expected_bytes_until_data;
            len += buffer.len();
        }
        Some(CardCommandOperation::Write(WriteOperation {
            buffer,
            expected_bytes_until_data,
            timeout,
        })) => {
            len += expected_bytes_until_data;
            len += buffer.len();
        }
    };
    len
}
enum CardCommand3Error<SpiError> {
    Spi(SpiError),
    /// `true` if any data that was not `0xFF` was received
    ReceiveResponseTimeout(bool),
    /// Expected a start block token, but got something else
    ExpectedStartBlockToken,
    InvalidCrc,
}

/// Supports all commands except for multi block read and write.
async fn card_command_3<S: SpiBus, C: Clock>(
    spi: &mut S,
    clock: &C,
    buffer: &mut [u8],
    command: &[u8; 6],
    expected_bytes_until_response: usize,
    response: &mut [u8],
    response_timeout: Duration,
    mut operation: Option<CardCommandOperation<'_>>,
) -> Result<(), CardCommand3Error<S::Error>> {
    enum Phase<C: Clock> {
        /// Bytes sent
        SendCommand(usize),
        ReceiveResponseStart((C::Instant, bool)),
        /// Number of bytes of the response received so far
        ReceiveResponse(NonZero<usize>),
        ReceiveStartBlockToken,
        /// Number of bytes of the data received so far
        ReceiveData(usize),
        /// The byte of the partial CRC received, if any
        ReceiveCrc(Option<u8>),
        WriteData(usize),
    }
    let mut phase = Phase::<C>::SendCommand(0);
    let mut number_of_bytes_to_process = 0;
    'spi: loop {
        while number_of_bytes_to_process > 0 {
            let bytes_to_process = &mut buffer[..number_of_bytes_to_process];
            match phase {
                Phase::SendCommand(bytes_sent) => {
                    let bytes_to_send = size_of::<Command>() - bytes_sent;
                    let command_bytes_sent = min(bytes_to_send, bytes_to_process.len());
                    number_of_bytes_to_process -= command_bytes_sent;
                    if bytes_to_send == command_bytes_sent {
                        phase = Phase::ReceiveResponseStart((clock.now(), false));
                    }
                }
                Phase::ReceiveResponseStart((start_time, data_received)) => {
                    // Scan for R1
                    let mut i = 0;
                    let mut data_received = data_received;
                    let r1_index = loop {
                        if let Some(&byte) = bytes_to_process.get(i) {
                            if byte != 0xFF {
                                data_received = true;
                                if !R1::from_bits_retain(byte).contains(R1::BIT_7) {
                                    break Some(i);
                                }
                            }
                        } else {
                            break None;
                        }
                        i += 1;
                    };
                    if let Some(r1_index) = r1_index {
                        response[0] = bytes_to_process[r1_index];
                        number_of_bytes_to_process -= r1_index + 1;
                        phase = Phase::ReceiveResponse(NonZero::new(1).unwrap());
                    } else if (clock.now() - start_time) >= response_timeout {
                        return Err(CardCommand3Error::ReceiveResponseTimeout(data_received));
                    } else {
                        number_of_bytes_to_process = 0;
                        phase = Phase::ReceiveResponseStart((start_time, data_received));
                    }
                }
                Phase::ReceiveResponse(bytes_received) => {
                    let bytes_to_receive = response.len() - bytes_received.get();
                    let copy_len = min(bytes_to_receive, bytes_to_process.len());
                    response[bytes_received.get()..bytes_received.get() + copy_len]
                        .copy_from_slice(&bytes_to_process[..copy_len]);
                    number_of_bytes_to_process -= copy_len;
                    if let Some(new_bytes_received) = NonZero::new(bytes_received.get() - copy_len)
                    {
                        phase = Phase::ReceiveResponse(new_bytes_received);
                    } else {
                        match &operation {
                            None => break 'spi,
                            Some(CardCommandOperation::Read(_)) => {
                                phase = Phase::ReceiveStartBlockToken;
                            }
                            Some(CardCommandOperation::Write(_)) => {
                                phase = Phase::WriteData(0);
                            }
                        }
                    }
                }
                Phase::ReceiveStartBlockToken => {
                    // Scan for token
                    if let Some((index, &byte)) = bytes_to_process
                        .iter()
                        .enumerate()
                        .find(|(_, byte)| **byte != 0xFF)
                    {
                        if byte == START_BLOCK_TOKEN {
                            number_of_bytes_to_process -= index + 1;
                            phase = Phase::ReceiveData(0);
                        } else {
                            return Err(CardCommand3Error::ExpectedStartBlockToken);
                        }
                    }
                }
                Phase::ReceiveData(bytes_received) => {
                    let operation =
                        if let Some(CardCommandOperation::Read(operation)) = &mut operation {
                            operation
                        } else {
                            unreachable!()
                        };
                    let bytes_left = operation.buffer.len() - bytes_received;
                    let copy_len = min(bytes_left, bytes_to_process.len());
                    operation.buffer[bytes_received..bytes_received + copy_len]
                        .copy_from_slice(&bytes_to_process[..copy_len]);
                    number_of_bytes_to_process -= copy_len;
                    let new_bytes_received = bytes_received + copy_len;
                    if new_bytes_received == operation.buffer.len() {
                        phase = Phase::ReceiveCrc(None);
                    } else {
                        phase = Phase::ReceiveData(new_bytes_received);
                    }
                }
                Phase::ReceiveCrc(byte_0) => {
                    if let Some(byte_0) = byte_0 {
                        let byte_1 = bytes_to_process[0];
                        let crc = u16::from_le_bytes([byte_0, byte_1]);
                        let operation =
                            if let Some(CardCommandOperation::Read(operation)) = &mut operation {
                                operation
                            } else {
                                unreachable!()
                            };
                        if crc == Crc::<u16>::new(&CRC_16_XMODEM).checksum(&operation.buffer) {
                            break 'spi;
                        } else {
                            return Err(CardCommand3Error::InvalidCrc);
                        }
                    } else {
                        let byte_0 = bytes_to_process[0];
                        number_of_bytes_to_process -= 1;
                        phase = Phase::ReceiveCrc(Some(byte_0));
                    };
                }
                Phase::WriteData(_) => todo!(),
            }
        }
        // Set up buffer
        let bytes_to_transfer = match phase {
            Phase::SendCommand(bytes_sent) => {
                let copy_len = min(size_of::<Command>() - bytes_sent, buffer.len());
                buffer[..copy_len].copy_from_slice(&command[bytes_sent..bytes_sent + copy_len]);
                let bytes_to_transfer = (copy_len
                    + expected_bytes_until_response
                    + response.len()
                    + match &operation {
                        None => 0,
                        Some(CardCommandOperation::Read(ReadOperation {
                            buffer,
                            expected_bytes_until_data,
                            timeout,
                        })) => expected_bytes_until_data + buffer.len() + size_of::<u16>(),
                        Some(CardCommandOperation::Write(WriteOperation {
                            buffer,
                            expected_bytes_until_data,
                            timeout,
                        })) => {
                            todo!()
                        }
                    })
                .min(buffer.len());
                buffer[copy_len..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveResponseStart(_) => {
                let bytes_to_transfer = (expected_bytes_until_response
                    + response.len()
                    + match &operation {
                        None => 0,
                        Some(CardCommandOperation::Read(ReadOperation {
                            buffer,
                            expected_bytes_until_data,
                            timeout,
                        })) => expected_bytes_until_data + buffer.len() + size_of::<u16>(),
                        Some(CardCommandOperation::Write(_)) => {
                            todo!()
                        }
                    })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveResponse(bytes_received) => {
                let bytes_to_transfer = (response.len() - bytes_received.get()
                    + match &operation {
                        None => 0,
                        Some(CardCommandOperation::Read(ReadOperation {
                            buffer,
                            expected_bytes_until_data,
                            timeout,
                        })) => expected_bytes_until_data + buffer.len() + size_of::<u16>(),
                        Some(CardCommandOperation::Write(_)) => {
                            todo!()
                        }
                    })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveStartBlockToken => {
                let bytes_to_transfer = (if let Some(CardCommandOperation::Read(ReadOperation {
                    buffer,
                    expected_bytes_until_data,
                    timeout,
                })) = &operation
                {
                    expected_bytes_until_data + buffer.len() + size_of::<u16>()
                } else {
                    unreachable!()
                })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveData(bytes_received) => {
                let bytes_to_transfer = (if let Some(CardCommandOperation::Read(ReadOperation {
                    buffer,
                    expected_bytes_until_data,
                    timeout,
                })) = &operation
                {
                    buffer.len() - bytes_received + size_of::<u16>()
                } else {
                    unreachable!()
                })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveCrc(byte_0) => {
                let bytes_to_transfer =
                    (size_of::<u16>() - if byte_0.is_some() { 1 } else { 0 }).min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::WriteData(_) => todo!(),
        };
        spi.transfer_in_place(&mut buffer[..bytes_to_transfer])
            .await
            .map_err(CardCommand3Error::Spi)?;
        number_of_bytes_to_process = bytes_to_transfer;
    }
    Ok(())
}

impl<Spi: SharedSpiBus<u8>, Cs: OutputPin, Delayer: DelayNs> SpiSdCard<Spi, Cs, Delayer> {
    pub fn new(spi: Spi, cs: Cs, delayer: Delayer) -> Self {
        Self { spi, cs, delayer }
    }

    pub async fn init_card(
        &mut self,
    ) -> Result<(), Error<<Spi::Bus as ErrorType>::Error, Cs::Error>> {
        // Wait at least 1ms
        self.delayer.delay_ms(1).await;

        let mut spi = self.spi.lock().await;

        // // Send 0xFF for at least 74 clock cycles according to the spec
        // // So 9 bytes
        spi.write(&[0xFF; 9]).await.unwrap();

        // Do CMD0
        self.cs.set_low().map_err(Error::CsPin)?;
        {
            let mut buffer = [0xFF; size_of::<Command>() + BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut got_response = false;
            let mut attempt_number = 0;
            let mut max_attempts = 50;
            loop {
                if attempt_number == max_attempts {
                    break Err(if got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    });
                }
                let mut is_first_transfer = true;
                let mut bytes_read = 0;
                let r1 = 'read_response: loop {
                    if bytes_read >= CMD_0_MAX_BYTES_UNTIL_RESPONSE {
                        break None;
                    }
                    if is_first_transfer {
                        buffer[..size_of::<Command>()].copy_from_slice(&format_command(0, 0));
                    } else {
                        buffer.fill(0xFF);
                    }
                    defmt::info!("Buffer: {:02X}", buffer);
                    spi.transfer_in_place(&mut buffer)
                        .await
                        .map_err(Error::SpiBus)?;
                    let read_bytes = if is_first_transfer {
                        &buffer[6..]
                    } else {
                        &buffer
                    };
                    let mut i = 0;
                    loop {
                        if let Some(&byte) = read_bytes.get(i) {
                            let r1 = R1::from_bits_retain(byte);
                            if !r1.contains(R1::BIT_7) {
                                break 'read_response Some(r1);
                            }
                            bytes_read += 1;
                            i += 1;
                        } else {
                            break;
                        }
                    }
                    todo!();
                    is_first_transfer = false;
                };
                if let Some(r1) = r1 {
                    got_response = true;
                    if r1 == R1::IN_IDLE_STATE {
                        break Ok(());
                    } else {
                        warn!("Got response: {:x}, trying again..", r1.bits());
                    }
                }
                buffer[6..].fill(0xFF);
                self.delayer.delay_us(10).await;
            }
        }?;

        Ok(())
    }
}
