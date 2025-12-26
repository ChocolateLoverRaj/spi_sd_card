#![no_std]
#![allow(async_fn_in_trait)]
#[cfg(feature = "esp32c3")]
mod dma;

use core::{cmp::min, num::NonZero, ops::DerefMut};

use defmt::warn;
#[cfg(feature = "esp32c3")]
pub use dma::*;
mod shared_spi_bus;
use embassy_embedded_hal::SetConfig;
pub use shared_spi_bus::*;
mod disk;

mod structs;
mod util;
pub use disk::*;
pub use util::*;

use crc::{CRC_7_MMC, CRC_16_XMODEM, Crc};
use embassy_time::{Duration, Instant, Timer};
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
/// If the bytes vary by command, we can use a separate value for different commands.
const COMMAND_BYTES_UNTIL_RESPONSE: usize = 2;
const COMMAND_TIMEOUT: Duration = Duration::from_millis(100);
/// This is just a guess
const BYTES_UNTIL_CSD: usize = 2;
const CSD_TIMEOUT: Duration = Duration::from_millis(100);
/// In my experience this is up to 2
/// Note that if we make this super big it will reduce performance
/// With `670` we are basically guaranteeing that the transfer speed will be <0.5x of the SPI transfer speed
const BYTES_UNTIL_READ_DATA: usize = 670;
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// The buffer can have any data
pub async fn card_command_2<S: SpiBus>(
    spi_bus: &mut S,
    command: &Command,
    buffer: &mut [u8; size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + 1],
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
    let mut buffer = [0xFF; size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + 5];
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

pub struct SpiSdCard<Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    spi: Spi,
    cs: Cs,
    delayer: Delayer,
    _400_khz_config: <Spi::Bus as SetConfig>::Config,
    _25_mhz_config: <Spi::Bus as SetConfig>::Config,
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

#[derive(Debug)]
enum CardCommand3Error<SpiError> {
    Spi(SpiError),
    /// `true` if any data that was not `0xFF` was received
    ReceiveResponseTimeout(bool),
    /// Expected a start block token, but got something else
    ExpectedStartBlockToken,
    InvalidCrc,
}

/// Supports all commands except for multi block read and write.
async fn card_command_3<S: SpiBus>(
    spi: &mut S,
    buffer: &mut [u8],
    command: &[u8; 6],
    expected_bytes_until_response: usize,
    response: &mut [u8],
    response_timeout: Duration,
    mut operation: Option<CardCommandOperation<'_>>,
) -> Result<(), CardCommand3Error<S::Error>> {
    enum Phase {
        /// Bytes sent
        SendCommand(usize),
        ReceiveResponseStart((Instant, bool)),
        /// Number of bytes of the response received so far
        ReceiveResponse(usize),
        ReceiveStartBlockToken,
        /// Number of bytes of the data received so far
        ReceiveData(usize),
        /// The byte of the partial CRC received, if any
        ReceiveCrc(Option<u8>),
        WriteData(usize),
    }
    let mut phase = Phase::SendCommand(0);
    let mut buffer_valid_bytes = 0;
    'spi: loop {
        defmt::trace!("number of bytes to process: {}", buffer_valid_bytes);
        let mut bytes_processed = 0;
        while buffer_valid_bytes > bytes_processed {
            let bytes_to_process = &mut buffer[bytes_processed..buffer_valid_bytes];
            match phase {
                Phase::SendCommand(bytes_sent) => {
                    defmt::trace!("send command phase: {}", bytes_sent);
                    let bytes_to_send = size_of::<Command>() - bytes_sent;
                    let command_bytes_sent = min(bytes_to_send, bytes_to_process.len());
                    bytes_processed += command_bytes_sent;
                    let new_bytes_sent = bytes_sent + command_bytes_sent;
                    if new_bytes_sent == size_of::<Command>() {
                        phase = Phase::ReceiveResponseStart((Instant::now(), false));
                    } else {
                        phase = Phase::SendCommand(new_bytes_sent)
                    }
                    defmt::trace!(
                        "send command phase: {}. new bytes sent: {}",
                        bytes_sent,
                        new_bytes_sent
                    );
                }
                Phase::ReceiveResponseStart((start_time, data_received)) => {
                    // Scan for R1
                    let mut i = 0;
                    let mut data_received = data_received;
                    let r1_index = loop {
                        if let Some(&byte) = bytes_to_process.get(i) {
                            defmt::trace!("Byte: 0x{:02X}", byte);
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
                    defmt::trace!(
                        "receive response start phase: {}, {}. r1 index: {}",
                        start_time,
                        data_received,
                        r1_index
                    );
                    if let Some(r1_index) = r1_index {
                        bytes_processed += r1_index;
                        phase = Phase::ReceiveResponse(0);
                    } else if start_time.elapsed() >= response_timeout {
                        return Err(CardCommand3Error::ReceiveResponseTimeout(data_received));
                    } else {
                        bytes_processed = buffer_valid_bytes;
                        phase = Phase::ReceiveResponseStart((start_time, data_received));
                    }
                }
                Phase::ReceiveResponse(bytes_received) => {
                    let bytes_to_receive = response.len() - bytes_received;
                    let copy_len = min(bytes_to_receive, bytes_to_process.len());
                    defmt::trace!(
                        "receive response phase: {}. processing bytes: {:02X}. copy len: {}",
                        bytes_received,
                        bytes_to_process,
                        copy_len
                    );
                    response[bytes_received..bytes_received + copy_len]
                        .copy_from_slice(&bytes_to_process[..copy_len]);
                    bytes_processed += copy_len;
                    let new_bytes_received = bytes_received + copy_len;
                    if new_bytes_received == response.len() {
                        match &operation {
                            None => break 'spi,
                            Some(CardCommandOperation::Read(_)) => {
                                phase = Phase::ReceiveStartBlockToken;
                            }
                            Some(CardCommandOperation::Write(_)) => {
                                phase = Phase::WriteData(0);
                            }
                        }
                    } else {
                        phase = Phase::ReceiveResponse(new_bytes_received);
                    }
                }
                Phase::ReceiveStartBlockToken => {
                    defmt::trace!("receive start block token phase");
                    Timer::after_millis(10).await;
                    for &mut byte in bytes_to_process {
                        bytes_processed += 1;
                        if byte != 0xFF {
                            if byte == START_BLOCK_TOKEN {
                                phase = Phase::ReceiveData(0);
                                break;
                            } else {
                                defmt::error!(
                                    "expected start block token, but got 0x{:02X} instead",
                                    byte
                                );
                                return Err(CardCommand3Error::ExpectedStartBlockToken);
                            }
                        }
                    }
                }
                Phase::ReceiveData(bytes_received) => {
                    defmt::trace!("receive data phase: {}", bytes_received);
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
                    bytes_processed += copy_len;
                    let new_bytes_received = bytes_received + copy_len;
                    if new_bytes_received == operation.buffer.len() {
                        phase = Phase::ReceiveCrc(None);
                    } else {
                        phase = Phase::ReceiveData(new_bytes_received);
                        defmt::trace!(
                            "did not receive all data in a single transfer (missing {} bytes)",
                            operation.buffer.len() - new_bytes_received
                        );
                    }
                }
                Phase::ReceiveCrc(byte_0) => {
                    defmt::trace!("receive CRC phase: {}", byte_0);
                    if let Some(byte_0) = byte_0 {
                        let byte_1 = bytes_to_process[0];
                        let crc = u16::from_be_bytes([byte_0, byte_1]);
                        let operation =
                            if let Some(CardCommandOperation::Read(operation)) = &mut operation {
                                operation
                            } else {
                                unreachable!()
                            };
                        let expected_crc =
                            Crc::<u16>::new(&CRC_16_XMODEM).checksum(&operation.buffer);

                        if crc == expected_crc {
                            break 'spi;
                        } else {
                            defmt::error!(
                                "Data: {:02X}. Received CRC: 0x{:04X}. Expected CRC: 0x{:04X}.",
                                operation.buffer,
                                crc,
                                expected_crc
                            );
                            return Err(CardCommand3Error::InvalidCrc);
                        }
                    } else {
                        let byte_0 = bytes_to_process[0];
                        bytes_processed += 1;
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
                let bytes_to_transfer = (response.len() - bytes_received
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
        assert_ne!(bytes_to_transfer, 0);
        defmt::trace!("transferring...");
        let before = Instant::now();
        spi.transfer_in_place(&mut buffer[..bytes_to_transfer])
            .await
            .map_err(CardCommand3Error::Spi)?;
        defmt::trace!(
            "Transferred {} bytes in {} us",
            bytes_to_transfer,
            before.elapsed().as_micros()
        );
        defmt::trace!("Bytes: {:02X}", &mut buffer[..bytes_to_transfer]);
        buffer_valid_bytes = bytes_to_transfer;
    }
    Ok(())
}

impl<Spi: SharedSpiBus<u8>, Cs: OutputPin, Delayer: DelayNs> SpiSdCard<Spi, Cs, Delayer>
where
    Spi::Bus: SetConfig,
{
    /// This assumes that the voltage you are providing to the SD card is 3.3V!
    /// If for some reason you are not providing 3.3V, create an issue
    /// so we can better check if the SD card is compatible with the voltage you are providing.
    ///
    /// Before the SD card's initialization is complete, a 400 kHz SPI speed is used. After that, a 25 MHz SPI speed can be used.
    /// Provide the correct SPI speeds.
    pub fn new(
        spi: Spi,
        cs: Cs,
        delayer: Delayer,
        _400_khz_config: <Spi::Bus as SetConfig>::Config,
        _25_mhz_config: <Spi::Bus as SetConfig>::Config,
    ) -> Self {
        Self {
            spi,
            cs,
            delayer,
            _400_khz_config,
            _25_mhz_config,
        }
    }

    pub async fn init_card(
        &mut self,
    ) -> Result<SdCardDisk<'_, Spi, Cs, Delayer>, Error<<Spi::Bus as ErrorType>::Error, Cs::Error>>
    {
        // Wait at least 1ms
        self.delayer.delay_ms(1).await;

        let mut spi = self.spi.lock().await;
        spi.set_config(&self._400_khz_config);

        // Send 0xFF for at least 74 clock cycles according to the spec
        // So 9 bytes
        spi.write(&[0xFF; 9]).await.unwrap();

        self.cs.set_low().map_err(Error::CsPin)?;

        // This might help if the card was previously in the middle of something
        // TODO: Is this needed?
        spi.write(&[0xFF; 1000]).await.unwrap();

        let mut got_response = false;
        // TODO: Gracefully handle failures (remember to set CS to high and write a 0xFF byte);
        // Do CMD0
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); 1];
            let mut attempt_number = 0;
            let max_attempts = 50;
            loop {
                if attempt_number == max_attempts {
                    break Err(if got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    });
                }
                let result = card_command_3(
                    spi.deref_mut(),
                    &mut buffer,
                    &format_command(0, 0),
                    COMMAND_BYTES_UNTIL_RESPONSE,
                    &mut response,
                    COMMAND_TIMEOUT,
                    None,
                )
                .await;
                match result {
                    Ok(_) => {
                        got_response = true;
                    }
                    Err(CardCommand3Error::ReceiveResponseTimeout(data_received)) => {
                        got_response |= data_received;
                    }
                    _ => {}
                }
                if result.is_ok() {
                    let r1 = R1::from_bits_retain(response[0]);
                    if r1 == R1::IN_IDLE_STATE {
                        break Ok(());
                    } else {
                        warn!("Got response: {:x}, trying again..", r1.bits());
                    }
                }
                // TODO: Release SPI lock?
                self.delayer.delay_us(10).await;
                attempt_number += 1;
            }
        }?;

        // Enable CRC
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); 1];
            card_command_3(
                spi.deref_mut(),
                &mut buffer,
                &format_command(59, {
                    let mut argument = Command59Argument::default();
                    argument.set(Command59Argument::CRC_ON, true);
                    argument.bits()
                }),
                COMMAND_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                    if got_response | command_got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    }
                }
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if r1 != R1::IN_IDLE_STATE {
                return Err(Error::InitFailed);
            }
        }

        // Do CMD8
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + size_of::<R7>()];
            let mut response = [Default::default(); size_of::<R7>()];
            // The check pattern can be anything we want
            let check_pattern = 0xE2;
            card_command_3(
                spi.deref_mut(),
                &mut buffer,
                &format_command(8, {
                    let mut argument = Command8Argument(Default::default());
                    argument.set_pcie1_2v_support(false);
                    argument.set_pcie_availability(false);
                    argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
                    argument.set_check_pattern(check_pattern);
                    argument.0
                }),
                COMMAND_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                    if got_response | command_got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    }
                }
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if r1 == R1::ILLEGAL_COMMAND {
                todo!("Handle version 1")
            } else if r1 != R1::IN_IDLE_STATE {
                return Err(Error::InitFailed);
            }
            let byte_3 = R7Byte3(response[3]);
            if !byte_3
                .get_voltage_accepted()
                .contains(VoltageAccpted::_2_7V_3_6V)
            {
                return Err(Error::VoltageNotSupported);
            }
            if response[4] != check_pattern {
                return Err(Error::InitFailed);
            }
        }

        // Get OCR to make sure voltage is compatible
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + size_of::<R3>()];
            let mut response = [Default::default(); size_of::<R3>()];
            card_command_3(
                spi.deref_mut(),
                &mut buffer,
                &format_command(58, 0),
                COMMAND_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                    if got_response | command_got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    }
                }
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if r1 != R1::IN_IDLE_STATE {
                return Err(Error::InitFailed);
            }
            let ocr = Ocr::from_bits_retain(u32::from_be_bytes(response[1..5].try_into().unwrap()));
            if !ocr.supports_3_3v() {
                return Err(Error::VoltageNotSupported);
            }
        }

        // Initialize card
        {
            let mut attempt_number = 0;
            let max_attempts = 100;
            let mut buffer = [Default::default();
                size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); size_of::<R1>()];
            loop {
                if attempt_number == max_attempts {
                    return Err(Error::InitFailed);
                }
                // CMD55 - next command is an "A" command
                card_command_3(
                    spi.deref_mut(),
                    &mut buffer,
                    &format_command(55, 0),
                    COMMAND_BYTES_UNTIL_RESPONSE,
                    &mut response,
                    COMMAND_TIMEOUT,
                    None,
                )
                .await
                .map_err(|e| match e {
                    CardCommand3Error::Spi(e) => Error::SpiBus(e),
                    CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                        defmt::info!("CMD55 ReceiveResponseTimeout");
                        if got_response | command_got_response {
                            Error::InitFailed
                        } else {
                            Error::NoResponse
                        }
                    }
                    _ => unreachable!(),
                })?;
                let r1 = R1::from_bits_retain(response[0]);
                defmt::info!("cmd 55 r1: 0b{:08b}", r1.bits());
                if !(r1 == R1::IN_IDLE_STATE || r1 == R1::empty()) {
                    return Err(Error::InitFailed);
                }

                // ACMD41
                card_command_3(
                    spi.deref_mut(),
                    &mut buffer,
                    &format_command(41, CommandA41Argument::HCS.bits()),
                    COMMAND_BYTES_UNTIL_RESPONSE,
                    &mut response,
                    COMMAND_TIMEOUT,
                    None,
                )
                .await
                .map_err(|e| match e {
                    CardCommand3Error::Spi(e) => Error::SpiBus(e),
                    CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                        if got_response | command_got_response {
                            Error::InitFailed
                        } else {
                            Error::NoResponse
                        }
                    }
                    _ => unreachable!(),
                })?;
                let r1 = R1::from_bits_retain(response[0]);
                defmt::info!("r1: 0b{:08b}", r1.bits());
                if r1 == R1::empty() {
                    break;
                } else if r1 != R1::IN_IDLE_STATE {
                    return Err(Error::InitFailed);
                }
                attempt_number += 1;
            }
        }

        defmt::info!("Reading OCR again");

        // Get OCR
        let ocr = {
            let mut buffer = [Default::default();
                size_of::<Command>() + COMMAND_BYTES_UNTIL_RESPONSE + size_of::<R3>()];
            let mut response = [Default::default(); size_of::<R3>()];
            card_command_3(
                spi.deref_mut(),
                &mut buffer,
                &format_command(58, 0),
                COMMAND_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                    if got_response | command_got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    }
                }
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::InitFailed);
            }
            Ocr::from_bits_retain(u32::from_be_bytes(response[1..5].try_into().unwrap()))
        };

        spi.flush().await.map_err(Error::SpiBus)?;
        self.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        defmt::info!("is SDHC or SDXC?: {}", ocr.supports_sdhc_or_sdxc());

        Ok(SdCardDisk { sd_card: self })
    }
}

pub struct SdCardDisk<'a, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    sd_card: &'a mut SpiSdCard<Spi, Cs, Delayer>,
}

impl<Spi, Cs: OutputPin, Delayer: DelayNs> Disk for SdCardDisk<'_, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    type Address = u64;
    type Error = Error<<Spi::Bus as ErrorType>::Error, Cs::Error>;

    async fn read(&mut self, start: Self::Address, buffer: &mut [u8]) -> Result<(), Self::Error> {
        assert_eq!(buffer.len(), 512);
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config);

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        let start_block = u32::try_from(start / 512).unwrap();
        let end_block = u32::try_from((start + buffer.len() as u64).div_ceil(512)).unwrap();

        for block_address in start_block..end_block {
            let mut spi_buffer = [Default::default();
                size_of::<Command>()
                    + COMMAND_BYTES_UNTIL_RESPONSE
                    + size_of::<R1>()
                    + BYTES_UNTIL_READ_DATA
                    + 512];
            let mut response = [Default::default(); size_of::<R1>()];
            // let mut block_bytes = [Default::default(); 512];
            card_command_3(
                spi.deref_mut(),
                &mut spi_buffer,
                &format_command(17, block_address),
                COMMAND_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                Some(CardCommandOperation::Read(ReadOperation {
                    buffer: buffer,
                    expected_bytes_until_data: BYTES_UNTIL_READ_DATA,
                    timeout: READ_TIMEOUT,
                })),
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                    if command_got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    }
                }
                CardCommand3Error::ExpectedStartBlockToken => Error::InitFailed,
                CardCommand3Error::InvalidCrc => Error::InitFailed,
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::InitFailed);
            }

            // defmt::trace!("read block: {:02X}", block_bytes);

            // if block_address == start_block {
            //     let start_offset = start as usize % 512;
            //     let copy_len = min(512 - start_offset, buffer.len());
            //     defmt::trace!("copying {} bytes", copy_len);
            //     buffer[..copy_len]
            //         .copy_from_slice(&block_bytes[start_offset..start_offset + copy_len]);
            // } else if block_address == end_block {
            //     let buffer_start = ((block_address - start_block) * 512) as usize;
            //     let copy_len = min((start as usize + buffer.len()) % 512, buffer.len());
            //     defmt::trace!("copying {} bytes", copy_len);
            //     buffer[buffer_start..].copy_from_slice(&block_bytes[..copy_len]);
            // } else {
            //     let buffer_start = ((block_address - start_block) * 512) as usize;
            //     defmt::trace!("copying 512 bytes");
            //     buffer[buffer_start..buffer_start + 512].copy_from_slice(&block_bytes)
            // }
        }

        spi.flush().await.map_err(Error::SpiBus)?;
        self.sd_card.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        Ok(())
    }

    async fn write(&mut self, start: Self::Address, buffer: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<Spi, Cs: OutputPin, Delayer: DelayNs> SdCardDisk<'_, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    /// Returns the card capacity in bytes
    pub async fn capacity(
        &mut self,
    ) -> Result<u64, Error<<Spi::Bus as ErrorType>::Error, Cs::Error>> {
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config);

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        let csd = {
            let mut buffer = [Default::default();
                size_of::<Command>()
                    + COMMAND_BYTES_UNTIL_RESPONSE
                    + size_of::<R1>()
                    + BYTES_UNTIL_CSD
                    + size_of::<CsdV2>()];
            let mut response = [Default::default(); size_of::<R1>()];
            let mut csd_bytes = [Default::default(); size_of::<CsdV2>()];
            card_command_3(
                spi.deref_mut(),
                &mut buffer,
                &format_command(9, 0),
                COMMAND_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                Some(CardCommandOperation::Read(ReadOperation {
                    buffer: &mut csd_bytes,
                    expected_bytes_until_data: BYTES_UNTIL_CSD,
                    timeout: CSD_TIMEOUT,
                })),
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(command_got_response) => {
                    if command_got_response {
                        Error::InitFailed
                    } else {
                        Error::NoResponse
                    }
                }
                CardCommand3Error::ExpectedStartBlockToken => Error::InitFailed,
                CardCommand3Error::InvalidCrc => Error::InitFailed,
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::InitFailed);
            }
            CsdV2(u128::from_be_bytes(csd_bytes))
        };

        spi.flush().await.map_err(Error::SpiBus)?;
        self.sd_card.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        Ok(csd.card_capacity_bytes())
    }
}
