#![no_std]
#![allow(async_fn_in_trait)]

use core::{fmt::Debug, ops::DerefMut};

use defmt::warn;
mod shared_spi_bus;
use embassy_embedded_hal::SetConfig;
pub use shared_spi_bus::*;
mod card_command;
mod disk;

mod structs;
mod util;
use card_command::*;
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
pub enum Error<Bus, CsError>
where
    Bus: SpiBus + SetConfig,
    <Bus as SetConfig>::ConfigError: Debug,
{
    // Lower level errors
    /// Error doing SPI transactions
    /// If this error happens, the CS pin might still be set low
    SpiBus(Bus::Error),
    /// Error calling `set_config` on the SPI bus
    SpiSetConfig(<Bus as SetConfig>::ConfigError),
    /// Error setting the level of the CS pin
    /// If this happens, the CS pin might still be set low
    CsPin(CsError),

    // Init errors
    /// Got errors doing CMD0, and retrying didn't succeed
    Cmd0Failed {
        card_present: bool,
    },
    EnableCrcFailed,
    Cmd8Failed,
    /// Command 8 - the SD Card does not support 3.3V
    Cmd8VoltageNotSupported,
    Cmd8InvalidCheckPattern,
    GetOcrFailed,
    /// The OCR has more fine grained info about supported voltage ranges.
    GetOcrVoltageNotSupported,
    Cmd55Failed,
    Acmd41Failed,
    /// The card did not switch from idle to ready before the timeout.
    ReadyTimeout,

    // Read errors
    /// Error receiving a response after sending the read command
    ReadReceiveResponseTimeout,
    /// Got a response from the read command, but it was not ok
    ReadResponseError,
    /// Received data that was in an unexpected format when reading
    ReadUnexpectedData,
    /// Got an ok response, but never received the actual data when reading
    ReadReceiveDataTimeout,
    /// Received data, but the CRC was invalid
    ReadInvalidCrc,
    StopTransmissionResponseTimeout,
    StopTransmissionResponseError,

    // Send CSD errors
    SendCsdResponseTimeout,
    SendCsdResponseError,
    SendCsdDataTimeout,
    SendCsdUnexpectedData,
    SendCsdInvalidCrc,
}

type Command = [u8; 6];

/// This is now many bytes between the end of a command and the start of a response (R1) we expect.
/// It is better to overestimate and read extra bytes than to have to do an additional transaction.
/// Based on my testing of 4 MicroSD cards, the most I've seen is 2
/// But if other people have MicroSD cards that take longer, we can increase this const.
/// If the bytes vary by command, we can use a separate value for different commands.
const EXPECTED_BYTES_UNTIL_RESPONSE: usize = 2;
const COMMAND_TIMEOUT: Duration = Duration::from_millis(100);
/// This is just a guess
const BYTES_UNTIL_CSD: usize = 2;
const CSD_TIMEOUT: Duration = Duration::from_millis(100);
/// In my experience this is up to 2
/// Note that if we make this super big it will reduce performance
/// With `670` we are basically guaranteeing that the transfer speed will be <0.5x of the SPI transfer speed
const BYTES_UNTIL_READ_DATA: usize = 670;
const READ_TIMEOUT: Duration = Duration::from_millis(100);
/// In the SD card I tested, it always had 1 busy byte
const BYTES_UNTIL_NOT_BUSY: usize = 1;
const MAX_ACMD_41_ATTEMPTS: usize = 10_000;

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

impl<Spi, Cs: OutputPin, Delayer: DelayNs> SpiSdCard<Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
    <Spi::Bus as SetConfig>::ConfigError: Debug,
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
    ) -> Result<SdCardDisk<'_, Spi, Cs, Delayer>, Error<Spi::Bus, Cs::Error>> {
        // Wait at least 1ms
        self.delayer.delay_ms(1).await;

        let mut spi = self.spi.lock().await;
        spi.set_config(&self._400_khz_config)
            .map_err(Error::SpiSetConfig)?;

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
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); 1];
            let mut attempt_number = 0;
            let max_attempts = 50;
            loop {
                if attempt_number == max_attempts {
                    break Err(Error::Cmd0Failed {
                        card_present: got_response,
                    });
                }
                let result = card_command(
                    spi.deref_mut(),
                    &mut buffer,
                    &format_command(0, 0),
                    EXPECTED_BYTES_UNTIL_RESPONSE,
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
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); 1];
            card_command(
                spi.deref_mut(),
                &mut buffer,
                &format_command(59, Command59Argument::CRC_ON.bits()),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::EnableCrcFailed,
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if r1 != R1::IN_IDLE_STATE {
                return Err(Error::EnableCrcFailed);
            }
        }

        // Do CMD8
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R7>()];
            let mut response = [Default::default(); size_of::<R7>()];
            // The check pattern can be anything we want
            let check_pattern = 0xE2;
            card_command(
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
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::Cmd8Failed,
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if r1 == R1::ILLEGAL_COMMAND {
                todo!("Handle version 1")
            } else if r1 != R1::IN_IDLE_STATE {
                return Err(Error::Cmd8Failed);
            }
            let byte_3 = R7Byte3(response[3]);
            if !byte_3
                .get_voltage_accepted()
                .contains(VoltageAccpted::_2_7V_3_6V)
            {
                return Err(Error::Cmd8VoltageNotSupported);
            }
            if response[4] != check_pattern {
                return Err(Error::Cmd8InvalidCheckPattern);
            }
        }

        // Get OCR to make sure voltage is compatible
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R3>()];
            let mut response = [Default::default(); size_of::<R3>()];
            card_command(
                spi.deref_mut(),
                &mut buffer,
                &format_command(58, 0),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::GetOcrFailed,
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if r1 != R1::IN_IDLE_STATE {
                return Err(Error::GetOcrFailed);
            }
            let ocr = Ocr::from_bits_retain(u32::from_be_bytes(response[1..5].try_into().unwrap()));
            if !ocr.supports_3_3v() {
                return Err(Error::GetOcrVoltageNotSupported);
            }
        }

        // Initialize card
        {
            let mut attempt_number = 0;
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); size_of::<R1>()];
            loop {
                if attempt_number == MAX_ACMD_41_ATTEMPTS {
                    return Err(Error::ReadyTimeout);
                }
                // CMD55 - next command is an "A" command
                card_command(
                    spi.deref_mut(),
                    &mut buffer,
                    &format_command(55, 0),
                    EXPECTED_BYTES_UNTIL_RESPONSE,
                    &mut response,
                    COMMAND_TIMEOUT,
                    None,
                )
                .await
                .map_err(|e| match e {
                    CardCommand3Error::Spi(e) => Error::SpiBus(e),
                    CardCommand3Error::ReceiveResponseTimeout(_) => Error::Cmd55Failed,
                    _ => unreachable!(),
                })?;
                let r1 = R1::from_bits_retain(response[0]);
                if !(r1 == R1::IN_IDLE_STATE || r1 == R1::empty()) {
                    return Err(Error::Cmd55Failed);
                }

                // ACMD41
                card_command(
                    spi.deref_mut(),
                    &mut buffer,
                    &format_command(41, CommandA41Argument::HCS.bits()),
                    EXPECTED_BYTES_UNTIL_RESPONSE,
                    &mut response,
                    COMMAND_TIMEOUT,
                    None,
                )
                .await
                .map_err(|e| match e {
                    CardCommand3Error::Spi(e) => Error::SpiBus(e),
                    CardCommand3Error::ReceiveResponseTimeout(_) => Error::Acmd41Failed,
                    _ => unreachable!(),
                })?;
                let r1 = R1::from_bits_retain(response[0]);
                if r1 == R1::empty() {
                    break;
                } else if r1 != R1::IN_IDLE_STATE {
                    return Err(Error::Acmd41Failed);
                }
                attempt_number += 1;
            }
        }

        defmt::info!("Reading OCR again");

        // Get OCR
        let ocr = {
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R3>()];
            let mut response = [Default::default(); size_of::<R3>()];
            card_command(
                spi.deref_mut(),
                &mut buffer,
                &format_command(58, 0),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::GetOcrFailed,
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::GetOcrFailed);
            }
            Ocr::from_bits_retain(u32::from_be_bytes(response[1..5].try_into().unwrap()))
        };

        spi.flush().await.map_err(Error::SpiBus)?;
        self.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        defmt::info!("is SDHC or SDXC?: {}", ocr.supports_sdhc_or_sdxc().unwrap());

        Ok(SdCardDisk {
            sd_card: self,
            enable_read_multiple: true,
        })
    }
}

pub struct SdCardDisk<'a, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    sd_card: &'a mut SpiSdCard<Spi, Cs, Delayer>,
    /// When reading data from SD cards, data is read as 512 B (aligned) blocks.
    /// To read a single block, we can use `CMD17` (`READ_SINGLE_BLOCK`).
    /// To achieve faster speeds when reading consequtive blocks, we can use `CMD18` (`READ_MULTIPLE_BLOCK`).
    /// But in my experience, some SD cards don't work with `CMD18`.
    /// They give a bad CRC.
    /// So you can disable this to always read using CMD17, even when reading consecutive blocks.
    pub enable_read_multiple: bool,
}

impl<Spi, Cs: OutputPin, Delayer: DelayNs> Disk for SdCardDisk<'_, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
    <Spi::Bus as SetConfig>::ConfigError: Debug,
{
    type Address = u64;
    type Error = Error<Spi::Bus, Cs::Error>;

    async fn read(&mut self, start: Self::Address, buffer: &mut [u8]) -> Result<(), Self::Error> {
        assert!(buffer.len().is_multiple_of(512));
        assert!(start.is_multiple_of(512));
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config)
            .map_err(Error::SpiSetConfig)?;

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        let start_block = u32::try_from(start / 512).unwrap();
        let end_block = u32::try_from((start + buffer.len() as u64).div_ceil(512)).unwrap();

        let before = Instant::now();
        if buffer.len() > 512 && self.enable_read_multiple {
            // for block_address in start_block..end_block {
            // The bigger this is, the better
            // from my testing, 1024 can achieve super fast speeds and there is no need for larger than that
            let mut spi_buffer = [Default::default(); 1024];
            let mut response = [Default::default(); size_of::<R1>()];
            // let mut block_bytes = [Default::default(); 512];
            card_command(
                spi.deref_mut(),
                &mut spi_buffer,
                &format_command(18, start_block),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                Some(CardCommandOperation::Read(ReadOperation {
                    expected_bytes_until_data: BYTES_UNTIL_READ_DATA,
                    timeout: READ_TIMEOUT,
                    parts: buffer.len() / 512,
                    part_size: 512,
                    buffer: buffer,
                    crc_enabled: true,
                    skip_bytes: 0,
                })),
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::ReadReceiveResponseTimeout,
                CardCommand3Error::ExpectedStartBlockToken => Error::ReadUnexpectedData,
                CardCommand3Error::InvalidCrc => Error::ReadInvalidCrc,
                CardCommand3Error::ReceiveDataTimeout(_) => Error::ReadReceiveResponseTimeout,
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::ReadResponseError);
            }
            card_command(
                spi.deref_mut(),
                &mut spi_buffer,
                &format_command(12, 0),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                Some(CardCommandOperation::BusySignal(BYTES_UNTIL_NOT_BUSY)),
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => {
                    Error::StopTransmissionResponseTimeout
                }
                _ => unreachable!(),
            })?;
            if !r1.is_empty() {
                return Err(Error::StopTransmissionResponseError);
            }
        } else {
            let mut spi_buffer = [Default::default();
                size_of::<Command>()
                    + EXPECTED_BYTES_UNTIL_RESPONSE
                    + size_of::<R1>()
                    + BYTES_UNTIL_READ_DATA
                    + 1
                    + 512
                    + size_of::<u16>()];
            let mut response = [Default::default(); size_of::<R1>()];
            for (i, block_address) in (start_block..end_block).into_iter().enumerate() {
                defmt::info!("Reading single block at 0x{:X}", block_address * 512);
                card_command(
                    spi.deref_mut(),
                    &mut spi_buffer,
                    &format_command(17, block_address),
                    EXPECTED_BYTES_UNTIL_RESPONSE,
                    &mut response,
                    COMMAND_TIMEOUT,
                    Some(CardCommandOperation::Read(ReadOperation {
                        expected_bytes_until_data: BYTES_UNTIL_READ_DATA,
                        timeout: READ_TIMEOUT,
                        parts: 1,
                        part_size: 512,
                        buffer: &mut buffer[512 * i..512 * (i + 1)],
                        crc_enabled: true,
                        skip_bytes: 0,
                    })),
                )
                .await
                .map_err(|e| match e {
                    CardCommand3Error::Spi(e) => Error::SpiBus(e),
                    CardCommand3Error::ReceiveResponseTimeout(_) => {
                        Error::ReadReceiveResponseTimeout
                    }
                    CardCommand3Error::ExpectedStartBlockToken => Error::ReadUnexpectedData,
                    CardCommand3Error::InvalidCrc => Error::ReadInvalidCrc,
                    CardCommand3Error::ReceiveDataTimeout(_) => Error::ReadReceiveDataTimeout,
                })?;
            }
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
        // }

        spi.flush().await.map_err(Error::SpiBus)?;
        defmt::info!(
            "[spi_sd_card] read {} B / {} us @ {:X}",
            buffer.len(),
            before.elapsed().as_micros(),
            start
        );
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
    <Spi::Bus as SetConfig>::ConfigError: Debug,
{
    /// Returns the card capacity in bytes
    pub async fn capacity(&mut self) -> Result<u64, Error<Spi::Bus, Cs::Error>> {
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config)
            .map_err(Error::SpiSetConfig)?;

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        let csd = {
            let mut buffer = [Default::default();
                size_of::<Command>()
                    + EXPECTED_BYTES_UNTIL_RESPONSE
                    + size_of::<R1>()
                    + BYTES_UNTIL_CSD
                    + size_of::<CsdV2>()];
            let mut response = [Default::default(); size_of::<R1>()];
            let mut csd_bytes = [Default::default(); size_of::<CsdV2>()];
            card_command(
                spi.deref_mut(),
                &mut buffer,
                &format_command(9, 0),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                Some(CardCommandOperation::Read(ReadOperation {
                    parts: 1,
                    part_size: csd_bytes.len(),
                    buffer: &mut csd_bytes,
                    expected_bytes_until_data: BYTES_UNTIL_CSD,
                    timeout: CSD_TIMEOUT,
                    crc_enabled: true,
                    skip_bytes: 0,
                })),
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::SendCsdResponseTimeout,
                CardCommand3Error::ExpectedStartBlockToken => Error::SendCsdUnexpectedData,
                CardCommand3Error::ReceiveDataTimeout(_) => Error::SendCsdDataTimeout,
                CardCommand3Error::InvalidCrc => Error::SendCsdInvalidCrc,
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::SendCsdResponseError);
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
