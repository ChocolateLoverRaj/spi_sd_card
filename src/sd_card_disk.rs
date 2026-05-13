use core::cmp::{max, min};

use embassy_time::Instant;

use crate::*;

pub struct SdCardDisk<'a, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    pub(crate) sd_card: &'a mut SpiSdCard<Spi, Cs, Delayer>,
    /// When reading data from SD cards, data is read as 512 B (aligned) blocks.
    /// To read a single block, we can use `CMD17` (`READ_SINGLE_BLOCK`).
    /// To achieve faster speeds when reading consequtive blocks, we can use `CMD18` (`READ_MULTIPLE_BLOCK`).
    /// But in my experience, some SD cards don't work with `CMD18`.
    /// They give a bad CRC.
    /// So you can disable this to always read using CMD17, even when reading consecutive blocks.
    pub enable_read_multiple: bool,
}

pub const BLOCK_SIZE: usize = 512;

impl<Spi, Cs: OutputPin, Delayer: DelayNs> Disk for SdCardDisk<'_, Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
    <Spi::Bus as SetConfig>::ConfigError: Debug,
{
    type Address = u64;
    type Error = Error<Spi::Bus, Cs::Error>;
    const BLOCK_SIZE: usize = BLOCK_SIZE;

    async fn read(&mut self, start: Self::Address, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config)
            .map_err(Error::SpiSetConfig)?;

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        let start_block = u32::try_from(start / 512).unwrap();

        if buffer.len() > 512 && self.enable_read_multiple {
            // for block_address in start_block..end_block {
            // The bigger this is, the better
            // from my testing, 1024 can achieve super fast speeds and there is no need for larger than that
            let mut spi_buffer = [Default::default(); 1024];
            let mut response = [Default::default(); size_of::<R1>()];
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
                    parts: (start as usize + buffer.len()).div_ceil(512) - start as usize / 512,
                    part_size: 512,
                    buffer: buffer,
                    crc_enabled: true,
                    skip_bytes: start as usize % 512,
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
            let end_block = u32::try_from((start + buffer.len() as u64).div_ceil(512)).unwrap();
            for block_address in start_block..end_block {
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
                        buffer: {
                            let start_address = max(block_address as u64 * 512, start);
                            let end_address = min(
                                (block_address as u64 + 1) * 512,
                                start + buffer.len() as u64,
                            );
                            &mut buffer
                                [(start_address - start) as usize..(end_address - start) as usize]
                        },
                        crc_enabled: true,
                        skip_bytes: if block_address == start_block {
                            start as usize % 512
                        } else {
                            0
                        },
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
                    + size_of::<CsdV2Old>()];
            let mut response = [Default::default(); size_of::<R1>()];
            let mut csd_bytes = [Default::default(); size_of::<CsdV2Old>()];
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
            CsdV2Old(u128::from_be_bytes(csd_bytes))
        };

        spi.flush().await.map_err(Error::SpiBus)?;
        self.sd_card.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        Ok(csd.card_capacity_bytes())
    }

    pub async fn get_status(&mut self) -> Result<R2Byte1, Error<Spi::Bus, Cs::Error>> {
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config)
            .map_err(Error::SpiSetConfig)?;

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        let status = {
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R2>()];
            let mut response = [Default::default(); size_of::<R2>()];
            card_command(
                spi.deref_mut(),
                &mut buffer,
                &format_command(13, 0),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::SendStatusResponseTimeout,
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::SendStatusResponseError);
            }
            R2Byte1::from_bits_retain(response[1])
        };

        spi.flush().await.map_err(Error::SpiBus)?;
        self.sd_card.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        Ok(status)
    }

    pub async fn set_card_detect_enable(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<Spi::Bus, Cs::Error>> {
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config)
            .map_err(Error::SpiSetConfig)?;

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        {
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); size_of::<R1>()];
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
            if !r1.is_empty() {
                return Err(Error::Cmd55Failed);
            }
        }
        {
            let mut buffer = [Default::default();
                size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R1>()];
            let mut response = [Default::default(); size_of::<R1>()];
            card_command(
                spi.deref_mut(),
                &mut buffer,
                &format_command(42, {
                    let mut argument = CommandA42Argument::empty();
                    argument.set(CommandA42Argument::SET_CLR_CARD_DETECT, enable);
                    argument.bits()
                }),
                EXPECTED_BYTES_UNTIL_RESPONSE,
                &mut response,
                COMMAND_TIMEOUT,
                None,
            )
            .await
            .map_err(|e| match e {
                CardCommand3Error::Spi(e) => Error::SpiBus(e),
                CardCommand3Error::ReceiveResponseTimeout(_) => Error::Acmd42ResponseError,
                _ => unreachable!(),
            })?;
            let r1 = R1::from_bits_retain(response[0]);
            if !r1.is_empty() {
                return Err(Error::Acmd42ResponseError);
            }
        }

        spi.flush().await.map_err(Error::SpiBus)?;
        self.sd_card.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        Ok(())
    }

    pub async fn test(&mut self) -> Result<(), Error<Spi::Bus, Cs::Error>> {
        let mut spi = self.sd_card.spi.lock().await;
        spi.set_config(&self.sd_card._25_mhz_config)
            .map_err(Error::SpiSetConfig)?;

        self.sd_card.cs.set_low().map_err(Error::CsPin)?;

        for _ in 0..4 {
            let cutoff_len = size_of::<Command>();

            let mut buffer =
                [0xFF; size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R2>()];
            buffer[..6].copy_from_slice(&format_command(13, 0));
            let buffer_to_transfer = &mut buffer[..cutoff_len];
            spi.transfer_in_place(buffer_to_transfer).await.unwrap();

            spi.flush().await.map_err(Error::SpiBus)?;
            self.sd_card.cs.set_high().map_err(Error::CsPin)?;
            let mut dummy_buffer = [0xFF];
            spi.write(&mut dummy_buffer).await.map_err(Error::SpiBus)?;
            spi.flush().await.map_err(Error::SpiBus)?;
            self.sd_card.cs.set_low().map_err(Error::CsPin)?;

            let buffer_to_transfer = &mut buffer[cutoff_len..];
            spi.transfer_in_place(buffer_to_transfer).await.unwrap();
        }

        spi.flush().await.map_err(Error::SpiBus)?;
        self.sd_card.cs.set_high().map_err(Error::CsPin)?;
        spi.write(&[0xFF]).await.map_err(Error::SpiBus)?;
        spi.flush().await.map_err(Error::SpiBus)?;

        Ok(())
    }
}
