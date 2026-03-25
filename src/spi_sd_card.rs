use core::fmt::Debug;

use embassy_embedded_hal::SetConfig;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;

use crate::*;

pub struct SpiSdCard<Spi, Cs, Delayer>
where
    Spi: SharedSpiBus<u8>,
    Spi::Bus: SetConfig,
{
    pub(crate) spi: Spi,
    pub(crate) cs: Cs,
    pub(crate) delayer: Delayer,
    pub(crate) _400_khz_config: <Spi::Bus as SetConfig>::Config,
    pub(crate) _25_mhz_config: <Spi::Bus as SetConfig>::Config,
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

        // Get OCR
        let _ocr = {
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

        Ok(SdCardDisk {
            sd_card: self,
            enable_read_multiple: true,
        })
    }
}
