#![no_std]
mod structs;

use crc::{CRC_7_MMC, CRC_16_XMODEM, Crc};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
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
}

/// Does not modify CS
pub async fn card_command<S: SpiBus>(spi_bus: &mut S, command: &[u8; 6]) -> Result<R1, S::Error> {
    spi_bus.write(command).await?;
    let r1 = loop {
        let mut buffer = [0xFF; 1];
        spi_bus.transfer_in_place(&mut buffer).await?;
        let r1 = R1::from_bits_retain(buffer[0]);
        if !r1.contains(R1::BIT_7) {
            break r1;
        } else {
            Timer::after_micros(10).await;
            // TODO: Timeout
        }
    };
    Ok(r1)
}

pub async fn command_0<Bus: SpiBus, Cs: OutputPin>(
    spi_bus: &mut Bus,
    cs: &mut Cs,
) -> Result<(), Error<Bus::Error, Cs::Error>> {
    cs.set_low().map_err(Error::CsPin)?;
    let result = {
        let r1 = card_command(spi_bus, &format_command(0, 0))
            .await
            .map_err(Error::SpiBus)?;
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
    cs.set_low().map_err(Error::CsPin)?;
    let r1 = card_command(
        spi_bus,
        &format_command(8, {
            let mut argument = Command8Argument(Default::default());
            argument.set_pcie1_2v_support(false);
            argument.set_pcie_availability(false);
            argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
            argument.set_check_pattern(check_pattern);
            argument.0
        }),
    )
    .await
    .map_err(Error::SpiBus)?;
    let result = {
        if r1 == R1::IN_IDLE_STATE {
            let mut buffer = [0xFF; 4];
            spi_bus
                .transfer_in_place(&mut buffer)
                .await
                .map_err(Error::SpiBus)?;

            let response_check_pattern = buffer[3];
            if response_check_pattern == check_pattern {
                let byte_3 = R7Byte3(buffer[2]);
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
        }
    };
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
