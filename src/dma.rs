use core::fmt::Debug;

use crate::{Error, format_command};
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_hal::{
    Async,
    dma::{DmaRxBuf, DmaTxBuf, DmaTxBuffer},
    spi::master::{Address, Command, DataMode, SpiDma},
};

pub async fn demo<Cs: OutputPin>(
    spi: SpiDma<'_, Async>,
    mut tx: DmaTxBuf,
    rx: DmaRxBuf,
    cs: &mut Cs,
) -> Result<(), Error<esp_hal::spi::Error, Cs::Error>> {
    // The spec says to wait 1ms from when the SD card gets power
    // Realistically it has already been 1ms but just to be sure we can wait 1ms
    Timer::after(Duration::from_millis(1)).await;

    defmt::info!("Writing");

    // Send 0xFF for at least 74 clock cycles according to the spec]
    // So 9 bytes
    let (spi, rx) = spi
        .half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            9,
            0,
            rx,
        )
        .unwrap()
        .wait();

    cs.set_low().unwrap();

    defmt::info!("Doing CMD0");

    // CMD0
    tx.fill(&format_command(0, 0));
    let (spi, (rx, tx)) = spi.transfer(100, rx, tx.len(), tx).unwrap().wait();

    defmt::info!(
        "Received {:02X} bytes. tx len: {}",
        rx.as_slice()[..100],
        tx.len()
    );

    Ok(())
}
