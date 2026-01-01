use core::fmt::Debug;

use embassy_embedded_hal::SetConfig;
use embedded_hal_async::spi::SpiBus;

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

    // Send status errors
    SendStatusResponseTimeout,
    SendStatusResponseError,

    // set_card_detect_enable errors
    Acmd42ResponseTimeout,
    Acmd42ResponseError,
}
