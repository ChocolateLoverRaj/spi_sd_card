#![no_std]
#![allow(async_fn_in_trait)]

mod card_command;
mod disk;
mod em_hal;
mod error;
mod format_command;
mod pure;
mod sd_card_disk;
mod shared_spi_bus;
mod spi_sd_card;
mod structs;
mod util;

use core::{fmt::Debug, ops::DerefMut};

use card_command::*;
pub use disk::*;
use embassy_embedded_hal::SetConfig;
use embassy_time::Duration;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiBus};
pub use error::*;
pub use format_command::*;
pub use pure::*;
pub use sd_card_disk::*;
pub use shared_spi_bus::*;
pub use spi_sd_card::*;
pub use structs::*;
pub use util::*;

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
