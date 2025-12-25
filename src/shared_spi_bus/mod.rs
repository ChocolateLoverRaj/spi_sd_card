#[cfg(feature = "embassy-sync")]
mod embassy;
use core::ops::DerefMut;

#[cfg(feature = "embassy-sync")]
pub use embassy::*;

use embedded_hal_async::spi::SpiBus;

/// This trait is very similar to [`embedded_hal_async::spi::SpiDevice`], but does not include a mechanism to control CS.
/// The user of the functions must ensure that CS is properly being used.
pub trait SharedSpiBus<Word: Copy + 'static> {
    type Bus: SpiBus<Word>;
    type Guard: DerefMut<Target = Self::Bus>;

    async fn lock(&self) -> Self::Guard;
}
