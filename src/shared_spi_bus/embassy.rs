use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    mutex::{Mutex, MutexGuard},
};
use embedded_hal_async::spi::SpiBus;

use crate::SharedSpiBus;

/// This is very similar to [`embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice`], except that it doesn't control a CS pin.
pub struct EmbassySharedSpiBus<'a, M: RawMutex, BUS> {
    bus: &'a Mutex<M, BUS>,
}

impl<'a, M: RawMutex, BUS> EmbassySharedSpiBus<'a, M, BUS> {
    pub fn new(bus: &'a Mutex<M, BUS>) -> Self {
        Self { bus }
    }
}

impl<'a, M: RawMutex, BUS: SpiBus<Word>, Word: Copy + 'static> SharedSpiBus<Word>
    for EmbassySharedSpiBus<'a, M, BUS>
{
    type Bus = BUS;
    type Guard = MutexGuard<'a, M, BUS>;

    async fn lock(&self) -> MutexGuard<'a, M, BUS> {
        self.bus.lock().await
    }
}
