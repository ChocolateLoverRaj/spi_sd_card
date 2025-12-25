use embedded_hal_async::spi::SpiBus;

/// We want to do as much as possible within the limits of an underlying buffer
pub fn magic<S: SpiBus>(spi_bus: &mut S) {}

pub struct CommandSender<'a, S> {
    spi_bus: &'a mut S,
    buffer: &'a mut [u8],
    command: &'a [u8],
    /// Includes the command bytes
    bytes_sent: usize,
    /// Does not include the command bytes
    bytes_to_receive: usize,
}

impl<'a, S> CommandSender<'a, S> {
    pub fn new(
        spi_bus: &'a mut S,
        buffer: &'a mut [u8],
        command: &'a mut [u8],
        bytes_to_receive: usize,
    ) -> Self {
        Self {
            spi_bus,
            buffer,
            command,
            bytes_sent: 0,
            bytes_to_receive,
        }
    }
}

impl<'a, S: SpiBus> CommandSender<'a, S> {
    pub async fn next(mut self) -> Result<(&'a [u8], bool), S::Error> {
        let copy_len = self.command.len().min(self.buffer.len());
        self.buffer[..copy_len].copy_from_slice(&self.command[..copy_len]);

        let bytes_to_transfer =
            (self.bytes_to_receive + self.command.len() - self.bytes_sent).min(self.buffer.len());
        self.buffer[copy_len..bytes_to_transfer].fill(0xFF);
        self.spi_bus
            .transfer_in_place(&mut self.buffer[..bytes_to_transfer])
            .await?;

        let bytes_received = &self.buffer[copy_len..bytes_to_transfer];

        self.bytes_sent += bytes_to_transfer;
        Ok((
            bytes_received,
            self.bytes_sent == self.command.len() + self.bytes_to_receive,
        ))
    }
}
