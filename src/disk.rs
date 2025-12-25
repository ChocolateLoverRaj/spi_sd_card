/// A simple trait for a disk, which can be thought of as an [u8] where reading and writing is async and fallible.
/// The disk guarantees that nothing else can read or write to the disk.
/// The length of the disk can never change.
pub trait Disk {
    type Address;
    type Error;

    // fn len(&self) -> Self::Address;
    async fn read(&mut self, start: Self::Address, buffer: &mut [u8]) -> Result<(), Self::Error>;
    async fn write(&mut self, start: Self::Address, buffer: &[u8]) -> Result<(), Self::Error>;
}
