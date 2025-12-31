use crc::{CRC_7_MMC, Crc};

use crate::*;

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
