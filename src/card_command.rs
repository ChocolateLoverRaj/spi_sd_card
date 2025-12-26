use core::cmp::min;

use crc::{CRC_16_XMODEM, Crc, Digest};
use embassy_time::{Duration, Instant};
use embedded_hal_async::spi::SpiBus;

use crate::{Command, R1, START_BLOCK_TOKEN};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReadOperation<'a> {
    pub buffer: &'a mut [u8],
    pub expected_bytes_until_data: usize,
    pub timeout: Duration,
    /// Lets you read multiple, so buffer will be N * 512 bytes and parts will be N
    pub parts: usize,
    pub part_size: usize,
    pub crc_enabled: bool,
    /// Lets you skip the first bytes to read into a buffer that wants data starting at an address that is not a multiple of 512
    pub skip_bytes: usize,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WriteOperation<'a> {
    pub buffer: &'a [u8],
    pub expected_bytes_until_data: usize,
    pub timeout: Duration,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CardCommandOperation<'a> {
    Read(ReadOperation<'a>),
    Write(WriteOperation<'a>),
    /// Expected bytes until not busy
    BusySignal(usize),
}

#[derive(Debug)]
pub enum CardCommand3Error<SpiError> {
    Spi(SpiError),
    /// `true` if any data that was not `0xFF` was received
    ReceiveResponseTimeout(bool),
    /// Expected a start block token, but got something else
    ExpectedStartBlockToken,
    InvalidCrc,
    /// Returns the number of data successfully read before the timeout
    ReceiveDataTimeout(usize),
}

/// Supports all commands except for multi block read and write.
pub async fn card_command<S: SpiBus>(
    spi: &mut S,
    buffer: &mut [u8],
    command: &[u8; 6],
    expected_bytes_until_response: usize,
    response: &mut [u8],
    response_timeout: Duration,
    mut operation: Option<CardCommandOperation<'_>>,
) -> Result<(), CardCommand3Error<S::Error>> {
    defmt::trace!("Operations: {:#?}", operation);
    const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_XMODEM);
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    enum Phase<'a> {
        /// Bytes sent
        SendCommand(usize),
        ReceiveResponseStart((Instant, bool)),
        /// Number of bytes of the response received so far
        ReceiveResponse(usize),
        /// Records number of busy bytes
        WaitUntilNotBusy(usize),
        /// Data: parts read
        ReceiveStartBlockToken((Instant, usize)),
        /// Digest, Number of parts, number of bytes of the data received so far
        ReceiveData((Digest<'a, u16>, usize, usize)),
        /// Expected crc, Number of parts read, The byte of the partial CRC received, if any
        ReceiveCrc((u16, usize, Option<u8>)),
        WriteData(usize),
    }
    let mut phase = Phase::SendCommand(0);
    let mut buffer_valid_bytes = 0;
    'spi: loop {
        defmt::trace!("number of bytes to process: {}", buffer_valid_bytes);
        let mut bytes_processed = 0;
        let before = Instant::now();
        while buffer_valid_bytes > bytes_processed {
            defmt::trace!("processing: {:?}", phase);
            let bytes_to_process = &mut buffer[bytes_processed..buffer_valid_bytes];
            match phase {
                Phase::SendCommand(bytes_sent) => {
                    defmt::trace!("send command phase: {}", bytes_sent);
                    let bytes_to_send = size_of::<Command>() - bytes_sent;
                    let command_bytes_sent = min(bytes_to_send, bytes_to_process.len());
                    bytes_processed += command_bytes_sent;
                    let new_bytes_sent = bytes_sent + command_bytes_sent;
                    if new_bytes_sent == size_of::<Command>() {
                        phase = Phase::ReceiveResponseStart((Instant::now(), false));
                    } else {
                        phase = Phase::SendCommand(new_bytes_sent)
                    }
                    defmt::trace!(
                        "send command phase: {}. new bytes sent: {}",
                        bytes_sent,
                        new_bytes_sent
                    );
                }
                Phase::ReceiveResponseStart((start_time, data_received)) => {
                    // Scan for R1
                    let mut i = 0;
                    let mut data_received = data_received;
                    let r1_index = loop {
                        if let Some(&byte) = bytes_to_process.get(i) {
                            defmt::trace!("Byte: 0x{:02X}", byte);
                            if byte != 0xFF {
                                data_received = true;
                                if !R1::from_bits_retain(byte).contains(R1::BIT_7) {
                                    break Some(i);
                                }
                            }
                        } else {
                            break None;
                        }
                        i += 1;
                    };
                    defmt::trace!(
                        "receive response start phase: {}, {}. r1 index: {}",
                        start_time,
                        data_received,
                        r1_index
                    );
                    if let Some(r1_index) = r1_index {
                        bytes_processed += r1_index;
                        phase = Phase::ReceiveResponse(0);
                    } else if start_time.elapsed() >= response_timeout {
                        return Err(CardCommand3Error::ReceiveResponseTimeout(data_received));
                    } else {
                        bytes_processed = buffer_valid_bytes;
                        phase = Phase::ReceiveResponseStart((start_time, data_received));
                    }
                }
                Phase::ReceiveResponse(bytes_received) => {
                    let bytes_to_receive = response.len() - bytes_received;
                    let copy_len = min(bytes_to_receive, bytes_to_process.len());
                    defmt::trace!(
                        "receive response phase: {}. processing bytes: {:02X}. copy len: {}",
                        bytes_received,
                        bytes_to_process,
                        copy_len
                    );
                    response[bytes_received..bytes_received + copy_len]
                        .copy_from_slice(&bytes_to_process[..copy_len]);
                    bytes_processed += copy_len;
                    let new_bytes_received = bytes_received + copy_len;
                    if new_bytes_received == response.len() {
                        match &operation {
                            None => break 'spi,
                            Some(CardCommandOperation::Read(_)) => {
                                phase = Phase::ReceiveStartBlockToken((Instant::now(), 0));
                            }
                            Some(CardCommandOperation::Write(_)) => {
                                phase = Phase::WriteData(0);
                            }
                            Some(CardCommandOperation::BusySignal(_)) => {
                                phase = Phase::WaitUntilNotBusy(0)
                            }
                        }
                    } else {
                        phase = Phase::ReceiveResponse(new_bytes_received);
                    }
                }
                Phase::WaitUntilNotBusy(busy_bytes) => {
                    let mut i = 0;
                    while let Some(&byte) = bytes_to_process.get(i) {
                        if byte != 0 {
                            defmt::trace!("{} bytes until not busy", busy_bytes + i);
                            break 'spi;
                        }
                        i += 1;
                    }
                    phase = Phase::WaitUntilNotBusy(i);
                }
                Phase::ReceiveStartBlockToken((start_time, parts_read)) => {
                    defmt::trace!("receive start block token phase");
                    for &mut byte in bytes_to_process {
                        bytes_processed += 1;
                        if byte != 0xFF {
                            if byte == START_BLOCK_TOKEN {
                                phase = Phase::ReceiveData((CRC.digest(), parts_read, 0));
                                break;
                            } else {
                                defmt::error!(
                                    "expected start block token, but got 0x{:02X} instead",
                                    byte
                                );
                                return Err(CardCommand3Error::ExpectedStartBlockToken);
                            }
                        }
                    }
                    let operation =
                        if let Some(CardCommandOperation::Read(operation)) = &mut operation {
                            operation
                        } else {
                            unreachable!()
                        };
                    if start_time.elapsed() > operation.timeout {
                        return Err(CardCommand3Error::ReceiveDataTimeout(parts_read));
                    }
                }
                Phase::ReceiveData((mut digest, parts_read, bytes_received)) => {
                    defmt::trace!("receive data phase: {}", bytes_received);
                    let operation =
                        if let Some(CardCommandOperation::Read(operation)) = &mut operation {
                            operation
                        } else {
                            unreachable!()
                        };
                    let bytes_left_to_read = operation.part_size - bytes_received;
                    let read_len = min(bytes_left_to_read, bytes_to_process.len());
                    let bytes_to_read = &bytes_to_process[..read_len];
                    {
                        let start = parts_read * operation.part_size + bytes_received;
                        let (dest_start, src_start, copy_len) = if start < operation.skip_bytes {
                            if start + read_len > operation.skip_bytes {
                                // skip some of beginning
                                let bytes_to_skip = operation.skip_bytes - bytes_received;
                                (0, bytes_to_skip, read_len - bytes_to_skip)
                            } else {
                                // skip all bytes we read
                                (0, 0, 0)
                            }
                        } else {
                            // don't skip anything
                            let start = start - operation.skip_bytes;
                            (start, 0, read_len)
                        };
                        // check for end
                        let copy_len = if dest_start + copy_len > operation.buffer.len() {
                            let skip_end = dest_start + copy_len - operation.buffer.len();
                            copy_len - skip_end
                        } else {
                            copy_len
                        };
                        operation.buffer[dest_start..dest_start + copy_len]
                            .copy_from_slice(&bytes_to_read[src_start..src_start + copy_len]);
                    }
                    digest.update(&bytes_to_read);
                    bytes_processed += read_len;
                    let new_bytes_received = bytes_received + read_len;
                    if new_bytes_received == operation.part_size {
                        phase = Phase::ReceiveCrc((digest.finalize(), parts_read, None));
                    } else {
                        phase = Phase::ReceiveData((digest, parts_read, new_bytes_received));
                        defmt::trace!(
                            "did not receive all data in a single transfer (missing {} bytes)",
                            operation.part_size - new_bytes_received
                        );
                    }
                }
                Phase::ReceiveCrc((expected_crc, parts_read, byte_0)) => {
                    defmt::trace!("receive CRC phase: {}", byte_0);
                    if let Some(byte_0) = byte_0 {
                        let byte_1 = bytes_to_process[0];
                        bytes_processed += 1;
                        let crc = u16::from_be_bytes([byte_0, byte_1]);
                        let operation =
                            if let Some(CardCommandOperation::Read(operation)) = &mut operation {
                                operation
                            } else {
                                unreachable!()
                            };

                        if crc == expected_crc || !operation.crc_enabled {
                            let new_parts_read = parts_read + 1;
                            if new_parts_read == operation.parts {
                                break 'spi;
                            } else {
                                phase =
                                    Phase::ReceiveStartBlockToken((Instant::now(), new_parts_read))
                            }
                        } else {
                            return Err(CardCommand3Error::InvalidCrc);
                        }
                    } else {
                        let byte_0 = bytes_to_process[0];
                        bytes_processed += 1;
                        phase = Phase::ReceiveCrc((expected_crc, parts_read, Some(byte_0)));
                    };
                }
                Phase::WriteData(_) => todo!(),
            }
        }
        defmt::trace!("procesing time: {} us", before.elapsed().as_micros());

        // Set up buffer
        let bytes_to_transfer = match &phase {
            Phase::SendCommand(bytes_sent) => {
                let bytes_sent = *bytes_sent;
                let copy_len = min(size_of::<Command>() - bytes_sent, buffer.len());
                buffer[..copy_len].copy_from_slice(&command[bytes_sent..bytes_sent + copy_len]);
                let bytes_to_transfer = (copy_len
                    + expected_bytes_until_response
                    + response.len()
                    + match &operation {
                        None => 0,
                        Some(CardCommandOperation::Read(op)) => {
                            (op.expected_bytes_until_data + op.buffer.len() + size_of::<u16>())
                                * op.parts
                        }
                        Some(CardCommandOperation::Write(_)) => {
                            todo!()
                        }
                        Some(CardCommandOperation::BusySignal(expected_bytes_until_not_busy)) => {
                            *expected_bytes_until_not_busy
                        }
                    })
                .min(buffer.len());
                buffer[copy_len..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveResponseStart(_) => {
                let bytes_to_transfer = (expected_bytes_until_response
                    + response.len()
                    + match &operation {
                        None => 0,
                        Some(CardCommandOperation::Read(op)) => {
                            (op.expected_bytes_until_data + op.buffer.len() + size_of::<u16>())
                                * op.parts
                        }
                        Some(CardCommandOperation::Write(_)) => {
                            todo!()
                        }
                        Some(CardCommandOperation::BusySignal(expected_bytes_until_not_busy)) => {
                            *expected_bytes_until_not_busy
                        }
                    })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveResponse(bytes_received) => {
                let bytes_to_transfer = (response.len() - bytes_received
                    + match &operation {
                        None => 0,
                        Some(CardCommandOperation::Read(op)) => {
                            (op.expected_bytes_until_data + op.buffer.len() + size_of::<u16>())
                                * op.parts
                        }
                        Some(CardCommandOperation::Write(_)) => {
                            todo!()
                        }
                        Some(CardCommandOperation::BusySignal(expected_bytes_until_not_busy)) => {
                            *expected_bytes_until_not_busy
                        }
                    })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveStartBlockToken((_, parts_read)) => {
                let bytes_to_transfer = (if let Some(CardCommandOperation::Read(op)) = &operation {
                    (op.expected_bytes_until_data + op.buffer.len() + size_of::<u16>())
                        * (op.parts - parts_read)
                } else {
                    unreachable!()
                })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveData((_digest, parts_read, bytes_received)) => {
                let bytes_to_transfer = (if let Some(CardCommandOperation::Read(op)) = &operation {
                    op.buffer.len() - bytes_received
                        + size_of::<u16>()
                        + (op.expected_bytes_until_data + op.buffer.len() + size_of::<u16>())
                            * (op.parts - (parts_read + 1))
                } else {
                    unreachable!()
                })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::ReceiveCrc((_expected_crc, parts_read, byte_0)) => {
                let bytes_to_transfer = (if let Some(CardCommandOperation::Read(op)) = &operation {
                    size_of::<u16>() - if byte_0.is_some() { 1 } else { 0 }
                        + (op.expected_bytes_until_data + op.buffer.len() + size_of::<u16>())
                            * (op.parts - (parts_read + 1))
                } else {
                    unreachable!()
                })
                .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::WaitUntilNotBusy(_) => {
                let bytes_to_transfer =
                    (if let Some(CardCommandOperation::BusySignal(expected_bytes_until_not_busy)) =
                        &operation
                    {
                        *expected_bytes_until_not_busy
                    } else {
                        unreachable!()
                    })
                    .min(buffer.len());
                buffer[..bytes_to_transfer].fill(0xFF);
                bytes_to_transfer
            }
            Phase::WriteData(_) => todo!(),
        };
        assert_ne!(bytes_to_transfer, 0, "{:#?}", phase);
        defmt::trace!("transferring...");
        let before = Instant::now();
        spi.transfer_in_place(&mut buffer[..bytes_to_transfer])
            .await
            .map_err(CardCommand3Error::Spi)?;
        defmt::trace!(
            "Transferred {} bytes in {} us",
            bytes_to_transfer,
            before.elapsed().as_micros()
        );
        defmt::trace!("Bytes: {:02X}", &mut buffer[..bytes_to_transfer]);
        buffer_valid_bytes = bytes_to_transfer;
    }
    Ok(())
}
