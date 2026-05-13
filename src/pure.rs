use crc::{CRC_16_XMODEM, Crc, Table};

use crate::{
    Command, Command8Argument, Command59Argument, CommandA41Argument, DataErrorToken, R1, R7,
    R7Byte1, R7Byte3, START_BLOCK_TOKEN, VoltageAccpted, format_command,
};

// Command-level stuff

#[derive(Default)]
pub struct SimpleCommand<const N: usize> {
    response: heapless::Vec<u8, N>,
}

impl<const N: usize> SimpleCommand<N> {
    pub fn process_bytes(mut self, bytes: &[u8]) -> SimpleCmdProcess<N> {
        if self.response.is_empty() {
            // Find first non-0xFF
            let response_pos = bytes.iter().position(|byte| *byte != 0xFF);
            if let Some(response_pos) = response_pos {
                self.response
                    .extend_from_slice(
                        &bytes[response_pos..bytes.len().min(response_pos + size_of::<R7>())],
                    )
                    .unwrap();
            }
        } else {
            self.response
                .extend_from_slice(&bytes[..bytes.len().min(N - self.response.len())])
                .unwrap();
        }
        match self.response.into_array() {
            Ok(res) => SimpleCmdProcess::Done(res),
            Err(res) => SimpleCmdProcess::InProgress(Self { response: res }),
        }
    }
}

pub enum SimpleCmdProcess<const N: usize> {
    InProgress(SimpleCommand<N>),
    Done([u8; N]),
}

pub fn format_command_0() -> Command {
    format_command(0, 0)
}

/// Buffer len must be at least 6.
pub fn prepare_command_0(buffer: &mut [u8]) {
    let command = format_command(0, 0);
    buffer[..command.len()].copy_from_slice(&command);
}

/// Call prepare_command_0, then send it, then send dummy bytes. Then call process_bytes.
pub struct Command0;
impl Command0 {
    pub fn process_bytes(self, bytes: &[u8]) -> Command0Process {
        if let Some(byte) = bytes.iter().copied().find(|byte| *byte != 0xFF) {
            Command0Process::Done(R1::from_bits_retain(byte))
        } else {
            Command0Process::InProgress(self)
        }
    }
}

pub enum Command0Process {
    /// Continue transmitting 0xFFs unless you want to timeout
    InProgress(Command0),
    /// Stop transmitting 0xFFs
    Done(R1),
}

#[derive(Debug)]
pub struct UnexpectedCmd0Response(pub R1);

pub fn process_cmd_0_response(response: R1) -> Result<(), UnexpectedCmd0Response> {
    if response == R1::IN_IDLE_STATE || response.is_empty() {
        Ok(())
    } else {
        Err(UnexpectedCmd0Response(response))
    }
}

pub fn format_cmd_8() -> Command {
    format_command(8, {
        let mut argument = Command8Argument(Default::default());
        argument.set_pcie1_2v_support(false);
        argument.set_pcie_availability(false);
        argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
        argument.set_check_pattern(CHECK_PATTERN);
        argument.0
    })
}

/// Assumes 3.3V power to SD card
#[derive(Default)]
pub struct Command8 {
    response: heapless::Vec<u8, { size_of::<R7>() - 1 }>,
}

impl Command8 {
    pub fn process_bytes(self, bytes: &[u8]) -> Command8Process {
        let mut response =
            heapless::Vec::<u8, { size_of::<R7>() }>::from_slice(&self.response).unwrap();
        if self.response.is_empty() {
            // Find first non-0xFF
            let response_pos = bytes.iter().position(|byte| *byte != 0xFF);
            if let Some(response_pos) = response_pos {
                response
                    .extend_from_slice(
                        &bytes[response_pos..bytes.len().min(response_pos + size_of::<R7>())],
                    )
                    .unwrap();
            }
        } else {
            response
                .extend_from_slice(&bytes[..bytes.len().min(size_of::<R7>() - response.len())])
                .unwrap();
        }
        if response.is_full() {
            Command8Process::Done(R7 {
                byte_0: R1::from_bits_retain(response[0]),
                byte_1: R7Byte1(response[1]),
                byte_2: response[2],
                byte_3: R7Byte3(response[3]),
                check_pattern: response[4],
            })
        } else {
            Command8Process::InProgress(Self {
                response: heapless::Vec::from_slice(&response).unwrap(),
            })
        }
    }
}

pub enum Command8Process {
    InProgress(Command8),
    Done(R7),
}

#[derive(Debug)]
pub enum Cmd8Error {
    UnexpectedR1(R1),
    VoltageNotAccepted(VoltageAccpted),
    UnexpectedCheckPattern(u8),
}

pub fn process_cmd_8_res(res: R7) -> Result<(), Cmd8Error> {
    if res.byte_0 != R1::IN_IDLE_STATE {
        return Err(Cmd8Error::UnexpectedR1(res.byte_0));
    }
    let voltage_accepted = res.byte_3.get_voltage_accepted();
    if !voltage_accepted.contains(VoltageAccpted::_2_7V_3_6V) {
        return Err(Cmd8Error::VoltageNotAccepted(voltage_accepted));
    }
    if res.check_pattern != CHECK_PATTERN {
        return Err(Cmd8Error::UnexpectedCheckPattern(res.check_pattern));
    }
    Ok(())
}

pub fn format_cmd_59(crc_on: bool) -> Command {
    format_command(
        59,
        if crc_on {
            Command59Argument::CRC_ON
        } else {
            Command59Argument::empty()
        }
        .bits(),
    )
}

pub struct Command59;
impl Command59 {
    pub fn process_bytes(self, bytes: &[u8]) -> Command59Process {
        if let Some(byte) = bytes.iter().copied().find(|byte| *byte != 0xFF) {
            Command59Process::Done(R1::from_bits_retain(byte))
        } else {
            Command59Process::InProgress(self)
        }
    }
}

pub enum Command59Process {
    /// Continue transmitting 0xFFs unless you want to timeout
    InProgress(Command59),
    /// Stop transmitting 0xFFs
    Done(R1),
}

#[derive(Debug)]
pub struct UnexpectedCmd59Response(pub R1);

pub fn process_cmd_59_res(response: R1) -> Result<(), UnexpectedCmd59Response> {
    if response == R1::IN_IDLE_STATE {
        Ok(())
    } else {
        Err(UnexpectedCmd59Response(response))
    }
}

pub fn format_cmd_55() -> Command {
    format_command(55, 0)
}

pub fn process_cmd_55_response(response: R1) -> Result<(), UnexpectedCmd55Response> {
    if response == R1::IN_IDLE_STATE || response.is_empty() {
        Ok(())
    } else {
        Err(UnexpectedCmd55Response(response))
    }
}

pub fn format_cmd_58() -> Command {
    format_command(58, 0)
}

#[derive(Debug)]
pub struct UnexpectedCmd55Response(pub R1);

pub fn format_acmd_41() -> Command {
    format_command(41, CommandA41Argument::HCS.bits())
}

pub fn process_acmd_41_res(res: R1) -> Result<Acmd41Output, UnexpectedAcmd41Res> {
    if res.is_empty() {
        Ok(Acmd41Output::Initialized)
    } else if res == R1::IN_IDLE_STATE {
        Ok(Acmd41Output::StillInitializing)
    } else {
        Err(UnexpectedAcmd41Res(res))
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Acmd41Output {
    StillInitializing,
    Initialized,
}

#[derive(Debug)]
pub struct UnexpectedAcmd41Res(pub R1);

/// The check pattern can be anything we want
const CHECK_PATTERN: u8 = 0xE2;

pub fn format_cmd_9() -> Command {
    format_command(9, 0)
}

enum ReadSinglePhase {
    WaitForR1,
    WaitForData,
    ProcessData,
}

pub struct ReadSingleCmd {
    len: usize,
    phase: ReadSinglePhase,
}

impl ReadSingleCmd {
    pub fn new(len: usize) -> Self {
        Self {
            len,
            phase: ReadSinglePhase::WaitForR1,
        }
    }

    pub fn process_bytes(
        mut self,
        data: &[u8],
        new_start: usize,
    ) -> Result<ReadSingleProcess, ReadSingleError> {
        let mut i = new_start;
        let mut keep_start = 0;
        loop {
            match self.phase {
                ReadSinglePhase::WaitForR1 => {
                    if let Some(r1_pos) = data[i..].iter().position(|byte| *byte != 0xFF) {
                        let r1 = R1::from_bits_retain(data[i + r1_pos]);
                        if r1 == R1::empty() {
                            i += r1_pos + 1;
                            keep_start = i;
                            self.phase = ReadSinglePhase::WaitForData;
                        } else {
                            return Err(ReadSingleError::UnexpectedResponse(r1));
                        }
                    } else {
                        return Ok(ReadSingleProcess::InProgress {
                            cmd: self,
                            keep_start: data.len(),
                        });
                    }
                }
                ReadSinglePhase::WaitForData => {
                    if let Some(data_pos) =
                        data[i..].iter().position(|byte| *byte == START_BLOCK_TOKEN)
                    {
                        i += data_pos + 1;
                        keep_start = i;
                        self.phase = ReadSinglePhase::ProcessData;
                    } else {
                        return Ok(ReadSingleProcess::InProgress {
                            cmd: self,
                            keep_start: data.len(),
                        });
                    }
                }
                ReadSinglePhase::ProcessData => {
                    if (keep_start..data.len()).len() >= self.len + size_of::<u16>() {
                        // Check CRC
                        let actual_data = &data[keep_start..keep_start + self.len];
                        let received_crc = u16::from_be_bytes(
                            data[keep_start + self.len..keep_start + self.len + size_of::<u16>()]
                                .try_into()
                                .unwrap(),
                        );
                        let crc_builder = Crc::<u16>::new(&CRC_16_XMODEM);
                        let mut digest = crc_builder.digest();
                        digest.update(actual_data);
                        let computed_crc = digest.finalize();
                        if computed_crc != received_crc {
                            return Err(ReadSingleError::CrcError {
                                data_start: keep_start,
                            });
                        }
                        return Ok(ReadSingleProcess::Done {
                            data_start: keep_start,
                        });
                    } else {
                        return Ok(ReadSingleProcess::InProgress {
                            cmd: self,
                            keep_start,
                        });
                    }
                }
            }
        }
    }
}

pub enum ReadSingleProcess {
    InProgress {
        cmd: ReadSingleCmd,
        keep_start: usize,
    },
    Done {
        data_start: usize,
    },
}

#[derive(Debug)]
pub enum ReadSingleError {
    UnexpectedResponse(R1),
    CrcError {
        /// Start of data (which could be corrupted due to CRC mismatch)
        data_start: usize,
    },
}

pub fn format_cmd_17(block_number: u32) -> Command {
    format_command(17, block_number)
}

pub fn format_cmd_18(block_number: u32) -> Command {
    format_command(18, block_number)
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
enum ReadMultiPhase {
    ReceiveResponse,
    ReceiveStartToken,
    ReceiveDataAndCrc,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct ReadMultiCmd {
    block_len: usize,
    phase: ReadMultiPhase,
}

impl ReadMultiCmd {
    pub fn new(block_len: usize) -> Self {
        Self {
            block_len,
            phase: ReadMultiPhase::ReceiveResponse,
        }
    }

    pub fn process_bytes(mut self, buffer: &[u8]) -> Result<ReadMultiOutput, ProcessBlockError> {
        let mut keep_start = 0;
        let mut process_index = 0;
        loop {
            match self.phase {
                ReadMultiPhase::ReceiveResponse => {
                    if let Some(r1_pos) = buffer[process_index..]
                        .iter()
                        .position(|byte| *byte != 0xFF)
                    {
                        let r1 = R1::from_bits_retain(buffer[process_index + r1_pos]);
                        if r1 != R1::empty() {
                            break Err(ProcessBlockError::UnexpectedResponse(r1));
                        }
                        process_index += r1_pos + 1;
                        keep_start = process_index;
                        self.phase = ReadMultiPhase::ReceiveStartToken;
                    } else {
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_start,
                            processed_block: None,
                        });
                    }
                }
                ReadMultiPhase::ReceiveStartToken => {
                    if let Some(token_pos) = buffer[process_index..]
                        .iter()
                        .position(|byte| *byte != 0xFF)
                    {
                        let token = buffer[process_index + token_pos];
                        if token != START_BLOCK_TOKEN {
                            break Err(ProcessBlockError::DataError(
                                DataErrorToken::new_with_raw_value(token),
                            ));
                        }
                        process_index += token_pos + 1;
                        keep_start = process_index;
                        self.phase = ReadMultiPhase::ReceiveDataAndCrc;
                    } else {
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_start,
                            processed_block: None,
                        });
                    }
                }
                ReadMultiPhase::ReceiveDataAndCrc => {
                    if (keep_start..buffer.len()).len() >= self.block_len + size_of::<u16>() {
                        // Check CRC
                        let actual_data = &buffer[keep_start..keep_start + self.block_len];
                        let received_crc = u16::from_be_bytes(
                            buffer[keep_start + self.block_len
                                ..keep_start + self.block_len + size_of::<u16>()]
                                .try_into()
                                .unwrap(),
                        );
                        const CRC: Crc<u16, Table<16>> = Crc::<u16, Table<16>>::new(&CRC_16_XMODEM);
                        let mut digest = CRC.digest();
                        digest.update(actual_data);
                        let computed_crc = digest.finalize();
                        let data_start = keep_start;
                        keep_start += self.block_len + size_of::<u16>();
                        self.phase = ReadMultiPhase::ReceiveStartToken;
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_start,
                            processed_block: Some(if computed_crc == received_crc {
                                Ok(data_start)
                            } else {
                                Err(ProcessedBlockCrcError { start: data_start })
                            }),
                        });
                    } else {
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_start,
                            processed_block: None,
                        });
                    }
                }
            }
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct ReadMultiOutput {
    pub cmd: ReadMultiCmd,
    /// You can throw away bytes before this position if you want. Next time you call the process
    /// bytes function, start at this position.
    pub keep_start: usize,
    /// If this is `Some`, call the process bytes function again immediately to process more blocks
    /// if available.
    pub processed_block: Option<Result<usize, ProcessedBlockCrcError>>,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct ProcessedBlockCrcError {
    pub start: usize,
}

#[derive(Debug)]
pub enum ProcessBlockError {
    UnexpectedResponse(R1),
    DataError(DataErrorToken),
}
