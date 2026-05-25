use core::{cmp::min, mem, num::NonZero};

use crc::{CRC_16_XMODEM, Crc, Table};
use embedded_hal::digital::PinState;

use crate::{
    Command, Command8Argument, Command59Argument, CommandA41Argument, DataErrorToken,
    EXPECTED_BYTES_UNTIL_RESPONSE, R1, R7Byte3, START_BLOCK_TOKEN, VoltageAccpted, format_command,
};

// Command-level stuff
enum SimpleCommandPhase {
    SendCmd { bytes_sent: usize },
    ReceiveResponse { bytes_sent: usize },
}

pub struct SimpleCommand<const N: usize> {
    phase: SimpleCommandPhase,
    response: heapless::Vec<u8, N>,
}

impl<const N: usize> Default for SimpleCommand<N> {
    fn default() -> Self {
        Self {
            phase: SimpleCommandPhase::SendCmd { bytes_sent: 0 },
            response: Default::default(),
        }
    }
}

impl<const N: usize> SimpleCommand<N> {
    pub fn remaining_bytes(&self) -> RemainingBytes {
        match self.phase {
            SimpleCommandPhase::SendCmd { bytes_sent } => RemainingBytes {
                min: size_of::<Command>() - bytes_sent + N,
                expected: size_of::<Command>() - bytes_sent + EXPECTED_BYTES_UNTIL_RESPONSE + N,
            },
            SimpleCommandPhase::ReceiveResponse { bytes_sent } => RemainingBytes {
                min: N,
                expected: EXPECTED_BYTES_UNTIL_RESPONSE - bytes_sent + N,
            },
        }
    }

    pub fn process_bytes(mut self, bytes: &[u8]) -> SimpleCmdProcess<N> {
        if self.response.is_empty() {
            // Find first non-0xFF
            let response_pos = bytes.iter().position(|byte| *byte != 0xFF);
            if let Some(response_pos) = response_pos {
                self.response
                    .extend_from_slice(&bytes[response_pos..bytes.len().min(response_pos + N)])
                    .unwrap();
            }
        } else {
            self.response
                .extend_from_slice(&bytes[..bytes.len().min(N - self.response.len())])
                .unwrap();
        }
        match self.response.into_array() {
            Ok(res) => SimpleCmdProcess::Done(res),
            Err(res) => SimpleCmdProcess::InProgress(Self {
                response: res,
                phase: self.phase,
            }),
        }
    }
}

struct RemainingBytes {
    min: usize,
    expected: usize,
}

pub enum SimpleCmdProcess<const N: usize> {
    InProgress(SimpleCommand<N>),
    Done([u8; N]),
}

pub fn format_cmd_0() -> Command {
    format_command(0, 0)
}

#[derive(Debug)]
pub struct UnexpectedCmd0Response(pub R1);

pub fn process_cmd_0_res(response: u8) -> Result<(), UnexpectedCmd0Response> {
    let response = R1::from_bits_truncate(response);
    if response == R1::IN_IDLE_STATE {
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

#[derive(Debug)]
pub enum Cmd8Error {
    UnexpectedR1(R1),
    VoltageNotAccepted(VoltageAccpted),
    UnexpectedCheckPattern(u8),
}

pub type Cmd8Res = [u8; 5];

pub fn process_cmd_8_res(res: Cmd8Res) -> Result<(), Cmd8Error> {
    let r1 = R1::from_bits_retain(res[0]);
    if r1 != R1::IN_IDLE_STATE {
        return Err(Cmd8Error::UnexpectedR1(r1));
    }
    let voltage_accepted = R7Byte3(res[3]).get_voltage_accepted();
    if !voltage_accepted.contains(VoltageAccpted::_2_7V_3_6V) {
        return Err(Cmd8Error::VoltageNotAccepted(voltage_accepted));
    }
    let check_pattern = res[4];
    if check_pattern != CHECK_PATTERN {
        return Err(Cmd8Error::UnexpectedCheckPattern(check_pattern));
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
    ReceiveData { bytes_received: usize },
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

    pub fn process_bytes(mut self, bytes: &[u8]) -> Result<ReadSingleProcess, ReadSingleError> {
        let mut bytes_processed = 0;
        loop {
            match self.phase {
                ReadSinglePhase::WaitForR1 => {
                    if let Some(r1_pos) = bytes.iter().position(|byte| *byte != 0xFF) {
                        let r1 = R1::from_bits_retain(bytes[r1_pos]);
                        if r1 == R1::empty() {
                            bytes_processed += r1_pos + 1;
                            self.phase = ReadSinglePhase::WaitForData;
                        } else {
                            break Err(ReadSingleError::UnexpectedResponse(r1));
                        }
                    } else {
                        break Ok(ReadSingleProcess::InProgress {
                            cmd: self,
                            keep_start: None,
                        });
                    }
                }
                ReadSinglePhase::WaitForData => {
                    if let Some(data_pos) = bytes[bytes_processed..]
                        .iter()
                        .position(|byte| *byte == START_BLOCK_TOKEN)
                    {
                        bytes_processed += data_pos + 1;
                        self.phase = ReadSinglePhase::ReceiveData { bytes_received: 0 };
                    } else {
                        break Ok(ReadSingleProcess::InProgress {
                            cmd: self,
                            keep_start: None,
                        });
                    }
                }
                ReadSinglePhase::ReceiveData { bytes_received } => {
                    let remaining = self.len + size_of::<u16>() - bytes_received;
                    let bytes_available = bytes.len() - bytes_processed;
                    if bytes_available >= remaining {
                        bytes_processed += remaining;
                        break Ok(ReadSingleProcess::Done { bytes_processed });
                    } else {
                        self.phase = ReadSinglePhase::ReceiveData {
                            bytes_received: bytes_received + bytes_available,
                        };
                        let keep_start = if bytes_received == 0 {
                            Some(bytes_processed)
                        } else {
                            None
                        };
                        break Ok(ReadSingleProcess::InProgress {
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
        keep_start: Option<usize>,
    },
    Done {
        bytes_processed: usize,
    },
}

#[derive(Debug)]
pub enum ReadSingleError {
    UnexpectedResponse(R1),
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
    ReceiveDataAndCrc { bytes_received: usize },
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct ReadMultiCmd {
    block_len: usize,
    phase: ReadMultiPhase,
}

pub const CRC: Crc<u16, Table<16>> = Crc::<u16, Table<16>>::new(&CRC_16_XMODEM);

impl ReadMultiCmd {
    pub fn new(block_len: usize) -> Self {
        Self {
            block_len,
            phase: ReadMultiPhase::ReceiveResponse,
        }
    }

    pub fn process_bytes(mut self, buffer: &[u8]) -> Result<ReadMultiOutput, ProcessBlockError> {
        let mut bytes_processed = 0;
        let mut bytes_waited = 0;
        loop {
            match self.phase {
                ReadMultiPhase::ReceiveResponse => {
                    if let Some(r1_pos) = buffer.iter().position(|byte| *byte != 0xFF) {
                        let r1 = R1::from_bits_retain(buffer[r1_pos]);
                        if r1 != R1::empty() {
                            break Err(ProcessBlockError::UnexpectedResponse(r1));
                        }
                        bytes_waited += r1_pos;
                        bytes_processed += r1_pos + 1;
                        self.phase = ReadMultiPhase::ReceiveStartToken;
                    } else {
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_action: None,
                            bytes_processed: buffer.len(),
                            bytes_waited,
                        });
                    }
                }
                ReadMultiPhase::ReceiveStartToken => {
                    if let Some(token_pos) = buffer[bytes_processed..]
                        .iter()
                        .position(|byte| *byte != 0xFF)
                    {
                        let token = buffer[bytes_processed + token_pos];
                        if token != START_BLOCK_TOKEN {
                            break Err(ProcessBlockError::DataError(
                                DataErrorToken::new_with_raw_value(token),
                            ));
                        }
                        bytes_waited += token_pos;
                        bytes_processed += token_pos + 1;
                        self.phase = ReadMultiPhase::ReceiveDataAndCrc { bytes_received: 0 };
                    } else {
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_action: None,
                            bytes_processed: buffer.len(),
                            bytes_waited,
                        });
                    }
                }
                ReadMultiPhase::ReceiveDataAndCrc { bytes_received } => {
                    let bytes_available = buffer.len() - bytes_processed;
                    let bytes_remaining = self.block_len + size_of::<u16>() - bytes_received;
                    if bytes_available >= bytes_remaining {
                        bytes_processed += bytes_remaining;
                        self.phase = ReadMultiPhase::ReceiveStartToken;
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_action: Some(KeepAction::Take),
                            bytes_processed,
                            bytes_waited,
                        });
                    } else {
                        self.phase = ReadMultiPhase::ReceiveDataAndCrc {
                            bytes_received: bytes_received + bytes_available,
                        };
                        let keep_action = if bytes_received == 0 {
                            Some(KeepAction::StartKeeping {
                                position: bytes_processed,
                            })
                        } else {
                            None
                        };
                        bytes_processed += bytes_available;
                        break Ok(ReadMultiOutput {
                            cmd: self,
                            keep_action,
                            bytes_processed,
                            bytes_waited,
                        });
                    }
                }
            }
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum KeepAction {
    /// Started a data + crc block
    StartKeeping { position: usize },
    /// Done processing a data + crc block
    Take,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct ReadMultiOutput {
    pub cmd: ReadMultiCmd,
    pub keep_action: Option<KeepAction>,
    pub bytes_processed: usize,
    pub bytes_waited: usize,
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

pub fn format_cmd_12() -> Command {
    format_command(12, 0)
}

#[derive(Debug)]
pub struct CrcMismatch;

pub fn check_crc(data: &[u8], crc: [u8; 2]) -> Result<(), CrcMismatch> {
    let received_crc = u16::from_be_bytes(crc);
    let mut digest = CRC.digest();
    digest.update(data);
    let computed_crc = digest.finalize();
    if received_crc == computed_crc {
        Ok(())
    } else {
        Err(CrcMismatch)
    }
}

enum CmdPhase {
    SetCsLow,
    Transfer,
    SetCsHigh,
    SendClock,
}

enum InitPhase {
    Set400Khz,
    SendClocks,
    Cmd0InProgress {
        phase: CmdPhase,
        command: SimpleCommand<{ size_of::<R1>() }>,
        failed_attempts: usize,
    },
    Cmd0Complete {
        phase: FinishCmdPhase,
    },
    Cmd0Failed {
        phase: FinishCmdPhase,
    },
}

enum FinishCmdPhase {
    SetCsHigh,
    SendClock,
}

pub struct Init {
    phase: InitPhase,
}

impl Default for Init {
    fn default() -> Self {
        Self {
            phase: InitPhase::Set400Khz,
        }
    }
}

impl Init {
    pub fn action(&self) -> Action {
        match &self.phase {
            InitPhase::Set400Khz => Action::SetSpiRate(400_000),
            InitPhase::SendClocks => Action::SendClocks(74 / 8),
            InitPhase::Cmd0InProgress {
                phase: CmdPhase::SetCsLow,
                ..
            } => Action::SetCs(PinState::Low),
            InitPhase::Cmd0InProgress {
                phase: CmdPhase::Transfer,
                ..
            } => Action::DoTransfer(TransferInfo {
                min: size_of::<Command>() + size_of::<R1>(),
                expected: size_of::<Command>() + EXPECTED_BYTES_UNTIL_RESPONSE + size_of::<R1>(),
            }),
            InitPhase::Cmd0InProgress {
                phase: CmdPhase::SetCsHigh,
                ..
            } => Action::SetCs(PinState::High),
            InitPhase::Cmd0InProgress {
                phase: CmdPhase::SendClock,
                ..
            } => Action::SendClocks(1),
            InitPhase::Cmd0Complete {
                phase: FinishCmdPhase::SetCsHigh,
            } => Action::SetCs(PinState::High),
            InitPhase::Cmd0Complete {
                phase: FinishCmdPhase::SendClock,
            } => Action::SendClocks(1),
            InitPhase::Cmd0Failed {
                phase: FinishCmdPhase::SetCsHigh,
            } => Action::SetCs(PinState::High),
            InitPhase::Cmd0Failed {
                phase: FinishCmdPhase::SendClock,
            } => Action::SendClocks(1),
        }
    }

    pub fn did_it(mut self, transferred_bytes: Option<&[u8]>) -> Result<InitOutput, InitError> {
        match self.phase {
            InitPhase::Set400Khz => {
                self.phase = InitPhase::Set400Khz;
                Ok(InitOutput::InProgress(self))
            }
            InitPhase::SendClocks => {
                self.phase = InitPhase::Cmd0InProgress {
                    phase: CmdPhase::SetCsLow,
                    command: Default::default(),
                    failed_attempts: 0,
                };
                Ok(InitOutput::InProgress(self))
            }
            InitPhase::Cmd0InProgress {
                phase,
                command,
                failed_attempts,
            } => match phase {
                CmdPhase::SetCsLow => {
                    self.phase = InitPhase::Cmd0InProgress {
                        phase: CmdPhase::Transfer,
                        command,
                        failed_attempts,
                    };
                    Ok(InitOutput::InProgress(self))
                }
                CmdPhase::Transfer => {
                    match command.process_bytes(transferred_bytes.unwrap()) {
                        SimpleCmdProcess::InProgress(command) => {
                            self.phase = InitPhase::Cmd0InProgress {
                                phase,
                                command,
                                failed_attempts,
                            };
                        }
                        SimpleCmdProcess::Done([response]) => match process_cmd_0_res(response) {
                            Ok(()) => {
                                self.phase = InitPhase::Cmd0Complete {
                                    phase: FinishCmdPhase::SetCsHigh,
                                };
                            }
                            Err(_e) => {
                                let failed_attempts = failed_attempts + 1;
                                if failed_attempts < MAX_CMD_0_ATTEMPTS {
                                    self.phase = InitPhase::Cmd0InProgress {
                                        phase: CmdPhase::SetCsHigh,
                                        command: Default::default(),
                                        failed_attempts: failed_attempts + 1,
                                    };
                                } else {
                                    self.phase = InitPhase::Cmd0Failed {
                                        phase: FinishCmdPhase::SetCsHigh,
                                    };
                                }
                            }
                        },
                    }
                    Ok(InitOutput::InProgress(self))
                }
                CmdPhase::SetCsHigh => {
                    self.phase = InitPhase::Cmd0InProgress {
                        phase: CmdPhase::SendClock,
                        command,
                        failed_attempts,
                    };
                    Ok(InitOutput::InProgress(self))
                }
                CmdPhase::SendClock => {
                    self.phase = InitPhase::Cmd0InProgress {
                        phase: CmdPhase::SetCsLow,
                        command,
                        failed_attempts,
                    };
                    Ok(InitOutput::InProgress(self))
                }
            },
            InitPhase::Cmd0Complete {
                phase: FinishCmdPhase::SetCsHigh,
            } => {
                self.phase = InitPhase::Cmd0Complete {
                    phase: FinishCmdPhase::SendClock,
                };
                Ok(InitOutput::InProgress(self))
            }
            InitPhase::Cmd0Complete {
                phase: FinishCmdPhase::SendClock,
            } => {
                todo!("next command");
            }
            InitPhase::Cmd0Failed {
                phase: FinishCmdPhase::SetCsHigh,
            } => {
                self.phase = InitPhase::Cmd0Failed {
                    phase: FinishCmdPhase::SendClock,
                };
                Ok(InitOutput::InProgress(self))
            }
            InitPhase::Cmd0Failed {
                phase: FinishCmdPhase::SendClock,
            } => Err(InitError::Cmd0Failed),
        }
    }

    /// You are responsible for writing this followed by `0xFF`s
    pub fn prepare_transfer(&self) -> [u8; 6] {
        format_cmd_0()
    }
}

pub enum InitError {
    Cmd0Failed,
}

pub enum InitOutput {
    InProgress(Init),
    Done { card_capacity: NonZero<usize> },
}

pub enum Action {
    /// Rate in Hz
    SetSpiRate(u32),
    /// Send [0xFF; n]
    SendClocks(usize),
    SetCs(PinState),
    DoTransfer(TransferInfo),
}

pub struct TransferInfo {
    pub min: usize,
    pub expected: usize,
}

const MAX_CMD_0_ATTEMPTS: usize = 50;

pub const MAX_SEND_CLOCKS: usize = 74 / 8;

const CMD_0_MAX_N: usize = 2;
const CMD_0_EXPECTED_N: usize = 2;

#[derive(defmt::Format)]
enum Cmd0Phase {
    SendCommand(usize),
    WaitForR1(usize),
}

#[derive(defmt::Format)]
pub struct Cmd0 {
    phase: Cmd0Phase,
}

impl Default for Cmd0 {
    fn default() -> Self {
        Self {
            phase: Cmd0Phase::SendCommand(0),
        }
    }
}

impl Cmd0 {
    pub fn transfer(&self) -> TransferInfo2 {
        match self.phase {
            Cmd0Phase::SendCommand(bytes_sent) => TransferInfo2 {
                write_count: size_of::<Command>() - bytes_sent,
                transfer_min: size_of::<Command>() - bytes_sent + size_of::<R1>(),
                transfer_expected: size_of::<Command>() - bytes_sent
                    + CMD_0_EXPECTED_N
                    + size_of::<R1>(),
                transfer_max: size_of::<Command>() - bytes_sent + CMD_0_MAX_N + size_of::<R1>(),
            },
            Cmd0Phase::WaitForR1(bytes_received) => TransferInfo2 {
                write_count: 0,
                transfer_min: size_of::<R1>(),
                transfer_expected: CMD_0_EXPECTED_N.saturating_sub(bytes_received)
                    + size_of::<R1>(),
                transfer_max: CMD_0_MAX_N - bytes_received + size_of::<R1>(),
            },
        }
    }

    pub fn prepare_buffer(&self, buffer: &mut [u8]) {
        match self.phase {
            Cmd0Phase::SendCommand(bytes_sent) => {
                let command = format_cmd_0();
                let bytes_to_copy = min(command.len() - bytes_sent, buffer.len());
                buffer[..bytes_to_copy]
                    .copy_from_slice(&command[bytes_sent..bytes_sent + bytes_to_copy]);
            }
            Cmd0Phase::WaitForR1(_) => {}
        }
    }

    pub fn process_data(mut self, mut data: &[u8]) -> Cmd0O {
        if let Cmd0Phase::SendCommand(bytes_sent) = self.phase {
            let bytes_remaining = size_of::<Command>() - bytes_sent;
            if data.len() >= bytes_remaining {
                self.phase = Cmd0Phase::WaitForR1(0);
                data = &data[bytes_remaining..];
            } else {
                self.phase = Cmd0Phase::SendCommand(bytes_sent + data.len())
            }
        }
        if let Cmd0Phase::WaitForR1(bytes_received) = self.phase {
            let r1 = data.iter().copied().find(|byte| *byte != 0xFF);
            if let Some(r1) = r1 {
                return Cmd0O::Done(Ok(r1));
            } else {
                let bytes_received = bytes_received + data.len();
                if bytes_received > CMD_0_MAX_N {
                    return Cmd0O::Done(Err(()));
                }
                self.phase = Cmd0Phase::WaitForR1(bytes_received);
            }
        }
        Cmd0O::InProgress(self)
    }
}

#[derive(defmt::Format)]
pub enum Cmd0O {
    InProgress(Cmd0),
    Done(Result<u8, ()>),
}

pub struct TransferInfo2 {
    pub write_count: usize,
    pub transfer_min: usize,
    pub transfer_expected: usize,
    pub transfer_max: usize,
}
