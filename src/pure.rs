use crate::{
    Command, Command8Argument, Command59Argument, CommandA41Argument, R1, R7, R7Byte1, R7Byte3,
    VoltageAccpted, format_command,
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
    if response == R1::IN_IDLE_STATE {
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

// #[derive(Debug)]
// pub struct ResetAndInit {
//     step: Step,
// }

// impl Default for ResetAndInit {
//     fn default() -> Self {
//         Self {
//             step: Step::SendClockCycles,
//         }
//     }
// }

// pub enum Action {
//     /// Create a buffer initialized to 0xFF, call the prepare fn, then transfer in place
//     TransferInPlace(usize),
//     SetCs(PinState),
// }

// #[derive(Debug)]
// pub enum Step {
//     SendClockCycles,
//     SetCsLow,
//     Cmd0,
//     Cmd8(heapless::Vec<u8, { size_of::<R7>() }>),
// }

// pub enum ActionResponse<'a> {
//     Transferred(&'a [u8]),
//     SetCs,
// }

// #[derive(Debug)]
// pub enum CardError {
//     NoResponse,
//     BadResponse(R1),
// }

// #[derive(Debug)]
// pub enum Next {
//     Init(ResetAndInit),
//     Done,
// }

/// The check pattern can be anything we want
const CHECK_PATTERN: u8 = 0xE2;

// const EXPECTED_BYTES_UNTIL_CMD0_RESPONSE: usize = 2;

// impl ResetAndInit {
//     pub fn action(&self) -> Action {
//         match self.step {
//             Step::SendClockCycles => Action::TransferInPlace(9),
//             Step::SetCsLow => Action::SetCs(PinState::Low),
//             Step::Cmd0 => Action::TransferInPlace(
//                 size_of::<Command>() + EXPECTED_BYTES_UNTIL_CMD0_RESPONSE + size_of::<R1>(),
//             ),
//             Step::Cmd8(response) => Action::TransferInPlace {
//                 len: size_of::<Command>() + 2 + size_of::<R7>(),
//                 prepare: {
//                     let a = true;
//                     move |buffer| {
//                         if a {
//                         } else {
//                         }
//                         let command = format_command(8, {
//                             let mut argument = Command8Argument(Default::default());
//                             argument.set_pcie1_2v_support(false);
//                             argument.set_pcie_availability(false);
//                             argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
//                             argument.set_check_pattern(CHECK_PATTERN);
//                             argument.0
//                         });
//                         buffer[..command.len()].copy_from_slice(&command);
//                     }
//                 },
//             },
//             _ => todo!(),
//         }
//     }

//     pub fn prepare_buffer(&self, buffer: &mut [u8]) {
//         match self.step {
//             Step::Cmd0 => buffer[..size_of::<Command>()].copy_from_slice(&format_command(0, 0)),
//             Step::Cmd8(response) => {
//                 if response
//                 let command = format_command(8, {
//                     let mut argument = Command8Argument(Default::default());
//                     argument.set_pcie1_2v_support(false);
//                     argument.set_pcie_availability(false);
//                     argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
//                     argument.set_check_pattern(CHECK_PATTERN);
//                     argument.0
//                 });
//                 buffer[..command.len()].copy_from_slice(&command);
//             }
//         }
//     }

//     pub fn next(self, response: ActionResponse<'_>) -> Result<Next, CardError> {
//         match self.step {
//             Step::SendClockCycles => Ok(Next::Init(Self {
//                 step: Step::SetCsLow,
//             })),
//             Step::SetCsLow => Ok(Next::Init(Self { step: Step::Cmd0 })),
//             Step::Cmd0 => {
//                 let buffer = match response {
//                     ActionResponse::Transferred(buffer) => buffer,
//                     _ => unreachable!(),
//                 };
//                 let r1 = R1::from_bits_retain(
//                     buffer
//                         .iter()
//                         .copied()
//                         .find(|byte| *byte != 0xFF)
//                         .ok_or(CardError::NoResponse)?,
//                 );
//                 if r1 == R1::IN_IDLE_STATE {
//                     Ok(Next::Init(Self { step: Step::Cmd8 }))
//                 } else {
//                     Err(CardError::BadResponse(r1))
//                 }
//             }
//             Step::Cmd8 => {
//                 let buffer = match response {
//                     ActionResponse::Transferred(buffer) => buffer,
//                     _ => unreachable!(),
//                 };
//                 let response_position = buffer
//                     .iter()
//                     .copied()
//                     .position(|byte| byte != 0xFF)
//                     .ok_or(CardError::NoResponse)?;
//                 let slice_end = response_position + size_of::<R7>();
//                 let response = &buffer[response_position..slice_end];
//                 Ok(Next::Init(Self { step: Step::Cmd8 }))
//             }
//             _ => todo!(),
//         }
//     }
// }

// pub trait TransferInPlaceNext {
//     type Output;

//     fn process(self, buffer: &[u8]) -> Self::Output;
// }

// pub struct TransferInPlaceAction<Next> {
//     buffer_len: usize,
//     prepare_buffer: fn(&mut [u8]),
//     next: Next,
// }

// impl<Next: TransferInPlaceNext> TransferInPlaceAction<Next> {
//     pub fn buffer_len(&self) -> usize {
//         self.buffer_len
//     }

//     pub fn prepare_buffer(&self, buffer: &mut [u8]) {
//         (self.prepare_buffer)(buffer)
//     }

//     pub fn next(self, buffer: &[u8]) -> Next::Output {
//         self.next.process(buffer)
//     }
// }

// pub struct SetCsAction<Next> {
//     pin_state: PinState,
//     next: Next,
// }

// impl<Next> SetCsAction<Next> {
//     pub fn pin_state(&self) -> PinState {
//         self.pin_state
//     }

//     pub fn next(self) -> Next {
//         self.next
//     }
// }
