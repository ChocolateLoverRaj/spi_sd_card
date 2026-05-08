use embedded_hal::digital::PinState;

use crate::{Command, Command8Argument, R1, R3, R7, VoltageAccpted, format_command};

pub struct Init;
pub struct SetCsLow;
pub struct Command0;

pub struct InitSequence<T> {
    a: T,
}

impl InitSequence<Init> {
    pub const BYTES_TO_TRANSFER: [u8; 9] = [0xFF; 9];
    pub fn next(self) -> InitSequence<SetCsLow> {
        InitSequence { a: SetCsLow }
    }
}

impl InitSequence<SetCsLow> {
    pub fn next(self) -> InitSequence<Command0> {
        InitSequence { a: Command0 }
    }
}

impl InitSequence<Command0> {}

/// Buffer len must be at least 6.
pub fn prepare_command_0(buffer: &mut [u8]) {
    let command = format_command(0, 0);
    buffer[..command.len()].copy_from_slice(&command);
}

pub fn process_command_0(buffer: &[u8]) -> Option<R1> {
    let r1 = buffer.iter().copied().find(|byte| *byte != 0xFF)?;
    Some(R1::from_bits_retain(r1))
}

#[derive(Debug)]
pub struct ResetAndInit {
    step: Step,
}

impl Default for ResetAndInit {
    fn default() -> Self {
        Self {
            step: Step::SendClockCycles,
        }
    }
}

pub enum Action {
    /// Create a buffer initialized to 0xFF, call the prepare fn, then transfer in place
    TransferInPlace(usize),
    SetCs(PinState),
}

#[derive(Debug)]
pub enum Step {
    SendClockCycles,
    SetCsLow,
    Cmd0,
    Cmd8(heapless::Vec<u8, { size_of::<R7>() }>),
}

pub enum ActionResponse<'a> {
    Transferred(&'a [u8]),
    SetCs,
}

#[derive(Debug)]
pub enum CardError {
    NoResponse,
    BadResponse(R1),
}

#[derive(Debug)]
pub enum Next {
    Init(ResetAndInit),
    Done,
}

/// The check pattern can be anything we want
const CHECK_PATTERN: u8 = 0xE2;

impl ResetAndInit {
    pub fn action(&self) -> Action {
        match self.step {
            Step::SendClockCycles => Action::TransferInPlace(9),
            Step::SetCsLow => Action::SetCs(PinState::Low),
            Step::Cmd0 => Action::TransferInPlace {
                len: size_of::<Command>() + 2 + size_of::<R1>(),
                prepare: |buffer| prepare_command_0(buffer),
            },
            Step::Cmd8(response) => Action::TransferInPlace {
                len: size_of::<Command>() + 2 + size_of::<R7>(),
                prepare: {
                    let a = true;
                    move |buffer| {
                        if a {
                        } else {
                        }
                        let command = format_command(8, {
                            let mut argument = Command8Argument(Default::default());
                            argument.set_pcie1_2v_support(false);
                            argument.set_pcie_availability(false);
                            argument.set_voltage_accepted(VoltageAccpted::_2_7V_3_6V.bits());
                            argument.set_check_pattern(CHECK_PATTERN);
                            argument.0
                        });
                        buffer[..command.len()].copy_from_slice(&command);
                    }
                },
            },
            _ => todo!(),
        }
    }

    pub fn prepare_buffer(&self, buffer: &mut [u8]) {}

    pub fn next(self, response: ActionResponse<'_>) -> Result<Next, CardError> {
        match self.step {
            Step::SendClockCycles => Ok(Next::Init(Self {
                step: Step::SetCsLow,
            })),
            Step::SetCsLow => Ok(Next::Init(Self { step: Step::Cmd0 })),
            Step::Cmd0 => {
                let buffer = match response {
                    ActionResponse::Transferred(buffer) => buffer,
                    _ => unreachable!(),
                };
                let r1 = R1::from_bits_retain(
                    buffer
                        .iter()
                        .copied()
                        .find(|byte| *byte != 0xFF)
                        .ok_or(CardError::NoResponse)?,
                );
                if r1 == R1::IN_IDLE_STATE {
                    Ok(Next::Init(Self { step: Step::Cmd8 }))
                } else {
                    Err(CardError::BadResponse(r1))
                }
            }
            Step::Cmd8 => {
                let buffer = match response {
                    ActionResponse::Transferred(buffer) => buffer,
                    _ => unreachable!(),
                };
                let response_position = buffer
                    .iter()
                    .copied()
                    .position(|byte| byte != 0xFF)
                    .ok_or(CardError::NoResponse)?;
                let slice_end = response_position + size_of::<R7>();
                let response = &buffer[response_position..slice_end];
                Ok(Next::Init(Self { step: Step::Cmd8 }))
            }
            _ => todo!(),
        }
    }
}

pub trait TransferInPlaceNext {
    type Output;

    fn process(self, buffer: &[u8]) -> Self::Output;
}

pub struct TransferInPlaceAction<Next> {
    buffer_len: usize,
    prepare_buffer: fn(&mut [u8]),
    next: Next,
}

impl<Next: TransferInPlaceNext> TransferInPlaceAction<Next> {
    pub fn buffer_len(&self) -> usize {
        self.buffer_len
    }

    pub fn prepare_buffer(&self, buffer: &mut [u8]) {
        (self.prepare_buffer)(buffer)
    }

    pub fn next(self, buffer: &[u8]) -> Next::Output {
        self.next.process(buffer)
    }
}

pub struct SetCsAction<Next> {
    pin_state: PinState,
    next: Next,
}

impl<Next> SetCsAction<Next> {
    pub fn pin_state(&self) -> PinState {
        self.pin_state
    }

    pub fn next(self) -> Next {
        self.next
    }
}
