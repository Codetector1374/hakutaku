use alloc::boxed::Box;
use alloc::vec::Vec;
use crate::process::state::State::*;
use crate::process::state::{EventPollFn, State};
use crate::interrupts::context_switch::TrapFrame;
use crate::process::stack::Stack;

/// Type alias for the type of a process ID.
pub type Id = u64;

/// A structure that represents the complete state of a process.
#[derive(Debug)]
pub struct Process {
    pub pid: Id,
    /// The saved trap frame of a process.
    pub context: Box<TrapFrame>,
    /// The memory allocation used for the process's stack.
    pub stack: Option<Stack>,
    //TODO Page Table
    /// The scheduling state of the process.
    pub state: State,
}

impl Process {
    /// Creates a new process with a zeroed `TrapFrame` (the default), a zeroed
    /// stack of the default size, and a state of `Ready`.
    ///
    /// If enough memory could not be allocated to start the process, returns
    /// `None`. Otherwise returns `Some` of the new `Process`.
    pub fn new() -> Process {
        Process {
            pid: 0,
            context: Box::new(TrapFrame::default()),
            stack: Stack::new(),
            state: State::Ready,
        }
    }

    pub fn new_kern(f: u64) -> Process
    {
        let mut proc = Process {
            pid: 0,
            context: Box::new(Default::default()),
            stack: Stack::new(),
            state: State::Ready
        };
        proc.context.rsp = proc.stack.as_ref().expect("stack").top().as_u64();
        proc.context.rip = f;
        proc
    }

    pub fn new_idle(f: u64) -> Process
    {
        let mut proc = Process {
            pid: 0,
            context: Box::new(Default::default()),
            stack: None,
            state: State::Ready
        };
        proc.context.rip = f;
        proc
    }

    /// Returns `true` if this process is ready to be scheduled.
    ///
    /// This functions returns `true` only if one of the following holds:
    ///
    ///   * The state is currently `Ready`.
    ///
    ///   * An event being waited for has arrived.
    ///
    ///     If the process is currently waiting, the corresponding event
    ///     function is polled to determine if the event being waiting for has
    ///     occured. If it has, the state is switched to `Ready` and this
    ///     function returns `true`.
    ///
    /// Returns `false` in all other cases.
    pub fn ready(&mut self) -> bool {
        let mut state = core::mem::replace(&mut self.state, State::Ready);
        let lmao = match state {
            Ready => true,
            State::Waiting(ref mut lol) => lol(self),
            _ => false
        };
        core::mem::replace(&mut self.state, state);
        return lmao;
    }
}
