use alloc::boxed::Box;
use alloc::collections::vec_deque::VecDeque;
use alloc::vec::Vec;

use core::fmt;
use core::mem;
use core::time::Duration;
use crate::process::process::{Process, Id, ProcessSummary};
use crate::process::state::State;
use crate::interrupts::context_switch::{TrapFrame, restore_context_wrapper};
use spin::Mutex;
use x86_64::instructions::hlt;
use crate::hardware::apic::GLOBAL_APIC;
use crate::{SCHEDULER, kernel_initialization_process};
use crate::process::state::State::Running;
use crate::process::cpu::Processors;
use x86_64::instructions::interrupts::{without_interrupts, enable_interrupts_and_hlt};

/// Process scheduler for the entire machine.
#[derive(Debug)]
pub struct GlobalScheduler(Mutex<Option<Scheduler>>);

const SCHEDULER_TICK: Duration = Duration::from_millis(200);

impl GlobalScheduler {
    /// Returns an uninitialized wrapper around a local scheduler.
    pub const fn uninitialized() -> GlobalScheduler {
        GlobalScheduler(Mutex::new(None))
    }

    /// Enters a critical region and execute the provided closure with a mutable
    /// reference to the inner scheduler.
    pub fn critical<F, R>(&self, f: F) -> R
        where
            F: FnOnce(&mut Scheduler) -> R,
    {
        without_interrupts(|| {
            let mut guard = self.0.lock();
            f(guard.as_mut().expect("scheduler uninitialized"))
        })
    }

    pub fn try_critical<F, R>(&self, f: F) -> Option<R>
        where F: FnOnce(&mut Scheduler) -> R
    {
        without_interrupts(|| {
            match self.0.try_lock() {
                Some(mut g) => {
                    Some(f(g.as_mut().expect("no scheduler?")))
                },
                None => {
                    None
                }
            }
        })
    }

    /// Adds a process to the scheduler's queue and returns that process's ID.
    /// For more details, see the documentation on `Scheduler::add()`.
    pub fn add(&self, process: Process) -> Option<Id> {
        self.critical(move |scheduler| scheduler.add(process))
    }

    /// Performs a context switch using `tf` by setting the state of the current
    /// process to `new_state`, saving `tf` into the current process, and
    /// restoring the next process's trap frame into `tf`. For more details, see
    /// the documentation on `Scheduler::schedule_out()` and `Scheduler::switch_to()`.
    pub fn switch(&self, new_state: State, tf: &mut TrapFrame) -> Id {
        match
            self.try_critical(|scheduler| {
                scheduler.schedule_out(new_state, tf)
            })
        {
            Some(_) => {
                self.switch_to(tf)
            },
            _ => {
                debug!("Failed to context switch on core {}", GLOBAL_APIC.read().apic_id());
                0
            }
        }
    }

    pub fn summary(&self) -> Vec<ProcessSummary> {
        let mut vec = Vec::new();
        self.critical(|s| {
            for process in s.processes.iter() {
                vec.push(ProcessSummary::from(process));
            }
        });
        vec
    }

    /// Loops until it finds the next process to schedule.
    /// Call `wfi()` in the loop when no process is ready.
    /// For more details, see the documentation on `Scheduler::switch_to()`.
    ///
    /// Returns the process's ID when a ready process is found.
    pub fn switch_to(&self, tf: &mut TrapFrame) -> Id {
        loop {
            let rtn = self.critical(|scheduler| scheduler.switch_to(tf));
            if let Some(id) = rtn {
                return id;
            }
            // Switch to an "idle process"
            self.critical(|s| s.idle(tf));
            return 0;
        }
    }

    /// Kills currently running process and returns that process's ID.
    /// For more details, see the documentation on `Scheduler::kill()`.
    #[must_use]
    pub fn kill(&self, tf: &mut TrapFrame) -> Option<Id> {
        self.critical(|scheduler| scheduler.kill(tf))
    }

    /// Starts executing processes in user space using timer interrupt based
    /// preemptive scheduling. This method should not return under normal
    /// conditions.
    pub fn start(&self) -> ! {
        GLOBAL_APIC.write().set_timer_interval(SCHEDULER_TICK).expect("unable to set timer");

        let mut trap = TrapFrame::default();
        SCHEDULER.critical(|s| {
            s.idle(&mut trap)
        });
        let tf = &mut trap as *mut TrapFrame;
        unsafe {
            asm!("mov rsp, {0}", in(reg) tf);
            restore_context_wrapper();
        }
    }

    /// Initializes the scheduler and add userspace processes to the Scheduler.
    pub unsafe fn initialize(&self) {
        self.0.lock().replace(Scheduler::new());
    }
}

/// Idle Process Code
#[no_mangle]
pub extern fn idle_process() {
    loop {
        x86_64::instructions::interrupts::enable_interrupts_and_hlt();
    }
}


/// Internal scheduler struct which is not thread-safe.
pub struct Scheduler {
    pub processes: VecDeque<Process>,
    last_id: Option<Id>,
    pub cpus: Processors,
}

impl Scheduler {
    /// Returns a new `Scheduler` with an empty queue.
    fn new() -> Scheduler {
        Scheduler {
            processes: VecDeque::new(),
            last_id: None,
            cpus: Default::default(),
        }
    }

    /// Adds a process to the scheduler's queue and returns that process's ID if
    /// a new process can be scheduled. The process ID is newly allocated for
    /// the process and saved in its `trap_frame`. If no further processes can
    /// be scheduled, returns `None`.
    ///
    /// It is the caller's responsibility to ensure that the first time `switch`
    /// is called, that process is executing on the CPU.
    fn add(&mut self, mut process: Process) -> Option<Id> {
        let next_pid = match self.last_id {
            Some(id) => {
                if id < core::u64::MAX {
                    Some(id + 1)
                } else {
                    None
                }
            }
            None => {
                Some(69)
            }
        };
        match next_pid {
            Some(pid) => {
                process.pid = pid;
                self.processes.push_back(process);
                self.last_id = Some(pid);
                Some(pid)
            }
            None => None
        }
    }

    /// Finds the currently running process, sets the current process's state
    /// to `new_state`, prepares the context switch on `tf` by saving `tf`
    /// into the current process, and push the current process back to the
    /// end of `processes` queue.
    ///
    /// If the `processes` queue is empty or there is no current process,
    /// returns `false`. Otherwise, returns `true`.
    fn schedule_out(&mut self, new_state: State, tf: &mut TrapFrame) -> bool {
        if self.processes.is_empty() || self.cpus.current_cpu().current_pid.is_none() {
            return false;
        }
        let mut running_process_idx: Option<usize> = None;
        let current_pid = self.cpus.current_cpu().current_pid.unwrap();

        for (idx, proc) in self.processes.iter().enumerate() {
            if let State::Running = proc.state {
                if proc.pid == current_pid {
                    running_process_idx = Some(idx);
                    break;
                }
            }
        }

        match running_process_idx {
            Some(idx) => {
                let mut proc = self.processes.remove(idx).expect("???");
                *proc.context = *tf;
                proc.state = new_state;
                self.processes.push_back(proc);
                self.cpus.current_cpu().current_pid = None;
                true
            },
            _ => {
                panic!("CPU should be running something can't find the process in the list?");
            }
        }
    }

    /// Finds the next process to switch to, brings the next process to the
    /// front of the `processes` queue, changes the next process's state to
    /// `Running`, and performs context switch by restoring the next process`s
    /// trap frame into `tf`.
    ///
    /// If there is no process to switch to, returns `None`. Otherwise, returns
    /// `Some` of the next process`s process ID.
    fn switch_to(&mut self, tf: &mut TrapFrame) -> Option<Id> {
        // debug!("Switch to on core {}", self.cpus.current_cpu().apic_id);
        for i in 0..self.processes.len() {
            let ready = self.processes[i].ready();
            if ready {
                let mut proc = self.processes.remove(i).expect("something");
                proc.state = Running;
                let pid = proc.pid;
                self.cpus.current_cpu().current_pid = Some(pid);
                *tf = *proc.context;
                self.processes.push_front(proc);
                return Some(pid);
            }
        }
        None
    }

    fn idle(&mut self, tf: &mut TrapFrame) {
        let task = &mut self.cpus.current_cpu().idle_task;
        *task.context = TrapFrame::default();
        task.context.rip = idle_process as u64;
        task.context.rsp = task.stack.as_ref().expect("").top().as_u64();
        *tf = *task.context;
    }

    /// Kills currently running process by scheduling out the current process
    /// as `Dead` state. Releases all process resources held by the process,
    /// removes the dead process from the queue, drops the dead process's
    /// instance, and returns the dead process's process ID.
    fn kill(&mut self, tf: &mut TrapFrame) -> Option<Id> {
        if self.processes.is_empty() {
            return None;
        }
        match self.processes.front().unwrap().state {
            Running => {
                if self.schedule_out(State::Dead, tf) {
                    let proc = self.processes.pop_back().expect("alskdjf");
                    return Some(proc.pid);
                }
            }
            _ => {}
        }
        None
    }

    // pub fn find_process(&mut self, tf: &TrapFrame) -> &mut Process {
    //     for i in 0..self.processes.len() {
    //         // if self.processes[i].context.tpidr == tf.tpidr {
    //         return &mut self.processes[i];
    //         // }
    //     }
    //     panic!("Invalid TrapFrame");
    // }
}

impl fmt::Debug for Scheduler {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let len = self.processes.len();
        write!(f, "  [Scheduler] {} processes in the queue\n", len)?;
        for i in 0..len {
            write!(
                f,
                "    queue[{}]: proc({:3})-{:?} \n",
                i, self.processes[i].pid, self.processes[i].state
            )?;
        }
        Ok(())
    }
}

