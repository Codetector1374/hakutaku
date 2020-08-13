//! Syscall Module
use crate::interrupts::context_switch::TrapFrame;
use core::time::Duration;
use crate::process::state::State;
use alloc::boxed::Box;
use crate::process::process::Process;
use crate::hardware::pit::PIT;
use crate::SCHEDULER;
use kernel_api::OsError;
use kernel_api::*;

// Syscall Calling Convention
// int 0x80
// rax: syscall number
// Argument are passed in SystemV ABI
// Status is returned in rax, return value in rdx

pub fn handle_syscall(tf: &mut TrapFrame) {
    trace!("syscall: {}", tf.rax);
    match tf.rax {
        NR_SLEEP => {
            sys_sleep(tf);
        },
        _ => {
            warn!("Unknown syscall with id: {}", tf.rax);
            tf.rax = 0;
        }
    }
}

/// ms in rax
pub fn sys_sleep(tf: &mut TrapFrame) {
    let start = PIT::current_time();
    let delta = Duration::from_millis(tf.rdi);
    let target = start + delta;
    debug!("handling sleep syscall: {:?}", delta);
    let wait = State::Waiting(Box::new(move |p: &mut Process| -> bool {
        let t = PIT::current_time();
        if t >= target {
            p.context.rdx = (t - start).as_millis() as u64;
            p.context.rax = OsError::Ok as u64;
            true
        } else {
            false
        }
    }));
    SCHEDULER.switch(wait, tf);
}
