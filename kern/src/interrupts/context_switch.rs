use x86_64::structures::idt::InterruptStackFrame;
use crate::hardware::apic::GLOBAL_APIC;
use x86_64::registers::rflags::RFlags;
use crate::SCHEDULER;
use crate::process::state::State::Ready;
use crate::interrupts::syscall::handle_syscall;

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum InterruptSource {
    APICTimer = 0x1,
    SysCall = 0x80
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct TrapFrame {
    pub rax: u64,
    pub rcx: u64,
    pub rdx: u64,
    pub rbx: u64,
    pub rsi: u64,
    pub rdi: u64,
    pub rbp: u64,
    pub r8: u64,
    pub r9: u64,
    pub r10: u64,
    pub r11: u64,
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
    pub rip: u64,
    pub cs: u64,
    pub rflags: RFlags,
    pub rsp: u64,
    pub ss: u64,
}

impl Default for TrapFrame {
    fn default() -> Self {
        TrapFrame {
            rax: 0,
            rcx: 0,
            rdx: 0,
            rbx: 0,
            rsi: 0,
            rdi: 0,
            rbp: 0,
            r8: 0,
            r9: 0,
            r10: 0,
            r11: 0,
            r12: 0,
            r13: 0,
            r14: 0,
            r15: 0,
            rip: 0,
            cs: 8,
            rflags: RFlags::INTERRUPT_FLAG,
            rsp: x86_64::registers::read_rsp(),
            ss: 0
        }
    }
}

extern "C" {
    pub fn apic_timer();
    pub fn syscall_handler();
    pub fn restore_context_wrapper() -> !;
}

#[no_mangle]
pub extern "C" fn handle_context_switch(tf: &mut TrapFrame, cause: InterruptSource) {
    match cause {
        InterruptSource::APICTimer => {
            SCHEDULER.switch(Ready, tf);
        },
        InterruptSource::SysCall => {
            handle_syscall(tf);
        },
        // _ => {}
    }
    GLOBAL_APIC.lock().end_of_interrupt();
}