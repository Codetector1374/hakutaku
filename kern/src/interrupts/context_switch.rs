use x86_64::structures::idt::InterruptStackFrame;
use crate::hardware::apic::GLOBAL_APIC;

#[repr(C)]
#[derive(Debug)]
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
    pub rflags: u64,
    pub rsp: u64,
    pub ss: u64,
}

extern "x86-interrupt" {
    pub fn save_context(_:&mut InterruptStackFrame);
}

#[no_mangle]
pub extern "C" fn handle_context_switch(tf: &mut TrapFrame) {
    GLOBAL_APIC.lock().end_of_interrupt();
    println!("Context Switching");
}