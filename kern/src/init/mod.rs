use core::panic::PanicInfo;
use crate::vga_buffer::CONSOLE;
use core::fmt::Write;
use x86_64::instructions::hlt;

pub mod oom;
pub mod init;
pub mod smp;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let cpuid_out: u32;
    unsafe {
        asm!(
             "mov eax, 1",
             "cpuid",
             lateout("eax") _,
             lateout("ebx") cpuid_out,
             lateout("ecx") _,
             lateout("edx") _
        );
    }
    let apic_id: u8 = (cpuid_out >> 24) as u8;

    println!("\n---------- PANIC ---------- (Core {})", apic_id);
    match info.location() {
        Some(loc) => {
            println!("File: {}\nLine: {}\nCol: {}", loc.file(), loc.line(), loc.column());
        }
        None => {}
    }
    let mut con = CONSOLE.lock();
    match info.message() {
        Some(s) => { con.write_fmt(*s).unwrap() }
        None => {}
    }
    loop {
        hlt();
    }
}
