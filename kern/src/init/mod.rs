use core::panic::PanicInfo;
use crate::vga_buffer::CONSOLE;
use core::fmt::Write;

pub mod oom;
pub mod init;
pub mod smp;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("\n---------- PANIC ----------");
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
    loop {}
}
