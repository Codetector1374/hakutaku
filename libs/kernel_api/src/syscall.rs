use core::time::Duration;

use crate::*;

macro_rules! err_or {
    ($ecode:expr, $rtn:expr) => {{
        let e = OsError::from($ecode);
        if let OsError::Ok = e {
            Ok($rtn)
        } else {
            Err(e)
        }
    }};
}

pub fn sleep(span: Duration) -> OsResult<Duration> {
    if span.as_millis() > core::u64::MAX as u128 {
        panic!("too big!");
    }

    let ms = span.as_millis() as u64;
    let mut ecode: u64;
    let mut elapsed_ms: u64;

    unsafe {
        asm!("mov rax, {syscall_num}",
             "mov rdi, {0}",
             "int 0x80",
             "mov {1}, rdx",
             "mov {2}, rax",
             in(reg) ms,
             lateout(reg) elapsed_ms,
             lateout(reg) ecode,
             syscall_num = const NR_SLEEP,
             lateout("rax") _,
             lateout("rdi") _,
             lateout("rdx") _,
             );
    }

    err_or!(ecode, Duration::from_millis(elapsed_ms))
}


// struct Console;
//
// impl fmt::Write for Console {
//     fn write_str(&mut self, s: &str) -> fmt::Result {
//         for b in s.bytes() {
//             write(b);
//         }
//         Ok(())
//     }
// }
//
// #[macro_export]
// macro_rules! print {
//     ($($arg:tt)*) => ($crate::syscall::vprint(format_args!($($arg)*)));
// }
//
// #[macro_export]
// macro_rules! println {
//  () => (print!("\n"));
//     ($($arg:tt)*) => ({
//         $crate::syscall::vprint(format_args!($($arg)*));
//         $crate::print!("\n");
//     })
// }
//
// pub fn vprint(args: fmt::Arguments) {
//     let mut c = Console;
//     c.write_fmt(args).unwrap();
// }
