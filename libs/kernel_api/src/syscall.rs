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
        asm!("mov rax, $3
              mov rdi, $2
              int 0x80
              mov $0, rdx
              mov $1, rax"
             : "=r"(elapsed_ms), "=r"(ecode)
             : "r"(ms), "i"(NR_SLEEP)
             : "rax", "rdx", "rdi"
             : "volatile", "intel");
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
