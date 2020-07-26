use core::ptr::Unique;
use volatile::Volatile;
use spin::Mutex;

#[allow(dead_code)]
#[repr(u8)]
pub enum Color {
    Black      = 0,
    Blue       = 1,
    Green      = 2,
    Cyan       = 3,
    Red        = 4,
    Magenta    = 5,
    Brown      = 6,
    LightGray  = 7,
    DarkGray   = 8,
    LightBlue  = 9,
    LightGreen = 10,
    LightCyan  = 11,
    LightRed   = 12,
    Pink       = 13,
    Yellow     = 14,
    White      = 15,
}

#[derive(Debug, Clone, Copy)]
pub struct ColorCode(u8);
impl ColorCode {
    pub const fn new(foreground: Color, background: Color) -> ColorCode {
        ColorCode((background as u8) << 4 | (foreground as u8))
    }
}


#[repr(C)]
#[derive(Copy, Clone)]
struct ScreenChar {
    ascii_character: u8,
    color_code: ColorCode,
}

const BUFFER_HEIGHT: usize = 25;
const BUFFER_WIDTH: usize = 80;

struct Buffer {
    chars: [[Volatile<ScreenChar>; BUFFER_WIDTH]; BUFFER_HEIGHT],
}

pub struct VGATextBuffer {
    column_position: usize,
    row_position: usize,
    color_code: ColorCode,
    buffer: Unique<Buffer>,
}

impl VGATextBuffer {
    pub fn write_byte(&mut self, byte: u8) {
        match byte {
            b'\n' => self.new_line(),
            32..=126 => {
                if self.column_position >= BUFFER_WIDTH {
                    self.new_line();
                }

                let row = self.row_position;
                let col = self.column_position;

                let color_code = self.color_code;
                self.buffer().chars[row][col].write(ScreenChar {
                    ascii_character: byte,
                    color_code,
                });
                self.column_position += 1;
            },
            8 => { // DEL
                if self.column_position > 0 {
                    self.column_position -= 1;
                } else if self.row_position > 0 {
                    self.row_position -= 1;
                    self.column_position = BUFFER_WIDTH - 1;
                }
            },
            _ => {}
        }
    }

    fn buffer(&mut self) -> &mut Buffer {
        unsafe{ self.buffer.as_mut() }
    }

    fn new_line(&mut self) {
        if self.row_position == BUFFER_HEIGHT - 1 {
            for row in 1..BUFFER_HEIGHT {
                for col in 0..BUFFER_WIDTH {
                    let buffer = self.buffer();
                    let character = buffer.chars[row][col].read();
                    buffer.chars[row - 1][col].write(character);
                }
            }
            self.clear_row(BUFFER_HEIGHT-1);
        } else {
            self.row_position += 1;
        }
        self.column_position = 0;
    }

    fn clear_row(&mut self, row: usize) {
        let blank = ScreenChar {
            ascii_character: b' ',
            color_code: self.color_code,
        };
        for col in 0..BUFFER_WIDTH {
            self.buffer().chars[row][col].write(blank);
        }
    }

    pub fn write_str(&mut self, s: &str) {
        for byte in s.bytes() {
            self.write_byte(byte)
        }
    }
}

use core::fmt;
use crate::memory::paging::{KERNEL_TEXT_BASE};

impl fmt::Write for VGATextBuffer {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_str(s);
        Ok(())
    }
}

pub static CONSOLE: Mutex<VGATextBuffer> = Mutex::new(VGATextBuffer {
    column_position: 0,
    row_position: BUFFER_HEIGHT - 1,
    color_code: ColorCode::new(Color::White, Color::Black),
    buffer: unsafe { Unique::new_unchecked((0xb8000 + KERNEL_TEXT_BASE) as *mut _) }
});

pub fn print(args: fmt::Arguments) {
    use core::fmt::Write;
    // Very sad LOL
    x86_64::instructions::interrupts::without_interrupts(|| {
        CONSOLE.lock().write_fmt(args).unwrap();
    });
}

pub fn disable_cursor() {
    unsafe {
        asm!(
        "mov dx, 0x3D4",
    	"mov al, 0xA",
	    "out dx, al",
	    "inc dx",
	    "mov al, 0x20",
	    "out dx, al",

	    lateout("rax") _,
	    lateout("rdx") _);
    }
}

macro_rules! println {
    ($fmt:expr) => (print!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (print!(concat!($fmt, "\n"), $($arg)*));
}

macro_rules! print {
    ($($arg:tt)*) => ({
        $crate::vga_buffer::print(format_args!($($arg)*));
    });
}