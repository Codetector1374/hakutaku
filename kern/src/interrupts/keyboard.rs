use pc_keyboard::{DecodedKey, HandleControl, Keyboard, layouts, ScancodeSet1};
use spin::Mutex;
use spin::MutexGuard;

use x86_64::structures::idt::InterruptStackFrame;

use crate::interrupts::{InterruptIndex, PICS};
use crate::sys::apic::GLOBAL_APIC;
use crate::sys::keyboard::GLOB_KEYBOARD;

lazy_static! {
    static ref KEYBOARD: Mutex<Keyboard<layouts::Us104Key, ScancodeSet1>> =
        Mutex::new(Keyboard::new(layouts::Us104Key, ScancodeSet1,
            HandleControl::MapLettersToUnicode)
    );
}

pub(super) extern "x86-interrupt" fn keyboard_interrupt_handler(_stack_frame: &mut InterruptStackFrame) {
    GLOB_KEYBOARD.process_key_stroke();
    unsafe {
        PICS.lock().notify_end_of_interrupt(InterruptIndex::Keyboard.as_u8());
    }
}
