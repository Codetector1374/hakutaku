use x86_64::structures::idt::InterruptStackFrame;
use spin::MutexGuard;
use crate::interrupts::{InterruptIndex, PICS};

pub(super) extern "x86-interrupt" fn keyboard_interrupt_handler(_stack_frame: &mut InterruptStackFrame) {
    use pc_keyboard::{layouts, DecodedKey, HandleControl, Keyboard, ScancodeSet1};
    use spin::Mutex;
    use x86_64::instructions::port::Port;

    lazy_static! {
        static ref KEYBOARD: Mutex<Keyboard<layouts::Us104Key, ScancodeSet1>> =
            Mutex::new(Keyboard::new(layouts::Us104Key, ScancodeSet1,
                HandleControl::MapLettersToUnicode)
            );
    }


    let mut ps2_port = Port::new(0x60); // PS2 Port
    let mut keyboard: MutexGuard<Keyboard<layouts::Us104Key, ScancodeSet1>> = KEYBOARD.lock();
    let scancode: u8 = unsafe { ps2_port.read() };

    if let Ok(Some(key_evnt)) = keyboard.add_byte(scancode) {
        if let Some(key) = keyboard.process_keyevent(key_evnt) {
            match key {
                DecodedKey::Unicode(c) => crate::hardware::keyboard::put_char(c),
                _ => {}
                // DecodedKey::RawKey(key) => println!("raw: {:?}", key),
            }
        }
    }
    unsafe {
        PICS.lock().notify_end_of_interrupt(InterruptIndex::Keyboard.as_u8());
    }
}
