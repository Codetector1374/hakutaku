use stack_vec::StackVec;
use alloc::collections::VecDeque;
use spin::Mutex;
use crate::sys::stdin::{StandardInput, STD_IN};
use x86_64::instructions::port::Port;
use pc_keyboard::{Keyboard, ScancodeSet1, HandleControl, DecodedKey};
use pc_keyboard::layouts::Us104Key;

lazy_static! {
    pub static ref GLOB_KEYBOARD: X86Keyboard = X86Keyboard {
        port: Port::new(0x60),
        state: Mutex::new(Keyboard::new(Us104Key, ScancodeSet1,
                                                      HandleControl::MapLettersToUnicode)),
    };
}

pub struct X86Keyboard {
    port: Port<u8>,
    state: Mutex<Keyboard<Us104Key, ScancodeSet1>>,
}

impl X86Keyboard {
    pub fn process_key_stroke(&self) {
        let scancode = unsafe { (*(self as *const Self as *mut Self)).port.read() };
        let mut keyboard = GLOB_KEYBOARD.state.lock();
        if let Ok(Some(key_evnt)) = keyboard.add_byte(scancode) {
            if let Some(key) = keyboard.process_keyevent(key_evnt) {
                match key {
                    DecodedKey::Unicode(c) => STD_IN.insert(c as u8),
                    _ => {}
                    // DecodedKey::RawKey(key) => println!("raw: {:?}", key),
                }
            }
        }
    }
}
