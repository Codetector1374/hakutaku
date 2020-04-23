use stack_vec::StackVec;
use alloc::collections::VecDeque;
use spin::Mutex;

pub static GLOB_KEYBOARD: Mutex<X86Keyboard> = Mutex::new(X86Keyboard::uninitialized());

pub struct X86Keyboard {
    key_buffer: Option<VecDeque<u8>>,
}

impl X86Keyboard {
    const fn uninitialized() -> X86Keyboard {
        X86Keyboard {
            key_buffer: None
        }
    }

    fn initialize(&mut self) {
        self.key_buffer = Some(VecDeque::with_capacity(16));
    }

    fn push_keystroke(&mut self, key: u8) {
        self.key_buffer.as_mut().expect("Uninitialized").push_back(key);
    }

    fn available(&self) -> usize {
        self.key_buffer.as_ref().expect("Uninitialized").len()
    }

    fn read(&mut self) -> Option<u8> {
        self.key_buffer.as_mut().expect("Uninitialized").pop_front()
    }
}

pub fn initialize() {
    x86_64::instructions::interrupts::without_interrupts(|| {
        GLOB_KEYBOARD.lock().initialize();
    });
}

pub fn put_char(char: char) {
    x86_64::instructions::interrupts::without_interrupts(|| {
        GLOB_KEYBOARD.lock().push_keystroke(char as u8);
    });
}

pub fn blocking_get_char() -> u8 {
    loop {
        let read_result = x86_64::instructions::interrupts::without_interrupts(|| {
            GLOB_KEYBOARD.lock().read()
        });
        match read_result {
            Some(c) => {
                return c;
            }
            None => {}
        }
        x86_64::instructions::hlt();
    }
}