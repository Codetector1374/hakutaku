use spin::{RwLock, Mutex};
use alloc::vec::Vec;
use alloc::boxed::Box;
use alloc::sync::Arc;

pub mod serial16650;

pub static SERIAL_PORTS: RwLock<SerialPorts> = RwLock::new(SerialPorts::new());

pub trait UART {
    fn set_baudrate(&mut self, baud: u32);
    fn write(&mut self, byte: u8);
}

pub struct SerialPorts {
    ports: Vec<Arc<Mutex<dyn UART + Send + Sync>>>
}

impl SerialPorts {
    pub const fn new() -> Self {
        Self {
            ports: Vec::new()
        }
    }
}
