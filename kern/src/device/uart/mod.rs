use spin::{RwLock, Mutex};
use alloc::vec::Vec;
use alloc::boxed::Box;
use alloc::sync::Arc;

pub mod serial16650;

pub static SERIAL_PORTS: RwLock<SerialPorts> = RwLock::new(SerialPorts::new());

pub trait UART : core_io::Write + core_io::Read + core::fmt::Write {
    fn set_baudrate(&mut self, baud: u32);
    fn write_byte(&mut self, byte: u8);
    fn read_byte(&mut self) -> Option<u8>;
    fn has_data(&self) -> bool;
}

pub struct SerialPorts {
    pub ports: Vec<Arc<Mutex<dyn UART + Send + Sync>>>
}

impl SerialPorts {
    pub const fn new() -> Self {
        Self {
            ports: Vec::new()
        }
    }
}
