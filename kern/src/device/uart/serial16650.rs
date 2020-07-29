use x86_64::instructions::port::Port;
use x86_64::structures::paging::OffsetPageTable;
use x86_64::VirtAddr;

use crate::device::uart::serial16650::Serial16650Base::{MMIO, PMIO};
use crate::device::uart::UART;

const OFFSET_DATA: u16 = 0;
const OFFSET_INTEN: u16 = 1;
const OFFSET_DIV_LB: u16 = 0;
const OFFSET_DIV_HB: u16 = 1;
const OFFSET_FCR: u16 = 2;
const OFFSET_LCR: u16 = 3;
const OFFSET_MCR: u16 = 4;
const OFFSET_LSR: u16 = 5;
const OFFSET_MSR: u16 = 6;
const OFFSET_SRATCH: u16 = 7;

pub const COM1_BASE_ADDR: u16 = 0x3F8;

enum Serial16650Base {
    MMIO(VirtAddr),
    PMIO(u16),
}

pub struct Serial16650 {
    base_addr: Serial16650Base
}

impl Serial16650 {
    pub fn new_from_port(base: u16) -> Self {
        Self {
            base_addr: PMIO(base)
        }
    }

    pub fn new_from_mmio(mmio_base: VirtAddr) -> Self {
        Self {
            base_addr: MMIO(mmio_base)
        }
    }

    pub fn verify(&mut self) -> bool {
        let value = 0xAA;
        match self.base_addr {
            PMIO(base) => {
                unsafe {
                    let mut scratch_port: Port<u8> = x86_64::instructions::port::Port::new(base + OFFSET_SRATCH);
                    scratch_port.write(value);
                    value == scratch_port.read()
                }
            }
            MMIO(base) => {
                unsafe {
                    let mut scratch_port: *mut u8 =  (base + OFFSET_SRATCH as u64).as_mut_ptr();
                    scratch_port.write_volatile(value);
                    value == scratch_port.read_volatile()
                }
            }
        }
    }
}

impl UART for Serial16650 {
    fn set_baudrate(&mut self, baud: u32) {}

    fn write(&mut self, byte: u8) {
        match self.base_addr {
            PMIO(base) => {
                unsafe { x86_64::instructions::port::Port::new(base + OFFSET_DATA).write(byte) }
            }
            MMIO(vabase) => {
                unsafe { ((vabase + (OFFSET_DATA as u64)).as_u64() as *mut u8).write_volatile(byte); }
            }
        }
    }
}
