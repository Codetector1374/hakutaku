use volatile::{Volatile, WriteOnly};
use x86_64::PhysAddr;
use crate::device::usb::xhci::consts::INT_IRQ_FLAG_INT_PENDING_MASK;

#[repr(C)]
pub struct InterrupterRegisters {
    /// Interrupt Enable | Int Pending
    pub iman: Volatile<u32>,
    pub imod: Volatile<u32>,
    pub event_ring_table_size: Volatile<u32>,
    _res3: u32,
    pub event_ring_seg_table_ptr: Volatile<PhysAddr>,
    ///  Busy(3) | (2:0)index
    pub event_ring_deque_ptr: Volatile<u64>,
}

impl InterrupterRegisters {
    pub fn pending(&self) -> bool {
        self.iman.read() & INT_IRQ_FLAG_INT_PENDING_MASK != 0
    }
}

#[repr(C)]
pub struct DoorBellRegister {
    pub reg: WriteOnly<u32>,
}