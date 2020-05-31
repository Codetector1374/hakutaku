use volatile::{Volatile, WriteOnly};
use x86_64::PhysAddr;

#[repr(C)]
pub struct InterrupterRegisters {
    /// Interrupt Enable | Int Pending
    pub irq_flags: Volatile<u32>,
    pub irq_control: Volatile<u32>,
    pub event_ring_table_size: Volatile<u32>,
    _res3: u32,
    pub event_ring_seg_table_ptr: Volatile<PhysAddr>,
    ///  Busy(3) | (2:0)index
    pub event_ring_deque_ptr: Volatile<u64>,
}

impl InterrupterRegisters {
    pub fn pending(&self) -> bool {
        self.irq_flags.read() & 0x1 == 1
    }
}

#[repr(C)]
pub struct DoorBellRegister {
    pub reg: WriteOnly<u32>,
}