use volatile::{Volatile, WriteOnly};

#[repr(C)]
pub struct InterrupterRegisters {
    /// Interrupt Enable | Int Pending
    pub flags: Volatile<u8>,
    _res0: u8,
    _res1: u16,
    pub moderation_interval: Volatile<u16>,
    pub moderation_counter: Volatile<u16>,
    _res2: u32,
    pub event_ring_seg_table_ptr: Volatile<u64>,
    ///  Busy(3) | (2:0)index
    pub event_ring_deque_ptr: Volatile<u64>,
}

impl InterrupterRegisters {
    pub fn pending(&self) -> bool {
        self.flags.read() & 0x1 == 1
    }
}

#[repr(C)]
pub struct DoorBellRegister {
    pub reg: WriteOnly<u32>,
}