use core::fmt::{Debug, Formatter};
use x86_64::PhysAddr;

#[repr(C, align(2048))]
pub struct DeviceContextBaseAddressArray {
    array: [u8; 2048]
}
const_assert_size!(DeviceContextBaseAddressArray, 2048);

impl Default for DeviceContextBaseAddressArray {
    fn default() -> Self {
        DeviceContextBaseAddressArray {
            array: [0u8; 2048],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct EventRingSegmentEntry {
    pub addr: PhysAddr,
    pub segment_size: u16,
    _res0: u16,
    _res1: u32,
}
const_assert_size!(EventRingSegmentEntry, 16);

impl Default for EventRingSegmentEntry {
    fn default() -> Self {
        Self {
            addr: PhysAddr::new(0),
            segment_size: 0,
            _res0: 0,
            _res1: 0
        }
    }
}

impl EventRingSegmentEntry {
    pub fn new(ptr: PhysAddr, size: u16) -> EventRingSegmentEntry {
        EventRingSegmentEntry {
            addr: ptr,
            segment_size: size,
            _res0: 0,
            _res1: 1,
        }
    }
}

#[repr(C, align(64))]
pub struct EventRingSegmentTable {
    pub segments: [EventRingSegmentEntry; 16],
}

impl Default for EventRingSegmentTable {
    fn default() -> Self {
        EventRingSegmentTable {
            segments: [Default::default(); 16],
        }
    }
}

#[repr(C, align(4096))]
pub struct XHCIRingSegment {
    pub trbs: [ControlTRB; 256],
}
const_assert_size!(XHCIRingSegment, 4096);

impl Default for XHCIRingSegment {
    fn default() -> Self {
        XHCIRingSegment {
            trbs: [Default::default(); 256],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct NormalTRB {
    pub data_block: u64,
    pub interrupter_td_trblen: u32,
    pub meta: u32,
}

impl Debug for NormalTRB {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let type_id = (self.meta >> 10) & 0b111111;
        write!(f, "NormalTRB {{ type: {} }}", type_id)
    }
}

impl NormalTRB {
    pub fn new_noop() -> NormalTRB {
        NormalTRB {
            data_block: 0,
            interrupter_td_trblen: 0,
            meta: 23 << 10 | 0x1,
        }
    }

    pub fn end_queue() -> NormalTRB {
        NormalTRB {
            data_block: 0,
            interrupter_td_trblen: 0,
            meta: 0x1 << 0
        }
    }

    // pub fn set_cycle_bit(&mut self, val: bool) {
    //     if val {
    //         self.meta |= 0x1 << 0;
    //     } else {
    //         self.meta &= !(0x1 << 0);
    //     }
    // }
}

#[repr(C)]
#[derive(Default, Copy, Clone)]
pub struct SetupStageTRB {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
    /// Interrupt Target: (31:22), TRBLength (16:0)
    int_target_trb_length: u32,
    /// Transfer Type: (17:16)
    /// > 0: No Data, 1: Reserved, 2: Out Data, 3: In Data
    /// Transfer Type (15:10)
    /// > Should be set to Setup Stage TRB Type (XHCI Table 6-91)
    /// immData (6:6)
    /// Interrupt On Complete (5:5)
    /// Cycle Bit (0:0)
    metadata: u32
}
const_assert_size!(SetupStageTRB, 16);

#[repr(C)]
#[derive(Copy, Clone)]
pub union ControlTRB {
    setup: SetupStageTRB,
    pub normal: NormalTRB,
}
const_assert_size!(ControlTRB, 16);

impl From<NormalTRB> for ControlTRB {
    fn from(n: NormalTRB) -> Self {
        ControlTRB {
            normal: n,
        }
    }
}

impl ControlTRB {
    pub fn active(&self) -> bool {
        unsafe {self.normal}.meta & 0x1 == 1
    }

    // pub fn get_type(&self) -> u8 {
    //
    // }
}

impl Default for ControlTRB {
    fn default() -> Self {
        ControlTRB {
            setup: Default::default(),
        }
    }
}





