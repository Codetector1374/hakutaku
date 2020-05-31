use core::fmt::{Debug, Formatter};
use x86_64::{PhysAddr, VirtAddr};
use alloc::vec::Vec;
use alloc::boxed::Box;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::MapperAllSizes;
use core::ops::Deref;
use crate::device::usb::xhci::consts::{TRB_COMMON_TYPE_SHIFT, TRB_TYPE_LINK, TRB_COMMON_TYPE_MASK, TRB_LINK_TOGGLE_MASK, TRBS_PER_SEGMENT, TRB_COMMON_CYCLE_STATE_MASK, TRB_TYPE_NOOP_COMMAND};

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
    pub segment_size: u32,
    _res1: u32,
}
const_assert_size!(EventRingSegmentEntry, 16);

impl Default for EventRingSegmentEntry {
    fn default() -> Self {
        Self {
            addr: PhysAddr::new(0),
            segment_size: 0,
            _res1: 0,
        }
    }
}

impl EventRingSegmentEntry {
    pub fn new(ptr: PhysAddr, size: u32) -> EventRingSegmentEntry {
        EventRingSegmentEntry {
            addr: ptr,
            segment_size: size,
            _res1: 0,
        }
    }
}

#[repr(C, align(64))]
#[derive(Default)]
pub struct EventRingSegmentTable {
    pub segments: [EventRingSegmentEntry; 16],
    pub segment_count: usize,
}

/* ----------------------- XHCI Ring ------------------------ */
pub struct XHCIRing {
    pub segments: Vec<Box<XHCIRingSegment>>,
    pub enqueue: (usize, usize),
    pub dequeue: (usize, usize),
    pub cycle_state: u32,
}

impl XHCIRing {
    pub fn new_with_capacity(segments: usize, link_trbs: bool) -> XHCIRing {
        let mut ring = XHCIRing {
            segments: vec![],
            enqueue: (0, 0),
            dequeue: (0, 0),
            /*
             * The ring is initialized to 0. The producer must write 1 to the
             * cycle bit to handover ownership of the TRB, so PCS = 1.
             * The consumer must compare CCS to the cycle bit to
             * check ownership, so CCS = 1.
             */
            cycle_state: 1, // Ring is initialized to 0, thus cycle state = 1
        };
        for idx in 0..segments {
            ring.segments.push(Box::new(XHCIRingSegment::default()));
            if link_trbs {
                if idx > 0 {
                    let ptr = VirtAddr::from_ptr(ring.segments[idx].deref() as *const XHCIRingSegment);
                    let ptr_pa = without_interrupts(|| {
                        use crate::PAGE_TABLE;
                        PAGE_TABLE.read().translate_addr(ptr).expect("va pa translate")
                    });
                    ring.segments[idx - 1].link_segment(ptr_pa);
                }
                if idx == segments - 1 {
                    let ptr = VirtAddr::from_ptr(ring.segments[0].deref() as *const XHCIRingSegment);
                    let ptr_pa = without_interrupts(|| {
                        use crate::PAGE_TABLE;
                        PAGE_TABLE.read().translate_addr(ptr).expect("va pa translate")
                    });
                    ring.segments[idx].link_segment(ptr_pa);
                    unsafe { ring.segments[idx].trbs[TRBS_PER_SEGMENT - 1].link.link_flags |= TRB_LINK_TOGGLE_MASK };
                }
            }
        }
        ring
    }

    pub fn push(&mut self, mut trb: TRB) {
        assert_ne!(self.segments.len(), 0, "no segments");
        trb.set_cycle_state(self.cycle_state as u8);
        self.segments[self.enqueue.0].trbs[self.enqueue.1] = trb;
        if self.enqueue.1 < (TRBS_PER_SEGMENT - 2) { // Last element is Link
            self.enqueue.1 += 1;
        } else {
            self.enqueue.1 = 0;
            if self.enqueue.0 < self.segments.len() - 1 {
                self.enqueue.0 += 1;
            } else {
                self.enqueue.0 = 0;
            }
        }
    }

    pub fn pop(&mut self, has_link: bool) -> TRB {
        let trb = self.segments[self.dequeue.0].trbs[self.dequeue.1].clone();
        if self.dequeue.1 < (TRBS_PER_SEGMENT - (if has_link { 2 } else { 1 })) {
            self.dequeue.1 += 1;
        } else {
            self.dequeue.1 = 0;
            if self.dequeue.0 < self.segments.len() - 1 {
                self.dequeue.0 += 1;
            } else {
                self.dequeue.0 = 0;
            }
        }
        trb
    }

    pub fn dequeue_pointer(&self) -> PhysAddr {
        without_interrupts(|| {
            crate::PAGE_TABLE.read().translate_addr(
                VirtAddr::from_ptr(
                    &self.segments[self.dequeue.0].trbs[self.dequeue.1] as *const TRB
                )
            ).expect("PT Translation Error")
        })
    }
}

impl Debug for XHCIRing {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "XHCIRing {{ enq: {:?}, deq: {:?}, cycle_state: {}, num_segs: {} }}",
               self.enqueue, self.dequeue, self.cycle_state, self.segments.len())
    }
}

/* --------------------- XHCI Ring Segment ---------------- */
#[repr(C, align(4096))]
pub struct XHCIRingSegment {
    pub trbs: [TRB; TRBS_PER_SEGMENT],
}
const_assert_size!(XHCIRingSegment, 16*TRBS_PER_SEGMENT);

impl Default for XHCIRingSegment {
    fn default() -> Self {
        XHCIRingSegment {
            trbs: [Default::default(); TRBS_PER_SEGMENT],
        }
    }
}

impl XHCIRingSegment {
    /// Make the prev segment point to the next segment.
    /// Change the last TRB in the prev segment to be a Link TRB which points to the
    /// address of the next segment.  The caller needs to set any Link TRB
    /// related flags, such as End TRB, Toggle Cycle, and no snoop.
    pub fn link_segment(&mut self, other: PhysAddr) {
        let mut link_trb = LinkTRB::new(other);
        self.trbs[TRBS_PER_SEGMENT - 1] = TRB { link: link_trb };
    }
}

/* --------------------------- TRBs --------------------------- */

#[repr(C)]
#[derive(Copy, Clone)]
pub union TRB {
    pub normal: NormalTRB,
    pub setup: SetupStageTRB,
    pub link: LinkTRB,
    pseudo: PseudoTRB,
}
const_assert_size!(TRB, 16);

impl TRB {
    pub fn as_link_mut(&mut self) -> Option<&LinkTRB> {
        if self.type_id() == TRB_TYPE_LINK {
            Some(unsafe { &mut self.link })
        } else {
            None
        }
    }
    pub fn set_cycle_state(&mut self, val: u8) {
        let mut tmp = unsafe { self.pseudo.flags };
        tmp &= !TRB_COMMON_CYCLE_STATE_MASK;
        tmp |= (val as u16) & TRB_COMMON_CYCLE_STATE_MASK;
        self.pseudo.flags = tmp
    }
}

impl From<NormalTRB> for TRB {
    fn from(n: NormalTRB) -> Self {
        TRB {
            normal: n,
        }
    }
}

impl TRB {
    pub fn active(&self) -> bool {
        unsafe { self.normal }.meta & 0x1 == 1
    }

    pub fn type_id(&self) -> u16 {
        (unsafe { self.pseudo.flags } & TRB_COMMON_TYPE_MASK) >> TRB_COMMON_TYPE_SHIFT
    }
}

impl Default for TRB {
    fn default() -> Self {
        TRB {
            pseudo: Default::default(),
        }
    }
}

/* ------------ Pseudo TRB -------- */
#[repr(C)]
#[derive(Copy, Clone)]
pub struct PseudoTRB {
    _res0: [u32; 3],
    flags: u16,
    _res1: u16,
}
const_assert_size!(PseudoTRB, 16);

impl Default for PseudoTRB {
    fn default() -> Self {
        unsafe {
            core::mem::zeroed()
        }
    }
}

/* -------- Link TRB -------------- */
#[repr(C)]
#[derive(Copy, Clone)]
pub struct LinkTRB {
    pub next_trb: PhysAddr,
    _res0: [u8; 3],
    pub int_target: u8,
    /// refer to Section 6.4.4.1
    pub link_flags: u16,
    _res1: u16,
}
const_assert_size!(LinkTRB, 16);

impl LinkTRB {
    pub fn new(link: PhysAddr) -> LinkTRB {
        LinkTRB {
            next_trb: link,
            _res0: [0u8; 3],
            int_target: 0,
            link_flags: ((TRB_TYPE_LINK as u16) << TRB_COMMON_TYPE_SHIFT),
            _res1: 0,
        }
    }
}

/* -------- Normal TRB ------------ */
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
            meta: (TRB_TYPE_NOOP_COMMAND as u32) << TRB_COMMON_TYPE_SHIFT,
        }
    }
}

/* ------- Setup TRB ------------- */

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
    metadata: u32,
}
const_assert_size!(SetupStageTRB, 16);






