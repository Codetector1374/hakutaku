use core::fmt::{Debug, Formatter};
use x86_64::{PhysAddr, VirtAddr};
use alloc::vec::Vec;
use alloc::boxed::Box;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::MapperAllSizes;
use core::ops::Deref;
use crate::device::usb::xhci::consts::*;
use alloc::alloc::Global;
use core::alloc::{AllocRef, Layout, AllocInit};
use core::ptr::NonNull;

#[repr(C, align(2048))]
pub struct DeviceContextBaseAddressArray {
    pub entries: [PhysAddr; 256]
}
const_assert_size!(DeviceContextBaseAddressArray, 2048);

impl Default for DeviceContextBaseAddressArray {
    fn default() -> Self {
        DeviceContextBaseAddressArray {
            entries: unsafe { core::mem::zeroed() },
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

    pub fn push(&mut self, mut trb: TRB) -> PhysAddr {
        assert_ne!(self.segments.len(), 0, "no segments");
        trb.set_cycle_state(self.cycle_state as u8);
        self.segments[self.enqueue.0].trbs[self.enqueue.1] = trb;
        let ptr = without_interrupts(|| {
            crate::PAGE_TABLE.read().translate_addr(
                VirtAddr::from_ptr(
                    &self.segments[self.enqueue.0].trbs[self.enqueue.1] as *const TRB
                )
            ).expect("")
        });
        if self.enqueue.1 < (TRBS_PER_SEGMENT - 2) { // Last element is Link
            self.enqueue.1 += 1;
        } else {
            if self.enqueue.0 < self.segments.len() - 1 {
                self.enqueue.0 += 1;
            } else {
                // Toggle Cycle State
                debug!("[XHCI] Toggling State");
                self.segments[self.enqueue.0].trbs[TRBS_PER_SEGMENT - 1].set_cycle_state(self.cycle_state as u8);
                self.cycle_state = if self.cycle_state == 0 { 1 } else { 0 };
                self.enqueue.0 = 0;
            }
            self.enqueue.1 = 0;
        }
        ptr
    }

    pub fn pop(&mut self, has_link: bool) -> Option<TRB> {
        // TODO: Check Cycle State
        let trb = self.segments[self.dequeue.0].trbs[self.dequeue.1].clone();
        if trb.get_cycle_state() != self.cycle_state as u8 {
            return None;
        }
        if self.dequeue.1 < (TRBS_PER_SEGMENT - (if has_link { 2 } else { 1 })) {
            self.dequeue.1 += 1;
        } else {
            self.dequeue.1 = 0;
            if self.dequeue.0 < self.segments.len() - 1 {
                self.dequeue.0 += 1;
            } else {
                self.dequeue.0 = 0;
                self.cycle_state = if self.cycle_state == 0 { 1 } else { 0 };
            }
        }
        Some(trb)
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
        let link_trb = LinkTRB::new(other);
        self.trbs[TRBS_PER_SEGMENT - 1] = TRB { link: link_trb };
    }
}

/* --------------------------- TRBs --------------------------- */
#[derive(Debug)]
pub enum TRBType {
    Unknown(TRB),
    Link(LinkTRB),
    PortStatusChange(PortStatusChangeTRB),
    CommandCompletion(CommandCompletionTRB),
    HostControllerEvent(HostControllerEventTRB),
}

impl From<TRB> for TRBType {
    fn from(t: TRB) -> Self {
        use TRBType::*;
        match t.type_id() {
            TRB_TYPE_LINK => Link(unsafe { t.link }),
            TRB_TYPE_EVNT_PORT_STATUS_CHG => PortStatusChange(unsafe { t.port_status_change }),
            TRB_TYPE_EVNT_CMD_COMPLETE => CommandCompletion(unsafe { t.command_completion }),
            TRB_TYPE_EVNT_HC => HostControllerEvent(unsafe { t.host_controller_event }),
            _ => TRBType::Unknown(t),
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub union TRB {
    pub command: CommandTRB,
    pub setup: SetupStageTRB,
    pub link: LinkTRB,
    pub port_status_change: PortStatusChangeTRB,
    pub command_completion: CommandCompletionTRB,
    pub host_controller_event: HostControllerEventTRB,
    pseudo: PseudoTRB,
}
const_assert_size!(TRB, 16);

impl Debug for TRB {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "Generic TRB with type={}", self.type_id())
    }
}

impl TRB {
    pub fn set_cycle_state(&mut self, val: u8) {
        let mut tmp = unsafe { self.pseudo.flags };
        tmp &= !TRB_COMMON_CYCLE_STATE_MASK;
        tmp |= (val as u16) & TRB_COMMON_CYCLE_STATE_MASK;
        self.pseudo.flags = tmp
    }

    pub fn get_cycle_state(&self) -> u8 {
        (unsafe { self.pseudo.flags } & TRB_COMMON_CYCLE_STATE_MASK) as u8
    }

    pub fn get_type(&self) -> u16 {
        unsafe { (self.pseudo.flags & TRB_COMMON_TYPE_MASK) >> TRB_COMMON_TYPE_SHIFT }
    }
}

impl TRB {
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
/* ------------ Event TRBs -------- */
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PortStatusChangeTRB {
    _res0: [u8; 3],
    pub port_id: u8,
    _res1: [u8; 7],
    pub completion_code: u8,
    pub flags: u16,
    _res2: u16,
}
const_assert_size!(PortStatusChangeTRB, 16);

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct HostControllerEventTRB {
    _res: [u8; 11],
    complete_code: u8,
    flags: u16,
    _res1: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct CommandCompletionTRB {
    pub trb_pointer: u64,
    pub params: [u8; 3],
    pub code: u8,
    pub flags: u16,
    pub vfid: u8,
    pub slot: u8,
}

/* ------------ Pseudo TRB -------- */
#[repr(C)]
#[derive(Copy, Clone, Debug)]
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
#[derive(Copy, Clone, Debug)]
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

/* -------- Command TRB ------------ */
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct CommandTRB {
    pub payload: [u32; 4],
}

impl Debug for CommandTRB {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let type_id = (self.payload[3] & TRB_COMMON_TYPE_MASK as u32) >> TRB_COMMON_TYPE_SHIFT as u32;
        write!(f, "TRB {{ type: {} }}", type_id)
    }
}

impl CommandTRB {
    pub fn new_noop() -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_NOOP_COMMAND as u32) << (TRB_COMMON_TYPE_SHIFT as u32);
        trb
    }

    pub fn enable_slot() -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_ENABLE_SLOT_CMD as u32) << (TRB_COMMON_TYPE_SHIFT as u32);
        trb
    }
}

impl Into<TRB> for CommandTRB {
    fn into(self) -> TRB {
        TRB { command: self }
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

/* ------------- Device Context ------------- */

#[repr(C, align(2048))]
#[derive(Default)]
pub struct DeviceContextArray {
    pub slot: SlotContext,
    pub endpoint: [EndpointContext; 31],
}

macro_rules! set_field {
    ($var: expr, $shift: expr, $mask: expr, $val: expr) => {{
        let tmp = $var & !$mask;
        $var = tmp | (($val.checked_shl($shift).unwrap_or(0)) & $mask);
    }};
}

#[repr(C)]
#[derive(Default)]
pub struct EndpointContext {
    dword1: u32,
    pub flags1: u8,
    pub max_burst_size: u8,
    pub max_packet_size: u16,
    pub dequeu_pointer: u64,
    pub average_trb_len: u16,
    max_esit_payload_lo: u16,
    _res0: [u32; 3],
}
const_assert_size!(EndpointContext, 32);
impl EndpointContext {
    pub fn set_lsa_bit(&mut self) {
        self.dword1 |= EP_CTX_LSA_MASK;
    }

    pub fn set_cerr(&mut self, val: u8) {
        set_field!(self.dword1,
            EP_CTX_CERR_SHIFT, EP_CTX_CERR_MASK,
            val as u32
        );
    }

    pub fn set_ep_type(&mut self, val: u8) {
        set_field!(self.dword1,
            EP_CTX_EPTYPE_SHIFT, EP_CTX_EPTYPE_MASK,
            val as u32
        );
    }
}

#[repr(C)]
#[derive(Default)]
pub struct SlotContext {
    // DWORD1
    pub dword1: u32,
    // DWORD2
    pub max_exit_latency: u16,
    pub root_hub_port_number: u8,
    pub numbr_ports: u8,
    // DWORD3
    pub hub_slot_id: u8,
    pub tt_port_number: u8,
    pub interrupter_ttt: u16,
    // DWORD 4
    pub device_addr: u8,
    _res0: [u8; 2],
    pub slot_state: u8,
    // DWORD 5-8
    _res1: [u32; 4],
}
const_assert_size!(SlotContext, 32);

impl SlotContext {
    pub fn set_speed(&mut self, speed: u8) {
        set_field!(self.dword1,
            SLOT_CTX_SPEED_SHIFT, SLOT_CTX_SPEED_MASK,
            (speed as u32)
        );
    }

    pub fn set_context_entries(&mut self, max_entry: u8) {
        set_field!(self.dword1,
            SLOT_CTX_ENTRYS_SHIFT, SLOT_CTX_ENTRYS_MASK,
            (max_entry as u32)
        );
    }
}

/* ------------- Scratchpad ----------------- */
#[repr(C, align(4096))]
pub struct ScratchPadBufferArray {
    scratchpads: [PhysAddr; 1024],
    page_size: usize,
}

impl ScratchPadBufferArray {
    pub fn new_with_capacity(num: usize, page_size: usize) -> Self {
        assert!(num <= 1024, "unsupported count > 1024");
        let mut thing = Self {
            scratchpads: unsafe { core::mem::zeroed() },
            page_size,
        };
        for i in 0..num {
            let ptr = VirtAddr::from_ptr(
                Global.alloc(Layout::from_size_align(page_size, page_size).expect("alignment"),
                             AllocInit::Zeroed).expect("alloc failed").ptr.as_ptr()
            );
            without_interrupts(|| {
                let pt = crate::PAGE_TABLE.read();
                thing.scratchpads[i] = pt.translate_addr(ptr).expect("mapped");
            });
        }
        thing
    }
}

impl Drop for ScratchPadBufferArray {
    fn drop(&mut self) {
        debug!("[XHCI] Freeing scratchpad buffers");
        for pad in self.scratchpads.iter() {
            let ptr = pad.as_u64();
            if ptr != 0 {
                unsafe {
                    Global.dealloc(NonNull::<u8>::new_unchecked(ptr as *mut u8),
                                   Layout::from_size_align(self.page_size, self.page_size)
                                       .expect("align"))
                };
            }
        }
    }
}



