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
use modular_bitfield::prelude::*;

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
    TransferEvent(TransferEventTRB),
}

impl From<TRB> for TRBType {
    fn from(t: TRB) -> Self {
        use TRBType::*;
        match t.type_id() {
            TRB_TYPE_LINK => Link(unsafe { t.link }),
            TRB_TYPE_EVNT_PORT_STATUS_CHG => PortStatusChange(unsafe { t.port_status_change }),
            TRB_TYPE_EVNT_CMD_COMPLETE => CommandCompletion(unsafe { t.command_completion }),
            TRB_TYPE_EVNT_HC => HostControllerEvent(unsafe { t.host_controller_event }),
            TRB_TYPE_EVNT_TRANSFER => TransferEvent(unsafe { t.transfer_event }),
            _ => TRBType::Unknown(t),
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub union TRB {
    pub command: CommandTRB,
    pub setup: SetupStageTRB,
    pub data: DataStageTRB,
    pub status_stage: StatusStageTRB,
    pub event_data: EventDataTRB,
    pub normal: NormalTRB,
    pub link: LinkTRB,
    pub port_status_change: PortStatusChangeTRB,
    pub command_completion: CommandCompletionTRB,
    pub host_controller_event: HostControllerEventTRB,
    pub transfer_event: TransferEventTRB,
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

#[bitfield]
#[derive(Copy, Clone, Debug)]
pub struct TransferEventTRBStatusWord {
    bytes_remain: B24,
    code: B8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct TransferEventTRB {
    ptr: u64,
    pub status: TransferEventTRBStatusWord,
    flags: u16,
    endpoint: u8,
    slot: u8,
}
const_assert_size!(TransferEventTRB, 16);

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
    pub fn noop() -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_NOOP_COMMAND as u32) << (TRB_COMMON_TYPE_SHIFT as u32);
        trb
    }

    pub fn enable_slot() -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_ENABLE_SLOT_CMD as u32) << (TRB_COMMON_TYPE_SHIFT as u32);
        trb
    }

    pub fn address_device(slot: u8, context_ptr: PhysAddr, block: bool) -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_ADDRESS_DEVICE_CMD as u32) << TRB_COMMON_TYPE_SHIFT as u32;
        assert_eq!(context_ptr.as_u64() & 0b1111, 0, "alignment");
        trb.payload[0] = context_ptr.as_u64() as u32;
        trb.payload[1] = (context_ptr.as_u64() >> 32) as u32;
        trb.payload[3] |= (slot as u32) << 24;
        if block {
            trb.payload[3] |= 1u32 << 9;
        }
        trb
    }
}

impl Into<TRB> for CommandTRB {
    fn into(self) -> TRB {
        TRB { command: self }
    }
}

/// Common to all Transfer TRBs
#[bitfield]
#[derive(Copy, Clone, Debug, Default)]
pub struct TransferTRBDW3 {
    transfer_size: B17,
    td_size: B5,
    interrupter: B10,
}

/// Normal TRB

#[bitfield]
#[derive(Debug, Copy, Clone, Default)]
pub struct NormalTRBDW4 {
    cycle: bool,
    eval_next: bool,
    int_on_short_packet: bool,
    ns: bool,
    chain: bool,
    ioc: bool,
    imm: bool,
    _res0: B2,
    block_event_interrupt: bool,
    trb_type: B6,
    _res1: B16,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct NormalTRB {
    pub buffer: PhysAddr,
    pub int_len: TransferTRBDW3,
    pub meta: NormalTRBDW4,
}

impl NormalTRB {
    pub fn new(buf: &[u8], max_len: usize) -> Self {
        // TODO Support max len
        assert!(buf.len() <= max_len, "exceed max len not supported");
        let buf_ptr = pt_translate!(VirtAddr::from_ptr(buf.as_ptr()));
        let mut thing = Self {
            buffer: buf_ptr,
            int_len: Default::default(),
            meta: Default::default()
        };
        thing.int_len.set_transfer_size(buf.len() as u32);
        thing.int_len.set_td_size(0);
        thing.meta.set_ioc(true);
        thing.meta.set_trb_type(TRB_TYPE_NORMAL as u8);
        thing
    }
}

/* ------- Setup TRB ------------- */
#[bitfield]
#[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
pub struct SetupStageDW3 {
    trb_length: B17,
    _res0: B5,
    interrupter: B10,
}


#[bitfield]
#[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
pub struct SetupStageDW4 {
    cycle: bool,
    _res0: B4,
    ioc: bool,
    imm: bool,
    _res1: B3,
    trb_type: B6,
    trt: B2,
    _res2: B14,
}

#[repr(C)]
#[derive(Default, Copy, Clone)]
pub struct SetupStageTRB {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
    /// Interrupt Target: (31:22), TRBLength (16:0)
    pub int_target_trb_length: SetupStageDW3,
    /// Transfer Type: (17:16)
    /// > 0: No Data, 1: Reserved, 2: Out Data, 3: In Data
    /// Transfer Type (15:10)
    /// > Should be set to Setup Stage TRB Type (XHCI Table 6-91)
    /// immData (6:6)
    /// Interrupt On Complete (5:5)
    /// Cycle Bit (0:0)
    pub metadata: SetupStageDW4,
}
const_assert_size!(SetupStageTRB, 16);

/* ----------- Data Stage TRB --------------- */

#[bitfield]
#[derive(Copy, Clone, Debug, Default)]
pub struct DataStageDW4 {
    cycle: bool,
    eval_next: bool,
    isp: bool,
    ns: bool,
    chain: bool,
    ioc: bool,
    idt: bool,
    _res: B3,
    trb_type: B6,
    read: bool,
    _res1: B15,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct DataStageTRB {
    pub buffer: PhysAddr,
    pub params: TransferTRBDW3,
    pub meta: DataStageDW4,
}

impl Default for DataStageTRB {
    fn default() -> Self {
        Self {
            buffer: PhysAddr::new(0),
            params: Default::default(),
            meta: Default::default(),
        }
    }
}

/* -------------- Event Data TRB ---------- */

#[bitfield]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct EventDataTRBDW4 {
    cycle: bool,
    eval_next: bool,
    _res: B2,
    chain: bool,
    ioc: bool,
    _res2: B3,
    block_event_interrupt: bool,
    trb_type: B6,
    _res3: B16
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct EventDataTRB {
    pub ptr: u64,
    interrupter: u32, // This needs to change to a bitfield, it's wrong
    pub meta: EventDataTRBDW4,
}

/* ------------- Status Stage TRB ----------- */
#[bitfield]
#[derive(Default, Debug, Copy, Clone)]
pub struct StatusStageTRBDW3 {
    _res: B22,
    interrupter: B10,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct StatusStageTRB {
    _resz: u64,
    pub interrupter: StatusStageTRBDW3,
    pub meta: DataStageDW4, // This is the same as DataStageDW4, so reused
}

/* -------------- Device Request Packet ----- */
#[repr(C)]
pub struct DeviceRequestPacket {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
}
const_assert_size!(DeviceRequestPacket, 8);

/* ------------- Device Context ------------- */

#[repr(C, align(2048))]
pub struct InputContext {
    pub input: [u32; 8],
    pub slot: SlotContext,
    pub endpoint: [EndpointContext; 31],
}

#[repr(C, align(2048))]
#[derive(Default, Clone)]
pub struct DeviceContextArray {
    pub slot: SlotContext,
    pub endpoint: [EndpointContext; 31],
}

macro_rules! set_field {
    ($var: expr, $shift: expr, $mask: expr, $val: expr) => {{
        let tmp = $var & (!$mask);
        $var = tmp | (($val << $shift) & $mask);
    }};
}

#[repr(C)]
#[derive(Default, Clone)]
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
        set_field!(self.flags1,
            EP_CTX_CERR_SHIFT, EP_CTX_CERR_MASK,
            val
        );
    }

    pub fn set_ep_type(&mut self, val: u8) {
        set_field!(self.flags1,
            EP_CTX_EPTYPE_SHIFT, EP_CTX_EPTYPE_MASK,
            val
        );
    }
}

#[bitfield]
#[derive(Debug, Copy, Clone, Default)]
pub struct SlotContextDW1 {
    route_string: B20,
    speed: B4,
    resz: B1,
    mtt: bool,
    hub: bool,
    context_entries: B5
}

#[repr(C)]
#[derive(Default, Clone, Debug)]
pub struct SlotContext {
    // DWORD1
    pub dword1: SlotContextDW1,
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



