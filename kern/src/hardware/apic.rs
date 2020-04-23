use crate::PAGE_TABLE;
use x86_64::structures::paging::{Mapper, Page, PhysFrame, PageTableFlags, Size4KiB};
use x86_64::VirtAddr;
use crate::memory::frame_allocator::FrameAllocWrapper;

pub mod timer;

pub const APIC_MAP_TARGET_BASE: usize = 0xFFFF_FF00_0000_0000;

const APIC_OFFSET_APICID: usize = 0x20;
const APIC_OFFSET_SPURIOUS_LVT: usize = 0xF0;
const APIC_OFFSET_EOI: usize = 0xB0;

pub fn map_apic_to_target() {
    let apic_base = x86_64::registers::model_specific::IA32ApicBase::read_apic_base_addr();
    let mut alloc_wrapper = FrameAllocWrapper{};
    unsafe {
        PAGE_TABLE.lock().map_to(
            Page::<Size4KiB>::containing_address(VirtAddr::new(APIC_MAP_TARGET_BASE as u64)),
            PhysFrame::containing_address(apic_base),
            PageTableFlags::PRESENT | PageTableFlags::WRITABLE,
            &mut alloc_wrapper
        ).expect("Unable to map APIC").flush()
    }
}

pub fn read_apic_id() -> u8 {
    let word = (APIC_MAP_TARGET_BASE + APIC_OFFSET_APICID) as *const u32;
    println!("{:?}", word);
    (unsafe {
        *word
    } >> 24) as u8
}

pub fn set_apic_spurious_lvt(vector: u8, enable: bool) {
    let word = (APIC_MAP_TARGET_BASE + APIC_OFFSET_SPURIOUS_LVT) as *mut u32;
    unsafe {
        *word = vector as u32 | (if enable { 1u32 } else { 0u32 }) << 8;
    }
}

pub fn end_of_interrupt() {
    let eoi = (APIC_MAP_TARGET_BASE + APIC_OFFSET_EOI) as *mut u32;
    unsafe {
        *eoi = 0;
    }
}

pub fn get_apic_spurious_lvt() -> u32 {
    let word = (APIC_MAP_TARGET_BASE + APIC_OFFSET_SPURIOUS_LVT) as *const u32;
    unsafe {
        *word
    }
}