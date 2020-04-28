use crate::PAGE_TABLE;
use x86_64::structures::paging::{Mapper, Page, PhysFrame, PageTableFlags, Size4KiB};
use x86_64::{VirtAddr, PhysAddr};
use crate::memory::frame_allocator::FrameAllocWrapper;
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use volatile::Volatile;
use spin::Mutex;
use crate::hardware::apic::timer::{APICTimerDividerOption, APICTimerMode};

pub mod timer;

pub static GLOBAL_APIC: Mutex<APIC> = Mutex::new(APIC::uninitialized());

pub struct APIC {
    base_va: VirtAddr,
    base_pa: PhysAddr,
    /// Scale is measured in tics per microsecond
    scale: u64,
}

const APIC_OFFSET_APICID: u64 = 0x20;
const APIC_OFFSET_SPURIOUS_LVT: u64 = 0xF0;
const APIC_OFFSET_EOI: u64 = 0xB0;
// Timers
const APIC_OFFSET_TIMER_LVT: u64 = 0x320;

const APIC_OFFSET_TIMER_INITIAL: u64 = 0x380;
const APIC_OFFSET_TIMER_CURRENT: u64 = 0x390;
const APIC_OFFSET_TIMER_DIVIDE: u64 = 0x3E0;


impl APIC {
    const fn uninitialized() -> APIC {
        APIC {
            base_va: VirtAddr::new_truncate(0),
            base_pa: PhysAddr::new_truncate(0),
            scale: 1,
        }
    }

    pub fn initialize(&mut self) {
        let apic_base = x86_64::registers::model_specific::IA32ApicBase::read_apic_base_addr();
        let (va, size) = without_interrupts(|| {
            GMMIO_ALLOC.lock().allocate(4096)
        });
        trace!("[APIC] Alloc virt: {:?}, size: {}", va, size);
        let mut alloc_wrapper = FrameAllocWrapper {};
        unsafe {
            without_interrupts(|| {
                PAGE_TABLE.lock().map_to(
                    Page::<Size4KiB>::from_start_address(va).expect("valid page bound"),
                    PhysFrame::from_start_address(apic_base).expect("pa alignment"),
                    PageTableFlags::PRESENT | PageTableFlags::WRITABLE ,
                    &mut alloc_wrapper,
                ).expect("Unable to map APIC").flush()
            })
        };
        self.base_pa = apic_base;
        self.base_va = va;
    }

    pub fn read_apic_id(&self) -> u8 {
        let word = (self.base_va.as_u64() + APIC_OFFSET_APICID) as *const Volatile<u32>;
        (unsafe {
            &*word
        }.read() >> 24) as u8
    }

    pub fn set_apic_spurious_lvt(&mut self, vector: u8, enable: bool) {
        let word = (self.base_va.as_u64() + APIC_OFFSET_SPURIOUS_LVT) as *mut Volatile<u32>;
        unsafe {
            &mut *word
        }.write(vector as u32 | (if enable { 1u32 } else { 0u32 }) << 8);
    }

    pub fn get_apic_spurious_lvt(&self) -> u32 {
        let word = unsafe { & *((self.base_va.as_u64() + APIC_OFFSET_SPURIOUS_LVT) as *const Volatile<u32>) };
        word.read()
    }

    pub fn end_of_interrupt(&mut self) {
        let eoi = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_EOI) as *mut Volatile<u32>) };
        eoi.write(0);
    }

    pub fn timer_read_divider(&self) -> APICTimerDividerOption {
        let read = unsafe {& *((self.base_va.as_u64() + APIC_OFFSET_TIMER_DIVIDE) as *const Volatile<u32>)};
        let read = read.read();
        ((read & 0b1011) as u8).into()
    }


    pub fn timer_set_divider(&mut self, opt: APICTimerDividerOption) {
        let read = unsafe {&mut *((self.base_va.as_u64() + APIC_OFFSET_TIMER_DIVIDE) as *mut Volatile<u32>)};
        read.write(opt as u8 as u32);
    }

    pub fn timer_set_initial_value(&mut self, val: u32) {
        let read = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_TIMER_INITIAL) as *mut Volatile<u32>)};
        read.write(val);
    }

    pub fn timer_get_lvt(&self) -> u32 {
        let lol = unsafe { & *((self.base_va.as_u64() + APIC_OFFSET_TIMER_LVT) as *mut Volatile<u32>)};
        lol.read()
    }

    pub fn timer_set_lvt(&mut self, vector: u8, mode: APICTimerMode, masked: bool) {
        let value: u32 = vector as u32 | (mode as u32) << 17 | (if masked { 1u32 } else { 0u32 }) << 16 ;
        let lol = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_TIMER_LVT) as *mut Volatile<u32>)};
        lol.write(value);
    }

}

