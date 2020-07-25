use crate::PAGE_TABLE;
use x86_64::structures::paging::{Mapper, Page, PhysFrame, PageTableFlags, Size4KiB};
use x86_64::{VirtAddr, PhysAddr};
use crate::memory::frame_allocator::FrameAllocWrapper;
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::VMALLOC;
use volatile::Volatile;
use spin::Mutex;
use crate::hardware::apic::timer::{APICTimerDividerOption, APICTimerMode};
use crate::hardware::pit::spin_wait;
use core::time::Duration;
use crate::interrupts::{PICS, InterruptIndex};

pub mod timer;

pub static GLOBAL_APIC: Mutex<APIC> = Mutex::new(APIC::uninitialized());

#[repr(u8)]
pub enum APICDeliveryMode {
    Fixed = 0b000,
    SMI = 0b010,
    NMI = 0b100,
    ExtINT = 0b111,
    INIT = 0b101,
}

pub struct APIC {
    base_va: VirtAddr,
    base_pa: PhysAddr,
    /// Scale is measured in tics per microsecond
    scale: u32,
}

const APIC_OFFSET_APICID: u64 = 0x20;
const APIC_OFFSET_SPURIOUS_LVT: u64 = 0xF0;
const APIC_OFFSET_EOI: u64 = 0xB0;
// Timers
const APIC_OFFSET_TIMER_LVT: u64 = 0x320;
const APIC_OFFSET_LINT0_LVT: u64 = 0x350;

const APIC_OFFSET_TIMER_INITIAL: u64 = 0x380;
const APIC_OFFSET_TIMER_CURRENT: u64 = 0x390;
const APIC_OFFSET_TIMER_DIVIDE: u64 = 0x3E0;

const MAX_TICK: u64 = 0xFFFF_FF00;
const MEASURE_DURATION: Duration = Duration::from_millis(10);

impl APIC {
    const fn uninitialized() -> APIC {
        APIC {
            base_va: VirtAddr::new_truncate(0),
            base_pa: PhysAddr::new_truncate(0),
            scale: 1,
        }
    }

    pub fn initialize(&mut self) {
        if self.base_va.as_u64() != 0 {
            warn!("[APIC] double initialization");
            return;
        }
        let apic_base = x86_64::registers::model_specific::IA32ApicBase::read_apic_base_addr();
        let (va, size) = without_interrupts(|| {
            VMALLOC.lock().allocate(4096)
        });
        trace!("[APIC] Alloc virt: {:?}, size: {}", va, size);
        let mut alloc_wrapper = FrameAllocWrapper {};
        unsafe {
            without_interrupts(|| {
                PAGE_TABLE.write().map_to(
                    Page::<Size4KiB>::from_start_address(va).expect("valid page bound"),
                    PhysFrame::from_start_address(apic_base).expect("pa alignment"),
                    PageTableFlags::PRESENT | PageTableFlags::WRITE_THROUGH | PageTableFlags::WRITABLE,
                    &mut alloc_wrapper,
                ).expect("Unable to map APIC").flush()
            })
        };
        self.base_pa = apic_base;
        self.base_va = va;
        trace!("[APIC] Mapped");


        trace!("[APIC] Measure Start");
        // Measurement
        without_interrupts(||{
            self.timer_set_divider(APICTimerDividerOption::DivideBy4);
            self.timer_set_initial_value(0xFFFF_FFFF);
        });
        spin_wait(MEASURE_DURATION);
        let diff = 0xFFFF_FFFF - self.timer_read_current_value();
        let tick_scale = diff / MEASURE_DURATION.as_micros() as u32;
        self.scale = tick_scale * 4;
        debug!("[APIC] 0x{:x}t in {:?}, {} t/us (x4)", diff, MEASURE_DURATION, tick_scale);
    }

    pub fn apic_id(&self) -> u8 {
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
        let word = unsafe { &*((self.base_va.as_u64() + APIC_OFFSET_SPURIOUS_LVT) as *const Volatile<u32>) };
        word.read()
    }

    pub fn end_of_interrupt(&mut self) {
        let eoi = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_EOI) as *mut Volatile<u32>) };
        eoi.write(0);
    }

    pub fn timer_read_divider(&self) -> APICTimerDividerOption {
        let read = unsafe { &*((self.base_va.as_u64() + APIC_OFFSET_TIMER_DIVIDE) as *const Volatile<u32>) };
        let read = read.read();
        ((read & 0b1011) as u8).into()
    }


    pub fn timer_set_divider(&mut self, opt: APICTimerDividerOption) {
        let read = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_TIMER_DIVIDE) as *mut Volatile<u32>) };
        read.write(opt as u8 as u32);
    }

    pub fn set_timer_interval(&mut self, duration: Duration) -> Result<(), ()> {
        use crate::hardware::apic::timer::APICTimerDividerOption::*;
        let ticks: u64 = duration.as_micros() as u64 * self.scale as u64;
        let (div_ticks, div) =
            if ticks < MAX_TICK as u64 {
                (ticks, DivideBy1)
            } else if (ticks / 2) < MAX_TICK {
                (ticks / 2, DivideBy2)
            } else if (ticks / 4) < MAX_TICK {
                (ticks / 4, DivideBy4)
            } else if (ticks / 8) < MAX_TICK {
                (ticks / 8, DivideBy8)
            } else if (ticks / 16) < MAX_TICK {
                (ticks / 16, DivideBy16)
            } else if (ticks / 32) < MAX_TICK {
                (ticks / 32, DivideBy32)
            } else if (ticks / 64) < MAX_TICK {
                (ticks / 64, DivideBy64)
            } else if (ticks / 128) < MAX_TICK {
                (ticks / 128, DivideBy128)
            } else {
                return Err(());
            };
        self.timer_set_divider(div);
        self.timer_set_initial_value(div_ticks as u32);
        Ok(())
    }

    pub fn timer_set_initial_value(&mut self, val: u32) {
        let read = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_TIMER_INITIAL) as *mut Volatile<u32>) };
        read.write(val);
    }

    pub fn timer_read_current_value(&self) -> u32 {
        let read = unsafe { &*((self.base_va.as_u64() + APIC_OFFSET_TIMER_CURRENT) as *const Volatile<u32>) };
        read.read()
    }

    pub fn timer_get_lvt(&self) -> u32 {
        let lol = unsafe { &*((self.base_va.as_u64() + APIC_OFFSET_TIMER_LVT) as *mut Volatile<u32>) };
        lol.read()
    }

    pub fn timer_set_lvt(&mut self, vector: u8, mode: APICTimerMode, masked: bool) {
        let value: u32 = vector as u32 | (mode as u32) << 17 | (if masked { 1u32 } else { 0u32 }) << 16;
        let lol = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_TIMER_LVT) as *mut Volatile<u32>) };
        lol.write(value);
    }

    pub fn lint0_set_lvt(&mut self, mode: APICDeliveryMode, masked: bool) {
        let value: u32 = InterruptIndex::XHCI as u32 | (mode as u32) << 8 | (if masked { 1u32 } else { 0u32 }) << 16;
        let lol = unsafe { &mut *((self.base_va.as_u64() + APIC_OFFSET_LINT0_LVT) as *mut Volatile<u32>) };
        lol.write(value);
    }
}

