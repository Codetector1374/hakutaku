#![no_std]
#![feature(global_asm)]
#![feature(ptr_internals)]
#![feature(asm)]
#![feature(panic_info_message)]
#![feature(abi_x86_interrupt)]
#![feature(alloc_error_handler)]
#![allow(unused_imports, dead_code)]

use core::fmt::Write;
use core::cmp::max;
use core::borrow::BorrowMut;
use x86_64::structures::paging::{RecursivePageTable, PageTable, PageTableIndex, MapperAllSizes, Mapper, Page, PageTableFlags, FrameAllocator};
use x86_64::VirtAddr;
use lazy_static::lazy_static;
use spin::{Mutex, MutexGuard};
use crate::interrupts::PICS;
use crate::memory::paging::P4_PAGETBALE;
use crate::memory::{align_up, align_down};
use crate::memory::frame_allocator::{SegmentFrameAllocator, MemorySegment};
use crate::memory::allocator::Allocator;
use alloc::vec::Vec;
use crate::hardware::apic::{map_apic_to_target, read_apic_id};
use crate::hardware::apic::timer::{APICTimerDividerOption, APICTimerMode};
use multiboot2::BootInformation;
use crate::hardware::keyboard::blocking_get_char;
use crate::shell::Shell;
use x86_64::instructions::interrupts::without_interrupts;

extern crate stack_vec;
extern crate cpuio;
extern crate spin;
extern crate multiboot2;
extern crate x86_64;
extern crate pc_keyboard;
extern crate alloc;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate log;

#[macro_use]
pub mod vga_buffer;
pub mod init;
pub mod hardware;
pub mod interrupts;
pub mod gdt;
pub mod memory;
pub mod pci;
pub mod shell;
pub mod logger;


lazy_static! {
    static ref PAGE_TABLE: Mutex<RecursivePageTable<'static>> = {
        unsafe {
            Mutex::new(RecursivePageTable::new_unchecked(&mut(*(P4_PAGETBALE as *mut PageTable)), PageTableIndex::new(511)))
        }
    };
}

lazy_static! {
    static ref FRAME_ALLOC: Mutex<SegmentFrameAllocator> = {
        Mutex::new({
            SegmentFrameAllocator::new()
        })
    };
}

fn kern_init(boot_info: &BootInformation) {
    gdt::init();
    interrupts::init_idt();
    unsafe {
        PICS.lock().initialize();
    };
    // Configure Memory System
    let mem_tags = boot_info.memory_map_tag().expect("No Mem Tags");
    let elf_sections_tag = boot_info.elf_sections_tag()
        .expect("Elf-sections tag required");
    let kernel_start = elf_sections_tag.sections().map(|s| s.start_address())
        .min().unwrap();
    let kernel_end = elf_sections_tag.sections().map(|s| s.start_address() + s.size())
        .max().unwrap();
    let multiboot_start = boot_info.start_address();
    let multiboot_end = multiboot_start + (boot_info.total_size() as usize);

    let max_kern_mem = max(multiboot_end, kernel_end as usize);
    for seg in mem_tags.memory_areas() {
        let mut seg_start = seg.start_address() as usize;
        let seg_end = align_down(seg.end_address() as usize, 4096);
        debug!("chkseg: {:016X} - {:016X}", seg_start, seg_end);
        if seg.end_address() < kernel_start {
            continue;
        } else if seg_start <= kernel_start as usize && seg.end_address() > max_kern_mem as u64 {
            // Section contains kernel
            seg_start = align_up(max_kern_mem, 4096);
        }
        debug!("AddSeg: {:016X} - {:016X}", seg_start, seg_end);
        without_interrupts(||{
            FRAME_ALLOC.lock().add_segment(
                MemorySegment::new(seg_start, seg_end - seg_start)
                    .expect("Unable to create")
            )
        });
    }

    // Initialize Allocator
    let total_mem: usize = FRAME_ALLOC.lock().free_space();
    unsafe {
        ALLOCATOR.initialize(align_up(kernel_end as usize, 4096), kernel_end as usize + total_mem);
    }

    // Initialize APIC
    map_apic_to_target();
    // TODO Change Timer to a determinable value
    hardware::apic::timer::set_divider(APICTimerDividerOption::DivideBy128);
    // hardware::apic::timer::set_timer_lvt(0x30, APICTimerMode::Periodic, false);
    // hardware::apic::set_apic_spurious_lvt(0xFF, true);
    // hardware::apic::timer::set_initial_value(0x000F_FFFF);

    // ENABLE Interrupt at the END
    x86_64::instructions::interrupts::enable();
}

#[cfg_attr(not(test), global_allocator)]
pub static ALLOCATOR: Allocator = Allocator::uninitialized();

#[no_mangle]
pub extern "C" fn kinit(multiboot_ptr: usize) -> ! {
    unsafe { crate::logger::init_logger() };
    let boot_info = unsafe { multiboot2::load(multiboot_ptr) };
    kern_init(&boot_info);
    // Must initialize after allocator
    hardware::keyboard::initialize();

    println!("Kern started");

    let mut shell = Shell::new();

    loop {
        shell.shell("> ");
        // println!("chr: 0x{:02X}", blocking_get_char());
    }
}
