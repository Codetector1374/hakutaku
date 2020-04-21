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
pub mod vga_buffer;
pub mod init;
pub mod hardware;
pub mod interrupts;
pub mod gdt;
pub mod memory;

fn kern_init() {
    println!("Re-Initializing GDT in Rust");
    gdt::init();
    println!("Setting Exception Handlers");
    interrupts::init_idt();
    println!("Success!");
    x86_64::instructions::interrupts::int3();
    println!("Continue Execution after int3!");
    println!("Initializing PICs");
    unsafe {
        PICS.lock().initialize();
    };
    x86_64::instructions::interrupts::enable();
}

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

#[cfg_attr(not(test), global_allocator)]
pub static ALLOCATOR: Allocator = Allocator::uninitialized();

#[no_mangle]
pub extern "C" fn kinit(multiboot_ptr: usize) -> ! {
    println!("Hello World \n Multiboot: 0x{:08X}", multiboot_ptr);
    let boot_info = unsafe { multiboot2::load(multiboot_ptr) };

    let mem_tags = boot_info.memory_map_tag().expect("No Mem Tags");
    println!("==== BEGIN MEM =====");
    for (id, m) in mem_tags.memory_areas().enumerate() {
        println!("Chunk {}: 0x{:08X} - 0x{:08X} - {:?}", id, m.start_address(), m.end_address(), m.typ());
    }
    println!("==== BEGIN KERN INFO =====");
    let elf_sections_tag = boot_info.elf_sections_tag()
        .expect("Elf-sections tag required");
    let kernel_start = elf_sections_tag.sections().map(|s| s.start_address())
        .min().unwrap();
    let kernel_end = elf_sections_tag.sections().map(|s| s.start_address() + s.size())
        .max().unwrap();
    let multiboot_start = boot_info.start_address();
    let multiboot_end = multiboot_start + (boot_info.total_size() as usize);
    println!("Kern: 0x{:08X} -> 0x{:08X}", kernel_start, kernel_end);
    println!("MB: 0x{:08X} -> 0x{:08X}", multiboot_start, multiboot_end);

    let max_kern_mem = max(multiboot_end, kernel_end as usize);

    kern_init();

    for (id, seg) in mem_tags.memory_areas().enumerate() {
        let mut seg_start = seg.start_address() as usize;
        let seg_end = align_down(seg.end_address() as usize, 4096);
        if seg.end_address() < kernel_start {
            continue;
        } else if seg_start <= kernel_start as usize && seg.end_address() > kernel_end {
            // Section contains kernel
            seg_start = align_up(kernel_end as usize, 4096);
        }
        FRAME_ALLOC.lock().add_segment(
            MemorySegment::new(seg_start, seg_end - seg_start)
                .expect("Unable to create")
        )
    }

    let total_mem: usize = FRAME_ALLOC.lock().free_space();
    unsafe {
        ALLOCATOR.initialize(align_up(kernel_end as usize, 4096), (kernel_end as usize + total_mem));
    }

    let lmao = (0xDEADBEEFFF0 as *mut u32);
    unsafe {
        *lmao = 0xB0BACAFE;
        println!("LOL: {:X}", *lmao);
    }

    loop {
        unsafe {
            asm!("hlt" :::: "volatile");
        }
    }
}
