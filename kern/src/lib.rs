#![no_std]
#![feature(global_asm)]
#![feature(ptr_internals)]
#![feature(asm)]
#![feature(panic_info_message)]
#![feature(abi_x86_interrupt, naked_functions)]
#![feature(alloc_error_handler)]
#![feature(const_fn)]
#![allow(unused_imports, dead_code)]
#![allow(deprecated)]
#![feature(const_in_array_repeat_expressions)]
#![feature(allocator_api)]
#![feature(str_internals)]

#[macro_use]
extern crate alloc;
extern crate core_io;
extern crate cpuio;
extern crate hashbrown;
extern crate kernel_api;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate log;
#[macro_use]
extern crate modular_bitfield;
extern crate multiboot2;
extern crate pc_keyboard;
extern crate spin;
extern crate stack_vec;
extern crate x86_64;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;
use core::borrow::BorrowMut;
use core::cmp::{max, min};
use core::fmt::Write;
use core::ops::Add;
use core::sync::atomic::Ordering;
use core::time::Duration;

use acpi::Acpi;
use lazy_static::lazy_static;
use multiboot2::BootInformation;
use spin::{Mutex, MutexGuard, RwLock};

use kernel_api::syscall::sleep;
use x86_64::{PhysAddr, VirtAddr};
use x86_64::instructions::hlt;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::registers::control::{Cr0Flags, Cr3, Cr3Flags};
use x86_64::structures::paging::{FrameAllocator, Mapper, MapperAllSizes, OffsetPageTable, Page, PageTable, PageTableFlags, PageTableIndex, PhysFrame, RecursivePageTable, Size2MiB, Size4KiB};
use x86_64::structures::paging::page_table::PageTableEntry;

use crate::arch::x86_64::KernACPIHandler;
use crate::device::ahci::G_AHCI;
use crate::device::pci::GLOBAL_PCI;
use crate::device::usb::G_USB;
use crate::sys::apic::{APICDeliveryMode, GLOBAL_APIC, IPIDeliveryMode, APIC};
use crate::sys::apic::timer::{APICTimerDividerOption, APICTimerMode};
use crate::sys::pit::{GLOBAL_PIT, PIT, spin_wait};
use crate::init::init::{boostrap_core_init, mp_initialization};
use crate::init::smp::CORE_BOOT_FLAG;
use crate::interrupts::{InterruptIndex, PICS};
use crate::memory::{align_down, align_up};
use crate::memory::allocator::Allocator;
use crate::memory::frame_allocator::{FrameAllocWrapper, MemorySegment, SegmentFrameAllocator};
use crate::memory::paging::{KERNEL_HEAP_BASE, KERNEL_HEAP_TOP, KERNEL_PML4_TABLE, KERNEL_TEXT_BASE, PHYSMAP_BASE};
use crate::memory::paging::KERNEL_PDPS;
use crate::process::process::Process;
use crate::process::scheduler::GlobalScheduler;
use crate::shell::Shell;
use crate::vga_buffer::disable_cursor;
use crate::sys::resman::GLOBAL_RESMAN;
use x86_64::structures::idt::InterruptDescriptorTable;

#[macro_use]
pub mod vga_buffer;
#[macro_use]
mod macros;
pub mod structure;
pub mod storage;
pub mod config;
pub mod init;
pub mod sys;
pub mod device;
pub mod interrupts;
pub mod memory;
pub mod shell;
pub mod logger;
pub mod process;
pub mod arch;
pub mod fs;

lazy_static! {
    static ref PAGE_TABLE: RwLock<OffsetPageTable<'static>> = {
        unsafe {RwLock::new(
            OffsetPageTable::new(&mut *(KERNEL_PML4_TABLE.lock().as_mut().unwrap().as_mut() as *mut PageTable), VirtAddr::new(PHYSMAP_BASE))
        )}
    };
}

pub static FRAME_ALLOC: Mutex<SegmentFrameAllocator> = Mutex::new(SegmentFrameAllocator::new());
pub static LOW_FALLOC: Mutex<SegmentFrameAllocator> = Mutex::new(SegmentFrameAllocator::new());

#[cfg_attr(not(test), global_allocator)]
pub static ALLOCATOR: Allocator = Allocator::uninitialized();
pub static SCHEDULER: GlobalScheduler = GlobalScheduler::uninitialized();
pub static ACPI: RwLock<Option<Acpi>> = RwLock::new(None);

extern "C" {
    pub static user_mode_test: u64;
    pub static user_mode_test_end: u64;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                      Kernel Entrypoint                                         */
/* ---------------------------------------------------------------------------------------------- */
#[no_mangle]
#[naked]
pub extern "C" fn kinit(multiboot_ptr: usize) -> ! {
    disable_cursor();
    debug!("Multiboot at {:#x}", multiboot_ptr);
    unsafe { crate::logger::init_logger() };
    let boot_info = unsafe { multiboot2::load(multiboot_ptr + KERNEL_TEXT_BASE as usize) };

    boostrap_core_init(boot_info);

    let mfg_string = x86_64::instructions::cpuid::mfgid();
    let str = String::from_utf8_lossy(&mfg_string).into_owned();
    println!("CPUID Brand: {}", &str);

    println!("Bootstrap Core Ready");

    unsafe {
        SCHEDULER.initialize();
    }

    unsafe {
        // Prepare a User Process
        let mut fallocw = FrameAllocWrapper{};
        let frame = FRAME_ALLOC.lock().allocate_frame().expect("no frame?");
        let user_va_start = 0x80000u64;
        PAGE_TABLE.write().map_to(
            Page::<Size4KiB>::containing_address(VirtAddr::new(user_va_start)),
            frame,
            PageTableFlags::USER_ACCESSIBLE | PageTableFlags::WRITABLE | PageTableFlags::PRESENT,
            &mut fallocw
        ).expect("mapped").flush();
        debug!("User Page Mapped");

        // Copy User Process
        let user_begin = &user_mode_test as *const u64 as u64;
        let user_end = &user_mode_test_end as *const u64 as u64;
        for src in user_begin..user_end {
            let dst = src - user_begin + user_va_start;
            *(dst as *mut u8) = *(src as *const u8);
        }
        debug!("User Process Copied");

        let selectors = GLOBAL_RESMAN.read().get_gdt(GLOBAL_APIC.read().apic_id()).selectors.clone();

        let mut user_proc = Process::new();
        user_proc.context.cs = (selectors.user_cs.0 | 0b11) as u64;
        user_proc.context.ss = (selectors.user_ds.0 | 0b11) as u64;
        user_proc.context.rip = user_va_start;
        user_proc.context.rsp = user_va_start + 4096;
        SCHEDULER.add(user_proc);
    }


    // Load the first process
    let mut main_proc = Process::new();
    main_proc.context.rsp = main_proc.stack.as_ref().unwrap().top().as_u64();
    main_proc.context.rip = kernel_initialization_process as u64;
    SCHEDULER.add(main_proc);
    SCHEDULER.start();
}

pub extern fn kernel_initialization_process() {
    // PCI
    GLOBAL_PCI.lock().initialize_bus_with_devices();

    // MP initialization
    mp_initialization();
    //
    // Usb Proc
    // let usbproc = Process::new_kern(usb_process as u64);
    // SCHEDULER.add(usbproc);

    let mut shell = Shell::new();
    loop {
        let str = format!("Core: {}> ", GLOBAL_APIC.read().apic_id());
        shell.shell(&str);
    }
}

pub extern fn usb_process() -> ! {
    loop {
        crate::device::usb::xhci::poll_xhci_devices();
    }
}
