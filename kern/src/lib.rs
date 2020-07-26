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
use core::time::Duration;

use acpi::Acpi;
use kernel_api::syscall::sleep;
use lazy_static::lazy_static;
use multiboot2::BootInformation;
use spin::{Mutex, MutexGuard, RwLock};
use x86_64::{PhysAddr, VirtAddr};
use x86_64::instructions::hlt;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::registers::control::{Cr0Flags, Cr3, Cr3Flags};
use x86_64::structures::paging::{FrameAllocator, Mapper, MapperAllSizes, OffsetPageTable, Page, PageTable, PageTableFlags, PageTableIndex, PhysFrame, RecursivePageTable, Size2MiB};
use x86_64::structures::paging::page_table::PageTableEntry;

use crate::arch::x86_64::KernACPIHandler;
use crate::device::ahci::G_AHCI;
use crate::device::pci::GLOBAL_PCI;
use crate::device::usb::G_USB;
use crate::hardware::apic::{APICDeliveryMode, GLOBAL_APIC, IPIDeliveryMode};
use crate::hardware::apic::timer::{APICTimerDividerOption, APICTimerMode};
use crate::hardware::pit::{GLOBAL_PIT, spin_wait};
use crate::interrupts::{InterruptIndex, PICS};
use crate::memory::{align_down, align_up};
use crate::memory::allocator::Allocator;
use crate::memory::frame_allocator::{MemorySegment, SegmentFrameAllocator};
use crate::memory::paging::{KERNEL_PML4_TABLE, KERNEL_TEXT_BASE, PHYSMAP_BASE, KERNEL_HEAP_BASE, KERNEL_HEAP_TOP};
use crate::memory::paging::KERNEL_PDPS;
use crate::process::process::Process;
use crate::process::scheduler::GlobalScheduler;
use crate::shell::Shell;
use core::ops::Add;
use crate::init::smp::CORE_BOOT_FLAG;
use core::sync::atomic::Ordering;
use crate::init::init::{kern_init, ap_initialization};
use crate::vga_buffer::disable_cursor;

#[macro_use]
pub mod vga_buffer;
#[macro_use]
mod macros;
pub mod storage;
pub mod config;
pub mod init;
pub mod hardware;
pub mod device;
pub mod interrupts;
pub mod gdt;
pub mod memory;
pub mod shell;
pub mod logger;
pub mod process;
pub mod arch;

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

/* ---------------------------------------------------------------------------------------------- */
/*                                      Kernel Entrypoint                                         */
/* ---------------------------------------------------------------------------------------------- */
#[no_mangle]
#[naked]
pub extern "C" fn kinit(multiboot_ptr: usize) -> ! {
    disable_cursor();
    println!("Multiboot at {:#x}", multiboot_ptr);
    unsafe { crate::logger::init_logger() };
    let boot_info = unsafe { multiboot2::load(multiboot_ptr + KERNEL_TEXT_BASE as usize) };

    kern_init(boot_info);

    // Must initialize after allocator
    hardware::keyboard::initialize();

    let mfg_string = x86_64::instructions::cpuid::mfgid();
    let str = String::from_utf8_lossy(&mfg_string).into_owned();
    println!("I'm running on {}", &str);

    println!("Kernel Core Ready");

    unsafe {
        SCHEDULER.initialize();
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
    //
    // // Usb Proc
    // let usbproc = Process::new_kern(usb_process as u64);
    // SCHEDULER.add(usbproc);
    ap_initialization();

    let mut shell = Shell::new();
    loop {
        shell.shell("1> ");
    }
}

pub extern fn usb_process() -> ! {
    use crate::device::usb::G_USB;

    loop {
        G_USB.xhci.read().iter().for_each(|c| { c.poll_ports() });
        sleep(Duration::from_millis(100)).unwrap();
    }
}
