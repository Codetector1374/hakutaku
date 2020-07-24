#![no_std]
#![feature(global_asm)]
#![feature(ptr_internals)]
#![feature(asm)]
#![feature(panic_info_message)]
#![feature(abi_x86_interrupt, naked_functions)]
#![feature(alloc_error_handler)]
#![allow(unused_imports, dead_code)]
#![allow(deprecated)]
#![feature(const_in_array_repeat_expressions)]
#![feature(allocator_api)]

use core::fmt::Write;
use core::cmp::max;
use core::borrow::BorrowMut;
use x86_64::structures::paging::{RecursivePageTable, PageTable, PageTableIndex, MapperAllSizes, Mapper, Page, PageTableFlags, FrameAllocator, Size2MiB};
use x86_64::{VirtAddr, PhysAddr};
use lazy_static::lazy_static;
use spin::{Mutex, MutexGuard, RwLock};
use crate::interrupts::{PICS, InterruptIndex};
use crate::memory::paging::P4_PAGETBALE;
use crate::memory::{align_up, align_down};
use crate::memory::frame_allocator::{SegmentFrameAllocator, MemorySegment};
use crate::memory::allocator::Allocator;
use alloc::vec::Vec;
use multiboot2::BootInformation;
use crate::shell::Shell;
use x86_64::instructions::interrupts::without_interrupts;
use crate::hardware::apic::{GLOBAL_APIC, APICDeliveryMode};
use crate::hardware::apic::timer::{APICTimerDividerOption, APICTimerMode};
use crate::hardware::pit::{GLOBAL_PIT, spin_wait};
use core::time::Duration;
use crate::process::scheduler::GlobalScheduler;
use crate::process::process::Process;
use kernel_api::syscall::sleep;
use crate::device::pci::GLOBAL_PCI;
use crate::device::ahci::G_AHCI;
use x86_64::registers::control::{Cr0Flags, Cr3Flags};
use alloc::string::String;
use crate::device::usb::G_USB;
use x86_64::instructions::hlt;
use crate::arch::x86_64::KernACPIHandler;
use acpi::Acpi;
use crate::memory::paging::KERNEL_PDPS;
use x86_64::structures::paging::page_table::PageTableEntry;

extern crate stack_vec;
extern crate kernel_api;
extern crate cpuio;
extern crate spin;
extern crate multiboot2;
extern crate x86_64;
extern crate pc_keyboard;
extern crate core_io;
#[macro_use]
extern crate alloc;
extern crate hashbrown;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate log;
#[macro_use]
extern crate modular_bitfield;

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
    static ref PAGE_TABLE: RwLock<RecursivePageTable<'static>> = {
        unsafe {RwLock::new(RecursivePageTable::new(&mut(*(P4_PAGETBALE as *mut PageTable))).expect("LOL"))}
    };
}

lazy_static! {
    static ref FRAME_ALLOC: Mutex<SegmentFrameAllocator> = {
        Mutex::new(SegmentFrameAllocator::new())
    };
}

lazy_static! {
    static ref LOW_FALLOC: Mutex<SegmentFrameAllocator> = {
        Mutex::new(SegmentFrameAllocator::new())
    };
}

#[cfg_attr(not(test), global_allocator)]
pub static ALLOCATOR: Allocator = Allocator::uninitialized();
pub static SCHEDULER: GlobalScheduler = GlobalScheduler::uninitialized();
pub static ACPI: RwLock<Option<Acpi>> = RwLock::new(None);

extern "C" {
    static mut __kernel_start: u64;
    static mut __kernel_end: u64;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                      Kernel Entrypoint                                         */
/* ---------------------------------------------------------------------------------------------- */
#[no_mangle]
#[naked]
pub extern "C" fn kinit(multiboot_ptr: usize) -> ! {

    println!("Multiboot at {:#x}", multiboot_ptr);
    unsafe { crate::logger::init_logger() };
    let boot_info = unsafe { multiboot2::load(multiboot_ptr) };

    kern_init(boot_info);

    // Must initialize after allocator
    hardware::keyboard::initialize();

    let mfg_string = x86_64::instructions::cpuid::mfgid();
    let str = String::from_utf8_lossy(&mfg_string).into_owned();
    println!("I'm running on {}", &str);

    println!("Kernel Core Ready");

    // Load the first process
    let mut main_proc = Process::new();
    main_proc.context.rsp = main_proc.stack.as_ref().unwrap().top().as_u64();
    main_proc.context.rip = kernel_initialization_process as u64;
    SCHEDULER.add(main_proc);
    SCHEDULER.start();
}

fn kern_init(boot_info: BootInformation) {
    gdt::init();
    interrupts::init_idt();
    unsafe {
        PICS.lock().initialize();
    };
    // Configure Memory System
    let mem_tags = boot_info.memory_map_tag().expect("No Mem Tags");
    let kernel_start = unsafe { &__kernel_start as *const u64 as u64 };
    let kernel_end = unsafe { &__kernel_end as *const u64 as u64 };
    let kernel_end_pa = kernel_end & !0xFFFFFFFF80000000u64;
    debug!("Kern Start - End: {:#08x} - {:#08x} ({:#x})", kernel_start , kernel_end, kernel_end_pa);

    let max_kern_mem = 32u64 * 1024 * 1024; // Reserved 32 MB

    debug!("MAX KERN MEM {:#x}, free: {}", max_kern_mem, max_kern_mem - kernel_end_pa);

    LOW_FALLOC.lock().add_segment(MemorySegment::new(kernel_end_pa as usize, (max_kern_mem - kernel_end_pa) as usize).expect(""));
    debug!("[LOW FALLOC] Free: {} MiB", LOW_FALLOC.lock().free_space() / 1024 / 1024);

    for seg in mem_tags.memory_areas() {
        let mut seg_start = seg.start_address() as usize;
        let seg_end = align_down(seg.end_address() as usize, 4096);
        trace!("[FALLOC] chkseg: {:#016X} - {:#016X}", seg_start, seg_end);
        if seg.end_address() < max_kern_mem {
            continue;
        } else if seg_start <= max_kern_mem as usize && seg.end_address() > max_kern_mem as u64 {
            // Section contains kernel
            seg_start = max_kern_mem as usize;
        }
        trace!("[FALLOC] AddSeg: {:016X} - {:016X}", seg_start, seg_end);
        without_interrupts(|| {
            FRAME_ALLOC.lock().add_segment(
                MemorySegment::new(seg_start, seg_end - seg_start)
                    .expect("Unable to create")
            )
        });
    }

    let total_mem: usize = FRAME_ALLOC.lock().free_space();
    debug!("[FALLOC] Registered {} MiB of usable memory.", total_mem / 1024 / 1024 );

    // Migrate Kernel Page Table

    // Step 1: Allocate Root Table
    let pml4_frame = LOW_FALLOC.lock().allocate_frame().expect("");
    let pml4 = unsafe { &mut *(pml4_frame.start_address().as_u64() as *mut PageTable) };
    pml4.zero();
    {
        let mut kpdps = KERNEL_PDPS.write();
        for i in 0..256usize  {
            let addr = PhysAddr::new(&kpdps[i] as *const PageTable as u64);
            pml4[i + 256].set_addr(addr, PageTableFlags::WRITABLE | PageTableFlags::PRESENT);
            kpdps[i].zero();
        }

        let pd_kern_text_start_frame = LOW_FALLOC.lock().allocate_frame().expect("");
        let pd_kern_text_start: &mut PageTable = unsafe { &mut *(pd_kern_text_start_frame.start_address().as_u64() as *mut PageTable) };
        pd_kern_text_start.zero();
        for i in 0..16 {
            pd_kern_text_start[i].set_addr(PhysAddr::new(2 * 1024 * 1024 * i as u64), PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::HUGE_PAGE);
        }
        (&mut kpdps[255])[510].set_addr(pd_kern_text_start_frame.start_address(), PageTableFlags::PRESENT | PageTableFlags::WRITABLE);
    }

    unsafe { x86_64::registers::control::Cr3::write(pml4_frame, Cr3Flags::empty()) };

    println!("Switched");

    loop {
        hlt();
    }

    println!("THING AT {:#x}", KERNEL_PDPS.read().as_ptr() as u64);

    // Initialize Allocator
    let total_mem: usize = FRAME_ALLOC.lock().free_space();
    unsafe {
        ALLOCATOR.initialize(align_up(kernel_end as usize, 4096), kernel_end as usize + total_mem);
    }

    // ENABLE Interrupt at the END
    x86_64::instructions::interrupts::enable();

    // Initialize APIC
    trace!("initializing APIC");
    GLOBAL_APIC.lock().initialize();
    trace!("initialized APIC");
    GLOBAL_APIC.lock().timer_set_lvt(0x30, APICTimerMode::Periodic, false);
    GLOBAL_APIC.lock().set_timer_interval(Duration::from_millis(0)).expect("apic set fail");
    GLOBAL_APIC.lock().set_apic_spurious_lvt(0xFF, true);
    GLOBAL_APIC.lock().lint0_set_lvt(APICDeliveryMode::ExtINT, false);

    // Start Clock
    trace!("starting clock");
    GLOBAL_PIT.write().start_clock();

    let mut acpi_handler = KernACPIHandler {};
    let acpitable = unsafe { acpi::search_for_rsdp_bios(&mut acpi_handler) };
    match acpitable {
        Ok(acpitable) => {
            ACPI.write().replace(acpitable);
            info!("[ACPI] ACPI Table Loaded");
        }
        Err(e) => {
            error!("[ACPI] Failed to located ACPI: {:?}", e);
        }
    }

    unsafe {
        SCHEDULER.initialize();
    }
}

pub extern fn kernel_initialization_process() {
    // PCI
    // GLOBAL_PCI.lock().initialize_bus_with_devices();
    //
    // // Usb Proc
    // let usbproc = Process::new_kern(usb_process as u64);
    // SCHEDULER.add(usbproc);

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
