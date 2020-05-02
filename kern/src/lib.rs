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
use x86_64::structures::paging::{RecursivePageTable, PageTable, PageTableIndex, MapperAllSizes, Mapper, Page, PageTableFlags, FrameAllocator, Size2MiB};
use x86_64::VirtAddr;
use lazy_static::lazy_static;
use spin::{Mutex, MutexGuard};
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
use crate::pci::device::xhci::XHCI;
use kernel_api::syscall::sleep;

extern crate stack_vec;
extern crate kernel_api;
extern crate cpuio;
extern crate spin;
extern crate multiboot2;
extern crate x86_64;
extern crate pc_keyboard;
extern crate alloc;
extern crate hashbrown;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate log;
#[macro_use]
extern crate bitflags;

#[macro_use]
pub mod vga_buffer;
#[macro_use]
mod macros;
pub mod init;
pub mod hardware;
pub mod device;
pub mod interrupts;
pub mod gdt;
pub mod memory;
pub mod pci;
pub mod shell;
pub mod logger;
pub mod process;
pub mod syscall;

lazy_static! {
    static ref PAGE_TABLE: Mutex<RecursivePageTable<'static>> = {
        unsafe {Mutex::new(RecursivePageTable::new(&mut(*(P4_PAGETBALE as *mut PageTable))).expect("LOL"))}
    };
}

lazy_static! {
    static ref FRAME_ALLOC: Mutex<SegmentFrameAllocator> = {
        Mutex::new(SegmentFrameAllocator::new())
    };
}
#[cfg_attr(not(test), global_allocator)]
pub static ALLOCATOR: Allocator = Allocator::uninitialized();
pub static SCHEDULER: GlobalScheduler = GlobalScheduler::uninitialized();

fn kern_init(boot_info: &BootInformation) {
    gdt::init();
    interrupts::init_idt();
    unsafe {
        PICS.lock().initialize();
        // PICS.lock().unmask_interrupt(InterruptIndex::XHCI as u8);
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

    let max_kern_mem = align_up(max(multiboot_end, kernel_end as usize), 1024*1024*2); // 2M
    debug!("Kernel end 0x{:x}", max_kern_mem);
    // Unmap extra memory (Only kernel is kept)
    for page_base in (max_kern_mem..1024*1024*1024).step_by(1024*1024*2) { // 1GB, 2MB
        let mut res = PAGE_TABLE.lock().unmap(Page::<Size2MiB>::from_start_address(VirtAddr::new(page_base as u64)).expect("page align"));
        match &mut res {
            Err(e) => {
                warn!("Unmap Err @ 0x{:x}, {:?}", page_base, e);
            },
            _ => {}
        }
    }
    x86_64::instructions::tlb::flush_all();

    for seg in mem_tags.memory_areas() {
        let mut seg_start = seg.start_address() as usize;
        let seg_end = align_down(seg.end_address() as usize, 4096);
        trace!("[FALLOC] chkseg: {:016X} - {:016X}", seg_start, seg_end);
        if seg.end_address() < kernel_start {
            continue;
        } else if seg_start <= kernel_start as usize && seg.end_address() > max_kern_mem as u64 {
            // Section contains kernel
            seg_start = max_kern_mem;
        }
        trace!("[FALLOC] AddSeg: {:016X} - {:016X}", seg_start, seg_end);
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
    GLOBAL_PIT.lock().start_clock();

    unsafe {
        SCHEDULER.initialize();
    }
}

#[no_mangle]
pub extern "C" fn kinit(multiboot_ptr: usize) -> ! {
    unsafe { crate::logger::init_logger() };
    let boot_info = unsafe { multiboot2::load(multiboot_ptr) };
    kern_init(&boot_info);
    // Must initialize after allocator
    hardware::keyboard::initialize();

    println!("Kern started");
    let usbproc = Process::new_kern(usb_process as u64);
    SCHEDULER.add(usbproc);
    SCHEDULER.start();
}

pub extern fn lol() {
    let mut shell = Shell::new();
    loop {
        shell.shell("1> ");
    }
}

pub extern fn usb_process() -> ! {
    use pci::GLOBAL_PCI;
    use pci::class::*;
    use pci::device::*;

    let mut xhci: Option<XHCI> = None;

    let stuff = GLOBAL_PCI.lock().enumerate_pci_bus();

    for dev in stuff {
        match dev.info.class.clone() {
            PCIDeviceClass::SerialBusController(c) => {
                match &c {
                    PCISerialBusController::USBController(usb) => {
                        match usb {
                            PCISerialBusUSB::XHCI => {
                                if xhci.is_some() {
                                    continue;
                                }
                                trace!("[XHCI] {:04X}:{:02X}.{:X} :({:?}): -> {:X?}",
                                       dev.bus,
                                       dev.device_number,
                                       dev.func,
                                       dev.info.header_type,
                                       dev.info.class,
                                );
                                let mut newxhci = XHCI::from(dev);
                                debug!("[XHCI] Claiming Ownership...");
                                let result = newxhci.transfer_ownership();
                                debug!("[XHCI] Ownership Result: {:?}", result);
                                newxhci.setup_controller();
                                xhci = Some(newxhci);
                            },
                            _ => {}
                        }
                    },
                    _ => {}
                }
            },
            _ => {}
        }
    }

    xhci.as_mut().expect("xhci").poll_ports();
    sleep(Duration::from_secs(1)).unwrap();
    xhci.as_mut().expect("").send_nop();


    loop {
        if xhci.is_some() {
            xhci.as_mut().expect("xhci").poll_ports();
            sleep(Duration::from_millis(100)).unwrap();
        }
    }
}
