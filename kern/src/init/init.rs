use alloc::boxed::Box;
use core::cmp::min;
use core::ops::Add;
use core::sync::atomic::Ordering;
use core::time::Duration;

use multiboot2::BootInformation;

use kernel_api::syscall::sleep;
use x86_64::{PhysAddr, VirtAddr};
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::registers::control::Cr3;
use x86_64::registers::control::Cr3Flags;
use x86_64::structures::paging::{FrameAllocator, PageTable, PageTableFlags, PhysFrame};

use crate::{ACPI, ALLOCATOR, FRAME_ALLOC, interrupts, LOW_FALLOC, SCHEDULER};
use crate::arch::x86_64::descriptor_table;
use crate::arch::x86_64::KernACPIHandler;
use crate::sys::apic::{APICDeliveryMode, GLOBAL_APIC, IPIDeliveryMode, IPIDestinationShorthand};
use crate::sys::apic::timer::APICTimerMode;
use crate::sys::pit::GLOBAL_PIT;
use crate::init::smp::CORE_BOOT_FLAG;
use crate::interrupts::{PICS, InterruptIndex};
use crate::KERNEL_PDPS;
use crate::memory::align_down;
use crate::memory::frame_allocator::MemorySegment;
use crate::memory::paging::{KERNEL_HEAP_BASE, KERNEL_HEAP_TOP, KERNEL_PML4_TABLE, KERNEL_TEXT_BASE, PHYSMAP_BASE};
use crate::sys::resman::GLOBAL_RESMAN;
use crate::device::uart::serial16650::COM1_BASE_ADDR;

extern "C" {
    static mut __kernel_start: u64;
    static mut __kernel_end: u64;
    static mut __ap_stack_top: u64;
}

pub struct BootArgs {
    early_serial_port: u16
}

pub fn boostrap_core_init(boot_info: BootInformation) {

    // Configure Memory System
    let mem_tags = boot_info.memory_map_tag().expect("No Mem Tags");
    let kernel_start = unsafe { &__kernel_start as *const u64 as u64 };
    let kernel_end = unsafe { &__kernel_end as *const u64 as u64 };
    let kernel_end_pa = kernel_end & !0xFFFFFFFF80000000u64;
    let max_phys_mem = mem_tags.memory_areas().fold(0u64, |prev_max, x| {
        if x.end_address() > prev_max {
            x.end_address()
        } else {
            prev_max
        }
    });
    debug!("Kern Start - End: {:#08x} - {:#08x} ({:#x})", kernel_start, kernel_end, kernel_end_pa);
    debug!("Max PhysMem {:#x}", max_phys_mem);

    let max_kern_mem = 16u64 * 1024 * 1024; // Reserved 16 MB

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

    // Migrate Kernel Page Table
    // Step 1: Allocate Root Table
    let pml4_frame = LOW_FALLOC.lock().allocate_frame().expect("");
    let pml4 = unsafe { &mut *(pml4_frame.start_address().as_u64() as *mut PageTable) };
    pml4.zero();
    {
        let mut kpdps = KERNEL_PDPS.write();
        for i in 0..256usize {
            let addr = PhysAddr::new(&kpdps[i] as *const PageTable as u64 - KERNEL_TEXT_BASE);
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

    // Borrow the old PT's PML4[0] table
    let old_pml4: &mut PageTable = unsafe { &mut *VirtAddr::new(Cr3::read().0.start_address().as_u64()).as_mut_ptr() };
    pml4[0] = old_pml4[0].clone();

    // ============================ PAGE TABLE SWAP!!!! ======================================
    unsafe { x86_64::registers::control::Cr3::write(pml4_frame, Cr3Flags::empty()) };
    // NOW THE PAGE_TABLE is accessible.
    KERNEL_PML4_TABLE.lock().replace(unsafe { Box::from_raw((pml4_frame.start_address().as_u64() + PHYSMAP_BASE) as *mut PageTable) });
    println!("================== \n[INIT] Switched Pagetable \n============");

    // Step 2: Create Identity Map in PhysMap region
    let phys_map_max = if max_phys_mem < 4 * 1024 * 1024 * 1024u64 {
        debug!("less than 4GB of phys mem is detected. Phys map will expand to 4GB");
        4 * 1024 * 1024 * 1024
    } else {
        PhysAddr::new(max_phys_mem).align_up(2 * 1024 * 1024u64).as_u64()
    };
    debug!("Physmap will be from 0 to {:#x}", phys_map_max);

    {
        let mut kpdps = KERNEL_PDPS.write();

        let mut saved_frame: Option<PhysFrame> = None;
        let mut backup_frame_cnt = 0;
        for pa in (0..phys_map_max).step_by(2 * 1024 * 1024) {
            let va = VirtAddr::new(pa) + PHYSMAP_BASE;
            assert!(!pml4[va.p4_index()].is_unused(), "PML4 table has unmapped entry in kaddr");
            assert!(u16::from(va.p4_index()) >= 256u16, "VA in incorrect range");
            if kpdps[usize::from(va.p4_index()) - 256][va.p3_index()].is_unused() {
                if saved_frame.is_none() {
                    saved_frame = Some(FRAME_ALLOC.lock().allocate_frame().expect(""));
                }

                let selected = if saved_frame.as_ref().unwrap().start_address().as_u64() < pa {
                    saved_frame.take().unwrap()
                } else {
                    backup_frame_cnt += 1;
                    LOW_FALLOC.lock().allocate_frame().expect("")
                };

                kpdps[usize::from(va.p4_index()) - 256][va.p3_index()]
                    .set_addr(selected.start_address(), PageTableFlags::PRESENT | PageTableFlags::WRITABLE);
            }

            let p2_table_addr = kpdps[usize::from(va.p4_index()) - 256][va.p3_index()].addr();
            let p2_table_addr =
                if p2_table_addr.as_u64() < 32 * 1024 * 1024 {
                    VirtAddr::new(p2_table_addr.as_u64())
                } else {
                    VirtAddr::new(p2_table_addr.as_u64()) + PHYSMAP_BASE
                };
            let p2_table: &mut PageTable = unsafe { &mut *(p2_table_addr.as_mut_ptr()) };
            p2_table[va.p2_index()]
                .set_addr(PhysAddr::new(pa),
                          PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::HUGE_PAGE);
        }
        trace!("[INIT] {} low_falloc pages used for PhysMap", backup_frame_cnt);
    }

    // Unmap the borrowed table
    pml4[0].set_unused();
    x86_64::instructions::tlb::flush_all();

    let total_mem: usize = FRAME_ALLOC.lock().free_space();
    info!("[INIT] Free memory from System Frame Allocator: {} MiB", total_mem / 1024 / 1024);

    // Initialize Allocator
    unsafe {
        ALLOCATOR.initialize
        (
            KERNEL_HEAP_BASE as usize,
            min(KERNEL_HEAP_TOP, KERNEL_HEAP_BASE.saturating_add(total_mem as u64)) as usize,
        );
    }
    debug!("[kALLOC] Kernel Allocator Initialized");

    // Initialize Early Serial
    if !crate::device::uart::SERIAL_PORTS.write().register_early_serial(COM1_BASE_ADDR) {
        crate::device::uart::SERIAL_PORTS.write().register_early_serial(0xd000);
    }

    // Initialize APIC
    trace!("initializing APIC");
    GLOBAL_APIC.write().initialize();
    trace!("initialized APIC");
    GLOBAL_APIC.read().enable_lapic(0xFF, true);
    GLOBAL_APIC.read().timer_set_lvt(InterruptIndex::ApicTimer as u8, APICTimerMode::Periodic, false);
    GLOBAL_APIC.read().set_timer_interval(Duration::from_millis(0)).expect("apic set fail");
    GLOBAL_APIC.read().lint0_set_lvt(APICDeliveryMode::ExtINT, false);

    // Setup TSS / GDT / IDT
    {
        let mut x = GLOBAL_RESMAN.write();
        x.initialize();
        x.register_core(GLOBAL_APIC.read().apic_id());
    }

    unsafe { GLOBAL_RESMAN.read().get_gdt(GLOBAL_APIC.read().apic_id()).load(); }
    interrupts::init_idt();

    unsafe {
        PICS.lock().initialize();
    };

    // ENABLE Interrupt at the END
    x86_64::instructions::interrupts::enable();

    // Start Clock
    trace!("starting clock");
    GLOBAL_PIT.write().start_clock();

    let (rsdtaddr, version) = match boot_info.rsdp_v2_tag() {
        Some(flag) => {
            (flag.xsdt_address(), flag.revision())
        }
        _ => {
            let lol = boot_info.rsdp_v1_tag().expect("gotta have RSDP right?");
            (lol.rsdt_address(), lol.revision())
        }
    };

    let mut acpi_handler = KernACPIHandler {};
    let acpitable = unsafe { acpi::parse_rsdt(&mut acpi_handler, version, rsdtaddr) };
    match acpitable {
        Ok(acpitable) => {
            ACPI.write().replace(acpitable);
            info!("[ACPI] ACPI Table Loaded");
        }
        Err(e) => {
            error!("[ACPI] Failed to located ACPI: {:?}", e);
        }
    }
}

pub fn mp_initialization() {
    let acpi_handle = ACPI.read();
    let acpi = acpi_handle.as_ref().expect("no table >>_<<");

    for x in &acpi.application_processors {
        CORE_BOOT_FLAG.store(true, Ordering::Release);
        let frame = LOW_FALLOC.lock().allocate_frame().expect("");
        let sp = frame.start_address().as_u64() + 4096 + KERNEL_TEXT_BASE;
        let ap_stack_top: &mut u64 = unsafe {
            &mut *VirtAddr::from_ptr(&__ap_stack_top as *const u64).add(PHYSMAP_BASE).as_mut_ptr()
        };
        *ap_stack_top = sp;
        let apic_id = x.local_apic_id;
        println!("Core {} stack: {:#x}", apic_id,sp);

        // Register core with Resman
        GLOBAL_RESMAN.write().register_core(apic_id);

        crate::sys::apic::send_ipi(apic_id, 0, IPIDeliveryMode::INIT, IPIDestinationShorthand::NoShorthand);
        sleep(Duration::from_millis(10)).expect("");
        crate::sys::apic::send_ipi(apic_id, 0x8, IPIDeliveryMode::StartUp, IPIDestinationShorthand::NoShorthand);
        sleep(Duration::from_millis(1)).unwrap();
        if CORE_BOOT_FLAG.load(Ordering::Relaxed) {
            crate::sys::apic::send_ipi(apic_id, 0x8, IPIDeliveryMode::StartUp, IPIDestinationShorthand::NoShorthand);
        }
        while CORE_BOOT_FLAG.load(Ordering::Relaxed) {
            sleep(Duration::from_millis(1)).expect("");
        }
    }
}
