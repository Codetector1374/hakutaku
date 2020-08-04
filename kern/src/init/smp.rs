use x86_64::instructions::hlt;
use core::sync::atomic::{AtomicBool, Ordering};
use crate::memory::paging::{KERNEL_PML4_TABLE, KERNEL_TEXT_BASE, PHYSMAP_BASE};
use x86_64::structures::paging::{PageTable, PhysFrame};
use x86_64::PhysAddr;
use x86_64::registers::control::Cr3Flags;
use crate::shell::Shell;
use crate::process::process::Process;
use crate::{SCHEDULER, interrupts};
use crate::arch::x86_64::descriptor_table;
use crate::hardware::apic::{GLOBAL_APIC, APICDeliveryMode};
use crate::hardware::resman::GLOBAL_RESMAN;
use crate::interrupts::{init_idt, InterruptIndex};
use x86_64::instructions::interrupts::int3;
use core::time::Duration;
use crate::hardware::apic::timer::APICTimerMode;

/// Is the core still booting?
pub static CORE_BOOT_FLAG: AtomicBool = AtomicBool::new(false);

#[no_mangle]
pub fn ap_entry() -> ! {
    let reference_table_base = KERNEL_PML4_TABLE.lock().as_ref()
        .expect("has kRefPT").as_ref() as *const PageTable as u64 - PHYSMAP_BASE;
    let table_pa = PhysAddr::new(reference_table_base);

    unsafe { x86_64::registers::control::Cr3::write(PhysFrame::from_start_address(table_pa).expect(""), Cr3Flags::empty()); }

    unsafe { GLOBAL_RESMAN.read().get_gdt(GLOBAL_APIC.read().apic_id()).load() };
    init_idt();

    // LAPIC Setup
    GLOBAL_APIC.read().set_timer_interval(Duration::from_millis(0)).expect("apic set fail");
    GLOBAL_APIC.read().enable_lapic(0xFF, true); // This also enables the APIC
    GLOBAL_APIC.read().timer_set_lvt(InterruptIndex::ApicTimer as u8, APICTimerMode::Periodic, false);
    GLOBAL_APIC.read().lint0_set_lvt(APICDeliveryMode::ExtINT, true);

    // Enable Interrupts
    x86_64::instructions::interrupts::enable();

    // Clear the pending flag so next core can boot while we setup ourselves.
    CORE_BOOT_FLAG.store(false, Ordering::Release);
    println!("Core with LAPIC: {} is ready", GLOBAL_APIC.read().apic_id());

    SCHEDULER.start();
}

pub extern fn thing() -> ! {
    let mut shell = Shell::new();
    loop {
        shell.shell("1> ");
    }
}