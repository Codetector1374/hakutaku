use x86_64::instructions::hlt;
use core::sync::atomic::{AtomicBool, Ordering};
use crate::memory::paging::{KERNEL_PML4_TABLE, KERNEL_TEXT_BASE, PHYSMAP_BASE};
use x86_64::structures::paging::{PageTable, PhysFrame};
use x86_64::PhysAddr;
use x86_64::registers::control::Cr3Flags;

/// Is the core still booting?
pub static CORE_BOOT_FLAG: AtomicBool = AtomicBool::new(false);

#[no_mangle]
pub fn ap_entry() -> ! {
    let reference_table_base = KERNEL_PML4_TABLE.lock().as_ref()
        .expect("has kRefPT").as_ref() as *const PageTable as u64 - PHYSMAP_BASE;
    let table_pa = PhysAddr::new(reference_table_base);

    unsafe { x86_64::registers::control::Cr3::write(PhysFrame::from_start_address(table_pa).expect(""), Cr3Flags::empty()); }

    // Clear the pending flag so next core can boot while we setup ourselves.
    CORE_BOOT_FLAG.store(false, Ordering::Relaxed);

    loop {
        hlt();
    }
}