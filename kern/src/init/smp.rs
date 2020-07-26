use x86_64::instructions::hlt;
use core::sync::atomic::{AtomicBool, Ordering};
use crate::memory::paging::KERNEL_PML4_TABLE;

/// Is the core still booting?
pub static CORE_BOOT_FLAG: AtomicBool = AtomicBool::new(false);

#[no_mangle]
pub fn ap_entry() -> ! {
    println!("New Core Running in Rust");
    CORE_BOOT_FLAG.store(false, Ordering::Relaxed);
    loop {
        hlt();
    }
}