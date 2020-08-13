//! Resource Manager

use hashbrown::HashMap;
use crate::arch::x86_64::descriptor_table::{GDTInfo, TSSInfo};
use spin::RwLock;
use crate::sys::apic::GLOBAL_APIC;
use alloc::boxed::Box;
use x86_64::VirtAddr;

pub static GLOBAL_RESMAN: RwLock<ResourceManager> = RwLock::new(ResourceManager::uninitialized());

pub struct ResourceManager {
    // Per-Core Resources
    gdts: Option<HashMap<u8, GDTInfo>>,
    tsses: Option<HashMap<u8, TSSInfo>>,
    // Global Resource
}

impl ResourceManager {
    const fn uninitialized() -> Self {
        Self {
            gdts: None,
            tsses: None
        }
    }

    pub fn initialize(&mut self) {
        // Initialize GDT
        self.gdts = Some(HashMap::new());

        // Initialize TSSes
        self.tsses = Some(HashMap::new());
    }

    pub fn register_core(&mut self, lapic_id: u8) {
        debug!("[RESMAN] Registering Core {}", lapic_id);
        let mut tss = crate::arch::x86_64::descriptor_table::create_tss();
        let gdt = crate::arch::x86_64::descriptor_table::create_gdt(tss.get_tss_ptr());

        self.gdts.as_mut().unwrap().insert(lapic_id, gdt);
        self.tsses.as_mut().unwrap().insert(lapic_id, tss);
    }

    pub fn get_gdt(&self, lapic_id: u8) -> &GDTInfo {
        self.gdts.as_ref().unwrap().get(&lapic_id).unwrap()
    }
}
