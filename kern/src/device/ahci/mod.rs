use spin::Mutex;
use alloc::vec::Vec;
use crate::pci::GLOBAL_PCI;
use crate::device::ahci::controller::AHCIController;

pub mod controller;
pub mod structures;
pub mod consts;

pub static G_AHCI: AHCI = AHCI::new();

pub struct AHCI {
    controllers: Mutex<Vec<AHCIController>>
}

impl AHCI {
    const fn new() -> AHCI {
        AHCI {
            controllers: Mutex::new(Vec::new())
        }
    }

    pub fn initialize(&self) {
        for dev in GLOBAL_PCI.lock().enumerate_pci_bus() {
            use crate::pci::class::*;
            if let PCIDeviceClass::MassStorageController(PCIClassMassStorage::SATA(_)) = dev.info.class {
                match AHCIController::create_from_device(dev) {
                    Some(ctrler) => {
                        self.controllers.lock().push(ctrler);
                    },
                    None => {
                        error!("[AHCI] Failed to create AHCI Device");
                    }
                }
            }
        }
    }
}