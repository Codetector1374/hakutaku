use spin::RwLock;
use alloc::vec::Vec;
use crate::pci::GLOBAL_PCI;
use crate::device::ahci::controller::AHCIController;
use crate::pci::device::PCIDevice;
use crate::device::ahci::device::{AHCIDevice, AHCIAttachedDevice};

pub mod controller;
pub mod structures;
pub mod consts;
pub mod device;

pub static G_AHCI: AHCI = AHCI::new();

pub struct AHCI {
    controllers: RwLock<Vec<AHCIController>>,
    pub attached_devices: RwLock<Vec<AHCIAttachedDevice>>,
}

impl AHCI {
    const fn new() -> AHCI {
        AHCI {
            controllers: RwLock::new(Vec::new()),
            attached_devices: RwLock::new(Vec::new()),
        }
    }

    pub fn initialize_device(&self, dev: PCIDevice) {
        let loc_str = dev.bus_location_str();
        for ctlr in self.controllers.read().iter() {
            if ctlr.dev == dev {
                warn!("[AHCI] Attempt to reinitialize a controller at {}", loc_str);
                return;
            }
        }
        let ctrler = AHCIController::create_from_device(dev);
        match ctrler {
            Some(ctlr) => {
                let (ctlr_id, devices) = {
                    let mut ctlrs = self.controllers.write();
                    ctlrs.push(ctlr);
                    let id = ctlrs.len() - 1;
                    (id, ctlrs[id].port_scan())
                };
                let mut vec = self.attached_devices.write();
                for device in devices.into_iter() {
                    vec.push(AHCIAttachedDevice::create(ctlr_id, device.port(), device))
                }
            },
            None => {
                error!("[AHCI] Failed to initialize controller at {}", loc_str);
            }
        }
    }
}
