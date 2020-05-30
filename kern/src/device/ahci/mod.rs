use spin::RwLock;
use alloc::vec::Vec;
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::device::PCIDevice;
use crate::storage::block::G_BLOCK_DEV_MGR;
use crate::device::ahci::controller::AHCIController;
use crate::device::ahci::device::{AHCIDevice, AHCIAttachedDevice, AHCIBlockDevice};
use alloc::boxed::Box;
use alloc::sync::Arc;

pub mod controller;
pub mod structures;
pub mod consts;
pub mod device;

pub static G_AHCI: AHCI = AHCI::new();

pub struct AHCI {
    controllers: RwLock<Vec<AHCIController>>,
    pub attached_devices: RwLock<Vec<Arc<AHCIAttachedDevice>>>,
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
                for device in devices.into_iter() {
                    self.attach_ahci_device(ctlr_id, device);
                }
            },
            None => {
                error!("[AHCI] Failed to initialize controller at {}", loc_str);
            }
        }
    }

    fn attach_ahci_device(&self, ctlr_id: usize, device: Box<dyn AHCIDevice + Send + Sync>) {
        // Also registers with BlockDevice
        let dev = Arc::new(AHCIAttachedDevice::create(ctlr_id, device.port(), device));
        self.attached_devices.write().push(dev.clone());
        G_BLOCK_DEV_MGR.write().register_root_device(Arc::new(AHCIBlockDevice::from(dev)));
    }
}
