pub mod consts;
pub mod interrupt;
pub mod xhci;
pub mod descriptor;
pub mod device;
pub mod error;

use spin::{Mutex, RwLock};
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::class::PCISerialBusUSB;
use crate::device::pci::device::PCIDevice;
use alloc::vec::Vec;
use alloc::sync::Arc;
// use crate::device::usb::xhci::port::XHCIPort;
use x86_64::instructions::interrupts::without_interrupts;
use crate::device::usb::device::USBDevice;
use core::sync::atomic::{AtomicU64, Ordering};
use crate::memory::mmio_bump_allocator::VMALLOC;

pub static G_USB: USBSystem = USBSystem {
    // xhci: RwLock::new(Vec::new()),
    devices: RwLock::new(Vec::new()),
    next_controller_id: AtomicU64::new(1),
    next_device_id: AtomicU64::new(1),
};


pub struct USBSystem {
    // TODO: Support Multiple Controller
    // pub xhci: RwLock<Vec<Arc<XHCI>>>,
    pub devices: RwLock<Vec<Arc<dyn USBDevice + Sync + Send>>>,
    pub next_controller_id: AtomicU64,
    pub next_device_id: AtomicU64,
}

impl USBSystem {
    pub fn setup_controller(&self, ctlr_type: PCISerialBusUSB, dev: PCIDevice) {
        match ctlr_type {
            PCISerialBusUSB::XHCI => {
                self::xhci::create_from_device(self.next_controller_id.fetch_add(1, Ordering::Acquire), dev);
            },
            _ => {
                debug!("[USB] Unknown USB Host Type at {}: {:?}",
                       dev.bus_location_str(), dev.info.class);
            }
        }
    }

    pub fn issue_device_id(&self) -> u64 {
        self.next_device_id.fetch_add(1, Ordering::Acquire)
    }

    pub fn register_device(&self, device: Arc<dyn USBDevice + Send + Sync>) {
        self.devices.write().push(device);
    }

    pub fn remove_device(&self, device_id: u64) {
        self.devices.write().retain(|dev| {
            dev.system_device_id() != device_id
        });
    }
}
