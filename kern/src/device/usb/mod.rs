pub mod interrupt;
pub mod xhci;

use spin::{Mutex, RwLock};
use crate::device::usb::xhci::XHCI;
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::class::PCISerialBusUSB;
use crate::device::pci::device::PCIDevice;
use alloc::vec::Vec;
use alloc::sync::Arc;
use crate::device::usb::xhci::port::XHCIPort;
use x86_64::instructions::interrupts::without_interrupts;

pub static G_USB: USBSystem = USBSystem {
    xhci: Mutex::new(None),
    ports: RwLock::new(Vec::new()),
};


pub struct USBSystem {
    // TODO: Support Multiple Controller
    pub xhci: Mutex<Option<XHCI>>,
    pub ports: RwLock<Vec<Arc<Mutex<XHCIPort>>>>
}

impl USBSystem {
    pub fn setup_controller(&self, ctlr_type: PCISerialBusUSB, dev: PCIDevice) {
        match ctlr_type {
            PCISerialBusUSB::XHCI => {
                let dev = XHCI::create_from_device(dev);
                if dev.is_some() {
                    self.xhci.lock().replace(dev.expect(""));
                }
                without_interrupts(|| {
                    self.xhci.lock().as_mut().expect("thing").send_slot_enable();
                });
            },
            _ => {
                debug!("[USB] Unknown USB Host Type at {}: {:?}",
                       dev.bus_location_str(), dev.info.class);
            }
        }
    }
}