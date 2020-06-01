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
    xhci: RwLock::new(None),
    ports: RwLock::new(Vec::new()),
};


pub struct USBSystem {
    // TODO: Support Multiple Controller
    pub xhci: RwLock<Option<XHCI>>,
    pub ports: RwLock<Vec<Arc<Mutex<XHCIPort>>>>
}

impl USBSystem {
    pub fn setup_controller(&self, ctlr_type: PCISerialBusUSB, dev: PCIDevice) {
        match ctlr_type {
            PCISerialBusUSB::XHCI => {
                without_interrupts(|| {
                    let dev = XHCI::create_from_device(dev);
                    if dev.is_some() {
                        match self.xhci.try_write() {
                            Some(mut guard) => {
                                guard.replace(dev.expect(""));
                            },
                            None => {
                                error!("[USB] Failed to obtain mutex");
                            }
                        }
                    }
                });
                // self.xhci.read().as_ref().expect("thing").send_slot_enable();
            },
            _ => {
                debug!("[USB] Unknown USB Host Type at {}: {:?}",
                       dev.bus_location_str(), dev.info.class);
            }
        }
    }
}