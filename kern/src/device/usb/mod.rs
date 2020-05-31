pub mod interrupt;
pub mod xhci;

use spin::Mutex;
use crate::device::usb::xhci::XHCI;
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::class::PCISerialBusUSB;
use crate::device::pci::device::PCIDevice;

pub static G_USB: USBSystem = USBSystem {
    xhci: Mutex::new(None),
};


pub struct USBSystem {
    // TODO: Support Multiple Controller
    pub xhci: Mutex<Option<XHCI>>
}

impl USBSystem {
    pub fn setup_controller(&self, ctlr_type: PCISerialBusUSB, dev: PCIDevice) {
        match ctlr_type {
            PCISerialBusUSB::XHCI => {
                let dev = XHCI::create_from_device(dev);
                if dev.is_some() {
                    self.xhci.lock().replace(dev.expect(""));
                }
            },
            _ => {
                debug!("[USB] Unknown USB Host Type at {}: {:?}",
                       dev.bus_location_str(), dev.info.class);
            }
        }
    }
}