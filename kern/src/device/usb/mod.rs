pub mod interrupt;

use spin::Mutex;
use crate::pci::device::xhci::XHCI;
use crate::pci::GLOBAL_PCI;

pub static G_USB: USBSystem = USBSystem {
    xhci: Mutex::new(None),
};


pub struct USBSystem {
    // TODO: Support Multiple Controller
    pub xhci: Mutex<Option<XHCI>>
}

impl USBSystem {
    pub fn initialize(&self) {
        // Populate XHCI Controller
        let opt_xhci = XHCI::create_from_bus(GLOBAL_PCI.lock());
        if let Some(xhci) = opt_xhci {
            self.xhci.lock().replace(xhci);
            debug!("[USB] XHCI Controller initialized");
        } else {
            warn!("[USB] No XHCI Controller Found");
        };
    }
}