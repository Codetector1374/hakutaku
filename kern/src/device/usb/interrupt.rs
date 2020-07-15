use crate::device::usb::G_USB;

pub fn usb_interrupt_handler() {
    let xhci = G_USB.xhci.read();
    for xhci_dev in xhci.iter() {
        xhci_dev.handle_interrupt();
    }
}