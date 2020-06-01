use crate::device::usb::G_USB;

pub fn usb_interrupt_handler() {
    let xhci = G_USB.xhci.read();
    match xhci.as_ref() {
        Some(r) => {
            r.handle_interrupt();
        },
        _ => {
            warn!("[XHCI] Interrupt while XHCI is None");
        }
    }
}