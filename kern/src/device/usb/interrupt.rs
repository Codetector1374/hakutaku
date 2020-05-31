use crate::device::usb::G_USB;

pub fn usb_interrupt_handler() {
    let xhci = G_USB.xhci.read();
    xhci.as_ref().unwrap().poll_interrupts();

}