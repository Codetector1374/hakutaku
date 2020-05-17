use crate::device::usb::G_USB;

pub fn usb_interrupt_handler() {
    debug!("XHCI Interrupt");
    let mut xhci = G_USB.xhci.lock();

    xhci.as_mut().unwrap().poll_interrupts();

}